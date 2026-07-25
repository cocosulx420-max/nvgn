--!strict
-- NVGN.Clean — boundaries v2, stage A: raycast-derived wall edges
--
-- The division of labour that v1 kept violating:
--
--   Nodes are a selection mask and a connectivity graph. Never geometry.
--   Part geometry is the only source of edge lines.
--
-- v2's original plan stole the line from `{killer's blocking face} ∩ {floor
-- top}`, reconstructing the killer's face plane ourselves from its OBB. This
-- module instead asks the ENGINE for that plane: one horizontal ray from the
-- standable node into the blocked direction returns `Position` (a point
-- exactly on the blocking surface), `Normal` (that surface's plane) and
-- `Instance` (the blocker, for grouping) in a single call.
--
-- Why the ray wins over reconstructing the face:
--   * It works on non-block blockers. Unions, MeshParts, wedges and Terrain
--     return a hit normal like anything else, so the v2 "known gap" — degrade
--     to a jagged node polyline when attribution names a Union or Terrain —
--     simply does not arise. No isBlock gate anywhere in this file.
--   * The origin is a live node's surface, which is outside every collider by
--     construction (the node survived LocalGrid's clearance probe), so the
--     engine's inside-origin trap cannot fire. That trap has bitten four
--     separate probes on this project; here it is excluded structurally
--     rather than guarded against.
--   * The ray both CLASSIFIES and MEASURES. A hit is a wall and carries its
--     own geometry; a miss is an open edge. v1 needed a chain of bespoke
--     probes to decide the class and then a separate construction to place
--     the line, and every new authoring pattern broke one or the other.
--
-- Stage A is DERIVATION ONLY. Closure (corner vertices from intersecting
-- adjacent runs' lines) is deliberately a separate pass: missing corners were
-- never caused by how a line was derived, but by exposure trimming removing
-- the samples near a corner, so it needs its own fix and its own review.

local LocalGrid = require(script.Parent:WaitForChild("LocalGrid"))

local Clean = {}

export type Sample = {
	grid: any,            -- host Grid
	cell: any,            -- live Cell the edge belongs to
	dir: Vector3,         -- outward horizontal direction (grid axis)
	mid: Vector3,         -- cell-edge midpoint, mask-space (never emitted)
	class: string,        -- "wall" | "seam" | "tier" | "dropoff"
	hit: RaycastResult?,  -- wall + seam (both steal their line from its plane)
}
export type Edge = {
	a: Vector3, b: Vector3,
	class: string,
	floor: BasePart,
	blocker: Instance?,   -- wall: the blocker. seam: the surface continued onto.
	exact: boolean,       -- true = plane-derived line, false = lattice polyline
	samples: number,
}
export type Config = {
	step: number?,
	rayHeights: {number}?,
	rayLen: number?,
	coverTol: number?,
	planeEps: number?,
	normalEps: number?,
	minRun: number?,
	maxSlope: number?,
	seamEps: number?,
	seamDrop: number?,
}

local DEFAULT = {
	step = 1,
	-- All below LocalGrid's minClearance (1.5), so every origin is provably in
	-- open air. Nearest hit wins, so a low kerb never masks the wall behind it.
	rayHeights = { 0.25, 0.75, 1.2 },
	rayLen = 1.6,     -- the blocked neighbour is one step out; its face a little beyond
	coverTol = 0.35,  -- flush-height tolerance for cross-grid coverage (round-7 seamEps)
	planeEps = 0.06,  -- run grouping: max spread in plane offset
	normalEps = 0.02, -- run grouping: max spread in plane normal
	minRun = 0.5,
	maxSlope = 65,    -- LocalGrid's walkability limit: a hit at or under it is a FLOOR
	seamEps = 0.3,    -- round-7 continuity tolerance ("small authored discrepancy")
	seamDrop = 2.0,   -- how far below the rim to look for a descending surface
}

local UP = Vector3.new(0, 1, 0)

-- Grouping keys must carry instance IDENTITY. `tostring(part)` returns the
-- NAME, so every part called "Pink" (or "ClipRamp") collapsed into one group
-- and runs merged across unrelated parts — a 56-stud "dropoff" spanning
-- several floors. Names are not unique in authored scenes; never key on them.
local nextId = 0
local idOf: { [Instance]: number } = setmetatable({}, { __mode = "k" }) :: any
local function iid(inst: Instance): number
	local id = idOf[inst]
	if not id then nextId += 1; id = nextId; idOf[inst] = id end
	return id
end

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

--------------------------------------------------------------------------
-- Cross-grid coverage (v2 stage 1)
--
-- Grids are per-part, so a node at part B's rim sees an "absent" neighbour
-- even where part A covers that spot at the same height. Those boundaries are
-- FICTIONAL, and they are line-breaking: a false dropoff one row behind a real
-- one puts a branch or a jog into any polyline fit. So before calling a
-- direction absent, ask whether ANY grid has a live node there.
--------------------------------------------------------------------------

local function coverKey(p: Vector3): string
	return string.format("%d:%d", math.floor(p.X), math.floor(p.Z))
end

local function buildCover(data: any): { [string]: {Vector3} }
	local cover: { [string]: {Vector3} } = {}
	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local k = coverKey(cell.pos)
			local b = cover[k]
			if not b then b = {}; cover[k] = b end
			b[#b + 1] = cell.pos
		end
	end
	return cover
end

-- Is some grid's live node standing at p (any part, including the host)?
local function covered(cover: any, p: Vector3, c: any): boolean
	local bx, bz = math.floor(p.X), math.floor(p.Z)
	local hTol = c.step * 0.5
	for dx = -1, 1 do
		for dz = -1, 1 do
			local b = cover[string.format("%d:%d", bx + dx, bz + dz)]
			if b then
				for _, q in ipairs(b) do
					if math.abs(q.Y - p.Y) <= c.coverTol
						and math.abs(q.X - p.X) <= hTol and math.abs(q.Z - p.Z) <= hTol then
						return true
					end
				end
			end
		end
	end
	return false
end

--------------------------------------------------------------------------
-- Stage A.1 — frontier samples, one per (node, direction) pair
--
-- The class belongs to the PAIR, not to the node: a stair tread is a wall
-- uphill and a dropoff downhill at the same time, and collapsing that to one
-- label per node with wall-beats-dropoff precedence destroys the dropoff on
-- every tread narrow enough for a single node to touch both — i.e. most.
--------------------------------------------------------------------------

local function sampleFrontier(data: any, c: any): ({Sample}, any)
	local cover = buildCover(data)
	local samples: {Sample} = {}
	local stats = { fictional = 0, wall = 0, seam = 0, tier = 0, dropoff = 0, fallbackSkipped = 0 }

	for part, g in pairs(data.grids) do
		if g.fallback then
			stats.fallbackSkipped += 1
			continue
		end
		local dirs = { g.u, -g.u, g.v, -g.v }
		local rp = RaycastParams.new()
		rp.FilterType = Enum.RaycastFilterType.Exclude
		rp.FilterDescendantsInstances = { part } -- Terrain deliberately left in
		rp.IgnoreWater = true

		for _, cell in ipairs(g.cells) do
			for di, dir in ipairs(dirs) do
				local du = (di <= 2) and ((di == 1) and 1 or -1) or 0
				local dv = (di <= 2) and 0 or ((di == 3) and 1 or -1)
				local nk = string.format("%d:%d", cell.ui + du, cell.vi + dv)
				if g.index[nk] then continue end -- interior

				local mid = cell.pos + dir * (c.step * 0.5)
				local dead = g.deadIndex[nk]
				local nbrPos = cell.pos + dir * c.step

				-- Another part's surface stands here: not a boundary at all.
				if not dead and covered(cover, nbrPos, c) then
					stats.fictional += 1
					continue
				end

				-- The ray decides the rest. Nearest hit across the height fan.
				local best: RaycastResult? = nil
				for _, h in ipairs(c.rayHeights) do
					local res = workspace:Raycast(cell.pos + UP * h, dir * c.rayLen, rp)
					if res and (not best or res.Distance < best.Distance) then best = res end
				end

				-- A hit whose normal is within the walkability limit is not a
				-- wall face at all — the ray grazed a FLOOR. That is what a
				-- clipramp's low end looks like from the ramp: its bottom sits
				-- a hair under the ground, so the ray clips the ground's top
				-- and v1 called the ramp entry a wall. Flush contact with a
				-- traversable surface is CONTINUITY, so it seams. Above
				-- seamEps it is a genuine lip: round 7 keeps that as wall from
				-- below / dropoff from above, paired by the pathfinder.
				local class
				if best then
					local slope = math.deg(math.acos(math.clamp((best :: RaycastResult).Normal:Dot(UP), -1, 1)))
					local dy = (best :: RaycastResult).Position.Y - cell.pos.Y
					class = (slope <= c.maxSlope and math.abs(dy) <= c.seamEps) and "seam" or "wall"
				elseif not dead then
					-- Nothing blocks at body height. Before calling it a drop,
					-- look just below the rim: a ramp descending from a flush
					-- top edge is an ENTRY, not a ledge. The flush gate is what
					-- keeps this scoped (round 6: a clipramp seams only where
					-- flush — you enter at its ends, never through its side
					-- mid-slope), so ordinary ledges stay dropoffs.
					local dres = workspace:Raycast(nbrPos + UP * c.seamEps, -UP * (c.seamEps + c.seamDrop), rp)
					if dres then
						local slope = math.deg(math.acos(math.clamp(dres.Normal:Dot(UP), -1, 1)))
						if slope <= c.maxSlope and math.abs(dres.Position.Y - cell.pos.Y) <= c.seamEps then
							best = dres
							class = "seam"
						end
					end
					class = class or "dropoff"
				end
				if not class then
					-- Killed, but nothing blocks at body height: the killer is
					-- overhead cover, so this is a headroom-tier frontier. It
					-- has NO blocking face to steal — the open case.
					class = "tier"
				end
				stats[class] += 1
				samples[#samples + 1] = {
					grid = g, cell = cell, dir = dir, mid = mid, class = class, hit = best,
				}
			end
		end
	end
	return samples, stats
end

--------------------------------------------------------------------------
-- Stage A.2 — runs
--
-- Wall samples group by (floor, blocker instance, plane). The plane comes
-- from the hit normals, so a rotated wall groups exactly rather than by
-- collinearity tolerance. The group supplies only the EXTENT; the line is
-- {hit plane} ∩ {floor top plane}, which contains no lattice term at all —
-- the staircase is not smoothed here, it is never generated.
--------------------------------------------------------------------------

local function q(x: number, eps: number): number
	return math.floor(x / eps + 0.5)
end

local function planeKey(n: Vector3, d: number, c: any): string
	return string.format("%d,%d,%d|%d",
		q(n.X, c.normalEps), q(n.Y, c.normalEps), q(n.Z, c.normalEps), q(d, c.planeEps))
end

-- The exact line: a point on both planes, plus their common direction.
local function intersectPlanes(N: Vector3, P: Vector3, Fn: Vector3, F0: Vector3, ref: Vector3)
	local D = N:Cross(Fn)
	if D.Magnitude < 1e-4 then return nil end -- blocking face parallel to the floor (a fold)
	D = D.Unit
	-- Drop the reference onto the floor plane, then slide it along the floor
	-- until it satisfies the wall plane. Two projections, no solve.
	local R = ref - Fn * ((ref - F0):Dot(Fn))
	local w = N - Fn * N:Dot(Fn)
	if w.Magnitude < 1e-4 then return nil end
	w = w.Unit
	local denom = N:Dot(w)
	if math.abs(denom) < 1e-4 then return nil end
	local X0 = R + w * (N:Dot(P - R) / denom)
	return X0, D
end

local function planeRuns(samples: {Sample}, class: string, c: any): {Edge}
	local groups: { [string]: {Sample} } = {}
	for _, s in ipairs(samples) do
		if s.class ~= class or not s.hit then continue end
		local hit = s.hit :: RaycastResult
		local n = hit.Normal
		local key = string.format("%d|%d|%s",
			iid(s.grid.part), iid(hit.Instance), planeKey(n, n:Dot(hit.Position), c))
		local b = groups[key]
		if not b then b = {}; groups[key] = b end
		b[#b + 1] = s
	end

	local edges: {Edge} = {}
	for _, grp in pairs(groups) do
		local g = grp[1].grid
		local Fn = g.n or UP
		-- averaged plane + floor reference
		local N, P, F0 = Vector3.zero, Vector3.zero, Vector3.zero
		for _, s in ipairs(grp) do
			local hit = s.hit :: RaycastResult
			N += hit.Normal; P += hit.Position; F0 += s.cell.pos
		end
		local inv = 1 / #grp
		N = N.Unit; P *= inv; F0 *= inv

		local X0, D = intersectPlanes(N, P, Fn, F0, F0)
		if not X0 then continue end
		X0 = X0 :: Vector3; D = D :: Vector3

		-- Extent from the mask: project each edge midpoint onto the line and
		-- split wherever the run is interrupted by more than one node.
		local ts = {}
		for _, s in ipairs(grp) do ts[#ts + 1] = (s.mid - X0):Dot(D) end
		table.sort(ts)
		local half = c.step * 0.5
		local i = 1
		while i <= #ts do
			local j = i
			while j < #ts and (ts[j + 1] - ts[j]) <= c.step * 1.5 do j += 1 end
			local a, b = X0 + D * (ts[i] - half), X0 + D * (ts[j] + half)
			if (b - a).Magnitude >= c.minRun then
				edges[#edges + 1] = {
					a = a, b = b, class = class, floor = g.part,
					blocker = (grp[1].hit :: RaycastResult).Instance,
					exact = true, samples = j - i + 1,
				}
			end
			i = j + 1
		end
	end
	return edges
end

-- Open edges (dropoff / tier) have no blocking face to steal, so they stay in
-- the floor's own lattice: merged along a row, axis-aligned in the FLOOR's
-- frame. Flagged exact=false — every one of these is an edge whose final
-- geometry is still owed.
local function latticeRuns(samples: {Sample}, class: string, c: any): {Edge}
	local groups: { [string]: {Sample} } = {}
	for _, s in ipairs(samples) do
		if s.class ~= class then continue end
		local d = s.dir
		local along = (math.abs(d:Dot(s.grid.u)) > 0.5) and "u" or "v"
		local row = (along == "u") and s.cell.ui or s.cell.vi
		local key = string.format("%d|%s|%d|%d,%d,%d", iid(s.grid.part), along, row,
			q(d.X, 0.02), q(d.Y, 0.02), q(d.Z, 0.02))
		local b = groups[key]
		if not b then b = {}; groups[key] = b end
		b[#b + 1] = s
	end

	local edges: {Edge} = {}
	for _, grp in pairs(groups) do
		local g = grp[1].grid
		local d = grp[1].dir
		local along = (math.abs(d:Dot(g.u)) > 0.5) and g.v or g.u
		table.sort(grp, function(x, y) return x.mid:Dot(along) < y.mid:Dot(along) end)
		local half = c.step * 0.5
		local i = 1
		while i <= #grp do
			local j = i
			while j < #grp do
				local gap = (grp[j + 1].mid - grp[j].mid).Magnitude
				if gap > c.step * 1.5 then break end
				j += 1
			end
			local a = grp[i].mid - along * half
			local b = grp[j].mid + along * half
			if (b - a).Magnitude >= c.minRun then
				edges[#edges + 1] = {
					a = a, b = b, class = class, floor = g.part,
					blocker = nil, exact = false, samples = j - i + 1,
				}
			end
			i = j + 1
		end
	end
	return edges
end

--------------------------------------------------------------------------

function Clean.fromLocal(data: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local samples, sstats = sampleFrontier(data, c)
	local edges: {Edge} = {}
	for _, e in ipairs(planeRuns(samples, "wall", c)) do edges[#edges + 1] = e end
	for _, e in ipairs(planeRuns(samples, "seam", c)) do edges[#edges + 1] = e end
	for _, e in ipairs(latticeRuns(samples, "dropoff", c)) do edges[#edges + 1] = e end
	for _, e in ipairs(latticeRuns(samples, "tier", c)) do edges[#edges + 1] = e end

	local byClass, exact, blockers = { wall = 0, seam = 0, dropoff = 0, tier = 0 }, 0, {}
	local nb = 0
	local studs = { wall = 0, seam = 0, dropoff = 0, tier = 0 }
	for _, e in ipairs(edges) do
		byClass[e.class] += 1
		studs[e.class] += (e.b - e.a).Magnitude
		if e.exact then exact += 1 end
		if e.blocker and not blockers[e.blocker] then blockers[e.blocker] = true; nb += 1 end
	end
	return {
		edges = edges, samples = samples, config = c,
		stats = {
			samples = sstats, edges = #edges, byClass = byClass, exact = exact,
			blockers = nb, studs = studs, seconds = os.clock() - t0,
		},
	}
end

function Clean.build(cfg: Config?)
	local data, floorData, tree, parts = LocalGrid.build(cfg)
	return Clean.fromLocal(data, cfg), data, floorData, tree, parts
end

--------------------------------------------------------------------------

local function bar(parent: Instance, a: Vector3, b: Vector3, colour: Color3, thick: number, name: string)
	local d = b - a
	local len = d.Magnitude
	if len < 1e-3 then return end
	local p = Instance.new("Part")
	p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
	p.Size = Vector3.new(len, thick, thick)
	p.Color = colour
	p.Material = Enum.Material.Neon
	p.CFrame = CFrame.fromMatrix((a + b) * 0.5, d.Unit, UP)
	p.Name = name
	p.Parent = parent
end

-- red = wall (exact, plane-derived) / cyan = dropoff / yellow = tier.
-- Yellow is the honest signal: every yellow stud is an edge whose geometry is
-- still owed, so its share of the map is the size of the remaining problem.
function Clean.visualize(result: any, parent: Instance?, showRaw: boolean?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Clean")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Clean"; folder.Parent = dbg

	local colours = {
		wall = Color3.new(1, 0.15, 0.15),
		seam = Color3.new(0.2, 1, 0.35),
		dropoff = Color3.new(0.15, 0.9, 1),
		tier = Color3.new(0.95, 0.85, 0.1),
	}
	for _, e in ipairs(result.edges) do
		-- nudge toward the walkable side so a line that drifts fractionally
		-- into geometry still reads (v1 round 12: an edge existing in data but
		-- drawn 0.2 inside a wall face looks exactly like a missing edge)
		local off = UP * 0.06
		bar(folder, e.a + off, e.b + off, colours[e.class], 0.3,
			e.class .. (e.exact and "" or "_raw"))
	end

	if showRaw then
		local rf = Instance.new("Folder"); rf.Name = "Samples"; rf.Parent = folder
		for _, s in ipairs(result.samples) do
			local half = result.config.step * 0.5
			local side = Vector3.new(-s.dir.Z, 0, s.dir.X)
			bar(rf, s.mid - side * half, s.mid + side * half,
				Color3.new(0.55, 0.55, 0.15), 0.12, s.class .. "_sample")
		end
	end
	return folder
end

return Clean
