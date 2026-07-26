--!strict
-- NVGN.Portals — polygon adjacency: where an agent may cross from one polygon
-- into the next.
--
-- A polygon partition is not yet a navmesh. Until every shared edge is a
-- traversal record, the polygons are 160 independent shapes and nothing can be
-- planned across them. This stage turns them into a graph.
--
-- TWO SOURCES, DELIBERATELY UNEQUAL. They are not the same problem and must not
-- share a code path:
--
--   INSIDE a region, faces share vertex IDENTITIES. Two faces are neighbours
--   exactly when they name the same edge, and `Polys` hands that over already
--   decided. No tolerance is involved, and none should be invented: this is the
--   overwhelming majority of portals and it is exact.
--
--   BETWEEN regions, the two sides are separate polygons on separate floor
--   parts, each derived in its own local frame from its own measured geometry.
--   Their edges are co-located but not identical -- a ramp meeting a floor is
--   authored flush to within a fraction of a stud, not to the bit. Matching
--   them is a geometric search, and it is the only place a tolerance enters.
--
-- WHICH EDGES CAN BE PORTALS. Class decides, and the classes already mean the
-- right things, so no new judgement is made here:
--
--   internal      a cut this pipeline chose through continuous ground. Always
--                 crossable -- there is nothing there.
--   seam          a real join between two surfaces: ramp entries, and small
--                 authored discrepancies between neighbouring floors. This is
--                 what makes a ClipRamp reachable at all.
--   continuation  the surface carries on onto an overlapping floor. No boundary
--                 was emitted because nothing blocks; the polygons on either
--                 side belong to different regions and must be linked.
--   wall / dropoff / tier   never. These are the reasons the region ended.
--
-- NO WIDTH FILTER. A portal's length is recorded and nothing is rejected for
-- being short. Width is resolved at pathfinding time against the actual agent,
-- and dropping a narrow portal here would bake in a minimum agent size through
-- the back door -- the same argument that keeps sub-agent-width ground in the
-- mesh in the first place.

local Polys = require(script.Parent:WaitForChild("Polys"))

local Portals = {}

export type Portal = {
	a: number, b: number,   -- polygon indices into the Polys result
	p1: Vector3, p2: Vector3,
	length: number,
	class: string,          -- internal | seam | continuation
	kind: string,           -- "exact" (shared identity) | "matched" (geometric)
}
export type Config = {
	matchLateral: number?, matchVertical: number?, matchParallel: number?,
	minPortalLength: number?, matchCell: number?,
}

local DEFAULT = {
	-- How far apart two edges may sit and still be the same join. Lateral is
	-- horizontal, vertical is the height step. Both are at the scale Clean
	-- already treats as "small authored discrepancy" (seamEps 0.3), not at the
	-- scale of a real step -- a genuine step is a wall from below and a dropoff
	-- from above, and pairing those is the pathfinder's job, not this stage's.
	-- LATERAL MUST EXCEED Polys.minStripWidth. Noise-width strip discard means a
	-- polygon edge can legitimately sit up to that far INSIDE the true boundary,
	-- so the two sides of a join no longer coincide even though the ground does.
	-- Measured: at 0.35 only 89% of seam length pairs up and the graph is in 8
	-- components; at 1.0 seam matching is 100% and it is 6. This is the debt the
	-- discard took on, and it is paid here rather than by loosening the height
	-- test, which degrades seam matching instead of helping it.
	matchLateral = 1.0,
	-- Height stays at Clean's "small authored discrepancy" scale. A genuine step
	-- must NOT pair here: it bakes as wall from below and dropoff from above,
	-- and pairing those per agent is the pathfinder's job.
	matchVertical = 0.35,
	matchParallel = 0.995,   -- |dot| of unit directions
	minPortalLength = 0.05,  -- degenerate only; NOT a width filter
	matchCell = 8,           -- spatial hash cell, studs
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local PORTAL_CLASS = { internal = true, seam = true, continuation = true }

--------------------------------------------------------------------------

function Portals.fromPolys(pres: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local portals: {Portal} = {}
	local stats = {
		polys = #pres.polys,
		exact = 0, matched = 0,
		seamEdges = 0, seamLen = 0, seamMatchedLen = 0,
		contEdges = 0, contLen = 0, contMatchedLen = 0,
		isolated = 0, maxDegree = 0, components = 0, largestComponent = 0,
		byClass = {},
	}

	local neighbours: {{any}} = {}
	for i = 1, #pres.polys do neighbours[i] = {} end

	local function addPortal(a: number, b: number, p1: Vector3, p2: Vector3, class: string, kind: string)
		local len = (p2 - p1).Magnitude
		if len < c.minPortalLength then return end
		portals[#portals + 1] = {
			a = a, b = b, p1 = p1, p2 = p2, length = len, class = class, kind = kind,
		}
		local pi = #portals
		table.insert(neighbours[a], { poly = b, portal = pi })
		table.insert(neighbours[b], { poly = a, portal = pi })
		stats.byClass[class] = (stats.byClass[class] or 0) + 1
		if kind == "exact" then stats.exact += 1 else stats.matched += 1 end
	end

	----------------------------------------------------------------
	-- 1. exact, from shared vertex identity inside a region
	----------------------------------------------------------------
	for _, ad in ipairs(pres.adjacency or {}) do
		if PORTAL_CLASS[ad.class] then
			addPortal(ad.a, ad.b, ad.p1, ad.p2, ad.class, "exact")
		end
	end

	----------------------------------------------------------------
	-- 2. matched, across regions
	--
	-- Only seam and continuation edges are candidates. An `internal` edge is a
	-- cut inside one region and is already handled exactly above; a wall or
	-- dropoff is the reason the region stopped.
	----------------------------------------------------------------
	type Cand = { poly: number, a: Vector3, b: Vector3, dir: Vector3, len: number, class: string }
	local cands: {Cand} = {}
	for pi, p in ipairs(pres.polys) do
		for i = 1, #p.verts do
			local cls = p.classes[i]
			if cls == "seam" or cls == "continuation" then
				local a, b = p.verts[i], p.verts[(i % #p.verts) + 1]
				local d = b - a
				local len = d.Magnitude
				if len > c.minPortalLength then
					cands[#cands + 1] = { poly = pi, a = a, b = b, dir = d / len, len = len, class = cls }
					if cls == "seam" then
						stats.seamEdges += 1; stats.seamLen += len
					else
						stats.contEdges += 1; stats.contLen += len
					end
				end
			end
		end
	end

	-- spatial hash on the horizontal midpoint
	local buckets: {[string]: {number}} = {}
	local function keyOf(x: number, z: number): string
		return math.floor(x / c.matchCell) .. ":" .. math.floor(z / c.matchCell)
	end
	for i, e in ipairs(cands) do
		local m = (e.a + e.b) * 0.5
		local k = keyOf(m.X, m.Z)
		buckets[k] = buckets[k] or {}
		table.insert(buckets[k], i)
	end

	local matchedLen: {number} = {}
	-- intervals along each candidate that already have a partner, so the
	-- containment pass below only works on what edge matching left over
	local covered: {{{number}}} = {}
	for i = 1, #cands do matchedLen[i] = 0; covered[i] = {} end
	local seenPair: {[string]: boolean} = {}

	for i, e in ipairs(cands) do
		local m = (e.a + e.b) * 0.5
		local bx, bz = math.floor(m.X / c.matchCell), math.floor(m.Z / c.matchCell)
		for gx = bx - 1, bx + 1 do
			for gz = bz - 1, bz + 1 do
				for _, j in ipairs(buckets[gx .. ":" .. gz] or {}) do
					local f = cands[j]
					if j > i and f.poly ~= e.poly then
						-- parallel?
						if math.abs(e.dir:Dot(f.dir)) >= c.matchParallel then
							-- overlap along e, measured by projecting f's ends
							local ta = (f.a - e.a):Dot(e.dir)
							local tb = (f.b - e.a):Dot(e.dir)
							local t0o, t1o = math.min(ta, tb), math.max(ta, tb)
							local lo = math.max(0, t0o)
							local hi = math.min(e.len, t1o)
							if hi - lo > c.minPortalLength then
								-- how far apart are the two lines over that span?
								local okDist = true
								for _, t in ipairs({ lo, (lo + hi) * 0.5, hi }) do
									local pOnE = e.a + e.dir * t
									local s = (pOnE - f.a):Dot(f.dir)
									s = math.clamp(s, 0, f.len)
									local pOnF = f.a + f.dir * s
									local d = pOnE - pOnF
									local horiz = Vector3.new(d.X, 0, d.Z).Magnitude
									if horiz > c.matchLateral or math.abs(d.Y) > c.matchVertical then
										okDist = false
										break
									end
								end
								if okDist then
									local pk = (e.poly < f.poly)
										and string.format("%d|%d|%.1f|%.1f", e.poly, f.poly, lo, hi)
										or string.format("%d|%d|%.1f|%.1f", f.poly, e.poly, lo, hi)
									if not seenPair[pk] then
										seenPair[pk] = true
										-- the portal sits on e's own span; both sides
										-- are within tolerance of it by construction
										local p1 = e.a + e.dir * lo
										local p2 = e.a + e.dir * hi
										local cls = (e.class == "seam" or f.class == "seam") and "seam" or "continuation"
										addPortal(e.poly, f.poly, p1, p2, cls, "matched")
										matchedLen[i] += (hi - lo)
										matchedLen[j] += (hi - lo)
										table.insert(covered[i], { lo, hi })
										local sa = math.clamp((p1 - f.a):Dot(f.dir), 0, f.len)
										local sb = math.clamp((p2 - f.a):Dot(f.dir), 0, f.len)
										table.insert(covered[j], { math.min(sa, sb), math.max(sa, sb) })
									end
								end
							end
						end
					end
				end
			end
		end
	end

	----------------------------------------------------------------
	-- 2b. containment, for the handover case edge matching cannot see.
	--
	-- Overlapping floors do NOT meet edge to edge. Where floor A ends on top of
	-- floor B, A's rim runs through the INTERIOR of B's polygon, so there is no
	-- second edge anywhere for the search above to pair it with -- and this is
	-- the common case, not a corner one: LocalGrid double-covers shared ground
	-- on purpose, and Loops closes such a ring with the rim and tags it
	-- `continuation` precisely because nothing blocks there.
	--
	-- Edge matching alone left 53 of 160 polygons isolated and the graph in 62
	-- components. So an uncovered span is also tested for lying ON another
	-- polygon's surface: inside it horizontally, and at the same height.
	----------------------------------------------------------------
	do
		-- horizontal footprint and plane of every polygon
		type Foot = { xs: {number}, zs: {number}, minX: number, maxX: number,
			minZ: number, maxZ: number, o: Vector3, n: Vector3, floor: BasePart }
		local feet: {Foot} = {}
		for pi, p in ipairs(pres.polys) do
			local xs, zs = {}, {}
			local minX, maxX = math.huge, -math.huge
			local minZ, maxZ = math.huge, -math.huge
			for _, v in ipairs(p.verts) do
				xs[#xs + 1] = v.X; zs[#zs + 1] = v.Z
				minX = math.min(minX, v.X); maxX = math.max(maxX, v.X)
				minZ = math.min(minZ, v.Z); maxZ = math.max(maxZ, v.Z)
			end
			local n = Vector3.new(0, 1, 0)
			if #p.verts >= 3 then
				local cr = (p.verts[2] - p.verts[1]):Cross(p.verts[3] - p.verts[1])
				if cr.Magnitude > 1e-6 then n = cr.Unit end
			end
			if n.Y < 0 then n = -n end
			feet[pi] = { xs = xs, zs = zs, minX = minX, maxX = maxX, minZ = minZ, maxZ = maxZ,
				o = p.verts[1], n = n, floor = p.floor }
		end

		local grid: {[string]: {number}} = {}
		for pi, ft in ipairs(feet) do
			for gx = math.floor(ft.minX / c.matchCell), math.floor(ft.maxX / c.matchCell) do
				for gz = math.floor(ft.minZ / c.matchCell), math.floor(ft.maxZ / c.matchCell) do
					local k = gx .. ":" .. gz
					grid[k] = grid[k] or {}
					table.insert(grid[k], pi)
				end
			end
		end

		local function insideFoot(ft: Foot, x: number, z: number): boolean
			if x < ft.minX or x > ft.maxX or z < ft.minZ or z > ft.maxZ then return false end
			local inside = false
			local n = #ft.xs
			local j = n
			for i = 1, n do
				if (ft.zs[i] > z) ~= (ft.zs[j] > z) then
					local xx = ft.xs[i] + (z - ft.zs[i]) / (ft.zs[j] - ft.zs[i]) * (ft.xs[j] - ft.xs[i])
					if x < xx then inside = not inside end
				end
				j = i
			end
			return inside
		end

		for i, e in ipairs(cands) do
			-- what is still unpartnered along this edge
			local iv = table.clone(covered[i])
			table.sort(iv, function(x, y) return x[1] < y[1] end)
			local gaps: {{number}} = {}
			local cur = 0
			for _, seg in ipairs(iv) do
				if seg[1] > cur + c.minPortalLength then gaps[#gaps + 1] = { cur, seg[1] } end
				cur = math.max(cur, seg[2])
			end
			if e.len > cur + c.minPortalLength then gaps[#gaps + 1] = { cur, e.len } end

			for _, gap in ipairs(gaps) do
				local span = gap[2] - gap[1]
				local steps = math.max(2, math.ceil(span / 0.5))
				-- which polygon, if any, carries each sample
				local run, runStart = nil, nil
				local function closeRun(endT: number)
					if run and runStart and endT - runStart > c.minPortalLength then
						local p1 = e.a + e.dir * runStart
						local p2 = e.a + e.dir * endT
						addPortal(e.poly, run, p1, p2, e.class, "matched")
						matchedLen[i] += (endT - runStart)
					end
					run, runStart = nil, nil
				end
				for s = 0, steps do
					local t = gap[1] + span * (s / steps)
					local pt = e.a + e.dir * t
					local found = nil
					local k = math.floor(pt.X / c.matchCell) .. ":" .. math.floor(pt.Z / c.matchCell)
					for _, qi in ipairs(grid[k] or {}) do
						local ft = feet[qi]
						if qi ~= e.poly and ft.floor ~= feet[e.poly].floor and insideFoot(ft, pt.X, pt.Z) then
							-- same height? solve the polygon's plane at (x, z)
							if math.abs(ft.n.Y) > 1e-6 then
								local y = ft.o.Y - (ft.n.X * (pt.X - ft.o.X) + ft.n.Z * (pt.Z - ft.o.Z)) / ft.n.Y
								if math.abs(y - pt.Y) <= c.matchVertical then found = qi; break end
							end
						end
					end
					if found ~= run then
						closeRun(t)
						if found then run, runStart = found, t end
					end
				end
				closeRun(gap[2])
			end
		end
	end

	for i, e in ipairs(cands) do
		local got = math.min(matchedLen[i], e.len)
		if e.class == "seam" then stats.seamMatchedLen += got else stats.contMatchedLen += got end
	end

	----------------------------------------------------------------
	-- 3. graph health. An unreachable polygon is not an error by itself -- an
	-- island genuinely is an island until jump links exist -- but the numbers
	-- are how we notice a join that silently failed to match.
	----------------------------------------------------------------
	for i = 1, #pres.polys do
		local d = #neighbours[i]
		if d == 0 then stats.isolated += 1 end
		stats.maxDegree = math.max(stats.maxDegree, d)
	end

	local seen: {[number]: boolean} = {}
	for i = 1, #pres.polys do
		if not seen[i] then
			stats.components += 1
			local size = 0
			local queue, head = { i }, 1
			seen[i] = true
			while head <= #queue do
				local cur = queue[head]; head += 1
				size += 1
				for _, nb in ipairs(neighbours[cur]) do
					if not seen[nb.poly] then
						seen[nb.poly] = true
						queue[#queue + 1] = nb.poly
					end
				end
			end
			stats.largestComponent = math.max(stats.largestComponent, size)
		end
	end

	stats.portals = #portals
	stats.seconds = os.clock() - t0
	return { portals = portals, neighbours = neighbours, stats = stats, config = c }
end

function Portals.build(cfg: any?)
	local pres, lres, cres, data = Polys.build(cfg)
	return Portals.fromPolys(pres, cfg), pres, lres, cres, data
end

--------------------------------------------------------------------------

function Portals.visualize(res: any, pres: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Portals")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Portals"; folder.Parent = dbg

	local UP = Vector3.new(0, 1, 0)
	local colours = {
		internal = Color3.new(0.45, 0.35, 0.9),
		seam = Color3.new(0.2, 1, 0.35),
		continuation = Color3.new(1, 0.65, 0.1),
	}
	for pi, p in ipairs(res.portals) do
		local d = p.p2 - p.p1
		if d.Magnitude > 1e-3 then
			local bar = Instance.new("Part")
			bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
			-- matched portals drawn fatter and higher: they are the ones carrying
			-- a tolerance, so they are the ones worth eyeballing
			local thick = (p.kind == "matched") and 0.45 or 0.22
			bar.Size = Vector3.new(d.Magnitude, thick, thick)
			bar.Color = colours[p.class] or Color3.new(1, 1, 1)
			bar.Material = Enum.Material.Neon
			bar.Transparency = (p.kind == "matched") and 0 or 0.35
			bar.CFrame = CFrame.fromMatrix(
				(p.p1 + p.p2) * 0.5 + UP * ((p.kind == "matched") and 0.6 or 0.4), d.Unit, UP)
			bar.Name = string.format("%s_%s_%d_%d_l%.1f", p.class, p.kind, p.a, p.b, p.length)
			bar.Parent = folder
		end
	end
	return folder
end

return Portals
