--!strict
-- NVGN.Volumes — clearance volumes: headroom as an overlay, not as geometry
--
-- Cocosulx's call. The earlier plan projected the low-headroom frontier down
-- as an edge and split polygons on it; that fails because most crawl spaces
-- are TUNNELS, so the restricted region is a volume, not a line you can
-- project. Instead the region is marked with a box carrying its measured
-- clearance, and the pathfinder consults it.
--
-- Three properties make this the principled choice rather than a shortcut:
--
--  1. It matches the decisions already made for width and step tolerance.
--     Both were pulled out of the bake because they are AGENT-DEPENDENT.
--     Clearance is the same: splitting geometry at 1.5 / 3 / 4 bakes three
--     arbitrary thresholds into the mesh, and a 3.2-stud agent then needs a
--     re-bake. A volume stores its measured minimum (2.31), not a tier label,
--     and therefore serves agent sizes nobody has invented yet.
--
--  2. Error here is ASYMMETRIC, and that is what makes it cheap. A walkable
--     boundary edge must be exact — wrong either walks an agent off a ledge
--     or seals a real doorway. A clearance volume that is too BIG only makes
--     an agent crouch slightly sooner than necessary; one that is too SMALL
--     is a headbutt. So volumes are rounded OUTWARD (`expand`), and the
--     jagged 1-stud lattice frontier — the defect that drove this project's
--     entire boundary design — is simply acceptable here. Headroom never
--     needs geometry stealing.
--
--  3. It survives destruction. Each volume records the `cover` instances that
--     caused it (free: LocalGrid already found them while measuring
--     clearance). Destroy the cover, drop the volume — no re-polygonization.
--
-- Volumes are DATA. `visualize` renders debug parts with CanQuery and
-- CanCollide off, under the debug folder only. A real part in the workspace
-- is picked up by the next bake — this project has already been bitten twice
-- by debug markers becoming walkable floor.

local LocalGrid = require(script.Parent:WaitForChild("LocalGrid"))

local Volumes = {}

export type Volume = {
	floor: BasePart,
	cframe: CFrame,        -- oriented in the floor's own frame
	size: Vector3,         -- (along u, height = minClearance, along v)
	minClearance: number,  -- the number the pathfinder tests against
	maxClearance: number,  -- spread within the band, for diagnostics
	covers: {Instance},    -- what causes it; destroy these and the volume dies
	cells: number,
	-- Connected-component id: boxes of one continuous restricted space share
	-- it. This is a SEMANTIC grouping, not a geometric merge — no box is
	-- combined and no clearance precision is lost. It gives "this crawl
	-- tunnel" a single handle for an NPC to reason about ("take the tunnel"),
	-- and one id to invalidate when the part causing it is destroyed.
	component: number,
}
export type Config = {
	cap: number?,
	band: number?,
	expand: number?,
	minCells: number?,
	linkDy: number?,
}

local DEFAULT = {
	-- Restricted for SOMEBODY. Normal NPCs stand 4-7 studs, so anything under
	-- 7 constrains at least the tallest of them. Open air reads clearCap (20)
	-- and never qualifies. Configurable: raise it toward the giant cutoff (8)
	-- to describe headroom for giants too, at the cost of many more volumes.
	cap = 7,
	-- Merge only cells whose clearance falls in the same 0.5-stud band. The
	-- merged volume bakes the band's MINIMUM, which is conservative (too
	-- restrictive = safe), but unbanded merging would drag a whole floor down
	-- to its lowest pocket and describe nothing useful.
	band = 0.5,
	-- Outward rounding, in the safe direction (see note 2 above). Half a cell
	-- covers the lattice quantization exactly.
	expand = 0.5,
	-- MUST stay 1. Raising it drops small rectangles, and dropping coverage is
	-- the UNSAFE direction — a hole means an agent walks upright into a low
	-- pocket. To get fewer volumes, widen `band` instead (that stays
	-- conservative). Enforced below rather than left as a footgun.
	minCells = 1,
	-- Two boxes join a component when their footprints touch and their bases
	-- sit within this height of each other. Stops a tunnel from linking to an
	-- unrelated restricted space on the floor above.
	linkDy = 1.5,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	if c.minCells > 1 then
		warn("[NVGN.Volumes] minCells > 1 would delete coverage (holes = an agent " ..
			"walks upright into a low pocket). Clamped to 1; widen `band` instead.")
		c.minCells = 1
	end
	return c
end

--------------------------------------------------------------------------
-- Geometry helpers. Everything is a box standing on a floor, so overlap is
-- tested as {rotated rectangle in XZ} x {height band}, not full 3D SAT: the
-- vertical axis is always world-up here, which makes the separating-axis set
-- just the four footprint edge normals.
--------------------------------------------------------------------------

local function footprintAxes(cf: CFrame)
	local r, f = cf.RightVector, cf.LookVector
	local a1 = Vector2.new(r.X, r.Z)
	local a2 = Vector2.new(f.X, f.Z)
	if a1.Magnitude < 1e-4 then a1 = Vector2.new(1, 0) else a1 = a1.Unit end
	if a2.Magnitude < 1e-4 then a2 = Vector2.new(0, 1) else a2 = a2.Unit end
	return a1, a2
end

local function corners2D(v: Volume): {Vector2}
	local a1, a2 = footprintAxes(v.cframe)
	local p = Vector2.new(v.cframe.Position.X, v.cframe.Position.Z)
	local hx, hz = v.size.X * 0.5, v.size.Z * 0.5
	return {
		p + a1 * hx + a2 * hz, p + a1 * hx - a2 * hz,
		p - a1 * hx - a2 * hz, p - a1 * hx + a2 * hz,
	}
end

local function project(pts: {Vector2}, axis: Vector2): (number, number)
	local mn, mx = math.huge, -math.huge
	for _, p in ipairs(pts) do
		local d = p:Dot(axis)
		mn = math.min(mn, d); mx = math.max(mx, d)
	end
	return mn, mx
end

-- Convex overlap in the horizontal plane (separating-axis, both shapes' edge
-- normals). `pad` lets a caller treat touching-but-not-overlapping as overlap.
local function overlaps2D(a: {Vector2}, b: {Vector2}, axes: {Vector2}, pad: number): boolean
	for _, ax in ipairs(axes) do
		local amn, amx = project(a, ax)
		local bmn, bmx = project(b, ax)
		if amx + pad < bmn or bmx + pad < amn then return false end
	end
	return true
end

-- Height of the walkable surface a volume rests on.
local function baseY(v: Volume): number
	return v.cframe.Position.Y - v.size.Y * 0.5
end

--------------------------------------------------------------------------
-- Greedy rectangle extraction over one band's cell mask.
--
-- Runs along v, then grows along u while the whole span is present. Rectangles
-- in the FLOOR's lattice, which is exactly the frame a volume wants: a rotated
-- floor gets rotated boxes, and no volume is ever axis-aligned to the world by
-- accident.
--------------------------------------------------------------------------

local function extractRects(mask: { [string]: any })
	local rects = {}
	local used: { [string]: boolean } = {}
	local keys = {}
	for k in pairs(mask) do keys[#keys + 1] = k end
	-- deterministic order: by ui, then vi
	table.sort(keys, function(a, b)
		local au, av = a:match("(-?%d+):(-?%d+)")
		local bu, bv = b:match("(-?%d+):(-?%d+)")
		local aui, bui = tonumber(au) :: number, tonumber(bu) :: number
		if aui ~= bui then return aui < bui end
		return (tonumber(av) :: number) < (tonumber(bv) :: number)
	end)

	for _, k in ipairs(keys) do
		if used[k] then continue end
		local su, sv = k:match("(-?%d+):(-?%d+)")
		local u0, v0 = tonumber(su) :: number, tonumber(sv) :: number

		-- extend along v
		local v1 = v0
		while true do
			local nk = string.format("%d:%d", u0, v1 + 1)
			if mask[nk] and not used[nk] then v1 += 1 else break end
		end

		-- extend along u, whole spans only
		local u1 = u0
		while true do
			local ok = true
			for vi = v0, v1 do
				local nk = string.format("%d:%d", u1 + 1, vi)
				if not (mask[nk] and not used[nk]) then ok = false; break end
			end
			if not ok then break end
			u1 += 1
		end

		local members = {}
		for ui = u0, u1 do
			for vi = v0, v1 do
				local nk = string.format("%d:%d", ui, vi)
				used[nk] = true
				members[#members + 1] = mask[nk]
			end
		end
		rects[#rects + 1] = { u0 = u0, u1 = u1, v0 = v0, v1 = v1, members = members }
	end
	return rects
end

--------------------------------------------------------------------------

function Volumes.fromLocal(data: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local out: {Volume} = {}
	local skippedFallback = 0

	for part, g in pairs(data.grids) do
		-- Fallback (non-block) grids are deferred with unions/meshes/terrain.
		if g.fallback then
			skippedFallback += 1
			continue
		end
		-- one mask per clearance band
		local bands: { [number]: { [string]: any } } = {}
		for _, cell in ipairs(g.cells) do
			if cell.clearance >= c.cap then continue end
			local b = math.floor(cell.clearance / c.band)
			local m = bands[b]
			if not m then m = {}; bands[b] = m end
			m[string.format("%d:%d", cell.ui, cell.vi)] = cell
		end

		for _, mask in pairs(bands) do
			for _, r in ipairs(extractRects(mask)) do
				if #r.members < c.minCells then continue end
				local mn, mx = math.huge, -math.huge
				local centre = Vector3.zero
				local coverSet: { [Instance]: boolean } = {}
				for _, cell in ipairs(r.members) do
					mn = math.min(mn, cell.clearance)
					mx = math.max(mx, cell.clearance)
					centre += cell.pos
					if cell.cover then coverSet[cell.cover] = true end
				end
				centre /= #r.members
				local covers = {}
				for inst in pairs(coverSet) do covers[#covers + 1] = inst end

				local du = (r.u1 - r.u0 + 1) * g.step + 2 * c.expand
				local dv = (r.v1 - r.v0 + 1) * g.step + 2 * c.expand
				local n = g.n or Vector3.yAxis
				-- Sits ON the floor and rises to the headroom limit: the box is
				-- the space an agent must fit inside, so `size.Y` is the number
				-- the pathfinder compares against standing height directly.
				out[#out + 1] = {
					floor = part,
					cframe = CFrame.fromMatrix(centre + n * (mn * 0.5), g.u, n),
					size = Vector3.new(du, mn, dv),
					minClearance = mn,
					maxClearance = mx,
					covers = covers,
					cells = #r.members,
					component = 0, -- assigned below
				}
			end
		end
	end

	local nComponents = Volumes.assignComponents(out, c)

	local tiers = { crawl = 0, crouch = 0, walk = 0 }
	local cells = 0
	for _, v in ipairs(out) do
		cells += v.cells
		if v.minClearance < 3 then tiers.crawl += 1
		elseif v.minClearance < 4 then tiers.crouch += 1
		else tiers.walk += 1 end
	end
	return {
		volumes = out, config = c,
		stats = {
			volumes = #out, cells = cells, tiers = tiers, components = nComponents,
			skippedFallback = skippedFallback, seconds = os.clock() - t0,
		},
	}
end

--------------------------------------------------------------------------
-- Connected components: one continuous restricted space, one id.
--
-- Purely semantic — no box is merged and no clearance is lost. Candidate
-- pairs come from the bucket index, so this is near-linear in practice rather
-- than O(n^2): only volumes sharing a bucket are ever compared.
--------------------------------------------------------------------------

function Volumes.assignComponents(vols: {Volume}, cfg: Config?): number
	local c = merged(cfg)
	local parent: {number} = {}
	for i = 1, #vols do parent[i] = i end

	local function find(i: number): number
		while parent[i] ~= i do
			parent[i] = parent[parent[i]] -- path halving
			i = parent[i]
		end
		return i
	end
	local function union(a: number, b: number)
		local ra, rb = find(a), find(b)
		if ra ~= rb then parent[rb] = ra end
	end

	-- bucket volumes by footprint so only plausible pairs are compared
	local size = 8
	local buckets: { [string]: {number} } = {}
	local cache: { {Vector2} } = {}
	for i, v in ipairs(vols) do
		cache[i] = corners2D(v)
		local mnx, mxx = math.huge, -math.huge
		local mnz, mxz = math.huge, -math.huge
		for _, p in ipairs(cache[i]) do
			mnx = math.min(mnx, p.X); mxx = math.max(mxx, p.X)
			mnz = math.min(mnz, p.Y); mxz = math.max(mxz, p.Y)
		end
		for bx = math.floor(mnx / size), math.floor(mxx / size) do
			for bz = math.floor(mnz / size), math.floor(mxz / size) do
				local k = string.format("%d:%d", bx, bz)
				local b = buckets[k]
				if not b then b = {}; buckets[k] = b end
				b[#b + 1] = i
			end
		end
	end

	for _, b in pairs(buckets) do
		for x = 1, #b - 1 do
			for y = x + 1, #b do
				local i, j = b[x], b[y]
				if find(i) ~= find(j) then
					local vi, vj = vols[i], vols[j]
					if math.abs(baseY(vi) - baseY(vj)) <= c.linkDy then
						local a1, a2 = footprintAxes(vi.cframe)
						local b1, b2 = footprintAxes(vj.cframe)
						local axes = {
							Vector2.new(-a1.Y, a1.X), Vector2.new(-a2.Y, a2.X),
							Vector2.new(-b1.Y, b1.X), Vector2.new(-b2.Y, b2.X),
						}
						-- volumes already overlap by `expand` on each side where
						-- they abut; a small pad also links exactly-touching ones
						if overlaps2D(cache[i], cache[j], axes, 0.05) then
							union(i, j)
						end
					end
				end
			end
		end
	end

	local ids: { [number]: number } = {}
	local n = 0
	for i, v in ipairs(vols) do
		local r = find(i)
		local id = ids[r]
		if not id then n += 1; id = n; ids[r] = id end
		v.component = id
	end
	return n
end

function Volumes.build(cfg: Config?)
	local data = LocalGrid.build(cfg)
	return Volumes.fromLocal(data, cfg), data
end

--------------------------------------------------------------------------
-- Query side. Volumes must be searchable DURING the A* search, never
-- discovered on contact: the pathfinder has to know before it commits, or it
-- plans a route and fails partway.
--------------------------------------------------------------------------

export type Index = { cell: number, buckets: { [string]: {Volume} } }

function Volumes.index(result: any, bucketSize: number?): Index
	local size = bucketSize or 8
	local buckets: { [string]: {Volume} } = {}
	for _, v in ipairs(result.volumes) do
		-- horizontal AABB of the oriented box, padded to whole buckets
		local r = v.cframe.RightVector * (v.size.X * 0.5)
		local f = v.cframe.LookVector * (v.size.Z * 0.5)
		local up = v.cframe.UpVector * (v.size.Y * 0.5)
		local ext = Vector3.new(
			math.abs(r.X) + math.abs(f.X) + math.abs(up.X),
			0,
			math.abs(r.Z) + math.abs(f.Z) + math.abs(up.Z)
		)
		local p = v.cframe.Position
		for bx = math.floor((p.X - ext.X) / size), math.floor((p.X + ext.X) / size) do
			for bz = math.floor((p.Z - ext.Z) / size), math.floor((p.Z + ext.Z) / size) do
				local k = string.format("%d:%d", bx, bz)
				local b = buckets[k]
				if not b then b = {}; buckets[k] = b end
				b[#b + 1] = v
			end
		end
	end
	return { cell = size, buckets = buckets }
end

-- Lowest headroom any volume imposes at `pos`, or math.huge where none does.
function Volumes.clearanceAt(idx: Index, pos: Vector3): number
	local k = string.format("%d:%d", math.floor(pos.X / idx.cell), math.floor(pos.Z / idx.cell))
	local b = idx.buckets[k]
	if not b then return math.huge end
	local best = math.huge
	for _, v in ipairs(b) do
		local rel = v.cframe:PointToObjectSpace(pos)
		if math.abs(rel.X) <= v.size.X * 0.5
			and math.abs(rel.Y) <= v.size.Y * 0.5
			and math.abs(rel.Z) <= v.size.Z * 0.5 then
			best = math.min(best, v.minClearance)
		end
	end
	return best
end

-- Does an agent of this standing height fit at `pos`?
function Volumes.fits(idx: Index, pos: Vector3, standingHeight: number): boolean
	return Volumes.clearanceAt(idx, pos) >= standingHeight
end

--------------------------------------------------------------------------
-- Per-polygon annotation.
--
-- Polygons are generated in ABSTRACTION of these volumes — headroom never
-- splits the mesh. But a polygon can then be partly covered by a volume, and
-- poly-level A* cannot express "passable, but not through the middle". So
-- each poly carries ONE scalar: the lowest clearance any volume imposes
-- anywhere over its area.
--
-- It is a conservative PRE-FILTER, not a replacement for the volumes:
--
--   polyMinClearance >= agentStanding  -> the whole poly fits; skip volume
--                                         checks entirely (the common ~5-stud
--                                         human case, so the hot path is free)
--   otherwise                          -> consult the volumes along the
--                                         smoothed corridor for per-segment
--                                         crouch/crawl modes, or to reject
--                                         the poly for an agent that cannot
--                                         fit at all
--
-- Returns math.huge where nothing restricts the polygon.
--------------------------------------------------------------------------

function Volumes.minClearanceOverPoly(idx: Index, verts: {Vector3}, cfg: Config?): number
	if #verts == 0 then return math.huge end
	local c = merged(cfg)

	local poly2: {Vector2} = {}
	local mnx, mxx, mnz, mxz = math.huge, -math.huge, math.huge, -math.huge
	local sumY = 0
	for _, p in ipairs(verts) do
		poly2[#poly2 + 1] = Vector2.new(p.X, p.Z)
		mnx = math.min(mnx, p.X); mxx = math.max(mxx, p.X)
		mnz = math.min(mnz, p.Z); mxz = math.max(mxz, p.Z)
		sumY += p.Y
	end
	local polyY = sumY / #verts

	-- polygon edge normals, for the separating-axis test
	local axes: {Vector2} = {}
	for i = 1, #poly2 do
		local a = poly2[i]
		local b = poly2[(i % #poly2) + 1]
		local e = b - a
		if e.Magnitude > 1e-4 then
			e = e.Unit
			axes[#axes + 1] = Vector2.new(-e.Y, e.X)
		end
	end

	local best = math.huge
	local seen: { [any]: boolean } = {}
	for bx = math.floor(mnx / idx.cell), math.floor(mxx / idx.cell) do
		for bz = math.floor(mnz / idx.cell), math.floor(mxz / idx.cell) do
			local b = idx.buckets[string.format("%d:%d", bx, bz)]
			if b then
				for _, v in ipairs(b) do
					if not seen[v] and v.minClearance < best then
						seen[v] = true
						-- same storey only: a low ceiling one floor up is not
						-- this polygon's problem
						if math.abs(baseY(v) - polyY) <= c.linkDy then
							local a1, a2 = footprintAxes(v.cframe)
							local all = table.clone(axes)
							all[#all + 1] = Vector2.new(-a1.Y, a1.X)
							all[#all + 1] = Vector2.new(-a2.Y, a2.X)
							if overlaps2D(poly2, corners2D(v), all, 0) then
								best = v.minClearance
							end
						end
					end
				end
			end
		end
	end
	return best
end

--------------------------------------------------------------------------

-- Debug only. CanQuery/CanCollide off and parented under NVGN_Debug, so a
-- later bake cannot see these: volumes are data, not world objects.
function Volumes.visualize(result: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Volumes")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Volumes"; folder.Parent = dbg

	for _, v in ipairs(result.volumes) do
		local p = Instance.new("Part")
		p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
		p.Size = v.size
		p.CFrame = v.cframe
		p.Transparency = 0.72
		p.Material = Enum.Material.ForceField
		if v.minClearance < 3 then
			p.Color = Color3.new(1, 0.25, 0.5)      -- crawl
		elseif v.minClearance < 4 then
			p.Color = Color3.new(1, 0.6, 0.15)      -- crouch
		else
			p.Color = Color3.new(0.5, 0.75, 1)      -- restricted, still walkable
		end
		-- name carries both numbers, so a click in the explorer answers "how low"
		-- and "which tunnel" at once
		p.Name = string.format("clr%.2f#%d", v.minClearance, v.component)
		p.Parent = folder
	end
	return folder
end

return Volumes
