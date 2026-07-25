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
}
export type Config = {
	cap: number?,
	band: number?,
	expand: number?,
	minCells: number?,
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
	minCells = 1,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
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
				}
			end
		end
	end

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
			volumes = #out, cells = cells, tiers = tiers,
			skippedFallback = skippedFallback, seconds = os.clock() - t0,
		},
	}
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
		p.Name = string.format("clr%.2f", v.minClearance)
		p.Parent = folder
	end
	return folder
end

return Volumes
