--!strict
-- NVGN.Floor — walkable surface extraction
--
-- Produces one surfel per 1-stud walkable cell. The SVO finds candidates
-- (solid voxels with empty space above); each candidate's top face is walked at
-- 1-stud resolution and a raycast onto the REAL part gives exact height + normal
-- (so ramps are smooth, not stair-stepped, and a collapsed node covering several
-- parts is sampled per-cell). Clearance is an upward raycast to the real ceiling.
-- Only walkable surfels are kept (steep faces feed the later boundary stage).

local SVO = require(script.Parent:WaitForChild("SVO"))

local Floor = {}

export type Surfel = {
	pos: Vector3,       -- exact surface position
	normal: Vector3,    -- surface normal
	slope: number,      -- degrees from world-up
	clearance: number,  -- studs of headroom above (capped)
	part: BasePart,     -- the part under this surfel
}

-- NOTE: horizontal "width" (distance to nearest wall) is intentionally NOT baked.
-- It is redundant with the navmesh boundary edges and is derived cheaply at
-- pathfinding time (portal-edge length + funnel radius offset). Clearance IS
-- baked because vertical headroom cannot be recovered from 2D boundaries.

export type Config = {
	leaf: number?, maxSlope: number?, agentHeight: number?,
	clearCap: number?, maxGroundFootprint: number?,
}

local DEFAULT = {
	leaf = 1,                 -- SVO leaf size (studs)
	maxSlope = 65,            -- max walkable slope (deg); Cocosulx-tested
	agentHeight = 5,          -- reference stand height
	clearCap = 20,            -- clearance raycast cap
	maxGroundFootprint = 400, -- parts wider than this (baseplate) are excluded
}

local UP = Vector3.new(0, 1, 0)

local function merged(cfg)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function isCharacter(p: Instance): boolean
	local a = p.Parent
	while a and a ~= workspace do
		if a:IsA("Model") and a:FindFirstChildOfClass("Humanoid") then return true end
		a = a.Parent
	end
	return false
end

local function cellKey(x: number, z: number): string
	return string.format("%d:%d", math.floor(x), math.floor(z))
end
Floor.cellKey = cellKey

-- Default world part filter: collidable, not terrain, not character,
-- not a huge flat ground slab (handled analytically elsewhere).
function Floor.gatherParts(cfg: Config?): {BasePart}
	local c = merged(cfg)
	local out = {}
	for _, d in ipairs(workspace:GetDescendants()) do
		if d:IsA("BasePart") and d.CanCollide and d.ClassName ~= "Terrain" and not isCharacter(d) then
			if math.max(d.Size.X, d.Size.Z) <= c.maxGroundFootprint then
				table.insert(out, d)
			end
		end
	end
	return out
end

-- Extract surfels from a prebuilt SVO over `parts`.
function Floor.extract(parts: {BasePart}, tree: any, cfg: Config?)
	local c = merged(cfg)
	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Include
	rp.FilterDescendantsInstances = parts

	local surfels: {Surfel} = {}
	local index: { [string]: {Surfel} } = {}

	tree:forEachSolidLeaf(function(ctr: Vector3, h: number)
		local edge = 2 * h
		local top = ctr.Y + h
		for i = 0, edge - 1 do
			for j = 0, edge - 1 do
				local cx = ctr.X - h + 0.5 + i
				local cz = ctr.Z - h + 0.5 + j
				if tree:isSolid(Vector3.new(cx, top + 0.5, cz)) then continue end
				local res = workspace:Raycast(Vector3.new(cx, top + 1, cz), Vector3.new(0, -(edge + 2), 0), rp)
				if not res then continue end
				local n = res.Normal
				local slope = math.deg(math.acos(math.clamp(n:Dot(UP), -1, 1)))
				local isClip = res.Instance.Name:find("ClipRamp") ~= nil
				if not ((slope <= c.maxSlope) or isClip) then continue end
				-- clearance: upward raycast to the real ceiling
				local upRes = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), Vector3.new(0, c.clearCap, 0), rp)
				local clearance = upRes and upRes.Distance or c.clearCap
				local surfel: Surfel = {
					pos = res.Position, normal = n, slope = slope,
					clearance = clearance, part = res.Instance,
				}
				surfels[#surfels + 1] = surfel
				local key = cellKey(cx, cz)
				local bucket = index[key]
				if not bucket then bucket = {}; index[key] = bucket end
				bucket[#bucket + 1] = surfel
			end
		end
	end)

	return { surfels = surfels, index = index, config = c }
end

-- Convenience one-call bake: gather parts, build SVO, extract floor.
-- Returns floorData, tree, parts.
function Floor.build(cfg: Config?)
	local c = merged(cfg)
	local parts = Floor.gatherParts(c)
	local tree = SVO.fromParts(parts, c.leaf, 2)
	local data = Floor.extract(parts, tree, c)
	return data, tree, parts
end

return Floor
