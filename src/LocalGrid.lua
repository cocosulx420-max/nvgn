--!strict
-- NVGN.LocalGrid — per-part, part-aligned sampling grids (approach B: surface-aligned)
--
-- The global floor grid (Floor.lua) samples on WORLD axes, so the edges of any
-- rotated part staircase against the world lattice. This stage rebuilds the
-- walkable surface as one grid PER PART, aligned to that part's own local axes,
-- so each part's edges fall on whole cell lines (no staircase on the part itself).
--
--   * Block parts  -> approach (B): sample the part's principal top face along its
--                     surface normal. Because we use the FULL local frame (incl.
--                     tilt), a rotated slab ramp is sampled square on its incline.
--   * Non-block    -> Unions / MeshParts / wedges have no meaningful surface axes,
--     (fallback)     so they reuse the world-aligned global surfels for that part.
--
-- Feeds the boundary stage (edges come from geometry; the local grid is the
-- interior fill/tessellation guide) and later polygonization.

local Floor = require(script.Parent:WaitForChild("Floor"))

local LocalGrid = {}

export type Cell = {
	ui: number, vi: number,   -- integer lattice indices in the part's local frame
	pos: Vector3,             -- exact surface position (world)
	normal: Vector3,
	slope: number,            -- degrees from world-up
	clearance: number,        -- studs of vertical headroom (capped)
}

export type Grid = {
	part: BasePart,
	fallback: boolean,        -- true => world-aligned (non-block part)
	origin: Vector3?,         -- face corner (world); block grids only
	u: Vector3?, v: Vector3?, -- in-plane unit axes (world); block grids only
	n: Vector3?,              -- surface normal (world); block grids only
	step: number,
	cells: {Cell},
	index: { [string]: Cell },-- "ui:vi" -> cell
}

export type Config = {
	step: number?, maxSlope: number?, clearCap: number?, minClearance: number?,
}

local DEFAULT = {
	step = 1,           -- local cell size (studs)
	maxSlope = 65,      -- max walkable slope (deg); Cocosulx-tested
	clearCap = 20,      -- clearance raycast cap
	minClearance = 1.5, -- below this a cell isn't standable floor (crawl minimum)
}

local UP = Vector3.new(0, 1, 0)

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function isBlock(p: BasePart): boolean
	return p:IsA("Part") and p.Shape == Enum.PartType.Block
end

local function isClip(p: Instance): boolean
	return p.Name:find("ClipRamp") ~= nil
end

-- Group walkable surfels by the part beneath them.
local function groupByPart(surfels: {any}): { [BasePart]: {any} }
	local byPart: { [BasePart]: {any} } = {}
	for _, s in ipairs(surfels) do
		local b = byPart[s.part]
		if not b then b = {}; byPart[s.part] = b end
		b[#b + 1] = s
	end
	return byPart
end

-- Pick a block's walkable top face: the +/- principal axis whose world normal
-- points most upward. Returns the surface normal, its half-extent, and the two
-- in-plane axes (each { dir, ext }).
local function topFace(part: BasePart)
	local cf = part.CFrame
	local sz = part.Size
	local axes = {
		{ dir = cf.RightVector, ext = sz.X * 0.5 },
		{ dir = cf.UpVector,    ext = sz.Y * 0.5 },
		{ dir = cf.RightVector:Cross(cf.UpVector), ext = sz.Z * 0.5 }, -- local Z basis
	}
	local bi, bn, maxY = 2, cf.UpVector, -math.huge
	for i, a in ipairs(axes) do
		if a.dir.Y > maxY then maxY = a.dir.Y; bi = i; bn = a.dir end
		if -a.dir.Y > maxY then maxY = -a.dir.Y; bi = i; bn = -a.dir end
	end
	local plane = {}
	for i, a in ipairs(axes) do
		if i ~= bi then plane[#plane + 1] = a end
	end
	return bn, axes[bi].ext, plane[1], plane[2]
end

-- Block part: surface-aligned grid over the principal top face.
local function buildBlockGrid(part: BasePart, c: any, filterAll: RaycastParams): Grid
	local n, nExt, ua, va = topFace(part)
	local u, uExt = ua.dir, ua.ext
	local v, vExt = va.dir, va.ext
	local surfaceCenter = part.Position + n * nExt
	local corner = surfaceCenter - u * uExt - v * vExt

	local rpPart = RaycastParams.new()
	rpPart.FilterType = Enum.RaycastFilterType.Include
	rpPart.FilterDescendantsInstances = { part }

	local grid: Grid = {
		part = part, fallback = false, origin = corner,
		u = u, v = v, n = n, step = c.step, cells = {}, index = {},
	}

	local step = c.step
	local nu = math.max(1, math.floor(2 * uExt / step + 1e-6))
	local nv = math.max(1, math.floor(2 * vExt / step + 1e-6))
	local castH = 2 -- studs above the surface to start the (downward-along-normal) ray

	for iu = 0, nu - 1 do
		for iv = 0, nv - 1 do
			local p = corner + u * ((iu + 0.5) * step) + v * ((iv + 0.5) * step)
			local res = workspace:Raycast(p + n * castH, -n * (castH + 0.5), rpPart)
			if not res then continue end
			local slope = math.deg(math.acos(math.clamp(res.Normal:Dot(UP), -1, 1)))
			if not ((slope <= c.maxSlope) or isClip(part)) then continue end
			local upRes = workspace:Raycast(res.Position + Vector3.new(0, 0.15, 0), UP * c.clearCap, filterAll)
			local clearance = upRes and upRes.Distance or c.clearCap
			if clearance < c.minClearance then continue end
			local cell: Cell = {
				ui = iu, vi = iv, pos = res.Position, normal = res.Normal,
				slope = slope, clearance = clearance,
			}
			grid.cells[#grid.cells + 1] = cell
			grid.index[string.format("%d:%d", iu, iv)] = cell
		end
	end
	return grid
end

-- Non-block part: reuse the world-aligned global surfels for this part.
local function buildFallbackGrid(part: BasePart, surfels: {any}, c: any): Grid
	local grid: Grid = {
		part = part, fallback = true, step = c.step, cells = {}, index = {},
	}
	for _, s in ipairs(surfels) do
		if s.clearance < c.minClearance then continue end
		local iu = math.floor(s.pos.X / c.step)
		local iv = math.floor(s.pos.Z / c.step)
		local cell: Cell = {
			ui = iu, vi = iv, pos = s.pos, normal = s.normal,
			slope = s.slope, clearance = s.clearance,
		}
		grid.cells[#grid.cells + 1] = cell
		grid.index[string.format("%d:%d", iu, iv)] = cell
	end
	return grid
end

-- Build per-part local grids from an existing floor extraction.
function LocalGrid.fromFloor(floorData: any, parts: {BasePart}, cfg: Config?)
	local c = merged(cfg)
	local filterAll = RaycastParams.new()
	filterAll.FilterType = Enum.RaycastFilterType.Include
	filterAll.FilterDescendantsInstances = parts

	local byPart = groupByPart(floorData.surfels)
	local grids: { [BasePart]: Grid } = {}
	local nBlock, nFallback, nCells = 0, 0, 0
	for part, sfs in pairs(byPart) do
		local g: Grid
		if isBlock(part) then
			g = buildBlockGrid(part, c, filterAll)
			nBlock += 1
		else
			g = buildFallbackGrid(part, sfs, c)
			nFallback += 1
		end
		grids[part] = g
		nCells += #g.cells
	end

	return {
		grids = grids, config = c,
		stats = { parts = nBlock + nFallback, block = nBlock, fallback = nFallback, cells = nCells },
	}
end

-- Convenience one-call bake: Floor.build + local grids.
-- Returns localData, floorData, tree, parts.
function LocalGrid.build(cfg: Config?)
	local floorData, tree, parts = Floor.build(cfg)
	local data = LocalGrid.fromFloor(floorData, parts, cfg)
	return data, floorData, tree, parts
end

-- Debug viz: one neon tile per cell, colored per-part, oriented to the surface
-- (block grids) so alignment is eyeball-able. Fallback parts are desaturated.
function LocalGrid.visualize(data: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then
		dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root
	end
	local old = dbg:FindFirstChild("LocalGrid")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "LocalGrid"; folder.Parent = dbg

	local step = data.config.step
	local i = 0
	for part, g in pairs(data.grids) do
		i += 1
		local hue = (i * 0.61803398875) % 1
		local col = Color3.fromHSV(hue, g.fallback and 0.3 or 0.9, 1)
		local pf = Instance.new("Folder"); pf.Name = part.Name; pf.Parent = folder
		for _, cell in ipairs(g.cells) do
			local dot = Instance.new("Part")
			dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
			dot.Size = Vector3.new(0.9 * step, 0.1, 0.9 * step)
			dot.Color = col; dot.Material = Enum.Material.Neon
			if not g.fallback and g.n then
				dot.CFrame = CFrame.fromMatrix(cell.pos, g.u, g.n)
			else
				dot.CFrame = CFrame.new(cell.pos)
			end
			dot.Parent = pf
		end
	end
	return folder
end

return LocalGrid
