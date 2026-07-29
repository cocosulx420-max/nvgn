--!strict
-- NVGN.Bounds — boundary extraction on the part-aligned local grids.
--
-- Supersedes the world-lattice attempt in Boundary2. See DESIGN.md, "Substrate
-- for boundaries": the input is LocalGrid's per-part grids, NOT a world lattice
-- over surfel positions. A world lattice staircases every surface that is not
-- aligned to world X/Z, and nothing downstream recovers from that.
--
-- This module covers the first stage: deciding, for every grid-extent cell edge,
-- which of three things it is.
--
--   SEAM     the floor continues into another grid   -> not a boundary at all
--   WALL     the outward neighbour is a DEAD cell    -> killer names the blocker
--   DROPOFF  the outward neighbour is simply absent  -> floor extent, a ledge
--
-- Classification is free: LocalGrid already attributed every dead cell to the
-- part that killed it, at bake time, with a GetPartsInPart probe. Nothing here
-- re-probes the world.

local Bounds = {}

export type Config = {
	stepTol: number?,   -- height difference below which two cells connect
	seamSlack: number?, -- horizontal slack when matching cells across grids
}

local DEFAULT = {
	stepTol = 2.0,
	seamSlack = 0.35,
}

local function merged(cfg: Config?)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

-- Lattice neighbours in a grid's OWN frame.
local NB = {
	{ du = 1, dv = 0 }, { du = -1, dv = 0 },
	{ du = 0, dv = 1 }, { du = 0, dv = -1 },
}

local function key(a: number, b: number): string
	return a .. ":" .. b
end

-- A grid's horizontal axes. Part-aligned grids carry their own u/v; fallback
-- grids (unions, meshes) were built on the world lattice and use world axes.
local function axesOf(g: any): (Vector3, Vector3)
	if not g.fallback and g.u and g.v then
		local u = Vector3.new(g.u.X, 0, g.u.Z)
		u = (u.Magnitude > 1e-4) and u.Unit or Vector3.xAxis
		return u, Vector3.new(-u.Z, 0, u.X)
	end
	return Vector3.xAxis, Vector3.zAxis
end

--------------------------------------------------------------------------------
-- World-space cell lookup
--------------------------------------------------------------------------------
-- Grids have different frames, so adjacency between them cannot be a lattice
-- index comparison. It is a proximity relation, which is legitimate to answer
-- with a world hash: this decides CONNECTIVITY, never geometry. Geometry always
-- comes from a grid's own frame.

local function buildHash(localData: any, step: number)
	local hash: { [string]: { any } } = {}
	local entries = {}
	for part, g in pairs(localData.grids) do
		for _, cell in ipairs(g.cells) do
			local e = { cell = cell, grid = g, part = part, id = #entries + 1 }
			entries[#entries + 1] = e
			local k = key(math.floor(cell.pos.X / step), math.floor(cell.pos.Z / step))
			local b = hash[k]
			if not b then b = {}; hash[k] = b end
			b[#b + 1] = e
		end
	end
	return hash, entries
end

-- Any live cell, in any grid, sitting at this world position within slack.
local function cellNear(hash: any, step: number, p: Vector3, stepTol: number, slack: number, skip: any): any
	local bx, bz = math.floor(p.X / step), math.floor(p.Z / step)
	for ox = -1, 1 do
		for oz = -1, 1 do
			local b = hash[key(bx + ox, bz + oz)]
			if b then
				for _, e in ipairs(b) do
					if e ~= skip then
						local d = e.cell.pos - p
						if math.abs(d.Y) <= stepTol
							and math.abs(d.X) <= slack and math.abs(d.Z) <= slack then
							return e
						end
					end
				end
			end
		end
	end
	return nil
end

--------------------------------------------------------------------------------
-- Components, spanning grids
--------------------------------------------------------------------------------

function Bounds.components(localData: any, cfg: Config?)
	local c = merged(cfg)
	local step = localData.config.step
	local hash, entries = buildHash(localData, step)

	local parent = table.create(#entries)
	for i = 1, #entries do parent[i] = i end
	local function find(a: number): number
		while parent[a] ~= a do parent[a] = parent[parent[a]]; a = parent[a] end
		return a
	end
	local function union(a: number, b: number)
		a, b = find(a), find(b)
		if a ~= b then parent[b] = a end
	end

	-- Step-tolerance adjacency, tested in world space so it crosses grid frames.
	for _, e in ipairs(entries) do
		local u, v = axesOf(e.grid)
		for _, d in ipairs(NB) do
			local probe = e.cell.pos + u * (d.du * step) + v * (d.dv * step)
			local other = cellNear(hash, step, probe, c.stepTol, c.seamSlack, e)
			if other then union(e.id, other.id) end
		end
	end

	local comps, byRoot = {}, {}
	for _, e in ipairs(entries) do
		local r = find(e.id)
		local comp = byRoot[r]
		if not comp then
			comp = { entries = {}, id = #comps + 1 }
			byRoot[r] = comp
			comps[#comps + 1] = comp
		end
		comp.entries[#comp.entries + 1] = e
		e.comp = comp
	end

	return comps, hash, entries
end

--------------------------------------------------------------------------------
-- Classify every grid-extent edge
--------------------------------------------------------------------------------

export type Edge = {
	kind: string,      -- "wall" | "dropoff"
	a: Vector3, b: Vector3,
	outDir: Vector3,
	cell: any, grid: any, part: any,
	killer: Instance?, -- wall only: the part that killed the neighbour cell
	comp: any,
}

function Bounds.classify(localData: any, cfg: Config?)
	local c = merged(cfg)
	local step = localData.config.step
	local comps, hash, entries = Bounds.components(localData, c)

	local edges: { Edge } = {}
	local nSeam, nWall, nDrop = 0, 0, 0

	for _, e in ipairs(entries) do
		local g, cell = e.grid, e.cell
		local u, v = axesOf(g)

		for _, d in ipairs(NB) do
			local nk = key(cell.ui + d.du, cell.vi + d.dv)

			-- 1. still inside this grid: interior, nothing to emit
			if g.index[nk] then continue end

			local dead = g.deadIndex[nk]
			local outDir = u * d.du + v * d.dv
			local half = (d.du ~= 0) and v or u
			local mid = cell.pos + outDir * (step * 0.5)
			local a = mid - half * (step * 0.5)
			local b = mid + half * (step * 0.5)

			if dead then
				-- 2. the neighbour sample was killed -> a wall, and we know by what
				edges[#edges + 1] = {
					kind = "wall", a = a, b = b, outDir = outDir,
					cell = cell, grid = g, part = e.part,
					killer = dead.killer, comp = e.comp,
				}
				nWall += 1
			else
				-- 3. off this grid entirely. Either the floor continues in another
				-- grid (a part seam, not a boundary) or it really stops (a ledge).
				local probe = cell.pos + outDir * step
				if cellNear(hash, step, probe, c.stepTol, c.seamSlack, e) then
					nSeam += 1
				else
					edges[#edges + 1] = {
						kind = "dropoff", a = a, b = b, outDir = outDir,
						cell = cell, grid = g, part = e.part,
						killer = nil, comp = e.comp,
					}
					nDrop += 1
				end
			end
		end
	end

	local killers: { [Instance]: number } = {}
	local nKillers = 0
	for _, ed in ipairs(edges) do
		if ed.killer then
			if killers[ed.killer] == nil then nKillers += 1; killers[ed.killer] = 0 end
			killers[ed.killer] += 1
		end
	end

	return {
		edges = edges,
		components = comps,
		entries = entries,
		hash = hash,
		config = c,
		stats = {
			cells = #entries,
			components = #comps,
			seams = nSeam,
			wallEdges = nWall,
			dropoffEdges = nDrop,
			distinctKillers = nKillers,
		},
	}
end

--------------------------------------------------------------------------------
-- Debug draw
--------------------------------------------------------------------------------

function Bounds.visualize(result: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Bounds")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Bounds"; folder.Parent = dbg
	local fW = Instance.new("Folder"); fW.Name = "Wall"; fW.Parent = folder
	local fD = Instance.new("Folder"); fD.Name = "Dropoff"; fD.Parent = folder

	local WALL = Color3.fromRGB(70, 170, 255)
	local DROP = Color3.fromRGB(120, 255, 130)

	for _, e in ipairs(result.edges) do
		local d = e.b - e.a
		local len = d.Magnitude
		if len > 1e-3 then
			local bar = Instance.new("Part")
			bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
			bar.Size = Vector3.new(len, 0.14, 0.14)
			bar.Color = (e.kind == "wall") and WALL or DROP
			bar.Material = Enum.Material.Neon
			bar.CFrame = CFrame.fromMatrix((e.a + e.b) * 0.5 + Vector3.new(0, 0.25, 0),
				d / len, Vector3.yAxis)
			bar.Parent = (e.kind == "wall") and fW or fD
		end
	end
	return folder
end

return Bounds
