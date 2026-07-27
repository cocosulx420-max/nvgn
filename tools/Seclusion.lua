--!strict
-- tools/Seclusion — which polygons are in space nothing can reach.
--
-- WHY THIS EXISTS. Every reachability test the project had ran on the navmesh
-- graph, which is the thing under suspicion: we were asking a broken mesh
-- whether its own regions were reachable. The SVO is INDEPENDENT evidence. It
-- knows where solid is without consulting a single boundary line, so it can
-- answer "can anything get here at all" even where the boundary stage failed.
--
-- THE TEST. Flood-fill empty space outward from each polygon's air. If the fill
-- escapes to open sky or off the edge of the world, that polygon is in the
-- connected outside and is KEPT. If the fill closes on itself, the polygon sits
-- in a sealed pocket and no agent -- walking, dropping, crawling, double
-- jumping, dashing -- can ever stand on it.
--
-- Seeding from the sky rather than from a spawn point is deliberate. A rooftop
-- with no route up is still open air, so it survives; it is "no route yet", not
-- "no such place", and jump links may change that answer later. Culling it here
-- would pre-empt a decision that is explicitly still open.
--
-- ─────────────────────────────────────────────────────────────────────────────
-- DESIGN NOTES, each of which is a decision rather than an accident
--
-- 6-CONNECTIVITY, NOT 26. This is the INVERSE of the rule that governs surface
-- extraction, where 6 leaves a diagonal lattice of false gaps and 26 is
-- required. For AIR TRAVERSAL 26 is wrong: it leaks diagonally through the
-- corner where two walls meet, and an agent cannot walk through a diagonal
-- seam. A sealed room would silently connect to the outside through its own
-- corner.
--
-- CONSERVATISM POINTS THE RIGHT WAY. The standing objection to the SVO is that
-- 1-stud voxels over-voxelise -- a thin wall eats a whole voxel row. Here that
-- bias makes passages read as blocked, so pockets read as sealed and get
-- removed. Cocosulx's rule for this work is that losing coverage beats allowing
-- an impossible path, so the SVO's known weakness lands on the safe side. This
-- is the same asymmetric-error argument already used for clearance volumes.
--
-- THE FILL IS PERMISSIVE ON PURPOSE. Passability is "this voxel is empty",
-- nothing more -- no agent height, no width. Requiring two empty voxels of
-- headroom would be stricter, and would cull legitimate crawl tunnels, which
-- are valid navmesh by an earlier decision. We are only trying to eliminate
-- space reachable by NOBODY; anything reachable by the smallest agent stays,
-- and per-agent fitness is a query-time question against clearance and the SVO,
-- which is where width was deliberately put.
--
-- BUDGET OVERRUN MEANS OPEN, NOT SEALED. Ambiguity normally means drop, but not
-- here: a fill that has visited tens of thousands of voxels without closing is
-- an open space almost by definition, and calling it sealed would delete a
-- storey. The budget exists to bound cost, not to make judgements.
--
-- THE SVO NEVER SUPPLIES A COORDINATE. It selects whole polygons and nothing
-- else. No voxel boundary becomes a polygon edge -- that is what produced the
-- staircase this project exists to kill, and it is why this module returns
-- flags rather than geometry.
--
-- FLAG, DO NOT DELETE. A flagged polygon is impassable to the pathfinder, so
-- the behaviour matches deletion, but the geometry survives for jump links to
-- reconsider. The standing instruction is to bake the opportunity, not the
-- verdict.

local Seclusion = {}

export type Config = {
	step: number?,        -- fill resolution, in studs
	seedUp: number?,      -- how far above a polygon's surface to seed the fill
	budget: number?,      -- max voxels per fill before we call it open
	skyMargin: number?,   -- how far above the tallest geometry counts as escaped
	pad: number?,         -- how far outside the world AABB counts as escaped
}

-- SEEDING IS THE FIDDLY PART, and it is where the SVO's conservatism actually
-- bites. A fixed offset above a polygon does not work: one stud above a stair
-- tread lands inside the NEXT tread's voxel. All 8 "buried" verdicts on the
-- first small-map run were exactly that -- 20 studs of open sky overhead,
-- reported solid.
--
-- Worse, a single column is not enough either. Voxelising TILTED geometry marks
-- whole voxels solid that are mostly air, so a ramp passing above a floor can
-- make one column read solid for 4+ studs where an overlap probe finds nothing
-- at any height. That was the last remaining false positive.
--
-- So: scan upward for the first empty voxel (seedRise), and do it from several
-- interior points, not just the centroid. One bad column must not condemn a
-- polygon.
local DEFAULT = {
	step = 1,
	seedUp = 0.5,
	seedRise = 3.0,
	budget = 20000,
	skyMargin = 4,
	pad = 4,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function centroidOf(verts: {Vector3}): Vector3
	local s = Vector3.zero
	for _, v in ipairs(verts) do s += v end
	return s / #verts
end

-- 6-connectivity. See the note above: 26 would leak through wall corners.
local NEIGHBOURS = {
	Vector3.new(1, 0, 0), Vector3.new(-1, 0, 0),
	Vector3.new(0, 1, 0), Vector3.new(0, -1, 0),
	Vector3.new(0, 0, 1), Vector3.new(0, 0, -1),
}

--------------------------------------------------------------------------

-- `bounds` is the world AABB of the parts the SVO was built from.
function Seclusion.classify(polys: {any}, tree: any, bounds: {min: Vector3, max: Vector3}, cfg: Config?)
	local c = merged(cfg)
	local step = c.step
	local skyY = bounds.max.Y + c.skyMargin
	local loX, hiX = bounds.min.X - c.pad, bounds.max.X + c.pad
	local loZ, hiZ = bounds.min.Z - c.pad, bounds.max.Z + c.pad
	local floorY = bounds.min.Y - c.pad

	-- voxel -> component id. Shared across fills so a second polygon in the
	-- same pocket costs one lookup instead of a second flood.
	local voxComp: {[string]: number} = {}
	local compOpen: {[number]: boolean} = {}
	local compSize: {[number]: number} = {}
	local nextComp = 0

	local function key(x: number, y: number, z: number): string
		return x .. ":" .. y .. ":" .. z
	end
	local function toVox(p: Vector3): (number, number, number)
		return math.floor(p.X / step), math.floor(p.Y / step), math.floor(p.Z / step)
	end
	local function centreOf(x: number, y: number, z: number): Vector3
		return Vector3.new((x + 0.5) * step, (y + 0.5) * step, (z + 0.5) * step)
	end
	local function escaped(x: number, y: number, z: number): boolean
		local p = centreOf(x, y, z)
		return p.Y >= skyY or p.Y <= floorY
			or p.X <= loX or p.X >= hiX or p.Z <= loZ or p.Z >= hiZ
	end

	local stats = {
		polys = #polys, sealed = 0, open = 0, buried = 0,
		components = 0, sealedComponents = 0, budgetHits = 0, fills = 0,
	}
	local result: {any} = {}

	for i, poly in ipairs(polys) do
		-- see the note on DEFAULT: scan upward, and from several points
		local cen = centroidOf(poly.verts)
		local seeds = { cen }
		for _, v in ipairs(poly.verts) do
			seeds[#seeds + 1] = cen + (v - cen) * 0.5
			seeds[#seeds + 1] = cen + (v - cen) * 0.8
		end
		local sx, sy, sz, found
		for _, base in ipairs(seeds) do
			local rise = c.seedUp
			while rise <= c.seedRise do
				local vx, vy, vz = toVox(base + Vector3.new(0, rise, 0))
				if not tree:isSolid(centreOf(vx, vy, vz)) then
					sx, sy, sz, found = vx, vy, vz, true
					break
				end
				rise += step
			end
			if found then break end
		end
		if not found then sx, sy, sz = toVox(cen + Vector3.new(0, c.seedUp, 0)) end
		local sk = key(sx, sy, sz)

		local verdict, comp
		if not found then
			-- the polygon's own standing space is inside geometry. Not a
			-- seclusion verdict -- reported separately, because the fix for it
			-- is emission, not culling.
			verdict = "buried"
			stats.buried += 1
		else
			comp = voxComp[sk]
			if comp then
				if compOpen[comp] then verdict = "open" else verdict = "sealed" end
			else
				-- bounded BFS
				stats.fills += 1
				nextComp += 1
				comp = nextComp
				local visited: {string} = { sk }
				voxComp[sk] = comp
				local stack = { { sx, sy, sz } }
				local n = 0
				local isOpen = false
				while #stack > 0 do
					local v = table.remove(stack)
					local x, y, z = v[1], v[2], v[3]
					n += 1
					if escaped(x, y, z) then isOpen = true; break end
					if n > c.budget then
						-- see the header: an overrun is evidence of open space,
						-- not of enclosure
						isOpen = true
						stats.budgetHits += 1
						break
					end
					for _, d in ipairs(NEIGHBOURS) do
						local nx, ny, nz = x + d.X, y + d.Y, z + d.Z
						local nk = key(nx, ny, nz)
						if voxComp[nk] == nil then
							if not tree:isSolid(centreOf(nx, ny, nz)) then
								voxComp[nk] = comp
								visited[#visited + 1] = nk
								stack[#stack + 1] = { nx, ny, nz }
							end
						end
					end
				end
				compOpen[comp] = isOpen
				compSize[comp] = n
				stats.components += 1
				if isOpen then verdict = "open" else verdict = "sealed"; stats.sealedComponents += 1 end
			end
		end

		if verdict == "sealed" then stats.sealed += 1
		elseif verdict == "open" then stats.open += 1 end

		result[i] = {
			poly = i,
			verdict = verdict,
			component = comp,
			-- what the pathfinder consumes. Buried polygons are NOT flagged
			-- unreachable here: that is a different defect with a different fix,
			-- and conflating them would hide it.
			reachable = (verdict ~= "sealed"),
		}
	end

	return { verdicts = result, stats = stats, compSize = compSize, compOpen = compOpen }
end

--------------------------------------------------------------------------

-- Diagnostic only. Green = open, red = sealed, yellow = buried.
function Seclusion.visualize(polys: {any}, res: any, parent: Instance?)
	local host = parent or workspace
	local root = host:FindFirstChild("NVGN_Debug")
	if not root then
		root = Instance.new("Folder")
		root.Name = "NVGN_Debug"
		root.Parent = host
	end
	local old = root:FindFirstChild("Seclusion")
	if old then old:Destroy() end
	local folder = Instance.new("Folder")
	folder.Name = "Seclusion"
	folder.Parent = root

	local COLOUR = {
		open = Color3.new(0.2, 1.0, 0.35),
		sealed = Color3.new(1.0, 0.15, 0.15),
		buried = Color3.new(0.95, 0.85, 0.10),
	}

	for i, poly in ipairs(polys) do
		local v = res.verdicts[i]
		if v and v.verdict ~= "open" then
			local c = centroidOf(poly.verts)
			local marker = Instance.new("Part")
			marker.Name = string.format("%s_%d", v.verdict, i)
			marker.Anchored = true
			-- debug parts must never answer a spatial query: the next bake would
			-- pick them up as walkable floor
			marker.CanCollide = false
			marker.CanQuery = false
			marker.CanTouch = false
			marker.Material = Enum.Material.Neon
			marker.Color = COLOUR[v.verdict] or Color3.new(1, 1, 1)
			marker.Size = Vector3.new(1.5, 6, 1.5)
			marker.CFrame = CFrame.new(c + Vector3.new(0, 3, 0))
			marker.Transparency = 0.35
			marker.Parent = folder
		end
	end
	return folder
end

return Seclusion
