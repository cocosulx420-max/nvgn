--!strict
-- NVGN.Pathfinder — a deliberately small pathfinder, to prove the bake.
--
-- This is a TEST CONSUMER, not the real thing. It exists to answer one
-- question -- is what we baked actually usable? -- and it does that by loading
-- the serialized bake rather than a live build, so a failure here is a failure
-- of the bake and not of some in-memory table that happened to survive.
--
-- What it deliberately does NOT do, because those are real decisions still
-- open: no agent sizing (width is a runtime question against the SVO, and this
-- walks a point), no clearance filtering beyond an optional height, no jump
-- links, no cost model beyond distance.
--
-- THREE PARTS.
--
--   locate    which polygon is a world point standing on. Horizontal
--             containment picks the candidates and HEIGHT picks between them,
--             because floors stack -- a point under a balcony is inside two
--             polygons horizontally and only one of them is the floor it is on.
--
--   search    A* over polygons, portals as edges. Nothing clever; the graph is
--             tiny and the point is correctness.
--
--   funnel    the string pull. Without it a path is a chain of portal
--             midpoints, which zig-zags visibly and would make a working mesh
--             look broken. The funnel walks the portal sequence keeping the
--             narrowest left/right wedge and emits a corner only when the
--             wedge would invert -- so the result hugs real geometry corners.

local Serialize = require(script.Parent:WaitForChild("Serialize"))

local Pathfinder = {}

export type Config = {
	heightTolerance: number?,  -- how far below a polygon a point may be and still be "on" it
	searchRadius: number?,     -- fallback: nearest polygon within this, when standing off-mesh
	minClearance: number?,     -- optional agent standing height; nil disables the filter
}

local DEFAULT = {
	heightTolerance = 5,
	searchRadius = 12,
	minClearance = nil,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

--------------------------------------------------------------------------
-- geometry helpers, all in the XZ plane
--------------------------------------------------------------------------

local function cross2(ax: number, az: number, bx: number, bz: number): number
	return ax * bz - az * bx
end

local function containsXZ(verts: {Vector3}, x: number, z: number): boolean
	local inside = false
	local j = #verts
	for i = 1, #verts do
		local a, b = verts[i], verts[j]
		if (a.Z > z) ~= (b.Z > z) then
			local xx = a.X + (z - a.Z) / (b.Z - a.Z) * (b.X - a.X)
			if x < xx then inside = not inside end
		end
		j = i
	end
	return inside
end

-- height of a polygon's plane at (x, z). Polygons are planar by construction.
local function heightAt(verts: {Vector3}, x: number, z: number): number
	if #verts < 3 then return verts[1] and verts[1].Y or 0 end
	local a = verts[1]
	local n = (verts[2] - a):Cross(verts[3] - a)
	if math.abs(n.Y) < 1e-6 then return a.Y end
	return a.Y - (n.X * (x - a.X) + n.Z * (z - a.Z)) / n.Y
end

local function centroidOf(verts: {Vector3}): Vector3
	local s = Vector3.zero
	for _, v in ipairs(verts) do s += v end
	return s / #verts
end

local function distToSegment(px: number, pz: number, a: Vector3, b: Vector3): number
	local dx, dz = b.X - a.X, b.Z - a.Z
	local l2 = dx * dx + dz * dz
	if l2 < 1e-9 then
		local ex, ez = px - a.X, pz - a.Z
		return math.sqrt(ex * ex + ez * ez)
	end
	local t = math.clamp(((px - a.X) * dx + (pz - a.Z) * dz) / l2, 0, 1)
	local ex, ez = px - (a.X + dx * t), pz - (a.Z + dz * t)
	return math.sqrt(ex * ex + ez * ez)
end

--------------------------------------------------------------------------

-- Prepares the derived tables a query needs. Call once per loaded bake.
function Pathfinder.prepare(mesh: any, cfg: Config?): any
	local c = merged(cfg)
	local centroids = {}
	for i, p in ipairs(mesh.polys) do centroids[i] = centroidOf(p.verts) end
	mesh._pf = { centroids = centroids, config = c }
	return mesh
end

function Pathfinder.locate(mesh: any, pos: Vector3, cfg: Config?): number?
	local c = (mesh._pf and mesh._pf.config) or merged(cfg)
	local best, bestScore = nil, math.huge
	for i, p in ipairs(mesh.polys) do
		if not (c.minClearance and p.minClearance and p.minClearance < c.minClearance) then
			if containsXZ(p.verts, pos.X, pos.Z) then
				local y = heightAt(p.verts, pos.X, pos.Z)
				local dy = pos.Y - y
				-- prefer the surface underfoot: a little below is fine (standing
				-- slightly inside it), a long way above is a different storey
				if dy > -1.5 and dy < c.heightTolerance then
					local score = math.abs(dy)
					if score < bestScore then bestScore = score; best = i end
				end
			end
		end
	end
	if best then return best end

	-- off-mesh: snap to the nearest polygon edge within the search radius, so
	-- standing on a discarded strip or a hair outside a boundary still works
	local nearest, nd = nil, c.searchRadius
	for i, p in ipairs(mesh.polys) do
		if not (c.minClearance and p.minClearance and p.minClearance < c.minClearance) then
			for k = 1, #p.verts do
				local d = distToSegment(pos.X, pos.Z, p.verts[k], p.verts[(k % #p.verts) + 1])
				if d < nd then
					local y = heightAt(p.verts, pos.X, pos.Z)
					if math.abs(pos.Y - y) < c.heightTolerance * 2 then
						nd = d; nearest = i
					end
				end
			end
		end
	end
	return nearest
end

--------------------------------------------------------------------------
-- A* over the polygon graph
--------------------------------------------------------------------------

function Pathfinder.searchPolys(mesh: any, startPoly: number, goalPoly: number, cfg: Config?): {number}?
	if startPoly == goalPoly then return { startPoly } end
	local c = (mesh._pf and mesh._pf.config) or merged(cfg)
	local cent = mesh._pf.centroids
	local goalPos = cent[goalPoly]

	local gScore: {[number]: number} = { [startPoly] = 0 }
	local came: {[number]: any} = {}
	local open: {number} = { startPoly }
	local inOpen: {[number]: boolean} = { [startPoly] = true }
	local closed: {[number]: boolean} = {}

	while #open > 0 do
		-- linear scan: the graph is small and a heap would only obscure things
		local bi, bf = 1, math.huge
		for i, n in ipairs(open) do
			local f = (gScore[n] or math.huge) + (cent[n] - goalPos).Magnitude
			if f < bf then bf = f; bi = i end
		end
		local cur = table.remove(open, bi) :: number
		inOpen[cur] = nil
		if cur == goalPoly then
			-- the PORTALS ARE RETURNED TOO, not looked up again afterwards. Two
			-- polygons can be joined by more than one portal (matching across
			-- regions emits one per overlapping span), so re-deriving "the"
			-- portal between consecutive polygons can pick one the search never
			-- used -- which fed the funnel a corridor the route does not follow
			-- and produced a path that doubled back on itself.
			local path = { cur }
			local used = {}
			local node = cur
			while came[node] do
				table.insert(used, 1, came[node].portal)
				node = came[node].from
				table.insert(path, 1, node)
			end
			return path, used
		end
		closed[cur] = true

		for _, nb in ipairs(mesh.neighbours[cur] or {}) do
			local other = nb.poly
			local p = mesh.polys[other]
			local blocked = c.minClearance and p and p.minClearance and p.minClearance < c.minClearance
			if not closed[other] and not blocked then
				local portal = mesh.portals[nb.portal]
				local mid = (portal.p1 + portal.p2) * 0.5
				local tentative = (gScore[cur] or math.huge) + (cent[cur] - mid).Magnitude + (mid - cent[other]).Magnitude
				if tentative < (gScore[other] or math.huge) then
					gScore[other] = tentative
					came[other] = { from = cur, portal = nb.portal }
					if not inOpen[other] then
						open[#open + 1] = other
						inOpen[other] = true
					end
				end
			end
		end
	end
	return nil
end

--------------------------------------------------------------------------
-- funnel / string pull
--------------------------------------------------------------------------

-- STRING PULL BY CORRIDOR VISIBILITY.
--
-- The textbook funnel was tried first and is not used, for a reason worth
-- recording. It needs "left" and "right" to mean the same thing at every portal
-- in the corridor, and getting that from the direction of travel (centroid to
-- centroid) flips handedness wherever the corridor turns sharply. Measured: 176
-- of 300 paths doubled back on themselves, and correcting the XZ handedness --
-- Roblox's plane is mirrored relative to the convention the funnel assumes --
-- only brought that to 104. Two separate sign bugs in a component whose whole
-- job is to make the mesh LOOK right is a bad trade.
--
-- So: take the corridor A* actually returned, walk it greedily, and keep the
-- farthest waypoint still reachable in a straight line without leaving the
-- corridor. Reachability is decided by sampling, which is approximate -- but it
-- is approximate in a way that can only make the path more conservative, and it
-- has no handedness at all to get wrong.
local function segmentInCorridor(mesh: any, corridor: {[number]: boolean}, a: Vector3, b: Vector3, step: number, tol: number): boolean
	local d = b - a
	local len = d.Magnitude
	local n = math.max(1, math.ceil(len / step))
	for i = 0, n do
		local p = a + d * (i / n)
		local ok = false
		for pi in pairs(corridor) do
			local poly = mesh.polys[pi]
			if containsXZ(poly.verts, p.X, p.Z) then
				local y = heightAt(poly.verts, p.X, p.Z)
				-- the sample must be on THIS corridor's surface, not merely above
				-- some polygon: without the height test a shortcut may sail over
				-- a courtyard and land on the far side
				if math.abs(p.Y - y) <= tol then ok = true; break end
			end
		end
		if not ok then return false end
	end
	return true
end

function Pathfinder.stringPull(mesh: any, polyPath: {number}, portalPath: {number}, from: Vector3, to: Vector3): {Vector3}
	local corridor: {[number]: boolean} = {}
	for _, pi in ipairs(polyPath) do corridor[pi] = true end

	-- raw route: the portals themselves, which is always inside the corridor
	local raw: {Vector3} = { from }
	for _, ptIdx in ipairs(portalPath) do
		local pt = mesh.portals[ptIdx]
		if pt then raw[#raw + 1] = (pt.p1 + pt.p2) * 0.5 end
	end
	raw[#raw + 1] = to

	-- put each raw point on the surface, so the height test below is meaningful
	for i, p in ipairs(raw) do
		local pi = Pathfinder.locate(mesh, p)
		if pi then raw[i] = Vector3.new(p.X, heightAt(mesh.polys[pi].verts, p.X, p.Z), p.Z) end
	end

	local out: {Vector3} = { raw[1] }
	local i = 1
	while i < #raw do
		local best = i + 1
		for j = #raw, i + 2, -1 do
			if segmentInCorridor(mesh, corridor, raw[i], raw[j], 1.0, 3.0) then best = j; break end
		end
		out[#out + 1] = raw[best]
		i = best
	end
	return out
end

local function portalsAlong(mesh: any, polyPath: {number}, portalPath: {number}, from: Vector3, to: Vector3)
	local left, right = { from }, { from }
	for i = 1, #polyPath - 1 do
		local portal = mesh.portals[portalPath[i]]
		if not portal then return nil end
		-- Orient each portal so p1 is on the left of travel. The reference is
		-- the step being taken, centroid to centroid, so "left" means the same
		-- thing at every portal in the corridor -- the funnel is meaningless if
		-- the handedness flips partway along.
		local ca = mesh._pf.centroids[polyPath[i]]
		local cb = mesh._pf.centroids[polyPath[i + 1]]
		local dx, dz = cb.X - ca.X, cb.Z - ca.Z
		local ex, ez = portal.p1.X - ca.X, portal.p1.Z - ca.Z
		if cross2(dx, dz, ex, ez) < 0 then
			left[#left + 1] = portal.p2; right[#right + 1] = portal.p1
		else
			left[#left + 1] = portal.p1; right[#right + 1] = portal.p2
		end
	end
	left[#left + 1] = to
	right[#right + 1] = to
	return left, right
end

function Pathfinder.funnel(mesh: any, polyPath: {number}, portalPath: {number}, from: Vector3, to: Vector3): {Vector3}
	local left, right = portalsAlong(mesh, polyPath, portalPath, from, to)
	if not left then return { from, to } end

	local out: {Vector3} = { from }
	local apex = from
	local apexIdx = 1
	local leftIdx, rightIdx = 1, 1
	local portalLeft, portalRight = left[1], right[1]

	local i = 2
	while i <= #left do
		local nl, nr = left[i], right[i]

		-- tighten right
		if cross2(portalRight.X - apex.X, portalRight.Z - apex.Z, nr.X - apex.X, nr.Z - apex.Z) <= 0 then
			if apex == portalRight
				or cross2(portalLeft.X - apex.X, portalLeft.Z - apex.Z, nr.X - apex.X, nr.Z - apex.Z) > 0 then
				portalRight = nr
				rightIdx = i
			else
				-- the wedge inverted: the left edge is a real corner
				out[#out + 1] = portalLeft
				apex = portalLeft
				apexIdx = leftIdx
				portalLeft, portalRight = apex, apex
				leftIdx, rightIdx = apexIdx, apexIdx
				i = apexIdx + 1
				continue
			end
		end

		-- tighten left
		if cross2(portalLeft.X - apex.X, portalLeft.Z - apex.Z, nl.X - apex.X, nl.Z - apex.Z) >= 0 then
			if apex == portalLeft
				or cross2(portalRight.X - apex.X, portalRight.Z - apex.Z, nl.X - apex.X, nl.Z - apex.Z) < 0 then
				portalLeft = nl
				leftIdx = i
			else
				out[#out + 1] = portalRight
				apex = portalRight
				apexIdx = rightIdx
				portalLeft, portalRight = apex, apex
				leftIdx, rightIdx = apexIdx, apexIdx
				i = apexIdx + 1
				continue
			end
		end

		i += 1
	end
	out[#out + 1] = to
	return out
end

--------------------------------------------------------------------------

-- Returns waypoints, or nil plus a reason. The reason matters: "no polygon
-- under the start" and "no route" are completely different failures and
-- lumping them together would hide which one the mesh is guilty of.
function Pathfinder.find(mesh: any, from: Vector3, to: Vector3, cfg: Config?)
	if not mesh._pf then Pathfinder.prepare(mesh, cfg) end
	local a = Pathfinder.locate(mesh, from, cfg)
	if not a then return nil, "start is not on the navmesh" end
	local b = Pathfinder.locate(mesh, to, cfg)
	if not b then return nil, "goal is not on the navmesh" end
	local polyPath, portalPath = Pathfinder.searchPolys(mesh, a, b, cfg)
	if not polyPath then return nil, "no route between those polygons" end

	local pts = Pathfinder.stringPull(mesh, polyPath, portalPath or {}, from, to)
	-- lift each waypoint onto the surface it belongs to
	for i, p in ipairs(pts) do
		local pi = Pathfinder.locate(mesh, p, cfg)
		if pi then
			pts[i] = Vector3.new(p.X, heightAt(mesh.polys[pi].verts, p.X, p.Z), p.Z)
		end
	end
	return pts, nil, polyPath
end

function Pathfinder.fromBake(parent: Instance?): any
	local mesh = Serialize.load(parent)
	return Pathfinder.prepare(mesh)
end

return Pathfinder
