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

	-- Agent mobility. These belong to the AGENT, not to the bake, which is why
	-- they live here as query parameters and not as baked flags.
	linkSteps = true,
	maxStepUp = 2.2,     -- a Roblox humanoid auto-steps 2; a little slack for authored lips
	maxStepDown = 8,     -- dropping is cheap; this is not a fall-damage model
	stepProbe = 1.0,     -- how far past the edge to look for the other surface
	stepSample = 4,      -- probe every this many studs ALONG an edge, not once at its midpoint

	-- How closely a smoothed segment must hug the corridor surface. Slack here
	-- is what lets a shortcut float above the ground it is supposed to follow.
	corridorSample = 1.0,
	corridorTol = 1.5,
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

-- STEP LINKS, built at load time and NOT baked.
--
-- This is the half of the mesh contract the bake deliberately leaves to us. A
-- step is stored as a WALL from below and a DROPOFF from above, and never as a
-- portal, because whether it can be crossed depends on the agent -- exactly the
-- reasoning that keeps width out of the bake. Pairing those two edges is the
-- pathfinder's job.
--
-- Skipping it is not a small omission. On the test scene 37 of the 44
-- polygons with no portals at all have another polygon within 4 studs of a
-- wall or dropoff edge: 136 contacts, most of them at 1.0-1.5 and 4.0-4.5
-- studs. Every staircase reads as a cliff without this, so upper floors are
-- simply unreachable and the mesh gets blamed for it.
--
-- Links are DIRECTIONAL. Climbing 2 studs and dropping 2 studs are different
-- questions, and a drop you can survive is usually far larger than a step you
-- can climb, so each direction carries its own height change and A* checks it
-- against the agent's limits rather than the graph baking one answer in.
local function outwardNormal(p: any, k: number): Vector3?
	local a, b = p.verts[k], p.verts[(k % #p.verts) + 1]
	local d = b - a
	if d.Magnitude < 1e-6 then return nil end
	local n = Vector3.new(-d.Z, 0, d.X)
	if n.Magnitude < 1e-6 then return nil end
	n = n.Unit
	local mid = (a + b) * 0.5
	-- pick the side that leaves the polygon
	if containsXZ(p.verts, mid.X + n.X * 0.05, mid.Z + n.Z * 0.05) then n = -n end
	return n
end

function Pathfinder.linkSteps(mesh: any, cfg: Config?): number
	local c = (mesh._pf and mesh._pf.config) or merged(cfg)
	local added = 0
	local seen: {[string]: boolean} = {}

	for i, p in ipairs(mesh.polys) do
		for k = 1, #p.verts do
			local cls = p.classes[k]
			if cls == "wall" or cls == "dropoff" then
				local n = outwardNormal(p, k)
				if n then
					local a, b = p.verts[k], p.verts[(k % #p.verts) + 1]
					local span = (b - a).Magnitude
					-- SAMPLE ALONG THE EDGE, not just its midpoint. Polygons here
					-- are large -- a single floor is often one polygon with 90+
					-- stud edges -- so a neighbour touching a short stretch of a
					-- long edge is invisible to a midpoint probe. That is exactly
					-- what severed an entire upper storey: an 800 stud^2 slab met
					-- a 3255 stud^2 slab at the same height, over 20 studs of a
					-- 93-stud edge, and the midpoint landed nowhere near it.
					-- Record WHERE ALONG THE EDGE each neighbour is actually met, and
					-- make the portal that stretch only.
					--
					-- Using the whole edge instead produced portals up to 121
					-- studs long, 39 of them over 20 studs. A portal's midpoint is
					-- a waypoint, so a portal far longer than the real contact
					-- puts a waypoint out over open air: 52% of paths left the
					-- navmesh entirely, with unsupported runs up to ~52 studs.
					-- A portal must be the ground you can actually cross on.
					local steps = math.max(1, math.ceil(span / c.stepSample))
					local hits: {[number]: any} = {}
					for s = 0, steps do
						local t = (s + 0.5) / (steps + 1)
						local at = a + (b - a) * t
						local probe = at + n * c.stepProbe
						for j, q in ipairs(mesh.polys) do
							if j ~= i and containsXZ(q.verts, probe.X, probe.Z) then
								local dy = heightAt(q.verts, probe.X, probe.Z) - at.Y
								if dy <= c.maxStepUp and dy >= -c.maxStepDown then
									local h = hits[j]
									if not h then
										hits[j] = { tmin = t, tmax = t, dySum = dy, n = 1 }
									else
										h.tmin = math.min(h.tmin, t)
										h.tmax = math.max(h.tmax, t)
										h.dySum += dy
										h.n += 1
									end
								end
							end
						end
					end

					local half = (0.5 / (steps + 1)) -- half a sample spacing, in t
					for j, h in pairs(hits) do
						local key = (i < j) and (i .. ":" .. j) or (j .. ":" .. i)
						if not seen[key] then
							seen[key] = true
							-- widen by half a sample either way so a single-sample
							-- contact is not a zero-length portal
							local t0 = math.max(0, h.tmin - half)
							local t1 = math.min(1, h.tmax + half)
							local p1 = a + (b - a) * t0
							local p2 = a + (b - a) * t1
							local dy = h.dySum / h.n
							mesh.portals[#mesh.portals + 1] = {
								a = i, b = j, p1 = p1, p2 = p2,
								length = (p2 - p1).Magnitude, class = "step", kind = "step",
							}
							local pi = #mesh.portals
							table.insert(mesh.neighbours[i], { poly = j, portal = pi, climb = dy })
							table.insert(mesh.neighbours[j], { poly = i, portal = pi, climb = -dy })
							added += 1
						end
					end
				end
			end
		end
	end
	return added
end

-- Removes previously added step links, so `prepare` can be called again for a
-- different agent without stacking a second set on top of the first. Without
-- this, re-preparing silently doubled the links and made an A/B comparison
-- meaningless -- the "off" run was still using the links from the load.
local function stripStepLinks(mesh: any)
	local keep = {}
	local remap: {[number]: number} = {}
	for i, pt in ipairs(mesh.portals) do
		if pt.kind ~= "step" then
			keep[#keep + 1] = pt
			remap[i] = #keep
		end
	end
	mesh.portals = keep
	for i = 1, #mesh.polys do
		local list = mesh.neighbours[i] or {}
		local out = {}
		for _, nb in ipairs(list) do
			if remap[nb.portal] then
				out[#out + 1] = { poly = nb.poly, portal = remap[nb.portal], climb = nb.climb }
			end
		end
		mesh.neighbours[i] = out
	end
end

-- Prepares the derived tables a query needs. Safe to call again to re-target a
-- different agent.
function Pathfinder.prepare(mesh: any, cfg: Config?): any
	local c = merged(cfg)
	local centroids = {}
	for i, p in ipairs(mesh.polys) do centroids[i] = centroidOf(p.verts) end
	stripStepLinks(mesh)
	mesh._pf = { centroids = centroids, config = c }
	mesh._pf.stepLinks = c.linkSteps and Pathfinder.linkSteps(mesh, c) or 0
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
			-- step links are directional, and the graph may have been built for a
			-- different agent than the one asking now, so the limits are checked
			-- here too rather than trusted from link time
			local climb = nb.climb
			if climb and (climb > c.maxStepUp or climb < -c.maxStepDown) then blocked = true end
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

function Pathfinder.stringPull(mesh: any, polyPath: {number}, portalPath: {number}, from: Vector3, to: Vector3, cfg: Config?): {Vector3}
	local c = (mesh._pf and mesh._pf.config) or merged(cfg)
	local corridor: {[number]: boolean} = {}
	for _, pi in ipairs(polyPath) do corridor[pi] = true end

	-- Raw route: the portals themselves, which lie on the corridor by
	-- construction.
	--
	-- A STEP PORTAL CONTRIBUTES TWO POINTS, not one. Its midpoint sits on the
	-- UPPER polygon's edge, and the next waypoint can be most of a floor away,
	-- so a single point lets the route interpolate the whole height change
	-- across the next polygon -- the path leaves the ledge and glides diagonally
	-- through open air until it finally meets the lower floor. Measured, that
	-- was 55% of raw paths off the mesh with unsupported runs up to 40 studs.
	-- Pinning the bottom of the step makes the descent happen AT the ledge,
	-- which is both what it looks like on the ground and what keeps the route
	-- supported.
	local raw: {Vector3} = { from }
	local pinned: {[number]: boolean} = {}   -- indices that must not be smoothed across
	for k, ptIdx in ipairs(portalPath) do
		local pt = mesh.portals[ptIdx]
		if pt then
			local mid = (pt.p1 + pt.p2) * 0.5
			if pt.kind == "step" then
				local nextPoly = polyPath[k + 1]
				local q = nextPoly and mesh.polys[nextPoly]
				raw[#raw + 1] = mid
				pinned[#raw] = true
				if q then
					-- step just inside the polygon being entered, and drop onto it
					local cen = mesh._pf.centroids[nextPoly]
					local dir = Vector3.new(cen.X - mid.X, 0, cen.Z - mid.Z)
					local landing = mid
					if dir.Magnitude > 1e-3 then
						landing = mid + dir.Unit * math.min(c.stepProbe * 1.5, dir.Magnitude * 0.5)
					end
					local y = containsXZ(q.verts, landing.X, landing.Z)
						and heightAt(q.verts, landing.X, landing.Z)
						or heightAt(q.verts, cen.X, cen.Z)
					raw[#raw + 1] = Vector3.new(landing.X, y, landing.Z)
					pinned[#raw] = true
				end
			else
				raw[#raw + 1] = mid
			end
		end
	end
	raw[#raw + 1] = to

	-- put each raw point on the surface, so the height test below is meaningful
	for i, p in ipairs(raw) do
		if not pinned[i] then
			local pi = Pathfinder.locate(mesh, p)
			if pi then raw[i] = Vector3.new(p.X, heightAt(mesh.polys[pi].verts, p.X, p.Z), p.Z) end
		end
	end

	-- Smoothing may never shortcut PAST a pinned point: doing so re-creates the
	-- diagonal the pinning exists to remove.
	local out: {Vector3} = { raw[1] }
	local i = 1
	while i < #raw do
		local best = i + 1
		for j = #raw, i + 2, -1 do
			local crossesPin = false
			for k = i + 1, j - 1 do
				if pinned[k] then crossesPin = true; break end
			end
			if not crossesPin and segmentInCorridor(mesh, corridor, raw[i], raw[j], c.corridorSample, c.corridorTol) then
				best = j
				break
			end
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

function Pathfinder.fromBake(parent: Instance?, cfg: Config?): any
	local mesh = Serialize.load(parent)
	return Pathfinder.prepare(mesh, cfg)
end

return Pathfinder
