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

	-- AGENT BODY. Width 4, height 5, thickness 1: it stands 5 studs tall and
	-- needs 4 studs of lateral room, so its radius is 2. Width is resolved here
	-- and not in the bake, by an explicit earlier decision -- the generator emits
	-- nothing width-related, because an agent of any size must be servable
	-- without re-baking.
	--
	-- The binding horizontal dimension is the LARGER of width and thickness. A
	-- character turns to face where it is going, so the 4-stud width sweeps the
	-- corridor; assuming it could squeeze through on its 1-stud edge would be
	-- betting the route on an orientation nothing guarantees.
	agentHeight = 5,
	agentRadius = 2,
	-- refuse to smooth a corner so tight the body would clip the geometry
	validateSolid = true,

	-- WHERE THE BODY IS ACTUALLY ENFORCED, and why not in the graph.
	--
	-- Both obvious graph filters are proxies, and both are wrong here:
	--
	--   Per-polygon `minClearance` is the MINIMUM over the whole polygon, so a
	--   floor with one low spot under a stairwell is excluded entirely. That is
	--   the atomicity problem the clearance design already flags -- the scalar is
	--   a conservative PRE-FILTER, not a verdict. Enforced per polygon it cut
	--   routing from 11% to 2%, excluding 1,302 polygons of 6,747.
	--
	--   Portal LENGTH is a doorway width only at a real constriction. Most
	--   portals here are internal cuts between two pieces of the same open
	--   floor, where the shared edge is short but the agent walks across freely.
	--   Step portals average 3.8 studs for that reason, so a 4-stud body was
	--   refused ground it can plainly walk: routing 11% -> 3%.
	--
	-- So the body is enforced where the agent actually goes -- swept along the
	-- smoothed route by `bodyFits`, at its true 4x5 size, against real collision
	-- geometry. That is exact rather than a proxy, and it cannot over-exclude a
	-- polygon for a low spot the route never visits.
	enforcePolyClearance = false,
	enforcePortalWidth = false,

	-- Agent mobility. These belong to the AGENT, not to the bake, which is why
	-- they live here as query parameters and not as baked flags.
	linkSteps = true,
	maxStepUp = 2.2,     -- a Roblox humanoid auto-steps 2; a little slack for authored lips
	maxStepDown = 8,     -- dropping is cheap; this is not a fall-damage model
	stepProbe = 1.0,     -- how far past the edge to look for the other surface
	-- Probe every this many studs ALONG an edge, not once at its midpoint. This
	-- also sets how accurately a step portal's LENGTH is measured, and that
	-- length is what the width filter judges: at a 4-stud pitch a genuinely wide
	-- contact was being recorded as a narrow one, and only 22% of step portals
	-- came out wide enough for a 4-stud body that could plainly walk them.
	stepSample = 1.5,

	-- How closely a smoothed segment must hug the corridor surface. Slack here
	-- is what lets a shortcut float above the ground it is supposed to follow.
	corridorSample = 1.0,
	corridorTol = 1.5,
	-- height change below which a step link is just a lip, not a step worth
	-- pinning two waypoints for
	stepPinMin = 0.75,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	-- The headroom filter IS the agent's standing height unless something has
	-- deliberately overridden it. Leaving it nil silently routed a 5-stud body
	-- through crawl space, which the bake had measured correctly all along.
	if c.minClearance == nil and c.enforcePolyClearance then c.minClearance = c.agentHeight end
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

--------------------------------------------------------------------------
-- XZ broadphase over polygons.
--
-- `linkSteps` and `locate` both asked "which polygon is under this point?" by
-- scanning every polygon. At 160 polys that is free. At 5,623 it is not:
-- linkSteps probes several samples along every wall and dropoff edge in the
-- mesh, so the scan is entered tens of thousands of times and the stage went
-- from imperceptible to 136 seconds -- paid again on every `prepare`, i.e. on
-- every Play.
--
-- A polygon is registered in every cell its XZ bounding box covers, so the
-- single cell containing a query point holds every polygon that could possibly
-- contain it. That makes the point query EXACT -- there is no neighbour scan to
-- get wrong, because AABB containment is a necessary condition for polygon
-- containment.
local POLY_CELL = 8

local function polyIndex(polys: {any}, cell: number): any
	local idx: { [string]: {number} } = {}
	for i, p in ipairs(polys) do
		local minx, minz = math.huge, math.huge
		local maxx, maxz = -math.huge, -math.huge
		for _, v in ipairs(p.verts) do
			if v.X < minx then minx = v.X end
			if v.X > maxx then maxx = v.X end
			if v.Z < minz then minz = v.Z end
			if v.Z > maxz then maxz = v.Z end
		end
		for cx = math.floor(minx / cell), math.floor(maxx / cell) do
			for cz = math.floor(minz / cell), math.floor(maxz / cell) do
				local k = cx .. ":" .. cz
				local b = idx[k]
				if not b then b = {}; idx[k] = b end
				b[#b + 1] = i
			end
		end
	end
	return idx
end

local function polysAt(idx: any, x: number, z: number, cell: number): {number}
	return idx[math.floor(x / cell) .. ":" .. math.floor(z / cell)] or {}
end

-- Every polygon whose bounding box lies within `radius` of the point, for the
-- off-mesh nearest-edge fallback.
local function polysNear(idx: any, x: number, z: number, cell: number, radius: number): {number}
	local out: {number} = {}
	local seen: { [number]: boolean } = {}
	for cx = math.floor((x - radius) / cell), math.floor((x + radius) / cell) do
		for cz = math.floor((z - radius) / cell), math.floor((z + radius) / cell) do
			local b = idx[cx .. ":" .. cz]
			if b then
				for _, i in ipairs(b) do
					if not seen[i] then seen[i] = true; out[#out + 1] = i end
				end
			end
		end
	end
	return out
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
	local idx = (mesh._pf and mesh._pf.polyIndex) or polyIndex(mesh.polys, POLY_CELL)

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
						for _, j in ipairs(polysAt(idx, probe.X, probe.Z, POLY_CELL)) do
							local q = mesh.polys[j]
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
					-- A step link is built from HEIGHT DIFFERENCE alone, so a floor
					-- buried under an overlapping slab reads exactly like a kerb:
					-- one stud down, step onto it. It is not a kerb -- that stud is
					-- inside the slab. `liveCoverage` is measured at bake time
					-- against the clearance-validated cell mask that killed those
					-- cells in the first place.
					local minCov = c.minLiveCoverage or 0.10
					local covI = mesh.polys[i].liveCoverage
					if covI == nil then covI = 1 end
					for j, h in pairs(hits) do
						local covJ = mesh.polys[j].liveCoverage
						if covJ == nil then covJ = 1 end
						local key = (i < j) and (i .. ":" .. j) or (j .. ":" .. i)
						if not seen[key] and covI >= minCov and covJ >= minCov then
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
	-- built once here rather than per call: linkSteps below hits it tens of
	-- thousands of times, and `locate` runs every frame the demo re-paths
	mesh._pf = { centroids = centroids, config = c, polyIndex = polyIndex(mesh.polys, POLY_CELL) }
	mesh._pf.stepLinks = c.linkSteps and Pathfinder.linkSteps(mesh, c) or 0
	return mesh
end

function Pathfinder.locate(mesh: any, pos: Vector3, cfg: Config?): number?
	local c = (mesh._pf and mesh._pf.config) or merged(cfg)
	local idx = (mesh._pf and mesh._pf.polyIndex) or polyIndex(mesh.polys, POLY_CELL)
	local best, bestScore = nil, math.huge
	for _, i in ipairs(polysAt(idx, pos.X, pos.Z, POLY_CELL)) do
		local p = mesh.polys[i]
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
	for _, i in ipairs(polysNear(idx, pos.X, pos.Z, POLY_CELL, c.searchRadius)) do
		local p = mesh.polys[i]
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
			-- WIDTH, resolved here rather than baked. A portal is the ground the
			-- two polygons actually share, so its length is the width of the gap
			-- an agent has to pass through; a 4-stud body does not fit a 2-stud
			-- doorway however walkable both sides are.
			if c.enforcePortalWidth and c.agentRadius and c.agentRadius > 0 then
				local pw = mesh.portals[nb.portal]
				if pw and pw.length and pw.length < c.agentRadius * 2 - 1e-3 then blocked = true end
			end
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

-- Does the AGENT'S BODY fit along this shortcut?
--
-- Corridor containment alone only asks whether the LINE stays over walkable
-- polygons. A 4-stud-wide body cutting a corner clips the wall long before its
-- centre line leaves the corridor, and where the mesh still covers a little
-- solid, containment is satisfied by a polygon that should not be there.
--
-- An overlap probe, never a raycast: a ray never hits the part its origin is
-- inside, and these samples sit right against surfaces. That trap has bitten
-- this project in four separate probes.
local function bodyFits(mesh: any, a: Vector3, b: Vector3, c: any): boolean
	if not c.validateSolid or not c.agentRadius or c.agentRadius <= 0 then return true end
	local pf = mesh._pf
	if not pf then return true end
	local probe = pf.bodyProbe
	if not probe or not probe.Parent then
		probe = Instance.new("Part")
		probe.Name = "NVGN_BodyProbe"
		probe.Anchored = true
		probe.CanCollide = false
		probe.CanQuery = false      -- a debug part that answers queries gets baked as floor
		probe.CanTouch = false
		probe.Transparency = 1
		probe.Size = Vector3.new(c.agentRadius * 2, c.agentHeight, c.agentRadius * 2)
		probe.Parent = workspace
		pf.bodyProbe = probe
		local op = OverlapParams.new()
		op.FilterType = Enum.RaycastFilterType.Exclude
		op.FilterDescendantsInstances = { probe, workspace:FindFirstChild("NVGN_Debug") }
		pf.bodyParams = op
	end
	local d = (b - a).Magnitude
	local n = math.max(1, math.floor(d / math.max(c.agentRadius, 0.5)))
	for k = 0, n do
		local p = a:Lerp(b, k / n)
		-- the body stands ON the surface, so its centre is half a height up
		probe.CFrame = CFrame.new(p + Vector3.new(0, c.agentHeight * 0.5, 0))
		if #workspace:GetPartsInPart(probe, pf.bodyParams) > 0 then return false end
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
			local nextPoly = polyPath[k + 1]
			local q = nextPoly and mesh.polys[nextPoly]

			-- Pin a step only where there is a step to pin. Pinning EVERY step
			-- link forced two waypoints at every 0.2-stud lip, and on a
			-- staircase -- one link per tread -- that is what turned a straight
			-- flight into a zigzag climbing through the air.
			local dy = 0
			if q and containsXZ(q.verts, mid.X, mid.Z) then
				dy = heightAt(q.verts, mid.X, mid.Z) - mid.Y
			elseif q then
				dy = (mesh._pf.centroids[nextPoly].Y) - mid.Y
			end

			if pt.kind == "step" and q and math.abs(dy) >= c.stepPinMin then
				raw[#raw + 1] = mid
				pinned[#raw] = true
				-- step just inside the polygon being entered, and drop onto it
				local cen = mesh._pf.centroids[nextPoly]
				local dir = Vector3.new(cen.X - mid.X, 0, cen.Z - mid.Z)
				local landing = mid
				if dir.Magnitude > 1e-3 then
					landing = mid + dir.Unit * math.min(c.stepProbe * 1.5, dir.Magnitude * 0.5)
				end
				-- The landing must sit on the polygon being ENTERED, near the
				-- portal. Falling back to the centroid's height put the waypoint
				-- at the height of somewhere else entirely on a large polygon,
				-- which is the waypoint hanging in mid-air over the stairs.
				local y
				if containsXZ(q.verts, landing.X, landing.Z) then
					y = heightAt(q.verts, landing.X, landing.Z)
				else
					landing = mid
					y = mid.Y + dy
				end
				raw[#raw + 1] = Vector3.new(landing.X, y, landing.Z)
				pinned[#raw] = true
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
			if not crossesPin
				and segmentInCorridor(mesh, corridor, raw[i], raw[j], c.corridorSample, c.corridorTol)
				and bodyFits(mesh, raw[i], raw[j], c) then
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
