--!strict
-- NVGN.Loops — boundary edges into closed, oriented walkable regions.
--
-- WHY NOT WALK THE EDGE GRAPH. The obvious assembly is to chain edges end to
-- end. It does not work, and the reason is structural rather than a tolerance
-- problem: on the test scene the per-floor endpoint graph has 196 degree-1
-- vertices and not one vertex of degree 3+. `closedA/closedB` from Clean means
-- an endpoint landed on a COMPUTED vertex — which is frequently produced by a
-- rim virtual line or by a line belonging to another floor — so an endpoint can
-- be perfectly closed while no other edge OF THIS FLOOR ends there. Small slabs
-- routinely emit boundary on two sides only, because neighbours cover the other
-- two and coverage correctly suppresses them. A chain walk sees an open end and
-- has nothing to attach.
--
-- WHAT WE DO INSTEAD. A floor's walkable region is bounded either by an emitted
-- edge or by the edge of the part itself. So take the floor's own rim rectangle
-- as the outer bound, cut it with the emitted boundary lines, and let the
-- resulting planar subdivision hand us closed faces. Then ask the live-cell
-- mask which faces are actually walkable.
--
-- That split of duties is the same one the rest of the pipeline runs on:
--
--   geometry decides WHERE every line is   (exact, no lattice term)
--   the mask decides WHICH faces are real  (selection only, never a coordinate)
--
-- Handovers need no special case. Where a region continues onto an overlapping
-- neighbour, no edge is emitted and the rim closes the loop on its own; the
-- resulting rim-borne loop segment is marked `continuation`, meaning "not an
-- obstacle — the surface carries on past here". Walls, seams and dropoffs keep
-- the class Clean gave them.
--
-- Everything happens in the floor's own 2D frame (u, v from LocalGrid), so
-- tilted and yawed floors are handled without a special case and without ever
-- touching world axes.

local Clean = require(script.Parent:WaitForChild("Clean"))

local Loops = {}

export type LoopEdge = {
	class: string,        -- wall | seam | dropoff | tier | continuation
	source: Instance?,
	a: Vector3, b: Vector3,
}
export type Region = {
	floor: BasePart,
	edges: {LoopEdge},    -- closed ring, CCW in the floor's own frame
	verts: {Vector3},
	area: number,
	cells: number,        -- live mask cells inside
	holes: {{LoopEdge}},   -- inner rings: obstacles standing on this floor
}
export type Config = {
	weldEps: number?, minArea: number?, minCellsPerFace: number?,
}

local DEFAULT = {
	-- Vertices closer than this in the floor frame are the same vertex. Clean
	-- already snapped endpoints onto computed intersections, so this only has to
	-- absorb float noise from re-intersecting those same lines here.
	weldEps = 0.02,
	minArea = 0.05,
	-- A face with no live cell in it is a sliver between two nearly-coincident
	-- lines, not a place anything stands.
	minCellsPerFace = 1,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

--------------------------------------------------------------------------
-- 2D helpers, in the floor's frame
--------------------------------------------------------------------------

type V2 = { x: number, y: number }

local function cross2(ax: number, ay: number, bx: number, by: number): number
	return ax * by - ay * bx
end

-- Segment intersection parameters, or nil when parallel.
local function segX(p: V2, r: V2, q: V2, s: V2): (number?, number?)
	local den = cross2(r.x, r.y, s.x, s.y)
	if math.abs(den) < 1e-9 then return nil, nil end
	local qpx, qpy = q.x - p.x, q.y - p.y
	local t = cross2(qpx, qpy, s.x, s.y) / den
	local u = cross2(qpx, qpy, r.x, r.y) / den
	return t, u
end

local function polyArea(pts: {V2}): number
	local a = 0
	for i = 1, #pts do
		local j = (i % #pts) + 1
		a += cross2(pts[i].x, pts[i].y, pts[j].x, pts[j].y)
	end
	return a * 0.5
end

local function pointInPoly(pts: {V2}, x: number, y: number): boolean
	local inside = false
	local j = #pts
	for i = 1, #pts do
		local pi, pj = pts[i], pts[j]
		if (pi.y > y) ~= (pj.y > y) then
			local xx = pi.x + (y - pi.y) / (pj.y - pi.y) * (pj.x - pi.x)
			if x < xx then inside = not inside end
		end
		j = i
	end
	return inside
end

--------------------------------------------------------------------------
-- Planar subdivision of one floor
--------------------------------------------------------------------------

type Seg = { a: V2, b: V2, class: string, source: Instance? }

-- Cut every segment at every crossing, so the result is a graph whose edges
-- only ever meet at endpoints.
local function splitAll(segs: {Seg}, c: any): {Seg}
	local out: {Seg} = {}
	for i, s in ipairs(segs) do
		local r = { x = s.b.x - s.a.x, y = s.b.y - s.a.y }
		local cuts = { 0, 1 }
		local rlen2 = r.x * r.x + r.y * r.y
		for j, t in ipairs(segs) do
			if i ~= j then
				local sr = { x = t.b.x - t.a.x, y = t.b.y - t.a.y }
				local ta, ub = segX(s.a, r, t.a, sr)
				if ta and ub then
					if ta > 1e-6 and ta < 1 - 1e-6 and ub >= -1e-6 and ub <= 1 + 1e-6 then
						cuts[#cuts + 1] = ta
					end
				else
					-- COLLINEAR OVERLAP. A floor's dropoff edge lies exactly on its
					-- own rim, so this is the common case, not a corner case. segX
					-- reports parallel and returns no parameter, which left the rim
					-- uncut at the emitted edge's ends; the two then coexist as
					-- duplicate overlapping edges and the face walk degenerates into
					-- zero-area out-and-back rings. Project the other segment's
					-- endpoints onto this one instead.
					local offx, offy = t.a.x - s.a.x, t.a.y - s.a.y
					if math.abs(cross2(r.x, r.y, offx, offy)) <= c.weldEps * math.sqrt(rlen2) then
						for _, ep in ipairs({ t.a, t.b }) do
							local tt = ((ep.x - s.a.x) * r.x + (ep.y - s.a.y) * r.y) / rlen2
							if tt > 1e-6 and tt < 1 - 1e-6 then cuts[#cuts + 1] = tt end
						end
					end
				end
			end
		end
		table.sort(cuts)
		for k = 1, #cuts - 1 do
			local t0, t1 = cuts[k], cuts[k + 1]
			if t1 - t0 > 1e-6 then
				local a = { x = s.a.x + r.x * t0, y = s.a.y + r.y * t0 }
				local b = { x = s.a.x + r.x * t1, y = s.a.y + r.y * t1 }
				local dx, dy = b.x - a.x, b.y - a.y
				if dx * dx + dy * dy > c.weldEps * c.weldEps then
					out[#out + 1] = { a = a, b = b, class = s.class, source = s.source }
				end
			end
		end
	end
	return out
end

-- Trace the minimal faces of the planar graph. Standard half-edge walk: at each
-- vertex the successor of an incoming half-edge is the next outgoing one
-- CLOCKWISE, which traces interior faces counter-clockwise and the unbounded
-- face clockwise (negative area) — so the outer face falls out by its sign.
local function traceFaces(segs: {Seg}, c: any)
	local verts: {V2} = {}
	local key: { [string]: number } = {}
	local function vid(p: V2): number
		local k = string.format("%d:%d", math.floor(p.x / c.weldEps + 0.5), math.floor(p.y / c.weldEps + 0.5))
		local id = key[k]
		if not id then
			verts[#verts + 1] = p
			id = #verts
			key[k] = id
		end
		return id
	end

	type Half = { from: number, to: number, class: string, source: Instance?, ang: number, twin: number }
	local halves: {Half} = {}
	-- Once collinear overlaps are cut, a rim segment and the boundary edge lying
	-- on it become the SAME vertex pair. Keep one: the measured edge, which
	-- carries a real class, beats the rim's `continuation` placeholder. Leaving
	-- both in makes the vertex fan ambiguous and the walk retraces itself into
	-- zero-area rings.
	local seen: { [string]: number } = {}
	for _, s in ipairs(segs) do
		local i, j = vid(s.a), vid(s.b)
		local dup = false
		if i ~= j then
			local dk = (i < j) and (i .. "_" .. j) or (j .. "_" .. i)
			local prev = seen[dk]
			if prev then
				dup = true
				if halves[prev].class == "continuation" and s.class ~= "continuation" then
					local tw = halves[prev].twin
					halves[prev].class = s.class; halves[prev].source = s.source
					halves[tw].class = s.class; halves[tw].source = s.source
				end
			else
				seen[dk] = #halves + 1
			end
		end
		if i ~= j and not dup then
			local n = #halves
			halves[n + 1] = {
				from = i, to = j, class = s.class, source = s.source,
				ang = math.atan2(verts[j].y - verts[i].y, verts[j].x - verts[i].x), twin = n + 2,
			}
			halves[n + 2] = {
				from = j, to = i, class = s.class, source = s.source,
				ang = math.atan2(verts[i].y - verts[j].y, verts[i].x - verts[j].x), twin = n + 1,
			}
		end
	end

	local outgoing: { [number]: {number} } = {}
	for hi, h in ipairs(halves) do
		local b = outgoing[h.from]
		if not b then b = {}; outgoing[h.from] = b end
		b[#b + 1] = hi
	end
	for _, b in pairs(outgoing) do
		table.sort(b, function(x, y) return halves[x].ang < halves[y].ang end)
	end
	local slot: { [number]: number } = {}
	for _, b in pairs(outgoing) do
		for idx, hi in ipairs(b) do slot[hi] = idx end
	end

	local faces = {}
	local used: { [number]: boolean } = {}
	for hi = 1, #halves do
		if not used[hi] then
			local ring, guard = {}, 0
			local cur = hi
			repeat
				used[cur] = true
				ring[#ring + 1] = cur
				-- step to the twin, then take the neighbour CLOCKWISE of it
				local tw = halves[cur].twin
				local b = outgoing[halves[tw].from]
				local idx = slot[tw] - 1
				if idx < 1 then idx = #b end
				cur = b[idx]
				guard += 1
			until cur == hi or guard > 100000
			local pts: {V2} = {}
			for _, h in ipairs(ring) do pts[#pts + 1] = verts[halves[h].from] end
			faces[#faces + 1] = { ring = ring, pts = pts, area = polyArea(pts) }
		end
	end
	return faces, halves, verts
end

--------------------------------------------------------------------------

function Loops.fromClean(result: any, data: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()

	local byFloor: { [BasePart]: {any} } = {}
	for _, e in ipairs(result.edges) do
		local b = byFloor[e.floor]
		if not b then b = {}; byFloor[e.floor] = b end
		b[#b + 1] = e
	end

	local regions: {Region} = {}
	local stats = {
		floors = 0, faces = 0, kept = 0, discardedEmpty = 0, discardedTiny = 0,
		openFloors = 0, continuationStuds = 0, boundaryStuds = 0,
		holes = 0, unbounded = 0,
		-- Validation. A region's area should be covered by its live cells. If it
		-- substantially exceeds them, the region is claiming ground nothing stands
		-- on — the signature of an obstacle whose ring failed to close, which the
		-- hole logic cannot see because an unclosed ring merges into the
		-- surrounding face and simply vanishes.
		areaTotal = 0, cellArea = 0, worstSlack = 0, worstFloor = nil,
	}

	for part, g in pairs(data.grids) do
		if not g.fallback and g.center and g.uExt and g.vExt then
			stats.floors += 1
			local center, u, v = g.center, g.u, g.v
			-- NB: no handedness term. The face walk is defined entirely in this
			-- floor's own (x, y) coordinates, so interior faces come out positive
			-- whether or not (u, v, n) is right-handed in world space. An earlier
			-- version carried a `hand` factor here to "fix" floors that produced no
			-- face; the real cause was collinear overlap in splitAll, and the
			-- factor was then selecting the unbounded face instead.
			local function proj(p: Vector3): V2
				local d = p - center
				return { x = d:Dot(u), y = d:Dot(v) }
			end
			local function unproj(p: V2): Vector3
				return center + u * p.x + v * p.y
			end

			-- the rim rectangle, which bounds everything this floor can own
			local ue, ve = g.uExt, g.vExt
			local segs: {Seg} = {
				{ a = { x = -ue, y = -ve }, b = { x = ue, y = -ve }, class = "continuation" },
				{ a = { x = ue, y = -ve }, b = { x = ue, y = ve }, class = "continuation" },
				{ a = { x = ue, y = ve }, b = { x = -ue, y = ve }, class = "continuation" },
				{ a = { x = -ue, y = ve }, b = { x = -ue, y = -ve }, class = "continuation" },
			}
			-- emitted boundary, clipped to the rectangle (closure can push an
			-- endpoint a fraction past the rim)
			for _, e in ipairs(byFloor[part] or {}) do
				local a2, b2 = proj(e.a), proj(e.b)
				a2.x = math.clamp(a2.x, -ue, ue); a2.y = math.clamp(a2.y, -ve, ve)
				b2.x = math.clamp(b2.x, -ue, ue); b2.y = math.clamp(b2.y, -ve, ve)
				local dx, dy = b2.x - a2.x, b2.y - a2.y
				if dx * dx + dy * dy > c.weldEps * c.weldEps then
					segs[#segs + 1] = { a = a2, b = b2, class = e.class, source = e.source }
				end
			end

			local faces, halves, verts = traceFaces(splitAll(segs, c), c)
			stats.faces += #faces

			-- The mask decides which faces are real. Assign every live cell to
			-- the face containing it; a face nothing stands in is a sliver
			-- between near-coincident lines.
			local cellsIn: { [number]: number } = {}
			for _, cell in ipairs(g.cells) do
				local p = proj(cell.pos)
				for fi, f in ipairs(faces) do
					if f.area > 0 and pointInPoly(f.pts, p.x, p.y) then
						cellsIn[fi] = (cellsIn[fi] or 0) + 1
						break
					end
				end
			end

			-- HOLES. An obstacle standing in the middle of a floor does not cut
			-- the region — you walk around it — so its boundary never reaches the
			-- rim and cannot separate faces. It appears instead as a disconnected
			-- component, which the walk emits as a MIRRORED PAIR of cycles: one
			-- positive (the obstacle's own interior, where nothing stands) and one
			-- negative (the same ring seen from the walkable side). The negative
			-- cycle is the hole.
			--
			-- Attach each negative cycle to the SMALLEST positive cycle that
			-- contains it, which is what makes nesting work: a courtyard inside a
			-- building attaches to the building's interior rather than jumping out
			-- to the floor. The container must be strictly LARGER in area, which
			-- excludes the cycle's own mirror without needing a tolerance — the
			-- mirror has exactly equal area by construction.
			local function ringPts(f: any): {V2}
				return f.pts
			end
			local holesFor: { [number]: {number} } = {}
			for ni, nf in ipairs(faces) do
				if nf.area < 0 and #nf.pts > 0 then
					local probe = nf.pts[1]
					local best, bestArea = nil, math.huge
					for pi, pf in ipairs(faces) do
						if pf.area > 0 and pf.area > -nf.area + c.minArea
							and pointInPoly(ringPts(pf), probe.x, probe.y) then
							if pf.area < bestArea then best, bestArea = pi, pf.area end
						end
					end
					if best then
						local b = holesFor[best]
						if not b then b = {}; holesFor[best] = b end
						b[#b + 1] = ni
						stats.holes += 1
					else
						-- contained by nothing: this is the unbounded face
						stats.unbounded += 1
					end
				end
			end

			local function ringOf(f: any): ({LoopEdge}, {Vector3})
				local ring: {LoopEdge} = {}
				local vs: {Vector3} = {}
				for _, hi in ipairs(f.ring) do
					local h = halves[hi]
					local a3, b3 = unproj(verts[h.from]), unproj(verts[h.to])
					ring[#ring + 1] = { class = h.class, source = h.source, a = a3, b = b3 }
					vs[#vs + 1] = a3
					local L = (b3 - a3).Magnitude
					if h.class == "continuation" then
						stats.continuationStuds += L
					else
						stats.boundaryStuds += L
					end
				end
				return ring, vs
			end

			for fi, f in ipairs(faces) do
				if f.area > 0 then
					local n = cellsIn[fi] or 0
					if n < c.minCellsPerFace then
						stats.discardedEmpty += 1
					elseif math.abs(f.area) < c.minArea then
						stats.discardedTiny += 1
					else
						local ring, vs = ringOf(f)
						local holes: {{LoopEdge}} = {}
						local holeArea = 0
						for _, ni in ipairs(holesFor[fi] or {}) do
							local hr = ringOf(faces[ni])
							holes[#holes + 1] = hr
							holeArea += -faces[ni].area
						end
						regions[#regions + 1] = {
							floor = part, edges = ring, verts = vs,
							area = f.area - holeArea, cells = n, holes = holes,
						}
						stats.kept += 1

						local step = g.step or 1
						local cellArea = n * step * step
						local slack = (f.area - holeArea) - cellArea
						stats.areaTotal += (f.area - holeArea)
						stats.cellArea += cellArea
						if slack > stats.worstSlack then
							stats.worstSlack = slack
							stats.worstFloor = part.Name
						end
					end
				end
			end
		end
	end

	stats.seconds = os.clock() - t0
	return { regions = regions, stats = stats, config = c }
end

function Loops.build(cfg: any?)
	local result, data = Clean.build(cfg)
	return Loops.fromClean(result, data, cfg), result, data
end

--------------------------------------------------------------------------

-- Region fill tinted by cell count, with the ring drawn in Clean's class
-- colours plus grey for continuation (where the surface carries on).
function Loops.visualize(res: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Loops")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Loops"; folder.Parent = dbg

	local colours = {
		wall = Color3.new(1, 0.15, 0.15),
		seam = Color3.new(0.2, 1, 0.35),
		dropoff = Color3.new(0.15, 0.9, 1),
		tier = Color3.new(0.95, 0.85, 0.1),
		continuation = Color3.new(0.6, 0.6, 0.65),
	}
	local UP = Vector3.new(0, 1, 0)
	for ri, r in ipairs(res.regions) do
		local sub = Instance.new("Folder")
		sub.Name = string.format("R%d_%s_c%d_h%d", ri, r.floor.Name, r.cells, #r.holes)
		sub.Parent = folder
		local function draw(e, thick, tag)
			local d = e.b - e.a
			local len = d.Magnitude
			if len > 1e-3 then
				local p = Instance.new("Part")
				p.Anchored = true; p.CanCollide = false; p.CanQuery = false; p.CanTouch = false
				p.Size = Vector3.new(len, thick, thick)
				p.Color = colours[e.class] or Color3.new(1, 1, 1)
				p.Material = Enum.Material.Neon
				p.Transparency = (e.class == "continuation") and 0.45 or 0
				p.CFrame = CFrame.fromMatrix((e.a + e.b) * 0.5 + UP * 0.1, d.Unit, UP)
				p.Name = tag .. e.class
				p.Parent = sub
			end
		end
		for _, e in ipairs(r.edges) do draw(e, 0.25, "") end
		-- inner rings drawn fatter and lifted, so an obstacle reads as a hole
		-- rather than as another outer boundary lying on top of it
		for _, h in ipairs(r.holes) do
			for _, e in ipairs(h) do draw(e, 0.4, "hole_") end
		end
	end
	return folder
end

return Loops
