--!strict
-- NVGN.Polys — oriented maximum-rectangle cover.
--
-- Open floor is covered by large rectangles GROWN ALONG THE LOCAL WALL
-- DIRECTIONS, not along world or floor axes. The scene's buildings sit at their
-- own yaws on the floors they stand on, so an axis-aligned cover shatters into
-- fringe against every one of them: a previous attempt produced 562 pieces on
-- the 200x200 floor alone, purely because its lattice disagreed with the walls.
--
-- ORIENTATIONS. Boundary directions are clustered modulo 90 degrees, weighted by
-- edge length, and the strongest few become candidate frames. The floor's own
-- axes are always a candidate. Each frame is processed in turn, so the dominant
-- walls get first claim on the floor and weaker orientations fill what is left.
--
-- PLACEMENT is exact, not rasterized. Within a frame the lattice is built from
-- the rotated coordinates of real vertices — including the corners of rectangles
-- already placed — so every rectangle edge lands on a coordinate the geometry
-- actually has. A 0.25-stud occupancy grid was considered and rejected: a
-- rectangle grown to the edge of a raster stops up to a quarter stud short of
-- the wall, which leaves a thin sliver along every wall and every seam between
-- rectangles, and rasterizing an oriented rectangle onto an axis-aligned grid
-- steps its own edges. Overlap does not need a raster to be robust — rectangles
-- placed in earlier frames are simply carried in as extra blocking segments, and
-- cells whose centre falls inside one are not free.
--
-- FATNESS IS A GROWTH CONSTRAINT, not a repair. Maximum-AREA rectangles happily
-- return 122x1; a rectangle is only accepted here if it meets minWidth and
-- maxAspect in the first place. Trying to fix bad shapes afterwards was measured
-- and does not work: repair fixed 2 of 135 violators.
--
-- RESIDUAL. What the rectangles do not cover is cut out exactly, by feeding
-- their edges into the region's planar subdivision as extra constraint lines and
-- re-tracing faces. A residual piece that is already convex is emitted WHOLE,
-- whatever its vertex count — a clean pentagon beats two quads. Only a concave
-- piece is split, and only at vertices that already exist, so no Steiner point
-- is ever invented to force a shape.
--
-- Holes are never bridged. Rectangles tile around them and the subdivision
-- treats them as boundary, so the partitioner only ever sees simply-connected
-- local scraps.

local Loops = require(script.Parent:WaitForChild("Loops"))

local Polys = {}

export type Poly = {
	floor: BasePart,
	verts: {Vector3},
	classes: {string},
	area: number,
	kind: string,        -- "rect" | "residual"
	angle: number?,      -- orientation a rectangle was grown along
}
export type Config = {
	weldEps: number?, convexEps: number?,
	minRectWidth: number?, maxRectAspect: number?, minRectArea: number?,
	angleBin: number?, maxOrients: number?, degenerateArea: number?,
	onEdgeEps: number?,
}

local DEFAULT = {
	weldEps = 0.02,
	convexEps = 1e-4,
	-- A rectangle must be genuinely fat to be worth placing. Anything thinner or
	-- longer is left to the residual, where its shape is dictated by the ground
	-- rather than chosen by us.
	minRectWidth = 2.0,
	maxRectAspect = 8,
	minRectArea = 6,
	-- Orientation clustering: bin width in degrees, and how many frames to try.
	angleBin = 3,
	maxOrients = 1,
	degenerateArea = 1e-3,
	onEdgeEps = 0.02,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

type V2 = { x: number, y: number }
type Seg = { a: V2, b: V2, class: string }

local function cross2(ax: number, ay: number, bx: number, by: number): number
	return ax * by - ay * bx
end

local function areaOf(pts: {V2}): number
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

local function isConvex(pts: {V2}, c: any): boolean
	if #pts < 3 then return false end
	local n = #pts
	for i = 1, n do
		local a, b, d = pts[((i - 2) % n) + 1], pts[i], pts[(i % n) + 1]
		if cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y) < -c.convexEps then
			return false
		end
	end
	return true
end

local function pointInTri(p: V2, a: V2, b: V2, cc: V2): boolean
	local eps = 1e-9
	local d1 = cross2(b.x - a.x, b.y - a.y, p.x - a.x, p.y - a.y)
	local d2 = cross2(cc.x - b.x, cc.y - b.y, p.x - b.x, p.y - b.y)
	local d3 = cross2(a.x - cc.x, a.y - cc.y, p.x - cc.x, p.y - cc.y)
	return (d1 > eps and d2 > eps and d3 > eps) or (d1 < -eps and d2 < -eps and d3 < -eps)
end

-- Ear clipping over EXISTING vertices only; it never introduces a new point.
local function earClip(pts: {V2}, c: any): {{V2}}
	local idx = {}
	for i = 1, #pts do idx[i] = i end
	if areaOf(pts) < 0 then
		local r = {}
		for i = #idx, 1, -1 do r[#r + 1] = idx[i] end
		idx = r
	end
	local out: {{V2}} = {}
	local guard = 0
	while #idx > 3 and guard < 10000 do
		guard += 1
		local bestK, bestScore = nil, -1
		for k = 1, #idx do
			local a = pts[idx[((k - 2) % #idx) + 1]]
			local b = pts[idx[k]]
			local d = pts[idx[(k % #idx) + 1]]
			if cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y) > c.convexEps then
				local ok = true
				for _, j in ipairs(idx) do
					local q = pts[j]
					if q ~= a and q ~= b and q ~= d and pointInTri(q, a, b, d) then ok = false; break end
				end
				if ok then
					local function ang(p1: V2, p2: V2, p3: V2): number
						local ux, uy = p1.x - p2.x, p1.y - p2.y
						local vx, vy = p3.x - p2.x, p3.y - p2.y
						local lu = math.sqrt(ux * ux + uy * uy)
						local lv = math.sqrt(vx * vx + vy * vy)
						if lu < 1e-12 or lv < 1e-12 then return 0 end
						return math.acos(math.clamp((ux * vx + uy * vy) / (lu * lv), -1, 1))
					end
					local s = math.min(ang(d, a, b), math.min(ang(a, b, d), ang(b, d, a)))
					if s > bestScore then bestScore, bestK = s, k end
				end
			end
		end
		if not bestK then break end
		local k = bestK :: number
		out[#out + 1] = { pts[idx[((k - 2) % #idx) + 1]], pts[idx[k]], pts[idx[(k % #idx) + 1]] }
		table.remove(idx, k)
	end
	if #idx == 3 then out[#out + 1] = { pts[idx[1]], pts[idx[2]], pts[idx[3]] } end
	return out
end

-- A point guaranteed to lie INSIDE a traced face.
--
-- The average of the vertices is not one: for a concave face it can sit outside
-- the face entirely, and it also drifts toward whichever side carries more
-- collinear split points, of which these rings have many. Using it to ask "is
-- this face in the region" both dropped real faces and kept covered ones, for a
-- net loss of 1893 studs^2. Step in from an edge midpoint along the inward
-- normal instead, which is exact for any simple ring.
local function interiorPoint(pts: {V2}): (number, number)
	local n = #pts
	local sign = (areaOf(pts) >= 0) and 1 or -1
	local bi, bl = 1, -1
	for i = 1, n do
		local a, b = pts[i], pts[(i % n) + 1]
		local dx, dy = b.x - a.x, b.y - a.y
		local l = dx * dx + dy * dy
		if l > bl then bl, bi = l, i end
	end
	local a, b = pts[bi], pts[(bi % n) + 1]
	local dx, dy = b.x - a.x, b.y - a.y
	local len = math.sqrt(dx * dx + dy * dy)
	if len < 1e-9 then return a.x, a.y end
	local step = math.min(1e-3, len * 0.01)
	return (a.x + b.x) * 0.5 - dy / len * step * sign,
		(a.y + b.y) * 0.5 + dx / len * step * sign
end

-- Clip a segment to a rectangle (Liang-Barsky); nil when it misses.
local function clipSeg(a: V2, b: V2, x0: number, y0: number, x1: number, y1: number): (V2?, V2?)
	local dx, dy = b.x - a.x, b.y - a.y
	local t0, t1 = 0, 1
	local ps = { -dx, dx, -dy, dy }
	local qs = { a.x - x0, x1 - a.x, a.y - y0, y1 - a.y }
	for i = 1, 4 do
		local pp, qq = ps[i], qs[i]
		if math.abs(pp) < 1e-12 then
			if qq < 0 then return nil, nil end
		else
			local t = qq / pp
			if pp < 0 then
				if t > t1 then return nil, nil end
				if t > t0 then t0 = t end
			else
				if t < t0 then return nil, nil end
				if t < t1 then t1 = t end
			end
		end
	end
	if t1 - t0 < 1e-9 then return nil, nil end
	return { x = a.x + dx * t0, y = a.y + dy * t0 }, { x = a.x + dx * t1, y = a.y + dy * t1 }
end

--------------------------------------------------------------------------
-- Dominant orientations
--------------------------------------------------------------------------

-- Angles are taken modulo 90 degrees: a rectangle grown along a wall is the same
-- rectangle whichever of its two axes that wall defines. Weighted by edge
-- length, so a long building face outvotes a scattering of short jogs.
local function dominantAngles(segs: {Seg}, c: any): {number}
	local bins: { [number]: number } = {}
	local half = math.pi / 2
	local binRad = math.rad(c.angleBin)
	for _, s in ipairs(segs) do
		local dx, dy = s.b.x - s.a.x, s.b.y - s.a.y
		local len = math.sqrt(dx * dx + dy * dy)
		if len > 1e-6 then
			local a = math.atan2(dy, dx) % half
			local k = math.floor(a / binRad + 0.5)
			bins[k] = (bins[k] or 0) + len
		end
	end
	local list = {}
	for k, wgt in pairs(bins) do list[#list + 1] = { k = k, w = wgt } end
	table.sort(list, function(x, y)
		if x.w ~= y.w then return x.w > y.w end
		return x.k < y.k -- deterministic: table order must never decide
	end)
	-- ONE frame per region unless asked for more. Mixing frames looked
	-- attractive but wrecks the fill: rectangles at different angles leave gaps
	-- that no single lattice can cover in large pieces, and the leftover sweep
	-- shattered into 6497 slivers. A region gets the direction its walls actually
	-- run in, and everything else is measured against that.
	if c.maxOrients <= 1 then
		if #list > 0 then return { (list[1].k * binRad) % half } end
		return { 0 }
	end
	local out = { 0 } -- the floor's own axes are always worth trying
	for i = 1, math.min(#list, c.maxOrients) do
		local a = (list[i].k * binRad) % half
		local dup = false
		for _, e in ipairs(out) do
			if math.abs(a - e) < binRad * 0.5 then dup = true; break end
		end
		if not dup then out[#out + 1] = a end
	end
	return out
end

--------------------------------------------------------------------------
-- Maximum-area rectangle over free cells, subject to the fatness limits.
--------------------------------------------------------------------------

local function bestRect(free: {{boolean}}, w: {number}, h: {number}, c: any): any
	local nx, ny = #w, #h
	local best = { area = 0 }
	local heights = {}
	local rows = {}
	for i = 1, nx do heights[i] = 0; rows[i] = 0 end
	for j = 1, ny do
		for i = 1, nx do
			if free[i][j] then heights[i] += h[j]; rows[i] += 1 else heights[i] = 0; rows[i] = 0 end
		end
		for i = 1, nx do
			if heights[i] > 0 then
				local minH, minRows = heights[i], rows[i]
				local wsum = 0
				for k = i, nx do
					if heights[k] <= 0 then break end
					if heights[k] < minH then minH = heights[k]; minRows = rows[k] end
					wsum += w[k]
					local a = minH * wsum
					local short = math.min(minH, wsum)
					local long = math.max(minH, wsum)
					if short >= c.minRectWidth and long <= short * c.maxRectAspect
						and a >= c.minRectArea and a > best.area then
						best = { area = a, i0 = i, i1 = k, jTop = j, height = minH, rows = minRows }
					end
				end
			end
		end
	end
	return best
end

--------------------------------------------------------------------------

function Polys.fromLoops(lres: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local polys: {Poly} = {}
	local stats = {
		regions = 0, rects = 0, residual = 0, polys = 0, orients = 0,
		areaIn = 0, areaOut = 0, rectArea = 0, residualArea = 0,
		nonConvex = 0, split = 0, cuts = 0, worstAreaErr = 0, worstFloor = "", badRegions = 0,
		overlaps = 0, maxVerts = 0,
	}

	for _, r in ipairs(lres.regions) do
		stats.regions += 1
		local fr = r.frame
		if not fr then continue end
		local o, e1, e2 = fr.o, fr.u, fr.v
		local function proj(p: Vector3): V2
			local d = p - o
			return { x = d:Dot(e1), y = d:Dot(e2) }
		end
		local function unproj(p: V2): Vector3
			return o + e1 * p.x + e2 * p.y
		end

		local outer: {V2} = {}
		local bsegs: {Seg} = {}
		for i, e in ipairs(r.edges) do
			outer[i] = proj(e.a)
		end
		for i = 1, #outer do
			bsegs[#bsegs + 1] = {
				a = outer[i], b = outer[(i % #outer) + 1], class = r.edges[i].class,
			}
		end
		local holes: {{V2}} = {}
		for _, hr in ipairs(r.holes) do
			local hp: {V2} = {}
			for i, e in ipairs(hr) do hp[i] = proj(e.a) end
			holes[#holes + 1] = hp
			for i = 1, #hp do
				bsegs[#bsegs + 1] = { a = hp[i], b = hp[(i % #hp) + 1], class = hr[i].class }
			end
		end

		local mark = #polys
		local regionArea = math.abs(areaOf(outer))
		for _, hp in ipairs(holes) do regionArea -= math.abs(areaOf(hp)) end
		stats.areaIn += regionArea

		local function inRegion(x: number, y: number): boolean
			if not pointInPoly(outer, x, y) then return false end
			for _, hp in ipairs(holes) do
				if pointInPoly(hp, x, y) then return false end
			end
			return true
		end

		local function classFor(a: V2, b: V2): string
			local mx, my = (a.x + b.x) * 0.5, (a.y + b.y) * 0.5
			for _, s in ipairs(bsegs) do
				local dx, dy = s.b.x - s.a.x, s.b.y - s.a.y
				local L2 = dx * dx + dy * dy
				if L2 > 1e-12 then
					local t = ((mx - s.a.x) * dx + (my - s.a.y) * dy) / L2
					if t >= -1e-6 and t <= 1 + 1e-6 then
						local px, py = s.a.x + dx * t, s.a.y + dy * t
						local ex, ey = mx - px, my - py
						if ex * ex + ey * ey <= c.onEdgeEps * c.onEdgeEps then return s.class end
					end
				end
			end
			return "internal"
		end

		local function emit(pts: {V2}, kind: string, angle: number?)
			local a = math.abs(areaOf(pts))
			if a < c.degenerateArea then return end
			local vs, cs = {}, {}
			for i = 1, #pts do
				vs[#vs + 1] = unproj(pts[i])
				cs[#cs + 1] = classFor(pts[i], pts[(i % #pts) + 1])
			end
			if not isConvex(pts, c) then stats.nonConvex += 1 end
			stats.maxVerts = math.max(stats.maxVerts, #pts)
			stats.areaOut += a
			stats.polys += 1
			if kind == "rect" then stats.rects += 1; stats.rectArea += a
			else stats.residual += 1; stats.residualArea += a end
			polys[#polys + 1] = {
				floor = r.floor, verts = vs, classes = cs, area = a, kind = kind, angle = angle,
			}
		end

		----------------------------------------------------------------
		-- place oriented rectangles, strongest wall direction first
		----------------------------------------------------------------
		local placed: { { pts: {V2}, angle: number } } = {}
		local placedSegs: {Seg} = {}

		local angles = dominantAngles(bsegs, c)
		stats.orients += #angles
		for _, ang in ipairs(angles) do
			local ca, sa = math.cos(ang), math.sin(ang)
			local function rot(p: V2): V2
				return { x = p.x * ca + p.y * sa, y = -p.x * sa + p.y * ca }
			end
			local function unrot(p: V2): V2
				return { x = p.x * ca - p.y * sa, y = p.x * sa + p.y * ca }
			end

			-- lattice from every real coordinate this frame can see, including the
			-- corners of rectangles already placed in earlier frames
			local xsAll, ysAll = {}, {}
			local function feed(p: V2)
				local q = rot(p)
				xsAll[#xsAll + 1] = q.x
				ysAll[#ysAll + 1] = q.y
			end
			for _, s in ipairs(bsegs) do feed(s.a) end
			for _, pr in ipairs(placed) do
				for _, p in ipairs(pr.pts) do feed(p) end
			end
			local function uniq(t: {number}): {number}
				table.sort(t)
				local out = {}
				for _, v in ipairs(t) do
					if #out == 0 or v - out[#out] > c.weldEps then out[#out + 1] = v end
				end
				return out
			end
			local xs, ys = uniq(xsAll), uniq(ysAll)
			if #xs >= 2 and #ys >= 2 then
				local nx, ny = #xs - 1, #ys - 1
				local w, h = {}, {}
				for i = 1, nx do w[i] = xs[i + 1] - xs[i] end
				for j = 1, ny do h[j] = ys[j + 1] - ys[j] end

				-- blockers in this frame: region boundary plus everything placed
				local blockers: {Seg} = {}
				for _, s in ipairs(bsegs) do
					blockers[#blockers + 1] = { a = rot(s.a), b = rot(s.b), class = s.class }
				end
				for _, pr in ipairs(placedSegs) do
					blockers[#blockers + 1] = { a = rot(pr.a), b = rot(pr.b), class = "internal" }
				end

				local free = {}
				for i = 1, nx do
					free[i] = {}
					for j = 1, ny do
						local x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
						local cx, cy = (x0 + x1) * 0.5, (y0 + y1) * 0.5
						local world = unrot({ x = cx, y = cy })
						local ok = inRegion(world.x, world.y)
						if ok then
							for _, pr in ipairs(placed) do
								if pointInPoly(pr.pts, world.x, world.y) then ok = false; break end
							end
						end
						if ok then
							for _, s in ipairs(blockers) do
								local q1, q2 = clipSeg(s.a, s.b, x0, y0, x1, y1)
								if q1 and q2 then
									local mx, my = (q1.x + q2.x) * 0.5, (q1.y + q2.y) * 0.5
									if mx > x0 + 1e-6 and mx < x1 - 1e-6
										and my > y0 + 1e-6 and my < y1 - 1e-6 then
										ok = false
										break
									end
								end
							end
						end
						free[i][j] = ok
					end
				end

				while true do
					local b = bestRect(free, w, h, c)
					if b.area <= 0 then break end
					local jTop = b.jTop :: number
					local acc, jBot = 0, jTop
					for j = jTop, 1, -1 do
						acc += h[j]
						if acc >= (b.height :: number) - 1e-9 then jBot = j; break end
					end
					local x0, x1 = xs[b.i0 :: number], xs[(b.i1 :: number) + 1]
					local y0, y1 = ys[jBot], ys[jTop + 1]
					local corners = {
						unrot({ x = x0, y = y0 }), unrot({ x = x1, y = y0 }),
						unrot({ x = x1, y = y1 }), unrot({ x = x0, y = y1 }),
					}
					placed[#placed + 1] = { pts = corners, angle = ang }
					for k = 1, 4 do
						placedSegs[#placedSegs + 1] = {
							a = corners[k], b = corners[(k % 4) + 1], class = "internal",
						}
					end
					emit(corners, "rect", ang)
					for i = b.i0 :: number, b.i1 :: number do
						for j = jBot, jTop do free[i][j] = false end
					end
				end
			end
		end

		----------------------------------------------------------------
		-- RESIDUAL. Not by arranging all the rectangles together: 163 rectangle
		-- rings in one planar graph fragments the leftovers into hundreds of
		-- faces, and a rectangle floating clear of everything makes its
		-- surroundings multiply connected, so the walk emits the surroundings and
		-- the rectangle separately and counts that ground twice (+1028 studs^2,
		-- caught by the area check).
		--
		-- Instead, sweep once more in the strongest frame. Cells still free are
		-- filled with rectangles under RELAXED limits — open leftovers stay large
		-- quads, and the smallest case is a single cell — while cells a boundary
		-- edge crosses are subdivided on their own, which bounds every fringe
		-- piece to one cell. Nothing is ever multiply connected, so no bridging
		-- and no cuts are needed anywhere.
		----------------------------------------------------------------
		local fillAng = angles[#angles > 1 and 2 or 1]
		do
			local ca, sa = math.cos(fillAng), math.sin(fillAng)
			local function rot(p: V2): V2
				return { x = p.x * ca + p.y * sa, y = -p.x * sa + p.y * ca }
			end
			local function unrot(p: V2): V2
				return { x = p.x * ca - p.y * sa, y = p.x * sa + p.y * ca }
			end

			local xsAll, ysAll = {}, {}
			local function feed(p: V2)
				local q = rot(p)
				xsAll[#xsAll + 1] = q.x
				ysAll[#ysAll + 1] = q.y
			end
			for _, sg in ipairs(bsegs) do feed(sg.a) end
			for _, pr in ipairs(placed) do
				for _, q2 in ipairs(pr.pts) do feed(q2) end
			end
			local function uniq(t: {number}): {number}
				table.sort(t)
				local o2 = {}
				for _, v in ipairs(t) do
					if #o2 == 0 or v - o2[#o2] > c.weldEps then o2[#o2 + 1] = v end
				end
				return o2
			end
			local xs, ys = uniq(xsAll), uniq(ysAll)
			if #xs >= 2 and #ys >= 2 then
				local nx, ny = #xs - 1, #ys - 1
				local w, h = {}, {}
				for i = 1, nx do w[i] = xs[i + 1] - xs[i] end
				for j = 1, ny do h[j] = ys[j + 1] - ys[j] end

				local blockers: {Seg} = {}
				for _, sg in ipairs(bsegs) do
					blockers[#blockers + 1] = { a = rot(sg.a), b = rot(sg.b), class = sg.class }
				end
				for _, sg in ipairs(placedSegs) do
					blockers[#blockers + 1] = { a = rot(sg.a), b = rot(sg.b), class = "internal" }
				end

				local free, partial = {}, {}
				for i = 1, nx do
					free[i] = {}
					partial[i] = {}
					for j = 1, ny do
						local x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
						local world = unrot({ x = (x0 + x1) * 0.5, y = (y0 + y1) * 0.5 })
						local ins = inRegion(world.x, world.y)
						if ins then
							for _, pr in ipairs(placed) do
								if pointInPoly(pr.pts, world.x, world.y) then ins = false; break end
							end
						end
						local crossed = false
						for _, sg in ipairs(blockers) do
							local q1, q2 = clipSeg(sg.a, sg.b, x0, y0, x1, y1)
							if q1 and q2 then
								local mx, my = (q1.x + q2.x) * 0.5, (q1.y + q2.y) * 0.5
								if mx > x0 + 1e-6 and mx < x1 - 1e-6
									and my > y0 + 1e-6 and my < y1 - 1e-6 then
									crossed = true
									break
								end
							end
						end
						free[i][j] = ins and not crossed
						partial[i][j] = crossed
					end
				end

				-- relaxed fill: every remaining free cell ends up in some rectangle
				local relaxed = {
					convexEps = c.convexEps, minRectWidth = 0, maxRectAspect = math.huge,
					minRectArea = 0,
				}
				while true do
					local b = bestRect(free, w, h, relaxed)
					if b.area <= c.degenerateArea then break end
					local jTop = b.jTop :: number
					local acc, jBot = 0, jTop
					for j = jTop, 1, -1 do
						acc += h[j]
						if acc >= (b.height :: number) - 1e-9 then jBot = j; break end
					end
					local x0, x1 = xs[b.i0 :: number], xs[(b.i1 :: number) + 1]
					local y0, y1 = ys[jBot], ys[jTop + 1]
					emit({
						unrot({ x = x0, y = y0 }), unrot({ x = x1, y = y0 }),
						unrot({ x = x1, y = y1 }), unrot({ x = x0, y = y1 }),
					}, "residual")
					for i = b.i0 :: number, b.i1 :: number do
						for j = jBot, jTop do free[i][j] = false end
					end
				end

				-- fringe: one cell at a time, so a piece can never span the map
				for i = 1, nx do
					for j = 1, ny do
						if partial[i][j] then
							local x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
							local cell: {Seg} = {
								{ a = { x = x0, y = y0 }, b = { x = x1, y = y0 }, class = "internal" },
								{ a = { x = x1, y = y0 }, b = { x = x1, y = y1 }, class = "internal" },
								{ a = { x = x1, y = y1 }, b = { x = x0, y = y1 }, class = "internal" },
								{ a = { x = x0, y = y1 }, b = { x = x0, y = y0 }, class = "internal" },
							}
							for _, sg in ipairs(blockers) do
								local q1, q2 = clipSeg(sg.a, sg.b, x0, y0, x1, y1)
								if q1 and q2 then
									cell[#cell + 1] = { a = q1, b = q2, class = sg.class }
								end
							end
							local faces, halves, verts = Loops._traceFaces(Loops._splitAll(cell, c), c)
							for _, f in ipairs(faces) do
								if f.area > c.degenerateArea then
									local px, py = interiorPoint(f.pts)
									local world = unrot({ x = px, y = py })
									local keep = px > x0 - 1e-9 and px < x1 + 1e-9
										and py > y0 - 1e-9 and py < y1 + 1e-9
										and inRegion(world.x, world.y)
									if keep then
										for _, pr in ipairs(placed) do
											if pointInPoly(pr.pts, world.x, world.y) then keep = false; break end
										end
									end
									if keep then
										local pts = {}
										for _, hi in ipairs(f.ring) do
											pts[#pts + 1] = unrot(verts[halves[hi].from])
										end
										-- convex pieces go out WHOLE, whatever their vertex
										-- count: a clean pentagon beats two quads
										if isConvex(pts, c) then
											emit(pts, "residual")
										else
											stats.split += 1
											for _, tri in ipairs(earClip(pts, c)) do emit(tri, "residual") end
										end
									end
								end
							end
						end
					end
				end
			end
		end

		-- PER-REGION AREA CONSERVATION, in studs^2 and not in cells.
		--
		-- Mandatory from the first commit, because every geometry bug this
		-- pipeline has produced was silent: rings that self-intersected, faces
		-- escaping their cell, ears that were never found. Each showed up here as
		-- a number and nowhere else. Asserting in rasterized cells instead would
		-- conserve the quantized area and hide exactly the errors that matter.
		local got = 0
		for k = mark + 1, #polys do got += polys[k].area end
		local err = math.abs(got - regionArea)
		if err > math.max(0.01, regionArea * 1e-6) then
			stats.badRegions += 1
			if err > stats.worstAreaErr then
				stats.worstAreaErr = err
				stats.worstFloor = r.floor.Name
			end
		end
	end

	stats.seconds = os.clock() - t0
	return { polys = polys, stats = stats, config = c }
end

function Polys.build(cfg: any?)
	local lres, cres, data = Loops.build(cfg)
	return Polys.fromLoops(lres, cfg), lres, cres, data
end

--------------------------------------------------------------------------

function Polys.visualize(res: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Polys")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Polys"; folder.Parent = dbg

	local UP = Vector3.new(0, 1, 0)
	local colours = {
		wall = Color3.new(1, 0.15, 0.15),
		seam = Color3.new(0.2, 1, 0.35),
		dropoff = Color3.new(0.15, 0.9, 1),
		tier = Color3.new(0.95, 0.85, 0.1),
		continuation = Color3.new(0.6, 0.6, 0.65),
		internal = Color3.new(0.45, 0.35, 0.8),
	}
	for pi, p in ipairs(res.polys) do
		local sub = Instance.new("Folder")
		sub.Name = string.format("P%d_%s_%s_a%.0f_v%d", pi, p.floor.Name, p.kind, p.area, #p.verts)
		sub.Parent = folder
		for i = 1, #p.verts do
			local a, b = p.verts[i], p.verts[(i % #p.verts) + 1]
			local d = b - a
			if d.Magnitude > 1e-3 then
				local bar = Instance.new("Part")
				bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
				bar.Size = Vector3.new(d.Magnitude, 0.2, 0.2)
				bar.Color = colours[p.classes[i]] or Color3.new(1, 1, 1)
				bar.Material = Enum.Material.Neon
				bar.Transparency = (p.classes[i] == "internal") and 0.45 or 0
				bar.CFrame = CFrame.fromMatrix((a + b) * 0.5 + UP * 0.15, d.Unit, UP)
				bar.Name = p.classes[i]
				bar.Parent = sub
			end
		end
	end
	return folder
end

return Polys
