--!strict
-- NVGN.Polys — regions into navmesh polygons.
--
-- SHAPE IS THE REQUIREMENT. Open ground must come out as a few huge rectangles;
-- everything else must be ordinary small polygons. Long stretched pieces are
-- unacceptable anywhere, whether or not they are convex and whether or not they
-- tile correctly.
--
-- Two general-purpose convex decompositions were built and measured first, and
-- both failed that requirement for the same underlying reason:
--
--   * ear clipping + fatness-ordered merging — the merge is blocked by
--     convexity, so strips around an obstacle stay strips (48 polygons on the
--     200x200 floor, 4166 studs of cuts longer than 20).
--   * trapezoidal sweep — worse on every axis (66 polygons, 6722 studs of long
--     cuts, 39 slivers), because a vertical cut spans the whole slab: "short in
--     x" is not short.
--
-- Neither is wrong as a decomposition. They are wrong as an ANSWER, because
-- both treat all ground alike, and the interesting property here is that most
-- ground is empty. So build the shape that is wanted directly.
--
-- PHASE 1 — MAXIMAL RECTANGLES. Take every vertex x and every vertex y in the
-- floor's own frame as lattice lines. That lattice is EXACT — its coordinates
-- are the geometry's own, so nothing is quantized and no staircase can appear —
-- and it is sparse, a few dozen lines rather than one per stud. Cells wholly
-- inside the region are then consumed repeatedly by the largest-area rectangle
-- available. Open floor collapses into a handful of huge rectangles, and near
-- obstacles the rectangles shrink by themselves, because that is where the
-- lattice is dense.
--
-- PHASE 2 — THE FRINGE. Only cells that a boundary edge actually crosses need
-- anything cleverer, and those are small by construction, bounded by adjacent
-- vertex coordinates. Their pieces are therefore small: the diagonal fringe of a
-- rotated part, never a 200-stud needle. Each such cell is subdivided locally
-- and the pieces inside the region are kept, ear-clipped if not already convex.
--
-- Class carrying: every polygon edge remembers whether it came from a wall,
-- seam, dropoff, tier or continuation, or whether it is an internal cut. Portal
-- derivation later reads exactly that.

local Loops = require(script.Parent:WaitForChild("Loops"))

local Polys = {}

export type Poly = {
	floor: BasePart,
	verts: {Vector3},
	classes: {string},    -- class of edge i -> i+1; "internal" = a cut
	area: number,
	fatness: number,
	rect: boolean,        -- came out of the rectangle phase
}
export type Config = {
	weldEps: number?, convexEps: number?, minFatness: number?,
	degenerateArea: number?, onEdgeEps: number?,
	minWidth: number?, maxAspect: number?,
}

local DEFAULT = {
	-- Must match what Loops built its rings with: the fringe phase borrows its
	-- subdivision, and a different weld would close faces differently.
	weldEps = 0.02,
	convexEps = 1e-4,
	-- HARD SHAPE LIMITS, enforced by the repair pass, not merely preferred.
	--
	-- A polygon's width is taken as area / longest edge — the average thickness
	-- of the strip it occupies — and its aspect as longest edge / that width.
	-- Anything thinner or longer than these is absorbed into a neighbour if any
	-- convex union will take it. Some ground is genuinely a 0.5-stud ledge, so
	-- violations that survive are reported rather than deleted: dropping them
	-- would punch holes in the mesh.
	minWidth = 1.5,
	maxAspect = 5,
	-- Reporting threshold only.
	minFatness = 0.15,
	degenerateArea = 1e-3,
	-- How close a cut must lie to a boundary edge to inherit its class.
	onEdgeEps = 0.02,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

type V2 = { x: number, y: number }
type Ring = { pts: {V2}, cls: {string} }

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

local function perimOf(pts: {V2}): number
	local p = 0
	for i = 1, #pts do
		local j = (i % #pts) + 1
		local dx, dy = pts[j].x - pts[i].x, pts[j].y - pts[i].y
		p += math.sqrt(dx * dx + dy * dy)
	end
	return p
end

-- Width = area / longest edge (mean thickness of the strip), aspect = longest
-- edge / width. These are what the limits are stated in, because they say
-- directly what is wrong with a bad polygon: too thin to stand in, or too long
-- for its thickness.
local function shapeOf(pts: {V2}): (number, number)
	local a = math.abs(areaOf(pts))
	local longest = 0
	for i = 1, #pts do
		local j = (i % #pts) + 1
		local dx, dy = pts[j].x - pts[i].x, pts[j].y - pts[i].y
		longest = math.max(longest, math.sqrt(dx * dx + dy * dy))
	end
	if longest <= 1e-9 then return 0, math.huge end
	local width = a / longest
	if width <= 1e-9 then return 0, math.huge end
	return width, longest / width
end

local function fatnessOf(pts: {V2}): number
	local p = perimOf(pts)
	if p <= 1e-9 then return 0 end
	return 4 * math.pi * math.abs(areaOf(pts)) / (p * p)
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

-- Clip a segment to a rectangle (Liang-Barsky). Returns nil when it misses.
-- Used for both "does this edge cross the cell" and "what does the cell see",
-- so the two can never disagree.
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

local function isConvex(pts: {V2}, c: any): boolean
	local n = #pts
	if n < 3 then return false end
	for i = 1, n do
		local a = pts[((i - 2) % n) + 1]
		local b = pts[i]
		local d = pts[(i % n) + 1]
		if cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y) < -c.convexEps then
			return false
		end
	end
	return true
end

-- Strictly inside: a point ON the boundary must not block an ear. Rings carry
-- vertices lying exactly on straight runs, because rim lines are split wherever
-- a boundary edge meets them, and an inclusive test then finds no ear at all.
local function pointInTri(p: V2, a: V2, b: V2, cc: V2): boolean
	local eps = 1e-9
	local d1 = cross2(b.x - a.x, b.y - a.y, p.x - a.x, p.y - a.y)
	local d2 = cross2(cc.x - b.x, cc.y - b.y, p.x - b.x, p.y - b.y)
	local d3 = cross2(a.x - cc.x, a.y - cc.y, p.x - cc.x, p.y - cc.y)
	return (d1 > eps and d2 > eps and d3 > eps) or (d1 < -eps and d2 < -eps and d3 < -eps)
end

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
					-- Score by the ear's SMALLEST ANGLE, not by side length. A
					-- long-sided triangle can still be a needle; the angle is what
					-- actually predicts it, and needles are what the limits below
					-- then have to clean up.
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
		out[#out + 1] = {
			pts[idx[((k - 2) % #idx) + 1]], pts[idx[k]], pts[idx[(k % #idx) + 1]],
		}
		table.remove(idx, k)
	end
	if #idx == 3 then
		out[#out + 1] = { pts[idx[1]], pts[idx[2]], pts[idx[3]] }
	end
	return out
end

--------------------------------------------------------------------------
-- Largest-area rectangle over a boolean cell grid.
--
-- The usual maximal-rectangle histogram scan, but weighted by real coordinates
-- rather than by cell counts: this lattice is irregular, so twelve narrow cells
-- can be worth less than one wide one.
--------------------------------------------------------------------------

local function largestRect(free: {{boolean}}, w: {number}, h: {number}): any
	local nx, ny = #w, #h
	local best = { area = 0 }
	local heights = {}
	for i = 1, nx do heights[i] = 0 end
	for j = 1, ny do
		for i = 1, nx do
			if free[i][j] then heights[i] += h[j] else heights[i] = 0 end
		end
		for i = 1, nx do
			if heights[i] > 0 then
				local minH = heights[i]
				local wsum = 0
				for k = i, nx do
					if heights[k] <= 0 then break end
					minH = math.min(minH, heights[k])
					wsum += w[k]
					local a = minH * wsum
					if a > best.area then
						best = { area = a, i0 = i, i1 = k, jTop = j, height = minH }
					end
				end
			end
		end
	end
	return best
end

-- Merge two adjacent cycles across EVERY edge they share.
--
-- Drop the shared edges and re-chain the leftovers; exactly one closed cycle
-- means the merge is valid, anything else would pinch or disconnect the union.
-- An edge counts as shared only when THIS cycle has (u,v) and the OTHER has
-- (v,u) — testing both directions against one set marks a polygon's own edges
-- as shared and discards everything.
local function mergeCycles(p: {number}, q: {number}, edgeClass): {number}?
	local pEdge: { [string]: boolean } = {}
	for i = 1, #p do pEdge[p[i] .. ">" .. p[(i % #p) + 1]] = true end
	local drop: { [string]: boolean } = {}
	local nShared = 0
	for i = 1, #q do
		local u, v = q[i], q[(i % #q) + 1]
		if pEdge[v .. ">" .. u] then
			-- never dissolve a real boundary
			if edgeClass(u, v) ~= "internal" then return nil end
			drop[u .. ">" .. v] = true
			drop[v .. ">" .. u] = true
			nShared += 1
		end
	end
	if nShared == 0 then return nil end

	local nextOf: { [number]: number } = {}
	local count = 0
	local function keep(cy: {number}): boolean
		for i = 1, #cy do
			local u, v = cy[i], cy[(i % #cy) + 1]
			if not drop[u .. ">" .. v] then
				if nextOf[u] then return false end -- vertex leaving twice: pinched
				nextOf[u] = v
				count += 1
			end
		end
		return true
	end
	if not keep(p) or not keep(q) or count < 3 then return nil end

	local startV: number? = nil
	for u in pairs(nextOf) do startV = u; break end
	local out, cur, guard = {}, startV :: number, 0
	repeat
		out[#out + 1] = cur
		local nx = nextOf[cur]
		if not nx then return nil end
		cur = nx
		guard += 1
	until cur == startV or guard > count + 2
	if #out ~= count then return nil end
	return out
end

--------------------------------------------------------------------------

function Polys.fromLoops(lres: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local polys: {Poly} = {}
	local stats = {
		regions = 0, polys = 0, rects = 0, fringe = 0, slivers = 0,
		areaIn = 0, areaOut = 0, nonConvex = 0, maxVerts = 0, degenerate = 0, merges = 0, repaired = 0, unfixable = 0,
		tooThin = 0, tooLong = 0,
		rectArea = 0, fringeArea = 0, worstFatness = 1,
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

		local outer: Ring = { pts = {}, cls = {} }
		for _, e in ipairs(r.edges) do
			outer.pts[#outer.pts + 1] = proj(e.a)
			outer.cls[#outer.cls + 1] = e.class
		end
		local holes: {Ring} = {}
		for _, h in ipairs(r.holes) do
			local hr: Ring = { pts = {}, cls = {} }
			for _, e in ipairs(h) do
				hr.pts[#hr.pts + 1] = proj(e.a)
				hr.cls[#hr.cls + 1] = e.class
			end
			holes[#holes + 1] = hr
		end
		local regionArea = math.abs(areaOf(outer.pts))
		for _, h in ipairs(holes) do regionArea -= math.abs(areaOf(h.pts)) end
		stats.areaIn += regionArea

		type Seg = { a: V2, b: V2, class: string }
		local bsegs: {Seg} = {}
		local function addRing(rr: Ring)
			for i = 1, #rr.pts do
				local j = (i % #rr.pts) + 1
				bsegs[#bsegs + 1] = { a = rr.pts[i], b = rr.pts[j], class = rr.cls[i] }
			end
		end
		addRing(outer)
		for _, h in ipairs(holes) do addRing(h) end

		local function inRegion(x: number, y: number): boolean
			if not pointInPoly(outer.pts, x, y) then return false end
			for _, h in ipairs(holes) do
				if pointInPoly(h.pts, x, y) then return false end
			end
			return true
		end

		-- class of a cut that lies along a boundary edge, else "internal"
		local function classFor(a: V2, b: V2): string
			local mx, my = (a.x + b.x) * 0.5, (a.y + b.y) * 0.5
			for _, s in ipairs(bsegs) do
				local dx, dy = s.b.x - s.a.x, s.b.y - s.a.y
				local L2 = dx * dx + dy * dy
				if L2 > 1e-12 then
					local t = ((mx - s.a.x) * dx + (my - s.a.y) * dy) / L2
					if t >= -1e-6 and t <= 1 + 1e-6 then
						local px, py = s.a.x + dx * t, s.a.y + dy * t
						local ddx, ddy = mx - px, my - py
						if ddx * ddx + ddy * ddy <= c.onEdgeEps * c.onEdgeEps then
							return s.class
						end
					end
				end
			end
			return "internal"
		end

		-- Pieces are collected, merged, and only then emitted.
		local pieces: {{V2}} = {}
		local pieceRect: {boolean} = {}
		local function collect(pts: {V2}, isRect: boolean)
			pieces[#pieces + 1] = pts
			pieceRect[#pieces] = isRect
		end

		local function emit(pts: {V2}, isRect: boolean)
			local a = math.abs(areaOf(pts))
			if a < c.degenerateArea then stats.degenerate += 1; return end
			local vs, cs = {}, {}
			for i = 1, #pts do
				vs[#vs + 1] = unproj(pts[i])
				cs[#cs + 1] = classFor(pts[i], pts[(i % #pts) + 1])
			end
			local f = fatnessOf(pts)
			local wdt, asp = shapeOf(pts)
			if wdt < c.minWidth then stats.tooThin += 1 end
			if asp > c.maxAspect then stats.tooLong += 1 end
			if not isConvex(pts, c) then stats.nonConvex += 1 end
			if f < c.minFatness then stats.slivers += 1 end
			stats.worstFatness = math.min(stats.worstFatness, f)
			stats.areaOut += a
			stats.maxVerts = math.max(stats.maxVerts, #vs)
			stats.polys += 1
			if isRect then
				stats.rects += 1; stats.rectArea += a
			else
				stats.fringe += 1; stats.fringeArea += a
			end
			polys[#polys + 1] = {
				floor = r.floor, verts = vs, classes = cs,
				area = a, fatness = f, rect = isRect,
			}
		end

		----------------------------------------------------------------
		-- lattice from the region's own vertex coordinates
		----------------------------------------------------------------
		local function axis(getter): {number}
			local vals = {}
			for _, s in ipairs(bsegs) do vals[#vals + 1] = getter(s.a) end
			table.sort(vals)
			local out = {}
			for _, v in ipairs(vals) do
				if #out == 0 or v - out[#out] > c.weldEps then out[#out + 1] = v end
			end
			return out
		end
		local xs = axis(function(p) return p.x end)
		local ys = axis(function(p) return p.y end)
		if #xs < 2 or #ys < 2 then continue end

		local nx, ny = #xs - 1, #ys - 1
		local w, h = {}, {}
		for i = 1, nx do w[i] = xs[i + 1] - xs[i] end
		for j = 1, ny do h[j] = ys[j + 1] - ys[j] end

		-- A cell is FREE when its interior lies wholly inside the region: its
		-- centre is inside and no boundary edge passes through it. An edge can
		-- only cross a cell interior when it is not axis-aligned in this frame,
		-- since every vertex coordinate is already a lattice line.
		local free, partial = {}, {}
		for i = 1, nx do
			free[i] = {}
			partial[i] = {}
			local x0, x1 = xs[i], xs[i + 1]
			for j = 1, ny do
				local y0, y1 = ys[j], ys[j + 1]
				-- Bounding-box overlap is not good enough: a single diagonal edge
				-- overlaps the boxes of every cell in its span, which marked
				-- hundreds of untouched cells as fringe. Clip the edge to the cell
				-- and require the surviving piece to run through the INTERIOR,
				-- not along a cell side.
				local crossed = false
				for _, s in ipairs(bsegs) do
					local ca, cb = clipSeg(s.a, s.b, x0, y0, x1, y1)
					if ca and cb then
						local mx, my = (ca.x + cb.x) * 0.5, (ca.y + cb.y) * 0.5
						if mx > x0 + 1e-6 and mx < x1 - 1e-6 and my > y0 + 1e-6 and my < y1 - 1e-6 then
							crossed = true
							break
						end
					end
				end
				local ins = inRegion((x0 + x1) * 0.5, (y0 + y1) * 0.5)
				free[i][j] = ins and not crossed
				partial[i][j] = crossed
			end
		end

		----------------------------------------------------------------
		-- PHASE 1 — consume free cells with the largest rectangle available
		----------------------------------------------------------------
		while true do
			local b = largestRect(free, w, h)
			if b.area <= c.degenerateArea then break end
			local jTop = b.jTop :: number
			local acc, jBot = 0, jTop
			for j = jTop, 1, -1 do
				acc += h[j]
				if acc >= (b.height :: number) - 1e-9 then jBot = j; break end
			end
			local x0, x1 = xs[b.i0 :: number], xs[(b.i1 :: number) + 1]
			local y0, y1 = ys[jBot], ys[jTop + 1]
			collect({
				{ x = x0, y = y0 }, { x = x1, y = y0 },
				{ x = x1, y = y1 }, { x = x0, y = y1 },
			}, true)
			for i = b.i0 :: number, b.i1 :: number do
				for j = jBot, jTop do free[i][j] = false end
			end
		end

		----------------------------------------------------------------
		-- PHASE 2 — the fringe: cells a boundary edge crosses
		----------------------------------------------------------------
		for i = 1, nx do
			for j = 1, ny do
				if partial[i][j] then
					local x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
					local segs = {
						{ a = { x = x0, y = y0 }, b = { x = x1, y = y0 }, class = "internal" },
						{ a = { x = x1, y = y0 }, b = { x = x1, y = y1 }, class = "internal" },
						{ a = { x = x1, y = y1 }, b = { x = x0, y = y1 }, class = "internal" },
						{ a = { x = x0, y = y1 }, b = { x = x0, y = y0 }, class = "internal" },
					}
					-- Only what this cell actually contains. Feeding the whole
					-- region's edges in produced faces far larger than the cell,
					-- which the centroid filter then let through, counting the same
					-- ground several times (area came out 1350 studs^2 OVER).
					for _, s in ipairs(bsegs) do
						local ca, cb = clipSeg(s.a, s.b, x0, y0, x1, y1)
						if ca and cb then
							segs[#segs + 1] = { a = ca, b = cb, class = s.class }
						end
					end
					local faces, halves, verts = Loops._traceFaces(Loops._splitAll(segs, c), c)
					for _, f in ipairs(faces) do
						if f.area > c.degenerateArea then
							local cx, cy = 0, 0
							for _, p in ipairs(f.pts) do cx += p.x; cy += p.y end
							cx, cy = cx / #f.pts, cy / #f.pts
							if cx > x0 - 1e-9 and cx < x1 + 1e-9
								and cy > y0 - 1e-9 and cy < y1 + 1e-9
								and inRegion(cx, cy) then
								local pts = {}
								for _, hi in ipairs(f.ring) do
									pts[#pts + 1] = verts[halves[hi].from]
								end
								if isConvex(pts, c) then
									collect(pts, false)
								else
									for _, tri in ipairs(earClip(pts, c)) do collect(tri, false) end
								end
							end
						end
					end
				end
			end
		end

		----------------------------------------------------------------
		-- PHASE 3 — fuse pieces back together, fattest merge first.
		--
		-- Phase 1 leaves the rectangle grid it happened to consume, and phase 2
		-- leaves a swarm around every obstacle whose edges are diagonal in this
		-- floor's frame — which is most of them, since the buildings are rotated
		-- relative to the floor they stand on. Both are cured by the same pass:
		-- merge adjacent pieces whenever the union stays convex, taking the
		-- FATTEST union available each time.
		--
		-- Ordering by fatness is what protects the shape. A merge that would
		-- stretch a polygon scores below one that squares it off, so rectangles
		-- fuse into bigger rectangles and fringe triangles fuse into compact
		-- pieces, while the long thin unions simply never get chosen.
		----------------------------------------------------------------
		local vlist: {V2} = {}
		local vkey: { [string]: number } = {}
		local function vid(p: V2): number
			local k = string.format("%d:%d",
				math.floor(p.x / c.weldEps + 0.5), math.floor(p.y / c.weldEps + 0.5))
			local id = vkey[k]
			if not id then vlist[#vlist + 1] = p; id = #vlist; vkey[k] = id end
			return id
		end
		local cyc: { {number}? } = {}
		local cycRect: {boolean} = {}
		for pi2, pts in ipairs(pieces) do
			local cy = {}
			for _, q2 in ipairs(pts) do
				local id = vid(q2)
				if cy[#cy] ~= id then cy[#cy + 1] = id end
			end
			if #cy >= 3 then
				cyc[#cyc + 1] = cy
				cycRect[#cyc] = pieceRect[pi2]
			end
		end

		local clsCache: { [string]: string } = {}
		local function edgeClass(u: number, v: number): string
			local k = (u < v) and (u .. "_" .. v) or (v .. "_" .. u)
			local got = clsCache[k]
			if not got then
				got = classFor(vlist[u], vlist[v])
				clsCache[k] = got
			end
			return got
		end
		local function ptsOf(cy: {number}): {V2}
			local out = {}
			for _, i in ipairs(cy) do out[#out + 1] = vlist[i] end
			return out
		end

		local improving = true
		while improving do
			improving = false
			local owner: { [string]: number } = {}
			for ci, cy in pairs(cyc) do
				if cy then
					for i = 1, #cy do owner[cy[i] .. ">" .. cy[(i % #cy) + 1]] = ci end
				end
			end
			local bestA, bestB, bestCy, bestScore = nil, nil, nil, -1
			for ci, cy in pairs(cyc) do
				if cy then
					for i = 1, #cy do
						local u, v = cy[i], cy[(i % #cy) + 1]
						local oj = owner[v .. ">" .. u]
						if oj and oj ~= ci and cyc[oj] ~= nil and edgeClass(u, v) == "internal" then
							local m = mergeCycles(cy, cyc[oj] :: {number}, edgeClass)
							if m and #m >= 3 then
								local mp = ptsOf(m)
								if isConvex(mp, c) then
									local sc = fatnessOf(mp)
									if sc > bestScore then
										bestScore, bestA, bestB, bestCy = sc, ci, oj, m
									end
								end
							end
						end
					end
				end
			end
			if bestCy then
				cyc[bestA :: number] = bestCy
				cycRect[bestA :: number] = cycRect[bestA :: number] and cycRect[bestB :: number]
				cyc[bestB :: number] = nil
				stats.merges += 1
				improving = true
			end
		end

		----------------------------------------------------------------
		-- PHASE 4 — REPAIR. Enforce the shape limits.
		--
		-- Phase 3 only PREFERS fat unions; it stops as soon as no merge improves
		-- fatness, which leaves any piece whose every union is worse than it is —
		-- exactly the thin stretched ones. So make a second pass whose acceptance
		-- test is different: for a polygon that breaks the limits, take any convex
		-- union that reduces the violation, even if the union scores worse on
		-- fatness than some other polygon would.
		--
		-- A violator with no convex neighbour to join is genuine thin ground (a
		-- 0.5-stud ledge, the strip between a clipramp seam and a rim). Those are
		-- left and counted, because deleting them would open holes.
		----------------------------------------------------------------
		local function violation(pts: {V2}): number
			local wdt, asp = shapeOf(pts)
			local v = 0
			if wdt < c.minWidth then v += (c.minWidth - wdt) / c.minWidth end
			if asp > c.maxAspect then v += (asp - c.maxAspect) / c.maxAspect end
			return v
		end

		-- Merging alone cannot fix this. Phase 3 stops precisely where every
		-- remaining union is non-convex, so a repair pass that also demands
		-- convexity finds nothing to do — measured: 0 repairs, 83 too-thin and
		-- 116 too-long polygons left standing.
		--
		-- So repair RE-DECOMPOSES. Merge the violator with a neighbour even though
		-- the union is concave, then split that union afresh into convex pieces,
		-- and keep the result only when the WORST shape among the new pieces beats
		-- the worst among the two originals. The union is a local, bounded region,
		-- so this is cheap, and it can escape the arrangement the lattice happened
		-- to impose — which merging never can, because it may only delete cuts,
		-- never move them.
		local function redecompose(cy: {number}): {{number}}?
			local pts, back = {}, {}
			for k, id in ipairs(cy) do
				pts[k] = vlist[id]
				back[k] = id
			end
			local tris = earClip(pts, c)
			if #tris == 0 then return nil end
			-- ear clipping returns points; map them back to vertex ids
			local idOf: { [V2]: number } = {}
			for k, pt in ipairs(pts) do idOf[pt] = back[k] end
			local out: { {number}? } = {}
			for _, t in ipairs(tris) do
				local cyi = {}
				for _, pt in ipairs(t) do
					local id = idOf[pt]
					if not id then return nil end
					cyi[#cyi + 1] = id
				end
				out[#out + 1] = cyi
			end
			-- fuse the fresh triangles back up, fattest first
			local going = true
			while going do
				going = false
				local own: { [string]: number } = {}
				for ci2, q2 in pairs(out) do
					if q2 then
						for i = 1, #q2 do own[q2[i] .. ">" .. q2[(i % #q2) + 1]] = ci2 end
					end
				end
				local bA, bB, bC, bS = nil, nil, nil, -1
				for ci2, q2 in pairs(out) do
					if q2 then
						for i = 1, #q2 do
							local u, v = q2[i], q2[(i % #q2) + 1]
							local oj = own[v .. ">" .. u]
							if oj and oj ~= ci2 and out[oj] ~= nil and edgeClass(u, v) == "internal" then
								local m = mergeCycles(q2, out[oj] :: {number}, edgeClass)
								if m and #m >= 3 then
									local mp = ptsOf(m)
									if isConvex(mp, c) then
										local sc = fatnessOf(mp)
										if sc > bS then bS, bA, bB, bC = sc, ci2, oj, m end
									end
								end
							end
						end
					end
				end
				if bC then
					out[bA :: number] = bC
					out[bB :: number] = nil
					going = true
				end
			end
			local res: {{number}} = {}
			for _, q2 in pairs(out) do
				if q2 then res[#res + 1] = q2 end
			end
			return res
		end

		local repairing = true
		local guardR = 0
		while repairing and guardR < 3000 do
			repairing = false
			guardR += 1
			local owner: { [string]: number } = {}
			for ci, cy in pairs(cyc) do
				if cy then
					for i = 1, #cy do owner[cy[i] .. ">" .. cy[(i % #cy) + 1]] = ci end
				end
			end
			for ci, cy in pairs(cyc) do
				if cy then
					local vBefore = violation(ptsOf(cy))
					if vBefore > 0 then
						local bestJ, bestSet, bestV = nil, nil, vBefore
						for i = 1, #cy do
							local u, v = cy[i], cy[(i % #cy) + 1]
							local oj = owner[v .. ">" .. u]
							if oj and oj ~= ci and cyc[oj] ~= nil and edgeClass(u, v) == "internal" then
								local other = cyc[oj] :: {number}
								local m = mergeCycles(cy, other, edgeClass)
								if m and #m >= 3 then
									local worstBefore = math.max(vBefore, violation(ptsOf(other)))
									local set = nil
									local mp = ptsOf(m)
									if isConvex(mp, c) then
										set = { m }
									else
										set = redecompose(m)
									end
									if set then
										local worstAfter = 0
										local okAll = true
										for _, q2 in ipairs(set) do
											local qp = ptsOf(q2)
											if not isConvex(qp, c) then okAll = false; break end
											worstAfter = math.max(worstAfter, violation(qp))
										end
										if okAll and worstAfter < worstBefore - 1e-9 and worstAfter < bestV then
											bestV, bestJ, bestSet = worstAfter, oj, set
										end
									end
								end
							end
						end
						if bestSet then
							local wasRect = cycRect[ci] and cycRect[bestJ :: number]
							cyc[ci] = bestSet[1]
							cycRect[ci] = wasRect
							cyc[bestJ :: number] = nil
							for k = 2, #bestSet do
								cyc[#cyc + 1] = bestSet[k]
								cycRect[#cyc] = false
							end
							stats.repaired += 1
							repairing = true
							break
						end
					end
				end
			end
		end

		for ci, cy in pairs(cyc) do
			if cy then
				-- drop vertices merging left mid-edge, unless the class changes there
				local keep = {}
				for i = 1, #cy do
					local prev, cur, nxt = cy[((i - 2) % #cy) + 1], cy[i], cy[(i % #cy) + 1]
					local a, b, d = vlist[prev], vlist[cur], vlist[nxt]
					local turn = cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y)
					local flat = math.abs(turn) <= 1e-6
					if not (flat and edgeClass(prev, cur) == edgeClass(cur, nxt)) then
						keep[#keep + 1] = b
					end
				end
				-- Collinear cleanup can tip a polygon over the convexity test when a
				-- dropped vertex was carrying a hair of curvature; keep the full
				-- cycle in that case rather than emit something concave.
				if #keep >= 3 and isConvex(keep, c) then
					emit(keep, cycRect[ci])
				elseif #cy >= 3 then
					emit(ptsOf(cy), cycRect[ci])
				end
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
		sub.Name = string.format("P%d_%s_%s_a%.0f_f%.2f",
			pi, p.floor.Name, p.rect and "rect" or "fringe", p.area, p.fatness)
		sub.Parent = folder
		for i = 1, #p.verts do
			local a = p.verts[i]
			local b = p.verts[(i % #p.verts) + 1]
			local d = b - a
			if d.Magnitude > 1e-3 then
				local bar = Instance.new("Part")
				bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
				bar.Size = Vector3.new(d.Magnitude, 0.2, 0.2)
				bar.Color = colours[p.classes[i]] or Color3.new(1, 1, 1)
				bar.Material = Enum.Material.Neon
				bar.Transparency = (p.classes[i] == "internal") and 0.5 or 0
				bar.CFrame = CFrame.fromMatrix((a + b) * 0.5 + UP * 0.15, d.Unit, UP)
				bar.Name = p.classes[i]
				bar.Parent = sub
			end
		end
	end
	return folder
end

return Polys
