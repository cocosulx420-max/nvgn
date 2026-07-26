--!strict
-- NVGN.Polys — claim-and-fill convex decomposition (attempt 5).
--
-- WHAT CHANGED, AND WHY THE LATTICE IS GONE. Attempts 1-4 and the oriented
-- maximum-rectangle cover all placed shapes on a lattice built in some chosen
-- frame, and all of them were rejected for SHAPE. The lattice is the common
-- cause. It forces one question -- "which angle?" -- that has no good answer: a
-- single global frame lets the 200x200 floor's own 800-stud rim outvote every
-- building standing on it, and multiple frames leave gaps that no single
-- lattice can then cover in large pieces. It also bounds fringe pieces to one
-- cell, which is what turned leftovers into hundreds of scraps.
--
-- There is no frame here at all. The input is already exact: Clean steals
-- boundary lines from real face planes (median deviation 0.0000) and Loops
-- closes them into oriented regions with holes. So the job is not to RECOVER a
-- shape, it is to PARTITION one that is already known:
--
--     partition an exact simple polygon with holes
--     into few, convex, well-proportioned pieces
--
-- Recast's pipeline (contour tracing, Douglas-Peucker, ear clipping) does not
-- apply -- every step of it exists to undo a voxel staircase, and there is no
-- staircase here. Ear clipping survives only as an INTERNAL step, never as
-- output: a triangle fan is exactly the shape that keeps getting rejected.
--
-- THE TWO TIERS, and why one algorithm serves both. The spec is: claim big open
-- areas with large well-proportioned convex pieces, then fill what is left with
-- whatever convex shape genuinely fits. That is not two algorithms. Triangulate
-- the region, then merge triangles back together in QUALITY ORDER, and the two
-- tiers fall out of the ordering: the open middle of a room merges into one big
-- piece first because it scores highest, and the awkward ground along walls and
-- around corners is what remains when no further merge stays convex. A leftover
-- that is already a clean pentagon ships as one pentagon; nothing is ever
-- re-cut into rectangles and nothing is ever fanned.
--
-- NO INVENTED VERTICES. Every vertex in the output is a vertex of the input
-- rings. Triangulation adds none (ear clipping only ever removes), merging adds
-- none, and holes are opened with a bridge between two REAL vertices rather
-- than by casting a ray and taking the point where it lands. So a polygon
-- corner is always a place the geometry actually has a corner.
--
-- UNDER-COVER, NEVER OVER-COVER. Negligible notches may be dropped, but only
-- inward: a convex ear of noise scale is cut off, so the mesh stops short of
-- the true boundary and that pocket is simply not navigable. Filling an inward
-- dent is refused outright -- it would put walkable polygon where there is a
-- wall or a ledge, which is the one error the whole boundary pipeline exists to
-- prevent. Discarded area is REPORTED, never absorbed.
--
-- Holes are never bridged ACROSS -- a bridge opens a hole to the outer ring so
-- the piece is simply connected, and merging cannot cross it (the merged ring
-- would repeat a vertex, which is rejected), so it survives as a real edge.

local Loops = require(script.Parent:WaitForChild("Loops"))

local Polys = {}

export type Poly = {
	floor: BasePart,
	verts: {Vector3},
	classes: {string},
	area: number,
	fatness: number,     -- 16*A/P^2; 1 for a square, -> 0 for a needle
	kind: string,        -- "claim" (merged from several) | "atom" (never merged)
	region: number,      -- which Loops region this came from
}
export type Config = {
	weldEps: number?, convexEps: number?, degenerateArea: number?,
	onEdgeEps: number?, discardNotches: boolean?,
	notchArea: number?, notchDepth: number?,
	thinFatness: number?,
}

local DEFAULT = {
	-- Coordinate welding. Loops already welds its own vertices; this only
	-- collapses the coincident pair a hole bridge creates.
	weldEps = 0.02,
	-- Convexity slack, as a raw cross product. Kept tight: a polygon that is
	-- "nearly" convex is a polygon a straight-line crossing can leave.
	convexEps = 1e-4,
	degenerateArea = 1e-3,
	onEdgeEps = 0.02,

	-- How much a cut is discounted for landing on a second concavity, as a
	-- factor on its squared length. 0.25 means such a cut may be twice as long
	-- as the alternative and still win. Unbounded preference reproduces the fans.
	reflexBonus = 0.25,

	-- May a hole bridge land on a point along a boundary edge rather than only
	-- on an existing vertex? The landing point lies ON a real boundary line, so
	-- it satisfies the fill rule, and it is the difference between cutting to
	-- the wall beside an obstacle and fanning to a corner 90 studs away.
	bridgeToEdges = true,

	-- How many of the shortest candidate cuts are re-ranked by the shape they
	-- leave behind. 1 reduces to pure shortest-cut, which produces needles.
	cutCandidates = 16,

	-- Noise-width strips. 0.75 is half the crawl clearance, so nothing of any
	-- size fits; raising this toward agent widths would reintroduce the width
	-- floor the bake deliberately does not have.
	discardStrips = true,
	minStripWidth = 0.75,
	stripBudgetFrac = 0.02, -- of the region's own area

	-- NEGLIGIBLE-NOTCH DISCARD. The threshold is ABSOLUTE and tiny on purpose.
	-- Width was removed from the bake because NPCs are not all human-sized and
	-- sub-agent-width ground stays in the mesh; a discard threshold measured
	-- against a human would reintroduce the width floor through the back door
	-- and strand small agents. This is the scale of AUTHORING NOISE -- two parts
	-- not quite meeting -- not the scale of a feature.
	discardNotches = true,
	notchArea = 0.05,    -- studs^2 of the ear being cut off
	notchDepth = 0.2,    -- studs the boundary is allowed to move inward
	-- and a cap on the ACCUMULATED loss per region, whichever is smaller
	notchBudget = 2.0,       -- studs^2
	notchBudgetFrac = 0.002, -- of the region's own area

	-- Reporting only. Thin polygons are counted, never rejected: narrow ground
	-- is legitimately narrow, and an absolute limit previously reported 15%
	-- "violations" that were mostly correct.
	thinFatness = 0.05,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

type V2 = { x: number, y: number }
type Seg = { a: V2, b: V2, class: string }

--------------------------------------------------------------------------
-- 2D primitives
--------------------------------------------------------------------------

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

local function perimeterOf(pts: {V2}): number
	local p = 0
	for i = 1, #pts do
		local j = (i % #pts) + 1
		local dx, dy = pts[j].x - pts[i].x, pts[j].y - pts[i].y
		p += math.sqrt(dx * dx + dy * dy)
	end
	return p
end

-- Isoperimetric fatness, normalised so a square scores 1. This is the shape
-- number the spec cares about: "well-proportioned" means fat, and a needle
-- tends to zero however large its area.
local function fatnessOf(pts: {V2}): number
	local p = perimeterOf(pts)
	if p <= 1e-9 then return 0 end
	return 16 * math.abs(areaOf(pts)) / (p * p)
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

-- Proper crossing only: segments that merely share an endpoint do not count,
-- because bridges and ring edges legitimately meet at vertices.
local function segCross(p1: V2, p2: V2, p3: V2, p4: V2): boolean
	local d1 = cross2(p4.x - p3.x, p4.y - p3.y, p1.x - p3.x, p1.y - p3.y)
	local d2 = cross2(p4.x - p3.x, p4.y - p3.y, p2.x - p3.x, p2.y - p3.y)
	local d3 = cross2(p2.x - p1.x, p2.y - p1.y, p3.x - p1.x, p3.y - p1.y)
	local d4 = cross2(p2.x - p1.x, p2.y - p1.y, p4.x - p1.x, p4.y - p1.y)
	local e = 1e-9
	return ((d1 > e and d2 < -e) or (d1 < -e and d2 > e))
		and ((d3 > e and d4 < -e) or (d3 < -e and d4 > e))
end

--------------------------------------------------------------------------
-- Ring cleanup
--------------------------------------------------------------------------

local function dedupe(pts: {V2}, eps: number): {V2}
	local out: {V2} = {}
	for _, p in ipairs(pts) do
		local q = out[#out]
		if not q or math.abs(p.x - q.x) > eps or math.abs(p.y - q.y) > eps then
			out[#out + 1] = p
		end
	end
	while #out >= 2 do
		local a, b = out[1], out[#out]
		if math.abs(a.x - b.x) <= eps and math.abs(a.y - b.y) <= eps then
			table.remove(out)
		else
			break
		end
	end
	return out
end

-- Drop vertices that lie on the straight line between their neighbours. This is
-- free: it moves no boundary and loses no area, and every vertex it removes is
-- one fewer place a merge can be blocked.
local function dropCollinear(pts: {V2}, eps: number): {V2}
	local n = #pts
	if n < 4 then return pts end
	local out: {V2} = {}
	for i = 1, n do
		local a, b, d = pts[((i - 2) % n) + 1], pts[i], pts[(i % n) + 1]
		local ux, uy = b.x - a.x, b.y - a.y
		local vx, vy = d.x - b.x, d.y - b.y
		local lu = math.sqrt(ux * ux + uy * uy)
		local lv = math.sqrt(vx * vx + vy * vy)
		local keep = true
		if lu > 1e-9 and lv > 1e-9 then
			-- perpendicular distance from b to the chord a->d
			local wx, wy = d.x - a.x, d.y - a.y
			local lw = math.sqrt(wx * wx + wy * wy)
			if lw > 1e-9 and math.abs(cross2(wx, wy, b.x - a.x, b.y - a.y)) / lw <= eps then
				keep = false
			end
		end
		if keep then out[#out + 1] = b end
	end
	return #out >= 3 and out or pts
end

-- Zero-width spurs: a vertex whose two neighbours are the same point, so the
-- ring runs out and straight back along itself. Loops emits these where a
-- boundary line touches a ring without crossing it. They enclose no area, so
-- removing one is not a discard and does not touch the discard budget -- but
-- leaving one in repeats a vertex, and a repeated vertex makes the piece
-- non-simple, which silently cost a whole 7x7 slab before this existed.
local function dropSpurs(pts: {V2}, eps: number): {V2}
	local ring = pts
	local changed = true
	while changed and #ring >= 3 do
		changed = false
		local n = #ring
		for i = 1, n do
			local p = ring[((i - 2) % n) + 1]
			local q = ring[(i % n) + 1]
			if math.abs(p.x - q.x) <= eps and math.abs(p.y - q.y) <= eps then
				local a, b = i, (i % n) + 1
				if a < b then a, b = b, a end
				table.remove(ring, a)
				table.remove(ring, b)
				changed = true
				break
			end
		end
	end
	return ring
end

-- Cleanup runs to a fixpoint: collapsing a spur can expose collinear vertices,
-- and dropping those can expose another spur.
local function cleanRing(pts: {V2}, eps: number): {V2}
	local ring = pts
	for _ = 1, 8 do
		local before = #ring
		ring = dropSpurs(dedupe(ring, eps), eps)
		ring = dropCollinear(ring, eps)
		if #ring == before then break end
	end
	return ring
end

-- NEGLIGIBLE-NOTCH DISCARD, one direction only.
--
-- Rings are stored outer-CCW and holes-CW, and in that convention a LEFT turn
-- always cuts into the walkable side -- on the outer ring it shaves the region,
-- on a hole ring it grows the hole. Both shrink walkable area, which is the
-- safe direction. A right turn is the opposite case: an inward dent whose
-- removal would ENLARGE the walkable area past a real boundary line. That is
-- refused; it is the one error that walks an agent into a wall.
--
-- This took `sign = -1` for holes at first, which inverted the test for hole
-- rings and discarded in the FORBIDDEN direction. It was invisible in the
-- shape numbers and showed up only as an area error of exactly twice the
-- discarded amount -- area moving the wrong way counts once for being gone and
-- once for being added. The two-column check is the only thing that saw it.
--
-- The ear-empty test doubles as the connectivity guard. If the region pinches
-- near this corner, some other part of the ring lies inside the ear, the ear is
-- rejected, and a narrow route can never be sealed off by noise removal.
-- `budget` caps the TOTAL area a single region may give up. Without it the ear
-- test nibbles: each bite is individually of noise scale, but a long tapering
-- sliver is a chain of such bites and the whole strip can disappear a fraction
-- at a time. The cap is what keeps "discard noise, never discard features"
-- true of the ACCUMULATED effect and not just of each step.
local function discardNotches(pts: {V2}, c: any, budget: number): ({V2}, number)
	if not c.discardNotches then return pts, 0 end
	local ring = pts
	local dropped = 0
	local changed = true
	while changed and #ring > 3 do
		changed = false
		local n = #ring
		for i = 1, n do
			local a, b, d = ring[((i - 2) % n) + 1], ring[i], ring[(i % n) + 1]
			local turn = cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y)
			if turn > 0 then -- left turn: cutting it shrinks walkable area
				local ear = { a, b, d }
				local earArea = math.abs(areaOf(ear))
				if earArea > 0 and earArea <= c.notchArea and dropped + earArea <= budget then
					-- how far the boundary would actually move
					local wx, wy = d.x - a.x, d.y - a.y
					local lw = math.sqrt(wx * wx + wy * wy)
					local depth = lw > 1e-9 and math.abs(cross2(wx, wy, b.x - a.x, b.y - a.y)) / lw or math.huge
					if depth <= c.notchDepth then
						local empty = true
						for k = 1, n do
							if k ~= i and k ~= ((i - 2) % n) + 1 and k ~= (i % n) + 1 then
								if pointInTri(ring[k], a, b, d) then empty = false; break end
							end
						end
						if empty then
							table.remove(ring, i)
							dropped += earArea
							changed = true
							break
						end
					end
				end
			end
		end
	end
	return ring, dropped
end

--------------------------------------------------------------------------
-- Holes: open each one to the outer ring with a bridge between REAL vertices.
--
-- The textbook construction casts a ray from the hole's rightmost vertex and
-- bridges to where it LANDS, which is a point the geometry does not have. We
-- bridge to the nearest visible existing vertex instead. It costs an O(n^2)
-- scan per hole -- rings here are tens of vertices, not thousands -- and it
-- keeps the promise that no polygon corner is ever invented.
--------------------------------------------------------------------------

local function bridgeHoles(outer: {V2}, holes: {{V2}}, c: any): {V2}
	local ring: {V2} = table.clone(outer)
	local pending = table.clone(holes)

	while #pending > 0 do
		-- Take the hole whose best bridge is shortest, so short obvious bridges
		-- are committed before long ones can get in their way.
		--
		-- Candidate landing points are ring vertices AND, when `bridgeToEdges` is
		-- on, the foot of the perpendicular onto a ring edge. Vertices alone are
		-- not enough: an obstacle standing in the middle of open floor has no
		-- vertex near it, so it reaches for whatever corner is visible and the
		-- bridge fans 90 studs across ground it should not touch. The foot lies
		-- ON a real boundary line, which is where the fill rule puts vertices,
		-- and inserting it splits that ring edge so no T-junction is created.
		local bestHole, bestRi, bestHi, bestD = nil, nil, nil, math.huge
		local bestFoot: V2? = nil
		for hIdx, hole in ipairs(pending) do
			for hi = 1, #hole do
				local m = hole[hi]

				local targets: {{ ri: number, p: V2, foot: boolean }} = {}
				for ri = 1, #ring do
					targets[#targets + 1] = { ri = ri, p = ring[ri], foot = false }
				end
				if c.bridgeToEdges then
					for ri = 1, #ring do
						local p, q = ring[ri], ring[(ri % #ring) + 1]
						local dx, dy = q.x - p.x, q.y - p.y
						local L2 = dx * dx + dy * dy
						if L2 > 1e-9 then
							local t = ((m.x - p.x) * dx + (m.y - p.y) * dy) / L2
							if t > 1e-4 and t < 1 - 1e-4 then
								targets[#targets + 1] = {
									ri = ri, foot = true,
									p = { x = p.x + dx * t, y = p.y + dy * t },
								}
							end
						end
					end
				end

				for _, tg in ipairs(targets) do
					local v = tg.p
					local dx, dy = v.x - m.x, v.y - m.y
					local dist = dx * dx + dy * dy
					if dist < bestD and dist > 1e-12 then
						-- visible: crosses no ring edge and no hole edge
						local ok = true
						for k = 1, #ring do
							local p, q = ring[k], ring[(k % #ring) + 1]
							if segCross(m, v, p, q) then ok = false; break end
						end
						if ok then
							for _, oh in ipairs(pending) do
								for k = 1, #oh do
									local p, q = oh[k], oh[(k % #oh) + 1]
									if segCross(m, v, p, q) then ok = false; break end
								end
								if not ok then break end
							end
						end
						-- and the bridge must run through the walkable side
						if ok then
							local mx, my = (m.x + v.x) * 0.5, (m.y + v.y) * 0.5
							if not pointInPoly(ring, mx, my) then
								ok = false
							else
								for _, oh in ipairs(pending) do
									if pointInPoly(oh, mx, my) then ok = false; break end
								end
							end
						end
						if ok then
							bestHole, bestRi, bestHi, bestD = hIdx, tg.ri, hi, dist
								bestFoot = tg.foot and tg.p or nil
						end
					end
				end
			end
		end

		if not bestHole then
			-- No visible bridge for any remaining hole. Leaving them out would
			-- silently cover the obstacle with walkable polygon, which is the
			-- forbidden direction, so bail and let the area assertion report it.
			break
		end

		local hole = table.remove(pending, bestHole :: number) :: {V2}
		local ri, hi = bestRi :: number, bestHi :: number
		if bestFoot then
			-- land on the edge itself: insert the foot as a real ring vertex so
			-- the edge is split rather than crossed
			table.insert(ring, ri + 1, bestFoot :: V2)
			ri = ri + 1
		end
		local out: {V2} = {}
		for i = 1, ri do out[#out + 1] = ring[i] end
		for k = 0, #hole - 1 do out[#out + 1] = hole[((hi - 1 + k) % #hole) + 1] end
		out[#out + 1] = hole[hi]          -- close the hole walk
		for i = ri, #ring do out[#out + 1] = ring[i] end
		ring = out
	end

	return ring
end

--------------------------------------------------------------------------
-- Ear-clip triangulation. Internal only: nothing here reaches the output.
--------------------------------------------------------------------------

local function triangulate(ring: {V2}, c: any): {{V2}}
	local idx: {number} = {}
	for i = 1, #ring do idx[i] = i end
	local tris: {{V2}} = {}
	if #ring < 3 then return tris end

	local guard = 0
	while #idx > 3 do
		guard += 1
		if guard > #ring * #ring + 16 then break end -- pathological ring; area check will say so
		local cut = false
		local n = #idx
		for i = 1, n do
			local ia = idx[((i - 2) % n) + 1]
			local ib = idx[i]
			local ic = idx[(i % n) + 1]
			local a, b, d = ring[ia], ring[ib], ring[ic]
			local area = cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y)
			if area > 1e-9 then
				local empty = true
				for k = 1, n do
					local ik = idx[k]
					if ik ~= ia and ik ~= ib and ik ~= ic then
						if pointInTri(ring[ik], a, b, d) then empty = false; break end
					end
				end
				if empty then
					tris[#tris + 1] = { a, b, d }
					table.remove(idx, i)
					cut = true
					break
				end
			end
		end
		if not cut then
			-- Nothing was a valid ear: drop the sharpest vertex so the loop
			-- terminates. Whatever this loses shows up in the area column.
			table.remove(idx, 1)
		end
	end
	if #idx == 3 then
		tris[#tris + 1] = { ring[idx[1]], ring[idx[2]], ring[idx[3]] }
	end
	return tris
end

--------------------------------------------------------------------------
-- CONVEX DECOMPOSITION BY RESOLVING CONCAVITIES.
--
-- Triangulating and merging back was tried first and looks wrong on screen, for
-- a reason worth keeping: ear clipping chooses its diagonals for convenience,
-- so it fans a region from whatever vertex happens to work, and the fan's long
-- diagonals cross open ground. Merging cannot undo them -- once the open middle
-- has been sliced by a cut that spans it, no pair of the resulting slices is
-- convex when united. The cuts have to be placed well in the first place.
--
-- So cut at the CONCAVITIES instead. A convex region needs no cut at all; every
-- cut a region does need exists to resolve some reflex vertex. Take a reflex
-- vertex, run one diagonal from it to the best vertex available, split, and
-- recurse. Preferring a diagonal that lands on ANOTHER reflex vertex resolves
-- two concavities with one cut, and preferring the shortest keeps the cut local
-- to the awkward corner instead of letting it span the room. Open ground is
-- never crossed, because open ground has no reflex vertex to justify a cut.
--
-- Both endpoints are existing vertices, so this invents nothing.
--------------------------------------------------------------------------

local function isReflex(pts: {V2}, i: number, c: any): boolean
	local n = #pts
	local a, b, d = pts[((i - 2) % n) + 1], pts[i], pts[(i % n) + 1]
	return cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y) < -c.convexEps
end

-- Does the ray from `a` toward `b` leave `a` on the INSIDE of the ring? Which
-- side of the two edges at `a` counts as inside depends on whether `a` is a
-- convex or a reflex corner, hence the two branches. The ring is CCW here.
local function inCone(prev: V2, a: V2, nxt: V2, b: V2): boolean
	local function left(p: V2, q: V2, r: V2): number
		return cross2(q.x - p.x, q.y - p.y, r.x - p.x, r.y - p.y)
	end
	if left(a, nxt, prev) >= 0 then -- convex corner
		return left(a, b, prev) > 0 and left(b, a, nxt) > 0
	end
	return not (left(a, b, nxt) >= 0 and left(b, a, prev) >= 0)
end

-- A diagonal is usable when it stays strictly inside the ring: it leaves the
-- corner on the inside, crosses no edge, and grazes no vertex or edge.
--
-- The interior test used to be a single midpoint sample, and that is not
-- sufficient: a long diagonal can pass straight through a hole and still have
-- its midpoint land on walkable ground. It read as valid, the split produced
-- reversed pieces, and 26 of 70 pieces on the big floor came out negatively
-- wound with 17k studs^2 of ground double-covered. The cone test is the
-- classical condition and is decided at the corner, not by sampling.
local function diagonalOk(pts: {V2}, i: number, j: number, c: any): boolean
	local n = #pts
	if i == j then return false end
	if (i % n) + 1 == j or (j % n) + 1 == i then return false end
	local a, b = pts[i], pts[j]
	-- A hole bridge puts two COINCIDENT vertices in the ring. The segment between
	-- them has zero length, so by "shortest cut wins" it beats every real
	-- candidate, and splitting on it produces overlapping pieces -- 17k studs^2
	-- of double-covered ground before this check existed.
	do
		local dx0, dy0 = b.x - a.x, b.y - a.y
		if dx0 * dx0 + dy0 * dy0 <= c.weldEps * c.weldEps then return false end
	end
	for k = 1, n do
		local k2 = (k % n) + 1
		if k ~= i and k ~= j and k2 ~= i and k2 ~= j then
			if segCross(a, b, pts[k], pts[k2]) then return false end
		end
	end
	local dx, dy = b.x - a.x, b.y - a.y
	local L2 = dx * dx + dy * dy
	if L2 <= 1e-12 then return false end
	for k = 1, n do
		if k ~= i and k ~= j then
			local p = pts[k]
			local t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2
			if t > 1e-6 and t < 1 - 1e-6 then
				local ex, ey = p.x - (a.x + dx * t), p.y - (a.y + dy * t)
				if ex * ex + ey * ey < 1e-12 then return false end
			end
		end
	end
	-- An edge lying ALONG the diagonal crosses nothing and grazes no vertex, so
	-- neither test above sees it. A hole bridge is exactly such an edge.
	for k = 1, n do
		local k2 = (k % n) + 1
		if k ~= i and k ~= j and k2 ~= i and k2 ~= j then
			local mx = (pts[k].x + pts[k2].x) * 0.5
			local my = (pts[k].y + pts[k2].y) * 0.5
			local t = ((mx - a.x) * dx + (my - a.y) * dy) / L2
			if t > 1e-6 and t < 1 - 1e-6 then
				local ex, ey = mx - (a.x + dx * t), my - (a.y + dy * t)
				if ex * ex + ey * ey < 1e-12 then return false end
			end
		end
	end
	return inCone(pts[((i - 2) % n) + 1], a, pts[(i % n) + 1], b)
		and inCone(pts[((j - 2) % n) + 1], b, pts[(j % n) + 1], a)
		and pointInPoly(pts, (a.x + b.x) * 0.5, (a.y + b.y) * 0.5)
end

-- The two rings a cut from i to j produces. Both keep the parent's winding.
local function splitRing(pts: {V2}, i: number, j: number): ({V2}, {V2})
	local n = #pts
	local A: {V2} = {}
	local k = i
	while true do
		A[#A + 1] = pts[k]
		if k == j then break end
		k = (k % n) + 1
	end
	local B: {V2} = {}
	k = j
	while true do
		B[#B + 1] = pts[k]
		if k == i then break end
		k = (k % n) + 1
	end
	return A, B
end

local function decompose(ring: {V2}, c: any): {{V2}}
	local out: {{V2}} = {}
	local stack: {{V2}} = { ring }
	local guard = 0

	while #stack > 0 do
		guard += 1
		if guard > 20000 then break end -- pathological input; the area check will say so
		local pts = table.remove(stack) :: {V2}
		local n = #pts
		if n < 3 then continue end
		if math.abs(areaOf(pts)) < c.degenerateArea then continue end

		local reflex = {}
		for i = 1, n do
			if isReflex(pts, i, c) then reflex[#reflex + 1] = i end
		end
		if #reflex == 0 then
			out[#out + 1] = pts
			continue
		end

		-- Choose the best cut over ALL concavities, not the first one found.
		-- Taking the first anchors every subsequent cut at the same vertex, which
		-- is what produced fans radiating across open floor.
		--
		-- TWO STAGES, because neither criterion works alone. Length shortlists:
		-- an earlier version let a reflex target win at any distance, so a
		-- 91-stud cut beat a 3-stud one and the fans came straight back. But
		-- length alone picks needles -- the shortest cut is often the one that
		-- shaves a sliver off a corner, and several such cuts converging on one
		-- vertex is exactly the thin-wedge shape being complained about. So the
		-- shortlist is ranked by length and the winner is chosen by the SHAPE it
		-- leaves: maximise the fatness of the WORSE of the two pieces, so a cut
		-- is only taken if both sides of it are worth having.
		local cands = {}
		for _, ri in ipairs(reflex) do
			for j = 1, n do
				if j ~= ri and diagonalOk(pts, ri, j, c) then
					local dx, dy = pts[j].x - pts[ri].x, pts[j].y - pts[ri].y
					local d2 = dx * dx + dy * dy
					cands[#cands + 1] = {
						i = ri, j = j,
						score = isReflex(pts, j, c) and d2 * c.reflexBonus or d2,
					}
				end
			end
		end
		table.sort(cands, function(x, y)
			if x.score ~= y.score then return x.score < y.score end
			if x.i ~= y.i then return x.i < y.i end -- deterministic
			return x.j < y.j
		end)

		local i, best, bestQ = nil, nil, -1
		for t = 1, math.min(#cands, c.cutCandidates) do
			local cd = cands[t]
			local A, B = splitRing(pts, cd.i, cd.j)
			local q = math.min(fatnessOf(A), fatnessOf(B))
			if q > bestQ then bestQ = q; i = cd.i; best = cd.j end
		end

		if not i or not best then
			-- No diagonal from any reflex vertex. Should not happen for a simple
			-- ring; fall back so the area still balances rather than dropping it.
			for _, t in ipairs(triangulate(pts, c)) do out[#out + 1] = t end
			continue
		end

		local A, B = splitRing(pts, i :: number, best :: number)
		if #A >= 3 then stack[#stack + 1] = A end
		if #B >= 3 then stack[#stack + 1] = B end
	end

	return out
end

--------------------------------------------------------------------------
-- Merge the pieces further where a union is still convex, best shape first.
--
-- This is Hertel-Mehlhorn with a quality ordering. HM removes every diagonal
-- whose removal leaves the result convex, and bounds the piece count at 4x
-- optimal whatever order is used -- so the order is free to chase SHAPE, which
-- is the thing four previous attempts were rejected on. Scoring a merge by
-- fatness*area is what makes tier 1 emerge: the open middle of a room is the
-- fattest, largest merge available, so it is claimed first and grows.
--------------------------------------------------------------------------

local function ekey(a: number, b: number): string
	if a < b then return a .. ":" .. b end
	return b .. ":" .. a
end

-- Which run of P's edges is shared with the neighbour. Two faces that have grown
-- against each other commonly share a CHAIN of edges, not a single one; refusing
-- those merges is what strands triangles along a wall, so the chain is removed
-- whole. More than one separate run means the union would enclose a hole, and
-- that merge is refused.
local function sharedRun(P: {number}, shared: {[string]: boolean}): (number?, number?)
	local n = #P
	local flag = {}
	local count = 0
	for i = 1, n do
		flag[i] = shared[ekey(P[i], P[(i % n) + 1])] or false
		if flag[i] then count += 1 end
	end
	if count == 0 or count == n then return nil, nil end
	-- start of a run: shared edge whose predecessor is not shared
	local starts = {}
	for i = 1, n do
		if flag[i] and not flag[((i - 2) % n) + 1] then starts[#starts + 1] = i end
	end
	if #starts ~= 1 then return nil, nil end
	local i0 = starts[1]
	local i1 = i0
	while flag[(i1 % n) + 1] do i1 = (i1 % n) + 1 end
	return i0, i1
end

-- `polys` carries `false` for a face that has been absorbed, never nil: a nil in
-- the middle of an array truncates every ipairs over it, which silently deletes
-- polygons and showed up as a quarter of the map missing from the area column.
local function mergeConvex(polys: any, V: {V2}, boundary: {[string]: boolean}, c: any): number
	local merges = 0
	while true do
		-- who owns each edge
		local owner: {[string]: {number}} = {}
		for pi = 1, #polys do
			local p = polys[pi]
			if p then
				for i = 1, #p do
					local k = ekey(p[i], p[(i % #p) + 1])
					local o = owner[k]
					if not o then o = {}; owner[k] = o end
					o[#o + 1] = pi
				end
			end
		end

		-- group the shared edges by the PAIR of faces that own them, so a chain is
		-- considered as one merge rather than as several blocked ones
		local adj: {[string]: any} = {}
		for k, o in pairs(owner) do
			if #o == 2 and o[1] ~= o[2] and not boundary[k] then
				local a, b = o[1], o[2]
				if a > b then a, b = b, a end
				local pk = a .. "|" .. b
				local e = adj[pk]
				if not e then e = { a = a, b = b, edges = {} }; adj[pk] = e end
				table.insert(e.edges, k)
			end
		end

		local cands = {}
		for _, pr in pairs(adj) do
			local A, B = polys[pr.a], polys[pr.b]
			if A and B then
				local shared: {[string]: boolean} = {}
				for _, k in ipairs(pr.edges) do shared[k] = true end
				local i0, i1 = sharedRun(A, shared)
				local j0, j1 = sharedRun(B, shared)
				if i0 and i1 and j0 and j1 then
					local na, nb = #A, #B
					local chainA = ((i1 - i0) % na) + 1
					local chainB = ((j1 - j0) % nb) + 1
					local walkA = na - chainA + 1
					local walkB = nb - chainB + 1
					local ring: {number} = {}
					local dup: {[number]: boolean} = {}
					local bad = false
					-- A, from the far end of the shared chain round to its start
					local ia = (i1 % na) + 1
					for _ = 1, walkA do
						local v = A[ia]
						if dup[v] then bad = true; break end
						dup[v] = true; ring[#ring + 1] = v
						ia = (ia % na) + 1
					end
					-- B, the same walk, minus the two vertices A already supplied
					if not bad and walkB >= 2 then
						local ib = ((j1 + 1) % nb) + 1
						for _ = 1, walkB - 2 do
							local v = B[ib]
							if dup[v] then bad = true; break end
							dup[v] = true; ring[#ring + 1] = v
							ib = (ib % nb) + 1
						end
					elseif not bad and walkB < 2 then
						bad = true
					end
					if not bad and #ring >= 3 then
						local pts: {V2} = {}
						for t, v in ipairs(ring) do pts[t] = V[v] end
						local ar = areaOf(pts)
						if ar > 0 and isConvex(pts, c) then
							cands[#cands + 1] = {
								a = pr.a, b = pr.b, ring = ring,
								score = fatnessOf(pts) * ar,
							}
						end
					end
				end
			end
		end

		if #cands == 0 then break end
		table.sort(cands, function(x, y)
			if x.score ~= y.score then return x.score > y.score end
			return #x.ring < #y.ring -- deterministic: table order must never decide
		end)

		local dirty: {[number]: boolean} = {}
		local did = 0
		for _, cd in ipairs(cands) do
			if not dirty[cd.a] and not dirty[cd.b] then
				polys[cd.a] = cd.ring
				polys[cd.b] = false
				dirty[cd.a] = true
				dirty[cd.b] = true
				did += 1
				merges += 1
			end
		end
		if did == 0 then break end

		-- compact, dropping the absorbed slots
		local out: {{number}} = {}
		for i = 1, #polys do
			local p = polys[i]
			if p then out[#out + 1] = p end
		end
		table.clear(polys :: any)
		for i, p in ipairs(out) do polys[i] = p end
	end
	return merges
end

--------------------------------------------------------------------------

function Polys.fromLoops(lres: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local polys: {Poly} = {}
	local adjacency = {}
	local stats = {
		edgeOverOwned = 0,
		regions = 0, polys = 0, tris = 0, merges = 0,
		claims = 0, atoms = 0,
		areaIn = 0, areaOut = 0, areaDiscarded = 0,
		maxVerts = 0, nonConvex = 0, thin = 0,
		badRegions = 0, worstAreaErr = 0, worstFloor = "",
		holes = 0, holesUnbridged = 0, facesDropped = 0,
		strips = 0, stripArea = 0,
		fatSum = 0, worstFat = 1, worstFatFloor = "",
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

		-- The rings as Loops handed them over, plus the segments we later ask
		-- "what class is this output edge?" against.
		local bsegs: {Seg} = {}
		local rawOuter: {V2} = {}
		for i, e in ipairs(r.edges) do rawOuter[i] = proj(e.a) end
		for i = 1, #rawOuter do
			bsegs[#bsegs + 1] = {
				a = rawOuter[i], b = rawOuter[(i % #rawOuter) + 1], class = r.edges[i].class,
			}
		end
		local rawHoles: {{V2}} = {}
		for _, hr in ipairs(r.holes) do
			local hp: {V2} = {}
			for i, e in ipairs(hr) do hp[i] = proj(e.a) end
			rawHoles[#rawHoles + 1] = hp
			for i = 1, #hp do
				bsegs[#bsegs + 1] = { a = hp[i], b = hp[(i % #hp) + 1], class = hr[i].class }
			end
		end
		stats.holes += #rawHoles

		-- Orientation is normalised here rather than trusted: the whole
		-- convexity and ear test suite is sign-sensitive, and a ring handed over
		-- the other way round silently produces the complement.
		local outer = cleanRing(rawOuter, c.weldEps)
		if areaOf(outer) < 0 then
			local rev: {V2} = {}
			for i = #outer, 1, -1 do rev[#rev + 1] = outer[i] end
			outer = rev
		end
		local holes: {{V2}} = {}
		for _, hp in ipairs(rawHoles) do
			local h = cleanRing(hp, c.weldEps)
			if #h >= 3 then
				if areaOf(h) > 0 then
					local rev: {V2} = {}
					for i = #h, 1, -1 do rev[#rev + 1] = h[i] end
					h = rev
				end
				holes[#holes + 1] = h
			end
		end
		if #outer < 3 then continue end

		local regionArea = math.abs(areaOf(outer))
		for _, hp in ipairs(holes) do regionArea -= math.abs(areaOf(hp)) end
		stats.areaIn += regionArea

		-- notch discard, outer ring inward and hole rings outward
		local discarded = 0
		do
			local budget = math.min(c.notchBudget, regionArea * c.notchBudgetFrac)
			local d
			outer, d = discardNotches(outer, c, budget - discarded)
			discarded += d
			for hi, hp in ipairs(holes) do
				local h2, d2 = discardNotches(hp, c, budget - discarded)
				holes[hi] = h2
				discarded += d2
			end
		end
		stats.areaDiscarded += discarded

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

		----------------------------------------------------------------
		-- triangulate, then merge back in quality order
		----------------------------------------------------------------
		local holesBefore = #holes
		local ring = bridgeHoles(outer, holes, c)
		local tris = decompose(ring, c)
		stats.tris += #tris

		-- weld triangle corners onto a shared vertex table so adjacency is by
		-- identity rather than by distance
		local V: {V2} = {}
		local lookup: {[string]: number} = {}
		local function vid(p: V2): number
			local k = string.format("%d:%d", math.round(p.x / c.weldEps), math.round(p.y / c.weldEps))
			local id = lookup[k]
			if id then return id end
			V[#V + 1] = p
			lookup[k] = #V
			return #V
		end

		local faces: {{number}} = {}
		for _, t in ipairs(tris) do
			local ids: {number} = {}
			local dup: {[number]: boolean} = {}
			local ok = true
			for _, p in ipairs(t) do
				local v = vid(p)
				if dup[v] then ok = false; break end
				dup[v] = true
				ids[#ids + 1] = v
			end
			if ok and #ids >= 3 then
				faces[#faces + 1] = ids
			else
				-- never silent: a rejected piece is area that went missing
				stats.facesDropped += 1
			end
		end

		-- Original ring edges are never removable: they are the boundary. Bridge
		-- edges are not listed, but a merge across one would repeat a vertex and
		-- is rejected on that basis, so they survive as real edges too.
		local boundary: {[string]: boolean} = {}
		local function markRing(pts: {V2})
			for i = 1, #pts do
				boundary[ekey(vid(pts[i]), vid(pts[(i % #pts) + 1]))] = true
			end
		end
		markRing(outer)
		for _, hp in ipairs(holes) do markRing(hp) end

		local atomCount = #faces
		stats.merges += mergeConvex(faces, V, boundary, c)

		----------------------------------------------------------------
		-- NOISE-WIDTH STRIP DISCARD (Cocosulx's call, after review).
		--
		-- Some polygons are thin because the GROUND is thin, not because the
		-- decomposition chose badly -- searching every candidate cut leaves the
		-- worst case identical, so these are forced by the region outline. The
		-- widest of them is 0.75 studs, which is half the crawl clearance: no
		-- agent of any size fits, and the ground exists only because a wall was
		-- authored a fraction of a stud off the floor's rim. That is the same
		-- noise scale Clean's coplanarEps already merges away for lines.
		--
		-- This is a width test, which the bake otherwise has none of, so it is
		-- fenced in hard. It only ever removes area (under-covering, the safe
		-- direction), and it refuses three ways:
		--
		--   SEAM      a seam is a portal. Every ClipRamp entry on the test scene
		--             is carried by a strip 0.58 studs wide; discarding those
		--             deletes the ramp entrances outright. Checked before
		--             writing this, and it is why the guard exists.
		--   CONTINUATION  the handover to an overlapping floor. Deleting one
		--             severs a link this region cannot even see.
		--   CONNECTIVITY  the strip must not be the only thing joining two parts
		--             of its region, re-tested after each removal so several
		--             discards cannot jointly disconnect what none does alone.
		----------------------------------------------------------------
		local dropped: {[number]: boolean} = {}
		if c.discardStrips then
			local function faceEdgeKeys(f: {number}): {string}
				local ks = {}
				for k = 1, #f do ks[#ks + 1] = ekey(f[k], f[(k % #f) + 1]) end
				return ks
			end
			local function connectedWithout(skip: number): boolean
				local owner: {[string]: {number}} = {}
				local live = {}
				for i, f in ipairs(faces) do
					if not dropped[i] and i ~= skip then
						live[#live + 1] = i
						for _, k in ipairs(faceEdgeKeys(f)) do
							owner[k] = owner[k] or {}
							table.insert(owner[k], i)
						end
					end
				end
				if #live <= 1 then return true end
				local adj: {[number]: {number}} = {}
				for _, o in pairs(owner) do
					if #o == 2 then
						adj[o[1]] = adj[o[1]] or {}; table.insert(adj[o[1]], o[2])
						adj[o[2]] = adj[o[2]] or {}; table.insert(adj[o[2]], o[1])
					end
				end
				local seenF: {[number]: boolean} = { [live[1]] = true }
				local queue, head, reached = { live[1] }, 1, 1
				while head <= #queue do
					local cur = queue[head]; head += 1
					for _, nb in ipairs(adj[cur] or {}) do
						if not seenF[nb] then
							seenF[nb] = true; reached += 1; queue[#queue + 1] = nb
						end
					end
				end
				return reached == #live
			end

			-- worst-shaped first, so the budget goes to the pieces that matter
			local order = {}
			for i, f in ipairs(faces) do
				local pts: {V2} = {}
				for k, v in ipairs(f) do pts[k] = V[v] end
				local longest = 0
				for k = 1, #pts do
					local q = pts[(k % #pts) + 1]
					local dx, dy = q.x - pts[k].x, q.y - pts[k].y
					longest = math.max(longest, math.sqrt(dx * dx + dy * dy))
				end
				local a = math.abs(areaOf(pts))
				if longest > 1e-6 and a / longest < c.minStripWidth then
					order[#order + 1] = { i = i, pts = pts, area = a, w = a / longest }
				end
			end
			table.sort(order, function(x, y)
				if x.w ~= y.w then return x.w < y.w end
				return x.i < y.i -- deterministic
			end)

			local budget = regionArea * c.stripBudgetFrac
			local used = 0
			for _, cd in ipairs(order) do
				local blocked = false
				for k = 1, #cd.pts do
					local cls = classFor(cd.pts[k], cd.pts[(k % #cd.pts) + 1])
					if cls == "seam" or cls == "continuation" then blocked = true; break end
				end
				if not blocked and used + cd.area <= budget and connectedWithout(cd.i) then
					dropped[cd.i] = true
					used += cd.area
					discarded += cd.area
					stats.strips += 1
					stats.stripArea += cd.area
				end
			end
			stats.areaDiscarded += used
		end

		local got = 0
		local faceToPoly: {[number]: number} = {}
		for fi, f in ipairs(faces) do
			if dropped[fi] then continue end
			local pts: {V2} = {}
			for i, v in ipairs(f) do pts[i] = V[v] end
			local a = math.abs(areaOf(pts))
			if a >= c.degenerateArea then
				local vs, cs = {}, {}
				for i = 1, #pts do
					vs[#vs + 1] = unproj(pts[i])
					cs[#cs + 1] = classFor(pts[i], pts[(i % #pts) + 1])
				end
				local fat = fatnessOf(pts)
				if not isConvex(pts, c) then stats.nonConvex += 1 end
				if fat < c.thinFatness then stats.thin += 1 end
				if fat < stats.worstFat then
					stats.worstFat = fat
					stats.worstFatFloor = r.floor.Name
				end
				stats.maxVerts = math.max(stats.maxVerts, #pts)
				stats.fatSum += fat
				stats.areaOut += a
				stats.polys += 1
				got += a
				local kind = (#pts > 3) and "claim" or "atom"
				if kind == "claim" then stats.claims += 1 else stats.atoms += 1 end
				polys[#polys + 1] = {
					floor = r.floor, verts = vs, classes = cs, area = a, fatness = fat, kind = kind,
					region = stats.regions,
				}
				faceToPoly[fi] = #polys
			end
		end

		-- INTRA-REGION ADJACENCY, taken here because it is exact and free.
		--
		-- Faces of one region share vertex IDENTITIES, not merely coincident
		-- coordinates, so two faces are neighbours exactly when they name the
		-- same edge. Recovering this later from world positions would replace an
		-- exact test with a tolerance, on the one part of the problem that does
		-- not need one.
		do
			local owner: {[string]: {number}} = {}
			for fi, f in ipairs(faces) do
				if faceToPoly[fi] then
					for k = 1, #f do
						local key = ekey(f[k], f[(k % #f) + 1])
						owner[key] = owner[key] or {}
						table.insert(owner[key], fi)
					end
				end
			end
			for key, o in pairs(owner) do
				if #o == 2 and o[1] ~= o[2] then
					local ia, ib = key:match("^(%d+):(%d+)$")
					local va, vb = V[tonumber(ia) :: number], V[tonumber(ib) :: number]
					adjacency[#adjacency + 1] = {
						a = faceToPoly[o[1]], b = faceToPoly[o[2]],
						p1 = unproj(va), p2 = unproj(vb),
						class = classFor(va, vb),
					}
				elseif #o > 2 then
					-- a hole bridge is named by faces on both sides of the slit
					stats.edgeOverOwned += 1
				end
			end
		end
		if holesBefore > 0 and #ring == #outer then stats.holesUnbridged += holesBefore end

		-- PER-REGION AREA CONSERVATION, in studs^2, in TWO columns.
		--
		-- Mandatory from the first commit. Every geometry bug this pipeline has
		-- produced was silent -- self-intersecting rings, faces escaping their
		-- cell, ears never found -- and each showed up here as a number and
		-- nowhere else. The second column is what keeps the discard honest:
		-- dropped notches are ACCOUNTED, not absorbed, so 0.3 studs^2 across a
		-- floor reads as the noise we meant to drop and 40 in one region reads
		-- as a bug, immediately and to both of us.
		local err = math.abs((got + discarded) - regionArea)
		if err > math.max(0.01, regionArea * 1e-6) then
			stats.badRegions += 1
			if err > stats.worstAreaErr then
				stats.worstAreaErr = err
				stats.worstFloor = r.floor.Name
				-- keep the offending input so a failure can be reproduced without
				-- re-baking the whole scene
				stats.worstRing = { outer = outer, holes = holes, faces = #faces, pieces = #tris }
			end
		end
	end

	stats.avgFat = stats.polys > 0 and stats.fatSum / stats.polys or 0
	stats.adjacency = #adjacency
	stats.seconds = os.clock() - t0
	return { polys = polys, adjacency = adjacency, stats = stats, config = c }
end

-- Exposed for diagnosis: a failing region can be replayed from its ring alone.
Polys._decompose = decompose
Polys._triangulate = triangulate
Polys._bridgeHoles = bridgeHoles
Polys._areaOf = areaOf

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
	-- Boundary edges keep the pipeline's established legend; interior edges (the
	-- cuts this stage chose) are the only new thing on screen, drawn dimmer so
	-- the shape of a polygon reads before its cuts do.
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
		sub.Name = string.format("P%d_%s_%s_a%.0f_v%d_f%.2f",
			pi, p.floor.Name, p.kind, p.area, #p.verts, p.fatness)
		sub.Parent = folder
		for i = 1, #p.verts do
			local a, b = p.verts[i], p.verts[(i % #p.verts) + 1]
			local d = b - a
			if d.Magnitude > 1e-3 then
				local cls = p.classes[i]
				local bar = Instance.new("Part")
				bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
				bar.Size = Vector3.new(d.Magnitude, 0.2, 0.2)
				bar.Color = colours[cls] or Color3.new(1, 1, 1)
				bar.Material = Enum.Material.Neon
				bar.Transparency = (cls == "internal") and 0.55 or 0
				bar.CFrame = CFrame.fromMatrix((a + b) * 0.5 + UP * 0.15, d.Unit, UP)
				bar.Name = cls
				bar.Parent = sub
			end
		end
	end
	return folder
end

return Polys
