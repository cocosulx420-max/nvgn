--!strict
-- NVGN.Polys — convex decomposition of the assembled regions.
--
-- The target shape of the output is a design requirement, not an afterthought:
-- big open ground should be a FEW LARGE polygons, detail should appear only
-- where the geometry actually demands it, and long thin slivers are unwanted
-- everywhere. That rules out the textbook route. Ear clipping followed by
-- Hertel–Mehlhorn merges diagonals in whatever order the vertices happen to sit
-- in, which gives a uniform drizzle of similar polygons and keeps whichever
-- slivers the triangulator produced.
--
-- So the merge is QUALITY-ORDERED. Every pair of adjacent polygons whose union
-- is convex is a candidate; the candidate whose union is FATTEST wins, and the
-- process repeats until no convex merge remains. Fatness is the isoperimetric
-- ratio 4*pi*area / perimeter^2 — 1 for a circle, ~0.79 for a square, and
-- falling toward 0 as a shape stretches. Ordering by it means a big open floor
-- collapses into one or two broad polygons, while a stair tread or a ledge keeps
-- the pieces its shape requires.
--
-- Two honest limits:
--
--   * A genuinely thin region yields a thin polygon. A 0.45-stud strip between a
--     seam and a rim has no fat decomposition, and inventing one would mean
--     covering ground that is not walkable. Those are measured and reported
--     rather than hidden.
--   * Greedy is not optimal. Optimal convex partition is far more expensive and
--     buys little here, since the acceptance test is exact convexity either way.
--
-- Class carrying: every polygon edge remembers whether it came from a wall,
-- seam, dropoff, tier or continuation, or whether it is an internal diagonal
-- this module introduced. Portal derivation later reads exactly that.

local Loops = require(script.Parent:WaitForChild("Loops"))

local Polys = {}

export type Poly = {
	floor: BasePart,
	verts: {Vector3},
	classes: {string},    -- class of edge i -> i+1; "internal" = a diagonal
	area: number,
	fatness: number,
}
export type Config = {
	convexEps: number?, collinearEps: number?, minFatness: number?,
}

local DEFAULT = {
	-- A reflex corner shallower than this is treated as flat. Boundary lines
	-- arrive exact, so this only absorbs float noise from the projection.
	convexEps = 1e-4,
	-- Merging often leaves a vertex sitting mid-edge; drop it when the turn is
	-- this small AND the two edges carry the same class, so class boundaries are
	-- never silently erased.
	collinearEps = 1e-3,
	-- Below this, a polygon is reported as a sliver. Reporting only: dropping
	-- them would punch holes in the mesh.
	minFatness = 0.15,
	-- Below this a polygon is a triangulation artifact along a bridge corridor,
	-- not walkable ground.
	degenerateArea = 1e-3,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

type V2 = { x: number, y: number }

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

local function fatnessOf(pts: {V2}): number
	local p = perimOf(pts)
	if p <= 1e-9 then return 0 end
	return 4 * math.pi * math.abs(areaOf(pts)) / (p * p)
end

-- Forward declaration: hole bridging needs the containment test, which is
-- defined with the ear-clipping helpers below.
local pointInTri: (V2, V2, V2, V2) -> boolean

local function pointInPoly2(pts: {V2}, x: number, y: number): boolean
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
-- Hole bridging: splice each inner ring into the outer one along a diagonal,
-- turning a polygon with holes into one simple polygon.
--
-- Standard construction: take the hole's rightmost vertex, shoot +x at the outer
-- ring, and bridge to the endpoint of the edge hit (the vertex of that edge with
-- the larger x, which is visible from the hole by construction). Holes here are
-- obstacle footprints well inside their floor, so the degenerate cases that make
-- this fiddly in general do not arise.
--------------------------------------------------------------------------

type Ring = { pts: {V2}, cls: {string} }

local function bridgeHoles(outer: Ring, holes: {Ring}): Ring
	local ring = { pts = table.clone(outer.pts), cls = table.clone(outer.cls) }
	-- innermost first, so a bridge never has to cross an earlier bridge
	local order = {}
	for i = 1, #holes do order[i] = i end
	table.sort(order, function(a, b)
		local ax, bx = -math.huge, -math.huge
		for _, p in ipairs(holes[a].pts) do ax = math.max(ax, p.x) end
		for _, p in ipairs(holes[b].pts) do bx = math.max(bx, p.x) end
		return ax > bx
	end)

	local doneHole: { [number]: boolean } = {}
	for _, hi in ipairs(order) do
		local h = holes[hi]
		doneHole[hi] = true
		local mi, mx = 1, -math.huge
		for i, p in ipairs(h.pts) do
			if p.x > mx then mx, mi = p.x, i end
		end
		local M = h.pts[mi]

		-- BRIDGE TARGET, CHOSEN BY VERIFICATION RATHER THAN BY RULE.
		--
		-- Two textbook rules were tried here and both picked blocked targets on
		-- the 200x200 floor. The +x ray with a reflex-vertex refinement misses
		-- the zero-width corridors left by earlier bridges, whose corners are
		-- collinear rather than reflex; nearest-visible-vertex instead pinches
		-- against holes already spliced in. Either way the ring self-intersects,
		-- ear clipping stalls, and the region loses its area silently.
		--
		-- So do not predict which bridge is legal — splice it and CHECK. The
		-- resulting ring must be simple, which is cheap to verify at these sizes
		-- (n <= 90) and admits no false positives. Candidates are tried nearest
		-- first, so the accepted bridge is still the short, sensible one.
		local function properCross(p1: V2, p2: V2, p3: V2, p4: V2): boolean
			local d1 = cross2(p2.x - p1.x, p2.y - p1.y, p3.x - p1.x, p3.y - p1.y)
			local d2 = cross2(p2.x - p1.x, p2.y - p1.y, p4.x - p1.x, p4.y - p1.y)
			local d3 = cross2(p4.x - p3.x, p4.y - p3.y, p1.x - p3.x, p1.y - p3.y)
			local d4 = cross2(p4.x - p3.x, p4.y - p3.y, p2.x - p3.x, p2.y - p3.y)
			return ((d1 > 1e-9 and d2 < -1e-9) or (d1 < -1e-9 and d2 > 1e-9))
				and ((d3 > 1e-9 and d4 < -1e-9) or (d3 < -1e-9 and d4 > 1e-9))
		end
		local function isSimple(pts: {V2}): boolean
			local n = #pts
			for i = 1, n do
				local i2 = (i % n) + 1
				for j = i + 1, n do
					local j2 = (j % n) + 1
					if i ~= j and i ~= j2 and i2 ~= j then
						if properCross(pts[i], pts[i2], pts[j], pts[j2]) then return false end
					end
				end
			end
			return true
		end

		local function spliceAt(B: number): Ring
			local np, nc = {}, {}
			for i = 1, B do
				np[#np + 1] = ring.pts[i]
				nc[#nc + 1] = ring.cls[i]
			end
			nc[#nc] = "internal" -- ring[B] -> hole bridge
			for k = 0, #h.pts - 1 do
				local idx = ((mi - 1 + k) % #h.pts) + 1
				np[#np + 1] = h.pts[idx]
				nc[#nc + 1] = h.cls[idx]
			end
			np[#np + 1] = h.pts[mi]
			nc[#nc + 1] = "internal" -- hole -> ring bridge back
			for i = B, #ring.pts do
				np[#np + 1] = ring.pts[i]
				nc[#nc + 1] = ring.cls[i]
			end
			return { pts = np, cls = nc }
		end

		local cand = {}
		for i = 1, #ring.pts do
			local dx, dy = ring.pts[i].x - M.x, ring.pts[i].y - M.y
			cand[#cand + 1] = { i = i, d = dx * dx + dy * dy }
		end
		table.sort(cand, function(a, b) return a.d < b.d end)

		local accepted = nil
		for _, cd in ipairs(cand) do
			local trial = spliceAt(cd.i)
			if isSimple(trial.pts) then accepted = trial; break end
		end
		ring = accepted or spliceAt(cand[1].i)
	end
	return ring
end

--------------------------------------------------------------------------
-- Ear clipping, picking the BEST ear rather than the first.
--
-- The choice matters because it sets the floor on quality: first-ear clipping
-- reliably shaves needle triangles off convex stretches, and while the merge
-- pass can rebuild fat polygons from needles, it cannot always do so without
-- leaving one behind. Scoring ears by their smallest angle costs one pass and
-- starts the merge from a much better place.
--------------------------------------------------------------------------

local function minAngle(a: V2, b: V2, c: V2): number
	local function ang(p: V2, q: V2, r: V2): number
		local ux, uy = p.x - q.x, p.y - q.y
		local vx, vy = r.x - q.x, r.y - q.y
		local lu = math.sqrt(ux * ux + uy * uy)
		local lv = math.sqrt(vx * vx + vy * vy)
		if lu < 1e-12 or lv < 1e-12 then return 0 end
		return math.acos(math.clamp((ux * vx + uy * vy) / (lu * lv), -1, 1))
	end
	return math.min(ang(c, a, b), math.min(ang(a, b, c), ang(b, c, a)))
end

-- STRICTLY inside — a point merely ON the triangle's boundary does not block the
-- ear. This is not a nicety: rim lines get split wherever a boundary edge meets
-- them, so rings are full of vertices lying exactly on a longer straight run. An
-- inclusive test treats every one of those as blocking, no ear is ever found,
-- and triangulation bails after one triangle. That silently lost 49% of the map
-- area, most of it the 200x200 floor.
function pointInTri(p: V2, a: V2, b: V2, c: V2): boolean
	local eps = 1e-9
	local d1 = cross2(b.x - a.x, b.y - a.y, p.x - a.x, p.y - a.y)
	local d2 = cross2(c.x - b.x, c.y - b.y, p.x - b.x, p.y - b.y)
	local d3 = cross2(a.x - c.x, a.y - c.y, p.x - c.x, p.y - c.y)
	return (d1 > eps and d2 > eps and d3 > eps)
		or (d1 < -eps and d2 < -eps and d3 < -eps)
end

-- Returns triangles as triples of indices into `ring.pts`.
local function earClip(ring: Ring, c: any): {{number}}
	local n = #ring.pts
	local idx = {}
	for i = 1, n do idx[i] = i end
	if areaOf(ring.pts) < 0 then
		local r = {}
		for i = n, 1, -1 do r[#r + 1] = idx[i] end
		idx = r
	end

	local tris: {{number}} = {}
	local guard = 0
	while #idx > 3 and guard < 100000 do
		guard += 1
		local bestK, bestScore = nil, -1
		for k = 1, #idx do
			local i0 = idx[((k - 2) % #idx) + 1]
			local i1 = idx[k]
			local i2 = idx[(k % #idx) + 1]
			local a, b, cc = ring.pts[i0], ring.pts[i1], ring.pts[i2]
			-- convex corner?
			if cross2(b.x - a.x, b.y - a.y, cc.x - b.x, cc.y - b.y) > c.convexEps then
				local ok = true
				for _, j in ipairs(idx) do
					if j ~= i0 and j ~= i1 and j ~= i2 then
						if pointInTri(ring.pts[j], a, b, cc) then ok = false; break end
					end
				end
				if ok then
					local s = minAngle(a, b, cc)
					if s > bestScore then bestScore, bestK = s, k end
				end
			end
		end
		if not bestK then break end -- degenerate remainder; keep what we have
		local k = bestK :: number
		local i0 = idx[((k - 2) % #idx) + 1]
		local i1 = idx[k]
		local i2 = idx[(k % #idx) + 1]
		tris[#tris + 1] = { i0, i1, i2 }
		table.remove(idx, k)
	end
	if #idx == 3 then tris[#tris + 1] = { idx[1], idx[2], idx[3] } end
	return tris
end

--------------------------------------------------------------------------
-- Quality-ordered convex merging
--------------------------------------------------------------------------

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

-- Splice two index cycles that share exactly the edge (u, v).
local function spliceCycles(p: {number}, q: {number}, u: number, v: number): {number}?
	local pi, qi = nil, nil
	for i = 1, #p do
		if p[i] == u and p[(i % #p) + 1] == v then pi = i break end
	end
	for i = 1, #q do
		if q[i] == v and q[(i % #q) + 1] == u then qi = i break end
	end
	if not pi or not qi then return nil end
	local out = {}
	-- p from v onward, up to u
	for k = 1, #p - 1 do out[#out + 1] = p[(((pi :: number) % #p) + k - 1) % #p + 1] end
	for k = 1, #q - 1 do out[#out + 1] = q[(((qi :: number) % #q) + k - 1) % #q + 1] end
	return out
end

--------------------------------------------------------------------------

function Polys.fromLoops(lres: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local polys: {Poly} = {}
	local stats = {
		regions = 0, triangles = 0, polys = 0, merges = 0,
		slivers = 0, worstFatness = 1, areaIn = 0, areaOut = 0,
		nonConvex = 0, maxVerts = 0,
		triLoss = 0, triLossRegions = 0, worstTri = {}, degenerate = 0,
		worstConvex = {},
	}

	for _, r in ipairs(lres.regions) do
		stats.regions += 1
		-- Back into the floor's own 2D frame, taken from Loops rather than rebuilt
		-- here. Deriving it from the part's CFrame is wrong for every rotated or
		-- tilted floor, and the failure is silent: the ring projects onto a skewed
		-- plane, ear clipping hits degenerate corners, and roughly half the map's
		-- area quietly failed to become polygons.
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
		stats.areaIn += math.abs(areaOf(outer.pts))
		for _, h in ipairs(holes) do stats.areaIn -= math.abs(areaOf(h.pts)) end

		local ring = (#holes > 0) and bridgeHoles(outer, holes) or outer
		local tris = earClip(ring, c)
		stats.triangles += #tris
		do
			-- Triangulation must reproduce the ring's area. Ear clipping bails on a
			-- degenerate remainder, and that loss is otherwise silent.
			local ta = 0
			for _, t in ipairs(tris) do
				ta += math.abs(areaOf({ ring.pts[t[1]], ring.pts[t[2]], ring.pts[t[3]] }))
			end
			local want = math.abs(areaOf(ring.pts))
			if want - ta > 0.01 then
				stats.triLoss += (want - ta)
				stats.triLossRegions += 1
				if #stats.worstTri < 6 then
					stats.worstTri[#stats.worstTri + 1] = string.format(
						"%s ringVerts=%d holes=%d tris=%d ringArea=%.1f triArea=%.1f lost=%.1f",
						r.floor.Name, #ring.pts, #holes, #tris, want, ta, want - ta)
				end
			end
		end

		-- class of the directed edge (i -> j) where both are ring indices
		local cls: { [string]: string } = {}
		for i = 1, #ring.pts do
			local j = (i % #ring.pts) + 1
			cls[i .. ">" .. j] = ring.cls[i]
		end
		local function classOf(i: number, j: number): string
			return cls[i .. ">" .. j] or "internal"
		end

		-- polygons as index cycles
		local cyc: { {number}? } = {}
		for _, t in ipairs(tris) do cyc[#cyc + 1] = { t[1], t[2], t[3] } end

		local function ptsOf(cy: {number}): {V2}
			local out = {}
			for _, i in ipairs(cy) do out[#out + 1] = ring.pts[i] end
			return out
		end

		-- greedy: repeatedly take the convex merge with the fattest union
		local improving = true
		while improving do
			improving = false
			local bestA, bestB, bestCy, bestScore = nil, nil, nil, -1
			-- shared-edge index
			local owner: { [string]: number } = {}
			for ci, cy in pairs(cyc) do
				if cy then
					for i = 1, #cy do
						local u, v = cy[i], cy[(i % #cy) + 1]
						owner[u .. ">" .. v] = ci
					end
				end
			end
			for ci, cy in pairs(cyc) do
				if cy then
					for i = 1, #cy do
						local u, v = cy[i], cy[(i % #cy) + 1]
						local oj = owner[v .. ">" .. u]
						if oj and oj ~= ci and (cyc[oj] ~= nil) then
							-- never dissolve a real boundary edge
							if classOf(u, v) == "internal" and classOf(v, u) == "internal" then
								local m = spliceCycles(cy, cyc[oj] :: {number}, u, v)
								if m and #m >= 3 then
									local mp = ptsOf(m)
									if isConvex(mp, c) then
										local s = fatnessOf(mp)
										if s > bestScore then
											bestScore, bestA, bestB, bestCy = s, ci, oj, m
										end
									end
								end
							end
						end
					end
				end
			end
			if bestCy then
				cyc[bestA :: number] = bestCy
				cyc[bestB :: number] = nil
				stats.merges += 1
				improving = true
			end
		end

		for _, cy in pairs(cyc) do
			if cy then
				local pts = ptsOf(cy)
				-- drop vertices that merging left mid-edge, but only when both
				-- sides carry the same class, so a class boundary is never erased
				local vs, cs = {}, {}
				for i = 1, #cy do
					local prev = cy[((i - 2) % #cy) + 1]
					local cur = cy[i]
					local nxt = cy[(i % #cy) + 1]
					local a, b, d = ring.pts[prev], ring.pts[cur], ring.pts[nxt]
					local turn = cross2(b.x - a.x, b.y - a.y, d.x - b.x, d.y - b.y)
					local flat = math.abs(turn) <= c.collinearEps
					if not (flat and classOf(prev, cur) == classOf(cur, nxt)) then
						vs[#vs + 1] = unproj(ring.pts[cur])
						cs[#cs + 1] = classOf(cur, nxt)
					end
				end
				-- Bridge corridors are zero-width slits, so triangulation leaves a
				-- few degenerate pieces along them. They carry no area and are not
				-- navmesh; dropping them costs nothing (area conservation is
				-- checked against areaIn either way).
				local fp0 = {}
				for _, i in ipairs(cy) do fp0[#fp0 + 1] = ring.pts[i] end
				if math.abs(areaOf(fp0)) < c.degenerateArea then
					stats.degenerate += 1
				elseif #vs >= 3 then
					local fp = {}
					for _, i in ipairs(cy) do fp[#fp + 1] = ring.pts[i] end
					local a = math.abs(areaOf(fp))
					local f = fatnessOf(fp)
					if not isConvex(fp, c) then
						stats.nonConvex += 1
						if #stats.worstConvex < 4 then
							stats.worstConvex[#stats.worstConvex + 1] = string.format(
								"%s verts=%d area=%.3f fat=%.4f", r.floor.Name, #fp, a, f)
						end
					end
					if f < c.minFatness then stats.slivers += 1 end
					stats.worstFatness = math.min(stats.worstFatness, f)
					stats.areaOut += a
					stats.maxVerts = math.max(stats.maxVerts, #vs)
					polys[#polys + 1] = {
						floor = r.floor, verts = vs, classes = cs, area = a, fatness = f,
					}
					stats.polys += 1
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

-- Each polygon gets its own hue so neighbours are distinguishable, drawn as
-- edge bars; boundary edges keep their class colour, internal diagonals go dim.
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
		sub.Name = string.format("P%d_%s_a%.0f_f%.2f", pi, p.floor.Name, p.area, p.fatness)
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
