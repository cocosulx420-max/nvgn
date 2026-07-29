--!strict
-- NVGN.Boundary2 — boundary extraction, new method (see DESIGN.md).
--
--   1. label connected components (step-tolerance adjacency)
--   2. Euclidean distance transform, per component
--   3. trace contours from surfel extent, per component
--   4. greedy line fit (TLS, max perpendicular deviation) + inward bias
--   5. offset inward by the agent radius, miter limit with bevel fallback
--   6. graded narrow-region offset + mandatory union-find severance check
--
-- Reads FloorData and nothing else. Zero raycasts, zero physics queries — so it
-- iterates against a cached surfel snapshot rather than behind a full bake.
-- The old face-projection module (NVGN.Boundary) is left untouched for comparison.

local Boundary2 = {}

export type Config = {
	radius: number?,        -- agent radius r; the offset budget
	fitTol: number?,        -- max perpendicular deviation of a fitted segment
	stepTol: number?,       -- height difference below which two cells connect
	narrowMargin: number?,  -- margin in clamp(halfWidth - margin, 0, r)
	miterLimit: number?,    -- miter length / r before falling back to a bevel
	minSegLen: number?,     -- segments shorter than this are merge candidates
	collinearDeg: number?,  -- adjacent segments within this angle are merged
	probeCap: number?,      -- how far inward the half-width probe marches
	offsetBand: number?,    -- offset quantisation; controls how often the offset steps
	minRunLen: number?,     -- cells; shorter offset runs are absorbed by a neighbour
	dropoffOutset: number?, -- how far OUTWARD a dropoff edge sits from the cell centre
	probeHeights: { number }?, -- heights above the floor sampled to test for a blocker
}

local DEFAULT = {
	radius = 1.5,
	fitTol = 1.0,
	stepTol = 2.0,
	narrowMargin = 0.25,
	miterLimit = 4.0,
	minSegLen = 2.0,
	collinearDeg = 8.0,
	probeCap = 12.0,
	offsetBand = 0.5,
	minRunLen = 4,
	dropoffOutset = 0.5,
	probeHeights = { 1.0, 2.0, 3.0 },
}

-- Boundary direction bits, in the order sides are emitted by the tracer.
local DIR_PX, DIR_NX, DIR_PZ, DIR_NZ = 1, 2, 4, 8

local INF = 1e20
local CELL_HALF = 0.5 -- a cell centre sits this far inward of the surface it hugs

local function merged(cfg: Config?)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function ckey(x: number, z: number): string
	return x .. ":" .. z
end

--------------------------------------------------------------------------------
-- 0. Wall vs dropoff classification
--------------------------------------------------------------------------------
-- A boundary is not one thing. A WALL boundary has something solid behind it: the
-- agent's body cannot occupy that space, so the boundary must be offset inward by
-- the agent radius. A DROPOFF boundary — a rooftop, ledge, cliff, or stair head —
-- has nothing behind it but air. Offsetting it inward is actively wrong: it walls
-- the agent off from the very edge it needs to reach in order to drop down.
--
-- The two are told apart by sampling, never by reading a face: for each of the
-- four neighbours, ask the SVO whether anything is solid in that column at torso
-- height. Something there means wall; nothing means dropoff. This answers
-- uniformly for a Block, a Union, a MeshPart, or a pile of rubble.
--
-- Sampling starts a stud above the floor because the SVO is conservative by
-- construction — surface voxels are marked solid, so the floor itself inflates
-- upward by up to one leaf and would read as a blocker at the surface.
--
-- Must run at bake time, while the SVO still exists; the result is carried in the
-- snapshot so the boundary stage itself stays pure arithmetic.
function Boundary2.classifyBlocked(floorData: any, tree: any, cfg: Config?)
	local c = merged(cfg)
	local dirs = {
		{ 1, 0, DIR_PX }, { -1, 0, DIR_NX },
		{ 0, 1, DIR_PZ }, { 0, -1, DIR_NZ },
	}
	for _, s in ipairs(floorData.surfels) do
		local ix, iz = math.floor(s.pos.X), math.floor(s.pos.Z)
		local mask = 0
		for _, d in ipairs(dirs) do
			local nx, nz = ix + d[1] + 0.5, iz + d[2] + 0.5
			for _, h in ipairs(c.probeHeights) do
				if tree:isSolid(Vector3.new(nx, s.pos.Y + h, nz)) then
					mask += d[3]
					break
				end
			end
		end
		s.blocked = mask
	end
	return floorData
end

--------------------------------------------------------------------------------
-- 1. Connected components
--------------------------------------------------------------------------------
-- Two neighbouring cells connect iff their height difference is below the step
-- tolerance — exactly the relation that decides whether an agent can step
-- between them, so a component is precisely a mutually reachable set. This is
-- what makes a balcony a separate component from the floor beneath it, and what
-- makes a spiral ramp fall out as one component that never self-adjoins.

local NB4 = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } }

function Boundary2.components(floorData: any, cfg: Config?)
	local c = merged(cfg)
	local surfels = floorData.surfels
	local nodes = table.create(#surfels)
	local grid: { [string]: { number } } = {}

	for i, s in ipairs(surfels) do
		local ix, iz = math.floor(s.pos.X), math.floor(s.pos.Z)
		nodes[i] = { ix = ix, iz = iz, y = s.pos.Y, s = s, blocked = s.blocked or 0 }
		local k = ckey(ix, iz)
		local b = grid[k]
		if not b then b = {}; grid[k] = b end
		b[#b + 1] = i
	end

	local parent = table.create(#nodes)
	for i = 1, #nodes do parent[i] = i end
	local function find(a: number): number
		while parent[a] ~= a do
			parent[a] = parent[parent[a]]
			a = parent[a]
		end
		return a
	end
	local function union(a: number, b: number)
		a, b = find(a), find(b)
		if a ~= b then parent[b] = a end
	end

	for i, nd in ipairs(nodes) do
		for _, d in ipairs(NB4) do
			local b = grid[ckey(nd.ix + d[1], nd.iz + d[2])]
			if b then
				for _, j in ipairs(b) do
					if math.abs(nodes[j].y - nd.y) <= c.stepTol then union(i, j) end
				end
			end
		end
	end

	local comps, byRoot = {}, {}
	for i, nd in ipairs(nodes) do
		local r = find(i)
		local comp = byRoot[r]
		if not comp then
			comp = { nodes = {}, cells = {}, id = #comps + 1 }
			byRoot[r] = comp
			comps[#comps + 1] = comp
		end
		comp.nodes[#comp.nodes + 1] = nd
		local k = ckey(nd.ix, nd.iz)
		-- a component holds at most one surfel per XZ cell; a second surfel in the
		-- same cell within step tolerance is a near-duplicate surface, not a level
		if comp.cells[k] == nil then comp.cells[k] = nd end
	end

	return comps
end

--------------------------------------------------------------------------------
-- 2. Euclidean distance transform (Felzenszwalb), per component
--------------------------------------------------------------------------------
-- A true EDT, not a 4- or 8-neighbour structuring element: those give a diamond
-- or square kernel, so the offset comes out short on the diagonals or long on
-- them. The offset has to be circular.

local function edt1d(f: { number }, n: number): { number }
	local d, v, z = {}, {}, {}
	local allInf = true
	for q = 0, n - 1 do
		if f[q] < INF then allInf = false; break end
	end
	if allInf then
		for q = 0, n - 1 do d[q] = INF end
		return d
	end

	local k = 0
	v[0] = 0; z[0] = -INF; z[1] = INF
	for q = 1, n - 1 do
		local s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
		while k > 0 and s <= z[k] do
			k -= 1
			s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
		end
		if s <= z[k] and k == 0 then
			v[0] = q; z[0] = -INF; z[1] = INF
		else
			k += 1
			v[k] = q; z[k] = s; z[k + 1] = INF
		end
	end

	k = 0
	for q = 0, n - 1 do
		while z[k + 1] < q do k += 1 end
		local dq = q - v[k]
		d[q] = dq * dq + f[v[k]]
	end
	return d
end

-- Distance from each component cell to the nearest cell outside the component,
-- expressed as clearance from the cell centre to that surface (hence -CELL_HALF,
-- so a cell hugging a wall reports 0.5 rather than 1).
function Boundary2.distanceField(comp: any)
	local minx, minz, maxx, maxz = math.huge, math.huge, -math.huge, -math.huge
	for _, nd in pairs(comp.cells) do
		if nd.ix < minx then minx = nd.ix end
		if nd.ix > maxx then maxx = nd.ix end
		if nd.iz < minz then minz = nd.iz end
		if nd.iz > maxz then maxz = nd.iz end
	end
	-- pad by one so the outside ring exists and edge cells get a finite distance
	minx -= 1; minz -= 1; maxx += 1; maxz += 1
	local W, H = maxx - minx + 1, maxz - minz + 1

	local g = {}
	for x = 0, W - 1 do
		local col = {}
		for z = 0, H - 1 do
			col[z] = comp.cells[ckey(x + minx, z + minz)] and INF or 0
		end
		g[x] = col
	end

	for x = 0, W - 1 do g[x] = edt1d(g[x], H) end
	local row, out = {}, {}
	for z = 0, H - 1 do
		for x = 0, W - 1 do row[x] = g[x][z] end
		local d = edt1d(row, W)
		for x = 0, W - 1 do
			out[ckey(x + minx, z + minz)] = math.max(math.sqrt(d[x]) - CELL_HALF, 0)
		end
	end

	comp.D = out
	comp.bounds = { minx = minx, minz = minz, maxx = maxx, maxz = maxz }
	return out
end

--------------------------------------------------------------------------------
-- 3. Contour tracing
--------------------------------------------------------------------------------
-- Traced from the surfel extent — where the component's cells end — not from
-- SVO solid voxels, which are conservative and inflated by up to one leaf. This
-- captures both boundary sources uniformly: a cell adjacent to a wall, and a
-- cell at a rooftop or cliff edge where the floor just stops.
--
-- Crack following rather than Moore tracing: walk the cell-edge cracks between
-- occupied and empty cells. Every loop closes exactly, holes come out as their
-- own loops, and each edge remembers the cell that owns it.

function Boundary2.traceLoops(comp: any)
	local cells = comp.cells
	local function has(x: number, z: number): boolean
		return cells[ckey(x, z)] ~= nil
	end

	local edges: { [string]: { any } } = {}
	local function addEdge(ax, az, bx, bz, cx, cz, dir, blocked)
		local k = ax .. "," .. az
		local b = edges[k]
		if not b then b = {}; edges[k] = b end
		b[#b + 1] = {
			ax = ax, az = az, bx = bx, bz = bz, cx = cx, cz = cz,
			dir = dir,
			-- the side this edge faces has something solid behind it
			wall = blocked % (dir * 2) >= dir,
			used = false,
		}
	end

	-- wound so the interior is always on the left of travel
	for _, nd in pairs(cells) do
		local x, z, b = nd.ix, nd.iz, nd.blocked or 0
		if not has(x, z - 1) then addEdge(x, z, x + 1, z, x, z, DIR_NZ, b) end
		if not has(x + 1, z) then addEdge(x + 1, z, x + 1, z + 1, x, z, DIR_PX, b) end
		if not has(x, z + 1) then addEdge(x + 1, z + 1, x, z + 1, x, z, DIR_PZ, b) end
		if not has(x - 1, z) then addEdge(x, z + 1, x, z, x, z, DIR_NX, b) end
	end

	local loops = {}
	for _, list in pairs(edges) do
		for _, seed in ipairs(list) do
			if not seed.used then
				local loop = {}
				local cur = seed
				while cur and not cur.used do
					cur.used = true
					loop[#loop + 1] = cur
					local cand = edges[cur.bx .. "," .. cur.bz]
					local nxt = nil
					if cand then
						local dx, dz = cur.bx - cur.ax, cur.bz - cur.az
						local best = -math.huge
						for _, e in ipairs(cand) do
							if not e.used then
								local ex, ez = e.bx - e.ax, e.bz - e.az
								-- at a diagonal pinch, take the most clockwise turn so
								-- the loop stays on one side instead of cutting across
								local score = math.atan2(-(dx * ez - dz * ex), dx * ex + dz * ez)
								if score > best then best = score; nxt = e end
							end
						end
					end
					cur = nxt
				end
				if #loop >= 4 then loops[#loops + 1] = loop end
			end
		end
	end

	comp.loops = loops
	return loops
end

--------------------------------------------------------------------------------
-- 4. Greedy line fit + inward bias
--------------------------------------------------------------------------------
-- Corners are where the fit fails — a byproduct of segmentation, not something
-- detected from geometry. Total least squares, because edges can run
-- near-vertical in XZ and OLS is unstable there.

local function tlsLine(pts: { any })
	local n = #pts
	local sx, sz = 0, 0
	for _, p in ipairs(pts) do sx += p.x; sz += p.z end
	local cx, cz = sx / n, sz / n
	local sxx, szz, sxz = 0, 0, 0
	for _, p in ipairs(pts) do
		local dx, dz = p.x - cx, p.z - cz
		sxx += dx * dx; szz += dz * dz; sxz += dx * dz
	end
	local theta = 0.5 * math.atan2(2 * sxz, sxx - szz)
	local dx, dz = math.cos(theta), math.sin(theta)
	return { cx = cx, cz = cz, dx = dx, dz = dz, nx = -dz, nz = dx }
end

local function maxDeviation(pts: { any }, L: any): number
	local m = 0
	for _, p in ipairs(pts) do
		local d = math.abs((p.x - L.cx) * L.nx + (p.z - L.cz) * L.nz)
		if d > m then m = d end
	end
	return m
end

-- Points to fit are the centres of the cells owning each boundary edge. The
-- cells hugging the wall are included: they define where the wall is, and
-- excluding them degrades the fit. The offset happens after.
local function loopPoints(loop: { any })
	local pts = {}
	for _, e in ipairs(loop) do
		local p = {
			x = e.cx + 0.5,
			z = e.cz + 0.5,
			ix = e.cx,
			iz = e.cz,
			-- inward is the left of travel, by the winding above
			inx = -(e.bz - e.az),
			inz = (e.bx - e.ax),
			wall = e.wall,
		}
		local last = pts[#pts]
		-- a corner cell owns two edges that may differ in kind, so identical cells
		-- collapse only when they also agree on wall vs dropoff
		if not last or last.ix ~= p.ix or last.iz ~= p.iz or last.wall ~= p.wall then
			pts[#pts + 1] = p
		end
	end
	return pts
end

function Boundary2.fitSegments(pts: { any }, cfg: Config?)
	local c = merged(cfg)
	local segs, i, n = {}, 1, #pts

	while i <= n do
		local kind = pts[i].wall
		local acc = { pts[i] }
		local j = i + 1
		while j <= n do
			-- never fit across a wall/dropoff transition: the two are offset by
			-- different rules, so a mixed segment could not be offset correctly
			-- either way
			if pts[j].wall ~= kind then break end
			acc[#acc + 1] = pts[j]
			local L = tlsLine(acc)
			-- maximum, not average: an average lets a shallow corner hide inside a
			-- long run
			if maxDeviation(acc, L) > c.fitTol then
				acc[#acc] = nil
				break
			end
			j += 1
		end

		if #acc >= 2 then
			segs[#segs + 1] = { pts = acc, line = tlsLine(acc), wall = kind }
			i += #acc -- restart from the cell that failed the fit
		else
			segs[#segs + 1] = { pts = acc, line = nil, wall = kind }
			i += 1
		end
	end

	for _, s in ipairs(segs) do
		if s.line then Boundary2.orientInward(s) end
	end
	return segs
end

-- Point the line's normal at the walkable interior.
function Boundary2.orientInward(s: any)
	local ax, az = 0, 0
	for _, p in ipairs(s.pts) do ax += p.inx; az += p.inz end
	if s.line.nx * ax + s.line.nz * az < 0 then
		s.line.nx = -s.line.nx
		s.line.nz = -s.line.nz
	end
end

-- Translate the line inward until it never sits outward of an accepted cell
-- centre. This is what makes the clearance guarantee exact instead of budgeted:
-- the true surface is at least CELL_HALF outward of every centre, so a line at
-- or inward of every centre is already at least CELL_HALF inward of the surface
-- before the agent-radius offset is applied on top.
--
-- Applied once, after merging — biasing a line and then re-biasing a merged
-- version of it would shift the boundary inward twice.
function Boundary2.biasInward(s: any)
	local maxT = 0
	for _, p in ipairs(s.pts) do
		local t = (p.x - s.line.cx) * s.line.nx + (p.z - s.line.cz) * s.line.nz
		if t > maxT then maxT = t end
	end
	s.bias = maxT
	s.line.cx += s.line.nx * maxT
	s.line.cz += s.line.nz * maxT
end

local function segLength(s: any): number
	local L, lo, hi = s.line, math.huge, -math.huge
	for _, p in ipairs(s.pts) do
		local u = (p.x - L.cx) * L.dx + (p.z - L.cz) * L.dz
		if u < lo then lo = u end
		if u > hi then hi = u end
	end
	return hi - lo
end

-- A rounded wall segments into many short pieces: correct, but poly-heavy. Merge
-- near-collinear neighbours before offsetting, so short segments do not reach the
-- corner-by-intersection step where adjacent offsets barely intersect and corner
-- positions go unstable.
function Boundary2.mergeCollinear(segs: { any }, cfg: Config?)
	local c = merged(cfg)
	local cosLimit = math.cos(math.rad(c.collinearDeg))
	local out = {}
	for _, s in ipairs(segs) do
		local prev = out[#out]
		if prev and prev.line and s.line and prev.wall == s.wall then
			local dot = math.abs(prev.line.dx * s.line.dx + prev.line.dz * s.line.dz)
			local short = segLength(s) < c.minSegLen
			if dot >= cosLimit or short then
				local pts = table.clone(prev.pts)
				table.move(s.pts, 1, #s.pts, #pts + 1, pts)
				local L = tlsLine(pts)
				if maxDeviation(pts, L) <= c.fitTol then
					prev.pts = pts
					prev.line = L
					continue
				end
			end
		end
		out[#out + 1] = s
	end
	for _, s in ipairs(out) do
		if s.line then Boundary2.orientInward(s) end
	end
	return out
end

--------------------------------------------------------------------------------
-- 5/6. Offset, graded by local half-width, with a miter limit
--------------------------------------------------------------------------------
-- NOTE (deviation from DESIGN.md as written): the doc says to grade the offset by
-- "local D along the segment being offset". D at a boundary cell is always ~0.5
-- by construction — that is what it means to be a boundary cell — so grading on
-- it would collapse every offset to zero. The quantity that actually varies is
-- how thick the region is behind the segment, so we march inward from each cell
-- sampling D and take the maximum: the local half-width. A component holding both
-- a wide hall and a narrow passage therefore still offsets the hall fully.

local function halfWidthAt(comp: any, px: number, pz: number, nx: number, nz: number, cap: number): number
	local best = 0
	local t = 0
	while t <= cap do
		local x, z = math.floor(px + nx * t), math.floor(pz + nz * t)
		local d = comp.D[ckey(x, z)]
		if d == nil or comp.cells[ckey(x, z)] == nil then break end
		if d > best then best = d end
		t += 0.5
	end
	return best
end

-- Half-width is measured per cell, and a segment is SPLIT where it changes band
-- rather than collapsed to its minimum. Taking the minimum over a whole segment
-- reproduces the very failure the grading exists to prevent, one level down: a
-- single doorway pinch anywhere along a 196-cell wall would drag the entire wall
-- into the narrow branch and rob the wide stretch of its offset. Sub-segments
-- keep the parent's direction, so the wall stays straight; only the offset steps.
function Boundary2.offsetSegments(comp: any, segs: { any }, cfg: Config?)
	local c = merged(cfg)
	local band = math.max(c.offsetBand, 0.05)
	local out = {}

	for _, s in ipairs(segs) do
		if not s.line then
			out[#out + 1] = s
			continue
		end

		local L = s.line

		-- A dropoff has nothing solid behind it, so there is no body to keep clear
		-- of: the agent must be able to stand at the real edge and step off. Place
		-- the boundary at the floor's true extent (CELL_HALF outward of the cell
		-- centres) with no inward bias and no radius offset. Eroding here is what
		-- walls NPCs off from ledges they are supposed to drop from.
		if not s.wall then
			local sub = {
				pts = s.pts,
				wall = false,
				line = { cx = L.cx, cz = L.cz, dx = L.dx, dz = L.dz, nx = L.nx, nz = L.nz },
				halfWidth = math.huge,
				offset = -c.dropoffOutset,
				narrow = false,
				dropoff = true,
			}
			sub.offsetLine = {
				cx = sub.line.cx + sub.line.nx * sub.offset,
				cz = sub.line.cz + sub.line.nz * sub.offset,
				dx = L.dx, dz = L.dz, nx = L.nx, nz = L.nz,
			}
			out[#out + 1] = sub
			continue
		end

		local hw = table.create(#s.pts)
		for i, p in ipairs(s.pts) do
			hw[i] = halfWidthAt(comp, p.x, p.z, L.nx, L.nz, c.probeCap)
		end

		-- Smooth the half-width signal with a running MINIMUM over a window before
		-- banding. Half-width wobbles cell to cell, so banding it raw chops a
		-- straight wall into confetti — every flip costs an offset step and two
		-- vertices. A running min is conservative (never offsets further than a
		-- nearby cell allows) and, unlike absorbing short runs into a neighbour,
		-- its influence is bounded by the window: a single pinch widens to the
		-- window instead of dragging the whole wall down to its offset.
		local w = math.max(math.floor(c.minRunLen / 2), 1)
		local smooth = table.create(#hw)
		for i = 1, #hw do
			local m = hw[i]
			for j = math.max(1, i - w), math.min(#hw, i + w) do
				if hw[j] < m then m = hw[j] end
			end
			smooth[i] = m
		end
		hw = smooth

		-- Group consecutive cells into offset bands. Quantise into band INDICES
		-- and split when the index changes; comparing clamped offsets against a
		-- running minimum instead lets a run whose min is exactly one band below
		-- the cap absorb every wide cell after it, which silently reinstates the
		-- whole-segment minimum this split exists to avoid.
		local function bandOf(h: number): number
			return math.floor(math.clamp(h - c.narrowMargin, 0, c.radius) / band + 1e-9)
		end

		local runs, cur = {}, { i0 = 1, min = hw[1], band = bandOf(hw[1]) }
		for i = 2, #hw do
			if bandOf(hw[i]) ~= cur.band then
				cur.i1 = i - 1
				runs[#runs + 1] = cur
				cur = { i0 = i, min = hw[i], band = bandOf(hw[i]) }
			elseif hw[i] < cur.min then
				cur.min = hw[i]
			end
		end
		cur.i1 = #hw
		runs[#runs + 1] = cur

		-- The running min above already guarantees runs are at least window-sized,
		-- so only degenerate single-cell runs are left to absorb.
		if #runs > 1 then
			local packed = { runs[1] }
			for i = 2, #runs do
				local rn = runs[i]
				local prev = packed[#packed]
				if (rn.i1 - rn.i0 + 1) < 2 then
					prev.i1 = rn.i1
					prev.min = math.min(prev.min, rn.min)
				else
					packed[#packed + 1] = rn
				end
			end
			runs = packed
		end

		for _, r in ipairs(runs) do
			local pts = {}
			for i = r.i0, r.i1 do pts[#pts + 1] = s.pts[i] end
			if #pts == 0 then continue end
			-- same direction as the parent fit, own centroid: collinear, not wobbly
			local sub = {
				pts = pts,
				wall = true,
				line = { cx = 0, cz = 0, dx = L.dx, dz = L.dz, nx = L.nx, nz = L.nz },
				parent = s,
			}
			local sx, sz = 0, 0
			for _, p in ipairs(pts) do sx += p.x; sz += p.z end
			sub.line.cx, sub.line.cz = sx / #pts, sz / #pts
			Boundary2.biasInward(sub)

			sub.halfWidth = r.min
			-- continuous in half-width: no jump at a threshold, so neighbouring
			-- runs straddling it still meet cleanly
			sub.offset = math.clamp(r.min - c.narrowMargin, 0, c.radius)
			sub.narrow = sub.offset < c.radius - 1e-6
			sub.width = 2 * r.min
			sub.offsetLine = {
				cx = sub.line.cx + sub.line.nx * sub.offset,
				cz = sub.line.cz + sub.line.nz * sub.offset,
				dx = sub.line.dx, dz = sub.line.dz,
				nx = sub.line.nx, nz = sub.line.nz,
			}
			out[#out + 1] = sub
		end
	end

	return out
end

local function intersect(A: any, B: any)
	local den = A.dx * B.dz - A.dz * B.dx
	if math.abs(den) < 1e-9 then return nil end
	local ex, ez = B.cx - A.cx, B.cz - A.cz
	local t = (ex * B.dz - ez * B.dx) / den
	return { x = A.cx + A.dx * t, z = A.cz + A.dz * t }
end

-- The miter join costs nothing extra to compute — corners were already found by
-- intersecting adjacent fitted lines. But it is not free geometrically: at an
-- acute corner the two offset lines intersect arbitrarily far out. Past the
-- limit, cut the corner with a bevel instead of extending to the intersection.
function Boundary2.buildPolygon(segs: { any }, cfg: Config?)
	local c = merged(cfg)
	local lines = {}
	for _, s in ipairs(segs) do
		if s.offsetLine then lines[#lines + 1] = s end
	end
	if #lines < 3 then return nil end

	local verts, bevels, steps = {}, 0, 0
	local limit = c.miterLimit * math.max(c.radius, 1e-3)

	for i = 1, #lines do
		local a = lines[i]
		local b = lines[(i % #lines) + 1]
		local p = intersect(a.offsetLine, b.offsetLine)
		local anchor = a.pts[#a.pts]
		local spiked = false
		if p then
			local dx, dz = p.x - anchor.x, p.z - anchor.z
			if math.sqrt(dx * dx + dz * dz) > limit then p = nil; spiked = true end
		end
		if p then
			verts[#verts + 1] = p
		else
			-- parallel offset lines are a band step, not an acute corner: the two
			-- runs share a direction and differ only in offset. Counted apart so
			-- the miter limit's real firing rate stays visible.
			if spiked then bevels += 1 else steps += 1 end
			-- bevel: end of a's offset line, then start of b's
			local ea = a.pts[#a.pts]
			local sb = b.pts[1]
			verts[#verts + 1] = {
				x = ea.x + a.offsetLine.nx * a.offset,
				z = ea.z + a.offsetLine.nz * a.offset,
			}
			verts[#verts + 1] = {
				x = sb.x + b.offsetLine.nx * b.offset,
				z = sb.z + b.offsetLine.nz * b.offset,
			}
		end
	end

	return { verts = verts, bevels = bevels, steps = steps }
end

--------------------------------------------------------------------------------
-- Severance check (mandatory)
--------------------------------------------------------------------------------
-- Erosion only ever removes walkable cells, so it cannot invent connectivity —
-- its error direction is always "slightly less walkable than reality". What it
-- CAN do is disconnect the map, and nothing else in the pipeline notices. This is
-- the only severance detector, so it is a required stage rather than a debug aid.

function Boundary2.severanceCheck(comp: any, cfg: Config?)
	local c = merged(cfg)
	local kept = {}
	local n = 0
	for k, nd in pairs(comp.cells) do
		if (comp.D[k] or 0) >= c.radius then kept[k] = nd; n += 1 end
	end

	local seen, pieces = {}, 0
	for k, nd in pairs(kept) do
		if not seen[k] then
			pieces += 1
			local stack = { nd }
			seen[k] = true
			while #stack > 0 do
				local cur = table.remove(stack)
				for _, d in ipairs(NB4) do
					local nk = ckey(cur.ix + d[1], cur.iz + d[2])
					local nn = kept[nk]
					if nn and not seen[nk] and math.abs(nn.y - cur.y) <= c.stepTol then
						seen[nk] = true
						stack[#stack + 1] = nn
					end
				end
			end
		end
	end

	local total = 0
	for _ in pairs(comp.cells) do total += 1 end
	return {
		cellsBefore = total,
		cellsAfter = n,
		piecesAfter = pieces,
		severed = pieces > 1,
		erasedEntirely = n == 0,
	}
end

--------------------------------------------------------------------------------
-- Driver
--------------------------------------------------------------------------------

function Boundary2.build(floorData: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()

	local comps = Boundary2.components(floorData, c)
	local tComp = os.clock()

	local tEdt, tTrace, tFit, tOff = 0, 0, 0, 0
	local regions = {}

	for _, comp in ipairs(comps) do
		local a = os.clock()
		Boundary2.distanceField(comp)
		local b = os.clock(); tEdt += b - a

		local loops = Boundary2.traceLoops(comp)
		local d = os.clock(); tTrace += d - b

		comp.severance = Boundary2.severanceCheck(comp, c)

		for _, loop in ipairs(loops) do
			local pts = loopPoints(loop)
			if #pts >= 3 then
				local e = os.clock()
				local segs = Boundary2.mergeCollinear(Boundary2.fitSegments(pts, c), c)
				local f = os.clock(); tFit += f - e

				segs = Boundary2.offsetSegments(comp, segs, c)
				local poly = Boundary2.buildPolygon(segs, c)
				tOff += os.clock() - f

				regions[#regions + 1] = {
					comp = comp,
					segs = segs,
					poly = poly,
					cellCount = #pts,
				}
			end
		end
	end

	local stats = {
		components = #comps,
		regions = #regions,
		surfels = #floorData.surfels,
		timeTotal = os.clock() - t0,
		timeComponents = tComp - t0,
		timeEDT = tEdt,
		timeTrace = tTrace,
		timeFit = tFit,
		timeOffset = tOff,
	}

	local segs, narrow, bevels, severed, steps = 0, 0, 0, 0, 0
	local wallSegs, dropSegs, wallLen, dropLen = 0, 0, 0, 0
	for _, r in ipairs(regions) do
		for _, s in ipairs(r.segs) do
			if s.line then
				segs += 1
				if s.narrow then narrow += 1 end
				local len = segLength(s)
				if s.wall then
					wallSegs += 1; wallLen += len
				else
					dropSegs += 1; dropLen += len
				end
			end
		end
		if r.poly then bevels += r.poly.bevels; steps += r.poly.steps end
	end
	stats.bandSteps = steps
	for _, comp in ipairs(comps) do
		if comp.severance.severed then severed += 1 end
	end
	stats.segments = segs
	stats.narrowSegments = narrow
	stats.wallSegments = wallSegs
	stats.dropoffSegments = dropSegs
	stats.wallLength = wallLen
	stats.dropoffLength = dropLen
	stats.bevelCorners = bevels
	stats.severedComponents = severed

	return { components = comps, regions = regions, stats = stats, config = c }
end

--------------------------------------------------------------------------------
-- Snapshot harness
--------------------------------------------------------------------------------
-- Boundary extraction reads FloorData and nothing else, so it does not have to
-- sit behind the full bake. Serialize the surfel field once and iterate against
-- the cache: seconds per iteration, no raycasts, no SVO.

-- v2 carries the per-surfel blocked mask, because wall vs dropoff can only be
-- decided while the SVO exists and the boundary stage must not need it.
function Boundary2.snapshot(floorData: any): string
	local buf = { "NVGNSURF2", tostring(#floorData.surfels) }
	for _, s in ipairs(floorData.surfels) do
		buf[#buf + 1] = string.format("%.3f %.3f %.3f %.2f %d",
			s.pos.X, s.pos.Y, s.pos.Z, s.clearance, s.blocked or 0)
	end
	return table.concat(buf, "\n")
end

function Boundary2.loadSnapshot(text: string)
	local lines = string.split(text, "\n")
	local v = lines[1]
	assert(v == "NVGNSURF1" or v == "NVGNSURF2", "not a surfel snapshot")
	local surfels, index = {}, {}
	for i = 3, #lines do
		local l = lines[i]
		if l ~= "" then
			local x, y, z, cl, bl = string.match(l, "(%S+) (%S+) (%S+) (%S+) ?(%S*)")
			local s = {
				pos = Vector3.new(tonumber(x), tonumber(y), tonumber(z)),
				clearance = tonumber(cl),
				blocked = tonumber(bl) or 0,
			}
			surfels[#surfels + 1] = s
			local k = string.format("%d:%d", math.floor(s.pos.X), math.floor(s.pos.Z))
			local b = index[k]
			if not b then b = {}; index[k] = b end
			b[#b + 1] = s
		end
	end
	return { surfels = surfels, index = index }
end

return Boundary2
