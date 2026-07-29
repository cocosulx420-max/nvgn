--!strict
-- NVGN.Bounds — boundary extraction on the part-aligned local grids.
--
-- Supersedes the world-lattice attempt in Boundary2. See DESIGN.md, "Substrate
-- for boundaries": the input is LocalGrid's per-part grids, NOT a world lattice
-- over surfel positions. A world lattice staircases every surface that is not
-- aligned to world X/Z, and nothing downstream recovers from that.
--
-- This module covers the first stage: deciding, for every grid-extent cell edge,
-- which of three things it is.
--
--   SEAM     the floor continues into another grid   -> not a boundary at all
--   WALL     the outward neighbour is a DEAD cell    -> killer names the blocker
--   DROPOFF  the outward neighbour is simply absent  -> floor extent, a ledge
--
-- Classification is free: LocalGrid already attributed every dead cell to the
-- part that killed it, at bake time, with a GetPartsInPart probe. Nothing here
-- re-probes the world.

local Bounds = {}

export type Config = {
	surfaceEps: number?, -- numerical tolerance for "the same surface continues"
}

local DEFAULT = {
	-- There is NO step tolerance here, deliberately. Whether an agent can step up
	-- or drop down a height change is a traversal question, answered by
	-- pathfinding against the finished mesh — not a meshing question. Baking a
	-- 2-stud tolerance into adjacency made the navmesh assert that a 2-stud
	-- ledge was continuous floor, which erased real boundaries: a ramp's side
	-- edge simply evaporated where its drop fell under the threshold.
	--
	-- This epsilon is numerical only: it decides "is this literally the same
	-- surface continuing", absorbing float noise and hairline part overlaps.
	-- Any real height change is a boundary, and pathfinding decides what to do
	-- with it.
	surfaceEps = 0.25,
	-- Extra tolerance on the covering test, beyond half a step. Incommensurate
	-- lattices (two abutting floors at different yaw) never line up exactly, so a
	-- little padding decides seams in favour of "the floor continues". That is
	-- the conservative choice here: a missed seam invents a boundary in the
	-- middle of walkable floor, which fragments the mesh, while a slightly
	-- generous seam only declines to cut where two floors genuinely touch.
	seamPad = 0.08,
	-- Vertical window when matching an outline sample to the floor it borders.
	-- Asymmetric on purpose: a wall standing ON a floor has its underside at the
	-- floor height, while an overhang that killed cells by CLEARANCE sits above
	-- the floor by up to minClearance. So look mostly downward.
	wallDrop = 3.0,
	wallRise = 1.0,
}

local function merged(cfg: Config?)
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

-- Lattice neighbours in a grid's OWN frame.
local NB = {
	{ du = 1, dv = 0 }, { du = -1, dv = 0 },
	{ du = 0, dv = 1 }, { du = 0, dv = -1 },
}

local function key(a: number, b: number): string
	return a .. ":" .. b
end

-- A grid's IN-PLANE axes, tilt included. LocalGrid samples a part on its full
-- local frame, so a 45 degree ramp's u and v lie in the inclined surface and its
-- normal has Y = 0.707. Those axes must be used as they are.
--
-- Flattening them to horizontal was a bug: on a 45 degree ramp the flattened
-- axis is only cos(45) = 0.707 long, so every cell edge came out 0.707 studs
-- instead of 1.0 and left a 0.29 stud gap per cell — a dashed line up every
-- ramp. Keeping the tilt means a run along a slope is a straight sloped
-- segment, at the surface's real angle, with no interpolation and no snapping.
local function axesOf(g: any): (Vector3, Vector3)
	if not g.fallback and g.u and g.v then
		return g.u, g.v
	end
	return Vector3.xAxis, Vector3.zAxis
end

--------------------------------------------------------------------------------
-- World-space cell lookup
--------------------------------------------------------------------------------
-- Grids have different frames, so adjacency between them cannot be a lattice
-- index comparison. It is a proximity relation, which is legitimate to answer
-- with a world hash: this decides CONNECTIVITY, never geometry. Geometry always
-- comes from a grid's own frame.

local function buildHash(localData: any, step: number)
	local hash: { [string]: { any } } = {}
	local entries = {}
	for part, g in pairs(localData.grids) do
		for _, cell in ipairs(g.cells) do
			local e = { cell = cell, grid = g, part = part, id = #entries + 1 }
			entries[#entries + 1] = e
			local k = key(math.floor(cell.pos.X / step), math.floor(cell.pos.Z / step))
			local b = hash[k]
			if not b then b = {}; hash[k] = b end
			b[#b + 1] = e
		end
	end
	return hash, entries
end

-- Does any live cell COVER this world point?
--
-- A cell is not a point, it is a step-by-step tile centred on its sample, lying
-- in its own grid's plane. So the test is containment in that cell's own frame,
-- not a world-axis distance to its centre.
--
-- Getting this wrong is what produced scattered one-cell stubs. Two abutting
-- floors rotated 8 and 24 degrees have incommensurate lattices: a probe from one
-- lands between cell centres of the other, so a world-axis test with 0.35 slack
-- missed it and emitted a spurious boundary in the middle of continuous floor.
-- Half a step in the covering cell's own frame answers correctly at any rotation.
local function coveringCell(hash: any, step: number, p: Vector3,
	down: number, up: number, skip: any, pad: number?): any
	local half = step * 0.5 + (pad or 1e-3)
	local bx, bz = math.floor(p.X / step), math.floor(p.Z / step)
	local best, bestOff = nil, math.huge
	for ox = -1, 1 do
		for oz = -1, 1 do
			local b = hash[key(bx + ox, bz + oz)]
			if b then
				for _, e in ipairs(b) do
					if e ~= skip then
						local g = e.grid
						local d = p - e.cell.pos
						local du, dv, dn
						if not g.fallback and g.u and g.v and g.n then
							du, dv, dn = d:Dot(g.u), d:Dot(g.v), d:Dot(g.n)
						else
							du, dv, dn = d.X, d.Z, d.Y
						end
						if math.abs(du) <= half and math.abs(dv) <= half
							and dn <= down and dn >= -up then
							local off = math.abs(du) + math.abs(dv) + math.abs(dn)
							if off < bestOff then bestOff = off; best = e end
						end
					end
				end
			end
		end
	end
	return best
end

--------------------------------------------------------------------------------
-- Components, spanning grids
--------------------------------------------------------------------------------

function Bounds.components(localData: any, cfg: Config?)
	local c = merged(cfg)
	local step = localData.config.step
	local hash, entries = buildHash(localData, step)

	local parent = table.create(#entries)
	for i = 1, #entries do parent[i] = i end
	local function find(a: number): number
		while parent[a] ~= a do parent[a] = parent[parent[a]]; a = parent[a] end
		return a
	end
	local function union(a: number, b: number)
		a, b = find(a), find(b)
		if a ~= b then parent[b] = a end
	end

	-- Step-tolerance adjacency, tested in world space so it crosses grid frames.
	for _, e in ipairs(entries) do
		local u, v = axesOf(e.grid)
		for _, d in ipairs(NB) do
			local probe = e.cell.pos + u * (d.du * step) + v * (d.dv * step)
			local other = coveringCell(hash, step, probe, c.surfaceEps, c.surfaceEps, e)
			if other then union(e.id, other.id) end
		end
	end

	local comps, byRoot = {}, {}
	for _, e in ipairs(entries) do
		local r = find(e.id)
		local comp = byRoot[r]
		if not comp then
			comp = { entries = {}, id = #comps + 1 }
			byRoot[r] = comp
			comps[#comps + 1] = comp
		end
		comp.entries[#comp.entries + 1] = e
		e.comp = comp
	end

	return comps, hash, entries
end

--------------------------------------------------------------------------------
-- Classify every grid-extent edge
--------------------------------------------------------------------------------

export type Edge = {
	kind: string,      -- "wall" | "dropoff"
	a: Vector3, b: Vector3,
	outDir: Vector3,
	cell: any, grid: any, part: any,
	killer: Instance?, -- wall only: the part that killed the neighbour cell
	comp: any,
}

function Bounds.classify(localData: any, cfg: Config?)
	local c = merged(cfg)
	local step = localData.config.step
	local comps, hash, entries = Bounds.components(localData, c)

	local edges: { Edge } = {}
	local nSeam, nWall, nDrop = 0, 0, 0

	for _, e in ipairs(entries) do
		local g, cell = e.grid, e.cell
		local u, v = axesOf(g)

		for _, d in ipairs(NB) do
			local nk = key(cell.ui + d.du, cell.vi + d.dv)

			-- 1. still inside this grid: interior, nothing to emit
			if g.index[nk] then continue end

			local dead = g.deadIndex[nk]
			local outDir = u * d.du + v * d.dv
			local half = (d.du ~= 0) and v or u
			local mid = cell.pos + outDir * (step * 0.5)
			local a = mid - half * (step * 0.5)
			local b = mid + half * (step * 0.5)

			if dead then
				-- 2. the neighbour sample was killed -> a wall, and we know by what
				edges[#edges + 1] = {
					kind = "wall", a = a, b = b, outDir = outDir,
					cell = cell, grid = g, part = e.part, entryId = e.id,
					du = d.du, dv = d.dv,
					killer = dead.killer, deadPos = dead.pos, comp = e.comp,
				}
				nWall += 1
			else
				-- 3. off this grid entirely. Either the floor continues in another
				-- grid (a part seam, not a boundary) or it really stops (a ledge).
				local probe = cell.pos + outDir * step
				if coveringCell(hash, step, probe, c.surfaceEps, c.surfaceEps, e, c.seamPad) then
					nSeam += 1
				else
					edges[#edges + 1] = {
						kind = "dropoff", a = a, b = b, outDir = outDir,
						cell = cell, grid = g, part = e.part,
						du = d.du, dv = d.dv,
						killer = nil, comp = e.comp,
					}
					nDrop += 1
				end
			end
		end
	end

	local killers: { [Instance]: number } = {}
	local nKillers = 0
	for _, ed in ipairs(edges) do
		if ed.killer then
			if killers[ed.killer] == nil then nKillers += 1; killers[ed.killer] = 0 end
			killers[ed.killer] += 1
		end
	end

	return {
		edges = edges,
		components = comps,
		entries = entries,
		hash = hash,
		config = c,
		stats = {
			cells = #entries,
			components = #comps,
			seams = nSeam,
			wallEdges = nWall,
			dropoffEdges = nDrop,
			distinctKillers = nKillers,
		},
	}
end

--------------------------------------------------------------------------------
-- Merge edges into runs
--------------------------------------------------------------------------------
-- THIS is the step that turns a row of 1-stud cell edges into one straight line,
-- and it is the same trick Footprint.forPart uses for its outline.
--
-- The staircase is not removed by fitting a line through stepped samples. It is
-- never created: because the lattice is aligned to the part, a straight edge
-- occupies exactly ONE lattice row, so every boundary cell in that row shares a
-- row index and differs only in the cross index. Sort by cross, walk while the
-- cross index increments by one, and the whole row collapses to a single
-- segment whose endpoints are the outer corners of the first and last cell.
--
-- A run is broken by anything that would make one segment a lie: a different
-- grid, a different outward direction, a different boundary kind, a different
-- killer, or a different component.

function Bounds.mergeEdges(edges: { any })
	local buckets: { [string]: { any } } = {}

	-- Identify grids and killers by a unique id, never by tostring(instance):
	-- tostring on an Instance yields its NAME, and a map has hundreds of parts
	-- all called "Part". Name collisions merge rows from unrelated grids and
	-- interleave their cross indices, producing runs that span the whole map.
	local gid, kid = {}, {}
	local nG, nK = 0, 0
	local function gridId(g: any): number
		if not gid[g] then nG += 1; gid[g] = nG end
		return gid[g]
	end
	local function killerId(k: Instance?): number
		if k == nil then return 0 end
		if not kid[k] then nK += 1; kid[k] = nK end
		return kid[k]
	end

	for _, e in ipairs(edges) do
		-- row runs perpendicular to the outward direction
		local row, cross
		if e.du ~= 0 then
			row, cross = e.cell.ui, e.cell.vi
		else
			row, cross = e.cell.vi, e.cell.ui
		end
		e._cross = cross
		local k = table.concat({
			gridId(e.grid), e.du, e.dv, row, e.kind,
			killerId(e.killer), e.comp.id,
		}, "|")
		local b = buckets[k]
		if not b then b = {}; buckets[k] = b end
		b[#b + 1] = e
	end

	local runs = {}
	for _, b in pairs(buckets) do
		table.sort(b, function(x, y) return x._cross < y._cross end)
		local i = 1
		while i <= #b do
			local j = i
			while j < #b and b[j + 1]._cross == b[j]._cross + 1 do j += 1 end
			runs[#runs + 1] = {
				kind = b[i].kind,
				source = "floor-frame",
				a = b[i].a, b = b[j].b,
				outDir = b[i].outDir,
				grid = b[i].grid, part = b[i].part,
				killer = b[i].killer, comp = b[i].comp,
				cells = j - i + 1,
				length = (b[j].b - b[i].a).Magnitude,
			}
			i = j + 1
		end
	end
	return runs
end

function Bounds.mergeRuns(result: any)
	local runs = Bounds.mergeEdges(result.edges)
	result.runs = runs

	local wl, dl, wn, dn = 0, 0, 0, 0
	for _, r in ipairs(runs) do
		if r.kind == "wall" then wn += 1; wl += r.length else dn += 1; dl += r.length end
	end
	result.stats.wallRuns = wn
	result.stats.dropoffRuns = dn
	result.stats.wallRunLength = wl
	result.stats.dropoffRunLength = dl
	return runs
end

--------------------------------------------------------------------------------
-- Cut outlines: where a killer actually cuts, in the killer's own frame
--------------------------------------------------------------------------------
-- Footprint gives a part's SILHOUETTE outline. That is the true cut line only
-- when the part's underside sits on the floor, as a wall does. It is wrong for
-- anything tilted or raised: a ramp 15x1x30 whose underside spans Y 0.30 to
-- 10.30 only kills floor where its underside closes to within minClearance, so
-- the frontier is a clearance CONTOUR running across the silhouette, nowhere
-- near its edge. Matching floor edges against the silhouette then fails and
-- everything falls back to the jagged floor lattice.
--
-- So select, in the cutter's own lattice, the footprint cells that actually cut,
-- and trace the boundary of THAT set. Same row-collapse as before, so the result
-- is still straight by construction — a ramp's cut line comes out as one clean
-- segment across its width. For a wall on a floor the cutting set is the whole
-- silhouette and this reduces to the original outline.

-- Exact version. Every wall edge already names its killer, and the dead cell it
-- faces has a world position, so the footprint cell responsible is found by a
-- coordinate lookup — not by probing outward and hoping.
--
-- One wall edge -> exactly one footprint boundary segment, in the cutter's frame.
-- No sweeps, no duplicates, no misses, and no dedupe pass needed: a stretch is
-- emitted once because exactly one set of edges maps to it.
function Bounds.cutOutlinesExact(footData: any, localData: any, result: any, cfg: Config?)
	local c = merged(cfg)
	local step = localData.config.step

	-- foot cells, hashed by world XZ, per killer
	local footHash = {}
	for part, foot in pairs(footData.foots) do
		local h = {}
		for _, fc in ipairs(foot.cells) do
			local k = key(math.floor(fc.bottom.X / step), math.floor(fc.bottom.Z / step))
			local b = h[k]
			if not b then b = {}; h[k] = b end
			b[#b + 1] = fc
		end
		footHash[part] = h
	end

	local dirs = {
		{ du = 1, dv = 0 }, { du = -1, dv = 0 },
		{ du = 0, dv = 1 }, { du = 0, dv = -1 },
	}

	-- (part, dir, row) -> list of {cross, footCell, floorY}
	local buckets, nMapped, nUnmapped = {}, 0, 0
	local mapped = {}

	for _, e in ipairs(result.edges) do
		if e.kind ~= "wall" or not e.killer or not e.deadPos then continue end
		local foot = footData.foots[e.killer]
		if not foot then continue end

		-- the footprint cell covering the dead sample
		local h = footHash[e.killer]
		local bx, bz = math.floor(e.deadPos.X / step), math.floor(e.deadPos.Z / step)
		local best, bestD = nil, math.huge
		for ox = -1, 1 do
			for oz = -1, 1 do
				local b = h[key(bx + ox, bz + oz)]
				if b then
					for _, fc in ipairs(b) do
						local dx = fc.bottom.X - e.deadPos.X
						local dz = fc.bottom.Z - e.deadPos.Z
						local d2 = dx * dx + dz * dz
						if d2 < bestD then bestD = d2; best = fc end
					end
				end
			end
		end
		if not best or bestD > (step * 1.1) ^ 2 then
			nUnmapped += 1
			continue
		end

		-- the outline faces back the way the wall edge came from
		local want = -e.outDir
		local bd, bdot = nil, -math.huge
		for _, dd in ipairs(dirs) do
			local wdir = foot.u * dd.du + foot.v * dd.dv
			local dot = wdir:Dot(want)
			if dot > bdot then bdot = dot; bd = dd end
		end

		local row, cross
		if bd.du ~= 0 then row, cross = best.ui, best.vi else row, cross = best.vi, best.ui end
		-- The footprint lattice is 2D, but one XZ column can cut floors at several
		-- heights: a tall wall cuts the ground AND a balcony above it, and both map
		-- to the same foot cell. Without the level in the key those edges share a
		-- bucket, and a run then stretches from one level to the other — that is
		-- what produced 251-stud walls and 2091 studs of wall from 1895 edges.
		local level = math.floor(e.cell.pos.Y * 2 + 0.5)
		local bk = ("%s|%d|%d|%d|%d"):format(tostring(e.killer), bd.du, bd.dv, row, level)
		local bucket = buckets[bk]
		if not bucket then
			bucket = { part = e.killer, foot = foot, dd = bd, items = {} }
			buckets[bk] = bucket
		end
		bucket.items[#bucket.items + 1] = { cross = cross, fc = best, floorY = e.cell.pos.Y, comp = e.comp }
		mapped[e] = true
		nMapped += 1
	end

	-- collapse each row into runs
	local out = {}
	for _, bucket in pairs(buckets) do
		local foot, dd = bucket.foot, bucket.dd
		local wdir = foot.u * dd.du + foot.v * dd.dv
		local half = (dd.du ~= 0) and foot.v or foot.u
		local seen, items = {}, {}
		for _, it in ipairs(bucket.items) do
			if not seen[it.cross] then seen[it.cross] = true; items[#items + 1] = it end
		end
		table.sort(items, function(x, y) return x.cross < y.cross end)

		-- Strict adjacency. Bridging gaps in the cross index was tried and is not
		-- safe: allowing a skip of two took total wall length to 2111 studs from
		-- 1760 one-stud edges — ~350 studs of wall no edge asked for — and the
		-- longest run reached 259 studs by bridging openings. Sealing a doorway is
		-- not an acceptable kind of conservative.
		--
		-- The fragmentation that tolerance was hiding is real and belongs
		-- elsewhere: the floor and cutter lattices are not commensurate, so
		-- consecutive floor edges sometimes skip a foot cell. That is a resampling
		-- problem, not something to paper over by inventing wall.
		local i = 1
		while i <= #items do
			local j = i
			while j < #items and items[j + 1].cross == items[j].cross + 1 do j += 1 end
			local ca, cb = items[i], items[j]
			local pa = Vector3.new(ca.fc.bottom.X, ca.floorY, ca.fc.bottom.Z)
				+ wdir * (step * 0.5) - half * (step * 0.5)
			local pb = Vector3.new(cb.fc.bottom.X, cb.floorY, cb.fc.bottom.Z)
				+ wdir * (step * 0.5) + half * (step * 0.5)
			out[#out + 1] = {
				kind = "wall", source = "footprint",
				a = pa, b = pb, outDir = wdir,
				part = bucket.part, killer = bucket.part,
				comp = ca.comp,
				cells = j - i + 1,
				crossSpan = cb.cross - ca.cross + 1,
				length = (pb - pa).Magnitude,
			}
			i = j + 1
		end
	end

	result.stats.edgesMapped = nMapped
	result.stats.edgesUnmapped = nUnmapped
	result.stats.cutOutlineRuns = #out
	return out, mapped
end

function Bounds.cutOutlines(footData: any, localData: any, result: any, cutBy: any, cfg: Config?)
	local c = merged(cfg)
	local step = localData.config.step
	local minClear = localData.config.minClearance or 1.5
	local hash = result.hash

	-- floor height under a point: consider live cells AND dead ones, since a dead
	-- cell is still floor, just floor this part killed
	local floorHash = {}
	local function addFloor(pos: Vector3, entry: any)
		local k = key(math.floor(pos.X / step), math.floor(pos.Z / step))
		local b = floorHash[k]
		if not b then b = {}; floorHash[k] = b end
		b[#b + 1] = { pos = pos, entry = entry }
	end
	for _, e in ipairs(result.entries) do addFloor(e.cell.pos, e) end
	for _, g in pairs(localData.grids) do
		for _, d in ipairs(g.dead) do addFloor(d.pos, nil) end
	end

	local function floorUnder(p: Vector3)
		local bx, bz = math.floor(p.X / step), math.floor(p.Z / step)
		local bestY, best = nil, nil
		for ox = -1, 1 do
			for oz = -1, 1 do
				local b = floorHash[key(bx + ox, bz + oz)]
				if b then
					for _, f in ipairs(b) do
						if math.abs(f.pos.X - p.X) <= step * 0.5 + c.seamPad
							and math.abs(f.pos.Z - p.Z) <= step * 0.5 + c.seamPad
							and f.pos.Y <= p.Y + 0.1 then
							if bestY == nil or f.pos.Y > bestY then bestY = f.pos.Y; best = f end
						end
					end
				end
			end
		end
		return bestY, best
	end

	local dirs = {
		{ du = 1, dv = 0 }, { du = -1, dv = 0 },
		{ du = 0, dv = 1 }, { du = 0, dv = -1 },
	}

	local outlines, covered = {}, {}
	local nCut, nRuns = 0, 0

	for part, foot in pairs(footData.foots) do
		-- 1. which footprint cells actually cut the floor beneath them
		local cutting = {}
		for _, fc in ipairs(foot.cells) do
			local fy = floorUnder(fc.bottom)
			if fy and (fc.bottom.Y - fy) < minClear then
				cutting[key(fc.ui, fc.vi)] = { fc = fc, floorY = fy }
				nCut += 1
			end
		end

		-- 2. trace the boundary of the cutting set, in this part's lattice
		for _, dd in ipairs(dirs) do
			local wdir = foot.u * dd.du + foot.v * dd.dv
			local half = (dd.du ~= 0) and foot.v or foot.u
			local rows = {}
			for _, cell in pairs(cutting) do
				if not cutting[key(cell.fc.ui + dd.du, cell.fc.vi + dd.dv)] then
					-- Only a boundary if WALKABLE floor lies just outside, and that
					-- floor was cut by this very part. The cut set's boundary also
					-- runs where the floor outside is itself dead, or absent
					-- entirely — emitting those invents wall along lines no agent
					-- could ever stand next to.
					local at = Vector3.new(cell.fc.bottom.X, cell.floorY, cell.fc.bottom.Z)
					-- The cut set and the floor's dead frontier live in different
					-- lattices, so the floor cell just outside is not reliably one
					-- step away. Sweep outward a little rather than sampling a
					-- single point, or coverage is patchy and everything it misses
					-- falls back to the jagged floor frame.
					-- Only that walkable floor lies outside. Requiring the probe to
					-- also resolve back to this exact killer was too strict: where
					-- it failed the wall was dropped entirely, and since fallback
					-- now only covers footprint-less killers, a miss here is a hole
					-- rather than a jagged line. If the cell cuts and floor lies
					-- outside it, that is a wall regardless of which part the floor
					-- lattice happened to attribute it to.
					local e = nil
					for _, t in ipairs({ 0.6, 1.0, 1.4 }) do
						local hit = coveringCell(hash, step, at + wdir * (step * t),
							c.wallDrop, c.wallRise, nil, c.seamPad)
						if hit then e = hit; break end
					end
					if e then
						covered[e.id .. "|" .. tostring(part)] = true
						local rowK, crossI
						if dd.du ~= 0 then rowK = cell.fc.ui; crossI = cell.fc.vi
						else rowK = cell.fc.vi; crossI = cell.fc.ui end
						local lst = rows[rowK]
						if not lst then lst = {}; rows[rowK] = lst end
						lst[#lst + 1] = { cross = crossI, cell = cell }
					end
				end
			end

			for _, lst in pairs(rows) do
				table.sort(lst, function(x, y) return x.cross < y.cross end)
				local i = 1
				while i <= #lst do
					local j = i
					while j < #lst and lst[j + 1].cross == lst[j].cross + 1 do j += 1 end
					local ca, cb = lst[i].cell, lst[j].cell
					local pa = Vector3.new(ca.fc.bottom.X, ca.floorY, ca.fc.bottom.Z)
						+ wdir * (step * 0.5) - half * (step * 0.5)
					local pb = Vector3.new(cb.fc.bottom.X, cb.floorY, cb.fc.bottom.Z)
						+ wdir * (step * 0.5) + half * (step * 0.5)
					outlines[#outlines + 1] = {
						kind = "wall", source = "footprint",
						a = pa, b = pb, outDir = wdir,
						part = part, killer = part,
						length = (pb - pa).Magnitude,
					}
					nRuns += 1
					i = j + 1
				end
			end
		end
	end

	result.stats.cutCells = nCut
	result.stats.cutOutlineRuns = nRuns
	return outlines, covered
end

--------------------------------------------------------------------------------
-- Wall geometry from footprint outlines
--------------------------------------------------------------------------------
-- A floor's dead-cell frontier is jagged in the FLOOR's lattice, so it is never
-- used as geometry. The part that did the cutting supplies the line instead,
-- already clean in its own yaw frame. Dead cells only SELECT which span of that
-- outline actually borders walkable floor.
--
-- Verticality: outline endpoints ride the killer's underside, which follows a
-- tilted part down its incline, so a ramp's outline is already sloped. The Y we
-- emit is taken from the floor cell each sample matched, not from the outline,
-- so a wall bordering a ramp tracks the ramp rather than sitting at one height.
--
-- Killers with no footprint (unions, meshes — Footprint.build skips non-blocks)
-- keep their floor-frame runs. Dropping them would open a route that does not
-- exist, which is the one error this pipeline must never make.

function Bounds.wallGeometry(result: any, footData: any, localData: any, cfg: Config?)
	local c = merged(cfg)

	-- which killers actually cut each live cell
	local cutBy: { [number]: { [Instance]: boolean } } = {}
	for _, e in ipairs(result.edges) do
		if e.kind == "wall" and e.killer and e.entryId then
			local s = cutBy[e.entryId]
			if not s then s = {}; cutBy[e.entryId] = s end
			s[e.killer] = true
		end
	end

	-- Geometry comes from the cut outlines: the boundary of the set of footprint
	-- cells that genuinely close to within minClearance of the floor, traced in
	-- the cutter's lattice. Straight by construction, and correct for tilted and
	-- raised parts where the silhouette is not the cut line.
	local out, mapped = Bounds.cutOutlinesExact(footData, localData, result, c)
	local nSpans = #out

	-- A wall edge falls back ONLY when its killer has no footprint at all.
	--
	-- Coverage used to be decided by probing outward from each cut cell and
	-- seeing whether it landed on the right floor cell. That is a heuristic, and
	-- where it missed, the edge was emitted TWICE: once as a clean cut-outline
	-- run and again as a jagged floor-frame run lying on top of it. Both defects
	-- in the report — the leftover staircase along stair and block bases, and the
	-- doubled line — are that same miss.
	--
	-- The cut outline is derived from the same minClearance rule LocalGrid used
	-- to kill the cell in the first place, so if a killer has a footprint, its
	-- cut contour is by construction the geometry for every edge it caused.
	-- Whether a probe happened to land is irrelevant.
	-- An edge falls back when it was not mapped onto a footprint: no killer, no
	-- footprint for that killer, or a dead sample that landed outside the
	-- footprint lattice. Keyed off the actual mapping rather than a guess about
	-- which edges the mapping should have caught.
	local leftover, dropoffEdges = {}, {}
	for _, e in ipairs(result.edges) do
		if e.kind == "wall" then
			if not mapped[e] then leftover[#leftover + 1] = e end
		else
			dropoffEdges[#dropoffEdges + 1] = e
		end
	end
	local fallback = Bounds.mergeEdges(leftover)

	local newRuns = Bounds.mergeEdges(dropoffEdges)
	for _, r in ipairs(out) do newRuns[#newRuns + 1] = r end
	for _, r in ipairs(fallback) do newRuns[#newRuns + 1] = r end
	result.runs = newRuns

	local fpLen, fbLen = 0, 0
	for _, r in ipairs(out) do fpLen += r.length end
	for _, r in ipairs(fallback) do fbLen += r.length end

	result.stats.wallFromFootprint = #out
	result.stats.wallFromFloorFrame = #fallback
	result.stats.wallFootprintLength = fpLen
	result.stats.wallFallbackLength = fbLen
	return result
end

--------------------------------------------------------------------------------
-- Deduplicate collinear wall runs
--------------------------------------------------------------------------------
-- Several killers can cut the same stretch of floor — a wall and the step above
-- it, two stacked parts, a block abutting a stair — and each one's cut contour
-- then emits its own run along the same line. The debug draw showed this as one
-- bright line with others buried inside it: three 30-stud runs stacked at one
-- spot, two 35-stud runs at another.
--
-- Picking a single "owner" per edge was what the old probe filter tried to do,
-- and it failed either by dropping walls or by duplicating them. Emitting freely
-- and then merging is robust: group runs that lie on the same line, then union
-- their intervals along it. Genuinely disjoint stretches on one line — either
-- side of a doorway — stay separate, because their intervals do not overlap.

function Bounds.dedupeWalls(result: any)
	local Q = 0.25
	local function q(v: number): number
		return math.floor(v / Q + 0.5)
	end

	local groups, others = {}, {}
	for _, run in ipairs(result.runs) do
		local d = run.b - run.a
		local len = d.Magnitude
		if run.kind ~= "wall" or run.source == "stitch" or len < 1e-4 then
			others[#others + 1] = run
			continue
		end
		local t = d / len
		-- canonical direction, so a run and its reverse land in one group
		if t.X < -1e-6 or (math.abs(t.X) <= 1e-6 and t.Z < -1e-6) then
			t = -t
		end
		local n = run.outDir
		local nl = n.Magnitude
		n = (nl > 1e-6) and (n / nl) or Vector3.xAxis
		local k = ("%d,%d,%d,%d,%d"):format(
			q(t.X * 4), q(t.Z * 4), q(run.a:Dot(n)), q(run.a.Y), q(n.X * 4))
		local g = groups[k]
		if not g then g = { t = t, runs = {} }; groups[k] = g end
		g.runs[#g.runs + 1] = run
	end

	local removed = 0
	for _, g in pairs(groups) do
		-- parameterise every run in the group along the group's shared direction
		local iv = {}
		for _, run in ipairs(g.runs) do
			local s1, s2 = run.a:Dot(g.t), run.b:Dot(g.t)
			if s1 > s2 then s1, s2 = s2, s1 end
			iv[#iv + 1] = { s1 = s1, s2 = s2, run = run }
		end
		table.sort(iv, function(x, y) return x.s1 < y.s1 end)

		local cur = nil
		local function flush()
			if not cur then return end
			local base = cur.run
			-- rebuild the endpoints from the unioned interval, keeping the
			-- perpendicular offset of the run we are extending
			local origin = base.a - g.t * (base.a:Dot(g.t))
			base.a = origin + g.t * cur.s1
			base.b = origin + g.t * cur.s2
			base.length = cur.s2 - cur.s1
			others[#others + 1] = base
			cur = nil
		end

		for _, e in ipairs(iv) do
			if cur and e.s1 <= cur.s2 + 1e-3 then
				if e.s2 > cur.s2 then cur.s2 = e.s2 end
				removed += 1
			else
				flush()
				cur = { s1 = e.s1, s2 = e.s2, run = e.run }
			end
		end
		flush()
	end

	result.runs = others
	result.stats.wallDuplicatesMerged = removed
	return result
end

--------------------------------------------------------------------------------
-- Stitch open ends
--------------------------------------------------------------------------------
-- Two partially overlapping grids at different yaw produce boundary chains built
-- from incommensurate lattices. Where they meet, the chains jog past each other
-- instead of meeting at a point, so a run can end a stud or two from its
-- continuation with no defect in either chain.
--
-- Close those with an explicit connector. This is safe in the only direction
-- that matters: a connector ADDS boundary, and adding boundary can never open a
-- route that does not exist. It can only decline to open one — the conservative
-- error the design asks for.
--
-- Gaps wider than maxGap are NOT stitched. Those are real defects and should be
-- reported rather than papered over.

function Bounds.stitch(result: any, maxGap: number?)
	local lim = maxGap or 2.5
	local eps = 0.05
	local function pk(p: Vector3): string
		return ("%d,%d,%d"):format(
			math.floor(p.X / eps + 0.5), math.floor(p.Y / eps + 0.5), math.floor(p.Z / eps + 0.5))
	end

	-- odd-degree endpoints are the open ones
	local deg, rep, owner = {}, {}, {}
	for _, r in ipairs(result.runs) do
		for _, p in ipairs({ r.a, r.b }) do
			local k = pk(p)
			deg[k] = (deg[k] or 0) + 1
			rep[k] = p
			owner[k] = r
		end
	end
	local open = {}
	for k, d in pairs(deg) do
		if d == 1 then open[#open + 1] = { k = k, p = rep[k], run = owner[k] } end
	end

	-- greedy nearest pairing
	local cand = {}
	for i = 1, #open do
		for j = i + 1, #open do
			local d = (open[i].p - open[j].p).Magnitude
			if d <= lim and d > 1e-6 then
				cand[#cand + 1] = { i = i, j = j, d = d }
			end
		end
	end
	table.sort(cand, function(x, y) return x.d < y.d end)

	local used, added = {}, 0
	for _, c in ipairs(cand) do
		if not used[c.i] and not used[c.j] then
			used[c.i], used[c.j] = true, true
			local A, B = open[c.i], open[c.j]
			result.runs[#result.runs + 1] = {
				kind = A.run.kind == "wall" and B.run.kind == "wall" and "wall" or "dropoff",
				source = "stitch",
				a = A.p, b = B.p,
				outDir = A.run.outDir,
				comp = A.run.comp,
				length = c.d,
			}
			added += 1
		end
	end

	-- Second pass: T-junctions. A chain often ends partway ALONG another run
	-- rather than at its endpoint — a flat floor's boundary meeting a tilted
	-- ramp's boundary, for instance. Endpoint pairing cannot see that, because
	-- the continuation is interior to the other run. Split the target run at the
	-- projection and connect there.
	local tees = 0
	for i = 1, #open do
		if not used[i] then
			local A = open[i]
			local bestRun, bestT, bestD = nil, 0, lim
			for _, run in ipairs(result.runs) do
				if run ~= A.run and run.source ~= "stitch" then
					local ab = run.b - run.a
					local L2 = ab:Dot(ab)
					if L2 > 1e-6 then
						local t = math.clamp((A.p - run.a):Dot(ab) / L2, 0, 1)
						-- interior only; endpoints were the first pass's job
						if t > 0.02 and t < 0.98 then
							local proj = run.a + ab * t
							local d = (proj - A.p).Magnitude
							if d < bestD then bestD = d; bestRun = run; bestT = t end
						end
					end
				end
			end
			if bestRun then
				local ab = bestRun.b - bestRun.a
				local proj = bestRun.a + ab * bestT
				-- split the target so the junction becomes a shared vertex
				local tail = {
					kind = bestRun.kind, source = bestRun.source,
					a = proj, b = bestRun.b, outDir = bestRun.outDir,
					grid = bestRun.grid, part = bestRun.part,
					killer = bestRun.killer, comp = bestRun.comp,
					length = (bestRun.b - proj).Magnitude,
				}
				bestRun.b = proj
				bestRun.length = (proj - bestRun.a).Magnitude
				result.runs[#result.runs + 1] = tail
				result.runs[#result.runs + 1] = {
					kind = A.run.kind, source = "stitch",
					a = A.p, b = proj, outDir = A.run.outDir,
					comp = A.run.comp, length = bestD,
				}
				used[i] = true
				tees += 1
			end
		end
	end

	local unpaired = 0
	for i = 1, #open do if not used[i] then unpaired += 1 end end

	result.stats.openBeforeStitch = #open
	result.stats.stitched = added
	result.stats.stitchedTees = tees
	result.stats.unstitched = unpaired
	return result
end

--------------------------------------------------------------------------------
-- Audit
--------------------------------------------------------------------------------
-- "Looks right" is not a standard. A boundary is correct when it is watertight:
-- every run endpoint meets another run endpoint, so the boundary closes into
-- loops with no open ends. An open end is a gap — a place the mesh would leak
-- through — and it is the metric to drive to zero before moving on.
--
-- Wall runs come from cutter frames and dropoff runs from floor frames, so the
-- two only meet where a wall abuts a ledge. Closure is therefore reported per
-- kind as well as overall.

function Bounds.audit(result: any, eps: number?)
	local e = eps or 0.05
	local function pk(p: Vector3): string
		return ("%d,%d,%d"):format(
			math.floor(p.X / e + 0.5), math.floor(p.Y / e + 0.5), math.floor(p.Z / e + 0.5))
	end

	local function closure(runs: { any })
		local deg: { [string]: number } = {}
		local rep: { [string]: Vector3 } = {}
		for _, r in ipairs(runs) do
			for _, p in ipairs({ r.a, r.b }) do
				local k = pk(p)
				deg[k] = (deg[k] or 0) + 1
				rep[k] = p
			end
		end
		-- An open end is a vertex only ONE run touches. Odd degree is the wrong
		-- test: a T-junction, where a chain meets the middle of another, is a
		-- perfectly closed vertex of degree 3.
		local open, openPts = 0, {}
		for k, d in pairs(deg) do
			if d == 1 then
				open += 1
				if #openPts < 12 then openPts[#openPts + 1] = rep[k] end
			end
		end
		return open, openPts, deg
	end

	local wall, drop = {}, {}
	for _, r in ipairs(result.runs) do
		if r.kind == "wall" then wall[#wall + 1] = r else drop[#drop + 1] = r end
	end

	local dOpen, dPts = closure(drop)
	local wOpen, wPts = closure(wall)
	local aOpen, aPts = closure(result.runs)

	result.audit = {
		dropoffRuns = #drop, dropoffOpenEnds = dOpen, dropoffOpenPoints = dPts,
		wallRuns = #wall, wallOpenEnds = wOpen, wallOpenPoints = wPts,
		allOpenEnds = aOpen, allOpenPoints = aPts,
	}
	return result.audit
end

--------------------------------------------------------------------------------
-- Debug draw
--------------------------------------------------------------------------------

function Bounds.visualize(result: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Bounds")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Bounds"; folder.Parent = dbg
	local fW = Instance.new("Folder"); fW.Name = "Wall"; fW.Parent = folder
	local fD = Instance.new("Folder"); fD.Name = "Dropoff"; fD.Parent = folder

	local WALL = Color3.fromRGB(70, 170, 255)      -- wall, from a footprint outline
	local WALL_FB = Color3.fromRGB(255, 150, 40)   -- wall, floor-frame fallback
	local DROP = Color3.fromRGB(120, 255, 130)

	-- draw merged runs when they exist, raw cell edges otherwise
	local items = result.runs or result.edges
	for _, e in ipairs(items) do
		local d = e.b - e.a
		local len = d.Magnitude
		if len > 1e-3 then
			local bar = Instance.new("Part")
			bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
			bar.Size = Vector3.new(len, 0.14, 0.14)
			if e.kind ~= "wall" then
				bar.Color = DROP
			elseif e.source == "floor-frame" then
				bar.Color = WALL_FB
			else
				bar.Color = WALL
			end
			bar.Material = Enum.Material.Neon
			bar.CFrame = CFrame.fromMatrix((e.a + e.b) * 0.5 + Vector3.new(0, 0.25, 0),
				d / len, Vector3.yAxis)
			bar.Parent = (e.kind == "wall") and fW or fD
		end
	end
	return folder
end

return Bounds
