--!strict
-- NVGN.Boundary — exact boundary segments from part geometry
-- Stage 1: construction (fromLocalGrid) · Stage 2: classification (classify)
--
-- Edges are NOT traced from surfels (1-stud staircase); they are computed as
-- plane/plane intersections, clipped to real face rectangles:
--   face: {blocking part's face plane} ∩ {floor part's top plane}, clipped to
--         both rectangles -> one clean 2-vertex segment per face/floor pair.
--         The rectangle clip encodes "the part crosses the standing surface":
--         a bridge overhead or a column buried below clips to empty, a wall
--         piercing THROUGH the floor lands exactly where it crosses the top.
--   rim:  the floor's own side faces against its own top plane -> its 4 top
--         edges (dropoff candidates / snap targets). Same construction, B == F.
-- Classification (wall vs seam vs dropoff) is a LATER stage; every segment
-- carries outDir (on-plane, toward the blocked/off-floor side) for its probes.
-- Non-block blockers (unions/meshes) have no planar faces: counted + skipped
-- here, handled by the surfel-seeded path later.
--
-- EXPOSURE TRIMMING: a plane/rect intersection can be real yet unstandable —
-- an interior slab embedded through a wall crosses the wall's OUTER face at
-- mid-height, and a partially buried part's edges continue through solid.
-- A boundary edge must bound walkable surface, so every raw span is sampled
-- against its floor's LocalGrid (the clearance-validated exposure oracle) and
-- only grid-supported runs are emitted. The grid only SEEDS each cut — the
-- endpoint is then snapped exactly onto the occluding part's face plane
-- (grid decides where, geometry decides exactly where), so vertices stay on
-- real geometry. Fully unsupported spans (phantom mid-wall edges) vanish;
-- cuts with no part occluder (e.g. terrain) keep the sampled position.

local Boundary = {}

export type Segment = {
	a: Vector3, b: Vector3, -- endpoints, exactly on the floor's top plane (world)
	floor: BasePart,        -- the walkable surface this edge lies on
	source: BasePart,       -- the part whose face produced it (== floor for rim)
	kind: string,           -- "face" | "rim"
	outDir: Vector3,        -- on-plane unit dir toward the blocked / off-floor side
	class: string?,         -- after classify(): "wall" | "seam" | "dropoff" | "internal"
	other: BasePart?,       -- seam: the part the floor continues onto; wall: blocker
}

export type Config = {
	eps: number?, minLen: number?, parallelEps: number?, sampleOff: number?,
	trimStep: number?, snapRadius: number?, stepUp: number?, probeOff: number?,
	classStep: number?, minClearance: number?, terrainCap: number?,
	maxSlope: number?, gapProbeOff: number?, flushEps: number?, clipSeamEps: number?,
	seamEps: number?, hopProbeOff: number?, weldTol: number?, steepEntry: number?, maxExtend: number?,
	keepInternal: boolean?, minRunLen: number?, minSeamLen: number?, minFragLen: number?,
}

local DEFAULT = {
	eps = 0.05,          -- slack on the blocker-face clip: flush contact (wall ON
	                     -- floor) intersects exactly at the rectangle edge
	minLen = 0.05,       -- drop degenerate slivers
	parallelEps = 0.0012,-- |n x m|^2 below this = face parallel to floor top, skip
	sampleOff = 0.6,     -- near-side offset for exposure sampling
	trimStep = 0.25,     -- sampling pitch along a span for support runs
	snapRadius = 2,      -- max distance a cut may snap to an occluder face plane
	-- classification (stage 2)
	stepUp = 2,          -- vertical half-window the far-side rays search; NOT a
	                     -- traversal rule (step traversal is the pathfinder's
	                     -- job — the bake only records continuity vs wall/drop)
	probeOff = 0.6,      -- how far beyond the edge the far side is probed
	classStep = 0.5,     -- sampling pitch for class runs along a segment
	minClearance = 1.5,  -- a far-side surfel must be standable (crawl floor)
	terrainCap = 20,     -- terrain blocked-side ray length
	maxSlope = 65,       -- a continuation surface steeper than this is not
	                     -- standable (ClipRamps exempt)
	gapProbeOff = 1.2,   -- second, farther perpendicular tap: authored
	                     -- sub-stud cracks between abutting parts must read
	                     -- as continuation, never as a dropoff pair
	flushEps = 0.15,     -- continuation at |dy| below this = coplanar flush
	                     -- contact -> "internal", not a navigation edge (a
	                     -- seam is a real lip you traverse).
	seamEps = 0.3,       -- seam = near-flush CONTINUITY only (clipramp joins,
	                     -- small authored discrepancies between neighbouring
	                     -- floors). Step-up/-down traversal is decided by the
	                     -- pathfinder per agent, not baked: a step reads wall
	                     -- from below / dropoff from above (Cocosulx's rule).
	hopProbeOff = 2.0,   -- farthest rim tap: a small authored hop (gap up to
	                     -- ~2, well under any agent's diameter) is walked
	                     -- across, not a dropoff pair; far taps must prove the
	                     -- path clear so they can never see through a fence
	weldTol = 0.75,      -- endpoints of any classes within this distance weld
	                     -- to their shared centroid so chains close (covers a
	                     -- piercing sheet's rim tips ~0.7 from the corner)
	steepEntry = 20,     -- a fold changing slope by more than this (deg) is a
	                     -- real transition edge (seam); gentler creases stay
	                     -- internal. Surfaces steeper than this also behave
	                     -- like clipramps: entry at the ends only, never
	                     -- through a side/end face — no name magic required.
	maxExtend = 2.5,     -- dangling ends continue along their own line up to
	                     -- this far to meet a transverse edge (trimming can
	                     -- stop an edge short where the floor strip beside it
	                     -- dies, e.g. a wall base beside a descending sheet)
	keepInternal = false,-- internal contacts are detected + counted but NOT
	                     -- emitted: they are weld-stage bookkeeping, not
	                     -- navigation edges. Flip on if a later stage wants
	                     -- them as explicit constraints.
	-- edge-size hygiene
	minRunLen = 1.2,     -- class runs shorter than this are probe noise:
	                     -- ABSORBED into their longer neighbour, never deleted
	                     -- (a hole in the chain is worse than a slightly wrong
	                     -- class over half a stud)
	minSeamLen = 1.2,    -- a seam narrower than ~2x the smallest agent radius
	                     -- is an unusable portal by definition: not an edge
	minFragLen = 0.4,    -- whole wall/dropoff pieces below this adopt the class
	                     -- of an adjacent collinear edge on the same floor
	                     -- (they close loop corners, so keep them, only
	                     -- harmonise the class)
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

local function isBlock(p: BasePart): boolean
	return p:IsA("Part") and p.Shape == Enum.PartType.Block
end

local function isClip(p: Instance): boolean
	return p.Name:find("ClipRamp") ~= nil
end

-- The three oriented axes of a block part with their half-extents.
local function partAxes(p: BasePart)
	local cf = p.CFrame
	local sz = p.Size
	return {
		{ dir = cf.RightVector, ext = sz.X * 0.5 },
		{ dir = cf.UpVector,    ext = sz.Y * 0.5 },
		{ dir = cf.RightVector:Cross(cf.UpVector), ext = sz.Z * 0.5 },
	}
end

-- Clip the line p0 + t*d against the slab |(x - c) . e| <= ext + eps.
-- Returns the narrowed [t0, t1]; t0 > t1 means fully rejected.
local function clipAxis(t0: number, t1: number, p0: Vector3, d: Vector3, c: Vector3, e: Vector3, ext: number, eps: number): (number, number)
	local g = d:Dot(e)
	local h = (p0 - c):Dot(e)
	local lo, hi = -ext - eps, ext + eps
	if math.abs(g) < 1e-9 then
		if h < lo or h > hi then return 1, 0 end
		return t0, t1
	end
	local ta = (lo - h) / g
	local tb = (hi - h) / g
	if ta > tb then ta, tb = tb, ta end
	return math.max(t0, ta), math.min(t1, tb)
end

-- Live-cell lookup with ±1-cell tolerance (cells right at a contact line may
-- be clearance-killed, e.g. under a clipramp sheet, so neighbours count).
local function cellNear(g: any, p: Vector3): boolean
	local du = p - g.origin
	local ui = math.floor(du:Dot(g.u) / g.step)
	local vi = math.floor(du:Dot(g.v) / g.step)
	for a = -1, 1 do
		for b = -1, 1 do
			if g.index[string.format("%d:%d", ui + a, vi + b)] then return true end
		end
	end
	return false
end

-- Exact OBB containment for a block part, with slack.
local function obbContains(axes: any, center: Vector3, p: Vector3, slack: number): boolean
	local dp = p - center
	for i = 1, 3 do
		if math.abs(dp:Dot(axes[i].dir)) > axes[i].ext + slack then return false end
	end
	return true
end

-- Snap a sampled cut at parameter tc onto the face plane of whichever nearby
-- block contains a probe 0.5 into the sgnU side (plus a lateral offset `off`,
-- e.g. toward the far side when classifying). Returns tc when no occluder.
local function snapToOccluder(near: {BasePart}, axCache: any, p0: Vector3, d: Vector3, tc: number, sgnU: number, off: Vector3, radius: number): number
	local pu = p0 + d * (tc + sgnU * 0.5) + off
	local bestT, bestD = tc, radius
	for _, O in ipairs(near) do
		local oa = axCache[O]
		if not oa then oa = partAxes(O); axCache[O] = oa end
		if obbContains(oa, O.Position, pu, 0.25) then
			for fi = 1, 3 do
				for sgn = -1, 1, 2 do
					local mf = oa[fi].dir * sgn
					local denom = d:Dot(mf)
					if math.abs(denom) > 1e-6 then
						local tp = ((O.Position + mf * oa[fi].ext) - p0):Dot(mf) / denom
						local dist = math.abs(tp - tc)
						if dist < bestD then bestD = dist; bestT = tp end
					end
				end
			end
		end
	end
	return bestT
end

-- Trim the raw span [t0,t1] to its grid-supported runs and emit them.
-- Returns how many sub-segments were emitted (0 = fully phantom).
local function trimEmit(segments: {Segment}, c: any, g: any, near: {BasePart}, axCache: any, p0: Vector3, d: Vector3, t0: number, t1: number, w: Vector3, part: BasePart, B: BasePart, isRim: boolean): number
	local len = t1 - t0
	local nS = math.max(2, math.ceil(len / c.trimStep))
	local sup = table.create(nS)
	local any = false
	for i = 1, nS do
		local t = t0 + len * (i - 0.5) / nS
		local s = cellNear(g, p0 + d * t - w * c.sampleOff)
		sup[i] = s
		any = any or s
	end
	if not any then return 0 end

	local function snapCut(tc: number, sgnU: number): number
		return snapToOccluder(near, axCache, p0, d, tc, sgnU, Vector3.zero, c.snapRadius)
	end

	local nEmit = 0
	local i = 1
	while i <= nS do
		if not sup[i] then i += 1; continue end
		local j = i
		while j < nS and sup[j + 1] do j += 1 end
		local ta = (i == 1) and t0 or math.max(t0, snapCut(t0 + len * (i - 1) / nS, -1))
		local tb = (j == nS) and t1 or math.min(t1, snapCut(t0 + len * j / nS, 1))
		if tb - ta >= c.minLen then
			segments[#segments + 1] = {
				a = p0 + d * ta, b = p0 + d * tb,
				floor = part, source = B,
				kind = isRim and "rim" or "face",
				outDir = w,
			}
			nEmit += 1
		end
		i = j + 1
	end
	return nEmit
end

-- Construct all boundary segments for the block floors in localData.
function Boundary.fromLocalGrid(localData: any, parts: {BasePart}, cfg: Config?)
	local c = merged(cfg)
	local segments: {Segment} = {}
	local nFloors, nSkippedFloors, nNonBlock, nDropped = 0, 0, 0, 0

	local blockers: {BasePart} = {}
	for _, p in ipairs(parts) do
		if isBlock(p) then blockers[#blockers + 1] = p else nNonBlock += 1 end
	end

	for part, g in pairs(localData.grids) do
		if g.fallback or not g.n or #g.cells == 0 then nSkippedFloors += 1; continue end
		nFloors += 1
		local n: Vector3 = g.n
		local axes = partAxes(part)
		local ni, best = 2, -math.huge
		for i, a in ipairs(axes) do
			local dd = math.abs(a.dir:Dot(n))
			if dd > best then best = dd; ni = i end
		end
		local c0 = part.Position + n * axes[ni].ext
		local fu = axes[(ni % 3) + 1]
		local fv = axes[((ni + 1) % 3) + 1]
		local rF = part.Size.Magnitude * 0.5

		-- broad-phase once per floor: nearby foreign blocks (also the snap
		-- occluder candidates), plus the floor itself for its rim faces
		local near: {BasePart} = {}
		for _, Bb in ipairs(blockers) do
			if Bb ~= part and (Bb.Position - c0).Magnitude <= rF + Bb.Size.Magnitude * 0.5 + 1 then
				near[#near + 1] = Bb
			end
		end
		local candidates = table.clone(near)
		candidates[#candidates + 1] = part
		local axCache = {}

		for _, B in ipairs(candidates) do
			local bAxes = partAxes(B)
			for fi = 1, 3 do
				local o1 = bAxes[(fi % 3) + 1]
				local o2 = bAxes[((fi + 1) % 3) + 1]
				for sgn = -1, 1, 2 do
					local m = bAxes[fi].dir * sgn
					local cross = n:Cross(m)
					if cross:Dot(cross) < c.parallelEps then continue end
					-- line shared by the two planes (n,a1) and (m,b1)
					local a1 = n:Dot(c0)
					local fc = B.Position + m * bAxes[fi].ext
					local b1 = m:Dot(fc)
					local ndm = n:Dot(m)
					local det = 1 - ndm * ndm
					local p0 = n * ((a1 - b1 * ndm) / det) + m * ((b1 - a1 * ndm) / det)
					local d = cross.Unit
					-- clip: floor top rectangle (tight), blocker face rectangle (slack)
					local t0, t1 = -1e9, 1e9
					t0, t1 = clipAxis(t0, t1, p0, d, c0, fu.dir, fu.ext, 0.01)
					if t0 <= t1 then t0, t1 = clipAxis(t0, t1, p0, d, c0, fv.dir, fv.ext, 0.01) end
					if t0 <= t1 then t0, t1 = clipAxis(t0, t1, p0, d, fc, o1.dir, o1.ext, c.eps) end
					if t0 <= t1 then t0, t1 = clipAxis(t0, t1, p0, d, fc, o2.dir, o2.ext, c.eps) end
					if t1 - t0 >= c.minLen then
						local isRim = (B == part)
						-- blocked side: a foreign part's interior is behind its face
						-- (-m); the floor's own rim points off the floor (+m)
						local w = (isRim and m or -m)
						w = w - n * w:Dot(n)
						if w.Magnitude > 1e-4 then
							if trimEmit(segments, c, g, near, axCache, p0, d, t0, t1, w.Unit, part, B, isRim) == 0 then
								nDropped += 1
							end
						end
					end
				end
			end
		end
	end

	return {
		segments = segments, config = c,
		stats = { floors = nFloors, skippedFloors = nSkippedFloors, blockers = #blockers, nonBlockBlockers = nNonBlock, segments = #segments, dropped = nDropped },
	}
end

-- Stage 1b: trace boundaries the face construction cannot see — floor
-- coverage ending against TERRAIN (cliff walls), non-block parts, or at
-- clearance death lines. Grid cell-edges with a missing neighbour and no
-- nearby part-derived segment are merged into axis-aligned runs and fed
-- through the same classifier with rim semantics (the terrain ray pair then
-- labels cliff bases wall, open ends dropoff). Organic edges keep a stepped
-- shape per DESIGN; runs under 2 studs are noise and dropped.
function Boundary.traceUncovered(localData: any, result: any, cfg: Config?)
	local c = merged(cfg)
	local buck: { [string]: { Segment } } = {}
	local function bkey(x: number, z: number): string
		return math.floor(x / 4) .. ":" .. math.floor(z / 4)
	end
	for _, s in ipairs(result.segments) do
		local n = math.max(1, math.ceil((s.b - s.a).Magnitude / 2))
		for i = 0, n do
			local p = s.a:Lerp(s.b, i / n)
			local k = bkey(p.X, p.Z)
			local lst = buck[k]
			if not lst then lst = {}; buck[k] = lst end
			lst[#lst + 1] = s
		end
	end
	local function covered(p: Vector3): boolean
		local bx, bz = math.floor(p.X / 4), math.floor(p.Z / 4)
		for a = -1, 1 do
			for b = -1, 1 do
				local lst = buck[(bx + a) .. ":" .. (bz + b)]
				if lst then
					for _, s in ipairs(lst) do
						local ab = s.b - s.a
						local t = math.clamp((p - s.a):Dot(ab) / ab:Dot(ab), 0, 1)
						if (s.a + ab * t - p).Magnitude < 0.9 then return true end
					end
				end
			end
		end
		return false
	end

	local added = 0
	for part, g in pairs(localData.grids) do
		if not g.fallback and g.n and g.origin and #g.cells > 0 then
			local dirs = {
				{ du = 1, dv = 0 }, { du = -1, dv = 0 },
				{ du = 0, dv = 1 }, { du = 0, dv = -1 },
			}
			for _, dd in ipairs(dirs) do
				local rows: { [number]: { { cross: number, cell: any } } } = {}
				for _, cell in ipairs(g.cells) do
					if not g.index[string.format("%d:%d", cell.ui + dd.du, cell.vi + dd.dv)] then
						local rowK, crossI
						if dd.du ~= 0 then rowK = cell.ui; crossI = cell.vi else rowK = cell.vi; crossI = cell.ui end
						local lst = rows[rowK]
						if not lst then lst = {}; rows[rowK] = lst end
						lst[#lst + 1] = { cross = crossI, cell = cell }
					end
				end
				local wdir = g.u * dd.du + g.v * dd.dv
				for _, lst in pairs(rows) do
					table.sort(lst, function(x, y) return x.cross < y.cross end)
					local i = 1
					while i <= #lst do
						local j = i
						while j < #lst and lst[j + 1].cross == lst[j].cross + 1 do j += 1 end
						local half = (dd.du ~= 0) and g.v or g.u
						local pa = lst[i].cell.pos + wdir * (g.step * 0.5) - half * (g.step * 0.5)
						local pb = lst[j].cell.pos + wdir * (g.step * 0.5) + half * (g.step * 0.5)
						if (pb - pa).Magnitude >= 2 then
							local cov = false
							for _, f in ipairs({ 0.1, 0.5, 0.9 }) do
								if covered(pa:Lerp(pb, f)) then cov = true; break end
							end
							if not cov then
								local w = wdir - g.n * wdir:Dot(g.n)
								if w.Magnitude > 1e-4 then
									result.segments[#result.segments + 1] = {
										a = pa, b = pb, floor = part, source = part,
										kind = "rim", outDir = w.Unit,
									}
									added += 1
								end
							end
						end
						i = j + 1
					end
				end
			end
		end
	end
	result.stats.traced = added
	return result
end

-- Stage 2: classify segments into "wall" | "seam" | "dropoff" | "internal".
-- Continuation evidence comes from ONE down-raycast DIRECTLY across the edge
-- (through the ±stepUp window at the perpendicular probe point) + standability
-- checks (slope, headroom) on the hit. Strictly perpendicular — proximity
-- lookups with a lateral radius repeatedly borrowed evidence from around
-- corners (fake seams over gaps, internal-suppressed holes at flush corners).
--   face: down-ray against the crossing part ONLY, probe offset capped by the
--         part's thickness (a thin low fence is steppable -> seam; a thin
--         tall fence stays wall). Flush hit -> internal; <=stepUp -> seam;
--         else/no hit -> wall.
--   rim:  down-ray against all parts at the probe point, second farther tap
--         bridges authored sub-stud cracks. Flush -> internal (suppressed),
--         <=stepUp -> seam; else overlap probe / terrain rays -> wall;
--         else dropoff. ClipRamp on either side of a contact forces seam.
-- A segment may change class along its length (a neighbor slab ends mid-edge):
-- it is split into uniform runs, cuts snapped onto the responsible part's face
-- plane (probed on both sides of the transition; sampled position kept if the
-- occluder is not a block, e.g. terrain).
function Boundary.classify(result: any, localData: any, floorData: any, parts: {BasePart}, cfg: Config?)
	local c = merged(cfg)
	-- pick up boundaries the face construction cannot see (terrain cliffs,
	-- non-block blockers, clearance death lines) before classifying
	Boundary.traceUncovered(localData, result, cfg)
	local out: {Segment} = {}
	local nWall, nSeam, nDrop, nInternal, nSplit, nTinySeam, nHarmonised, nVoid = 0, 0, 0, 0, 0, 0, 0, 0
	local UPv = Vector3.new(0, 1, 0)

	local op = OverlapParams.new()
	op.FilterType = Enum.RaycastFilterType.Include
	op.FilterDescendantsInstances = parts
	local rpTerrain = RaycastParams.new()
	rpTerrain.FilterType = Enum.RaycastFilterType.Include
	rpTerrain.FilterDescendantsInstances = { workspace.Terrain }
	local probe = Instance.new("Part")
	probe.Name = "NVGN_ClassProbe"
	probe.Size = Vector3.new(0.05, c.minClearance - 0.1, 0.05)
	probe.Anchored = true; probe.CanCollide = false; probe.CanQuery = false; probe.CanTouch = false
	probe.Transparency = 1
	probe.Parent = workspace

	local blockers: {BasePart} = {}
	for _, p in ipairs(parts) do
		if isBlock(p) then blockers[#blockers + 1] = p end
	end
	local nearByFloor: { [BasePart]: {BasePart} } = {}
	local axCache = {}
	local function nearOf(f: BasePart): {BasePart}
		local nr = nearByFloor[f]
		if nr then return nr end
		nr = {}
		local rF = f.Size.Magnitude * 0.5
		for _, Bb in ipairs(blockers) do
			if Bb ~= f and (Bb.Position - f.Position).Magnitude <= rF + Bb.Size.Magnitude * 0.5 + 3 then
				nr[#nr + 1] = Bb
			end
		end
		nearByFloor[f] = nr
		return nr
	end

	local rpParts = RaycastParams.new()
	rpParts.FilterType = Enum.RaycastFilterType.Include
	rpParts.FilterDescendantsInstances = parts
	local rpBySource: { [BasePart]: RaycastParams } = {}
	local function rpFor(part: BasePart): RaycastParams
		local rp = rpBySource[part]
		if not rp then
			rp = RaycastParams.new()
			rp.FilterType = Enum.RaycastFilterType.Include
			rp.FilterDescendantsInstances = { part }
			rpBySource[part] = rp
		end
		return rp
	end

	-- Is the surface at this hit standable? (slope limit + crawl headroom)
	local function standableHit(res: RaycastResult): boolean
		local slope = math.deg(math.acos(math.clamp(res.Normal.Y, -1, 1)))
		if slope > c.maxSlope and not isClip(res.Instance) then return false end
		probe.CFrame = CFrame.new(res.Position + UPv * (0.1 + (c.minClearance - 0.1) * 0.5))
		for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
			if hit ~= res.Instance then return false end
		end
		return true
	end

	-- The standable surface DIRECTLY across the edge, or nil: a vertical
	-- down-ray through the ±stepUp window at each perpendicular offset in turn.
	-- An unstandable hit falls through to the next, farther tap — a shingled
	-- neighbour's lip overhangs the first stud of the surface beyond (arch path
	-- strips), and authored sub-stud cracks must not read as dropoffs. Offsets
	-- stay strictly perpendicular so corners cannot borrow lateral evidence.
	local function continuation(s: Segment, pt: Vector3, offs: {number}, rp: RaycastParams): RaycastResult?
		for _, off in ipairs(offs) do
			local pp = pt + s.outDir * off
			-- low pass: window-sized ray, correct under overhangs
			local res = workspace:Raycast(
				Vector3.new(pp.X, pt.Y + c.stepUp + 0.3, pp.Z),
				Vector3.new(0, -(2 * c.stepUp + 0.6), 0), rp)
			if not (res and res.Instance ~= s.floor and standableHit(res)) then
				-- high pass: a THICK pitched neighbour (arch planks) puts the low
				-- origin INSIDE its body (inside-origin miss) or presents its
				-- beveled end face first; cast from well above and accept only
				-- hits inside the step window, so overhead structures cannot
				-- masquerade
				local top = c.stepUp + 6
				res = workspace:Raycast(
					Vector3.new(pp.X, pt.Y + top, pp.Z),
					Vector3.new(0, -(top + c.stepUp + 0.3), 0), rp)
				if not (res and res.Instance ~= s.floor
					and math.abs(res.Position.Y - pt.Y) <= c.stepUp + 0.3
					and standableHit(res)) then
					res = nil
				end
			end
			if res then
				if off <= c.probeOff + 0.01 then return res end
				-- far tap: the hop must not pass through solid — a fence between
				-- two floors is still a wall no matter what lies beyond it
				local pmid = pt + s.outDir * (off - 0.5)
				probe.CFrame = CFrame.new(pmid + UPv * (0.1 + (c.minClearance - 0.1) * 0.5))
				local blocked = false
				for _, h in ipairs(workspace:GetPartsInPart(probe, op)) do
					if h ~= s.floor and h ~= res.Instance then blocked = true; break end
				end
				if not blocked then return res end
				break -- solid in the way: farther taps would cross it too
			end
		end
		return nil
	end

	-- clip-by-name OR steep walkable sheet: entry rules apply (fold ends only)
	local function entryRestricted(part: BasePart): boolean
		if isClip(part) then return true end
		local g = localData.grids[part]
		if g and g.n and math.deg(math.acos(math.clamp(g.n.Y, -1, 1))) > c.steepEntry then
			return true
		end
		return false
	end

	local function classAt(s: Segment, pt: Vector3): (string, BasePart?)
		-- VOID: an edge must be adjacent to standable floor on its NEAR side.
		-- Exposure trimming's ±1-cell tolerance lets segments survive slightly
		-- past the standable surface: buried rims of piercing ramps, and a
		-- descending sheet's bottom-face fold line inside the crawl wedge —
		-- "obscured" edges no agent can ever stand next to. Dropped at emit.
		-- the LINE itself must not be buried inside a third part: a clipramp's
		-- bottom rim sits slightly BELOW the floor it pierces, duplicating the
		-- true fold edge from inside the geometry (its poking ends were the
		-- "green corners"). Floor and source are exempt — a fold line rightly
		-- grazes both of its own surfaces.
		probe.CFrame = CFrame.new(pt + UPv * (0.1 + (c.minClearance - 0.1) * 0.5))
		for _, h in ipairs(workspace:GetPartsInPart(probe, op)) do
			if h ~= s.floor and h ~= s.source then return "void", nil end
		end
		local pn = pt - s.outDir * 0.5
		local rN = workspace:Raycast(Vector3.new(pn.X, pn.Y + 0.3, pn.Z), Vector3.new(0, -0.6, 0), rpParts)
		local nearOk = rN ~= nil and math.abs(rN.Position.Y - pn.Y) <= 0.25
			-- the hit must be the floor ITSELF (or an exact-coplanar tie): a
			-- buried rim's near side can poke past a thin cover into someone
			-- else's surface, which is not "my floor continues here"
			and (rN.Instance == s.floor or math.abs(rN.Position.Y - pn.Y) <= 0.02)
			and standableHit(rN)
		if not nearOk then
			return "void", nil
		end
		if s.kind == "face" then
			local oa = axCache[s.source]
			if not oa then oa = partAxes(s.source); axCache[s.source] = oa end
			-- FOLD: the generating face can be the source's own walkable TOP —
			-- two pitched tops meeting at a crease (arch planks laid end-to-end).
			-- The perpendicular probe is degenerate there (the surfaces meet AT
			-- the line), and a fold between walkable surfaces is continuous
			-- floor: internal (clip involved => seam). Detect: pt lies on the
			-- source's walkable-face plane.
			local g2 = localData.grids[s.source]
			if g2 and not g2.fallback and g2.n then
				local ni2, best2 = 2, -math.huge
				for i, a in ipairs(oa) do
					local dd = math.abs(a.dir:Dot(g2.n))
					if dd > best2 then best2 = dd; ni2 = i end
				end
				local c0s = s.source.Position + g2.n * oa[ni2].ext
				-- tolerance = seamEps: a ramp arriving a couple tenths above the
				-- floor is the same "small authored discrepancy" a seam covers
				if math.abs((pt - c0s):Dot(g2.n)) <= c.seamEps then
					-- a fold that changes slope sharply is a REAL transition edge
					-- (ramp/stairs entry — regardless of the part's name); gentle
					-- creases (arch planks) stay internal
					local gF = localData.grids[s.floor]
					local relDeg = math.deg(math.acos(math.clamp(
						math.abs(g2.n:Dot(gF and gF.n or UPv)), -1, 1)))
					if isClip(s.source) or isClip(s.floor) or relDeg > c.steepEntry then
						return "seam", s.source
					end
					return "internal", s.source
				end
			end
			-- probe INSIDE the crossing part: offsets capped by its thickness so
			-- a thin fence is probed at its core, not overshot
			local proj = 0
			for i = 1, 3 do proj += math.abs(oa[i].dir:Dot(s.outDir)) * oa[i].ext end
			local lim = math.max(0.15, proj * 0.8)
			-- source buried just under the surface (a clipramp's far end, or any
			-- part terminating below floor level): the FLOOR itself continues
			-- flush across this line — continuity, not an edge
			local ppb = pt + s.outDir * math.min(c.probeOff, lim)
			local rF2 = workspace:Raycast(
				Vector3.new(ppb.X, pt.Y + 0.3, ppb.Z), Vector3.new(0, -0.6, 0), rpParts)
			if rF2 and rF2.Instance == s.floor and math.abs(rF2.Position.Y - pt.Y) <= c.flushEps then
				-- guard: the floor hit only counts if the surface is EXPOSED — a
				-- solid standing on the floor puts this ray's origin inside itself
				-- (inside-origin miss) and the ray still finds the floor beneath
				probe.CFrame = CFrame.new(ppb + UPv * (0.1 + (c.minClearance - 0.1) * 0.5))
				local covered = false
				for _, h in ipairs(workspace:GetPartsInPart(probe, op)) do
					if h ~= s.floor then covered = true; break end
				end
				if not covered then
					return "internal", s.source
				end
			end
			local res = continuation(s, pt,
				{ math.min(c.probeOff, lim), math.min(c.gapProbeOff, lim) }, rpFor(s.source))
			if res then
				local dy = math.abs(res.Position.Y - pt.Y)
				if entryRestricted(s.source) or entryRestricted(s.floor) then
					-- clip/steep-sheet entries are FOLD lines only (caught above):
					-- a side or end face is never an entry — fall through
				elseif dy <= c.flushEps then
					return "internal", s.source
				elseif dy <= c.seamEps then
					return "seam", s.source
				end
			end
			-- no standable continuation: wall only if solid actually occupies
			-- standing space past the edge — a steep wedge face falling away is a
			-- DROPOFF, not a wall (faces can drop too)
			probe.CFrame = CFrame.new(ppb + UPv * (0.1 + (c.minClearance - 0.1) * 0.5))
			for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
				if hit ~= s.floor then return "wall", hit end
			end
			local tUp = workspace:Raycast(ppb + UPv * 0.15, UPv * c.terrainCap, rpTerrain)
			if tUp and tUp.Distance < c.minClearance then return "wall", nil end
			if not tUp and workspace:Raycast(ppb + UPv * c.terrainCap, -UPv * (c.terrainCap - 0.25), rpTerrain) then
				return "wall", nil
			end
			return "dropoff", s.source
		end
		-- rim
		local res = continuation(s, pt, { c.probeOff, c.gapProbeOff, c.hopProbeOff }, rpParts)
		if res then
			local dy = math.abs(res.Position.Y - pt.Y)
			if entryRestricted(res.Instance) or entryRestricted(s.floor) then
				-- clip/steep-sheet contacts never "internal": a ramp join is the
				-- canonical portal, but only where genuinely continuous
				if dy <= c.seamEps then return "seam", res.Instance end
			elseif dy <= c.flushEps then
				return "internal", res.Instance
			elseif dy <= c.seamEps then
				return "seam", res.Instance
			end
		end
		local pp = pt + s.outDir * c.probeOff
		probe.CFrame = CFrame.new(pp + UPv * (0.1 + (c.minClearance - 0.1) * 0.5))
		for _, hit in ipairs(workspace:GetPartsInPart(probe, op)) do
			if hit ~= s.floor then return "wall", hit end
		end
		local tUp = workspace:Raycast(pp + UPv * 0.15, UPv * c.terrainCap, rpTerrain)
		if tUp and tUp.Distance < c.minClearance then return "wall", nil end
		if not tUp and workspace:Raycast(pp + UPv * c.terrainCap, -UPv * (c.terrainCap - 0.25), rpTerrain) then
			return "wall", nil -- embedded in a terrain hillside
		end
		return "dropoff", nil
	end

	for _, s in ipairs(result.segments) do
		local dvec = s.b - s.a
		local len = dvec.Magnitude
		if len < 1e-3 then continue end
		local d = dvec / len
		local nS = math.max(2, math.ceil(len / c.classStep))
		local cls = table.create(nS)
		local oth = table.create(nS)
		for i = 1, nS do
			local pt = s.a:Lerp(s.b, (i - 0.5) / nS)
			cls[i], oth[i] = classAt(s, pt)
		end

		local runs = {}
		local i = 1
		while i <= nS do
			local j = i
			while j < nS and cls[j + 1] == cls[i] do j += 1 end
			runs[#runs + 1] = { i = i, j = j, class = cls[i] }
			i = j + 1
		end

		-- absorb runs too short to be real edges into their longer neighbour;
		-- coalesce equal-class neighbours after each merge. Never deletes span:
		-- the chain stays continuous, only the class over the sliver changes.
		local function runLen(r): number
			return (r.j - r.i + 1) * len / nS
		end
		while #runs > 1 do
			local si, sl = nil, c.minRunLen
			for k, r in ipairs(runs) do
				local L = runLen(r)
				if L < sl then sl = L; si = k end
			end
			if not si then break end
			local r = runs[si]
			local prev, nxt = runs[si - 1], runs[si + 1]
			local into
			if prev and nxt then
				into = (runLen(prev) >= runLen(nxt)) and prev or nxt
			else
				into = prev or nxt
			end
			into.i = math.min(into.i, r.i)
			into.j = math.max(into.j, r.j)
			table.remove(runs, si)
			local k = 1
			while k < #runs do
				if runs[k].class == runs[k + 1].class then
					runs[k].j = runs[k + 1].j
					table.remove(runs, k + 1)
				else
					k += 1
				end
			end
		end
		if #runs > 1 then nSplit += 1 end

		local nr = nearOf(s.floor)
		local off = s.outDir * c.probeOff
		local cuts = {}
		for k = 1, #runs - 1 do
			local tcRaw = len * runs[k].j / nS
			local tA = snapToOccluder(nr, axCache, s.a, d, tcRaw, -1, off, c.snapRadius)
			local tB = snapToOccluder(nr, axCache, s.a, d, tcRaw, 1, off, c.snapRadius)
			local t = tcRaw
			if tA ~= tcRaw and (tB == tcRaw or math.abs(tA - tcRaw) <= math.abs(tB - tcRaw)) then
				t = tA
			elseif tB ~= tcRaw then
				t = tB
			end
			cuts[k] = math.clamp(t, 0, len)
		end

		for k, r in ipairs(runs) do
			local ta = (k == 1) and 0 or cuts[k - 1]
			local tb = (k == #runs) and len or cuts[k]
			if tb - ta >= c.minLen then
				if r.class == "void" then
					-- no standable floor beside this span: not an edge at all
					nVoid += 1
					continue
				end
				if r.class == "seam" and (tb - ta) < c.minSeamLen then
					-- a portal no agent fits through is BLOCKED, not absent —
					-- deleting it left holes in chains (short first-step seams)
					nTinySeam += 1
					r.class = "wall"
				end
				if r.class == "internal" then
					-- flush contact: weld-stage bookkeeping, not a navigation
					-- edge — counted, and emitted only on request
					nInternal += 1
					if not c.keepInternal then continue end
				end
				out[#out + 1] = {
					a = s.a + d * ta, b = s.a + d * tb,
					floor = s.floor, source = s.source, kind = s.kind,
					outDir = s.outDir, class = r.class,
					other = oth[math.floor((r.i + r.j) / 2)],
				}
			end
		end
	end
	-- whole tiny wall/dropoff fragments close loop corners: keep them, but
	-- adopt the class of an adjacent collinear same-floor edge so chains read
	-- uniformly instead of flickering at corners
	for _, s in ipairs(out) do
		local L = (s.b - s.a).Magnitude
		if L < c.minFragLen and L > 1e-3 and (s.class == "wall" or s.class == "dropoff") then
			local d = (s.b - s.a) / L
			for _, t in ipairs(out) do
				if t ~= s and t.floor == s.floor and t.class ~= s.class
					and (t.class == "wall" or t.class == "dropoff") then
					local dv = t.b - t.a
					local Lt = dv.Magnitude
					if Lt >= c.minFragLen and math.abs(d:Dot(dv / Lt)) > 0.98 then
						local touch = math.min(
							(t.a - s.a).Magnitude, (t.a - s.b).Magnitude,
							(t.b - s.a).Magnitude, (t.b - s.b).Magnitude)
						if touch < 0.15 then
							if s.class == "wall" then nWall -= 1 else nDrop -= 1 end
							if t.class == "wall" then nWall += 1 else nDrop += 1 end
							s.class = t.class
							s.other = t.other
							nHarmonised += 1
							break
						end
					end
				end
			end
		end
	end
	probe:Destroy()

	-- merge collinear neighbours: same floor + class + side, parallel and
	-- touching — fragments from per-pair construction and classification cuts
	-- that ended up equal-class collapse into single clean edges
	local nMerged = 0
	local changed = true
	while changed do
		changed = false
		for i = 1, #out do
			local a = out[i]
			if a then
				local da = a.b - a.a
				local la = da.Magnitude
				if la > 1e-3 then
					da = da / la
					for j = 1, #out do
						local b = j ~= i and out[j] or nil
						if b and b.floor == a.floor and b.class == a.class
							and b.outDir:Dot(a.outDir) > 0.99 then
							local db = b.b - b.a
							local lb = db.Magnitude
							if lb > 1e-3 then
								db = db / lb
								local dd = da:Dot(db)
								if math.abs(dd) > 0.9995 then
									local b1, b2 = b.a, b.b
									if dd < 0 then b1, b2 = b2, b1 end
									if (a.b - b1).Magnitude <= 0.2 then
										a.b = b2
										out[j] = false
										nMerged += 1
										changed = true
									elseif (b2 - a.a).Magnitude <= 0.2 then
										a.a = b1
										out[j] = false
										nMerged += 1
										changed = true
									end
								end
							end
						end
					end
				end
			end
		end
	end
	local compacted: {Segment} = {}
	for i = 1, #out do
		if out[i] then compacted[#compacted + 1] = out[i] end
	end
	out = compacted

	-- weld nearby endpoints: edges of all sorts that nearly meet connect at a
	-- shared point so chains close; centroid snap moves nothing further than
	-- weldTol so the geometry is not meaningfully altered
	local nWelded = 0
	local pts = {}
	for _, s2 in ipairs(out) do
		pts[#pts + 1] = { s = s2, e = "a" }
		pts[#pts + 1] = { s = s2, e = "b" }
	end
	local used = table.create(#pts, false)
	for i = 1, #pts do
		if not used[i] then
			local pi = (pts[i].s :: any)[pts[i].e] :: Vector3
			local cluster = { pts[i] }
			local sum = pi
			for j = i + 1, #pts do
				if not used[j] and pts[j].s ~= pts[i].s then
					local pj = (pts[j].s :: any)[pts[j].e] :: Vector3
					if (pj - pi).Magnitude <= c.weldTol then
						used[j] = true
						cluster[#cluster + 1] = pts[j]
						sum += pj
					end
				end
			end
			if #cluster > 1 then
				local ctr = sum / #cluster
				for _, r2 in ipairs(cluster) do
					(r2.s :: any)[r2.e] = ctr
				end
				nWelded += #cluster
			end
		end
	end

	-- extend dangling ends along their own line to meet a transverse edge:
	-- welding closes near-coincident corners, but an edge stopped SHORT (dead
	-- floor strip beside a sheet) has nothing nearby to weld to — continue it
	-- straight until it touches another edge, so chains close
	local nExtended = 0
	local function tryExtend(s2: Segment, endName: string)
		local pe = (s2 :: any)[endName] :: Vector3
		for _, o in ipairs(out) do
			if o ~= s2 and ((o.a - pe).Magnitude < 0.15 or (o.b - pe).Magnitude < 0.15) then
				return -- not dangling: already meets another edge
			end
		end
		local dirv = (endName == "b") and (s2.b - s2.a) or (s2.a - s2.b)
		if dirv.Magnitude < 1e-3 then return end
		dirv = dirv.Unit
		local bestT, bestPoint = math.huge, nil
		for _, q in ipairs(out) do
			if q ~= s2 then
				local u = q.b - q.a
				local ul = u.Magnitude
				if ul > 1e-3 then
					u = u / ul
					local w0 = pe - q.a
					local bb = dirv:Dot(u)
					local denom = 1 - bb * bb
					if math.abs(denom) > 1e-4 then
						local t = (bb * u:Dot(w0) - dirv:Dot(w0)) / denom
						local sq = u:Dot(w0) + bb * t
						if t > 0.05 and t <= c.maxExtend and sq >= -0.1 and sq <= ul + 0.1 then
							local pRay = pe + dirv * t
							local pSeg = q.a + u * math.clamp(sq, 0, ul)
							if (pRay - pSeg).Magnitude <= 0.35 and t < bestT then
								bestT = t
								bestPoint = pSeg
							end
						end
					end
				end
			end
		end
		-- collinear case: the continuation is another edge's ENDPOINT straight
		-- ahead (parallel edges defeat the transverse intersection above) —
		-- bridge to the nearest endpoint inside a 45° forward cone
		for _, q in ipairs(out) do
			if q ~= s2 then
				for _, qe in ipairs({ q.a, q.b }) do
					local v = qe - pe
					local dist = v.Magnitude
					if dist > 0.05 and dist <= c.maxExtend and v:Dot(dirv) >= 0.7 * dist and dist < bestT then
						bestT = dist
						bestPoint = qe
					end
				end
			end
		end
		if bestPoint then
			(s2 :: any)[endName] = bestPoint
			nExtended += 1
		end
	end
	for _, s2 in ipairs(out) do
		tryExtend(s2, "a")
		tryExtend(s2, "b")
	end

	for _, s2 in ipairs(out) do
		if s2.class == "wall" then nWall += 1
		elseif s2.class == "seam" then nSeam += 1
		elseif s2.class == "internal" then
		else nDrop += 1 end
	end

	return {
		segments = out, config = c,
		stats = { segments = #out, walls = nWall, seams = nSeam, dropoffs = nDrop, internals = nInternal, splitSegments = nSplit, tinySeams = nTinySeam, harmonised = nHarmonised, voids = nVoid, mergedCollinear = nMerged, weldedEndpoints = nWelded, extendedEnds = nExtended },
	}
end

-- Debug viz. Classified: red = wall, green = seam, cyan = dropoff (endpoint
-- balls on walls/dropoffs — "vertices only at real corners" stays checkable).
-- Unclassified: orange = face, dim white = rim.
function Boundary.visualize(result: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Boundary")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Boundary"; folder.Parent = dbg

	local ORANGE = Color3.new(1, 0.45, 0)
	local GREY = Color3.new(0.85, 0.85, 0.85)
	local COLORS = {
		wall = Color3.new(1, 0.15, 0.15),
		seam = Color3.new(0.15, 1, 0.35),
		dropoff = Color3.new(0, 0.85, 1),
		internal = Color3.new(0.45, 0.6, 0.55), -- flush contact: barely-there
	}
	for _, s in ipairs(result.segments) do
		local dvec = s.b - s.a
		local len = dvec.Magnitude
		if len < 1e-3 then continue end
		local du = dvec / len
		local pn = du:Cross(s.outDir)
		if pn.Y < 0 then pn = -pn end
		local mid = (s.a + s.b) * 0.5 + pn * 0.12
		if s.class == "seam" then
			-- nudge toward the open side: a fold seam's true line sits exactly
			-- where a clipramp sheet meets the floor — drawn dead-on it hides
			-- inside the wedge between the two surfaces
			mid = mid - s.outDir * 0.18 + Vector3.new(0, 0.06, 0)
		elseif s.class == "wall" or s.class == "dropoff" then
			-- welding/merging can drift a line fractionally into its face —
			-- present every edge from the walkable side so none hide in geometry
			mid = mid - s.outDir * 0.12 + Vector3.new(0, 0.03, 0)
		end
		local bar = Instance.new("Part")
		bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
		bar.Material = Enum.Material.Neon
		local doBalls
		if s.class == "internal" then
			bar.Color = COLORS.internal
			bar.Size = Vector3.new(len, 0.08, 0.05)
			bar.Transparency = 0.6
			doBalls = false
		elseif s.class then
			bar.Color = COLORS[s.class] or ORANGE
			bar.Size = Vector3.new(len, s.class == "wall" and 0.24 or 0.18, s.class == "wall" and 0.1 or 0.08)
			doBalls = (s.class ~= "seam")
		elseif s.kind == "face" then
			bar.Color = ORANGE
			bar.Size = Vector3.new(len, 0.24, 0.1)
			doBalls = true
		else
			bar.Color = GREY
			bar.Size = Vector3.new(len, 0.12, 0.06)
			bar.Transparency = 0.4
			doBalls = false
		end
		-- slight overlength: bars whose data endpoints coincide can still show a
		-- visual gap from differing lift normals — let tips overlap at corners
		bar.Size = Vector3.new(bar.Size.X + 0.24, bar.Size.Y, bar.Size.Z)
		bar.CFrame = CFrame.fromMatrix(mid, du, pn)
		bar.Name = (s.class or s.kind) .. ":" .. s.source.Name
		bar.Parent = folder
		if doBalls then
			for _, e in ipairs({ s.a, s.b }) do
				local dotp = Instance.new("Part")
				dotp.Anchored = true; dotp.CanCollide = false; dotp.CanQuery = false; dotp.CanTouch = false
				dotp.Shape = Enum.PartType.Ball
				dotp.Size = Vector3.new(0.3, 0.3, 0.3)
				dotp.Color = bar.Color
				dotp.Material = Enum.Material.Neon
				dotp.CFrame = CFrame.new(e + pn * 0.12)
				dotp.Parent = folder
			end
		end
	end
	return folder
end

return Boundary

