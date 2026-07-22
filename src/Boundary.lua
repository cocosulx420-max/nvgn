--!strict
-- NVGN.Boundary — exact boundary segments from part geometry (stage 1: construction)
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
}

export type Config = { eps: number?, minLen: number?, parallelEps: number?, sampleOff: number?, trimStep: number?, snapRadius: number? }

local DEFAULT = {
	eps = 0.05,          -- slack on the blocker-face clip: flush contact (wall ON
	                     -- floor) intersects exactly at the rectangle edge
	minLen = 0.05,       -- drop degenerate slivers
	parallelEps = 0.0012,-- |n x m|^2 below this = face parallel to floor top, skip
	sampleOff = 0.6,     -- near-side offset for exposure sampling
	trimStep = 0.25,     -- sampling pitch along a span for support runs
	snapRadius = 2,      -- max distance a cut may snap to an occluder face plane
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

	-- snap a sampled cut onto the face plane of whichever nearby block
	-- contains a probe just inside the unsupported side
	local function snapCut(tc: number, sgnU: number): number
		local pu = p0 + d * (tc + sgnU * 0.5)
		local bestT, bestD = tc, c.snapRadius
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

-- Debug viz: one thin neon bar per segment on the floor plane. face = orange
-- (unclassified; the classification stage recolors red/cyan), rim = dim white.
-- Endpoint spheres on face segments make "vertices only at real corners" checkable.
function Boundary.visualize(result: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Boundary")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Boundary"; folder.Parent = dbg

	local ORANGE = Color3.new(1, 0.45, 0)
	local GREY = Color3.new(0.85, 0.85, 0.85)
	for _, s in ipairs(result.segments) do
		local dvec = s.b - s.a
		local len = dvec.Magnitude
		if len < 1e-3 then continue end
		local du = dvec / len
		local pn = du:Cross(s.outDir)
		if pn.Y < 0 then pn = -pn end
		local mid = (s.a + s.b) * 0.5 + pn * 0.12
		local bar = Instance.new("Part")
		bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
		bar.Material = Enum.Material.Neon
		if s.kind == "face" then
			bar.Color = ORANGE
			bar.Size = Vector3.new(len, 0.24, 0.1)
		else
			bar.Color = GREY
			bar.Size = Vector3.new(len, 0.12, 0.06)
			bar.Transparency = 0.4
		end
		bar.CFrame = CFrame.fromMatrix(mid, du, pn)
		bar.Name = s.kind .. ":" .. s.source.Name
		bar.Parent = folder
		if s.kind == "face" then
			for _, e in ipairs({ s.a, s.b }) do
				local dotp = Instance.new("Part")
				dotp.Anchored = true; dotp.CanCollide = false; dotp.CanQuery = false; dotp.CanTouch = false
				dotp.Shape = Enum.PartType.Ball
				dotp.Size = Vector3.new(0.3, 0.3, 0.3)
				dotp.Color = ORANGE
				dotp.Material = Enum.Material.Neon
				dotp.CFrame = CFrame.new(e + pn * 0.12)
				dotp.Parent = folder
			end
		end
	end
	return folder
end

return Boundary
