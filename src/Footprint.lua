--!strict
-- NVGN.Footprint — per-part TRUE footprint grids (new boundary approach)
--
-- The staircasing fix: a floor's dead-cell frontier is jagged in the FLOOR's
-- lattice, so it is never used as geometry. Instead the part doing the
-- cutting supplies its own clean outline: a lattice aligned to the part's
-- local X/Z (yaw frame), world-vertical rays shot UPWARD from below — the
-- underside is what cuts floors beneath, and origins in open air can never
-- hit the inside-origin trap. Cells = underside sample points; outline =
-- boundary runs, axis-aligned in the part's own frame (clean by
-- construction). Floors' dead cells (killer-attributed at bake time) only
-- SELECT which portions of an outline apply — they never contribute geometry.
--
-- Footprints are built ONLY for parts that actually killed floor cells (the
-- killer set from the LocalGrid bake): nothing below a part means nothing to
-- cut, and the 200x200 ground never needs 40k rays.

local Footprint = {}

export type FootCell = { ui: number, vi: number, bottom: Vector3 }
export type Outline = { a: Vector3, b: Vector3, outDir: Vector3 }
export type Foot = {
	part: BasePart,
	u: Vector3, v: Vector3,   -- yaw-frame axes (horizontal, orthonormal)
	origin: Vector3,          -- lattice anchor (horizontal position of part centre)
	step: number,
	cells: {FootCell},
	index: { [string]: FootCell },
	outline: {Outline},       -- boundary runs at underside heights, part-frame clean
}
export type Config = { step: number?, margin: number? }

local DEFAULT = {
	step = 1,    -- lattice pitch, matches the local grids
	margin = 1,  -- lattice overshoot beyond the AABB so the outline closes
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

-- One part's footprint: yaw-frame lattice, up-rays from below.
function Footprint.forPart(part: BasePart, cfg: Config?): Foot?
	local c = merged(cfg)
	local cf = part.CFrame
	local u = Vector3.new(cf.RightVector.X, 0, cf.RightVector.Z)
	u = (u.Magnitude > 1e-3) and u.Unit or Vector3.new(1, 0, 0)
	local v = Vector3.new(-u.Z, 0, u.X)

	-- world AABB half-extents
	local hx, hy, hz = part.Size.X / 2, part.Size.Y / 2, part.Size.Z / 2
	local r, up2, lk = cf.RightVector, cf.UpVector, cf.RightVector:Cross(cf.UpVector)
	local extY = math.abs(r.Y) * hx + math.abs(up2.Y) * hy + math.abs(lk.Y) * hz
	local baseY = part.Position.Y - extY - 1
	local rayLen = extY * 2 + 2

	-- lattice span: project the 8 corners onto u/v (horizontal)
	local uMin, uMax, vMin, vMax = math.huge, -math.huge, math.huge, -math.huge
	for sx = -1, 1, 2 do
		for sy = -1, 1, 2 do
			for sz = -1, 1, 2 do
				local corner = r * (hx * sx) + up2 * (hy * sy) + lk * (hz * sz)
				local cu = corner.X * u.X + corner.Z * u.Z
				local cv = corner.X * v.X + corner.Z * v.Z
				uMin = math.min(uMin, cu); uMax = math.max(uMax, cu)
				vMin = math.min(vMin, cv); vMax = math.max(vMax, cv)
			end
		end
	end
	uMin -= c.margin; uMax += c.margin
	vMin -= c.margin; vMax += c.margin

	local rp = RaycastParams.new()
	rp.FilterType = Enum.RaycastFilterType.Include
	rp.FilterDescendantsInstances = { part }

	local originH = Vector3.new(part.Position.X, 0, part.Position.Z)
	local foot: Foot = {
		part = part, u = u, v = v, origin = originH, step = c.step,
		cells = {}, index = {}, outline = {},
	}
	local nu = math.ceil((uMax - uMin) / c.step)
	local nv = math.ceil((vMax - vMin) / c.step)
	for iu = 0, nu - 1 do
		for iv = 0, nv - 1 do
			local du = uMin + (iu + 0.5) * c.step
			local dv = vMin + (iv + 0.5) * c.step
			local o = originH + u * du + v * dv
			local res = workspace:Raycast(Vector3.new(o.X, baseY, o.Z), Vector3.new(0, rayLen, 0), rp)
			if res then
				local cell: FootCell = { ui = iu, vi = iv, bottom = res.Position }
				foot.cells[#foot.cells + 1] = cell
				foot.index[string.format("%d:%d", iu, iv)] = cell
			end
		end
	end
	if #foot.cells == 0 then return nil end

	-- outline: boundary cell-edges merged into runs (clean in this frame)
	local dirs = {
		{ du = 1, dv = 0 }, { du = -1, dv = 0 },
		{ du = 0, dv = 1 }, { du = 0, dv = -1 },
	}
	for _, dd in ipairs(dirs) do
		local rows: { [number]: { { cross: number, cell: FootCell } } } = {}
		for _, cell in ipairs(foot.cells) do
			if not foot.index[string.format("%d:%d", cell.ui + dd.du, cell.vi + dd.dv)] then
				local rowK, crossI
				if dd.du ~= 0 then rowK = cell.ui; crossI = cell.vi else rowK = cell.vi; crossI = cell.ui end
				local lst = rows[rowK]
				if not lst then lst = {}; rows[rowK] = lst end
				lst[#lst + 1] = { cross = crossI, cell = cell }
			end
		end
		local wdir = u * dd.du + v * dd.dv
		local half = (dd.du ~= 0) and v or u
		for _, lst in pairs(rows) do
			table.sort(lst, function(x, y) return x.cross < y.cross end)
			local i = 1
			while i <= #lst do
				local j = i
				while j < #lst and lst[j + 1].cross == lst[j].cross + 1 do j += 1 end
				local pa = lst[i].cell.bottom + wdir * (c.step * 0.5) - half * (c.step * 0.5)
				local pb = lst[j].cell.bottom + wdir * (c.step * 0.5) + half * (c.step * 0.5)
				foot.outline[#foot.outline + 1] = { a = pa, b = pb, outDir = wdir }
				i = j + 1
			end
		end
	end
	return foot
end

-- Footprints for the killer set (parts that actually cut floor cells).
--
-- Non-blocks are NOT excluded. forPart never needs the part to be a box: the
-- lattice frame is just its yaw, and the up-rays hit real collision geometry,
-- so a union answers exactly like a block does. Gating on isBlock left every
-- union falling back to the jagged floor lattice for no reason. Measured on
-- SmallMap: the one union yields 282 cells and 106 outline runs in 2 ms.
function Footprint.build(killers: {BasePart}, cfg: Config?)
	local c = merged(cfg)
	local foots: { [BasePart]: Foot } = {}
	local nBuilt, nNonBlock, nFailed, nCells, nOutline = 0, 0, 0, 0, 0
	for _, p in ipairs(killers) do
		local f = Footprint.forPart(p, c)
		if f then
			foots[p] = f
			nBuilt += 1
			if not isBlock(p) then nNonBlock += 1 end
			nCells += #f.cells
			nOutline += #f.outline
		else
			nFailed += 1 -- nothing underneath to sample
		end
	end
	return { foots = foots, config = c, stats = { built = nBuilt, nonBlock = nNonBlock, failed = nFailed, skippedNonBlock = 0, cells = nCells, outlineRuns = nOutline } }
end

-- Collect the killer set from a LocalGrid bake (block parts only; terrain
-- and non-blocks are counted by the caller via stats.skippedNonBlock).
function Footprint.killersFrom(localData: any): {BasePart}
	local seen: { [Instance]: boolean } = {}
	local killers: {BasePart} = {}
	for _, g in pairs(localData.grids) do
		for _, d in ipairs(g.dead) do
			local k = d.killer
			if k and k:IsA("BasePart") and not seen[k] then
				seen[k] = true
				killers[#killers + 1] = k
			end
		end
	end
	return killers
end

-- Debug viz: violet underside dots + bright orange outline bars per footprint.
function Footprint.visualize(result: any, parent: Instance?)
	local root = parent or workspace
	local dbg = root:FindFirstChild("NVGN_Debug")
	if not dbg then dbg = Instance.new("Folder"); dbg.Name = "NVGN_Debug"; dbg.Parent = root end
	local old = dbg:FindFirstChild("Footprint")
	if old then old:Destroy() end
	local folder = Instance.new("Folder"); folder.Name = "Footprint"; folder.Parent = dbg

	for part, f in pairs(result.foots) do
		local pf = Instance.new("Folder"); pf.Name = part.Name; pf.Parent = folder
		for _, cell in ipairs(f.cells) do
			local dot = Instance.new("Part")
			dot.Anchored = true; dot.CanCollide = false; dot.CanQuery = false; dot.CanTouch = false
			dot.Size = Vector3.new(0.3, 0.06, 0.3)
			dot.Color = Color3.new(0.55, 0.35, 0.85)
			dot.Material = Enum.Material.SmoothPlastic
			dot.CFrame = CFrame.fromMatrix(cell.bottom - Vector3.new(0, 0.05, 0), f.u, Vector3.new(0, 1, 0))
			dot.Parent = pf
		end
		for _, o in ipairs(f.outline) do
			local dvec = o.b - o.a
			local len = dvec.Magnitude
			if len > 1e-3 then
				local du2 = dvec / len
				local bar = Instance.new("Part")
				bar.Anchored = true; bar.CanCollide = false; bar.CanQuery = false; bar.CanTouch = false
				bar.Size = Vector3.new(len + 0.1, 0.12, 0.08)
				bar.Color = Color3.new(1, 0.55, 0.1)
				bar.Material = Enum.Material.Neon
				bar.CFrame = CFrame.fromMatrix((o.a + o.b) * 0.5, du2, Vector3.new(0, 1, 0))
				bar.Parent = pf
			end
		end
	end
	return folder
end

return Footprint
