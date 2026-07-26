--!strict
-- NVGN.Serialize — bake to text, and load it back.
--
-- The project's premise is a BAKED navmesh: an expensive build once, a cheap
-- load at runtime. Until this exists the whole result is live tables rebuilt
-- from scratch every run, which means nothing can be shipped and no pathfinder
-- can be tested against a bake rather than against a build.
--
-- FORMAT is line-based text, versioned, one record per line. Not JSON: the
-- encoder would be simpler but every number would carry quotes and punctuation,
-- and the result is meant to be diffable so a bake that changes unexpectedly
-- shows what changed. Not binary either -- the map is tens of kilobytes and
-- legibility is worth more here than the last factor of two.
--
-- COORDINATES ARE NOT QUANTIZED. Every stage of this pipeline exists to keep
-- boundary geometry exact -- lines are stolen from real face planes and are
-- accurate to 0.002 studs -- so rounding coordinates on the way out would throw
-- that away at the very last step, for a saving that does not matter. Numbers
-- are written with %.9g, which round-trips float32 exactly.
--
-- PART REFERENCES are the hard part. A polygon knows which BasePart it was
-- built on, and destruction needs that attribution to survive a reload, but an
-- instance cannot go into a string. Two mechanisms, deliberately:
--
--   PRIMARY: an `NVGN_Id` attribute written onto each referenced part at save
--   time. Robust against renaming and against a rename to something containing
--   a dot, which is what breaks path parsing. Attributes are not instances, so
--   unlike a marker part they cannot be picked up by the next bake -- a trap
--   this project has already fallen into twice.
--
--   FALLBACK: the full name, stored alongside and used only when the attribute
--   is missing. Purely for diagnosing a bake whose parts were replaced.
--
-- A part that resolves to nothing on load is NOT an error. That is what a
-- destroyed building looks like, and reporting it is the point.

local Serialize = {}

local VERSION = "NVGN1"
local ID_ATTR = "NVGN_Id"

local function fmt(n: number): string
	return string.format("%.9g", n)
end

local function v3(v: Vector3): string
	return fmt(v.X) .. " " .. fmt(v.Y) .. " " .. fmt(v.Z)
end

--------------------------------------------------------------------------

function Serialize.encode(mesh: any): string
	local out: {string} = {}
	local parts: {BasePart} = {}
	local partIndex: {[BasePart]: number} = {}
	local function idOf(p: BasePart?): number
		if not p then return 0 end
		local i = partIndex[p]
		if i then return i end
		parts[#parts + 1] = p
		partIndex[p] = #parts
		return #parts
	end
	for _, p in ipairs(mesh.polys) do idOf(p.floor) end

	-- header carries totals so a load can check itself rather than trust us
	local area = 0
	for _, p in ipairs(mesh.polys) do area += p.area end

	out[#out + 1] = VERSION
	out[#out + 1] = string.format("meta %d %d %d %d %s",
		#parts, #mesh.polys, #mesh.portals, #(mesh.volumes or {}), fmt(area))

	out[#out + 1] = "parts"
	for i, p in ipairs(parts) do
		p:SetAttribute(ID_ATTR, i)
		out[#out + 1] = i .. "\t" .. p:GetFullName()
	end

	out[#out + 1] = "polys"
	for _, p in ipairs(mesh.polys) do
		local mc = p.minClearance
		-- math.huge means "no volume reaches this polygon", which is a real
		-- statement and not a missing value; `inf` keeps it that way on reload
		local mcs = (mc == nil or mc == math.huge) and "inf" or fmt(mc)
		local row = { tostring(partIndex[p.floor] or 0), mcs, fmt(p.area), tostring(#p.verts) }
		for i, v in ipairs(p.verts) do
			row[#row + 1] = v3(v)
			row[#row + 1] = p.classes[i] or "internal"
		end
		out[#out + 1] = table.concat(row, " ")
	end

	out[#out + 1] = "portals"
	for _, pt in ipairs(mesh.portals) do
		out[#out + 1] = string.format("%d %d %s %s %s %s",
			pt.a, pt.b, pt.class, pt.kind, v3(pt.p1), v3(pt.p2))
	end

	out[#out + 1] = "volumes"
	for _, v in ipairs(mesh.volumes or {}) do
		local c = v.cframe
		local x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22 = c:GetComponents()
		local nums = { x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22 }
		local row = {}
		for _, n in ipairs(nums) do row[#row + 1] = fmt(n) end
		row[#row + 1] = v3(v.size)
		row[#row + 1] = fmt(v.minClearance)
		row[#row + 1] = tostring(v.component or 0)
		out[#out + 1] = table.concat(row, " ")
	end

	out[#out + 1] = "end"
	return table.concat(out, "\n")
end

--------------------------------------------------------------------------

local function findByAttribute(id: number): BasePart?
	for _, d in ipairs(workspace:GetDescendants()) do
		if d:IsA("BasePart") and d:GetAttribute(ID_ATTR) == id then return d end
	end
	return nil
end

function Serialize.decode(text: string): any
	local lines = string.split(text, "\n")
	if lines[1] ~= VERSION then
		error("NVGN.Serialize: unknown format '" .. tostring(lines[1]) .. "'")
	end

	local meta = string.split(lines[2], " ")
	local expect = {
		parts = tonumber(meta[2]) or 0,
		polys = tonumber(meta[3]) or 0,
		portals = tonumber(meta[4]) or 0,
		volumes = tonumber(meta[5]) or 0,
		area = tonumber(meta[6]) or 0,
	}

	-- one sweep of the workspace, not one per part: resolving 99 parts by
	-- scanning descendants each time is quadratic and the map is not small
	local byId: {[number]: BasePart} = {}
	for _, d in ipairs(workspace:GetDescendants()) do
		if d:IsA("BasePart") then
			local a = d:GetAttribute(ID_ATTR)
			if typeof(a) == "number" then byId[a] = d end
		end
	end

	local parts: {BasePart?} = {}
	local names: {string} = {}
	local polys, portals, volumes = {}, {}, {}
	local missing = 0

	local section = nil
	for i = 3, #lines do
		local line = lines[i]
		if line == "" then continue end
		if line == "parts" or line == "polys" or line == "portals" or line == "volumes" then
			section = line
		elseif line == "end" then
			break
		elseif section == "parts" then
			local tab = string.find(line, "\t")
			local idx = tonumber(string.sub(line, 1, (tab or 1) - 1)) or 0
			local name = tab and string.sub(line, tab + 1) or ""
			names[idx] = name
			parts[idx] = byId[idx]
			if not parts[idx] then missing += 1 end
		elseif section == "polys" then
			local f = string.split(line, " ")
			local pi = tonumber(f[1]) or 0
			local mc = (f[2] == "inf") and math.huge or (tonumber(f[2]) or math.huge)
			local area = tonumber(f[3]) or 0
			local n = tonumber(f[4]) or 0
			local verts, classes = {}, {}
			local k = 5
			for _ = 1, n do
				verts[#verts + 1] = Vector3.new(
					tonumber(f[k]) or 0, tonumber(f[k + 1]) or 0, tonumber(f[k + 2]) or 0)
				classes[#classes + 1] = f[k + 3]
				k += 4
			end
			polys[#polys + 1] = {
				floor = parts[pi], floorName = names[pi], floorId = pi,
				minClearance = mc, area = area, verts = verts, classes = classes,
			}
		elseif section == "portals" then
			local f = string.split(line, " ")
			portals[#portals + 1] = {
				a = tonumber(f[1]) or 0, b = tonumber(f[2]) or 0,
				class = f[3], kind = f[4],
				p1 = Vector3.new(tonumber(f[5]) or 0, tonumber(f[6]) or 0, tonumber(f[7]) or 0),
				p2 = Vector3.new(tonumber(f[8]) or 0, tonumber(f[9]) or 0, tonumber(f[10]) or 0),
			}
			local pt = portals[#portals]
			pt.length = (pt.p2 - pt.p1).Magnitude
		elseif section == "volumes" then
			local f = string.split(line, " ")
			local n = {}
			for j = 1, 12 do n[j] = tonumber(f[j]) or 0 end
			volumes[#volumes + 1] = {
				cframe = CFrame.new(n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8], n[9], n[10], n[11], n[12]),
				size = Vector3.new(tonumber(f[13]) or 0, tonumber(f[14]) or 0, tonumber(f[15]) or 0),
				minClearance = tonumber(f[16]) or 0,
				component = tonumber(f[17]) or 0,
			}
		end
	end

	-- the neighbour lists are derivable, so they are rebuilt rather than stored
	local neighbours = {}
	for i = 1, #polys do neighbours[i] = {} end
	for pi, pt in ipairs(portals) do
		if neighbours[pt.a] and neighbours[pt.b] then
			table.insert(neighbours[pt.a], { poly = pt.b, portal = pi })
			table.insert(neighbours[pt.b], { poly = pt.a, portal = pi })
		end
	end

	local area = 0
	for _, p in ipairs(polys) do area += p.area end

	local report = {
		polys = #polys, portals = #portals, volumes = #volumes,
		expectedPolys = expect.polys, expectedPortals = expect.portals,
		expectedVolumes = expect.volumes,
		area = area, expectedArea = expect.area, areaError = math.abs(area - expect.area),
		partsMissing = missing, parts = expect.parts,
		ok = (#polys == expect.polys) and (#portals == expect.portals)
			and (#volumes == expect.volumes) and math.abs(area - expect.area) <= 0.01,
	}

	return { polys = polys, portals = portals, volumes = volumes,
		neighbours = neighbours, report = report }
end

--------------------------------------------------------------------------

function Serialize.save(mesh: any, parent: Instance?): StringValue
	local host = parent or game:GetService("ServerStorage")
	local text = Serialize.encode(mesh)
	local sv = host:FindFirstChild("NVGN_Bake")
	if not sv or not sv:IsA("StringValue") then
		if sv then sv:Destroy() end
		sv = Instance.new("StringValue")
		sv.Name = "NVGN_Bake"
		sv.Parent = host
	end
	;(sv :: StringValue).Value = text
	return sv :: StringValue
end

function Serialize.load(parent: Instance?): any
	local host = parent or game:GetService("ServerStorage")
	local sv = host:FindFirstChild("NVGN_Bake")
	if not sv or not sv:IsA("StringValue") then
		error("NVGN.Serialize: no NVGN_Bake under " .. host:GetFullName())
	end
	return Serialize.decode((sv :: StringValue).Value)
end

return Serialize
