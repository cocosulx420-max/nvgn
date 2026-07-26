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

local SVO = require(script.Parent:WaitForChild("SVO"))

local Serialize = {}

local VERSION = "NVGN1"
local SVO_VERSION = "NVGNSVO1"
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

--------------------------------------------------------------------------
-- SVO
--
-- The octree has to reach runtime as well: width is deliberately NOT baked, it
-- is resolved per agent at query time from portal edge lengths TOGETHER WITH
-- solid queries against this tree. A bake without it is a mesh no pathfinder
-- can size an agent against.
--
-- ENCODING is a preorder walk, and it stores STRUCTURE ONLY. There is nothing
-- else to store: a node is solid, or it is subdivided, and the tree already
-- prunes empty children to nil and collapses eight solid octants into their
-- parent, so what is written is exactly the compressed tree.
--
--   '.'        this node is solid; do not descend
--   two hex    this node is subdivided; the byte is the 8-bit mask of which
--              octants are present, and they follow in ascending order
--   '-'        empty; only reachable at the root
--
-- Solid subtrees therefore cost ONE character however large they are, which is
-- the whole point of the octree being compressed in the first place. No
-- coordinates are written at all -- every node's cube is implied by its path
-- from the root, so position cannot drift on reload, by construction.
--
-- Kept in its own StringValue rather than appended to the mesh bake: it is an
-- order of magnitude bigger, and a consumer that only wants polygons and
-- portals should not have to parse it.

local HEX = "0123456789abcdef"

function Serialize.encodeSVO(tree: any): string
	local out: {string} = {}
	local nodes, solids = 0, 0

	local function walk(node)
		nodes += 1
		if node.solid then
			solids += 1
			out[#out + 1] = "."
			return
		end
		local ch = node.children
		if not ch then
			out[#out + 1] = "-"
			return
		end
		local mask = 0
		for i = 0, 7 do
			if ch[i] then mask += bit32.lshift(1, i) end
		end
		out[#out + 1] = string.sub(HEX, bit32.rshift(mask, 4) + 1, bit32.rshift(mask, 4) + 1)
			.. string.sub(HEX, bit32.band(mask, 15) + 1, bit32.band(mask, 15) + 1)
		for i = 0, 7 do
			if ch[i] then walk(ch[i]) end
		end
	end
	walk(tree.root)

	local body = table.concat(out)
	local header = table.concat({
		SVO_VERSION,
		string.format("root %s %s %s", fmt(tree.center.X), fmt(tree.center.Y), fmt(tree.center.Z)),
		string.format("half %s", fmt(tree.half)),
		string.format("leaf %s", fmt(tree.leaf)),
		string.format("depth %d", tree.maxDepth),
		string.format("counts %d %d %d", nodes, solids, #body),
		"tree",
	}, "\n")

	-- wrapped so the value is not one enormous line
	local wrapped: {string} = {}
	local width = 120
	for i = 1, #body, width do
		wrapped[#wrapped + 1] = string.sub(body, i, i + width - 1)
	end
	return header .. "\n" .. table.concat(wrapped, "\n") .. "\nend"
end

function Serialize.decodeSVO(text: string): any
	local lines = string.split(text, "\n")
	if lines[1] ~= SVO_VERSION then
		error("NVGN.Serialize: unknown SVO format '" .. tostring(lines[1]) .. "'")
	end
	local center, half, leaf, depth, expectNodes, expectSolids
	local bodyParts: {string} = {}
	local inTree = false
	for i = 2, #lines do
		local line = lines[i]
		if line == "end" then
			break
		elseif inTree then
			bodyParts[#bodyParts + 1] = line
		elseif line == "tree" then
			inTree = true
		else
			local f = string.split(line, " ")
			if f[1] == "root" then
				center = Vector3.new(tonumber(f[2]) or 0, tonumber(f[3]) or 0, tonumber(f[4]) or 0)
			elseif f[1] == "half" then half = tonumber(f[2])
			elseif f[1] == "leaf" then leaf = tonumber(f[2])
			elseif f[1] == "depth" then depth = tonumber(f[2])
			elseif f[1] == "counts" then
				expectNodes = tonumber(f[2]); expectSolids = tonumber(f[3])
			end
		end
	end
	local body = table.concat(bodyParts)

	local tree = SVO.new(center, half, leaf)
	-- maxDepth is derived in SVO.new from half and leaf; the stored value is
	-- authoritative, because a bake must reload as the tree that was written
	tree.maxDepth = depth or tree.maxDepth

	local pos = 1
	local nodes, solids = 0, 0
	local function read()
		local node = {}
		nodes += 1
		local ch = string.sub(body, pos, pos)
		if ch == "." then
			pos += 1
			node.solid = true
			solids += 1
			return node
		elseif ch == "-" then
			pos += 1
			return node
		end
		local mask = tonumber(string.sub(body, pos, pos + 1), 16) or 0
		pos += 2
		local children = {}
		node.children = children
		for i = 0, 7 do
			if bit32.band(bit32.rshift(mask, i), 1) == 1 then
				children[i] = read()
			end
		end
		return node
	end
	tree.root = read()

	tree.loadReport = {
		nodes = nodes, solids = solids,
		expectedNodes = expectNodes, expectedSolids = expectSolids,
		consumed = pos - 1, bodyLength = #body,
		ok = (nodes == expectNodes) and (solids == expectSolids) and (pos - 1 == #body),
	}
	return tree
end

-- STORAGE. Roblox caps a StringValue at 200,000 characters. The old test
-- scene's entire artefact was 142 KB, so one value per document was fine; this
-- map's mesh alone encodes to 2.3 MB and assignment simply throws.
--
-- Text over the cap is therefore split across numbered StringValues under a
-- Folder of the same name and reassembled on load. The split is by raw
-- character count, not by line, because the decoders parse the reassembled
-- document rather than the pieces -- so no chunk boundary has to be meaningful.
--
-- A document that fits is still written as a plain StringValue, so bakes
-- produced before this change load unchanged.
local CHUNK = 190000

local function writeText(host: Instance, name: string, text: string): Instance
	local existing = host:FindFirstChild(name)
	if existing then existing:Destroy() end
	if #text <= CHUNK then
		local sv = Instance.new("StringValue")
		sv.Name = name
		sv.Value = text
		sv.Parent = host
		return sv
	end
	local folder = Instance.new("Folder")
	folder.Name = name
	local n = 0
	for i = 1, #text, CHUNK do
		n += 1
		local part = Instance.new("StringValue")
		part.Name = tostring(n)
		part.Value = text:sub(i, i + CHUNK - 1)
		part.Parent = folder
	end
	-- the count is authoritative: a missing chunk must be an error, never a
	-- silently truncated navmesh
	folder:SetAttribute("Chunks", n)
	folder.Parent = host
	return folder
end

local function readText(host: Instance, name: string): string
	local node = host:FindFirstChild(name)
	if not node then
		error("NVGN.Serialize: no " .. name .. " under " .. host:GetFullName())
	end
	if node:IsA("StringValue") then return (node :: StringValue).Value end
	local n = node:GetAttribute("Chunks")
	if type(n) ~= "number" then
		error("NVGN.Serialize: " .. name .. " is a folder with no Chunks attribute")
	end
	local parts = table.create(n)
	for i = 1, n do
		local sv = node:FindFirstChild(tostring(i))
		if not sv or not sv:IsA("StringValue") then
			error(string.format("NVGN.Serialize: %s is missing chunk %d of %d", name, i, n))
		end
		parts[i] = (sv :: StringValue).Value
	end
	return table.concat(parts)
end

function Serialize.saveSVO(tree: any, parent: Instance?): Instance
	local host = parent or game:GetService("ServerStorage")
	return writeText(host, "NVGN_SVO", Serialize.encodeSVO(tree))
end

function Serialize.loadSVO(parent: Instance?): any
	local host = parent or game:GetService("ServerStorage")
	return Serialize.decodeSVO(readText(host, "NVGN_SVO"))
end

--------------------------------------------------------------------------

function Serialize.save(mesh: any, parent: Instance?): Instance
	local host = parent or game:GetService("ServerStorage")
	return writeText(host, "NVGN_Bake", Serialize.encode(mesh))
end

function Serialize.load(parent: Instance?): any
	local host = parent or game:GetService("ServerStorage")
	return Serialize.decode(readText(host, "NVGN_Bake"))
end

return Serialize
