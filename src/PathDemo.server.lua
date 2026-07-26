--!strict
-- NVGN_PathDemo — draws a live path from the part named "Start" to each player.
--
-- A Script, so it runs on Play and does nothing in edit mode. Its whole purpose
-- is to answer "is the bake usable?" with something you can watch, so it loads
-- the SERIALIZED bake rather than building one -- if this draws a path, the
-- bake round-tripped, the portal graph reconnected, and the geometry is sane.
--
-- It reuses its parts instead of rebuilding them every frame: at 10 Hz across a
-- long route that is thousands of Instance creations a second otherwise.
--
-- Debug parts are CanQuery and CanCollide off. A visible part that answers
-- spatial queries would be picked up by the next bake as walkable floor, which
-- this project has been bitten by twice.

local Players = game:GetService("Players")
local RunService = game:GetService("RunService")
local ServerStorage = game:GetService("ServerStorage")
local ServerScriptService = game:GetService("ServerScriptService")

local NVGN = ServerScriptService:WaitForChild("NVGN")
local Pathfinder = require(NVGN:WaitForChild("Pathfinder"))

local UPDATE_HZ = 10
local MOVE_EPS = 1.5      -- studs the target must move before we re-path
local LIFT = 1.5          -- draw the line this far above the surface

local OK_COLOUR = Color3.new(0.2, 1, 0.4)
local FAIL_COLOUR = Color3.new(1, 0.3, 0.2)

local ok, mesh = pcall(function() return Pathfinder.fromBake(ServerStorage) end)
if not ok then
	warn("[NVGN_PathDemo] no bake to load: " .. tostring(mesh))
	return
end
print(string.format("[NVGN_PathDemo] loaded bake: %d polys, %d portals", #mesh.polys, #mesh.portals))

local startPart = workspace:FindFirstChild("Start")
if not startPart or not startPart:IsA("BasePart") then
	warn("[NVGN_PathDemo] no BasePart named 'Start' in Workspace")
	return
end

local root = Instance.new("Folder")
root.Name = "NVGN_PathDemo"
root.Parent = workspace

local pool: {BasePart} = {}
local used = 0

local function segment(a: Vector3, b: Vector3, colour: Color3, thickness: number)
	used += 1
	local bar = pool[used]
	if not bar then
		bar = Instance.new("Part")
		bar.Anchored = true
		bar.CanCollide = false
		bar.CanQuery = false
		bar.CanTouch = false
		bar.Material = Enum.Material.Neon
		bar.Parent = root
		pool[used] = bar
	end
	local d = b - a
	local len = d.Magnitude
	if len < 1e-3 then
		bar.Transparency = 1
		return
	end
	bar.Transparency = 0
	bar.Size = Vector3.new(len, thickness, thickness)
	bar.Color = colour
	bar.CFrame = CFrame.lookAt(a + d * 0.5, b) * CFrame.Angles(0, math.pi / 2, 0)
end

local function endFrame()
	for i = used + 1, #pool do pool[i].Transparency = 1 end
	used = 0
end

local lastTarget: Vector3? = nil
local accum = 0

RunService.Heartbeat:Connect(function(dt)
	accum += dt
	if accum < 1 / UPDATE_HZ then return end
	accum = 0

	local target: Vector3? = nil
	for _, plr in ipairs(Players:GetPlayers()) do
		local ch = plr.Character
		local hrp = ch and ch:FindFirstChild("HumanoidRootPart")
		if hrp then target = (hrp :: BasePart).Position; break end
	end
	if not target then return end

	-- only recompute when the player has actually gone somewhere
	if lastTarget and (target - lastTarget).Magnitude < MOVE_EPS then return end
	lastTarget = target

	local pts, why = Pathfinder.find(mesh, startPart.Position, target)
	if pts then
		for i = 2, #pts do
			segment(pts[i - 1] + Vector3.new(0, LIFT, 0), pts[i] + Vector3.new(0, LIFT, 0), OK_COLOUR, 0.6)
		end
	else
		-- Draw the failure rather than nothing. "No route" is a real answer from
		-- an honest mesh -- an isolated rooftop has no walkable path to the
		-- ground until jump links exist -- and a blank screen would look like a
		-- crash instead of a result.
		segment(startPart.Position, target, FAIL_COLOUR, 0.3)
		if why ~= nil then
			root:SetAttribute("LastFailure", why)
		end
	end
	endFrame()
end)
