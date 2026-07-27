--!strict
-- NVGN.Navmesh — the whole bake, assembled once.
--
-- Every stage until now has been able to build its own inputs (`Loops.build`
-- calls `Clean.build` calls `LocalGrid.build`), which is convenient while a
-- stage is being developed and wasteful once they are all real: running the
-- pipeline through `Polys.build` and then asking for clearance volumes rebuilt
-- the ENTIRE local grid a second time, because `Volumes.build` starts from
-- scratch too. This module builds each stage exactly once and hands the result
-- forward.
--
-- It is also where the two things a partition plus a portal graph still lack
-- get attached:
--
--   CLEARANCE. `Volumes` measures headroom and is validated, but nothing was
--   calling it, so no polygon carried a headroom number and a pathfinder had no
--   way to tell a crawl tunnel from open sky. Per Cocosulx's decision volumes do
--   NOT split polygons -- polygonization runs in abstraction of them -- so the
--   attachment is a per-polygon SCALAR: the lowest clearance any volume imposes
--   anywhere over that polygon's area. It is deliberately conservative, and it
--   is a PRE-FILTER, not the whole answer: a polygon partly covered by a low
--   volume is no longer atomic, so hard exclusion for tall agents uses this
--   scalar while mode selection (walk/crouch/crawl cost) works off the smoothed
--   corridor and the volume index directly.
--
--   math.huge means no volume reaches this polygon at all, i.e. its headroom is
--   at least the volume cap. It is not "unknown" and must not be treated as 0.
--
-- WHAT THIS IS NOT YET. There is no serialization here: the result is live
-- tables, rebuilt per call. Jump links do not exist, so polygons reachable only
-- by dropping or hopping are correctly disconnected. Destruction is unbuilt.

local LocalGrid = require(script.Parent:WaitForChild("LocalGrid"))
local Regions = require(script.Parent:WaitForChild("Regions"))
local Polys = require(script.Parent:WaitForChild("Polys"))
local Portals = require(script.Parent:WaitForChild("Portals"))
local Volumes = require(script.Parent:WaitForChild("Volumes"))

local Navmesh = {}

export type Config = {
	volumeBucket: number?,
	-- Reporting bands only. These are NOT baked as classes: the whole reason
	-- clearance is a measured number rather than a tier label is that an agent
	-- of any height must be servable without a re-bake.
	reportBands: {number}?,
}

local DEFAULT = {
	volumeBucket = 8,
	reportBands = { 1.5, 3, 4, 7 },
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

--------------------------------------------------------------------------

function Navmesh.build(cfg: any?)
	local c = merged(cfg)
	local t0 = os.clock()
	local timing = {}
	local function mark(name: string, from: number): number
		local now = os.clock()
		timing[name] = now - from
		return now
	end

	local t = os.clock()
	-- the SVO is kept, not discarded: width is resolved at query time against
	-- solid queries, so the tree is part of what has to reach runtime
	local data, floorData, tree, parts = LocalGrid.build(cfg)
	t = mark("localgrid", t)

	-- Regions replaces Clean + Loops. Those built a region as the floor's rim
	-- rectangle cut by emitted boundary lines, with the mask only SELECTING
	-- which faces survived -- so a face holding a live strip plus hundreds of
	-- studs of dead ground was kept whole, and a polygon covered a wall. A line
	-- partitions a face only if it CLOSES, and on authored geometry they mostly
	-- do not, which is why adding exact lines never fixed it.
	--
	-- Tracing the live-cell frontier instead makes closure free: the boundary of
	-- a set of cells is a closed loop by construction. The mask decides
	-- topology, geometry still decides every coordinate.
	local rres = Regions.fromLocal(data, cfg)
	t = mark("regions", t)

	local pres = Polys.fromLoops(rres, cfg)
	t = mark("polys", t)

	local ptres = Portals.fromPolys(pres, cfg)
	t = mark("portals", t)

	local vres = Volumes.fromLocal(data, cfg)
	local vidx = Volumes.index(vres, c.volumeBucket)
	t = mark("volumes", t)

	----------------------------------------------------------------
	-- attach clearance
	----------------------------------------------------------------
	local restricted, minSeen = 0, math.huge
	local bandCount: {[number]: number} = {}
	for _, b in ipairs(c.reportBands) do bandCount[b] = 0 end

	for _, p in ipairs(pres.polys) do
		local mc = Volumes.minClearanceOverPoly(vidx, p.verts, cfg)
		p.minClearance = mc
		if mc < math.huge then
			restricted += 1
			minSeen = math.min(minSeen, mc)
			for _, b in ipairs(c.reportBands) do
				if mc < b then bandCount[b] += 1 end
			end
		end
	end
	t = mark("clearance", t)

	local stats = {
		polys = #pres.polys,
		portals = #ptres.portals,
		volumes = #vres.volumes,
		restrictedPolys = restricted,
		openPolys = #pres.polys - restricted,
		minClearance = minSeen,
		bands = bandCount,
		seconds = os.clock() - t0,
		timing = timing,
	}

	return {
		polys = pres.polys,
		portals = ptres.portals,
		neighbours = ptres.neighbours,
		volumes = vres.volumes,
		volumeIndex = vidx,
		svo = tree,
		stats = stats,
		-- stage results kept for debugging and for the viz helpers
		stages = { data = data, floor = floorData, parts = parts, svo = tree,
			regions = rres, polys = pres, portals = ptres, volumes = vres },
	}
end

--------------------------------------------------------------------------

-- Polygons only. Everything else stays off unless asked for, because the
-- polygons are what is being judged and the overlays bury them.
function Navmesh.visualize(res: any, parent: Instance?)
	return Polys.visualize(res.stages.polys, parent)
end

return Navmesh
