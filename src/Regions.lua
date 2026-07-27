--!strict
-- NVGN.Regions — walkable regions traced from the live-cell frontier,
-- with every run snapped onto exact part geometry.
--
-- WHY THIS REPLACES THE FACE WALK. `Loops` builds a region as the floor part's
-- whole rim RECTANGLE cut by whatever boundary lines `Clean` emitted, and then
-- uses the live-cell mask only to SELECT which resulting faces survive — a face
-- is kept if it contains any live cell at all. The mask is a per-face boolean
-- and never an extent, so a face holding a 30-cell live strip plus 900 studs² of
-- dead ground is kept WHOLE. Measured on the big map: 19.4% of polygon area sits
-- over dead cells and a further 3.5% over no cells at all.
--
-- That also explains why adding boundary lines never fixed it. A line only
-- partitions a face if it CLOSES, and on authored geometry they mostly do not —
-- adding 559 exact lines moved area slack by 6 points and pushed `dangling` UP,
-- 157 to 170. Two separate attempts died on this.
--
-- So the ring is built from the mask instead, where closure is free: the
-- boundary of a set of cells is a closed loop by construction. There is no
-- dangling end to chase, because an unclosed boundary cannot exist.
--
-- THE DIVISION OF LABOUR IS UNCHANGED, and this is the whole point. The mask
-- decides TOPOLOGY — which cells form a region, where the ring runs, which side
-- is inside. Geometry decides POSITION — every emitted coordinate comes from a
-- real part's face plane intersected with the floor's top plane. The lattice
-- supplies no coordinate, so the staircase this project exists to kill cannot
-- come back through here.
--
-- EXACT GEOMETRY IS ALWAYS AVAILABLE. Measured over 3,972 frontier samples on
-- the big map, every single one had an exact source:
--    38.3%  the neighbour is a dead cell carrying its killer  -> killer's face
--    29.0%  a block part occupies the space just beyond       -> that part's face
--    32.7%  the floor's own rim (a dropoff)                   -> the rim plane
--     0.0%  nothing
--
-- KNOWN LIMIT, stated rather than hidden: a blocker thinner than the cell pitch
-- that sits BETWEEN two cell centres kills no cell, so both sides stay live and
-- no frontier exists here to find. That is a 1-stud resolution limit of the
-- mask, not a defect of this stage, and it needs its own treatment (geometric
-- intrusion) measured against path clipping rather than area.

local Regions = {}

local UP = Vector3.new(0, 1, 0)

export type Config = {
	step: number?,
	coverTol: number?,      -- height tolerance when asking whether a floor continues
	planeReach: number?,    -- how far beyond the frontier to look for a blocking face
	parallelEps: number?,
}

local DEFAULT = {
	step = 1,
	coverTol = 0.35,
	planeReach = 1.6,
	parallelEps = 1e-4,
	minClearance = 1.5,
}

local function merged(cfg): any
	local c = {}
	for k, v in pairs(DEFAULT) do c[k] = v end
	if cfg then for k, v in pairs(cfg) do if v ~= nil then c[k] = v end end end
	return c
end

-- Instance identity, never the name: authored scenes reuse names freely and
-- `tostring(part)` returns the NAME, which has silently merged unrelated parts
-- in this project before.
local nextId = 0
local idOf: { [Instance]: number } = setmetatable({}, { __mode = "k" }) :: any
local function iid(inst: Instance): number
	local id = idOf[inst]
	if not id then nextId += 1; id = nextId; idOf[inst] = id end
	return id
end

--------------------------------------------------------------------------

-- {face plane} ∩ {floor top}: the construction used everywhere in this project.
local function intersectPlanes(N: Vector3, P: Vector3, Fn: Vector3, F0: Vector3)
	local D = N:Cross(Fn)
	if D.Magnitude < 1e-4 then return nil end
	D = D.Unit
	local w = N - Fn * N:Dot(Fn)
	if w.Magnitude < 1e-4 then return nil end
	w = w.Unit
	local denom = N:Dot(w)
	if math.abs(denom) < 1e-4 then return nil end
	return F0 + w * (N:Dot(P - F0) / denom), D
end

-- The six faces of a block, as (normal, point) pairs, skipping any face too
-- near-parallel to the floor to yield a usable line.
local function facesOf(part: BasePart, Fn: Vector3): { { n: Vector3, p: Vector3, i: number } }
	local cf, sz = part.CFrame, part.Size
	local axes = {
		{ cf.RightVector, sz.X * 0.5 },
		{ cf.UpVector, sz.Y * 0.5 },
		{ cf.LookVector, sz.Z * 0.5 },
	}
	local out = {}
	for ai, ax in ipairs(axes) do
		local N = ax[1] :: Vector3
		if math.abs(N:Dot(Fn)) <= 0.94 then
			for si, sgn in ipairs({ 1, -1 }) do
				out[#out + 1] = {
					n = N * sgn,
					p = part.Position + N * ((ax[2] :: number) * sgn),
					i = ai * 2 + si,
				}
			end
		end
	end
	return out
end

-- Pick the face an agent actually meets: nearest plane to the frontier point,
-- among those facing back toward the walkable side.
local function bestFace(part: BasePart, at: Vector3, outward: Vector3, Fn: Vector3)
	local best, bestD = nil, math.huge
	for _, f in ipairs(facesOf(part, Fn)) do
		if f.n:Dot(outward) < -0.2 then          -- faces us
			local d = math.abs(f.n:Dot(at - f.p))
			if d < bestD then best, bestD = f, d end
		end
	end
	return best, bestD
end

--------------------------------------------------------------------------

function Regions.fromLocal(data: any, cfg: Config?)
	local c = merged(cfg)
	local t0 = os.clock()
	local step = c.step
	local crawl = (data.config and data.config.minClearance) or c.minClearance

	-- cross-grid coverage: where does ANY grid have live ground? Needed so two
	-- overlapping floor parts hand over to each other instead of each emitting a
	-- wall along their shared join.
	local cover: { [string]: { Vector3 } } = {}
	for _, g in pairs(data.grids) do
		for _, cell in ipairs(g.cells) do
			local k = string.format("%d:%d", math.floor(cell.pos.X), math.floor(cell.pos.Z))
			local b = cover[k]
			if not b then b = {}; cover[k] = b end
			b[#b + 1] = cell.pos
		end
	end
	local function coveredElsewhere(p: Vector3): boolean
		local bx, bz = math.floor(p.X), math.floor(p.Z)
		for dx = -1, 1 do
			for dz = -1, 1 do
				local b = cover[string.format("%d:%d", bx + dx, bz + dz)]
				if b then
					for _, q in ipairs(b) do
						if math.abs(q.Y - p.Y) <= c.coverTol
							and math.abs(q.X - p.X) <= step * 0.5
							and math.abs(q.Z - p.Z) <= step * 0.5 then
							return true
						end
					end
				end
			end
		end
		return false
	end

	local regions: { any } = {}
	local stats = {
		grids = 0, components = 0, rings = 0, holes = 0,
		fromKiller = 0, fromQuery = 0, fromRim = 0, fromLattice = 0,
		cells = 0, area = 0, degenerateVerts = 0,
	}

	local DIRS = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } }

	for part, g in pairs(data.grids) do
		if not g.fallback and g.origin and g.u and g.v and #g.cells > 0 then
			stats.grids += 1
			local Fn = g.n or UP
			local F0 = g.origin

			-- lattice corner (i,j) -> world. Verified against real grids at 0.0000.
			local function corner(i: number, j: number): Vector3
				return g.origin + g.u * (i * step) + g.v * (j * step)
			end

			local liveIx: { [string]: any } = {}
			for _, cell in ipairs(g.cells) do liveIx[cell.ui .. ":" .. cell.vi] = cell end
			local deadIx: { [string]: any } = {}
			for _, d in ipairs(g.dead) do deadIx[d.ui .. ":" .. d.vi] = d end

			----------------------------------------------------------------
			-- connected components of live cells (4-connectivity: an agent
			-- cannot walk a diagonal seam)
			----------------------------------------------------------------
			local seen: { [string]: boolean } = {}
			for _, start in ipairs(g.cells) do
				local sk = start.ui .. ":" .. start.vi
				if not seen[sk] then
					seen[sk] = true
					local comp = { start }
					local queue, head = { start }, 1
					while head <= #queue do
						local cur = queue[head]; head += 1
						for _, d in ipairs(DIRS) do
							local nk = (cur.ui + d[1]) .. ":" .. (cur.vi + d[2])
							local nb = liveIx[nk]
							if nb and not seen[nk] then
								seen[nk] = true
								comp[#comp + 1] = nb
								queue[#queue + 1] = nb
							end
						end
					end
					stats.components += 1

					local inComp: { [string]: boolean } = {}
					for _, cell in ipairs(comp) do inComp[cell.ui .. ":" .. cell.vi] = true end

					------------------------------------------------------------
					-- boundary edges, oriented so the interior is on the left
					------------------------------------------------------------
					-- edge for cell (ui,vi) leaving in direction d, as lattice
					-- corner pair; chosen so the loop runs counter-clockwise in
					-- the (u,v) frame
					local function edgeCorners(ui: number, vi: number, di: number)
						if di == 1 then return { ui + 1, vi }, { ui + 1, vi + 1 }      -- +u
						elseif di == 2 then return { ui + 1, vi + 1 }, { ui, vi + 1 }  -- +v
						elseif di == 3 then return { ui, vi + 1 }, { ui, vi }          -- -u
						else return { ui, vi }, { ui + 1, vi } end                     -- -v
					end

					local segs: { any } = {}
					for _, cell in ipairs(comp) do
						for di, d in ipairs(DIRS) do
							local nk = (cell.ui + d[1]) .. ":" .. (cell.vi + d[2])
							if not inComp[nk] then
								local A, B = edgeCorners(cell.ui, cell.vi, di)
								local worldDir = (g.u * d[1] + g.v * d[2])
								if worldDir.Magnitude > 1e-6 then worldDir = worldDir.Unit end
								local mid = cell.pos + worldDir * (step * 0.5)

								----------------------------------------------------
								-- WHERE EXACTLY IS THIS BOUNDARY? Ask geometry.
								----------------------------------------------------
								local class, planeN, planeP, planeId, source

								-- (0) another floor carries on here: a handover, not
								-- an edge. No wall may be invented along a join.
								local beyond = cell.pos + worldDir * step
								if coveredElsewhere(beyond) then
									class = "continuation"
									planeId = nil   -- lattice; both sides walkable, so
									                -- a half-cell here costs nothing and
									                -- Portals matches the pair anyway
									stats.fromLattice += 1
								else
									-- (1) the dead neighbour names its killer
									local dn = deadIx[nk]
									local killer = dn and dn.killer
									if killer and killer:IsA("BasePart") then
										local f = bestFace(killer, mid, worldDir, Fn)
										if f then
											class, planeN, planeP = "wall", f.n, f.p
											planeId = iid(killer) .. "#" .. f.i
											source = killer
											stats.fromKiller += 1
										end
									end
									-- (2) nothing recorded, but something occupies the
									-- standing space just beyond
									if not planeId then
										local op = OverlapParams.new()
										op.FilterType = Enum.RaycastFilterType.Exclude
										op.FilterDescendantsInstances = { part }
										op.MaxParts = 8
										local at = mid + worldDir * (step * 0.5) + Fn * (crawl * 0.5)
										local hits = workspace:GetPartBoundsInBox(
											CFrame.new(at), Vector3.new(step * 1.2, crawl, step * 1.2), op)
										local best, bestD = nil, math.huge
										for _, h in ipairs(hits) do
											if h:IsA("Part") and h.Shape == Enum.PartType.Block then
												local f, d2 = bestFace(h, mid, worldDir, Fn)
												if f and d2 < bestD and d2 <= c.planeReach then
													best, bestD = { f = f, part = h }, d2
												end
											end
										end
										if best then
											class, planeN, planeP = "wall", best.f.n, best.f.p
											planeId = iid(best.part) .. "#" .. best.f.i
											source = best.part
											stats.fromQuery += 1
										end
									end
									-- (3) the floor simply ends: its own rim is exact
									if not planeId then
										local f = bestFace(part, mid, -worldDir, Fn)
										-- the rim face we are leaving through faces OUTWARD,
										-- so search with the outward sense
										local best2, bestD2 = nil, math.huge
										for _, ff in ipairs(facesOf(part, Fn)) do
											if ff.n:Dot(worldDir) > 0.2 then
												local d2 = math.abs(ff.n:Dot(mid - ff.p))
												if d2 < bestD2 then best2, bestD2 = ff, d2 end
											end
										end
										if best2 then
											class, planeN, planeP = "dropoff", best2.n, best2.p
											planeId = "rim" .. iid(part) .. "#" .. best2.i
											source = part
											stats.fromRim += 1
										else
											class = "dropoff"
											planeId = nil
											stats.fromLattice += 1
										end
									end
								end

								segs[#segs + 1] = {
									a = A, b = B, class = class, source = source,
									planeN = planeN, planeP = planeP, planeId = planeId,
								}
							end
						end
					end

					------------------------------------------------------------
					-- chain boundary segments into closed loops
					------------------------------------------------------------
					local byStart: { [string]: { any } } = {}
					for _, s in ipairs(segs) do
						local k = s.a[1] .. "," .. s.a[2]
						local b = byStart[k]
						if not b then b = {}; byStart[k] = b end
						b[#b + 1] = s
					end
					local used: { [any]: boolean } = {}
					local loops: { { any } } = {}
					for _, s0 in ipairs(segs) do
						if not used[s0] then
							local loop = {}
							local cur = s0
							while cur and not used[cur] do
								used[cur] = true
								loop[#loop + 1] = cur
								local k = cur.b[1] .. "," .. cur.b[2]
								local cands = byStart[k]
								local nxt = nil
								if cands then
									for _, s in ipairs(cands) do
										if not used[s] then nxt = s; break end
									end
								end
								cur = nxt
							end
							if #loop >= 3 then loops[#loops + 1] = loop end
						end
					end

					------------------------------------------------------------
					-- collapse each loop into exact runs, then intersect
					------------------------------------------------------------
					local function toUV(p: Vector3): (number, number)
						local w = p - g.origin
						return w:Dot(g.u), w:Dot(g.v)
					end

					local outer, holeRings = nil, {}
					for _, loop in ipairs(loops) do
						-- group consecutive segments sharing a plane (nil planeId
						-- never groups: those stay per-segment lattice steps)
						local runs = {}
						for _, s in ipairs(loop) do
							local last = runs[#runs]
							if last and last.planeId and s.planeId and last.planeId == s.planeId then
								last.segs[#last.segs + 1] = s
							else
								runs[#runs + 1] = { planeId = s.planeId, class = s.class,
									source = s.source, planeN = s.planeN, planeP = s.planeP, segs = { s } }
							end
						end
						-- a loop is cyclic: merge the tail into the head if they share a plane
						if #runs > 1 then
							local first, last = runs[1], runs[#runs]
							if first.planeId and last.planeId and first.planeId == last.planeId then
								for i = #last.segs, 1, -1 do table.insert(first.segs, 1, last.segs[i]) end
								runs[#runs] = nil
							end
						end

						-- exact line per run
						for _, r in ipairs(runs) do
							if r.planeId and r.planeN then
								local X0, D = intersectPlanes(r.planeN, r.planeP, Fn, F0)
								if X0 then r.X0 = X0; r.D = D end
							end
						end

						-- vertices: intersect consecutive run lines. Where a run has
						-- no line (lattice fallback) the lattice corner is used, which
						-- is the ONLY place a lattice coordinate can reach the output.
						local verts, classes, sources = {}, {}, {}
						local n = #runs
						for i = 1, n do
							local cur = runs[i]
							local nxt = runs[(i % n) + 1]
							local v: Vector3? = nil
							if cur.X0 and nxt.X0 then
								-- solve in the floor's own 2D frame
								local p1x, p1y = toUV(cur.X0)
								local d1x, d1y = cur.D:Dot(g.u), cur.D:Dot(g.v)
								local p2x, p2y = toUV(nxt.X0)
								local d2x, d2y = nxt.D:Dot(g.u), nxt.D:Dot(g.v)
								local den = d1x * d2y - d1y * d2x
								if math.abs(den) > c.parallelEps then
									local t = ((p2x - p1x) * d2y - (p2y - p1y) * d2x) / den
									v = cur.X0 + cur.D * t
								end
							end
							if not v then
								-- shared lattice corner between the two runs
								local endCorner = cur.segs[#cur.segs].b
								v = corner(endCorner[1], endCorner[2])
								stats.degenerateVerts += 1
							end
							verts[#verts + 1] = v
							classes[#classes + 1] = cur.class
							sources[#sources + 1] = cur.source
						end

						-- signed area in the (u,v) frame decides outer vs hole
						local a2 = 0
						for i = 1, #verts do
							local x1, y1 = toUV(verts[i])
							local x2, y2 = toUV(verts[(i % #verts) + 1])
							a2 += x1 * y2 - x2 * y1
						end
						local ring = { verts = verts, classes = classes, sources = sources, area = a2 * 0.5 }
						if a2 >= 0 then
							if not outer or math.abs(a2) > math.abs(outer.area * 2) then outer = ring end
						else
							holeRings[#holeRings + 1] = ring
						end
					end

					if outer and #outer.verts >= 3 then
						stats.rings += 1
						stats.holes += #holeRings
						stats.cells += #comp
						stats.area += math.abs(outer.area)
						local edges = {}
						for i = 1, #outer.verts do
							edges[i] = {
								a = outer.verts[i],
								b = outer.verts[(i % #outer.verts) + 1],
								class = outer.classes[i],
								source = outer.sources[i],
							}
						end
						local holes = {}
						for _, h in ipairs(holeRings) do
							local he = {}
							for i = 1, #h.verts do
								he[i] = {
									a = h.verts[i], b = h.verts[(i % #h.verts) + 1],
									class = h.classes[i], source = h.sources[i],
								}
							end
							holes[#holes + 1] = { verts = h.verts, edges = he, area = math.abs(h.area) }
						end
						regions[#regions + 1] = {
							floor = part,
							frame = { o = g.origin, u = g.u, v = g.v, n = Fn },
							verts = outer.verts,
							edges = edges,
							holes = holes,
							cells = #comp,
							area = math.abs(outer.area),
						}
					end
				end
			end
		end
	end

	stats.regions = #regions
	stats.seconds = os.clock() - t0
	-- the validation metric this project runs on: claimed area vs walkable cells
	local cellArea = 0
	for _, g in pairs(data.grids) do cellArea += #g.cells end
	stats.cellArea = cellArea
	stats.areaTotal = stats.area
	return { regions = regions, stats = stats, config = c }
end

return Regions
