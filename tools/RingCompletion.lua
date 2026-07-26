--!strict
-- tools/RingCompletion — EXPERIMENT, not a pipeline stage.
--
-- Kept in tools/ deliberately: this is a measurement harness for one hypothesis
-- about why polygons cover solid geometry on a large authored map. It is not
-- wired into the bake and nothing requires it.
--
-- ─────────────────────────────────────────────────────────────────────────────
-- THE PROBLEM
--
-- On a 12,571-part multi-storey map, paths visibly walk through walls. That is
-- not a pathfinder bug: sampling path points with an overlap probe (rays are
-- useless here -- waypoints sit exactly on boundary faces and trip the
-- inside-origin trap) puts 87.6% of the clipping INSIDE the corridor's own
-- polygons. The mesh covers solid space and the pathfinder walks it faithfully.
--
-- Cocosulx's own validation metric shows it in one number: summed region area
-- 723,118 vs 586,413 live cells = +23.31% slack. On the old 178-part scene the
-- same metric read -0.1%.
--
-- ─────────────────────────────────────────────────────────────────────────────
-- THE CAUSE
--
-- Clean v2 measures a boundary by firing a ray from a live node into a blocked
-- direction -- but it only does so at a FRONTIER pair, i.e. where the neighbour
-- node is dead or absent. A blocker face that does not happen to face a frontier
-- is therefore never rayed and never emitted.
--
-- The ends of walls are exactly that case: floor continues around them, so both
-- neighbours are live and no ray is ever fired at the end cap. Measured on this
-- map's 843 thin tall walls:
--
--     311  emit no boundary at all
--     235  of the remaining 532 (44.2%) emit ONLY long faces, no end cap
--     end caps are 26.3% of emitted wall length
--
-- A ring that is open on two ends does not partition anything: the face walk in
-- Loops runs around the free end, one face keeps both the live and the dead
-- side, and Polys emits a polygon spanning the wall.
--
-- This also explains the 3,292 dangling endpoints Clean reports, and why they
-- are NOT a budget-tuning problem -- only 15% have a partner crossing within the
-- current 0.75 outward budget, 42% have nothing within 3 studs, 6% have nothing
-- at all. Those lines have no partner because the partner was never emitted.
--
-- ─────────────────────────────────────────────────────────────────────────────
-- THE EXPERIMENT
--
-- For every (floor, blocker) pair Clean ALREADY emitted a wall for, add the
-- blocker's remaining vertical faces as lines: intersect each face plane with
-- the floor plane and clip to the face rectangle. This is v1's exact
-- plane-intersection construction, applied only where v2 left a hole. No
-- lattice, no invented vertices, no Steiner points -- the line comes from the
-- part's own geometry, which is the project's standing rule for boundary truth.
--
-- Faces near-parallel to the floor are skipped (tops and bottoms are not
-- boundary), and a face is skipped when an existing edge already lies on that
-- line, so measured geometry always wins over synthesised.
--
-- ─────────────────────────────────────────────────────────────────────────────
-- MEASURED (whole map, Loops + Polys re-run on the augmented edge set)
--
--   edges            22,428 -> 37,012   (14,584 added from 25,636 faces)
--   regions           4,006 ->  4,036
--   polys             5,623 ->  6,054
--   AREA SLACK      +23.31% -> +14.33%   (+136,705 -> +84,045 studs^2)
--   worstSlack       18,651 -> 14,935
--
-- So it removes 38% of the over-claim. But polygon interior sitting in solid
-- did NOT improve on its own (21.6% -> 22.2%), because separating the dead area
-- into its own faces does not delete them -- zero-live polygons nearly doubled,
-- 291 -> 457. Loops discards empty FACES, but Polys then splits regions
-- (4,006 -> 6,054) and never re-applies the live-cell mask.
--
-- Re-applying the mask after polygonization is what converts the separation
-- into a fix:
--
--   ring completion only                     21.6% -> 22.2% interior in solid
--   + drop zero-live polygons                        17.7%   (457 polys, 21,002 studs^2)
--   + require 25% live coverage                      17.0%   (596 polys, 57,088 studs^2)
--   + require 50% live coverage                      16.1%   (826 polys, 103,384 studs^2)
--
-- ─────────────────────────────────────────────────────────────────────────────
-- WHY THIS IS NOT MERGED
--
-- 1. It is a partial fix. 17.7% of polygon interior still sits in solid. The
--    311 walls emitting NO boundary at all are untouched by this, and buried
--    double-cover surfaces (a floor 0.25 studs under the slab that covers it)
--    are a separate defect.
-- 2. Polys goes from 5.35s to 267s on the augmented edge set. Its per-region
--    quadratics are fine at 22k edges and not at 37k. Same family as the
--    closeFloor quadratics already noted as debt.
-- 3. The real home for this is Clean's emission, not a post-pass. Doing it
--    properly means deciding whether v2 keeps frontier-only sampling at all,
--    which is Cocosulx's architectural call, not a patch.
-- 4. The mask-after-polygonization half belongs in Polys and is independently
--    useful; it is deliberately left as a measurement here rather than smuggled
--    into the stage.

local RingCompletion = {}

-- Returns a NEW edge list (the input is not mutated), plus counts.
function RingCompletion.augment(clean: any, data: any, cfg: any?): ({any}, number, number)
	local c = cfg or {}
	local eps = c.eps or 0.15
	local minLen = c.minLen or 0.5
	-- faces this close to parallel with the floor are tops/bottoms, not boundary
	local parallelDot = c.parallelDot or 0.85

	-- blockers Clean already proved cut each floor
	local byFloor: {[any]: {[any]: {any}}} = {}
	for _, e in ipairs(clean.edges) do
		if e.source and e.floor and e.source ~= e.floor and e.class == "wall" then
			local m = byFloor[e.floor]
			if not m then m = {}; byFloor[e.floor] = m end
			local l = m[e.source]
			if not l then l = {}; m[e.source] = l end
			l[#l + 1] = e
		end
	end

	local out: {any} = {}
	for _, e in ipairs(clean.edges) do out[#out + 1] = e end
	local added, considered = 0, 0

	for floor, blockers in pairs(byFloor) do
		local g = data.grids[floor]
		if g and g.center and g.n then
			local Fn, P0 = g.n, g.center
			for blocker, existing in pairs(blockers) do
				-- blocks only: a Union's faces are not its bounding box, and
				-- using Size there would fabricate boundary that is not real
				if blocker:IsA("Part") and blocker.Shape == Enum.PartType.Block then
					local cf, sz = blocker.CFrame, blocker.Size
					local axes = {
						{ d = cf.RightVector, e = sz.X * 0.5 },
						{ d = cf.UpVector,    e = sz.Y * 0.5 },
						{ d = cf.LookVector,  e = sz.Z * 0.5 },
					}
					for fi = 1, 3 do
						for sgn = -1, 1, 2 do
							local m = axes[fi].d * sgn
							if math.abs(m:Dot(Fn)) < parallelDot then
								considered += 1
								local fcenter = cf.Position + m * axes[fi].e
								local i1 = (fi % 3) + 1
								local i2 = ((fi + 1) % 3) + 1
								local a1, e1 = axes[i1].d, axes[i1].e
								local a2, e2 = axes[i2].d, axes[i2].e
								-- the face rectangle is fcenter + a1*s + a2*t.
								-- Fn . (that - P0) = 0 is a line in (s, t);
								-- clip it to |s|<=e1, |t|<=e2.
								local c0 = Fn:Dot(fcenter - P0)
								local c1 = Fn:Dot(a1)
								local c2 = Fn:Dot(a2)
								local pts: {Vector3} = {}
								if math.abs(c2) > 1e-6 then
									for _, s in ipairs({ -e1, e1 }) do
										local t = -(c0 + c1 * s) / c2
										if math.abs(t) <= e2 + 1e-6 then
											pts[#pts + 1] = fcenter + a1 * s + a2 * t
										end
									end
								end
								if math.abs(c1) > 1e-6 then
									for _, t in ipairs({ -e2, e2 }) do
										local s = -(c0 + c2 * t) / c1
										if math.abs(s) <= e1 + 1e-6 then
											pts[#pts + 1] = fcenter + a1 * s + a2 * t
										end
									end
								end
								if #pts >= 2 then
									local A, B, best = nil, nil, -1
									for i = 1, #pts do
										for j = i + 1, #pts do
											local d = (pts[i] - pts[j]).Magnitude
											if d > best then best = d; A = pts[i]; B = pts[j] end
										end
									end
									if best > minLen and A and B then
										local D = ((B :: Vector3) - (A :: Vector3)).Unit
										-- measured geometry wins: skip a face an
										-- existing edge already lies on
										local dup = false
										for _, ex in ipairs(existing) do
											if math.abs(ex.D:Dot(D)) > 0.999 then
												local off = ex.X0 - (A :: Vector3)
												if (off - D * off:Dot(D)).Magnitude < eps then
													dup = true
													break
												end
											end
										end
										if not dup then
											local outDir = -m
											outDir = outDir - Fn * outDir:Dot(Fn)
											if outDir.Magnitude > 1e-4 then
												outDir = outDir.Unit
											else
												outDir = Vector3.zero
											end
											out[#out + 1] = {
												a = A, b = B, class = "wall",
												floor = floor, source = blocker,
												outDir = outDir,
												lineId = string.format("cap:%d:%d:%d",
													blocker:GetDebugId(), fi, sgn),
												X0 = A, D = D, samples = 0,
												closedA = false, closedB = false,
												synthetic = true,
											}
											added += 1
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end
	return out, added, considered
end

return RingCompletion
