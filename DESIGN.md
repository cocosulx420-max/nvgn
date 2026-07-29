# nvgn — Navmesh Generator Design

A baked navmesh generator for Roblox with emergent, physics-driven destruction.

> **Status of this document.** This supersedes the earlier design (kept as
> `OLDMETHODDESIGN.md`). Two things changed:
>
> 1. **Boundary extraction is fully replaced.** Boundaries are no longer
>    derived by interpreting wall faces. See
>    [Boundary extraction](#boundary-extraction).
> 2. **Destruction is emergent, not authored.** The old design assumed a
>    finite, known destructible set whose post-destruction topologies could all
>    be pre-baked. That is false — players can level a building or a town. Every
>    pre-baked destruction mechanism in the old document is dead. The navmesh
>    side is **deferred** until the destruction system exists. See
>    [Destruction (deferred)](#destruction-deferred).
>
> The substrate, floor extraction, agent model, and polygon optimization are
> unchanged and validated.
>
> **Revision (boundary extraction).** Four corrections since the first draft of
> this method, all inside the boundary pipeline:
>
> - Contours are traced per **connected component** (step-tolerance adjacency),
>   not per height layer. Balconies no longer poison the floor below them.
> - The fit is **inward-biased by construction**. The old offset *budget* did
>   not close; this replaces a probabilistic bound with an exact one.
> - Miter joins take a **miter limit with bevel fallback**. Acute corners
>   otherwise spike arbitrarily far out.
> - The narrow-region branch is **graded**, not binary, and union-find is
>   **mandatory** — it is the only severance detector in the pipeline.

## Guiding constraints

- **Bake everything expensive.** The client should never pay for navmesh generation at runtime. Long bake times are acceptable; runtime hitches are not.
- **Navmesh is polygonal**, optimized for readability and pathfinding speed: large convex polygons for open areas, small polygons only for tricky geometry, no long thin slivers.
- **Simpler and always correct beats detailed and messy.** Where the two conflict, take the coarser result. Errors must be conservative — never open a route that does not exist.
- **Sample geometry, do not interpret it.** A raycast asks "is there a surface here" and answers uniformly for a Block, a Union, a MeshPart, or overlapping parts. Reading faces, normals, and CFrames does not. Every stage downstream of sampling inherits the robustness of sampling.
- **Destruction is emergent.** Post-destruction topology cannot be enumerated in advance at any N. Whatever the navmesh does about it must work on geometry nobody authored.

## Substrate

1. **Sparse Voxel Octree (SVO)** gives a fast solid/empty read of the world. Also the future substrate for flying NPCs. Implemented in `src/SVO.lua` (`ServerScriptService.NVGN.SVO` in-place).
2. A **surface/voxel pass** over the SVO yields:
   - **floor** — walkable surface locations
   - **clearance** — empty vertical space above a floor voxel before hitting a ceiling/obstacle

   (Width is *not* produced here — see [Width](#width).)

### SVO implementation notes (validated)

- **Build = OBB rasterization.** Each solid part's oriented bounding box is rasterized into the octree; nodes fully inside an OBB collapse to a single solid leaf, nodes on the surface subdivide to `leafSize`. Rotated/intersecting parts voxelize cleanly — confirmed on the `project nuhh` test scene.
- **Leaf size = 1 stud.** Test scene (177 parts) builds in ~0.9 s / 51k solid leaves. Coarser leaves (2 -> 20k, 4 -> 5k) available if a broad-phase-only tree is ever wanted.
- **Do NOT voxelize as volume:**
  - **Terrain** — deferred. `ClassName == "Terrain"` *does* inherit `BasePart`, so it must be explicitly excluded or it rasterizes its whole 2000^3 region.
  - **Huge flat ground slabs** (e.g. a 2048^2 baseplate) — a flat floor is analytically "floor across this rectangle," not 4M voxels. Skipped by a footprint threshold and handled as flat floor primitives in the floor stage. Measured: ~14M leaves / 3 min before the exclusion.
- **Key lesson:** an octree's sparse win only appears where large regions *collapse*. At 1-stud resolution surfaces never collapse, so the SVO is 1-stud only over detailed structure; broad flat ground and (later) terrain floor are represented analytically, and fine precision lives in the floor extraction stage (raycast-down), not in a global volume.
- **Block vs non-block voxelization.** Block parts (`Part` with `Shape==Block`) use fast OBB rasterization. Non-block parts — **Unions, MeshParts, wedges, cylinders** — are voxelized against their **real collision geometry** via `GetPartsInPart`. Using `part.Size` on a union fills the octree with *phantom solid*: an arch union measured 100% solid by bbox but only ~33% real, which suppressed the real floor under and around it. The precise path is conservative (surface voxels solid -> ~46% for that union) and only marginally slower (~+150 ms for one union). See `SVO.isBlockPart` / `SVO.insertPartPrecise`.
- **The SVO is conservative by construction** — surface voxels are marked solid, so the hull is inflated by up to one leaf. This is why clearance uses an upward raycast rather than voxel-stepping, and why **boundaries are traced from surfel extent, not from SVO solid voxels.**

## Floor extraction (implemented — `src/Floor.lua`)

`NVGN.Floor` produces one **surfel** per 1-stud walkable cell:

```
Surfel    = { pos, normal, slope, clearance, part }
FloorData = { surfels = {...}, index = {"x:z" -> {surfels}}, config }
```

- **Candidates** come from the SVO (solid voxel with empty space above). Each candidate's top face is walked at **1-stud resolution regardless of node size** — a collapsed big node sampled once misses ~60% of the floor, and one node can cover several parts, so every 1x1 cell gets its own raycast.
- **Exact surface** (height + normal) comes from a **downward raycast onto the real part**, so ramps are smooth, not stair-stepped. `slope > maxSlope` (default **65 degrees**, tested) is dropped unless the part is a `ClipRamp` (always walkable).
- **Clearance** = an **upward raycast** to the real ceiling. Not SVO voxel-stepping — the conservative over-voxelization corrupts sub-voxel clearance near tilted and thin geometry.
- The `index` is a 1-stud spatial hash; a key can hold **several surfels at different heights** (multi-level floors). This is the structure the boundary stage traces against, and the structure that absorbs the irregular vertical extent of rubble.
- Cost: full bake (gather + SVO + extract) ~= **1.7 s** for the 177-part test scene -> ~49.5k surfels.

Because heights are exact per cell, **vertical staircasing does not exist in this pipeline.** Ramps are smooth and stair risers stay crisp (a riser exceeds both the step tolerance and the slope limit, so it forces a clean break rather than the mushy registration a pure grid produces). The only residual quantization is the **XZ outline of the surfel set**, which is what the boundary stage resolves.

### Known cost problem

A real map with verticality currently bakes in **20-30 minutes**. The test-scene figure above does not extrapolate to that and should not be used to reason about it.

**Profiling is a prerequisite for the destruction work.** Instrument with `os.clock()` per stage — gather, SVO build, floor extraction, and within floor extraction, raycasts vs. everything else. Three candidates behave very differently under any regional-rebake scheme:

- **Floor raycasts** — per-cell independent, scales with tile area, tiles cleanly.
- **SVO build** — has global structure; a local rebuild may mean subtree surgery.
- **Part gathering / spatial queries** — may be superlinear depending on implementation.

Floor extraction is embarrassingly parallel (per-cell independent). Parallel Luau across Actors is the obvious lever and is likely a large multiple, not a few percent. Worth doing on its own merits regardless of destruction.

## Boundary extraction

> **This replaces the previous face-projection method.** The old approach took
> each blocking vertical face, projected its bottom edge down to the floor, and
> clipped the excess. It degraded badly on intersecting and overlapping parts
> (coplanar faces, T-junctions, parts poking into each other) and had no
> mechanism at all for Unions and MeshParts, which expose no readable planar
> face with a usable CFrame. It also needed a *second*, separate mechanism for
> floor-extent edges — rooftops, ledges, cliffs — where there is no face to
> read because the floor simply ends.
>
> The method below handles walls, meshes, unions, and cliffs with one
> mechanism, and never reads a face, a normal, or a CFrame. This matters far
> more now that destruction is emergent: a collapsed structure is the worst
> possible input for face interpretation and an unremarkable input for
> sampling.

### Cost note

Every step below is arithmetic over cells already in memory. **Zero raycasts, zero physics queries.** Against the raycast-bound floor stage this is a rounding error, and it removes the old approach's per-face clipping work. Expensive stage is unchanged; this one is downstream of it.

This is a **complexity argument, not an extrapolation from the test scene**, so the missing bake profile does not weaken it: floor extraction is two raycasts per cell, component labelling / EDT / fitting are a handful of arithmetic ops per cell, and that ratio is invariant in map size. Untested, but not uncalibrated.

The one part that is not per-cell arithmetic is **contour tracing** — loop-following with pointer chasing, so on a large multi-level map its cost is cache behaviour, not op count. Still nowhere near raycast cost.

### Iterating on this stage

Boundary extraction reads `FloorData` and nothing else, so **it does not have to sit behind the 20-30 minute bake.** Serialize the surfel field once and iterate the boundary code against the cached snapshot: seconds per iteration, no raycasts, no SVO. The constants below (offset radius, fit tolerance, minimum segment length, narrow-region margin) all need empirical tuning, and this is the loop to tune them in.

Keep snapshots of scenes that actually stress the pipeline — a spiral ramp, a balcony over floor, a curved union, an acute corner. A snapshot of the 177-part test scene makes every constant look fine and calibrates none of them.

### Pipeline

**1. Label connected components.**
Everything downstream operates **per component**, never per height slice.

Label connected components over surfel adjacency, where two neighbouring cells connect **iff their height difference is below the step tolerance**. That predicate is not arbitrary: it is exactly the relation that decides whether an agent can step between two cells, so a component is precisely a mutually reachable set.

This replaces the earlier "trace per height layer," which was never a specification. Height banding is the wrong abstraction and a flattened XZ grid is outright wrong — a balcony would make the floor beneath it read as narrow and erode it away. Consequences of doing it this way:

- **Balconies and overhangs** are separate components from the floor below, so neither contaminates the other's distance field.
- **Spiral ramps** fall out for free: a ramp passing over itself is one component that never self-adjoins, because the vertical gap at the overlap exceeds the step tolerance. No global height banding is needed.

**2. Distance transform (per component).**
Compute a **Euclidean** distance transform (EDT) over each component's occupancy: every walkable cell gets `D`, the distance to the nearest cell not in that component. One cheap grid pass, no geometry.

Because the EDT runs per component, "distance to the nearest non-walkable cell" means what it should have meant all along — distance to the edge of the reachable set.

Use a true EDT, **not** a 4- or 8-neighbour structuring element — those produce a diamond or square kernel, so the resulting offset is short on the diagonals or long on them. The offset must be circular.

`D` serves three purposes at once: it *is* the thickness map, it *is* the erosion test (`erode by r` == `discard cells where D < r`), and it drives the narrow-region branch below. There is no separate thickness pass.

**3. Trace contours (per component).**
Trace boundary loops from the **surfel extent** — where the component's cells end. Not from SVO solid voxels (conservative, inflated).

This uniformly captures both boundary sources: a cell adjacent to a wall, and a cell at a rooftop or cliff edge where the floor just stops. No distinction is needed.

**4. Segment into lines (greedy fit).**
Walk each contour, maintaining a best-fit line through the cells accepted so far:

- After each cell, test the **maximum perpendicular distance** from any accepted cell centre to the line. Maximum, not average — an average lets a shallow corner hide inside a long run.
- Tolerance ~= **1 stud**.
- When the test fails, close the segment and restart from that cell.

Corners are **where the fit fails** — a byproduct of segmentation, not a prerequisite for it. Nothing detects corners from geometry. This is the step that made every previous attempt fail: corner detection was treated as a separate problem solved against real parts, and it inherited every failure mode of face interpretation.

Fit with **total least squares** (PCA on the cell centres), not ordinary least squares — edges can run near-vertical in XZ and OLS is unstable there.

Include the cells hugging the wall in the fit. They define where the wall is; excluding them degrades the fit. The offset happens after.

**Then bias the fit inward.** After fitting, translate the line inward along its normal until **no accepted cell centre lies outward of it**. Only then hand it to step 5.

This is not a tuning detail, it is what makes the safety guarantee exact. The earlier version of this document budgeted the error instead — offset `1.5`, allow the fit to deviate outward at most `1.0`, retain `0.5` residual — and that arithmetic does not close. Boundary cell centres already sit up to ~0.5 stud inward of the true wall face, and the free fit can sit outward of those centres by up to the tolerance, so a `1.5` offset can leave well under `0.5` residual clearance. Against a hard constraint ("never open a route that does not exist") a probabilistic bound is not acceptable.

With the inward bias the guarantee is exact rather than probabilistic, and the fit tolerance stops carrying any safety burden — it becomes purely a polygon-count knob, free to be tuned for mesh quality alone.

The translation is cheap. TLS already gives the centroid and the normal, so it is one pass over the accepted cells to find the maximum outward signed distance and shift by it.

**5. Offset the lines inward.**
Offset each inward-biased line further along its normal by the agent radius `r`, then compute corners by **intersecting the offset lines** of adjacent segments.

Deliberately *not* done by eroding cells before tracing. Cell erosion bevels every sharp convex corner into a small arc, which the segmenter reads as two or three short segments instead of one clean corner — more polygons, the opposite of the goal. Offsetting fitted lines reconstructs corners sharp, gives a sub-cell-exact offset rather than a cell-quantized one, and leaves occupancy intact for step 6.

The miter join costs nothing extra to *compute* — corners were already found by intersecting adjacent fitted lines, so intersecting the offset versions is the same operation. But it is not free geometrically: **at an acute corner the two offset lines intersect arbitrarily far out**, the classic miter spike. Apply a **miter limit with a bevel fallback** — past a threshold ratio of miter length to `r`, cut the corner with a bevel edge instead of extending to the intersection. Cheap, and without it sharp corners emit spikes into solid geometry.

**6. Narrow-region branch (graded, automatic).**
Regions too thin for a full offset are handled by **grading the offset down**, not by switching it off:

```
offset = clamp(D_local - margin, 0, r)
```

Continuous in `D`, so there is no jump at the threshold and adjacent polygons straddling it still share clean edges. A binary "skip the offset if too narrow" rule would flip between an offset of `r` and an offset of `0` across a hair's width of `D`, which is a discontinuity in the walkable boundary, not a graceful degradation. The `width` annotation grades with it rather than appearing as a flag.

**`D_local`, not a region-wide max.** Use the local `D` along the segment being offset. A component containing both a wide hall and a narrow passage would otherwise take one offset for the whole thing and under-offset the hall.

A 1-stud crawler passes the query-time filter; a human does not. No human decision, no bake warning, no manual flagging.

**Union-find is mandatory, not optional.** Run it on surfel adjacency before and after the offset. Any component that *would* be severed is by definition a narrow region. This is the pipeline's **only severance detector** — nothing else notices when the offset disconnects the map — so it is a required stage, not a debug aid.

### Invariants

- **Erosion only ever removes walkable cells, so it cannot invent connectivity.** Its error direction is always "slightly less walkable than reality."
- **Never apply morphological closing** (dilate-then-erode) to clean up protrusions. Closing bridges any wall thinner than the kernel, welding two rooms into one polygon — manufacturing exactly the through-wall portal bug this is meant to eliminate. Erode, never close.

### What erosion does and does not fix

Erosion is **asymmetric**:

- A **notch** (alcove recessed into a wall) narrower than `2r` erodes away completely. Free cleanup.
- A **protrusion** (bump sticking into the corridor) survives — the boundary offsets around it. Protrusions are absorbed by the fit tolerance in step 4, which is safe because the **inward bias** guarantees the fitted line never sits outward of any accepted cell centre before the offset is applied. Without that bias, a straight-line fit would cut through real geometry.

### Known limits

- **Curved geometry has no straight line to find.** A rounded union wall segments into many short pieces — correct, but poly-heavy. Cap with a minimum segment length plus a near-collinear merge pass. Careful: minimum segment length interacts badly with corner-by-intersection, since once segments get short enough adjacent offset lines barely intersect and corner positions go unstable. The miter limit covers the acute end of this; the near-parallel end needs the collinear merge to fire first.
- Precision is capped near cell size. Intersecting fitted lines recovers sub-cell corner positions, so the result beats 1 stud, but it is not CFrame-exact. Accepted trade.
- Boundaries carry no semantic label by construction (wall base vs. ledge vs. mesh edge are all just "boundary"). If a later system needs to know which part blocks a given boundary cell, query the SVO for the adjacent solid leaf and read its **source part ID** — sampling, not interpretation. Boundary cells with no adjacent solid are floor-extent edges (cliffs, ledges, rooftops).

## Polygon optimization

Merge aggressively into large convex polygons, splitting only when one of these changes:

```
split if  floor-deviation > 2 studs   (discrete step tolerance)
       OR clearance changes           (e.g. open air vs. a crawl tunnel)
```

Slopes use the angle limit rather than the +/-2 deviation, so a long ramp is not merged into flat floor. Small polygons are reserved for genuinely tricky geometry.

There is **no width split** — narrow corridors get their own polygons automatically because the boundary edges bound them. A corridor is thin because its two boundary edges are close.

**Note for destroyed regions:** rubble is irregular, so these tolerances split constantly and a ruin generates far more polygons per square stud than the building did. That is a bigger threat to A* cost than rebake time. Post-destruction regions likely want deliberately coarser tolerances. Deferred with the rest of destruction.

## Agents & sizing

Two dimensions decide whether an agent fits: **clearance** (vertical) and **width** (`2 * radius`).

### Clearance — baked

Vertical headroom is not present in the 2D boundary geometry, so it must be baked per surfel and per polygon.

### Width

**Revised from the original design.** Width is still not baked in open areas — the boundary edges encode it, and a portal's shared-edge length gives it directly at query time.

But agent radius is now **partially baked into the mesh** by the step-5 offset, so the mesh is no longer fully continuous in agent size. Narrow regions take a graded offset (`clamp(D_local - margin, 0, r)`) and carry a `width` annotation that grades with it.

Consequence, stated plainly: a small agent cannot exploit the extra room available to it in a normal-width corridor, because that room has already been offset away. **This is an accepted trade** — a streamlined mesh for the 90% case is worth more than preserving every crack for a few small entities. Narrow *routes* remain available to small agents via the width annotation; narrow *margins* inside wide corridors do not.

**Crouch and crawl are unaffected.** They are keyed on clearance, which is vertical; the XZ offset does not touch them. A 1.5-stud-high crawl space that is 6 studs wide bakes exactly as before.

### Detail band (~1-9 studs) — one annotated mesh

A single mesh with per-polygon/per-edge `clearance`, plus `width` on narrow polys only. Any agent filters at query time:

```
skip edge if clearance < agent_height
          OR (poly has width annotation AND width < 2 * agent_radius)
          OR portal_edge_length < 2 * agent_radius
```

Movement modes are per-edge, not separate meshes:

```
walk   if clearance >= 5
crouch if clearance >= 3     (cost penalty, triggers crouch anim)
crawl  if clearance >= 1.5   (larger penalty, triggers crawl anim)
```

Because spaces are authored to their inhabitants, the common case — a ~5-stud human/player — filters almost nothing, so the filter is effectively free on the hot path.

### Giant tier (~9+ studs, configurable) — coarse mesh

A separate coarse mesh of few big polygons over open areas only. Giants don't thread doorways or route around small obstacles — they **destroy** them. The 9-stud cutoff is configurable per project.

### Slope split (decided, independent of destruction)

`Humanoid.MaxSlopeAngle` is per-humanoid. Set **NPCs to 65 degrees** (matching bake `maxSlope`, or the mesh promises routes the humanoid cannot take) and **players lower, ~40 degrees**.

Same geometry, asymmetric traversal: players slide off steep irregular surfaces, NPCs walk up them. This is the primary mechanism denying players stable refuge on rubble, and it costs nothing at runtime. It is independent of everything in the destruction section and can be implemented now.

## Destruction (deferred)

**Blocked on the destruction system existing.** Nothing here is committed. This section records what is already known so the navmesh work can resume without re-deriving it.

### What changed

Destruction is **emergent and physics-driven**. Players can blow a hole in a castle, topple a tower with charges at its base, or level a town. Post-destruction topology is not enumerable.

**Everything pre-baked in the old design is dead:** destruction records, phantom polygons, per-part portal links, pairwise compound-breakable records, and the authoring rule about minimum breakable-opening widths. Do not resurrect them.

The `record.enabled = true` runtime model is also gone. There is no record to enable.

### What the navmesh needs from the destruction system

Stated as an interface, so the destruction system can be designed against it:

1. **A settle signal.** Physics quiescence in a bounded region, **debounced** — a collapsing tower must produce one signal after motion stops, not hundreds as individual parts anchor.
2. **A bounded affected region.** An AABB or tile set. Unbounded "something changed somewhere" is unusable.
3. **Final geometry that is anchored and stable.** The navmesh cannot track moving parts.
4. **A bounded part count in the settled result.** This is the one most likely to be violated, and the one most worth designing for. See below.

### Options on the table (none chosen)

- **Steering zones — cheapest, currently favoured.** On settle, delete the polygons in the affected tiles and mark them as no-navmesh. Navmesh routes NPCs to the zone perimeter; inside, NPCs drop to direct local steering at the target. Zero bake cost, zero poly cost, and the stale-interior problem disappears because there is no mesh inside to be stale. Fits the usage profile: explosives are rare and non-stackable, regions are small, engagements are short, targets usually in line of sight. Cost is stuck NPCs in local minima — mitigate behaviourally (jump when blocked, jitter heading, abandon after N seconds and reroute around the perimeter). Local steering is needed for stairs and ledges anyway.
- **Off-screen NPC cheating.** Unobserved NPCs skip the navmesh entirely — straight-line movement toward the target, one downward raycast per tick to sit on the ground. Combined with steering zones this removes most of the rubble navmesh problem. Caveat: "unobserved" means no player camera, not just the nearest player, with a hysteresis margin so NPCs do not pop as someone turns around.
- **Secondary fragmentation.** On ground impact, a toppling structure breaks down further into a rubble field rather than resting as an intact toppled building. Physically motivated, and it solves the interior-rooms problem at the source. A rubble field is also poor refuge — irregular and sloped — which reinforces the slope split. **Risk: part count.** Thousands of unanchored simulating parts replicated to every client cuts directly against keeping the client light.
- **Fragment then swap to props.** Simulate the collapse for a few seconds for spectacle, then on settle replace the debris cluster with a small number of **pre-made anchored rubble props** sized to the footprint, and delete the fragments. Recovers authoring control: chosen slope, no interior voids, fixed part count, and geometry that raycasts into a handful of fat polygons. Makes any future rebake cheap in a way rebaking real debris would not be.
- **Coarse regional rebake — fallback only.** Same pipeline over destroyed tiles at 4-stud cells instead of 1-stud (~16x fewer raycasts) with relaxed merge tolerances. Deliberately bad geometry that is good enough to path across. Build only if steering measurably fails.
- **Full-fidelity regional rebake — not currently viable.** Cannot be assessed without the profile, and may be a non-starter if the SVO does not tile cleanly.

### Decided regardless of which option wins

- **Rubble must not be treated as obstacle-only at scale.** Nuke a town and every tile becomes solid obstacle — NPCs cannot traverse the ruins and the map is functionally deleted.
- **The slope split** (NPCs 65, players ~40) is the refuge-denial mechanism and does not depend on any of the above.
- **Trigger on debounced quiescence**, never per-part-anchored.
- **During collapse the region is unwalkable.** Correct — it is a hazard zone.

### Open questions

- Does secondary fragmentation plus prop-swap keep part count bounded in practice?
- Is steering sufficient, or is coarse rebake needed?
- Tile size, stale-tile policy, and rebake queue budgeting — all downstream of the profile, and only relevant if a rebake option is chosen at all.

## Open items

- **Component labelling and the inward-bias translation** are the two changes that affect code structure. Land both before tuning any constant.
- **Profile the 20-30 minute bake.** Urgent on its own terms and a prerequisite for any rebake decision in the destruction section — but it does **not** gate boundary extraction, which iterates against a cached surfel snapshot.
- **Parallelise floor extraction** across Actors.
- **Offset radius value** — `1.5` is a starting figure. Tune against snapshots once boundaries are traced. It no longer carries a safety budget, so it can be tuned for mesh quality alone.
- **Fit tolerance** — `1` stud is a starting figure. Now purely a polygon-count knob.
- **Miter limit threshold** — needs an acute-corner case to calibrate.
- **Minimum segment length / collinear merge threshold** — needs a real curved-union case to calibrate.
- **Narrow-region margin** — the `margin` in `clamp(D_local - margin, 0, r)`, and whether narrow polys need their own merge rules.
- **Step tolerance for component adjacency** — shared with the polygon-merge step tolerance, or independent?
- **Terrain** — still deferred. Boundary extraction from a grid works on terrain in principle, since it never needed a readable face; terrain's open problem is floor *sampling*, not boundary tracing.
- **Giant tier count** — one coarse mesh, or a couple of buckets across the 9-50 stud range.
