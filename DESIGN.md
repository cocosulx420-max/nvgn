# nvgn — Navmesh Generator Design

A baked navmesh generator for Roblox with authored, destructible environments.

## Guiding constraints

- **Bake everything expensive.** The client should never pay for navmesh generation at runtime. Long bake times are acceptable; runtime hitches are not.
- **Destruction is authored, not emergent.** Only specific parts break, and only specific explosive tools break them. Because the destructible set is finite and known, every post-destruction topology can be pre-baked.
- **Navmesh is polygonal**, optimized for readability and pathfinding speed: large convex polygons for open areas, small polygons only for tricky geometry, no long thin slivers.
- **The one runtime exception** is settled debris (see Destruction), which stamps a single bounded temp-obstacle carve — never a rebuild.

## Substrate

1. **Sparse Voxel Octree (SVO)** gives a fast solid/empty read of the world. Also the future substrate for flying NPCs. Implemented in `src/SVO.lua` (`ServerScriptService.NVGN.SVO` in-place).
2. A **surface/voxel pass** over the SVO yields the three fields the generator needs:
   - **floor** — walkable surface locations
   - **clearance** — empty vertical space above a floor voxel before hitting a ceiling/obstacle
   - **width** — distance transform to the nearest blocking voxel (corridor width)

### SVO implementation notes (validated)

- **Build = OBB rasterization.** Each solid part's oriented bounding box is rasterized into the octree; nodes fully inside an OBB collapse to a single solid leaf, nodes on the surface subdivide to `leafSize`. Rotated/intersecting parts voxelize cleanly (this was the original staircase pain point) — confirmed on the `project nuhh` test scene.
- **Leaf size = 1 stud.** Over real structures this is cheap: the test scene (177 parts) builds in **~0.9 s / 51k solid leaves**. Coarser leaves (2 → 20k, 4 → 5k) are available if a broad-phase-only tree is ever wanted.
- **Do NOT voxelize as volume:**
  - **Terrain** — deferred (`ClassName == "Terrain"`, which *does* inherit `BasePart`, so it must be explicitly excluded or it rasterizes its whole 2000³ region).
  - **Huge flat ground slabs** (e.g. a 2048² baseplate) — a flat floor is analytically "floor across this rectangle," not 4M voxels. These are skipped by a footprint threshold and handled as flat floor primitives in the floor stage. A global 1-stud volume of such a slab is what makes a naive build explode (measured: ~14M leaves / 3 min before the exclusion).
- **Key lesson:** an octree's sparse win only appears where large regions *collapse*. At 1-stud resolution surfaces never collapse, so the SVO is 1-stud only over detailed structure; broad flat ground and (later) terrain floor are represented analytically, and the fine 1-stud precision lives in the floor/wall extraction stage (raycast-down), not in a global volume.
- **Block vs non-block voxelization.** Block parts (`Part` with `Shape==Block`) use the fast OBB rasterization. Non-block parts — **Unions, MeshParts, wedges, cylinders** — must be voxelized against their **real collision geometry** via `GetPartsInPart` (octree subdivides only where the part's hull actually overlaps a node). Using `part.Size` (the axis-aligned bounding box) on a union fills the octree with *phantom solid* — e.g. an arch union measured 100% solid by bbox but only ~33% real, which suppressed the real floor under/around it. Precise path is conservative (surface voxels solid → ~46% for that union) and only marginally slower (~+150ms for one union). See `SVO.isBlockPart` / `SVO.insertPartPrecise`.

## Floor & boundary extraction

### Floor extraction (implemented — `src/Floor.lua`)

`NVGN.Floor` produces one **surfel** per 1-stud walkable cell:

```
Surfel   = { pos, normal, slope, clearance, width, part }
FloorData = { surfels = {...}, index = {"x:z" -> {surfels}}, config }
```

- **Candidates** come from the SVO (solid voxel with empty space above). Each candidate's top face is walked at **1-stud resolution regardless of node size** — a collapsed big node sampled once misses ~60% of the floor and misses that one node can cover several parts, so every 1×1 cell gets its own raycast.
- **Exact surface** (height + normal) comes from a **downward raycast onto the real part**, so ramps are smooth, not stair-stepped. `slope > maxSlope` (default **65°**, Cocosulx-tested) is dropped unless the part is a `ClipRamp` (always walkable).
- **Clearance** = an **upward raycast** to the real ceiling. (Not SVO voxel-stepping — the conservative over-voxelization inflates surfaces and corrupts sub-voxel clearance near tilted/thin geometry.)
- **Width is deliberately NOT baked.** Horizontal distance-to-wall is redundant with the navmesh boundary edges: portal-fit is a shared-edge *length*, corridor width is a poly's two boundary edges, and agent-radius clearance from walls is a funnel offset — all derived cheaply at *pathfinding* time. Baking it would be per-surfel raycasts of data the boundaries already encode. (Clearance is different — vertical headroom is not present in the 2D boundary, so it must be baked.)
- The `index` is a 1-stud spatial hash; a key can hold **several surfels at different heights** (multi-level floors), for neighbour lookups in the boundary/polygonization stage.
- Cost: full bake (gather + SVO + extract) ≈ **1.7 s** for the 177-part test scene → ~49.5k surfels.

### Boundaries (implemented — being corrected)

The floor filter answers only *"is there floor here?"*. Boundaries come from **geometry**, from two sources — and the key correction is that **each source uses a different derivation**. The first implementation derived *both* from the surfel field, which is wrong for walls (see the staircase failure mode below).

**Unifying principle — every part-derived edge is a surface intersection.** An edge from a part is the line where the part's face **intersects the walkable surface**, computed as `{face plane} ∩ {floor-part top}` and clipped to their overlapping footprints — *not* the part's bottom edge and *not* a naive downward projection. Walls and clipramps frequently pierce *through* the floor, so their bottom sits in dead space below where anyone walks; the edge belongs where the part crosses the surface you actually stand on. Classify the resulting segment *after* computing it: non-walkable on the far side → a hard **boundary** (wall); walkable on both sides → a **seam** (a clipramp meeting a floor; later a portal). Two consequences:

- A part can cross **several** walkable surfaces at different heights (a wall piercing two floors) → one segment per layer, at each layer's height.
- A part crossing a floor **seam** (two abutting slabs, a step) splits into one segment per floor part beneath it.

- **Walls — from the wall's face, NOT the surfel field.** Apply the intersection construction above → **one clean segment per face/floor pair**, endpoints on the real intersection. A straight wall yields a single two-vertex edge, never a run of grid steps. Then:
  - **Top edge** → decide whether the face is a *true* blocker at all. A face with a ceiling/part directly above and no walkable space on top is a real wall (carve it); a low lip you step over, or an overhang you pass under, is not.
  - A face contributes a boundary only if it actually blocks the agent: height exceeds step-up **and** clearance below its top is less than agent height.
  - **Clip** where the segment runs into other parts or into walkable space.
- **Dropoffs — surfel-seeded, then simplified + snapped.** Rooftops, ledges, and cliffs are bounded by the floor simply *ending*; there is no part face to read, so the trace *must* seed from the surfel field wherever the floor ends with a drop beyond step height and no wall. But the raw surfel trace is grid-quantized, so it is not the final edge:
  - **Simplify** the polyline (collinear-merge / Douglas–Peucker) so a straight ledge collapses to a straight run instead of a staircase.
  - **Snap** vertices to a nearby part edge when one lies within ~1 stud, so a ledge backed by real geometry lands on it. Only genuinely organic edges keep a stepped shape, and even those are smoothed.

**Failure mode to avoid — the staircase.** Deriving *wall* boundaries from surfel adjacency (scanning where walkable cells stop) quantizes every edge to the 1-stud grid: a single straight wall becomes ~20 stepped micro-segments hugging the voxel field instead of two endpoints on the real face. This was the first-pass result. Walls must come from the face; only true open dropoffs may seed from surfels, and those get simplified. Expected output is sparse polylines with vertices only at real corners.

**Debug viz legend:** **red = wall boundary**, **cyan = dropoff edge**.

**Robustness note:** tracing boundaries geometrically across a town of intersecting, overlapping destructible parts is where the bugs will live (coplanar faces, T-junctions, parts poking into each other). Budget for solid clip/merge handling.

### Slopes, steps, and stairs

- **Slope-angle limit** governs continuous inclines.
- **±2-stud step tolerance** governs discrete steps (a normal 5-stud-tall character auto-steps a ≤2-stud lip).
- Stairs stay crisp because riser faces exceed both the step tolerance and the slope limit, forcing clean boundary edges instead of the mushy registration a grid produces. Stairs use clipramps.

## Polygon optimization

Merge aggressively into large convex polygons, splitting only when one of these changes:

```
split if  floor-deviation > 2 studs   (discrete step tolerance)
       OR clearance changes           (e.g. open air vs. a crawl tunnel)
```

Slopes use the angle limit rather than the ±2 deviation, so a long ramp is not merged into flat floor. Small polygons are reserved for genuinely tricky geometry. Note there is **no width split** — narrow corridors get their own polygons automatically because the *walls* (boundary edges) bound them; a corridor is thin because its two boundary edges are close, not because of any width annotation.

## Agents & sizing

Two dimensions decide whether an agent fits: **clearance** (vertical) and **width** (`2 × radius`). Clearance is a **baked** per-surfel/per-polygon annotation (it isn't in the 2D geometry). Width is **not** baked — it comes from the polygon/portal geometry itself at query time (a portal's shared-edge length; a corridor poly's opposing boundary edges).

**Height tiers:** normal NPCs stand **4–7 studs**; **8+ is the "giant" tier** (coarse mesh, see below). The cutoff is configurable per project but 8 is the default boundary between the two.

### Detail band (~1.5–7 studs standing) — one annotated mesh

A single mesh with per-polygon/per-edge `clearance`. Any agent filters the shared mesh at query time:

```
skip edge if clearance < agent_height  OR  portal_edge_length < 2 * agent_radius
```

This is **continuous in agent size** — no height buckets. The mesh only splits where real geometry changes clearance/width. Because spaces are authored to their inhabitants (and larger races' architecture inherently accommodates humans), the common case — a ~5-stud human/player — filters almost nothing, so the filter is effectively free on the hot path. Its real work is limited to crawl/crouch spaces, giants, and post-destruction changes.

**Crouch & crawl are edge movement-modes, not separate meshes.** Each edge carries its min-clearance; an agent picks the cheapest mode that fits, with a speed/cost penalty so A\* prefers standing routes:

```
walk   if clearance >= agent_standing_height   (4–7, scales with the agent)
crouch if clearance >= 3    (cost penalty, triggers crouch anim)
crawl  if clearance >= 1.5  (larger penalty, triggers crawl anim)
```

### Giant tier (~8+ studs, configurable) — coarse mesh

A separate coarse mesh of few big polygons over open areas only. Giants don't thread doorways or route around small obstacles — they **destroy** them. A large creature walking into a breakable simply triggers the destruction system (enable portals, spawn the debris carve) instead of avoiding it. The coarse mesh and the destruction pipeline reinforce each other.

The **8-stud cutoff is configurable** per project.

## Destruction

Split every destruction event into two effects:

### Removal — fully baked

Because breakables are authored, the complete set of post-destruction topologies is known at bake time. For each breakable part, bake a **destruction record**, stored disabled and keyed to the part's ID:

- **Phantom polygon(s)** — the walkable floor patch inside the part's footprint that exists only once the part is gone.
- **Portal links** — the shared edges connecting the phantom poly to neighbouring polys, including the left/right gap endpoints so the funnel/string-pull steers cleanly through the opening.
- **Stateful clearance/width** — a destroyed ceiling raises clearance below; toggled with the record.

At runtime, destroying a part is `record.enabled = true`. A\* then sees the portal. No generation, no re-triangulation, no runtime SVO traversal during search.

Because boundaries are derived from wall footprints, a breakable wall's footprint *is* the portal — boundary extraction and destruction baking are the same operation.

**Cascading collapse** (unsupported parts fall) is handled for free on the removal side: one hit that drops many parts just flips many pre-baked records at once.

**Tradeoff accepted:** portals give correct connectivity but not re-merging. A destroyed wall that split a room leaves two polys + a portal, not one merged mega-poly. Pathfinding stays correct and fast; the mesh is just not as clean as a from-scratch rebake — the right price for staying baked.

**Compound breakables (open):** if two adjacent breakables only open a path when *both* are gone, per-part records miss it. Pairwise "if both A and B destroyed, also enable C" records cover realistic cases without full `2^N` baking. Decide per layout whether this matters.

### Addition (settled debris) — the one runtime exception

Falling parts settle at physics-determined positions that cannot be pre-baked. When a part anchors (unsupported parts fall, then despawn or anchor for optimization), it stamps **one bounded temp-obstacle carve** onto the baked mesh: mark blocked and reduce local clearance/width. This is Detour-style — O(1)-ish, bounded, **not** a rebuild. It is the single sanctioned runtime mutation of the navmesh.

## Open items

- **Debris carve fidelity** — confirm the carve shape (box/cylinder) and how long anchored debris persists before despawn.
- **Compound-breakable baking** — decide whether pairwise combination records are needed for the intended layouts.
- **SVO resolution** — verify the octree is fine enough to extract clearance/width at the precision surface extraction wants (~agent-radius/2), or run a finer secondary voxel field for the annotation pass.
- **Giant tier count** — one coarse mesh, or a couple of buckets across the 9–50 stud range.
