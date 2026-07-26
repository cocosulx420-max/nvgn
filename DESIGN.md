# nvgn — Navmesh Generator Design

A baked navmesh generator for Roblox with authored, destructible environments.

## Guiding constraints

- **Bake everything expensive.** The client should never pay for navmesh generation at runtime. Long bake times are acceptable; runtime hitches are not.
- **Destruction is authored, not emergent.** Only specific parts break, and only specific explosive tools break them. Because the destructible set is finite and known, every post-destruction topology can be pre-baked.
- **Navmesh is polygonal**, optimized for readability and pathfinding speed: large convex polygons for open areas, small polygons only for tricky geometry, no long thin slivers.
- **The one runtime exception** is settled debris (see Destruction), which stamps a single bounded temp-obstacle carve — never a rebuild.

## Substrate

1. **Sparse Voxel Octree (SVO)** gives a fast solid/empty read of the world. Also the future substrate for flying NPCs. Implemented in `src/SVO.lua` (`ServerScriptService.NVGN.SVO` in-place).
2. A **surface/voxel pass** over the SVO yields the two fields the generator needs:
   - **floor** — walkable surface locations
   - **clearance** — empty vertical space above a floor voxel before hitting a ceiling/obstacle

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
Surfel   = { pos, normal, slope, clearance, part }
FloorData = { surfels = {...}, index = {"x:z" -> {surfels}}, config }
```

- **Candidates** come from the SVO (solid voxel with empty space above). Each candidate's top face is walked at **1-stud resolution regardless of node size** — a collapsed big node sampled once misses ~60% of the floor and misses that one node can cover several parts, so every 1×1 cell gets its own raycast.
- **Exact surface** (height + normal) comes from a **downward raycast onto the real part**, so ramps are smooth, not stair-stepped. `slope > maxSlope` (default **65°**, Cocosulx-tested) is dropped unless the part is a `ClipRamp` (always walkable).
- **Clearance** = an **upward raycast** to the real ceiling. (Not SVO voxel-stepping — the conservative over-voxelization inflates surfaces and corrupts sub-voxel clearance near tilted/thin geometry.)
- The `index` is a 1-stud spatial hash; a key can hold **several surfels at different heights** (multi-level floors), for neighbour lookups in the boundary/polygonization stage.
- Cost: full bake (gather + SVO + extract) ≈ **1.7 s** for the 177-part test scene → ~49.5k surfels.

### Boundaries (v1 implemented — **TEMPORARILY DISABLED**, alternative approach being explored)

> **STATUS (2026-07-22):** `src/Boundary.lua` implements the full pipeline described below — exact segment construction, probe-based classification (wall/seam/dropoff/internal/void), exposure trimming, uncovered-coverage tracing (terrain cliffs), collinear merging, endpoint welding and dangling-end extension. It produces closed, correctly-classed chains on the whole test scene (457 edges, ~0.7s), but getting there took **twelve rounds of case-by-case probe fixes** (inside-origin raycasts, corner evidence bleed, folds, thick pitched planks, buried rims, weld drift…). The per-sample physics-probe classification is judged **too fragile to build on**: every new authoring pattern found a new hole. The module is kept as a checkpoint and as a library of reusable pieces (plane∩rect construction, exposure oracle, chain-closure passes), but it is **not** the path forward — a different approach to the staircasing/boundary problem is being designed to replace the classification layer.

The floor filter answers only *"is there floor here?"*. Boundaries come from **geometry**, from two sources — and the key correction is that **each source uses a different derivation**. The first implementation derived *both* from the surfel field, which is wrong for walls (see the staircase failure mode below).

**Unifying principle — every part-derived edge is a surface intersection.** An edge from a part is the line where the part's face **intersects the walkable surface**, computed as `{face plane} ∩ {floor-part top}` and clipped to their overlapping footprints — *not* the part's bottom edge and *not* a naive downward projection. Walls and clipramps frequently pierce *through* the floor, so their bottom sits in dead space below where anyone walks; the edge belongs where the part crosses the surface you actually stand on. Classify the resulting segment *after* computing it: non-walkable on the far side → a hard **boundary** (wall); walkable on both sides → a **seam** (a clipramp meeting a floor; later a portal). Two consequences:

- A part can cross **several** walkable surfaces at different heights (a wall piercing two floors) → one segment per layer, at each layer's height.
- A part crossing a floor **seam** (two abutting slabs, a step) splits into one segment per floor part beneath it.

- **Walls — from the wall's face, NOT the surfel field.** Apply the intersection construction above → **one clean segment per face/floor pair**, endpoints on the real intersection. A straight wall yields a single two-vertex edge, never a run of grid steps. Then:
  - **Top edge** → decide whether the face is a *true* blocker at all. A face with a ceiling/part directly above and no walkable space on top is a real wall (carve it); a low lip you step over, or an overhang you pass under, is not.
  - A face contributes a boundary only if it actually blocks the agent: height exceeds step-up **and** clearance below its top is less than agent height.
  - **Clip to exposed surface (implemented: exposure trimming).** A plane∩rect segment can be geometrically real yet unstandable — an interior slab embedded through a wall crosses the wall's *outer* face at mid-height ("edge in the middle of a wall"), and a partially buried part's edges continue through solid. Every raw span is sampled against the floor's **local grid** (the clearance-validated record of where standable surface exists) and only grid-supported runs are emitted; each cut endpoint is then **snapped onto the occluding part's face plane**. The grid decides *where* an edge may exist, geometry decides *exactly* where it ends — vertices stay on real geometry, fully unsupported spans vanish.
- **Rims — the floor's own top edges, for free.** The same intersection construction with the blocker being the floor itself (its side faces ∩ its own top plane) yields the part's exact top-face perimeter: the candidate set for dropoffs and the snap targets for surfel-seeded traces. Exposure trimming applies equally — a rim span buried in a hillside or under an abutting structure is no edge.
- **Dropoffs — surfel-seeded, then simplified + snapped.** Rooftops, ledges, and cliffs are bounded by the floor simply *ending*; there is no part face to read, so the trace *must* seed from the surfel field wherever the floor ends with a drop beyond step height and no wall. But the raw surfel trace is grid-quantized, so it is not the final edge:
  - **Simplify** the polyline (collinear-merge / Douglas–Peucker) so a straight ledge collapses to a straight run instead of a staircase.
  - **Snap** vertices to a nearby part edge when one lies within ~1 stud, so a ledge backed by real geometry lands on it. Only genuinely organic edges keep a stepped shape, and even those are smoothed.

**Failure mode to avoid — the staircase.** Deriving *wall* boundaries from surfel adjacency (scanning where walkable cells stop) quantizes every edge to the 1-stud grid: a single straight wall becomes ~20 stepped micro-segments hugging the voxel field instead of two endpoints on the real face. This was the first-pass result. Walls must come from the face; only true open dropoffs may seed from surfels, and those get simplified. Expected output is sparse polylines with vertices only at real corners.

**Debug viz legend:** **red = wall boundary**, **cyan = dropoff edge**.

**Robustness note:** tracing boundaries geometrically across a town of intersecting, overlapping destructible parts is where the bugs will live (coplanar faces, T-junctions, parts poking into each other). Budget for solid clip/merge handling.

### Boundaries v2 — node graph, attribution, geometry stealing (in design, 2026-07-25)

The replacement for v1's per-sample probe classification. The organising principle is a **strict division of labour**, and every v1 failure traces back to violating it:

> **Nodes are a selection mask and a connectivity graph. They are never geometry.**
> **Part geometry is the only source of edge lines.**

The moment a node position is used *as* a line, the 1-stud lattice is baked into the output and the staircase is unavoidable. Nodes decide *whether* an edge exists and *which stretch* of it is real; the blocking part decides *exactly where it lies*.

**Substrate — dead cells carry their killer.** The LocalGrid bake persists killed cells (`grid.dead` / `deadIndex`) with the part that killed them, captured at kill time from the occupancy probe, the clearance up-ray or the terrain pair. No re-probing, and no inside-origin exposure. Test scene: 18,190 dead cells, 112 distinct killers. This attribution is what makes geometry stealing possible later, and it is nearly free.

**Stage order.** Each stage exists to feed the next one trustworthy input; running them out of order means classifying against neighbour data already known to be wrong.

1. **Cross-grid adjacency.** Grids are per-part, so a cell at the rim of part B has an "absent" neighbour even when part A covers that spot at the same height. These are *fictional* boundaries, and they are line-breaking: a false dropoff one row behind a real one puts a branch or a jog into any polyline fit. Before calling a direction absent, ask whether **any** grid has a live node there, not just the host. This deletes most of the seam class structurally instead of patching it with smarter probes.
2. **Per-direction classification.** A class belongs to the **(node, direction) pair** — the cell-boundary segment — not to the node. A stair tread is simultaneously a wall uphill and a dropoff downhill; collapsing that to one label per node with `wall > dropoff` precedence destroys the dropoff wherever a tread is narrow enough for one cell to touch both, which is most treads.
3. **Node welding.** Collapse coincident lattices from overlapping parts into one node set. This is **simplification, not correctness** — stage 1 already fixed the truth. Cluster-to-centroid rather than pairwise "nearest neighbour" averaging: pairwise is order-dependent (if A's nearest is B but B's nearest is C the result depends on iteration order) and leaves a stray node whenever three parts overlap. Gates before any merge: heights flush within ~0.3, compatible normals, and **no geometry between the two nodes** — without all three, a floor node merges with a ramp node and the result floats in space.
4. **Attribution.** Record the blocking part on every wall edge. For a dead neighbour it is already stored (`killer`); for an absent-but-occupied neighbour the occupancy probe already returns it and we currently discard it. Free either way.
5. **Geometry stealing.** Replace each attributed run with an exact line.

**Geometry stealing — the staircase cure.** Once an edge knows its floor `F` and blocker `K`, the clean line is the **plane–plane intersection** `{K's blocking face} ∩ {F's top}`. That is exact: no lattice, no sampling, no quantization. A rotated wall yields a perfectly straight diagonal because the face plane lives in K's frame and F's lattice never enters the computation — the staircase is not smoothed, it is never generated. The contiguous run of wall edges sharing `(F, K)` supplies only the **extent**. Jagged mask, clean line. Where a run's blocker changes from `K₁` to `K₂`, the corner is the intersection of the two face planes rather than a welded approximation. Dropoff edges use the same construction with `F` as its own blocker, giving F's exact top-face rim.

This reuses v1's committed plane∩rect construction, but **driven by node attribution instead of enumerating every part pair** — strictly better, because attribution names exactly which pairs matter and which stretch applies.

**Why not the footprint lattice for this.** `NVGN.Footprint` (built, Studio-only) recovers a cutter's true underside outline via world-vertical up-rays on a yaw-aligned lattice — origins in open air, so the inside-origin trap cannot fire — and merges boundary runs into axis-aligned runs clean in the cutter's own frame (test scene: 111 block killers → 444 runs, exactly 4 per part, 0.2 s). Measured against the cutters' real face planes the runs land at **median 0.000 / p90 0.041** studs, so it is accurate. But it carries up to ±0.5 stud of lattice quantization *by construction*, and plane∩plane has none. The footprint's job is recovering a **cutting shape** (needed for non-block and tilted cutters); for wall lines, plane intersection is both exact and simpler.

**ClipRamp as a named special case.** Stairs are always authored as parts named `ClipRamp`. Round 10 deliberately removed name-matching in favour of a `steepEntry` slope-change test after a sheet named `RAMP1THIS` slipped through — the authoring guarantee is what makes name-matching safe again, and it should be recorded as a dependency on that guarantee. The end/side distinction is computable from the grid alone: the ramp's height gradient gives the slope direction, **ends** are the edges perpendicular to it (entries → seam), **sides** are parallel (classify normally). This matches the round-6 rule that you enter a ramp at its ends, never through its side mid-slope.

**~~Known gap — non-block blockers.~~** *(closed by the raycast method below.)* When attribution returns Terrain or a Union there is no single face plane to *reconstruct*, so the plan was to degrade to a simplified node polyline. Asking the engine for the plane instead removes the distinction entirely.

**Open decision.** Cocosulx wants the edge of a block adjacent to a clipramp classed **wall** ("a clipramp below you means you cannot fall"). This is a real exception to the round-7 rule that step tolerance is out of classification (a drop onto an adjacent surface reads *dropoff from above, wall from below*, paired by the pathfinder per agent). It needs to be scoped tightly to clipramp adjacency or it will start swallowing legitimate ledges.

### Boundaries v2, stage A — raycast-derived edges (`src/Clean.lua`, built 2026-07-25)

Cocosulx's refinement, and the form geometry stealing actually shipped in. Rather than reconstruct the blocker's face plane from its OBB, **fire one horizontal ray from the standable node into the blocked direction** and let the engine hand it over: `Position` (a point exactly on the blocking surface), `Normal` (its plane) and `Instance` (the blocker, for grouping), from a single call.

Three properties follow, and each one deletes a class of problem rather than handling it:

- **No blocker is special.** Unions, MeshParts, wedges and Terrain return a hit normal like anything else, so the "known gap" above never arises — there is no `isBlock` gate in the module. On the test scene 136 samples come off the arch Union and produce ordinary exact lines.
- **The inside-origin trap cannot fire.** Every origin is a live node's surface, which passed LocalGrid's clearance probe and is therefore outside every collider by construction. Ray heights `{0.25, 0.75, 1.2}` all sit below `minClearance` 1.5 to keep that guarantee; nearest hit wins, so a low kerb cannot mask the wall behind it. That trap has bitten four separate probes on this project — this is the first construction where it is *excluded* rather than *guarded*.
- **The ray classifies and measures at once.** A hit is a wall carrying its own geometry; a miss is an open edge. v1 needed a chain of bespoke probes to pick the class and a separate construction to place the line, and each new authoring pattern broke one or the other.

Runs group by **(floor, blocker instance, plane)** — the plane itself, not a collinearity tolerance — and supply only the **extent**; the line remains `{hit plane} ∩ {floor top}`. Measured accuracy: hit deviation from the group plane **median 0.0000, p99 0.0001, max 0.0020** studs (the footprint lattice, for comparison, is p90 0.041 by construction).

**Entries are a slope test, not a name test.** A hit whose normal is within the walkability limit did not find a wall — it grazed a **floor**. A clipramp's low end sits a hair under the ground, so the ray clips the ground's top face and v1 called the ramp entry a wall. The rule: walkable normal + flush within `seamEps` (0.3) → **seam**; beyond `seamEps` it stays a wall, preserving round 7. For ramp **tops** the ray misses, so one down-probe just past the rim (`seamDrop` 2.0) applies the same walkable-and-flush test before a dropoff is declared. The flush gate is what keeps this scoped to real entries — it reproduces round 6's end/side distinction without consulting a part name, so `RAMP1THIS` cannot slip through. Both seam kinds retain their hit and get exact plane∩plane lines. Result: all six test-scene clipramps seam at both ends, dropoff along the sides, zero ramp-vs-ground walls.

**Never key on instance names.** Grouping originally keyed on `tostring(part)`, which returns the *name*: every part called `Pink` or `ClipRamp` collapsed into one group, merging runs across unrelated floors and — where interleaved samples tripped the run-gap test — shattering the remainder into 1,571 junk dropoff edges (299 after the fix). Authored scenes reuse names freely; keys must carry identity.

> **STATUS: keep. Refine later, do not redesign.** This stage works and is validated; it is committed as the foundation the rest of the boundary work builds on. Returning to it means *refining* the remaining pieces listed below — not revisiting the raycast method itself, which is settled. Two categories are explicitly out of scope until the block-part pipeline is perfect, and are being deferred deliberately rather than forgotten: **Unions/MeshParts you stand *on*** (they get world-aligned `fallback` grids, which `sampleFrontier` skips, so a union floor currently produces zero boundary — note that unions you walk *into* already work, since the ray does not care what it hit), and **Terrain**, on the same footing. Get normal parts perfect first.

#### Closure — vertices come from line intersections, never from samples

Stage A originally emitted each run's extent straight from the sample mask, which is wrong by up to half a cell in **both** directions at once: it overshoots past a corner (the last sample's half-cell extension) and undershoots before one (exposure trimming removed the samples there). Those looked like three unrelated defects — edges crossing past each other, edges failing to meet, and stray fragments — but all three are the same error, and none of them is a *line* error: the lines are plane-derived and exact to 0.002 studs.

So runs are **lines**; the mask only hints which portion is real. A floor's lines are intersected with each other, those intersections are the **only** legal vertices, and endpoints snap **along their own line** onto them. Overshoot trims back and undershoot extends forward — the same operation, landing on the identical point — so a corner closes because two edges share one *computed* vertex, not because two measured endpoints were welded together. Collinearity is likewise identity rather than tolerance: two runs share a line iff the same plane pair generated them.

Consequences worth keeping straight:

- **Every class is now a plane pair**, so there is one code path. `dropoff` uses the floor part's own rim plane (which also fixes the lattice under-covering a rim whenever a part isn't a whole number of cells); `tier` uses the overhang's leading edge. Dropoff is ~60% of all boundary length, so leaving it on the lattice would have walked the staircase back in through polygonization.
- **Rim planes double as virtual lines.** Where two slabs abut flush, coverage correctly emits no boundary, so a wall crossing both floors has nothing to intersect at the junction. The rim plane exists geometrically regardless, and both slabs' rims coincide there.
- **Snapping is asymmetric.** Trimming inward is free — the run really does end at the corner. Extending outward claims boundary where no sample was found, so it is capped at roughly the mask's own quantization (`maxExtend` 0.75). A generous outward reach lets an end jump to whatever vertex lies ahead, which produced 2-stud overshoots into open space.
- **Except at strong vertices.** A vertex is *strong* when the other line's own extent already reaches it, rather than being extrapolated there too. Real corners earn `maxExtendStrong` (1.6); a ramp base needed 1.07. Weak vertices keep the tight cap.
- **Near-coplanar faces are one wall.** Authored walls are built from several parts flush to a fraction of a stud but not equal, which yields parallel lines that can never intersect — no corner-closure can reach them. Faces within `coplanarEps` resolve to a single line, taking the offset furthest toward the walkable side (conservative: the agent is held a hair further from the wall).

> **Trap, twice burned:** `cond and X or Y` silently yields `Y` whenever `X` is false. It shipped once in the trim/extend budget selection — handing outward moves the inward budget and quietly restoring the over-extension the cap existed to prevent — and once in an audit script, which then under-reported open endpoints and sent an investigation down the wrong path. Write branches explicitly in this file.

#### Open endpoints are not all defects

Three distinct outcomes, and conflating them buries the real ones:

- **Handover** — walkable floor continues past the end onto a *different* part, whose own boundary picks the region up elsewhere. Normal; loop assembly stitches across it.
- **Deferred bridge** — a rim gap that exists only because the neighbouring floor is a union/mesh (deferred, so it emits nothing). Both sides lie on the same rim line, so the bridge is exact, follows the part's own rim, and is marked `provisional`. It walls off an un-meshed surface, which is the conservative reading, and disappears once unions emit boundaries.
- **Hole** — anything else. These are the only ones worth chasing.

**Still owed.** Each ramp entry emits a **matched pair** (the ramp's seam and the adjacent floor's), correct as half-edges but needing dedup into one portal record at the weld stage. Two frontier samples on the test scene still produce no edge: a 7×1×7 slab resting **0.13 studs** above a tilted floor — an obstacle you walk around, not a crawl space — which every body-height ray misses *even unfiltered*, because near-flush geometry is what rays handle worst (the same family as the inside-origin trap). The fix is to fall back to the killer's own face planes, which are exact for a Block and only needed where rays have demonstrably failed. Everything else on the test scene closes: **0 holes**, 3 handovers, 1 provisional bridge.

### Slopes, steps, and stairs

- **Slope-angle limit** governs continuous inclines.
- **±2-stud step tolerance** governs discrete steps (a normal 5-stud-tall character auto-steps a ≤2-stud lip). **This is a pathfinding-time concern, not a bake-time one** — see the round-7 decision below.
- Stairs stay crisp because riser faces exceed both the step tolerance and the slope limit, forcing clean boundary edges instead of the mushy registration a grid produces. Stairs use clipramps.

> **DECISION (round 7): step tolerance is out of boundary classification**, for the same reason width is — it is agent-dependent, and baking it collapses information the pathfinder needs. A 2-stud step bakes as **wall from below** and **dropoff from above**; the pathfinder pairs the two at runtime per agent. Baked seams mean near-flush **continuity only** (`seamEps` 0.3): clipramp joins and small authored discrepancies between neighbouring floors. `stepUp = 2` survives only as the vertical **search window** for far-side probes, never as a classification threshold.

## Polygon optimization

Merge aggressively into large convex polygons, splitting only when one of these changes:

```
split if  floor-deviation > 2 studs   (discrete step tolerance)
       OR clearance changes           (e.g. open air vs. a crawl tunnel)
```

"Clearance changes" concretely means two things. **Hard splits on the movement-mode tier thresholds** (3 = crouch, 4 = min standing height; see Agents & sizing): each poly bakes its **min clearance** and gets one homogeneous traversal class, so walk vs crouch vs crawl space is **resolved at the navmesh stage** — the pathfinder filters/costs whole polys and never re-derives headroom per step. **Soft splits within the walk band** on clearance deviation beyond a tolerance (~2 studs, mirroring floor-dev), so the baked per-poly min stays representative and `poly_min_clearance >= agent_standing_height` remains exact-not-overconservative for the 4–7 scaling walkers (a low-ceiling pocket must not drag down the min of a huge open poly). A walk→crawl transition is a **clearance seam** (poly split), not a wall or dropoff — the boundary stage emits no edge there.

> **DECISION (Cocosulx, 2026-07-25): headroom is carried by CLEARANCE VOLUMES, not by tier-split polygons.** The tier thresholds above are superseded as *geometry*; see the next section. Polygons still split against a volume's footprint, but the split constraint is a conservative box rather than a derived tier line, and the baked number is a measured clearance rather than a tier label.

### Clearance volumes — headroom as an overlay, not as geometry

The original plan projected the low-headroom frontier down as an edge and split polygons on it. Cocosulx rejected that on the grounds that most crawl spaces are **tunnels**, so their restricted region is a volume, not a line you can project — and proposed instead marking the region with a volume that carries *"traverse me crouching/crawling"*, letting the pathfinder consult it. That is the right call, for three reasons that go beyond convenience:

**1. It is the same decision already made for width and step tolerance.** Both were removed from the bake because they are *agent-dependent*, and baking them destroys information the pathfinder needs. Clearance is identical in kind. Splitting geometry at 1.5 / 3 / 4 bakes three arbitrary thresholds into the mesh, and a 3.2-stud NPC then needs a re-bake. A volume storing its **measured minimum clearance** (`2.31`, not `"crawl"`) serves every agent height, including ones invented later.

**2. Error is asymmetric here, and that is what makes it cheap.** A walkable polygon edge must be exact — a wrong edge either walks an agent off a ledge or seals a real doorway, which is the entire reason boundaries steal planes from geometry. A clearance volume has no such symmetry: **over-covering is safe** (an agent crouches slightly sooner than strictly necessary), **under-covering is a headbutt**. Round volumes *outward* and sloppiness is free. Consequently the jagged 1-stud lattice frontier — the defect that has driven this project's whole boundary design — is **acceptable for volumes**, and headroom never needs geometry stealing at all. This also retires the one open case in `NVGN.Clean`: the `tier` class (a node killed by overhead cover, with no face to steal a line from) stops being an edge and becomes a volume border.

**3. It survives destruction better than splitting does.** A volume is caused by one identifiable overhead part, and that attribution is already available at bake time (the `cover` instance recorded when clearance is measured). Destroy the part, delete the volume. A tier-split polygon would instead require re-polygonizing the region at runtime.

**Constraint — volumes must be searchable, not discovered on contact.** "The pathfinder walks into the marker and learns it must crawl" cannot be the mechanism: A\* has to know *before* it commits, or it plans a route and fails partway. Worse, if a volume covers only part of a polygon, that polygon is no longer atomic — a tall agent routing corner-to-corner may clip the low region even though both portals fit, and poly-level A\* has no way to express "passable, but not through the middle". So volumes are **indexed and queried during search**, and polygonization **splits against the volume's box**. The result keeps one homogeneous clearance per poly, but the split constraint is a conservative rectangle already in hand rather than an exact curve that must be derived.

**Constraint — volumes are DATA, never live parts.** Store an OBB plus min clearance plus source part; render as parts only for debugging, with `CanQuery`/`CanCollide` off and only under the debug folder. A real part in the workspace is picked up by the next bake: the project has already been bitten twice, by `HERE` markers growing dropoff rings and by a `CHECK HERE` marker becoming walkable floor.

Slopes use the angle limit rather than the ±2 deviation, so a long ramp is not merged into flat floor. Small polygons are reserved for genuinely tricky geometry. **The generator has no width concept, so splits are never width-driven** — a narrow corridor already gets its own polygons simply because its two boundary edges are close.

### Polygonization — attempt 5: claim-and-fill (Cocosulx's spec, 2026-07-26)

Four earlier attempts were rejected, all for **shape** rather than for correctness: `58c7220` convex + fatness merge, `108ec1f` trapezoidal sweep (rejected by measurement — 66 polys, 6722 studs of long cuts, 39 slivers), `7c5d694` maximal rectangles + fringe, `d08b26d` explicit width/aspect limits. A fifth, `8bb27f6` oriented maximum-rectangle cover, stalled on frame selection: with one global frame the 200×200 floor's own 800-stud rim outvotes every building in the length-weighted angle histogram, and with multiple frames the differing lattices leave gaps no single frame covers.

This section supersedes those. It is written from Cocosulx's reframing of the goal, which removed the frame problem rather than solving it.

#### What the input actually is

The polygonizer does **not** consume voxels, and none of Recast's contour pipeline applies here. `Clean` produces exact boundary lines stolen from real face planes (deviation from the group plane: median 0.0000, max 0.0020) and `Loops` assembles them into closed, oriented regions with holes, validated at 51556/51556 live cells inside a region and −0.1% area slack. Contour tracing, Douglas–Peucker simplification and ear clipping are all machinery for *recovering* geometry from a staircase, and there is no staircase to recover from. Ear clipping is additionally ruled out by the shape spec: it yields triangle fans where the target is a single trapezoid.

So the problem is narrow and exact: **partition a known-exact simple polygon with holes into few, convex, well-proportioned pieces.**

#### The shape rule, in two tiers

The mesh is not built from one vocabulary of shapes. Two tiers with different jobs:

**Tier 1 — claim the open area.** Cover the uneventful middle of a space with the *largest single convex piece that is still well-proportioned*. Not a tiled grid, and **not rectangle-first**: a rectangle is merely what wins this contest in a boxy room. A splayed alcove is claimed by one trapezoid; a room with a single angled wall by a pentagon. The prior is **few, big, fat, convex** — shape follows the space.

**Tier 2 — fill the remainder exactly.** Everything left over (strips along walls, corners, wedges beside diagonals, the band between a claim and an obstacle) is covered by whatever convex shape genuinely fits. Irregular quads, trapezoids, pentagons. Leftovers are **not** re-decomposed into rectangles, and **not** fanned into triangles: a leftover that is already a clean convex pentagon ships as one pentagon.

Rectangles are therefore a *strategy for the easy majority*, never the vocabulary of the mesh.

#### Where "no invented vertices" applies — and where it does not

The earlier no-Steiner-points rule was being applied too broadly, and that broad reading is what made rectangle placement look self-contradictory (a rect grown into open floor necessarily has corners in the middle of nowhere). The rule is scoped:

- **Tier 1 claims may invent vertices.** A claim is a deliberate placement into empty space. Its corners owe nothing to geometry.
- **Tier 2 fill may not.** Every fill vertex must land on a real boundary line, a real corner, or the intersection of two real lines. This is where a wrong vertex becomes a wrong navmesh, so the constraint bites exactly where it matters.

Fill pieces are cut only where they are actually concave, using lines the scrap's own edges already lie on — so vertices come from line intersections, the same construction `Loops` and `Clean` closure already use. Building a full line arrangement across a whole floor was considered and rejected: it over-cuts badly (a 3-stud wall stub extended across a 200-stud slab slices the entire floor, reproducing the trapezoidal sweep's long-cut failure). Extension is local to the scrap being fixed.

#### Placement and fill are one problem, not two

The dominant failure mode is a claim placed next to an angled wall leaving a long thin wedge, which then gets chopped into slivers. That is a **placement** defect, not a fill defect. A claim is only well-placed if what it leaves behind is also usable, so claim quality is scored on the claim *and its residue* together — pull the claim back, or angle it, so the leftover is a workable trapezoid rather than a needle.

#### Why the frame problem disappears

Claim orientation is decided by the space a claim sits in, read from the walls bounding that space. There is no floor-wide voted angle, so a big slab's own rim cannot outvote the buildings standing on it, and differing local orientations need no shared lattice because tier 2 fills between them exactly.

#### Negligible-notch discard — accepted in one direction only

Cocosulx offered, explicitly as a suggestion open to refusal, that tiny notches may be ignored for overall quality. Accepted, bounded as follows:

- **Under-covering is allowed.** A polygon may fall short of the true boundary; that pocket is simply not navigable. Same asymmetry already accepted for clearance volumes — rounding toward the safe side is free.
- **Over-covering is refused.** A polygon may never extend past a boundary line. That is an agent walking into a wall or off a ledge, and it discards the exactness the whole boundary pipeline exists to provide. **Polygons may fall short of the boundary, never past it.**
- **Connectivity guard.** Nothing is discarded if it is the sole connection between two areas — a 1.5-stud gap can be the only route across the map, and deleting it silently removes a path. A discarded piece must not be a cut vertex of the region's adjacency.
- **The threshold is absolute and tiny, not agent-relative.** Width was removed from the bake because NPCs are not all human-sized and sub-agent-width ground stays in the mesh; a discard threshold measured against a human reintroduces the width floor through the back door and strands small agents. The threshold is the scale of *authoring noise* — a fraction of a stud, two parts not quite meeting. **Discard noise, never discard features.**

#### Area accounting, from commit #1

The per-region area assertion in studs² is mandatory from the first commit (it has already caught three silent bugs in an hour: centroid-based fans on concave faces, a floating rectangle making the residual multiply-connected, and a one-graph arrangement shattering leftovers into 6497 slivers). With discard, it gains a second column:

```
polygon area + discarded area == region area      (exact, per region)
```

Discarded area is **reported per region**, never silently absorbed. 0.3 studs² across a floor is the noise we meant to drop; 40 studs² in one region is a bug, visible immediately to both of us.

#### Quality metric

Polygon thinness is measured **relative to the ground the polygon sits on**, never against an absolute width. An absolute limit previously reported 15% "violations" that were mostly narrow ground being legitimately narrow.

#### Consequence for debug viz

Discarded notches leave polygon edges slightly *inside* `Clean`'s lines in places, so the two will not always coincide. The viz must draw both, or a polygon edge sitting a quarter stud off a wall bar reads as a bug when it is the design.

## Agents & sizing

Two dimensions decide whether an agent fits: **clearance** (vertical) and **width** (`2 × radius`, horizontal). Only **clearance** is baked — a per-surfel/per-polygon annotation, because vertical headroom can't be recovered from the 2D geometry. **Width is never baked and is not a generation concern at all.** It is resolved entirely at **pathfinding time**, by the pathfinder, using the navmesh (portal shared-edge length, a poly's opposing boundary edges) together with **SVO** solid queries. The generator emits nothing width-related — no field, no annotation, no split.

**Height tiers:** normal NPCs stand **4–7 studs**; **8+ is the "giant" tier** (coarse mesh, see below). The cutoff is configurable per project but 8 is the default boundary between the two.

### Detail band (~1.5–7 studs standing) — one annotated mesh

A single mesh with per-polygon/per-edge `clearance`. Any agent filters the shared mesh at query time:

```
skip edge if clearance < agent_height  OR  portal_edge_length < 2 * agent_radius
```

This is **continuous in agent size** — no height buckets. The mesh only splits where real geometry changes clearance. Because spaces are authored to their inhabitants (and larger races' architecture inherently accommodates humans), the common case — a ~5-stud human/player — filters almost nothing, so the filter is effectively free on the hot path. Its real work is limited to crawl/crouch spaces, giants, and post-destruction changes.

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
- **Stateful clearance** — a destroyed ceiling raises clearance below; toggled with the record.

At runtime, destroying a part is `record.enabled = true`. A\* then sees the portal. No generation, no re-triangulation, no runtime SVO traversal during search.

Because boundaries are derived from wall footprints, a breakable wall's footprint *is* the portal — boundary extraction and destruction baking are the same operation.

**Cascading collapse** (unsupported parts fall) is handled for free on the removal side: one hit that drops many parts just flips many pre-baked records at once.

**Tradeoff accepted:** portals give correct connectivity but not re-merging. A destroyed wall that split a room leaves two polys + a portal, not one merged mega-poly. Pathfinding stays correct and fast; the mesh is just not as clean as a from-scratch rebake — the right price for staying baked.

**Compound breakables (open):** if two adjacent breakables only open a path when *both* are gone, per-part records miss it. Pairwise "if both A and B destroyed, also enable C" records cover realistic cases without full `2^N` baking. Decide per layout whether this matters.

### Addition (settled debris) — the one runtime exception

Falling parts settle at physics-determined positions that cannot be pre-baked. When a part anchors (unsupported parts fall, then despawn or anchor for optimization), it stamps **one bounded temp-obstacle carve** onto the baked mesh: mark blocked and reduce local clearance. This is Detour-style — O(1)-ish, bounded, **not** a rebuild. It is the single sanctioned runtime mutation of the navmesh.

## Open items

- **Debris carve fidelity** — confirm the carve shape (box/cylinder) and how long anchored debris persists before despawn.
- **Compound-breakable baking** — decide whether pairwise combination records are needed for the intended layouts.
- **SVO resolution** — verify the octree is fine enough for the clearance annotation and for the pathfinder's runtime width queries (~agent-radius/2), or run a finer secondary voxel field.
- **Giant tier count** — one coarse mesh, or a couple of buckets across the 9–50 stud range.
