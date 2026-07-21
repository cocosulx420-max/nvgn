# nvgn — Navmesh Generator Design

A baked navmesh generator for Roblox with authored, destructible environments.

## Guiding constraints

- **Bake everything expensive.** The client should never pay for navmesh generation at runtime. Long bake times are acceptable; runtime hitches are not.
- **Destruction is authored, not emergent.** Only specific parts break, and only specific explosive tools break them. Because the destructible set is finite and known, every post-destruction topology can be pre-baked.
- **Navmesh is polygonal**, optimized for readability and pathfinding speed: large convex polygons for open areas, small polygons only for tricky geometry, no long thin slivers.
- **The one runtime exception** is settled debris (see Destruction), which stamps a single bounded temp-obstacle carve — never a rebuild.

## Substrate

1. **Sparse Voxel Octree (SVO)** gives a fast solid/empty broad-phase read of the world. Also the future substrate for flying NPCs.
2. A **voxel pass** over the SVO yields the three fields the generator needs:
   - **floor** — walkable surface locations
   - **clearance** — empty vertical space above a floor voxel before hitting a ceiling/obstacle
   - **width** — distance transform to the nearest blocking voxel (corridor width)

## Floor & boundary extraction

The floor filter answers only *"is there floor here?"*. Boundaries come from geometry, from two sources:

- **Vertical faces (walls/obstacles).** For each blocking face:
  - **Bottom edge** → project down to the floor to get the boundary line, then clip away excess that runs into other parts or into walkable space.
  - **Top edge** → decide whether the face is a *true* blocker at all. A face with a ceiling/part directly above and no walkable space on top is a real wall (carve it); a low lip you step over, or an overhang you pass under, is not.
  - A face contributes a boundary only if it actually blocks the agent: height exceeds step-up **and** clearance below its top is less than agent height.
- **Floor-extent edges.** Not every boundary is a wall. Rooftops, ledges, and cliffs are bounded by the floor simply *ending*. Wherever the floor filter ends with a drop beyond step height and no wall, that outer edge is itself a boundary.

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
       OR width changes               (e.g. corridor narrows)
```

Slopes use the angle limit rather than the ±2 deviation, so a long ramp is not merged into flat floor. Small polygons are reserved for genuinely tricky geometry.

## Agents & sizing

Two dimensions decide whether an agent fits: **clearance** (vertical) and **width** (`2 × radius`). Both are baked annotations from the voxel pass.

### Detail band (~1–9 studs) — one annotated mesh

A single mesh, with per-polygon/per-edge `clearance` and `width`. Any agent filters the shared mesh at query time:

```
skip edge if clearance < agent_height  OR  width < 2 * agent_radius
```

This is **continuous in agent size** — no height buckets. The mesh only splits where real geometry changes clearance/width. Because spaces are authored to their inhabitants (and larger races' architecture inherently accommodates humans), the common case — a ~5-stud human/player — filters almost nothing, so the filter is effectively free on the hot path. Its real work is limited to crawl/crouch spaces, giants, and post-destruction changes.

**Crouch & crawl are edge movement-modes, not separate meshes.** Each edge carries its min-clearance; an agent picks the cheapest mode that fits, with a speed/cost penalty so A\* prefers standing routes:

```
walk   if clearance >= 5
crouch if clearance >= 3   (cost penalty, triggers crouch anim)
crawl  if clearance >= 1.5 (larger penalty, triggers crawl anim)
```

### Giant tier (~9+ studs, configurable) — coarse mesh

A separate coarse mesh of few big polygons over open areas only. Giants don't thread doorways or route around small obstacles — they **destroy** them. A large creature walking into a breakable simply triggers the destruction system (enable portals, spawn the debris carve) instead of avoiding it. The coarse mesh and the destruction pipeline reinforce each other.

The **9-stud cutoff is configurable** per project.

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
