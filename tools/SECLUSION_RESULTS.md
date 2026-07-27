# Seclusion — results so far

Diagnostic only. Nothing is wired into the bake, nothing is culled.

## Small map (178 parts) — the control

The bake reproduces v0.2 exactly: 433 edges, 104 regions, 160 polys, 148 portals.

| | |
|---|---|
| polys | 160 |
| open | 160 |
| sealed | 0 |
| buried | 0 |
| budget overruns | 0 |
| time | 0.35s |

**Regression gate passes**: a scene known good has nothing flagged.

### Positive control

"0 sealed" proves nothing if the detector never fires, so: two identical 24x24
shells, each with a free-standing island slab inside, one sealed and one with a
wall omitted.

| | verdict | |
|---|---|---|
| sealed shell, inner island | sealed | correct |
| open shell, inner island | open | correct |
| both roofs | open | correct — rooftops survive |
| floor strips under walls | buried | correct |

## Big map (12,749 parts)

Bake 1,373s: SVO 112s, Floor 343s, LocalGrid 246s, Clean 655s, Loops 2.9s,
Polys 14.1s, Portals 0.5s, Volumes 0.4s.
569,776 surfels, 22,430 edges, 4,005 regions, 5,626 polys, 4,123 portals,
2,243 volumes.

Classify: 30.0s.

| | count | share |
|---|---|---|
| open | 4,496 | 79.9% |
| sealed | 799 | 14.2% |
| buried | 331 | 5.9% |

386 sealed air components, 1,278 fills, 159 budget overruns (forced open).

### Sealed pocket sizes

| pocket | components |
|---|---|
| 1–8 voxels | 95 |
| 9–64 | 98 |
| 65–512 | 81 |
| 513–4096 | 68 |
| 4097+ | 44 |

Sensibly bimodal: cracks between parts at one end, genuine enclosed rooms at
the other (largest pocket 5,831 voxels).

### OPEN QUESTION — needs a human look before anything is culled

**490 of the 799 sealed polygons sit in the mesh's LARGEST connected
component** (3,323 polys, after 8,155 step links).

Two readings, and they are not distinguishable from inside the data:

1. The SVO is right and the mesh only *appears* to connect those rooms, through
   the wall-crossing links measured earlier at ~15% of all links. Then Seclusion
   is doing exactly its job.
2. The classifier is wrong and is sealing rooms that are plainly walkable.

Sample sealed locations to inspect:

| position | pocket |
|---|---|
| (521, 183, 850) | 5,831 voxels |
| (893, 127, 575) | 3,179 |
| (884, 127, 487) | 3,179 |
| (551, 84, 607) | 1,696 |
| (529, 127, 749) | 7 |
| (580, 116, 567) | 2 |

Verdict markers are placed by `Seclusion.visualize`: RED sealed, YELLOW buried,
open undrawn. All debug parts are CanQuery/CanCollide off.

## Not addressed here

- **`buried` is reported but never acted on.** Single-column solidity is the
  sub-voxel question the SVO is documented to get wrong, and the fix for a
  buried polygon is emission, not culling. Conflating them would hide it.
- **An enclosed room's interior floor produces surfels but no polygon.** Found
  while building the positive control: the sealed shell's interior gave 72
  walkable surfels and only the strips beneath its walls. Something in
  Loops/Polys loses the interior of a closed room. Independent of Seclusion and
  worth its own investigation, especially on a building-heavy map.
