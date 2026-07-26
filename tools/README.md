# tools/ — test harness, not part of the generator

Nothing in here is shipped with the navmesh. These exist to answer
"is the bake usable?" and nothing more.

- **Pathfinder.lua** — a deliberately small consumer: locate, A*, and a
  string pull. It loads the *serialized* bake rather than a live build,
  so a failure here is a failure of the bake. It has no agent sizing, no
  cost model beyond distance, and no jump links.
- **PathDemo.server.lua** — a Script (runs on Play only) that draws a
  live path from the part named `Start` to the player.

Step pairing lives here on purpose. A step is baked as a **wall from
below** and a **dropoff from above**, never as a portal, because whether
it can be crossed is agent-dependent — the same reason width is not
baked. Pairing those edges is the consumer's job, and `maxStepUp` /
`maxStepDown` are query parameters, not bake data. A real pathfinder
must do the same.
