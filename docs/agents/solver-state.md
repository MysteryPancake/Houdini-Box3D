# Shared world state

The `SIM_Box3DWorld` cache and `Box3DSharedState`: what lives in the world, what is rebuilt, and who owns it.
The step itself: [solver.md](solver.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Shared state (`SIM_Box3DWorld`)

The Box3D world is a **rebuildable cache**; the DOP objects' point attributes (a whole object's `Position` data) are
authoritative. Snapshotting the world into the DOP cache is too slow, so don't.

* DOPs copy `SIM_Data` (`makeEqual`) every step, so copies share a `std::shared_ptr<Box3DSharedState>`.
* The stamp (`stampTime`, `stamped`) lives in the shared state, so an older timestep cannot claim the world is current.
  `canContinue(now, dt)`: `(now - stampTime) == dt` within `1e-3 * dt` (a difference, so start vs end of step does not
  matter).
* Anything else (first frame, scrub, re-solve, cache reload, changed thread count) -> `createFreshWorld`: a new shared
  state for this data only; other timesteps keep the old one. The empty mirror makes every point a new body.
* No save / load: reloaded sims rebuild. `getMemorySizeSubclass` reports the state's size
  (`Box3DSharedState::estimatedBytes`, set by `claimMemory` after each solve) from the one copy that stepped it last
  (`memoryOwner`), so sharing copies do not each count the world against the cache. `Box3DWorld::estimatedBytes` is
  Box3D's own per-world count (`b3World_GetCounters`, which includes the hulls cloned into its database) plus the
  meshes and height fields it owns; a rough figure, not exact. `getRequestedCacheSubclass` returns 1 (debug:
  `BOX3DDOP_REQUESTED_CACHE`).
* Because a rebuild has only the geometry, the write-back always writes `P`, `orient`, `v`, `w` and packed transforms.

`Box3DBodyMirror` (per body): the last pose / velocity given to or read from Box3D (to detect edits by other nodes),
`type` (`dynamic()` derives from it), `handle`, the point (`pointOff`, `pointId`, `pointIndex`, `objectId`), `stamp`,
the packed primitive and its 3x3, and what the body was built from (`props`, `volume`, `shapeSig`, `shapeless`,
`autoScale`, `sizeKey`, `animated`, `driven`, `sleeping`). Box3D's body user data points at the entry, so entries live
in `Box3DMirrorMap`'s slab of fixed-size chunks (never moved; a `std::deque` would give each of these 500-byte entries
its own heap block), found through a flat key table, and are erased only with their body. A fresh world's
`reserveMirror` allocates the chunks on the TBB pool, which touches their pages there, and `collectSlots` does the
same for the slot list. The entry's fields are ordered hot first (pose, handle, type, stamp, point, packed data: what
the write-back, forces and the quiet-point check read per body), then the settings it was built from; keep new
per-step fields in the first part.

Scratch reused every step: `slots` (per object: offset, key, cached entry, freshly read state and this scan's shape
decisions per point; a cached entry is trusted while offset and key match; when entries are erased, only the slots of
the objects that lost them forget their cached entries), `writeItems`, `moves`, `contacts`. `syncStamp` /
`stampedCount`: equal counts mean nothing to remove. `isLive(entry)` = its object was visited by this sync and the
entry carries the object's `liveStamp`; the entry keeps a pointer to its object's record (`sync`), valid because
records are erased only after their unvisited bodies are removed.
