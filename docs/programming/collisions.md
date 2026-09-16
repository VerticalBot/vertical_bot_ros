# Collision detection

## Colliders

| Item | Collider |
|---|---|
| Robot links without meshes | Capsules along the procedural links, hubs at the joints |
| Robot links with meshes | Oriented bounding boxes (fast) or triangle-accurate mesh checks (option) |
| Objects: box / cylinder / sphere | Exact primitives |
| Objects: meshes | Bounding boxes, optional mesh BVH |
| Tools | Capsule from the flange to the TCP plus tool geometry |

Adjacent robot links and resting contacts (a box standing on a table) are ignored so that valid stations
report zero collisions; the baseline is recomputed when items are attached/detached.

## Checks

```{image} ../_static/screens/14-collision-check-log.png
:alt: Collision check in the log
:class: screenshot
```

- **Static** — *Robot › Check collisions now (static)*; API `Collisions()`, `CollisionItems()`, `CollisionPairs()`.
- **During validation** — *Robot › Check collisions during program validation*: every trajectory is sampled
  (step configurable) and the first colliding pair is reported per instruction; items turn red.
- **Move tests** — API `MoveJ_Test`, `MoveL_Test` return the collision status of a planned move; `Collision_Line`
  casts a segment through the scene (sensors, laser lines).

## Collision map

```{image} ../_static/screens/15-collision-map.png
:alt: Collision map dialog
:class: screenshot
:width: 640px
```

*Tools › Collision map…* (API `Collision_SetPair`, `setCollisionActivePair`) enables or disables pairs and whole
groups, e.g. tool vs part while gripping, or two robots that never share space.

## Tips

- Give tools and fixtures real geometry: capsules are conservative.
- For mesh-heavy stations enable triangle checks only for the pairs that matter.
