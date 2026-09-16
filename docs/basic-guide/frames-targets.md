# Reference frames and targets

## Reference frames

Frames are coordinate systems for parts, fixtures, tables, conveyors and cameras. Create them with
*Add › Reference frame*, position them numerically or with the gizmo, nest them (a frame under a frame) and
attach objects under them so a fixture and its part move together.

The orientation entry supports the conventions of every major controller (RoboDK XYZ + Rx Ry Rz, KUKA A B C,
Fanuc / Motoman W P R, ABB quaternion, UR rotation vector, Euler ZYZ) — the pose is the same matrix, only its
display changes, so values can be typed straight from a teach pendant.

Frames can be calibrated from measured points (3 points, 6 points, turntable) through the API
(`Calibrate_Reference`), see {doc}`../programming/calibration`.

## Targets

A target stores a pose relative to its parent frame **and** the joint values used when it was taught.

- **Cartesian target** — the pose is authoritative; IK uses the stored joints as the configuration hint.
- **Joint target** — the joints are authoritative (typical for home and via positions); the pose is derived.

Switch the type in the properties panel (*Target › Cartesian / Joint*). Teach targets with `J`/`L`, *Add ›
Target (teach)*, the context menu *Target (at robot TCP)*, or import them from CSV / controller programs.

Moving a target with the gizmo or by editing its pose updates every move that references it. Right-click a
target for *Move robot here* (MoveJ / MoveL), *Teach current position* and *Duplicate*.

## Tips

- Keep approach and retract targets relative to the part frame so a re-measured frame realigns them.
- For symmetric tools let IK ignore the rotation about the tool Z axis (curve follow / machining options).
- Targets of imported controller programs keep their names (`P[1]`, `pApproach`, `XP1`) for traceability.
