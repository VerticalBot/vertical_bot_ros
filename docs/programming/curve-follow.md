# Curve and point following

*Robot › Follow curve / points of an object…* — the general tool behind welding, gluing, deburring, spraying,
pruning and drilling programs.

## Inputs

- An object with **curves** (polylines with optional normals) or **points** (with optional normals) — from NC
  import, RoboDK projects (`GetPoints`), the API (`AddCurve`/`AddPoints`), or CSV files.
- The active robot and tool; optionally a carrier mechanism for external-axis optimisation.

## Options

| Option | Effect |
|---|---|
| Approach / retract | Distance along −Z of the tool before the first and after the last point |
| Step | Resample the curve at a fixed spacing (0 keeps the vertices) |
| Tool Z | Along the surface normal (into the surface), fixed −Z, or a custom direction |
| Spin | Rotation about the tool Z axis (torch lead angle) |
| Free tool Z | IK ignores rotation about the tool axis (symmetric tools) |
| Optimise tool Z | Tries spin candidates per point to keep joints comfortable (RoboDK “tool orientation optimisation”) |
| Preferred configuration | Keeps the elbow/wrist configuration of the first point |
| Speed, linear moves, IO | Path speed, MoveL vs MoveJ, output switched on during the path (Arc, Spray, Glue) |
| Carrier | Rail/turntable moved jointly with the arm |

Point follow adds approach/retract on every point.

## Result

A program under the station with a path frame at the object, targets `Approach`, `P1…Pn`, `Retract`, `Home`,
speed and IO instructions; unreachable points are counted and skipped. Move the object and re-run the dialog
(or `Update()` on the API) to regenerate.
