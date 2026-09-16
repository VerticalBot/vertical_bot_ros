# FAQ and troubleshooting

**Can I open my RoboDK `.rdk` stations?**
Yes, losslessly when the studio server runs next to a RoboDK installation (free version is enough); otherwise a
best-effort import recovers names, poses and meshes. See {doc}`../interop/robodk`.

**Where do the robots with meshes come from?**
From the open ROS-Industrial / vendor URDF packages on GitHub, downloaded by the browser
({doc}`../robots/online-library`). No account or licence needed.

**The online library says “Could not download”.**
The browser needs access to `raw.githubusercontent.com`. Corporate proxies that intercept TLS or block GitHub
prevent it; import the URDF package manually instead.

**My robot reaches the target in RoboDK but not here.**
Check the tool TCP and the reference frame first (properties › Status shows TCP in the active frame). Then the
configuration: double-click a taught target to adopt its joints as the seed. Library robots marked *approx.* may
differ slightly from the vendor geometry — use the online library model.

**Programs run but export shows a different post.**
The export dialog uses the robot's default post unless you pick another; set it in properties › Kinematics.

**A program stops with “crosses a singularity”.**
A linear move passes a wrist or elbow singularity. Change the approach configuration, split the move, or use
MoveJ for that segment. The message includes where along the move it happens.

**Collisions are reported for objects that are just touching.**
Resting contacts present at program start are ignored; if you attach objects mid-program the baseline is
recomputed. Use *Tools › Collision map* to disable pairs deliberately.

**The world simulation does not move mobile robots.**
Start it with *Mobile & Fleet › Start world simulation*; robots need a fleet with tasks or a `Navigate`
instruction, and a map (create one or generate a field).

**How do I reset everything?**
*File › New station*. Browser storage keeps user posts and preferences only.

**Where are logs?**
The **Log** tab; the server prints to stdout. For bug reports export the station (`.vbstation`) and the log.
