# Programs

A program is a list of instructions executed by one robot (or a mobile robot). The **Program** tab is the
editor; the **Simulation** tab runs it.

## Instructions

| Instruction | Notes |
|---|---|
| MoveJ / MoveL / MoveC | Joint, linear and circular moves to targets (MoveC needs a via target); per-move speed and rounding overrides |
| Speed | Linear speed (mm/s), joint speed (deg/s), accelerations |
| Rounding | Blending radius (mm); `-1` = fine point |
| Set reference frame / Set tool | Change the active frame or tool for the following moves |
| Pause | Milliseconds, or wait for the operator |
| Set DO / Wait DI | Digital IO by name/number with optional timeout |
| Gripper close/open, attach/detach, show/hide | Simulation events on the nearest object or a named object |
| Code / Call | Raw controller code inserted by the post, or a call to another program |
| Comment / Message | Comment in the generated code / message shown during simulation |
| Signal / Wait signal | Station-wide signals for multi-robot and process synchronisation |
| Thread | Run another program in parallel on its own timeline |
| Navigate / Follow path / Mission task | Mobile robot instructions |
| Joint path | Replays a recorded joint trajectory (RoboDK `InstructionListJoints`, ROS bags, CSV logs) exactly |

Add instructions with the *Add:* buttons (they are inserted after the selected row), drag rows to reorder,
toggle the checkbox to disable a row, double-click a move to jump the robot there.

## Teaching

`J` / `L` teach the current robot position as a new target under the active frame and append the move to the
active program. *Program › New program* starts another program for the active robot; the active program is
selected in the tab's drop-down.

## Validation

*Program › Validate (compile)* simulates the whole program without rendering and reports:

- cycle time and TCP path length,
- unreachable targets, joint-limit violations, large joint jumps between consecutive points,
- linear moves crossing singularities (with the residual and a suggestion),
- collisions along the sampled trajectory when *Robot › Check collisions during program validation* is on,
- missing programs in `Call`, missing via points, programs without a robot.

Problems are listed per instruction and highlighted in the editor.

## Importing programs

Drop controller programs onto the 3D view: KUKA `.src/.dat`, ABB `.mod/.prg`, Fanuc `.ls`, URScript
`.script`, CSV target lists, G-code/NC ({doc}`../programming/machining`). Moves become targets under the
active frame; speeds, IO and comments are preserved where the language allows.

## Sub-programs and threads

`Call` executes another program inline (nested calls allowed). `Thread` starts it in parallel — the cycle
time covers all threads and the simulator runs them on separate timelines, which is how multi-robot cells and
process components are coordinated together with `Signal` / `Wait signal`.
