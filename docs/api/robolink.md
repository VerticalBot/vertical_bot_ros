# RoboDK-compatible API (Robolink)

```{include} ../../studio/docs/python-api.md
:heading-offset: 1
```

## Clients

| Language | Package / file | Transport |
|---|---|---|
| Python | `studio/python/robodk` (drop-in for `robodk.robolink` / `robodk.robomath`), or the official `robodk` package | WebSocket JSON-RPC or TCP JSON-lines to the studio server |
| C# | `studio/clients/csharp/Robolink.cs` | TCP JSON-lines |
| C++ | `studio/clients/cpp/robolink.hpp` | TCP JSON-lines |
| MATLAB | `studio/clients/matlab/Robolink.m` | TCP JSON-lines |
| JavaScript | **Console (RoboDK API)** tab, plugins | in-process |

## Method coverage

Station: `Item`, `ItemList`, `ItemUserPick`, `AddFile`, `AddStation`, `AddFrame`, `AddTarget`, `AddProgram`,
`AddRobot`, `AddTool`, `AddFolder`, `AddShape`, `AddCurve`, `AddPoints`, `ProjectPoints`, `AddMachiningProject`,
`AddMobileRobot`, `BuildMechanism`, `Save`, `Copy`/`Paste`, `Delete`, `Collisions`, `CollisionItems`,
`Collision_Line`, `setCollisionActive`, `Cam2D_*`, `Spray_*`, `Calibrate_*`, `MeasurePose`,
`LaserTracker_Measure`, `RunProgram`, `RunCode`, `RunMessage`, `setRunMode`, `setSimulationSpeed`,
`SimulationTime`, `getParam`/`setParam`, `Command`, `EventsListen`/`WaitForEvent`, `PluginLoad`/`PluginCommand`,
`ShowRoboDK`/`HideRoboDK`, `setWindowState`, `Version`, `License`.

Item: `Name`, `Type`, `Parent`, `Childs`, `Valid`, `Visible`, `setVisible`, `Pose`/`setPose`, `PoseAbs`,
`PoseTool`, `PoseFrame`, `setPoseTool`, `setPoseFrame`, `Joints`, `setJoints`, `JointsHome`, `JointLimits`,
`setJointLimits`, `JointsConfig`, `SolveFK`, `SolveIK`, `SolveIK_All`, `FilterTarget`, `MoveJ`, `MoveL`,
`MoveC`, `MoveJ_Test`, `MoveL_Test`, `SearchL`, `setSpeed`, `setRounding`, `setDO`, `getDI`, `waitDI`,
`Instruction`, `InstructionCount`, `InstructionList`, `InstructionListJoints`, `setInstruction`,
`InstructionSelect`, `InstructionDelete`, `Update`, `MakeProgram`, `setMachiningParameters`, `setRobot`,
`setFrame`, `setTool`, `setLink`, `getLink`, `AttachClosest`, `DetachClosest`, `DetachAll`, `Recolor`,
`setColor`, `Scale`, `setParam`/`getParam`, `Connect`, `ConnectedState`, `setConnectionParams`, `Stop`,
`WaitMove`, `WaitFinished`.

See the {doc}`../reference/coverage` for the complete matrix.
