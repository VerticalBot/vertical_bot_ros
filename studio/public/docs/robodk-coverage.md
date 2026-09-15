# RoboDK feature & API coverage (honest status)

Legend: ✅ implemented · 🟡 partial / simplified · ❌ not implemented

## Application features

| RoboDK feature | Status | Notes |
|---|---|---|
| Station tree (frames, robots, tools, targets, programs, objects, folders) | ✅ | |
| Robot library (online library with hundreds of robots, .robot files) | 🟡 | 22 built-in models (UR exact DH, others approximate) + URDF/DH import; no .robot file support |
| Import geometry: STL, OBJ | ✅ | |
| Import geometry: STEP/IGES/SLDPRT, 3DS, WRL | ❌ | needs a CAD kernel (out of scope in browser) |
| .rdk station load/save | 🟡 | best-effort binary reader; lossless path is `rdk_export.py` + API script export |
| .robot / .tool file build ("Robot builder") | 🟡 | DH table import/export instead |
| Targets: cartesian / joint, teach, configuration flags | ✅ | flags derived (elbow/wrist/front), no explicit conf editing |
| Programs: MoveJ/MoveL/MoveC, speed, rounding, frame/tool, pause, IO, code, comment, message, call | ✅ | |
| Program instruction types: Wait for target (`WaitMove`), thread start | ❌ | |
| Program simulation, timeline, cycle time, run modes | ✅ | RUN_ON_ROBOT only via ROS bridge |
| Collision checking (mesh-accurate, collision map UI, `Collision_SetPair`) | 🟡 | capsules/OBB/mesh bounds + optional triangle checks; no collision map UI |
| Singularity / joint limit / reach checks in linear moves | ✅ | |
| Post processors (~100 vendor posts in RoboDK) | 🟡 | 12 posts (KUKA, ABB, Fanuc, UR, Motoman, Stäubli, Doosan, Mecademic, CSV, JSON, RoboDK, ROS 2); no Denso, Kawasaki, Nachi, Comau, Epson, Techman, Hanwha, Omron… |
| Post-processor customisation (Python post files) | ❌ | posts are TypeScript modules |
| Program import from controllers (KRL, RAPID, LS, URScript) | ✅ | others ❌ |
| Robot machining projects (curve/point follow, NC/G-code/APT) | 🟡 | curve/point follow + G-code; no APT/CAM plugins, tool orientation optimisation, turntable optimisation, feed conversion |
| External axes / turntables / rails synchronisation | 🟡 | combined IK; no automatic axis optimisation in programs |
| 3D printing / welding / spray add-ins | 🟡 | via curve follow with IO; no spray deposition simulation (`Spray_*`) |
| Conveyor tracking | 🟡 | process conveyors move products; no tracked picking |
| Robot drivers (live connection to KUKA/ABB/UR/… controllers) | ❌ | only ROS 2 via rosbridge |
| Robot calibration, ballbar, ISO 9283, laser tracker, TCP/frame calibration wizards | ❌ | |
| 2D/3D camera simulation (`Cam2D_*`) | 🟡 | camera view render + simulated fruit detections; no depth/segmentation output |
| Simulation events (attach/detach objects, show/hide) | ✅ | |
| Python API (`robolink`, `robomath`) | 🟡 | see tables below |
| C#, C++, MATLAB/Simulink APIs, plugin interface (C++ add-ins) | ❌ | JS + Python + JSON-RPC only |
| Multi-station tabs, copy/paste between stations | ❌ | one station per tab |
| Measurements, notes, ISO cube, layout dimensions | 🟡 | notes item only |
| Export simulation (3D HTML/PDF), video recording | ❌ | PNG screenshots only |

## Python API — Robolink

Implemented (39): `ActiveStation AddCurve AddFile(stub) AddFolder AddFrame AddPoints AddProgram AddShape AddStation AddTarget
Cam2D_Snapshot(stub) Collisions(stub) Command Connect Delete Disconnect Finish IsInside(stub) Item ItemList ItemUserPick License
ProjectPoints(stub) Render RunMode Save(stub) Selection ShowMessage SimulationSpeed Update Version getParam setParam setRunMode
setSelection setSimulationSpeed` + extensions `AddRobot AddMobileRobot App`.

Missing (documented in RoboDK): `AddMachiningProject AddMillingProject AddTargetJ AddMechanism/BuildMechanism BuildMechanism
Cam2D_Add Cam2D_Close Cam2D_SetParams Calibrate_Reference Calibrate_Robot CalibrateTool CloseRoboDK CloseStation
Collision_Line Collision_SetPair Collision_SetPairList CollisionItems CollisionPairs Copy Paste EmbedWindow
getOpenStations setActiveStation getFlagsItem setFlagsItem getFlagsRoboDK setFlagsRoboDK HideRoboDK ShowRoboDK setWindowState
Joints setJoints (multi-robot) LaserTracker_Measure MeasurePose StereoCamera_Measure MergeItems Popup_ISO9283_CubeProgram
PluginCommand PluginLoad RunCode RunMessage RunProgram setCollisionActive setCollisionActivePair setInteractiveMode setPoses
setRobotParams setViewPose ViewPose Spray_Add Spray_Clear Spray_GetStats Spray_SetState ShowSequence FilterTarget
getParams SimulationTime setSimulationTime EventsListen/WaitForEvent`.

## Python API — Item

Implemented (68): `AddFrame AddTarget AddTool Busy Childs Connect ConnectedState DOF Delete Instruction InstructionCount
InstructionDelete InstructionList JointLimits Joints JointsHome MakeProgram MoveC MoveJ MoveL Name Parent Pause Pose PoseAbs
PoseFrame PoseTool ProgramStart Recolor RunCode RunCodeCustom RunInstruction RunProgram ShowInstructions ShowTargets SolveFK
SolveIK SolveIK_All Stop Type Update Valid Visible WaitMove getLink getParam isJointTarget setAsCartesianTarget setAsJointTarget
setColor setDO setGeometryPose setJointLimits setJoints setName setParam setParent setParentStatic setPose setPoseAbs
setPoseFrame setPoseTool setRobot setRounding setSpeed setVisible setZoneData waitDI`.

Missing: `AttachClosest DetachClosest DetachAll Collision Copy Paste GeometryPose InstructionListJoints InstructionSelect
setInstruction JointsConfig setJointsHome setAccuracyActive AccuracyActive setAcceleration setAccelerationJoints setSpeedJoints
setLink ObjectLink Save setMachiningParameters MachiningParameters FilterTarget FilterProgram setRunType RunType WaitFinished
MoveJ_Test MoveL_Test SearchL Scale setColorShape setColorCurve Color setValue Value setAO getDI getAI customInstruction
addMoveJ addMoveL ConnectSafe ConnectionParams setConnectionParams Disconnect setPoseFrame(pose) JointPoses
setJointsConfig setDefaultTargetConfig setNoCurve setMillingParameters...`.

## Behavioural differences

- `MoveJ/MoveL` on a robot item move it instantly (no blocking animation in API mode).
- `MakeProgram(folder, post_id)` returns `(ok, code, filename)`; posts are selected by studio id.
- `Update()` returns `[valid, time_s, distance_mm, ratio, message]`.
- Poses are exact 4×4; units mm/deg as in RoboDK.

## Roadmap to close the gap (priority order)

1. Item: `InstructionListJoints`, `AttachClosest/DetachClosest/DetachAll`, `MoveJ_Test/MoveL_Test`, `setAccuracyActive`,
   `setSpeedJoints/setAcceleration*`, `Scale`, `Copy/Paste`, `setJointsHome`, `JointsConfig`.
2. Robolink: `AddTargetJ`, `Collision_SetPair*/CollisionPairs`, `setCollisionActive`, `Cam2D_Add/SetParams`, `setViewPose/ViewPose`,
   `SimulationTime`, `Copy/Paste`, `AddMachiningProject` (mapped to curve follow).
3. More post processors (Denso, Kawasaki, Nachi, Comau, Epson, Techman, Omron TM, Hanwha, Kinova…).
4. Mesh-accurate collision by default (BVH), collision map UI.
5. STEP import (WebAssembly OpenCascade) and `.robot` file support.
6. Machining project options (tool orientation optimisation, preferred configuration, rail/turntable optimisation).
