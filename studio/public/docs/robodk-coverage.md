# RoboDK feature & API coverage (honest status)

Legend: ✅ implemented · 🟡 partial / simplified · ❌ not implemented

## Application features

| RoboDK feature | Status | Notes |
|---|---|---|
| Station tree (frames, robots, tools, targets, programs, objects, folders) | ✅ | |
| Robot library (online library with hundreds of robots, .robot files) | 🟡 | 22 built-in models (UR exact DH, others approximate) + URDF/DH import; no .robot file support |
| Import geometry: STL, OBJ | ✅ | |
| Import geometry: STEP/IGES/BREP | ✅ | OpenCascade WebAssembly (occt-import-js), loaded on demand; SLDPRT/3DS/WRL ❌ |
| .rdk station load/save | 🟡 | best-effort binary reader; lossless path is `rdk_export.py` + API script export |
| .robot / .tool file build ("Robot builder") | ✅ | `BuildMechanism` (1R/1T/2R/2T/3R/3T/4R/4T/6DOF/7DOF/SCARA), `setRobotParams` (modified DH), DH import/export |
| Targets: cartesian / joint, teach, configuration flags | ✅ | flags derived (elbow/wrist/front), no explicit conf editing |
| Programs: MoveJ/MoveL/MoveC, speed, rounding, frame/tool, pause, IO, code, comment, message, call | ✅ | |
| Program instruction types: Wait, thread start | ✅ | threads compile on parallel timelines |
| Program simulation, timeline, cycle time, run modes | ✅ | RUN_ON_ROBOT only via ROS bridge |
| Collision checking (mesh-accurate, collision map, `Collision_SetPair`, `Collision_Line`) | ✅ | BVH triangle checks for meshes, collision map with pair rules + dialog, ray casting |
| Singularity / joint limit / reach checks in linear moves | ✅ | |
| Post processors (~100 vendor posts in RoboDK) | 🟡 | 27 built-in posts + any RoboDK Python post via Pyodide: KUKA KRC2/KRC4, ABB IRC5/S4C, Fanuc R30/RJ3, UR, Motoman, Stäubli, Doosan, Mecademic, Denso, Kawasaki, Nachi, Comau, Epson, Techman/Omron TM, Hanwha, Kinova, Mitsubishi, AUBO, JAKA, Elite, Dobot, CSV, JSON, RoboDK, ROS 2 |
| Post-processor customisation (Python post files) | ✅ | genuine RoboDK `RobotPost` Python posts run in the browser (Pyodide) or with CPython through `python/post_shim.py`; TypeScript posts also pluggable |
| Program import from controllers (KRL, RAPID, LS, URScript) | ✅ | others ❌ |
| Robot machining projects (curve/point follow, NC/G-code/APT) | 🟡 | curve/point follow, G-code, tool-Z orientation optimisation, preferred configuration, external-axis (rail) optimisation, `AddMachiningProject/setMachiningParameters`; APT/CAM plugins ❌ |
| External axes / turntables / rails synchronisation | ✅ | combined IK and rail-assisted curve following |
| 3D printing / welding / spray add-ins | ✅ | curve follow with IO + spray deposition simulation (`Spray_*` coverage statistics) |
| Conveyor tracking | 🟡 | process conveyors move products; no tracked picking |
| Robot drivers (live connection to controllers) | ✅ | server drivers: UR (URScript + real-time joints), ABB RWS, KUKA KUKAVARPROXY, generic ROS 2 (rosbridge + ros2_control, covers Fanuc/Yaskawa/Stäubli/Techman ROS drivers); `RUNMODE_RUN_ROBOT` from Python |
| Robot calibration, ballbar, ISO 9283, laser tracker, TCP/frame calibration | ✅ | numeric DH identification, ISO cube/ballbar programs and statistics, TCP by point/line, frame 3P/6P/turntable; measuring devices simulated |
| 2D/3D camera simulation (`Cam2D_*`) | 🟡 | `Cam2D_Add/SetParams/Snapshot/Close`, camera view render, camera parameters dialog, simulated detections; depth/segmentation buffers ❌ |
| Simulation events (attach/detach objects, show/hide) | ✅ | |
| Python API (`robolink`, `robomath`) | 🟡 | see tables below |
| C#, C++, MATLAB APIs, plugin interface | 🟡 | C# (WebSocket), C++ and MATLAB (TCP JSON-lines) clients in `clients/`; JS plugins via `PluginLoad` (ES modules); Simulink and C++ add-ins ❌ |
| Multi-station tabs, copy/paste between stations | ✅ | station tab bar, clipboard Copy/Paste (UI + API) |
| Measurements, notes, ISO cube | ✅ | Tools › Measure (distance/angle), notes, ISO 9283 cube program |
| Export simulation (3D HTML), video recording, glTF | ✅ | self-contained 3D HTML viewer, WebM recording of the 3D view, glTF export; 3D PDF ❌ |

## Python API — Robolink (updated)

Now implemented in addition to the original set: `AddTargetJ AddMachiningProject AddMillingProject BuildMechanism setRobotParams
Cam2D_Add Cam2D_SetParams Cam2D_Close Cam2D_Snapshot Calibrate_Reference CalibrateTool Calibrate_Robot LaserTracker_Measure
MeasurePose StereoCamera_Measure Popup_ISO9283_CubeProgram BallbarProgram Collision_Line Collision_SetPair Collision_SetPairList
setCollisionActive setCollisionActivePair Collisions CollisionItems CollisionPairs Copy Paste Duplicate getOpenStations
setActiveStation CloseStation CloseRoboDK getFlagsItem setFlagsItem getFlagsRoboDK setFlagsRoboDK HideRoboDK ShowRoboDK
setWindowState setInteractiveMode setViewPose ViewPose Joints setJoints setPoses MergeItems RunCode RunMessage RunProgram
ShowSequence FilterTarget getParams SimulationTime setSimulationTime Spray_Add Spray_SetState Spray_GetStats Spray_Clear
EventsListen WaitForEvent PluginLoad PluginCommand ProjectPoints IsInside Save`.

Previously implemented (39): `ActiveStation AddCurve AddFile(stub) AddFolder AddFrame AddPoints AddProgram AddShape AddStation AddTarget
Cam2D_Snapshot(stub) Collisions(stub) Command Connect Delete Disconnect Finish IsInside(stub) Item ItemList ItemUserPick License
ProjectPoints(stub) Render RunMode Save(stub) Selection ShowMessage SimulationSpeed Update Version getParam setParam setRunMode
setSelection setSimulationSpeed` + extensions `AddRobot AddMobileRobot App`.

Still missing / stubbed: `EmbedWindow` (no native windows in a browser), `AddFile` from a local path (browser sandbox:
use drag & drop or the station file). Measurement functions return simulated values (configurable noise) unless a real
device is bridged through the server.

## Python API — Item (updated)

Now implemented in addition: `AttachClosest DetachClosest DetachAll Collision Copy Paste GeometryPose InstructionListJoints
InstructionSelect setInstruction JointsConfig setJointsHome setAccuracyActive AccuracyActive setAcceleration
setAccelerationJoints setSpeedJoints setLink ObjectLink Save setMachiningParameters MachiningParameters FilterTarget
FilterProgram setRunType RunType WaitFinished MoveJ_Test MoveL_Test SearchL Scale setColorShape setColorCurve Color setValue
Value setAO getDI getAI customInstruction addMoveJ addMoveL ConnectSafe ConnectionParams setConnectionParams Disconnect
JointPoses setRobotParams RobotParams`.

Previously implemented (68): `AddFrame AddTarget AddTool Busy Childs Connect ConnectedState DOF Delete Instruction InstructionCount
InstructionDelete InstructionList JointLimits Joints JointsHome MakeProgram MoveC MoveJ MoveL Name Parent Pause Pose PoseAbs
PoseFrame PoseTool ProgramStart Recolor RunCode RunCodeCustom RunInstruction RunProgram ShowInstructions ShowTargets SolveFK
SolveIK SolveIK_All Stop Type Update Valid Visible WaitMove getLink getParam isJointTarget setAsCartesianTarget setAsJointTarget
setColor setDO setGeometryPose setJointLimits setJoints setName setParam setParent setParentStatic setPose setPoseAbs
setPoseFrame setPoseTool setRobot setRounding setSpeed setVisible setZoneData waitDI`.

Behavioural notes: `Collision`/`MoveJ_Test`/`MoveL_Test` use the collision map and ignore resting contacts present at the
start of the move; `FilterProgram`/`setAccuracyActive` are no-ops because the simulated kinematics are nominal (calibrated
robots replace their DH tables instead).

## Behavioural differences

- `MoveJ/MoveL` on a robot item move it instantly (no blocking animation in API mode).
- `MakeProgram(folder, post_id)` returns `(ok, code, filename)`; posts are selected by studio id.
- `Update()` returns `[valid, time_s, distance_mm, ratio, message]`.
- Poses are exact 4×4; units mm/deg as in RoboDK.

## Remaining gaps

- Proprietary binary containers (`.rdk`, `.robot`, `.tool`) stay best-effort; the lossless path is the API bridge.
- Native (non-ROS) drivers exist for UR, ABB (RWS) and KUKA (KVP); other brands connect through their ROS 2 drivers.
- Depth/segmentation camera buffers, 3D PDF export, Simulink blocks and C++ add-ins are not implemented.
- `EmbedWindow` and file-path based `AddFile` do not apply to a browser application.
