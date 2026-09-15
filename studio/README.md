# VerticalBot Studio

Browser-native robot simulation and offline programming platform in the class of **RoboDK** and
**Visual Components**, extended to **mobile robotics, fleet management and agricultural robotics**
(orchards, vineyards, greenhouses, field crops and the surrounding logistics). No install: it runs
in the browser; a small Node server adds a RoboDK-compatible Python API and headless batch use.

Part of the `vertical_bot_ros` repository — the ROS 2 packages in the repo root (vertical robot,
palletizer) can be imported directly (URDF/xacro + STL) and driven through the ROS 2 post processor
or the rosbridge connection.

## Features

**Industrial robot OLP (RoboDK class)**
- Station tree: frames, robots, tools (TCP), targets (cartesian/joint), programs, objects, folders, cameras, notes.
- Kinematics engine: generic serial chains (URDF-style origin/axis joints, DH tables, mimic joints, prismatic axes),
  forward kinematics, geometric Jacobian, damped-least-squares IK with limits, restarts, automatic orientation
  relaxation for < 6-DOF robots (palletizers, SCARA, gantries), multi-solution IK.
- Robot library: UR3e/5e/10e/16e/20 (official DH), KUKA Agilus/Cybertech/Quantec, ABB IRB 120/1200/2600/6700,
  Fanuc LR Mate/M-20iA/M-410iC, Yaskawa GP12, Stäubli TX2-60, Doosan M1013, Mecademic Meca500, generic palletizer,
  SCARA, XYZ gantry, 7-DOF telescopic harvesting arm — plus any URDF/xacro import with STL meshes.
- Programs: MoveJ/MoveL/MoveC, speed/acceleration, rounding (blending), frames/tools, pauses, digital IO,
  gripper/attach/detach events, raw code, comments/messages, sub-program calls, mobile navigation and signals.
- Simulator: trapezoidal joint/cartesian trajectory generation, cycle time, TCP distance, reachability and
  singularity/joint-jump detection, object attachment, timeline scrubbing, speed factor.
- Post processors: KUKA KRC4 (KRL .src/.dat), ABB RAPID (.mod), Fanuc (.ls), Universal Robots (URScript),
  Yaskawa Motoman (INFORM .JBI), Stäubli VAL3 (.pgx), Doosan (DRL), Mecademic (Python), generic CSV, JSON,
  ROS 2 (rclpy node + JointTrajectory JSON), **RoboDK API Python script** (rebuilds the program inside RoboDK).
- Program importers: KRL, RAPID, Fanuc LS, URScript, CSV → targets + instructions.
- Pose conventions: RoboDK XYZ-Rx-Ry-Rz, KUKA ABC, Fanuc/Motoman WPR, ABB quaternion, UR rotation vector, ZYZ.

**RoboDK project interoperability** (see `docs/robodk-compatibility.md`)
- RoboDK-compatible scripting API (JavaScript in the built-in console and a **Python drop-in `robodk` package**
  in `python/`): `Robolink`, `Item`, `AddFrame/AddTarget/AddProgram`, `MoveJ/MoveL/MoveC`, `SolveFK/SolveIK`,
  `setPoseFrame/setPoseTool`, `Update`, `MakeProgram`, `RunInstruction`, `setDO/waitDI`, …
- Studio server on port 20500 (RoboDK's API port) relays API calls into the live browser session.
- `python/rdk_export.py` runs inside RoboDK and dumps a station losslessly to the studio format;
  `File > Export station as RoboDK API script` rebuilds a studio station inside RoboDK (then save as .rdk).
- Best-effort `.rdk` reader (the container is proprietary): recovers names, poses and embedded meshes.
- Target CSV/TXT import/export in RoboDK conventions.

**Process simulation (Visual Components class)**
- Behaviour components: feeder, conveyor (path, speed, spacing, stop signal), process/machine (cycle time,
  capacity, MTBF/MTTR failures), buffer/pallet grid, transfer/human, sensor, sink. Signals, statistics
  (throughput, utilisation, blocking, WIP), products as live 3D objects.

**Mobile robotics & fleets**
- Mobile robot item: differential/Ackermann/omni/tracked kinematics, battery model, sensors, capabilities,
  ROS namespace; arms and sensors mount on platforms (full pose chain).
- Occupancy grid maps (inflation, rasterised from fields/objects), A* with smoothing, pure-pursuit tracking,
  coverage (boustrophedon) and orchard row traversal planners, travel time estimation.
- Fleet manager: task queue, cost-based auction / nearest / round-robin allocation, per-row traffic reservations,
  charging policy, KPIs (throughput, utilisation, distance, energy, wait), live dashboard.

**Agriculture**
- Field/orchard generator: 15 crop presets (apple, pear, cherry, citrus, grape, strawberry, tomato, cucumber,
  blueberry, olive, almond, kiwi, …) with training systems, row/plant spacing, headlands, procedural canopies and
  fruit with ripeness; greenhouse (indoor) mode; GeoJSON field import/export with WGS84 ↔ local conversion.
- Missions: harvest, spray, mow, prune, scout, weed, pollinate, transport, thin, irrigate → fleet tasks per row
  side with traffic segments; progress tracking.
- Arm harvesting program generator: reachable ripe fruit → approach/pick/retreat targets + gripper events.

**ROS 2**
- rosbridge client: publishes `/cmd_joint_state`, `/cmd_point`, `/tcp_pose`, `/cmd_vel`, `/robot_pose`,
  `/battery_state` (namespaced), follows `/joint_states` and `/odom` (digital twin).
- ROS 2 post generates an rclpy node compatible with the controllers in this repository.

## Quick start

```bash
cd studio
npm install
npm run dev          # http://localhost:5173  (?demo=orchard | pickplace | packing | welding | greenhouse | verticalbot)
npm test             # unit tests (kinematics, posts, importers, planners, fleet, agri, API)
npm run build        # static build in dist/
npm run server       # RoboDK-compatible API server on ws://localhost:20500
```

Open the studio with `?server=ws://localhost:20500` to let Python scripts drive the browser session:

```bash
PYTHONPATH=studio/python python studio/python/examples/hello_studio.py
```

Import this repository's robots: drag `vertical_robot_model/urdf/vertical_robot.urdf` together with the STL
files from `vertical_robot_model/meshes/` onto the 3D view (or `palletizer_model_pkg/urdf/*.xacro` + meshes).

## Layout

```
studio/
  src/core        pose math, item tree, kinematics (FK/IK/DH), motion planning, program simulator, robot library
  src/io          station file, URDF/xacro/XML, STL, RoboDK (targets, .rdk, station script), program importers
  src/posts       post processors (KUKA, ABB, Fanuc, UR, Motoman, Stäubli, Doosan, Mecademic, CSV, JSON, RoboDK, ROS 2)
  src/vc          Visual-Components-style process components and simulator
  src/mobile      mobile robot items, maps, planners, controllers
  src/fleet       fleet manager (tasks, allocation, traffic, charging, KPIs)
  src/agri        geo, fields/rows/missions, orchard generator, mission planners, harvest program generator
  src/api         RoboDK-compatible JS API + JSON-RPC + browser bridge
  src/ros         rosbridge client
  src/scene       three.js renderer & assets
  src/ui          panels, dialogs, menu, console
  server/         WebSocket/HTTP server (relay + headless)
  python/         robodk drop-in package, rdk_export.py, examples
  tests/          vitest suites
  docs/           architecture, RoboDK compatibility, file formats, agriculture workflow
```

## Status and honest limits

- Library robots not marked *(official DH)* use representative geometry; import the vendor URDF for exact kinematics.
- The `.rdk` binary container is proprietary and undocumented; the reader is best-effort. Use `rdk_export.py`
  (inside RoboDK) or the API bridge for lossless transfer.
- Collision checking is bounding-box based (attachment proximity, map rasterisation); mesh-level collision
  is on the roadmap (three-mesh-bvh).
- ROS 2 integration is via rosbridge websocket; native DDS is out of scope for a browser.

License: MIT.
