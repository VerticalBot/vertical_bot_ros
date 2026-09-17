# vertical_bot_ros

Repository of the **VerticalBot** project: the ROS 2 packages of the vertical robot / palletizer platform and
**VerticalBot Studio** — a browser-native simulation, offline-programming and perception/navigation engineering
tool in the class of RoboDK and Visual Components, extended to mobile robots, fleets and agriculture.

| Part | What it is |
|---|---|
| `studio/` | VerticalBot Studio: web app (Vite + TypeScript + three.js), Node server with a RoboDK-compatible API, Python `robodk` drop-in package, Blender add-on, tests and demo scenarios |
| `docs/` | User and developer documentation (Sphinx + MyST) published on **Read the Docs**: https://vertical-bot-ros.readthedocs.io |
| `vertical_robot_model`, `vertical_robot_base_pkg`, `palletizer_model_pkg`, `palletizer_control_pkg`, `controller` | ROS 2 packages of the physical robots (URDF/xacro models, position control, palletizer control) |

## VerticalBot Studio in one paragraph

Open a station in the browser, add robots from the built-in or online library (91 vendor URDFs), import URDF /
STEP / glTF / RoboDK files, teach targets, write and simulate programs, check collisions and cycle time, and export
to KUKA, ABB, Fanuc, UR, Motoman, Stäubli, Doosan, Mecademic or ROS 2 — the same workflow as RoboDK, including
RoboDK's own Python post processors and API. On top of that the studio models **process lines** (Visual
Components style), **mobile platforms and fleets** (VDA 5050, task allocation, traffic), **orchards, vineyards and
greenhouses** with harvesting / spraying / scouting missions, and three engineering layers that turn the simulation
into a design tool for real machines:

- **Navigation & SLAM stack** — choose localization and navigation for a platform and environment (2D/3D LiDAR
  SLAM, LIO, visual SLAM / VIO, RTK / INS, UWB, tape / QR / reflectors / rail, hybrids), simulate its errors,
  outages and losses while the fleet drives, export a Nav2 + SLAM + EKF ROS 2 package.
- **Machine-vision stack (СТЗ)** — choose sensor, compute and models per task (YOLO / DETR / SAM / trackers /
  6D pose / depth / point-cloud nets / VLM / VLA), run the pipeline on the station's cameras with simulated ground
  truth or **real models plugged in** (ONNX Runtime Web, inference servers, OpenAI-compatible VLMs, VLA policy
  servers, ROS 2 `vision_msgs`), get grasp targets, following behaviour and point-cloud analysis, export a ROS 2
  perception package.
- **Control design (the theory of control of robotic complexes as features)** — design the upper control level of
  any robot or complex of robots in the Control tab: automata and Ramadge–Wonham supervisory control (synthesis,
  modular non-conflict, observability, diagnosability), Petri nets (invariants, siphons, deadlock prevention with
  GMEC monitors, GSPN performance, max-plus cycle time), behavior trees and statecharts, LTL / CTL model checking
  with counterexamples, GR(1) reactive synthesis, hybrid modes with hysteresis and CBF filters, PDDL / HTN planning,
  STN / STNU, MDP / POMDP decisions, task allocation and MAPF, job-shop scheduling, real-time analysis, reliability
  and functional safety (FMEA, FTA, ISO 13849 PL, ISO/TS 15066), STL falsification and acceptance statistics —
  with deadlock / livelock / realisability checks, missions that run on the station robots under the synthesised
  supervisor with runtime monitors, and supervisor export (JSON table, Python) for real controllers. Automata,
  Petri nets and behavior trees are also drawn in **graphical editors** synchronised with the text, and their states,
  transitions and action leaves are bound to the station's own robot programs, targets, zones and signals, so an
  automaton, a net or a tree runs the cell. The two worked
  examples of the course (mobile manipulator, production cell) are built in with step-by-step guides.

Everything is verified by 236 unit / integration tests and 45 published demo scenarios (one per localization
method, vision task and control-design method), and wired to the outside world through documented ports and contracts
(RoboDK-compatible API on 20500/20501, rosbridge topics, VDA 5050 over MQTT, HTTP inference / VLM / VLA
contracts, webhooks).

```bash
cd studio && npm install
npm run dev          # http://localhost:5173  (?demo=orchard | pickplace | tutorial | packing | welding | greenhouse | verticalbot)
npm test             # 236 tests
npm run scenarios    # 45 demo scenarios → docs/scenario-results.md
npm run server       # RoboDK-compatible API + drivers + VDA 5050 + vision inference (ws/http :20500, tcp :20501)
```

- Documentation: https://vertical-bot-ros.readthedocs.io — start with *Getting started › Your first station*;
  for wiring see *Developer › Integration: ROS 2, custom nodes, protocols and ports*; results in *Reference › Demo scenarios*;
  the control-design layer and the course demo guides are under *Control design (course methods)*.
- Roadmap / status: `studio/TODO.md` (published as the *Roadmap* page). Developer notes: `studio/README.md`, `studio/docs/`.

## ROS 2 packages

Start the URDF model publisher:

```bash
ros2 launch vertical_robot_model model.launch.py
```

Start the node that sets the robot position and orientation:

```bash
ros2 run vertical_robot_base_pkg position_control.py
```

- Robot position: topic `/robot_pose`, type `geometry_msgs/Pose`.
- Joint positions: topic `/joint_states`, type `sensor_msgs/msg/JointState`, joint names
  `[joint_1, joint_2, joint_1_1, joint_1_2, joint_2_1, joint_2_2]` with `joint_1_2 = joint_1_1` and
  `joint_2_1 = joint_2_2`.

Platform with manipulator (`ros2 launch vertical_robot_model model.launch.py`):

| Interface | Topic | Type | Notes |
|---|---|---|---|
| Cartesian command | `/cmd_point` | `std_msgs/msg/Float32MultiArray` | `data: [x, y, z]` |
| Joint command | `/cmd_joint_state` | `sensor_msgs/msg/JointState` | e.g. `position = [0.0, 0.0, 0.0, 0.0]` |
| Robot state | `/palletizer_robot_state` | `std_msgs/msg/Int32` | 0 — idle, 1 — moving |

The studio's ROS 2 post processor and rosbridge connection speak exactly these topics, so programs written in the
studio run on the physical robot; the robots themselves are imported from `vertical_robot_model/urdf` and
`palletizer_model_pkg/urdf` with their meshes.
