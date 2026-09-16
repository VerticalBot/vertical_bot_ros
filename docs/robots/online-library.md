# Online robot library

*Add › Robot from online library (ROS-Industrial)…* downloads robot descriptions straight from GitHub and
builds them in the browser — the open counterpart of RoboDK's online library.

## What you get

- **91 robots**: Fanuc (LR Mate 200i…200iD, M-6iB, M-10iA, M-16iB, M-20iA/iB, M-430iA, CR-7iA, CR-35iA,
  M-710iC, M-900iA/iB, R-1000iA, R-2000iC), ABB (IRB 120, 2400, 4400L, 5400, 6600, 6640), KUKA (KR 3, KR 5 arc,
  KR 6/10 Agilus, KR 16, KR 120/150/210, LBR iiwa 14), Yaskawa Motoman (GP4…GP200R, HC10/HC20, MH5…MH110,
  MA2010, SIA5/10/20), Stäubli (TX60, TX90, TX2-60/90/90L, RX160), Universal Robots (UR3/5/10, UR3e/5e/10e),
  Franka Emika Panda, Kinova Gen3 / Gen3 lite, Doosan (M0609…M1509, A0509/A0912, H2017/H2515).
- **Exact kinematics** from the vendor URDF (joint origins, axes, limits), **vendor meshes** (STL/COLLADA),
  the ROS-Industrial `tool0` flange convention, brand and default post processor.
- Sources: `ros-industrial/{fanuc,abb,kuka_experimental,motoman,staubli_experimental,staubli,universal_robot}`,
  `frankaemika/franka_ros`, `Kinovarobotics/ros_kortex`, `doosan-robotics/doosan-robot` (BSD / Apache licences).

## How it works

1. The top-level xacro is fetched from `raw.githubusercontent.com`; `xacro:include` targets (`$(find pkg)`,
   `package://`, relative paths — including names known only after macro expansion) are resolved to the
   hosting repository and fetched recursively.
2. The built-in xacro processor expands properties, macros, conditionals and math; the URDF becomes a serial
   chain ending at `tool0` (or `flange` / `tool_frame`).
3. Visual meshes are downloaded in parallel and registered under their `package://` URIs; COLLADA keeps metres.

The same resolver serves URDFs you drop yourself: `package://` meshes of a known package are fetched
automatically when they are missing.

## Requirements and limits

- Internet access from the browser to `raw.githubusercontent.com` (CORS is allowed by GitHub).
- Robots defined through YAML-parameterised macros (newer UR ROS 2 descriptions) are provided through the
  classic `ur_description` / `ur_e_description` packages instead.
- Mesh downloads for large robots take a few seconds; untick *Download 3D meshes* to get kinematics only.

Programmatic use: `fetchOnlineRobot(id, { assets })` and `fetchRobotFromUrl(url)` in
`src/io/library/online_library.ts`.
