# Navigation & SLAM stack selection

Every mobile platform needs two decisions: **how it localises and maps** (SLAM / odometry / global
positioning) and **how it navigates** (planner and controller, or fixed routes). The studio holds a catalogue of
both, recommends a combination for the platform, the environment and the sensors on board, simulates the
localization behaviour of the chosen stack so fleets are debugged against realistic errors, and exports a
ready-to-tune ROS 2 package (Nav2 + SLAM + EKF + launch).

```{image} ../_static/screens/35-navstack-dialog.png
:alt: Navigation & SLAM stack wizard
:class: screenshot
:width: 760px
```

## Choosing a stack

*Mobile & Fleet › Navigation & SLAM stack…* (or *Select stack…* in the robot properties) opens the wizard:

1. **Platform** — AMR (free navigation), AGV with natural navigation (laser / reflectors), AGV on tape / QR
   grid, autonomous tractor (Ackermann), tracked / skid-steer platform, legged robot, pipe-rail trolley.
2. **Environment** — orchard (canopy → GNSS-denied rows), vineyard, open field, greenhouse, warehouse, factory,
   forest, urban yard, underground, mixed.
3. **Sensors** on the vehicle (the platform preset fills a default list): wheel odometry, IMU, 2D/3D LiDAR,
   mono / stereo / RGB-D camera, GNSS standard / RTK, UWB tag, tape sensor, floor QR reader, reflector survey,
   radar, rail encoder.
4. **Constraints** — GNSS sky view (0–1), night operation, need for a global repeatable frame (row entries,
   docking, fleet coordination), hardware cost budget, compute budget.

The list on the right ranks the localization methods with a score, the reasons (“fuses IMU, wheel odometry”,
“bridges GNSS outages with onboard odometry”), the warnings (“camera-based: not usable at night”, “needs 3D
LiDAR”, “GNSS available only 25 % of the time”) and the real software behind each choice. Selecting an entry
also selects the matching navigation method for the platform.

### Localization methods in the catalogue

| Family | Methods | Typical use |
|---|---|---|
| 2D LiDAR SLAM | slam_toolbox, Cartographer (+ AMCL on the saved map) | warehouses, factories, greenhouses |
| 3D LiDAR / LiDAR-inertial | LIO-SAM, FAST-LIO2, KISS-ICP | orchards, forests, yards, tunnels |
| Visual | ORB-SLAM3 mono, ORB-SLAM3 visual-inertial, OpenVINS, VINS-Fusion, RTAB-Map (RGB-D / stereo) | low-cost platforms, indoor and daylight outdoor |
| Global positioning | GNSS RTK (+ heading), GNSS/INS dead reckoning, UWB beacons | open fields, greenhouses (UWB) |
| Infrastructure AGV | magnetic / optical tape, QR floor grid, laser reflector triangulation, rail encoder | fixed-route AGVs, goods-to-person, pipe rails |
| Hybrid | RTK + LiDAR-inertial odometry, RTK + visual-inertial odometry, UWB + 2D LiDAR SLAM | orchard and vineyard fleets, greenhouses |

Each entry carries suitability per environment, sensor requirements, typical accuracy, drift per metre,
infrastructure needs, compute and cost classes, night capability and notes.

### Navigation methods

Nav2 combinations (Smac Hybrid-A* + Regulated Pure Pursuit for Ackermann and large footprints, NavFn + DWB for
indoor differential drives, Smac 2D + MPPI for dynamic obstacles), crop-row following with headland turns,
fixed-route line following (tape / QR, VDA 5050 order graphs), GNSS waypoint / coverage following and 1-D rail
position control.

## Simulating the chosen stack

With *Simulate localization* enabled, the world simulation keeps two poses per robot: the **ground truth** and
the **estimate** produced by an error model of the selected method — noise, drift proportional to distance,
heading drift, monocular scale drift, loop closures that pull the drift back, GNSS outages inside
`gnss_denied` zones (the orchard generator adds a canopy zone automatically) or indoors, localization loss in
feature-poor areas for camera methods, and recovery. The pure-pursuit controller steers from the **estimate**,
so a drifting stack really makes the vehicle leave its lane, and a lost robot stops until it relocalises. The
fleet KPIs and the Navigation tab show what happens.

```{image} ../_static/screens/34-navigation-tab.png
:alt: Navigation tab
:class: screenshot
```

The **Navigation** tab (bottom panel) draws the occupancy map, GNSS-denied zones (purple), the SLAM map built
from simulated 2D LiDAR scans taken from the *estimated* pose (so localization error distorts it, like a real
map), the true (green) and estimated (orange) trajectories, the planned path and the last scan; the header
reports the error, RMSE, maximum error, GNSS state, tracking/lost status, loop closures and map coverage.

## Exporting to ROS 2

*Export ROS 2 package* (properties, Navigation tab or *Mobile & Fleet › Export ROS 2 navigation package…*)
writes a colcon package with:

- `config/nav2_params.yaml` — planner, controller (Regulated Pure Pursuit / MPPI / DWB with the vehicle's
  speed, acceleration, yaw-rate and turning-radius limits), costmaps with the vehicle footprint and obstacle
  sources matching the sensors (`scan`, `points`, depth), behaviours, velocity smoother, waypoint follower;
- the SLAM / odometry configuration of the chosen method (`slam_toolbox.yaml` + `amcl.yaml`,
  `cartographer_2d.lua`, `lio_sam.yaml`, `fast_lio.yaml`, `kiss_icp.yaml`, `rtabmap.yaml`, `orb_slam3.yaml`,
  `openvins_estimator.yaml`, `vins_fusion.yaml`, `uwb_anchors.yaml`);
- `robot_localization` EKF files (`ekf_odom.yaml`, and `ekf_map.yaml` + `navsat_transform.yaml` + `ntrip.yaml`
  for GNSS stacks);
- `launch/bringup.launch.py`, `package.xml`, `CMakeLists.txt` and a README with the apt / source packages,
  frames, topics and tuning notes.

Values follow the studio model of the vehicle; calibrate sensors and tune on the machine.

## API

`recommendStacks(request)`, `LOCALIZATION_METHODS`, `NAVIGATION_METHODS`, `setNavStack(robot, config)`,
`LocalizationEstimator`, `simulateLidar2D`, `SlamMap` in `src/mobile/navstack.ts`; `generateRosNavPackage` in
`src/mobile/navstack_ros.ts`. Tests: `tests/navstack.test.ts`.
