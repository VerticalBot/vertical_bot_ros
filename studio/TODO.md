# VerticalBot Studio — TODO / roadmap

Living task list for the studio. Keep it honest: an item is *done* only when it is merged with tests and
documented in `docs/` (Read the Docs). Дорожная карта студии; статус «сделано» ставится только после
тестов и документации.

Legend: `[x]` done · `[~]` partial / best-effort · `[ ]` open

## Status (September 2026)

| | |
|---|---|
| Code | ~21 k lines TypeScript in `src/` + server, Python `robodk` drop-in, Blender add-on |
| Verification | 157 vitest tests (7 network tests skipped offline), Playwright smoke, 32 demo scenarios (`npm run scenarios`), GitHub Actions (typecheck, tests, builds, Sphinx `-W`) |
| Docs | ~50 Read the Docs pages with screenshots generated from the running app, RoboDK guide map, integration page, scenario results |
| Done this cycle | machine-vision stack (catalogue, recommender, pluggable YOLO / VLM / VLA adapters, point clouds, Vision tab, ROS 2 perception export), navigation & SLAM stack, demo scenarios for every method, rosbridge publishers for navigation and perception, API params / webhooks, protocol tests |

### Next up (приоритеты)

1. Hardware validation loop: real camera + arm through the exported perception package (ROS 2 detections → targets), real AMR through the exported Nav2 package; record the first field logs and calibrate the simulated noise models from them.
2. Photorealistic camera simulation so real detectors can run on simulated images (textures, lighting, leaves, motion blur) and depth from the WebGL depth buffer.
3. Open-RMF / Nav2 action bridge for real AMRs; VDA 5050 with a real fleet manager.
4. Packaging (Docker image server + static app, PWA offline bundle) and server authentication.
5. Russian documentation (sphinx-intl).

## 1. RoboDK parity (ядро, паритет с RoboDK)

- [x] Station tree, frames, targets, tools, programs, folders, notes; RoboDK item-type numbering
- [x] Kinematics: DH / modified DH / URDF chains, FK, Jacobian, DLS IK with configuration awareness, <6-DOF relaxation, external axes (combined IK)
- [x] Programs: MoveJ/L/C, speed, rounding, frames/tools, pauses, IO, events, code, calls, threads, wait, mobile moves, joint-path replay
- [x] Simulator: trapezoidal profiles, timeline, attachments, collision sampling, singularity diagnostics, KPIs
- [x] Post processors: 28 built-in + original RoboDK Python posts in the browser (Pyodide) — `Program › Import RoboDK post processors`
- [x] Program importers: KRL, RAPID, LS, URScript, CSV, G-code/NC
- [x] Robot library: 22 built-in + 91-robot online library (ROS-Industrial / vendor URDF packages)
- [x] `.rdk/.robot/.tool`: lossless import through a headless RoboDK behind the server (`rdk2vbs.py`), best-effort binary scan otherwise
- [x] Machining: curve/point follow, NC → milling/cutting/3D-printing programs (feeds, spindle, extruder), RoboDK machining projects transfer
- [x] Calibration: TCP, frames, robot DH identification, ISO 9283, ballbar; cameras; spray deposition; mechanism builder
- [x] API: Robolink/Item (Python drop-in package, C#, C++, MATLAB, browser console), WS/TCP server, drivers UR/ABB RWS/KUKA KVP/ROS 2
- [x] Tutorial station and guide, RoboDK documentation map (feature-by-feature coverage)
- [~] Deep `.rdk` container parsing without RoboDK — needs real sample files (`.rdk`, `.robot`) to go beyond names/poses/meshes
- [ ] RoboDK plug-in interface (C++ `IAppRoboDK`) — not planned; the JS plugin API covers the same hooks
- [ ] More posts on request (Yaskawa MotoPlus, Fanuc KAREL, Stäubli VAL3 advanced, Siemens NX CAM handshake)
- [ ] Accuracy: robot calibration with real tracker drivers (Leica / API / FARO) — only simulated measurements today
- [ ] Mesh/mesh collision everywhere (BVH), self-collision matrices per robot from URDF `<disable_collisions>`

## 2. Visual Components / KUKA.Sim (процессное моделирование)

- [x] Process components (feeder, conveyor, process, buffer, sink), signals, threads, world clock, KPIs
- [~] `.vcmx` / `.vcm` import — meshes + XML metadata; behaviours and `.rsc` geometry are proprietary (need sample files to improve)
- [ ] Component behaviour scripting parity (VC Python API subset) — design a mapping onto the studio plugin API
- [ ] Layout-level export towards VC/KUKA.Sim beyond glTF/URDF (e.g. a `.vcm`-compatible layout skeleton once the schema is confirmed)
- [ ] KUKA.Sim / OfficeLite handshake: round-trip KRL with `$CONFIG`/`.dat` variables verified on a real controller

## 3. Mobile robotics & fleets (мобильные роботы и флот)

- [x] Drive models (diff, ackermann, omni, tracked), A* / coverage / row traversal, pure pursuit, occupancy maps, zones
- [x] Fleet manager: auction allocation, alley reservations, deadlock-free braking, charging, KPIs incl. yield
- [x] VDA 5050 v2 (master + AGV-twin bridge over MQTT), tested against an embedded broker
- [x] Navigation & SLAM stack: catalogue of 21 localization + 7 navigation methods, recommender (platform / environment / sensors / constraints), localization error simulation driving the controller (drift, GNSS outages under canopy, loop closures, scale drift, tracking loss), 2D LiDAR + SLAM map view, Nav2 + SLAM + EKF ROS 2 package export
- [x] Demo scenario per localization method (warehouse loop, greenhouse rail, orchard alleys with GNSS-denied canopy, open-field passes) with RMSE / deviation / lost-event metrics — `npm run scenarios`
- [x] ROS 2 publishing of the simulated stack: `odom_estimate`, `ground_truth`, `scan`, `slam_map`; `navEstimate` through the API
- [ ] Navigation stack: 3D LiDAR ray-casting against scene meshes for SLAM (today: analytic primitives), multi-robot map sharing, Nav2 behaviour trees export, sensor-noise calibration from real logs
- [ ] Field validation of VDA 5050 with a real broker / KUKA Fleet / MiR — needs access to a fleet
- [ ] Open-RMF adapter (fleet adapter API) and ROS 2 Nav2 action bridge for real AMRs
- [ ] Multi-map / elevator / door handling in VDA orders (zones, `zoneSetId`)
- [ ] Battery/energy models per terrain slope and load (agriculture)

## 4. Machine vision — СТЗ (техническое зрение)

- [x] Catalogue: 15 sensors with depth error models, 10 compute targets, 45 models (detection, segmentation, classification, tracking, keypoints, 6D pose, depth, point-cloud nets, grasping, VLM, VLA) with latency per compute class, licences and sources
- [x] Recommender: tasks + environment + modality + compute + working distance + constraints → ranked sensor / compute / model stacks with depth error budget, pipeline latency, warnings
- [x] Pipeline on station cameras: ground-truth capture (fruit, trunks, objects, vehicles, occlusion), simulated detector statistics, ByteTrack, 3D localisation (depth median, mono size prior, LiDAR clusters), grasp / approach targets, follow controller, VLM queries, VLA actions applied to the TCP
- [x] Pluggable real models: ONNX Runtime Web (Ultralytics exports: detect / seg / pose / cls, RT-DETR, YOLO-World), studio server `/vision/infer` (ultralytics / onnxruntime / forwarding), OpenAI-compatible VLM endpoints, VLA policy servers (openpi, OpenVLA, studio JSON), ROS 2 `vision_msgs` via rosbridge
- [x] Point clouds: PCD / PLY import & export, voxel, RANSAC ground, Euclidean clustering with shape classes, trunk slice + row lines, canopy metrics, simulated 3D LiDAR / depth camera
- [x] Vision tab (overlay, bird's-eye cloud, precision / recall / position error vs truth), wizard, camera properties, ROS 2 perception package export (drivers, yolo_ros, hand-eye TF, cloud pipeline, VLM / VLA bridges)
- [x] Demo scenario per task family (stereo picking, mono size prior, segmentation / keypoints / classification, RGB-D bin picking, LiDAR rows, ToF canopy, following, VLM, VLA, open vocabulary); protocol tests with mock inference / VLM / VLA servers and a real onnxruntime-web run
- [x] Publishing: `vision_msgs` detections / 3D detections / targets / `PointCloud2` / `CompressedImage` over rosbridge, `visionLast` through the API, webhooks
- [ ] Photorealistic rendering for real detectors on simulated images (textures, lighting, leaves, motion blur); depth from the WebGL depth buffer instead of ground truth
- [ ] Real perception validated on hardware (ROS 2 detections → targets with a physical camera and arm); noise models calibrated from field data
- [ ] Hand-eye calibration workflow in the UI (collect TCP / camera correspondences, `rigidTransform`, write the static TF)
- [ ] Full 6D orientation from depth (surface normals / PCA of the mask cloud) and in-browser FoundationPose-class models when WebGPU allows
- [ ] Dataset export: rendered images + ground-truth labels (YOLO / COCO) for training and auto-labelling with VLMs

## 5. Agriculture (сельское хозяйство)

- [x] Crop presets, orchard/greenhouse generators, GeoJSON import, missions (harvest, spray, mow, prune, scout, pollinate, weed), fruit detection sim, canopy GNSS-denied zones
- [ ] Terrain elevation (DEM import, slopes in planning and energy)
- [ ] Seasonal/phenology model for yield forecasting; weather windows in mission scheduling
- [ ] Multi-arm harvesting cycle optimisation (fruit assignment between arms, reach clustering)
- [ ] Spray dose maps from canopy volume (vision) → variable-rate spraying missions

## 6. Interoperability (обмен данными)

- [x] URDF/xacro import (+ online package meshes), URDF package export, STL/OBJ/COLLADA/glTF/STEP import
- [x] Blender: animated glTF export, `.vbstation` add-on (import with FK + animation, export meshes/pose logs)
- [x] Integration page: ports, protocols, ROS 2 topics, JSON contracts (inference, VLM, VLA, webhook, API), example nodes (`python/examples/`)
- [ ] Test the Blender add-on inside real Blender 3.6 / 4.x (only the bpy-free core is CI-tested)
- [ ] COLLADA / glTF assets inside `.vbstation` decoded by the add-on (today: STL/OBJ)
- [ ] Import glTF animations from Blender onto robots (retargeting joint curves)
- [ ] USD / OpenUSD export for Omniverse / Isaac Sim

## 7. Platform & quality (платформа и качество)

- [x] Vite + TypeScript + three.js app, vitest (157 tests), Playwright smoke, GitHub Actions
- [x] Read the Docs documentation (`docs/`, Sphinx + MyST) — this roadmap is published there
- [x] Screenshots generated from the running app (`studio/scripts/docs-screenshots.mjs`), tutorial station + guide, RoboDK documentation map
- [x] Demo scenarios suite with a generated Markdown report (`npm run scenarios`, Help › Demo scenarios…)
- [ ] Russian translation of the documentation (sphinx-intl) — UI is already RU/EN
- [ ] Undo/redo coverage audit for every dialog; keyboard-only workflow
- [ ] Performance: instanced rendering for orchards > 10k trees, worker-thread simulation, faster scenario dialog load
- [ ] Authentication and multi-user sessions for the server (today: trusted network)
- [ ] Packaging: Docker image for server + static app; Electron/PWA offline bundle
