# VerticalBot Studio — TODO / roadmap

Living task list for the studio. Keep it honest: an item is *done* only when it is merged with tests and
documented in `docs/` (Read the Docs). Дорожная карта студии; статус «сделано» ставится только после
тестов и документации.

Legend: `[x]` done · `[~]` partial / best-effort · `[ ]` open

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
- [~] Deep `.rdk` container parsing without RoboDK — needs real sample files (`.rdk`, `.robot`) to go beyond names/poses/meshes
- [ ] RoboDK plug-in interface (C++ `IAppRoboDK`) — not planned; the JS plugin API covers the same hooks
- [ ] More posts on request (Yaskawa MotoPlus, Fanuc KAREL, Stäubli VAL3 advanced, Siemens NX CAM handshake)
- [ ] Accuracy: robot calibration with real tracker drivers (Leica / API / FARO) — only simulated measurements today

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
- [ ] Field validation of VDA 5050 with a real broker / KUKA Fleet / MiR — needs access to a fleet
- [ ] Open-RMF adapter (fleet adapter API) and ROS 2 Nav2 action bridge for real AMRs
- [ ] Multi-map / elevator / door handling in VDA orders (zones, `zoneSetId`)
- [ ] Battery/energy models per terrain slope and load (agriculture)

## 4. Agriculture (сельское хозяйство)

- [x] Crop presets, orchard/greenhouse generators, GeoJSON import, missions (harvest, spray, mow, prune, scout, pollinate, weed), vision detection sim
- [ ] Terrain elevation (DEM import, slopes in planning and energy)
- [ ] Seasonal/phenology model for yield forecasting; weather windows in mission scheduling
- [ ] Multi-arm harvesting cycle optimisation (fruit assignment between arms, reach clustering)
- [ ] Real perception pipeline hooks (ROS 2 topics with detections → targets) validated on hardware

## 5. Interoperability (обмен данными)

- [x] URDF/xacro import (+ online package meshes), URDF package export, STL/OBJ/COLLADA/glTF/STEP import
- [x] Blender: animated glTF export, `.vbstation` add-on (import with FK + animation, export meshes/pose logs)
- [ ] Test the Blender add-on inside real Blender 3.6 / 4.x (only the bpy-free core is CI-tested)
- [ ] COLLADA / glTF assets inside `.vbstation` decoded by the add-on (today: STL/OBJ)
- [ ] Import glTF animations from Blender onto robots (retargeting joint curves)
- [ ] USD / OpenUSD export for Omniverse / Isaac Sim

## 6. Platform & quality (платформа и качество)

- [x] Vite + TypeScript + three.js app, vitest (83 tests), Playwright smoke, GitHub Actions
- [x] Read the Docs documentation (`docs/`, Sphinx + MyST) — this roadmap is published there
- [ ] Russian translation of the documentation (sphinx-intl) — UI is already RU/EN
- [ ] Undo/redo coverage audit for every dialog; keyboard-only workflow
- [ ] Performance: instanced rendering for orchards > 10k trees, worker-thread simulation
- [ ] Authentication and multi-user sessions for the server (today: trusted network)
- [ ] Packaging: Docker image for server + static app; Electron/PWA offline bundle
