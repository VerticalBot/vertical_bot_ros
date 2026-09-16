# RoboDK documentation map

The table follows the structure of the RoboDK documentation (Basic Guide, Interface, Robot Programs, Robot
Machining, Post Processors, API, Accuracy, Plugins, Export Simulation, Robot Library, Custom Robots, Drivers,
Stations) and points to the equivalent chapter here. *Verified* means the feature was exercised in the running
application while producing this documentation (the screenshots come from that run); *partial* names the gap.

| RoboDK topic | VerticalBot Studio | Status |
|---|---|---|
| Getting started › Select a robot (online library) | {doc}`../robots/library`, {doc}`../robots/online-library` | verified — 22 built-in + 91 online robots |
| Getting started › Add a reference frame | {doc}`../basic-guide/frames-targets` | verified |
| Getting started › Import 3D objects | {doc}`../robots/import` (STL/OBJ/DAE/glTF/STEP, primitives) | verified |
| Getting started › Create a tool (TCP) | {doc}`../basic-guide/robots` › Tools | verified |
| Getting started › Create targets | {doc}`../basic-guide/frames-targets` | verified (teach `J`/`L`, edit poses, joint/cartesian) |
| Getting started › Create a program | {doc}`../basic-guide/programs` | verified |
| Getting started › Simulate | {doc}`../basic-guide/simulation` | verified (timeline, speed, validation) |
| Getting started › Generate a robot program | {doc}`../basic-guide/export`, {doc}`../programming/post-processors` | verified (URScript, KRL, RAPID, LS …) |
| Getting started › Export simulation (3D HTML / PDF / video) | {doc}`../basic-guide/simulation` › Recording; *Tools › Export 3D HTML*, glTF, video | verified (HTML/glTF/WebM; PDF 3D not supported) |
| Interface › Menus, toolbar, station tree, 3D view, keyboard shortcuts | {doc}`../getting-started/interface`, {doc}`shortcuts` | verified |
| Interface › Multiple stations / tabs | {doc}`../basic-guide/station` | verified |
| Robot programs › Joint / linear / circular moves | {doc}`../basic-guide/programs` | verified |
| Robot programs › Set reference frame / tool frame | {doc}`../basic-guide/programs` | verified |
| Robot programs › Program call, show message, pause, set/wait IO, event, speed, rounding | {doc}`../basic-guide/programs` | verified |
| Robot programs › Simulation events (attach/detach/show/hide) | {doc}`../basic-guide/programs` | verified |
| Robot programs › Program flow (threads, signals, loops) | {doc}`../basic-guide/programs` | verified (threads/signals); loops/if via API |
| Robot programs › Import controller programs | {doc}`../robots/import` › Programs | verified (KRL, RAPID, LS, URScript, CSV) |
| Robot machining › Curve follow project | {doc}`../programming/curve-follow` | verified |
| Robot machining › Point follow project | {doc}`../programming/curve-follow` | verified |
| Robot machining › Robot machining from NC / APT | {doc}`../programming/machining` | verified (G-code); APT not parsed |
| Robot machining › 3D printing | {doc}`../programming/machining` | verified (E-axis extrusion, extruder IO) |
| Robot machining › Tool orientation / external axis optimisation | {doc}`../programming/curve-follow` | verified |
| Collision detection › Collision map, checks during simulation | {doc}`../programming/collisions` | verified |
| Accuracy › TCP calibration, reference calibration, robot calibration, ISO 9283, ballbar | {doc}`../programming/calibration` | implemented via API with simulated devices; no real tracker drivers |
| Post processors › Built-in and custom posts | {doc}`../programming/post-processors` | verified (28 built-in) |
| Post processors › RoboDK Python posts | {doc}`../programming/robodk-posts` | verified with the bundled sample post; vendor posts imported by the user |
| RoboDK API › Python, C#, C++, MATLAB | {doc}`../api/robolink`, {doc}`../api/server` | verified (Python end-to-end against the server) |
| RoboDK API › Run mode (simulate / run on robot / make program) | {doc}`../api/drivers` | verified in simulation; drivers need hardware |
| Plug-ins | {doc}`../api/plugins` | JavaScript plugin API (no C++ plug-ins) |
| Robot library › Online library, `.robot` files | {doc}`../robots/online-library`, {doc}`../interop/robodk` | verified (URDF packages); `.robot` via server converter |
| Custom robots › Robot builder (mechanisms) | {doc}`../robots/mechanisms` | verified via API; no drag-and-drop builder dialog |
| External axes › Rails, turntables, synchronised axes | {doc}`../robots/external-axes` | verified |
| Robot drivers › UR, ABB, KUKA, Fanuc, Motoman, ROS | {doc}`../api/drivers` | UR, ABB RWS, KUKA KVP, ROS 2 implemented; Fanuc/Motoman drivers not yet |
| Stations › Save / open `.rdk`, export scripts | {doc}`../interop/robodk`, {doc}`../basic-guide/station` | `.vbstation` native; `.rdk` through the converter |
| Cameras, 2D/3D simulated cameras | {doc}`../programming/cameras-spray` | verified |
| Spray / painting simulation | {doc}`../programming/cameras-spray` | verified via API |
| Conveyors and simulation events | {doc}`../process/components` | verified |
| — beyond RoboDK: mobile robots, fleets, VDA 5050, agriculture, Blender add-on | {doc}`../mobile/mobile-robots`, {doc}`../mobile/fleet`, {doc}`../mobile/vda5050`, {doc}`../mobile/agriculture`, {doc}`../interop/blender-ros-vc` | verified |

Open gaps are tracked in the {doc}`../roadmap`.
