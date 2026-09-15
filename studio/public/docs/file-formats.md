# File formats

| Extension | Direction | Description |
|---|---|---|
| `.vbstation` / `.json` | open/save | Native station: serialized item tree + base64 mesh assets (`src/io/station-file.ts`) |
| `.urdf`, `.xacro` | import | Robots via built-in URDF/xacro processor (properties, macros, includes, if/unless, math). Meshes referenced by `package://` are resolved by file name from dropped STL files |
| `.stl` | import | Binary/ASCII meshes (mm). Also used as robot link meshes |
| `.rdk` | import (best effort) | RoboDK station container: names, poses, embedded meshes recovered heuristically |
| `.src` / `.dat` | import/export | KUKA KRL |
| `.mod` / `.prg` | import/export | ABB RAPID |
| `.ls` | import/export | Fanuc TP (ASCII) |
| `.script` | import/export | Universal Robots URScript |
| `.JBI` | export | Yaskawa INFORM |
| `.pgx` | export | Stäubli VAL3 |
| `.drl` | export | Doosan DRL |
| `.py` | export | Mecademic Python, RoboDK API script (program or whole station), ROS 2 rclpy node |
| `.csv` / `.txt` | import/export | Targets (X,Y,Z,Rx,Ry,Rz[,joints]) in RoboDK / KUKA / Fanuc conventions; generic program CSV |
| `.json` | export | Program JSON (poses, quaternions, joints) and ROS 2 JointTrajectory |
| `.geojson` | import/export | Field boundaries (WGS84) → local mm polygons around a computed origin |
| `.png` | export | Viewport screenshot |
