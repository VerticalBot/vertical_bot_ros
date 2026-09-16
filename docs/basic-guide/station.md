# Stations and items

A **station** is the document: a tree of items with poses, plus the meshes it uses. It is saved as one
`.vbstation` JSON file (meshes embedded as base64) so a project is a single portable file.

## Item types

| Item | Purpose | Created with |
|---|---|---|
| Robot | Serial mechanism (arm, gantry, positioner, rail) with kinematics, joints, tools, active frame/tool | Add › Robot from library / online library, URDF import, DH import, mechanism builder |
| Tool | Geometry on the flange with a TCP pose; a robot can have several tools and one active | Robot › Add tool; children of a robot |
| Reference frame | Coordinate system for targets and objects (fixtures, tables, parts, cameras) | Add › Reference frame |
| Target | Cartesian (pose relative to a frame + joint hint) or joint target | Program › Teach, Add › Target, API |
| Program | Instruction list bound to a robot | Add › Program, teach |
| Object | Geometry: primitives, STL/OBJ/COLLADA/glTF/STEP meshes, curves and points (for machining) | Add › Box/Cylinder/Sphere, drop files |
| Folder | Grouping | Add › Folder |
| Mobile robot, Map, Zone, Fleet, Field, Crop row, Mission, Path, Component, Camera, Machining project, Notes | See the corresponding chapters | menus / API |

Item numbering follows RoboDK's `ITEM_TYPE_*` constants so API scripts behave the same; studio-specific
types start at 100.

## Poses

Every item has a pose relative to its parent. The properties panel edits it as position + orientation in the
convention you select; the API works with 4×4 matrices (`robomath.Mat` in Python). Absolute poses combine the
chain of parents; for children of a robot the parent frame is the robot **flange**, so a tool, a camera or a
part attached to the robot follows its motion.

## Selection, undo, clipboard

Click in the tree or the 3D view (Ctrl-click for multi-selection). Every change is undoable (`Ctrl+Z` /
`Ctrl+Y`), including dialog results and API commands executed from the console. Items can be copied and pasted
through the API (`Copy`/`Paste`) and cloned with the context menu.

## Multiple stations

*Tools › New station tab* opens another station in the same browser tab; the API's `setActiveStation` and
`getOpenStations` switch between them.

## Saving and sharing

- *File › Save station (.vbstation)* — complete project, reopen anywhere.
- *File › Export station JSON* — the same JSON without downloading meshes twice, for the headless server or
  `rdk_export.py` workflows.
- *Tools › Export 3D HTML* — a self-contained viewer page (glTF embedded) to share a scene by e-mail.
- *File › Export screenshot (PNG)* and *Tools › Record video (WebM)* for presentations.
