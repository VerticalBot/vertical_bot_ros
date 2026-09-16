# Importing robots and geometry

## URDF / xacro

Drop a `.urdf` or `.xacro` together with its meshes (STL, DAE, OBJ) onto the 3D view. The built-in xacro
processor supports properties, macros with parameters and blocks, `xacro:if/unless`, includes, math and
`$(find …)`. Branching robots are flattened to the longest chain; side branches (gripper fingers, cameras)
become static visuals; `mimic` joints are honoured. Meshes referenced by `package://` are matched by file name
to the dropped files or downloaded from the online library repositories.

## DH tables

A `.dh` text file or a JSON `{"dh": [[theta, d, a, alpha, lower, upper, prismatic?, home?], …]}` builds a
robot from standard DH parameters (RoboDK "Robot parameters" dialog style). *Parameters…* in the properties
panel edits the table of an existing robot; modified DH is available through `setRobotParams`.

## RoboDK `.robot` / `.tool` / `.rdk`

RoboDK item files are proprietary. Dropped files are sent to the studio server's converter when a RoboDK
installation is available there (`STUDIO_ROBODK_PYTHON`); otherwise a best-effort scan recovers names, poses and
embedded meshes. See {doc}`../interop/robodk`.

## Geometry

| Format | Notes |
|---|---|
| STL (binary/ASCII), OBJ | Assumed millimetres |
| COLLADA `.dae` | Unit and up-axis honoured |
| glTF / GLB | All meshes flattened, metres → mm, Y-up → Z-up |
| STEP / IGES / BREP | OpenCascade WebAssembly, loaded on demand |
| `.zip`, `.vcmx`, `.vcm` | Archives (Visual Components / KUKA.Sim components): meshes and metadata |

Dropped meshes become objects under the station (or the active frame); drag them onto a tool to make them tool
geometry, or onto a robot link through the API (`AddShape` on a link item).

## Programs and paths

Controller programs (KRL, RAPID, LS, URScript, CSV) import as programs with targets; NC / G-code files import as
machining paths ({doc}`../programming/machining`); GeoJSON imports fields for agriculture.
