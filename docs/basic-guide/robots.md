# Robots

## Adding a robot

- **Built-in library** (*Add › Robot from library…*): 22 models grouped by category; UR e-Series use the
  official DH parameters, other brands use representative geometry unless marked otherwise.
- **Online library** (*Add › Robot from online library (ROS-Industrial)…*): 91 industrial and collaborative
  robots downloaded from the open URDF packages with vendor meshes — see {doc}`../robots/online-library`.
- **URDF / xacro** drop (with STL/DAE meshes), **DH table** (`.dh` / JSON), **mechanism builder** (API
  `BuildMechanism`), or a **RoboDK `.robot`** through the server converter ({doc}`../robots/import`).

## Moving the robot

- **Joints** — sliders and value boxes in the properties panel; *Home* returns to the home configuration.
- **Cartesian jog** — ±X/Y/Z and ±Rx/Ry/Rz in the tool frame with configurable step.
- **Gizmo on the tool** — drag the TCP gizmo in the 3D view; IK follows.
- **Double-click a target** in the tree or program to move there (MoveJ or MoveL depending on the instruction).

Inverse kinematics keeps the current configuration (elbow up/down, wrist flip, like RoboDK's configuration
flags) and reports when a target is out of reach or crosses a singularity. Robots with fewer than 6 axes get
automatic orientation relaxation (palletizers, SCARA, gantries).

## Tools

A robot can hold several tools; the active one is used for teaching and IK. The tool item stores the TCP pose
relative to the flange (properties › *Tool*), geometry (drop an STL onto the tool) and a colour. TCP calibration
by points or lines is available through the API ({doc}`../programming/calibration`).

## Reference frames

The robot's active reference frame is selected in properties › *Frames*. Targets are stored relative to their
parent frame, so moving a fixture frame moves the whole program with it.

## Robot parameters

Properties › *Kinematics* shows the joint limits, home position, maximum speeds, brand, model and the post
processor used by *Export with post processor*. *Parameters…* opens the RoboDK-style robot parameters dialog
(DH/modified-DH table, base and tool offsets); *Set active* makes the robot the target of the Program menu.

## Reach and status

*View › Show robot reach* draws the reach sphere; the *Status* block shows TCP position in the active frame,
active frame/tool, reach and post processor.

## External axes

Drop a robot onto a rail, gantry or turntable robot in the tree: it is mounted on the carrier's flange and
*Robot › Move with external axes* solves carrier + arm together ({doc}`../robots/external-axes`).
