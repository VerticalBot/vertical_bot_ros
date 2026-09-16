# Quick start

This guide builds a small robot cell, teaches a program, simulates it and generates controller code — the
same first steps as in any offline-programming package.

## 1. Open the studio

```{image} ../_static/screens/04-robot-added.png
:alt: Studio with a robot
:class: screenshot
```

Open the studio in your browser. The **Help › Quick start** entry shows a short in-app version of this page.
The window is divided into the station tree (left), the 3D view (centre), the properties panel (right) and the
bottom tabs (Program, Simulation, Fleet, Process, Camera, Console, Log). See {doc}`interface`.

## 2. Add a robot

*Add › Robot from library…* lists the built-in robots (UR e-Series with official kinematics, KUKA, ABB, Fanuc,
Yaskawa, Stäubli, Doosan, Mecademic, palletizer, SCARA, gantry, harvesting arm). For a robot with the vendor
meshes and exact URDF kinematics use *Add › Robot from online library (ROS-Industrial)…* — the package is
downloaded straight from GitHub ({doc}`../robots/online-library`).

A tool (`Tool 1`, 100 mm along Z) is created on the flange automatically; edit its TCP in the properties panel.

## 3. Add a reference frame and targets

*Add › Reference frame* creates a frame under the station; move it with the gizmo (`T` translate, `R` rotate)
or type the pose in the properties panel (XYZ + Rx Ry Rz, RoboDK convention). Select the frame in the robot's
*Frames* section to make it the active reference.

Jog the robot (joint sliders or Cartesian jog buttons in the properties panel) and press `J` (*Program › Teach
MoveJ*) or `L` (*Teach MoveL*): a target is recorded under the active frame and a move instruction is appended to
the active program (created on demand).

## 4. Edit the program

The **Program** tab lists the instructions. Use the *Add:* buttons for speed, rounding, frame/tool changes,
pauses, digital outputs, wait-for-input, gripper events, code, comments, calls, navigation (mobile robots),
signals, threads and waits. Double-click a move to jump the robot there; drag rows to reorder.

## 5. Simulate

*Program › Run* (`F5`) plays the program with realistic trapezoidal motion; the **Simulation** tab has the
timeline, speed factor and step controls. *Program › Validate (compile)* reports cycle time, unreachable
targets, joint limits, singularities and — when *Robot › Check collisions during program validation* is on —
collisions along the trajectory ({doc}`../basic-guide/simulation`).

## 6. Generate the robot program

*Program › Export with post processor…* shows the code for the robot's post (KUKA KRL, ABB RAPID, Fanuc LS,
URScript, Motoman INFORM, Stäubli VAL3, Doosan DRL, ROS 2 …) and downloads the file(s). Original RoboDK Python
posts can be imported and used as well ({doc}`../programming/robodk-posts`).

## 7. Save

*File › Save station (.vbstation)* writes a single JSON file with all items and meshes. Reopen it with
*File › Open / import…* or by dropping it onto the 3D view. Demo stations are under *File › Demo stations*.

```{tip}
Everything above can be scripted: open the **Console (RoboDK API)** tab and type
`RDK.AddFrame("Frame 2")` — the same calls work from Python, C#, C++ and MATLAB through the studio server.
```
