# Your first station (tutorial)

This tutorial builds a complete pick-and-place cell from an empty station: a UR5e on a pedestal, a table with a
part and a bin, a gripper, taught targets, a program, simulation with validation, and generated URScript.
It takes about 15 minutes. The finished station is also available as *File › Demo stations › Tutorial — first
station (UR5e)* and as a file: {download}`first_station.vbstation <../_static/examples/first_station.vbstation>`.

All screenshots on this page are taken from the application by the script `studio/scripts/docs-screenshots.mjs`,
so they match the current build.

## 1. Start with an empty station

*File › New station* (`Ctrl+N`). The tree on the left holds only the station; the 3D view shows the grid and the
world frame; the properties panel is empty until something is selected.

```{image} ../_static/screens/01-empty-station.png
:alt: Empty station
:class: screenshot
```


## 2. Add the robot

*Add › Robot from library…*, choose **Universal Robots UR5e** and press *OK*.

```{image} ../_static/screens/02-add-menu.png
:alt: Add menu
:class: screenshot
```

```{image} ../_static/screens/03-robot-library-dialog.png
:alt: Robot library dialog
:class: screenshot
:width: 620px
```


The robot appears at the origin with a default `Tool 1` on its flange and becomes the *active robot* (the one the
Program and Robot menus act on). Select it and set its pose in the properties panel: Z = 500 mm to stand it on a
pedestal we will add next.

```{image} ../_static/screens/04-robot-added.png
:alt: UR5e added
:class: screenshot
```


```{tip}
For the vendor's own meshes and exact URDF kinematics use *Add › Robot from online library (ROS-Industrial)…*
instead — it downloads the description package from GitHub.
```

```{image} ../_static/screens/05-online-library-dialog.png
:alt: Online library dialog
:class: screenshot
:width: 620px
```


## 3. Add a reference frame for the table

*Add › Reference frame*, rename it **Table** (properties › name or double-click in the tree) and set its
position to X = 450, Y = −250, Z = 0 mm. Everything on the table — the part, the bin and the targets — will be
defined relative to this frame, so re-measuring the table in the real cell moves the whole program at once.

```{image} ../_static/screens/06-frame-properties.png
:alt: Frame properties
:class: screenshot
```


Select the robot, open its *Frames* section and choose **Table** as the active reference frame.

## 4. Add geometry

*Add › Cylinder* for the pedestal (radius 150, length 500, under the station) and *Add › Box* three times under
the **Table** frame: the table top (500 × 700 × 30 at 200, 300, 385), the **Part** (100 × 100 × 80 at 150, 200,
400) and the **Bin** (200 × 200 × 20 at 100, 500, 400). Drag items in the tree to make them children of the
frame; edit size, position and colour in the properties panel. Real parts are imported the same way by dropping
STL/STEP/glTF files on the 3D view.

```{image} ../_static/screens/07-objects-added.png
:alt: Objects on the table
:class: screenshot
```


## 5. Define the tool

Select `Tool 1` under the robot, rename it **Gripper** and set the TCP to Z = 150 mm (properties › *Tool*). The
TCP is the point the targets refer to; the tool kind (gripper / vacuum / welding …) selects the simulation event
used by *Grip* / *Release* instructions.

```{image} ../_static/screens/08-tool-properties.png
:alt: Tool properties
:class: screenshot
```


## 6. Teach the targets and the program

1. Jog the robot to a comfortable home position (joints `0, −90, 90, −90, −90, 0`) and press `J`: a target
   **Home** is created under the Table frame and a program with a *MoveJ Home* is started. Mark it as a *joint
   target* in the properties panel so it is always reached with these joints.
2. Move the TCP above the part (properties › Cartesian jog, or type the pose of a target: X 150, Y 200, Z 620,
   Rx 180) and press `J` → **Approach**.
3. Lower to the part (Z 482) and press `L` → **Pick** (linear move).
4. In the Program tab add *Grip*, *Set DO* or an *attach* event for the Part, and a 300 ms *Pause*.
5. `L` back to Approach, `J` above the bin (X 100, Y 500, Z 650), `L` down (Z 502) → **Place**, add
   *Release* / *detach*, pause, `L` up, `J` Home.

Targets can be renamed and edited at any time; double-click one to move the robot there.

```{image} ../_static/screens/09-targets-taught.png
:alt: Targets in the tree and the robot at Pick
:class: screenshot
```

```{image} ../_static/screens/10-program-editor.png
:alt: Program editor
:class: screenshot
```


## 7. Validate and simulate

*Program › Validate (compile)* checks reachability, joint limits, singularities and (optionally) collisions and
reports the cycle time; problems are listed per instruction.

```{image} ../_static/screens/11-validate.png
:alt: Validation result
:class: screenshot
```


*Program › Run* (`F5`) plays the program; the Simulation tab shows the timeline. Drag the slider to scrub, change
the speed factor, or step. The part follows the gripper between *attach* and *detach*.

```{image} ../_static/screens/12-simulation-timeline.png
:alt: Simulation timeline
:class: screenshot
```


## 8. Generate the robot program

*Program › Export with post processor…* shows the URScript generated by the robot's default post (Universal
Robots). Change the post in the drop-down to see the same program as KUKA KRL, ABB RAPID, Fanuc LS, ROS 2 …
Press *Download* to get the file(s) for the controller.

```{image} ../_static/screens/13-export-post-dialog.png
:alt: Post processor export dialog
:class: screenshot
:width: 760px
```


## 9. Save

*File › Save station (.vbstation)*. Reopen it with *File › Open / import…* or by dropping it onto the 3D view.

## Where to go next

- Check the cell for collisions and tune the collision map — {doc}`../programming/collisions`.
- Replace the primitive part with real geometry and program a machining or dispensing path from NC code —
  {doc}`../programming/machining`.
- Put the robot on a rail or a mobile platform — {doc}`../robots/external-axes`, {doc}`../mobile/mobile-robots`.
- Script the same steps with the RoboDK-compatible API — {doc}`../api/console`.
