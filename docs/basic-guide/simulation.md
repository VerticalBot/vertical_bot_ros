# Simulation

## Running programs

```{image} ../_static/screens/12-simulation-timeline.png
:alt: Simulation tab
:class: screenshot
```

*Program › Run* (`F5`), *Pause*, *Stop*. The **Simulation** tab has the timeline (drag to scrub), the current
time and total cycle time, the speed factor and step buttons. Motion uses trapezoidal velocity profiles with
the robot's speed and acceleration limits; linear moves are interpolated in Cartesian space with IK at every
sample and joint-speed limiting; circular moves follow the arc through the via point.

While running, attach/detach events move objects with the tool, digital outputs are recorded (visible in the
Log and available to process components), messages are shown and signals are raised.

## World simulation

Mobile robots, fleets, conveyors and process components live on a **world clock**: *Mobile & Fleet › Start
world simulation* / *Pause world* / *Reset world*. Robot programs can run at the same time — a harvesting arm on a
tracked platform, or an arm loading a conveyor, are simulated together.

## Collision checking

- *Robot › Check collisions now (static)* checks the current state.
- *Robot › Check collisions during program validation* samples every trajectory and reports the first
  colliding pair per instruction; colliding items are highlighted in red.
- *Tools › Collision map…* enables or disables pairs (RoboDK-style collision map), including robot self-collision.

Colliders are capsules and boxes for procedural links, exact primitives for boxes/cylinders/spheres and, for
meshes, bounding boxes with an optional triangle-accurate check ({doc}`../programming/collisions`).

## Measurements and cameras

*Tools › Measure* (`M`) measures between two selected items or from an item to the TCP. Cameras (2D/depth)
render from an item's pose into the **Camera** tab and through the API (`Cam2D_Snapshot`). Spray deposition
can be simulated for painting/spraying programs (`Spray_Add`).

## Recording

*Tools › Record video of the 3D view (WebM)*, *File › Export screenshot (PNG)*, *Tools › Export animation as
glTF (Blender)…* for a keyframed animation of the whole cell ({doc}`../interop/blender-ros-vc`).

## Performance notes

Large orchards (thousands of trees) render as instanced geometry; reduce *View › Show targets/frames* when a
station has many targets. The simulation itself runs in the browser's main thread — very long programs are
validated faster with *Validate* than by playing them.
