# VerticalBot Studio documentation

**VerticalBot Studio** is a browser-native platform for robot simulation and offline programming — a
RoboDK / Visual Components class tool that also covers mobile robots, fleets and agricultural robotics
(orchards, greenhouses, field work). It runs in any modern browser, needs no installation, and speaks the
RoboDK API, VDA 5050, ROS 2, glTF/URDF and the native formats of the major robot controllers.

This documentation is organised like a robot-programming manual: start with the **Basic Guide**, then go to the
topic you need. Every page is generated from the repository (`docs/` folder) and published on Read the Docs.

```{admonition} Quick links
:class: tip
- New here? Read {doc}`getting-started/quick-start` (10 minutes) and build {doc}`getting-started/tutorial`.
- Coming from RoboDK? See {doc}`reference/robodk-guide-map`, {doc}`interop/robodk` and the {doc}`reference/coverage`.
- Driving real machines? {doc}`api/server`, {doc}`api/drivers`, {doc}`mobile/vda5050`.
```

```{toctree}
:maxdepth: 2
:caption: Basic Guide

getting-started/installation
getting-started/quick-start
getting-started/tutorial
getting-started/interface
basic-guide/station
basic-guide/robots
basic-guide/frames-targets
basic-guide/programs
basic-guide/simulation
basic-guide/export
```

```{toctree}
:maxdepth: 2
:caption: Robots and mechanisms

robots/library
robots/online-library
robots/import
robots/mechanisms
robots/external-axes
```

```{toctree}
:maxdepth: 2
:caption: Programming

programming/post-processors
programming/robodk-posts
programming/machining
programming/curve-follow
programming/collisions
programming/calibration
programming/cameras-spray
programming/vision-stack
```

```{toctree}
:maxdepth: 2
:caption: Mobile robots, fleets, agriculture

mobile/mobile-robots
mobile/fleet
mobile/navigation-slam
mobile/vda5050
mobile/agriculture
process/components
```

```{toctree}
:maxdepth: 2
:caption: Interoperability

interop/robodk
interop/blender-ros-vc
interop/file-formats
```

```{toctree}
:maxdepth: 2
:caption: API and automation

api/robolink
api/console
api/server
api/drivers
api/plugins
```

```{toctree}
:maxdepth: 2
:caption: Reference

reference/coverage
reference/robodk-guide-map
reference/shortcuts
reference/faq
developer/architecture
developer/building
developer/contributing
roadmap
```
