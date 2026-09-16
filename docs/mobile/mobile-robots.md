# Mobile robots

```{image} ../_static/screens/27-mobile-robot-dialog.png
:alt: Add mobile robot dialog
:class: screenshot
:width: 620px
```

*Mobile & Fleet › Add mobile robot…* creates an AMR / AGV / tractor / tracked platform with:

- **Drive model** — differential, Ackermann (steering angle, min turning radius), omnidirectional, tracked,
  legged; footprint, speed/acceleration/yaw limits.
- **Battery** — capacity, idle and drive power, charge power, low threshold.
- **Sensors and capabilities** — tags used by the fleet allocator (e.g. `harvest`, `spray`, `lift`).
- **Payload** and a home position.

Arms, cameras and tools can be children of a mobile robot (a harvesting arm on a tracked platform).

## Maps and zones

```{image} ../_static/screens/28c-map-dialog.png
:alt: Occupancy map dialog
:class: screenshot
:width: 560px
```
```{image} ../_static/screens/28b-zone-dialog.png
:alt: Zone dialog
:class: screenshot
:width: 560px
```

*Mobile & Fleet › Occupancy map…* creates a grid map (resolution, inflation) built from the station geometry;
*Zone (charging / no-go)…* defines work, no-go, charging, loading/unloading, parking, speed-limit and headland
areas. Fields generate their maps automatically ({doc}`agriculture`).

## Navigation

The localization and navigation software of each robot is selected and simulated per {doc}`navigation-slam`.


- **Planning** — A* on the inflated map, coverage planning, orchard row traversal with headland turns.
- **Control** — pure pursuit with speed limits per zone; velocity commands integrated per drive model.
- **Program instructions** — `Navigate to`, `Follow path`, `Mission task` in robot programs; the API offers
  `AddMobileRobot` and navigation helpers.

## Simulation

Mobile robots move on the world clock (*Start world simulation*). Their state (pose, speed, battery, status,
current task) is visible in the properties panel and the **Fleet** tab; real robots can shadow them through
ROS 2 (rosbridge) or VDA 5050 ({doc}`vda5050`).
