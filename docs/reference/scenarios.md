# Demo scenarios and test results

The studio ships a suite of reproducible demo scenarios: one per **localization / navigation method** (22 cases
over warehouse, factory, greenhouse, orchard, vineyard, forest and open-field scenes) and one per **machine-vision
task family** (stereo picking, mono size-prior, segmentation / keypoints / classification, RGB-D bin picking,
LiDAR rows, ToF canopy measurement, following, VLM, VLA, open vocabulary). Each scenario builds a small station,
runs the simulation headlessly, measures what matters for that method and compares it with an expectation derived
from the catalogue (sensor error model, method accuracy and drift).

```{image} ../_static/screens/38-scenarios-dialog.png
:alt: Demo scenarios dialog
:class: screenshot
:width: 760px
```

- **In the studio** — *Help › Demo scenarios (navigation & vision)…*: *Load into the studio* builds the scene,
  selects the robot / camera and opens the Navigation or Vision tab (navigation scenarios start the world
  simulation); *Run headless* / *Run all* execute the same measurements the CI runs and show pass / fail with the
  key metrics; *Download report* saves the Markdown below.
- **From the command line** — `npm run scenarios` in `studio/` (writes `studio/docs/scenario-results.md`, the
  table below) and `npm test` (asserts every scenario passes).
- **In code** — `import { ALL_SCENARIOS } from 'src/scenarios'`; `scenario.build()` returns the station,
  `scenario.run()` the metrics. Add your own scene / method the same way and it is picked up by the tests, the
  report and the dialog.

## How the scenarios are judged

| Group | Measured | Pass criterion |
|---|---|---|
| Navigation | localization RMSE and max error against the true pose, max / mean deviation of the true path from the planned path, lost-tracking events and time, loop closures / fixes, GNSS fix time, monocular scale, SLAM map coverage and agreement | RMSE ≤ 3σ of the method accuracy + accumulated drift over the distance without a global fix (halved for methods with loop closure), minimum 150 mm; global methods must also stay within 1.5 m of the path. Relative methods (VIO, LiDAR odometry, dead reckoning) are judged on RMSE only — their path deviation is reported to show why they need a global reference. |
| Vision | precision / recall against the ground truth (objects ≥ 10 px), mean 3D position error, tracked ids, targets created, range error of the mono size prior, mask / keypoint counts and keypoint error, ripeness accuracy, LiDAR row-line lateral error against the true row, canopy height, follow range error, VLM count accuracy, VLA convergence (grasp signalled, final distance), open-vocabulary precision | bounds per scenario (e.g. position error ≤ 4σ + 30 mm of the sensor's depth model at the working distance; row error < 400 mm; follow range error < 1.2 m; VLA reaches < 200 mm) |

The simulated adapters model *statistics* (recall vs. object size, false positives, box jitter, attribute
confusion, drift and outages), not appearance or physics; the numbers are therefore an honest first-order
comparison between stacks, not a certification.

## Results

```{include} ../../studio/docs/scenario-results.md
```

## Reproducing a case by hand

Every entry in the report carries a *How to reproduce* line (menu path, wizard settings). The general recipe:

1. **Navigation** — *Mobile & Fleet › Add mobile robot*, choose the platform; *Navigation & SLAM stack…*, set the
   environment and sensors, pick the method the scenario names, enable *Simulate localization*; start the world
   simulation and open the **Navigation** tab (truth in green, estimate in orange, LiDAR / SLAM map, GNSS-denied
   zones). *Export ROS 2 package* gives the matching Nav2 + SLAM configuration for the real vehicle.
2. **Vision** — *Add › Camera / vision sensor* on the item that carries it; *Tools › Machine vision stack…*, tick
   the tasks, set modality / compute / working distance / classes, choose the stack, set the runtime; **Vision**
   tab › *Run* or *live*; *Targets → station* for picking; *follow tracked target* for following; the prompt field
   for VLM questions and VLA instructions. *Export ROS 2 package* gives the perception launch for the real camera.
3. **Connecting the result to your software** — {doc}`../developer/integration` (ROS 2 topics, API params, webhook,
   inference / VLM / VLA contracts, ports).
