# Demo scenarios and test results

The studio ships a suite of reproducible demo scenarios: one per **localization / navigation method** (22 cases
over warehouse, factory, greenhouse, orchard, vineyard, forest and open-field scenes) and one per **machine-vision
task family** (stereo picking, mono size-prior, segmentation / keypoints / classification, RGB-D bin picking,
LiDAR rows, ToF canopy measurement, following, VLM, VLA, open vocabulary) and one per **control-design method**
of the course (supervisor synthesis, Petri-net deadlock prevention, model checking, GR(1) synthesis, the mission
runtime on a station robot, planning, decisions under uncertainty, coordination, scheduling, real-time analysis,
safety, V&V, hybrid modes). Each scenario builds a small station, runs the simulation or the analysis headlessly,
measures what matters for that method and compares it with an expectation derived from the catalogue (sensor
error model, method accuracy and drift) or from the worked examples of the course (the numbers of examples A and B).

```{image} ../_static/screens/38-scenarios-dialog.png
:alt: Demo scenarios dialog
:class: screenshot
:width: 760px
```

- **In the studio** — *Help › Demo scenarios (navigation, vision, control design)…*: *Load into the studio* builds
  the scene, selects the robot / camera / model and opens the Navigation, Vision or Control tab (navigation
  scenarios start the world simulation; the control scenario *Mission runtime* adds the robot and the zones so
  that *Run mission* works immediately); *Run headless* / *Run all* execute the same measurements the CI runs and
  show pass / fail with the key metrics; *Download report* saves the Markdown below.
- **From the command line** — `npm run scenarios` in `studio/` (writes `studio/docs/scenario-results.md`, the
  table below) and `npm test` (asserts every scenario passes).
- **In code** — `import { ALL_SCENARIOS } from 'src/scenarios'`; `scenario.build()` returns the station,
  `scenario.run()` the metrics. Add your own scene / method the same way and it is picked up by the tests, the
  report and the dialog.

## How the scenarios are judged

| Group | Measured | Pass criterion |
|---|---|---|
| Navigation | localization RMSE and max error against the true pose, max / mean deviation of the true path from the planned path, lost-tracking events and time, loop closures / fixes, GNSS fix time, monocular scale, SLAM map coverage and agreement | RMSE ≤ 3σ of the method accuracy + accumulated drift over the distance without a global fix (halved for methods with loop closure), minimum 150 mm; global methods must also stay within 1.5 m of the path. Relative methods (VIO, LiDAR odometry, dead reckoning) are judged on RMSE only — their path deviation is reported to show why they need a global reference. |
| Control design | the quantities of the course examples: plant / supervisor state counts, realisability, deadlock markings and the monitor that removes them, model-checking verdicts and counterexamples, GR(1) game size and realisability, mission outcome on the station robot (reported, denials, violations, STL robustness), plan length / cost, MDP values and switch points, assignment costs, MAPF success, makespans and bounds, response times and latencies, MTTF / S_p / PL / cut-set order, falsification vs random search, acceptance bounds | equality with the course numbers where the course states them (648 / 324 states, 32 s cycle time, 12.943 s, 81, 54 vs 44, 1.54 m, PL d …), otherwise the qualitative verdict of the course (unrealisable, deadlock, starvation, controllable) |
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
