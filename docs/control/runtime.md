# Mission runtime: behavior trees on a station robot under a supervisor

The analysis answers *is the design right?*; the mission runtime answers *does it work on the robot?*. A behavior
tree model is executed tick by tick inside the world simulation of the studio (the same clock that runs fleets and
process components): its actions drive a mobile robot of the station, every controllable event is asked from the
synthesised supervisor before the action starts, the mode automaton limits the speed, and the LTL₃ monitors watch
the blackboard. Denials, model mismatches and violations are counted, logged and shown in the status line.

```{image} ../_static/screens/55-mission-runtime.png
:alt: Mission runtime: the course mission tree drives the youBot between zones
:class: screenshot
:width: 900px
```

## Running a mission

1. The station needs a **mobile robot** (*Mobile & Fleet › Add mobile robot…*) and **zones** named as in the tree
   (`home`, `table`, `bin`, `dock` in the course example; *Mobile & Fleet › Zone…*). An item whose name contains
   *Human* (an object, a frame) is used as the human position for `dist_human` / `human` in the blackboard.
   The scenario *Mission runtime* (*Help › Demo scenarios…*, group *Control design*, **Load into the studio**)
   builds such a scene with the course models.
2. Select the `bt` model in the Control tab, choose the robot, set **targets** (objects to collect) and press
   **▶ Run mission**. The world clock starts (*Simulation* tab › World › Start / Pause also apply).
3. The status line shows the tree status, the simulated time and ticks, the **plant state** of the supervisor
   (one state per component, e.g. `mT,S,O,Sr,Tb` = base moving to the table, arm stowed, gripper open, vision
   searching, object on the table), the current **mode**, the numbers of **denied** events and **violations**, and
   the verdict of every monitor (`?` undecided, `⊤` satisfied for good, `⊥` violated). The last log lines are
   under it; the full log is in the *Log* tab.
4. The run stops when the tree reports (`action report`), when a tree without a report action succeeds, when a
   failure persists (or comes from a supervisor denial), at 600 s, or with **⏹ Stop**. A toast summarises the
   result; *Export ▾* offers the **trace (CSV)** — time and every numeric / boolean blackboard signal, ready for
   an `stl` document — and the **log**.

## How the tree is bound to the world

| Action (leaf binding) | Effect on the station robot |
|---|---|
| `goto zone=<name> [event=… arrive=…]` | plans a straight path to the zone centroid and follows it (`running` until arrival; `failure` when the zone is unknown); `arrive=` names the uncontrollable event reported to the supervisor |
| `halt` | stops the robot (also the halt handler when a running `goto` is pre-empted) |
| `wait seconds=` | timed no-op |
| `dock` / `undock` | timed docking (status `charging`) / release |
| `reach` / `stow` | timed arm motions; set the `arm_extended` blackboard flag (`done=` observed on success) |
| `grasp [p=] [ok= miss=]` | timed grasp with success probability `p`; sets `held`; reports `ok=` or `miss=` |
| `place [put=]` | timed placement; clears `held`, sets `placed`, decrements `targets` |
| `set key=value` | writes the blackboard |
| `event emit=<e>` | reports an uncontrollable event (e.g. `v_detect`) |
| `report`, `select_target`, `refine_pose` | mission bookkeeping (`reported`), target availability (`targets > 0`), a timed pose refinement |

Sensed every tick: `battery`, `x`, `y`, `v`, `zone`, `moving`, `dist_human`, `human`, `held`, `placed`,
`arm_extended`, `docked`, `time`, plus `plantState` / `enabled` from the supervisor and `mode` / `vmax` from the
mode automaton. Conditions (`condition "battery < 0.2"`) and monitors read these names.

## Supervisor gating

A `bt` document may name its supervisor (`supervisor <des model name>`); otherwise the first `des` model of the
station is used. The supervisor table is synthesised from that document (plant `automaton` blocks, `spec` blocks,
`uncontrollable`) when the mission starts. Then, for every action with `event=`:

- the uncontrollable events reported by the world since the last check (`arrive=`, `ok=`, `emit=`…) are fed to the
  supervisor first, so that its plant state is current;
- if the supervisor **disables** the event in its current state, the action fails immediately, the denial is
  counted and logged with the plant state (`supervisor denied b_go_box (goto) in plant state T,X,D,F,Hn`) — this is
  the interlock of specification E2 at work: the base may not move while the arm is out;
- otherwise the event is observed (the supervisor advances) and the action starts; `done=` is observed when the
  action succeeds.

An event the plant model does not allow in its current state is a **model mismatch** (the tree and the plant
disagree); it is counted separately. The scenario *Mission runtime* contains a deliberately unsafe tree (move to
the bin with the arm extended) whose `b_go_box` is denied.

## Monitors and modes

`monitor <LTL>` lines become LTL₃ monitors on the blackboard trace: `G(held -> (held U placed))` catches a dropped
object (`⊥` as soon as `held` falls without `placed`), `G(estop -> X halted)` checks the e-stop reaction. A
violation is logged, counted and stored in `bb.violation`; the tree can react to it. A `hybrid` model (named with
`modes <name>` or the first one in the station) runs as the mode machine: its `vmax` becomes the speed limit of
the robot (`SLOW` near a human, `STOP` on e-stop), with the hysteresis and dwell times of the model.

## Automata and Petri nets on the station

Behavior trees are not the only executable model: a `des` document runs as parallel state machines with entry
actions and a `petri` document as a cell controller whose transitions are robot operations — the actions are the
station's programs, targets, zones and signals, bound in the diagram inspector or with `action` lines. See
{doc}`editors` for the semantics and the buttons; the same world bindings (and the same `program`, `move`,
`signal`, `wait`, `set`, `event` host actions) are available to behavior trees through `stationBindings`.

## Exporting the supervisor to a real controller

*Export ▾* on a `des` model:

- **Supervisor table (JSON)** — the course format: `initial`, `uncontrollable`, `unobservable`, and per state the
  plant state tuple, the enabled events and the transitions. A controller loads it and asks `allowed(event)`
  before issuing a controllable command, `observe(event)` on every event that happened.
- **Supervisor runtime (Python)** — a self-contained class with the table embedded (`allowed`, `observe`,
  `enabled`, `plant_state`, a `SupervisorViolation` exception for mismatches), usable in a ROS 2 node or a PLC
  gateway.

Together with the navigation package (Nav2) and the perception package exported from the same station, the
supervisor closes the loop of chapter 15: the mission logic (behavior tree) issues commands, the supervisor gates
them, monitors watch the requirements, and the real robot reports the uncontrollable events.

## From the API and in tests

```ts
import { parseBt } from 'src/ctl/dsl';
import { ControlRuntime, stationBindings, simulatedBindings } from 'src/ctl/runtime';

const rt = new ControlRuntime({ bt: parseBt(source), supervisor: table, bindings: simulatedBindings({ targets: 1 }) });
rt.run(400, 0.1, (bb) => bb.reported === true);
rt.summary();   // ticks, time, status, denied, violations, supervisorErrors, plantState, verdicts
rt.trace;       // { t: number[], values: Record<string, number|boolean>[] } — feed it to robustness(parseSTL(...), rt.trace)
```

`stationBindings({ robot, station, durations, humanPosition })` binds the same tree to a station robot; the app
steps it inside the world loop (`app.worldHooks`). The test suite `tests/ctl_docs.test.ts` runs the course mission
in the simulated world and on a station robot, and asserts zero denials, zero violations and the STL robustness of
the delivery requirement.
