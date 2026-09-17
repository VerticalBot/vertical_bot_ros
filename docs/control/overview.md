# Control design: the upper control level of robots and robot complexes

The **Control** tab turns the studio into a design tool for the *upper* control level of a robot or of a complex of
robots — the level above the servo loops and the motion planner where missions, resource sharing, safety
interlocks, supervisors and schedules are decided. Every method of the course *Управление робототехническими
комплексами* (control of robotic complexes, BMSTU) is implemented as a studio feature: you write a small text model
(an automaton, a Petri net, a behavior tree, a temporal specification, a planning domain, a task set…), press
**Analyse**, and get a verdict with the evidence — deadlocks with the path that leads to them, livelocks as cycles,
unrealisable specifications with the reason, model-checking counterexamples, synthesised supervisors and
controllers, schedules, bounds and safety numbers. A behavior tree can then be **run on a station robot** under the
synthesised supervisor with runtime monitors, and the supervisor can be exported as a JSON table or a Python class
for a real controller.

```{image} ../_static/screens/42-control-tab-des.png
:alt: Control tab with the course example A (youBot plant, supervisor synthesis)
:class: screenshot
:width: 900px
```

```{admonition} Where to start
:class: tip
- The two worked examples of the course are available as ready models: *Control › Course examples…* adds them to
  the station (example **A** — a mobile manipulator collecting parts, example **B** — a production cell with two
  machines, a shared robot and an AMR). The step-by-step guides are {doc}`example-a-mobile-manipulator` and
  {doc}`example-b-production-cell`.
- Prefer drawing? {doc}`editors` — automata and Petri nets edited as diagrams, with robot programs, targets, zones and signals bound to states and transitions and executed on the station.
- {doc}`dsl` is the reference of the model languages; {doc}`checks` lists every check the analysis performs and
  what to do when it fails; {doc}`runtime` explains how a behavior tree is executed on the station and how the
  supervisor reaches a real robot.
- *Help › Demo scenarios…* → group *Control design* runs the same analyses headlessly and compares the numbers with
  the course (see {doc}`../reference/scenarios`).
```

## The Control tab

The bottom dock has a **Control** tab (also *Control › Open Control tab*). Left: the list of control models of the
station (they live in the tree under the folder *Control design* and are saved with the station). Right: the editor
of the selected model, the analysis report and the mission runtime.

```{image} ../_static/screens/40-control-menu.png
:alt: Control menu
:class: screenshot
:width: 520px
```

```{image} ../_static/screens/41-course-examples-dialog.png
:alt: Course examples dialog
:class: screenshot
:width: 560px
```

| Element | What it does |
|---|---|
| **New ▾** | creates a model of one of the 22 kinds from a template (grouped: discrete-event, executive, verification, planning, coordination, engineering) |
| **Course examples ▾** | adds a course example (or all of them) — A · … are example A, B · … example B, with the chapter of the course |
| **Delete** | removes the selected model from the station |
| name, kind | the model name (tree item) and its kind; **Detect kind** reads the first keyword of the document |
| **Text** / **Diagram** | the text editor, or the graphical editors for automata, Petri nets and behavior trees ({doc}`editors`): states / places / transitions / arcs / tree nodes drawn and inspected, robot actions bound to states, transitions and action leaves |
| editor | the model text; `Tab` indents, `Ctrl+Enter` analyses; comments start with `#` |
| **Analyse** / **Analyse all** | runs the analysis of the selected / of every model; the verdict is stored on the item (tree badge ✓ / ✗, properties panel) |
| report | sections with a level (✅ ok, ⚠️ warning, ❌ error, ℹ️ info), tables, graph views (automata, Petri nets, behavior trees, mode automata, synthesised controllers) and the metrics table |
| **Export ▾** | report (Markdown), the document, the graph view (SVG); for automata models the synthesised **supervisor table (JSON)** and a **supervisor runtime (Python)**; after a mission run the trace (CSV) and the log |
| **Mission runtime** | robot selector, number of objects (`targets`), **▶ Run mission** (behavior trees) or **▶ Run on station** (automata and Petri nets with bound actions) / **⏹ Stop**; status line with the plant state / marking, running actions, the mode, denials, monitor verdicts |

The properties panel of a control model shows its kind, the course chapter, the last verdict and buttons to open
it in the tab or analyse it.

```{image} ../_static/screens/53-control-model-properties.png
:alt: Properties of a control model
:class: screenshot
:width: 420px
```
 Models are ordinary items: they are copied with the station file, undo / redo applies,
and the Python API sees them as items of type `CONTROL_MODEL`.

## From theory to features: the course chapter by chapter

| Chapter | Theory | In the studio | Model kind | Course example |
|---|---|---|---|---|
| 1 Architecture of control | levels of control, executive / supervisory / planning layers, requirements R1–R6 | the Control tab itself: models per layer, traceability matrix in the acceptance model | `acceptance` | B · Acceptance |
| 2 Automata and languages | DFA, parallel composition, accessible / co-accessible / trim, blocking | `automaton` blocks, `G = G₁ ‖ … ‖ Gₙ`, reachability, deadlock / livelock detection with traces | `des` | A · DES plant |
| 3 Supervisory control (Ramadge–Wonham) | controllability, supremal controllable sublanguage (Algorithm 3.1), modular supervisors and non-conflict, partial observation (observer, observability), diagnosability | `spec` blocks + `uncontrollable`, synthesis with the iteration log, disabled events, shortest marked run, modular non-conflict check, observer and observability, twin-plant diagnoser; supervisor table and Python export | `des` | A · DES plant, A · E4 unrealisable |
| 4 Petri nets | reachability graph, boundedness, liveness, reversibility, P/T-invariants (Farkas), siphons and traps (Commoner), GMEC monitors, S³PR, banker's algorithm, resource ordering | `petri` and `s3pr` documents: full structural and behavioural analysis, deadlock cause as an emptying siphon, monitor synthesis `C_V = −lᵀC`, controlled net re-checked | `petri`, `s3pr` | B · Cell Petri net, B · Cell as an S³PR |
| 5 Performance of discrete-event systems | bottleneck bound, saturation, Little's law, timed nets, GSPN / CTMC steady state, max-plus cycle time | `perf` document (bottleneck, saturation curve, resource-circuit cycle time, Little), timed simulation and GSPN steady state inside the Petri analysis | `perf`, `petri` | B · Bottleneck, saturation and cycle time |
| 6 Executive layer: behavior trees and statecharts | tick semantics, memory nodes, decorators, preemption, skill contracts (pre / post / timeout), hierarchical FSM with history and regions | `bt` documents with structural checks, Monte Carlo finite-time success, abstraction to a Kripke structure for LTL / CTL; `statechart` documents flattened to an automaton | `bt`, `statechart` | A · Mission behavior tree |
| 7 Model checking | Kripke structures, LTL / CTL, Büchi automata (GPVW), nested DFS, fairness, counterexamples | `smv` synchronous models (NuSMV-like `case` / `esac`), `ltlspec` / `ctlspec` / `fairness`; `check ltl|ctl` lines in every discrete model; shortest lasso counterexamples | `smv`, `check` lines | B · Shared zone: r2 starves, B · alternation fix |
| 8 Reactive synthesis (GR(1)) | environment assumptions, system guarantees, three nested fixpoints, strategy extraction, counter-strategies | `gr1` documents: realisability, winning region, Mealy controller (graph view ≤ 60 states) or the environment counter-strategy | `gr1` | A · GR(1) mission synthesis |
| 9 Hybrid systems and runtime safety | mode automata, hysteresis, dwell time (Theorem 9.3), control barrier functions, LTL₃ runtime monitors | `hybrid` documents (chattering / hysteresis check, dwell time), the CBF filter in the runtime, `monitor` lines attached to behavior trees | `hybrid`, `bt` | A · Mode automaton with hysteresis |
| 10 Task planning | STRIPS / PDDL, grounding, delete relaxation (h_max, h_add, h_FF), A* / GBFS, plan validation, STN / STNU | `pddl` documents (typed PDDL with `forall`, costs), plan with cost and expansions, validation; `stn` / `stnu` documents (Floyd–Warshall, Morris dynamic controllability) | `pddl`, `stn` | A · PDDL domain, B · Delivery to a machine (STNU) |
| 11 Hierarchical planning and integration | HTN methods, plan–execute–replan, task and motion planning | `method` / `task` lines in a `pddl` document (decomposition tree), replanning loop in the API (`executeWithReplanning`), TAMP helpers | `pddl` | A · PDDL domain, planning and HTN |
| 12 Decisions under uncertainty | MDP value / policy iteration, sensitivity, POMDP beliefs, α-vectors, QMDP, decision thresholds, shields | `mdp` documents (V*, π*, Q table), `pomdp` documents (lookahead, belief update, α-vector value, threshold β) | `mdp`, `pomdp` | A · Grasp strategy MDP, A · Classification POMDP |
| 13 Multi-robot coordination | task allocation (Hungarian, bottleneck, auctions, CBBA), MAPF (priority planning, CBS, temporal plan graph) | `mrta` and `mapf` documents | `mrta`, `mapf` | B · Three AMRs, B · Corridor with a pocket |
| 14 Scheduling | dispatch rules, lower bounds, job shop / FJSP, branch and bound, tabu, robustness, OEE | `jobshop` documents (rules table, bounds, optimal schedule with critical path, robustness under duration noise) | `jobshop` | B · Job shop: 54 s vs LB 44 s |
| 15 Real-time and software architecture | rate-monotonic analysis, response times, EDF, priority ceiling, end-to-end latency of cause–effect chains, QoS | `realtime` documents (RTA table, EDF test, chain latency asynchronous / synchronous, speed–tolerance budget) | `realtime` | A · Onboard task set and latency budget |
| 16 Reliability and functional safety | series reliability, MTTF, availability, FMEA, FTA (MOCUS, cut sets), ISO 13849 PL, ISO/TS 15066 speed and separation, FDIR thresholds | `reliability`, `fta`, `fmea` documents | `reliability`, `fta`, `fmea` | A · Fault tree, B · Reliability budget, SSM, PL |
| 17 Verification and validation | STL robustness, falsification, pairwise testing, rule of three, Clopper–Pearson, sim-to-real gap, acceptance, traceability | `stl` documents (robustness on a signal, falsification over a parameter space), `acceptance` documents | `stl`, `acceptance` | A · STL robustness, B · Acceptance statistics |

## Model kinds

| Kind | Document starts with | Group | Analyses |
|---|---|---|---|
| `des` | `des <name>` | discrete-event | composition, blocking, supervisor synthesis, modular non-conflict, observer / observability, diagnosability, LTL / CTL, supervisor table |
| `petri` | `petri <name>` | discrete-event | invariants, reachability, bounds, liveness, reversibility, deadlocks, siphons / traps, GMEC monitors, timed simulation, GSPN steady state, LTL / CTL |
| `s3pr` | `s3pr <name>` | discrete-event | the S³PR is built from process routes and analysed like a Petri net |
| `perf` | `perf <name>` | discrete-event | bottleneck bound, saturation curve, max-plus cycle time, Little's law |
| `bt` | `bt <name>` | executive | structure (contracts, timeouts, memory), finite-time success, Kripke abstraction + LTL / CTL, monitors; runnable on a station robot |
| `statechart` | `statechart <name>` | executive | hierarchy / regions / history flattening, reachability, LTL / CTL |
| `hybrid` | `hybrid <name>` | executive | chattering and hysteresis, dwell time, mode graph; usable as the mode machine of a mission |
| `smv` | `smv <name>` | verification | Kripke structure from a synchronous model, LTL with fairness, CTL, counterexamples |
| `gr1` | `gr1 <name>` | verification | realisability, winning region, controller extraction, counter-strategy |
| `stl` | `stl <name>` | verification | robustness of STL formulas on a signal, falsification |
| `pddl` | `pddl [options]` | planning | grounding, heuristic search, validation, HTN decomposition |
| `stn` / `stnu` | `stn <name>` | planning | consistency, earliest / latest times, dynamic controllability |
| `mdp` | `mdp <name>` | planning | value / policy iteration, Q values |
| `pomdp` | `pomdp <name>` + JSON | planning | lookahead, belief update, α-vectors, decision threshold |
| `mrta` | `mrta <name>` + JSON | coordination | Hungarian, greedy, bottleneck assignment, auction, CBBA |
| `mapf` | `mapf <name>` + JSON | coordination | priority planning, CBS, temporal plan graph, critical sections |
| `jobshop` | `jobshop <name>` | coordination | lower bounds, dispatch rules, branch and bound, critical path, robustness |
| `realtime` | `realtime <name>` | engineering | RTA, EDF, blocking, end-to-end latency, latency budget |
| `reliability` | `reliability <name>` | engineering | MTTF / availability, ISO/TS 15066 SSM, ISO 13849 PL, FDIR threshold, Weibull replacement |
| `fta` | `fta <name>` | engineering | minimal cut sets, top-event probability, sensitivity |
| `fmea` | `fmea <name>` + JSON | engineering | RPN and action priority table |
| `acceptance` | `acceptance <name>` + JSON | engineering | trials / confidence, bounds on samples, sim-to-real gap, pairwise array, traceability |

## Verdicts

A report is **OK** when every check passed and nothing was found that the course treats as a defect: no
blocking state, a realisable specification, a live and bounded net, all LTL / CTL properties satisfied, all
deadlines met, PL ≥ PLr, statistics that demonstrate the target. **Issues found** is not necessarily a failure of
the model — the course examples are chosen so that several of them *must* report issues (the deadlock of the cell,
the unrealisable E4, the starvation under static priority, the PL that misses PLr, the 97/100 trials). Each section
says what was found and how the course fixes it; the fixed variants are also among the examples.

## Designing your own complex

1. **Plant**: one `automaton` per physical component (base, arm, gripper, machine, conveyor, door…) with its
   events; mark the uncontrollable ones (arrivals, sensor events, faults). Analyse: the composition should be
   non-blocking; if not, the report shows the deadlock and the path to it.
2. **Specifications**: one `spec` automaton per rule (mutual exclusion, ordering, interlock). Analyse: the
   supervisor is synthesised or the specification is reported unrealisable with the state where an uncontrollable
   event breaks it. Export the supervisor table for the real controller.
3. **Resources**: when the interaction is about shared resources, model it as a `petri` net or an `s3pr`; the
   analysis finds the deadlock cause (a siphon that empties) and adds the monitor places that prevent it; the
   `perf` document gives the throughput bound before any simulation.
4. **Mission logic**: a `bt` document per robot, with `event=` / `done=` bindings to the plant events so that the
   supervisor gates every controllable action, `monitor` lines for the runtime requirements and a `hybrid` mode
   automaton for speed limits near humans. Run it on the station robot ({doc}`runtime`).
5. **Properties**: `check ltl` / `check ctl` in any discrete model, an `smv` model for the coordination protocol,
   a `gr1` specification when the controller should be synthesised rather than written.
6. **Plans, decisions, schedules, coordination**: `pddl`, `mdp` / `pomdp`, `jobshop`, `mrta` / `mapf` documents
   for the parts of the system that plan rather than react.
7. **Engineering evidence**: `realtime`, `reliability`, `fta`, `fmea`, `stl`, `acceptance` documents make the
   numbers of chapters 15–17 part of the station file — the traceability matrix ties requirements to the models
   that verify them.

```{image} ../_static/screens/54-control-scenarios.png
:alt: Demo scenarios dialog, control-design group
:class: screenshot
:width: 760px
```

*Control › Analyse all models* re-runs everything; the log lists the verdicts, the tree badges show them, and
*Export ▾ › Report (Markdown)* produces the document for the design review.
