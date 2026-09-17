# Demo guide — example A: a mobile manipulator collecting parts

Example A of the course is a mobile manipulator (a youBot-class platform: omnidirectional base, arm, gripper,
camera) that drives from its home to a table, detects and grasps a part, carries it to a bin, places it and
returns, while humans may enter the area, the battery drains and grasps may fail. The course develops it through
chapters 2–17; this guide reproduces every step in the studio and states the expected result. Each step is one
model in *Control › Course examples…* (the `A · …` entries) and one scenario in *Help › Demo scenarios…*.

```{admonition} Setup (2 minutes)
:class: tip
1. Start the studio, *File › New station* (or keep the current one).
2. *Control › Course examples (A: mobile manipulator, B: production cell)…* → **All examples** → *Add*. The
   folder *Control design* appears in the tree with 22 models; the Control tab opens.
3. Drag the top edge of the bottom dock up to give the report more room.
```

## Step 1 — the plant as automata (chapter 2)

Select **A · DES plant + specifications E1 E2 E3 E5** and press **Analyse**. The document has five component
automata — `Base` (home, moving to table / box / home, at table, at box, stopped), `Arm` (stowed, reaching,
extended, withdrawing), `Grip` (open, closing, holding), `Vision` (searching, found), `Object` (on the table, in
hand, fallen, in the box, unrecoverable) — and the uncontrollable events (`b_arrive`, `b_block`, `a_reached`,
`a_stowed`, `g_ok`, `g_miss`, `g_slip`, `v_detect`, `v_lost`).

Expected report:

- *Components* table: Base 9 states / 17 transitions, Arm 4, Grip 3, Vision 2, Object 5.
- *Plant G = Base ‖ Arm ‖ Grip ‖ Vision ‖ Object*: upper bound 1080 states, **648 reachable**, 3172 transitions,
  **non-blocking** (the course: |G| = 648 after the object model).

Try it: remove the `marked Bx Un` line of `Object` (only `Tb` marked) and analyse again — the plant becomes
blocking and the report shows the deadlock / livelock states with the trace that reaches them.

## Step 2 — specifications and the supervisor (chapter 3)

The same document contains the four specifications the course keeps: **E1** the base and the arm never move at
the same time, **E2** the base does not move while the arm is out, **E3** grasp only after a detection, **E5** the
gripper closes only with the arm extended.

Expected report:

- *Supervisor synthesis*: `H = G ‖ E1 ‖ E2 ‖ E3 ‖ E5` has **324 states**; the supervisor keeps **324 states,
  1078 transitions**; iteration 1 removes 0 states by controllability and 0 by blocking — the specifications are
  controllable and non-blocking as they stand (course: 324).
- Disabled controllable events, e.g. `b_go_table ×162`, `g_close ×132`, `a_reach ×108`: the states where the
  plant would allow the event but the supervisor forbids it — that is the interlock.
- Shortest marked run under supervision (13 events): `b_go_table b_arrive a_reach a_reached v_detect g_close g_ok
  a_stow a_stowed b_go_home b_arrive g_slip o_giveup`.
- *Modular supervisors*: E1 alone gives 432 states, E2 / E3 / E5 648 each; their joint behaviour is 324 states and
  **non-conflicting** — four small supervisors can be deployed instead of the monolithic one.
- *Partial observation* (`unobservable g_slip`): the observer has 324 states, 36 of them ambiguous, and the
  decisions are **observable** — the supervisor does not need to see the slip to act correctly.
- *Diagnosability of g_slip*: **not diagnosable** — the fault can stay hidden forever; the course adds a
  presence sensor or a periodic check to fix this (that is requirement R2 monitored at runtime in step 4).
- `check ltl "R1"` holds on the supervised system (Büchi 8 states, product 324); `check ctl` "an object can
  always still be delivered or given up" holds in 324 / 324 states.

*Export ▾ › Supervisor table (JSON)* writes the 324-entry table in the course format; *Supervisor runtime
(Python)* the class with `allowed()` / `observe()`.

```{image} ../_static/screens/43-des-e4-unrealisable.png
:alt: E4 never drop is unrealisable
:class: screenshot
:width: 620px
```

Now select **A · E4 "never drop" is unrealisable** and analyse. E4 forbids `g_slip`, an uncontrollable event:

- *Specification E4: controllability*: **not controllable** — 72 states where the plant enables an uncontrollable
  event the specification forbids (e.g. `g_slip` after `a_reach a_reached b_go_table b_arrive g_close g_ok`).
- *Supervisor synthesis*: `H` has 180 states; iteration 1 removes 36 by controllability and 144 by blocking; the
  result is **empty — UNREALISABLE**. The course conclusion: you cannot legislate physics away; the drop is handled
  by detection and recovery (E2, the monitor, the `Object` model with `o_giveup`).

## Step 3 — the mission behavior tree (chapter 6)

Select **A · Mission behavior tree + supervisor + monitors** (it analyses automatically). The tree has the three
priority layers of the course: `emergency` (e-stop → halt), `battery` (below 20 % → stow, go to the dock, dock),
`mission` (collect one object: go to the table, detect, reach, refine and grasp with two retries, stow, go to the
bin, reach, place, stow; then finish: go home and report). Every action carries its supervisor event
(`event=b_go_table`, `done=a_reached`…), timeouts and the `post="held"` contract on the grasp.

```{image} ../_static/screens/45-bt-report.png
:alt: Behavior tree report with the tree view
:class: screenshot
:width: 620px
```

The graph view of the report (also *Export ▾ › Graph view (SVG)*):

```{image} ../_static/screens/ctl-graph-bt.png
:alt: Behavior tree of the mission: priority layers, memory sequences (*), retry decorator, actions and conditions
:class: screenshot
:width: 900px
```

Expected report:

- *Structure*: 30 nodes, no errors; three warnings about actions without a timeout (`select_target`, `event`,
  `report`) — "a liveness obligation that cannot be monitored" (chapter 9). Add `timeout=1` to silence them.
- *Finite-time success* (Monte Carlo over the leaf models `grasp p=0.8 ticks=3`, `estop p=0`): P(success) = 1.
- *Abstraction for model checking*: 4018 abstract states; `check ltl "halt follows e-stop in the same tick"
  G("ok:estop" -> "tick:halt")` **holds**.
- The two runtime monitors `G(held -> (held U placed))` (R2: a held object is never lost before it is placed) and
  `G(estop -> X halted)` are compiled to 13-state LTL₃ monitors and attached at runtime.

## Step 4 — run the mission under the supervisor (chapters 3, 6, 9)

*Help › Demo scenarios…* → group **Control design** → *Mission runtime: behavior tree under the supervisor with
monitors* → **Load into the studio**. The station now has the youBot, the zones `home`, `table`, `bin`, `dock`
and the two models. In the Control tab select the behavior tree, robot *youBot*, targets 1, **▶ Run mission**.

Expected: the robot drives to the table (plant state `mT,S,O,Sr,Tb` → `T,…`), the arm reaches (`T,R,…` →
`T,X,…`), the gripper closes and holds (`…,C,…` → `…,D,F,Hn`), the arm stows, the robot drives to the bin, reaches,
places (`B,W,O,F,Bx`), stows, returns home and reports; about **38 s** of simulated time, **0 denied, 0 violations,
0 model mismatches**, final plant state `H,S,O,F,Bx`. The toast and the log summarise the run; *Export ▾ › Mission
trace (CSV)* saves the blackboard signals.

```{image} ../_static/screens/56-mission-done.png
:alt: Mission finished: status line with the plant state, denials and monitor verdicts
:class: screenshot
:width: 620px
```

The same scenario, run headlessly (*Run headless*), also executes an **unsafe tree** — go to the table, reach, go
to the bin with the arm extended: the supervisor **denies `b_go_box`** in plant state `T,X,…` (E2), the tree
fails. Edit the mission tree yourself (delete the `stow` before `goto zone=bin`) and run it: the denial appears in
the status line and the log, the robot stays at the table.

Drop test: in a `bt` model add `action set held=false` after the grasp and run — the monitor
`G(held -> (held U placed))` reports `⊥`, *violations 1*.

## Step 5 — synthesising the controller instead of writing it (chapter 8)

Select **A · GR(1) mission synthesis** and analyse. The specification has the system variables `loc` (base,
corridor, table1, table2, bins) and `holding`, the environment variables `obj_at_t1`, `obj_at_t2`, `human_present`,
`battery_low`, the assumptions (objects stay until picked, `GF ¬human_present`, `GF ¬battery_low`) and the
guarantees (motion graph, freeze while a human is present, never carry an object to the base, pick only at a
table with an object, drop only at the bins, deliver both objects, return to base, go to base when the battery
is low).

```{image} ../_static/screens/47-gr1-synthesis.png
:alt: GR(1) synthesis report
:class: screenshot
:width: 620px
```

Expected: game of **160 states**, winning region 152, **REALISABLE**, a Mealy controller with 466 states
extracted (course §8.5). Delete the line `env_live !human_present` and analyse again: the environment can keep
the human present forever, the system cannot move, the liveness guarantees fail — **unrealisable**, and the
report shows the counter-strategy (the environment move that defeats every controller) — exactly the course
argument for why assumptions must be stated.

## Step 6 — task planning and HTN (chapters 10–11)

Select **A · PDDL domain, planning and HTN**. The PDDL domain is the course one (`move`, `pick` with a
`forall` "not blocked" precondition, `place`, `put-aside`, `observe`, action costs = travel times); the problem
has two bolts, one nut, two bins.

```{image} ../_static/screens/48-pddl-plan.png
:alt: PDDL plan and HTN decomposition
:class: screenshot
:width: 620px
```

Expected report:

- *Grounding*: 5 schemas, 11 predicates, 10 objects → 275 ground actions, 301 atoms.
- *Plan (GBFS / h_FF)*: **25 actions, cost 197**, 33 expansions, validated. Change the first line to
  `pddl search=astar heuristic=hmax` to see the optimal-cost search (more expansions, lower or equal cost) and
  `heuristic=hadd` for the informed but inadmissible estimate — the course comparison of table 10.2.
- *HTN decomposition* of `task deliver(bolt1 bin-bolts)`: method M3 "not detected" → goto table1 → observe →
  M2 "on a table" → pick → goto binzone → place: **7 primitive actions after 13 decomposition nodes**, with the
  recursion of `deliver` inside `deliver` shown as a tree.

## Step 7 — grasp decisions under uncertainty (chapter 12)

**A · Grasp strategy MDP**: refine the pose first (3 s, then grasp succeeds with 0.88) or grasp immediately
(8 s, 0.55), abort costs 60 s.

```{image} ../_static/screens/51-mdp.png
:alt: Grasp strategy MDP: values, policy and Q table
:class: screenshot
:width: 620px
```

Expected: value iteration converges in 21 sweeps, policy iteration in 1; **V*(s0) = 12.943** with `refine`
(grasp immediately would cost 15.287), V*(s1) = 9.943, V*(s3) = 16.193 — the course numbers of §12.2.5 (the switch
point p ≈ 0.70 where the direct grasp becomes better is in the scenario *Decisions under uncertainty*).

**A · Classification POMDP**: belief bolt 0.4 / nut 0.4 / other 0.2; `look` (cheap, 0.7 accurate) or
`look_closer` (−4, 0.94) before placing.

```{image} ../_static/screens/47c-pomdp.png
:alt: Classification POMDP report
:class: screenshot
:width: 620px
```

Expected: one-step lookahead `look_closer` **8.68** beats `look` (−1.1) and every immediate placement (−22);
exact two-step value 8.74 with 31 α-vectors; after `look → obs_bolt` the belief is bolt **0.718**; the decision
threshold is **β ≥ 0.943** — "act only when the belief exceeds 0.943; the threshold is a property of the task,
not of the classifier" (§12.4.3). QMDP suggests `look` because it never values information.

## Step 8 — speed modes near a human (chapter 9)

**A · Mode automaton with hysteresis**: `NORMAL` (1.2 m/s) → `SLOW` (0.3 m/s) when the human is closer than 2.0 m,
back above 2.5 m, both with a 0.4 s dwell; `STOP` on e-stop or a human in the zone, restart on acknowledgement.

```{image} ../_static/screens/47b-modes.png
:alt: Mode automaton report with the mode graph
:class: screenshot
:width: 620px
```

Expected: no chattering pairs, hysteresis 0.5 m reported; dwell time bound **τ_d > 0.347 s** for λ = 2 s⁻¹,
μ = 4 (Theorem 9.3), so 0.4 s is enough. Name this model in the behavior tree (`modes A · Mode automaton with
hysteresis`), add an object called *Human* near the table and run the mission: the status shows the mode and
the robot slows down near the human. The CBF filter that keeps the distance constraint is exercised in the
scenario *Hybrid modes with hysteresis and the CBF safety filter* (course numbers 0.102 m/s² … 1.5 m).

## Step 9 — onboard computing and latency (chapter 15)

**A · Onboard task set and latency budget**: four periodic tasks (drive 1.2/5 ms, localization 8/50, planner
25/100, perception 30/200) under the priority-ceiling protocol and the visual-servoing cause–effect chain.

```{image} ../_static/screens/47d-realtime.png
:alt: Real-time analysis report
:class: screenshot
:width: 620px
```

Expected: utilisation 0.80 is above the Liu–Layland bound 0.757, so the exact test runs — response times
**1.2 / 11.6 / 43.8 / 93.8 ms**, all deadlines met; EDF schedulable. Chain latency **313 ms** asynchronous
(register buffers, Σ(T+R)) versus **155 ms** synchronous; at 0.15 m/s the latency costs **47 mm** against a
tolerance of 15 mm → **not OK**, admissible speed 0.048 m/s or move the image stage to its own core — the verdict
of the report is *issues found* on purpose.

## Step 10 — the fault tree of a contact with a human (chapter 16)

**A · Fault tree: contact with a human**: `top = undetected ∧ moving`, `undetected = A ∨ B ∨ C` (scanner
failure 1e-5, human outside the field of view 1e-2, processing delay 1e-3), `moving = D ∧ protect`,
`protect = E ∧ F` (CBF failure 1e-3, hardware chain failure 1e-7).

```{image} ../_static/screens/47e-fta.png
:alt: Fault tree report: cut sets and sensitivity
:class: screenshot
:width: 620px
```

Expected: **3 minimal cut sets of order 4**, P(top) ≈ **1.1e-12**, dominant cut set {B, D, E, F}; the
sensitivity table shows that improving E or F gains 90 %, B 81.7 %, A only 0.1 %. Remove the `event F` line (no
hardware chain) and analyse: the order drops to 3 and the probability rises by seven orders of magnitude — the
course argument for the independent hardware stop chain.

## Step 11 — verifying requirement R6 on a trace (chapter 17)

**A · STL robustness of the distance requirement**: `G[0,60] (human -> dist >= 0.8)` on a six-sample signal.

Expected: robustness **ρ = 0.1** (satisfied with a 0.1 m margin — Theorem 17.2). Change the sample `4 0.9 true`
to `4 0.7 true`: ρ = −0.1, violated. Add `falsify dist0=[0.5,3] v=[0,1.5] budget=200` to search the worst case; the
scenario *V&V* shows falsification finding a violation that random testing with the same budget misses. Paste a
mission trace (*Export ▾ › Mission trace (CSV)*, converted to the `signal` block) to check R6 on the run of step 4.

## Checklist of expected numbers

| Step | Quantity | Expected |
|---|---|---|
| 1 | plant states / transitions | 648 / 3172, non-blocking |
| 2 | supervisor states | 324 (1078 transitions); modular non-conflicting; observer 324 / 36 ambiguous; g_slip not diagnosable |
| 2 | E4 | not controllable (72 states), synthesis empty |
| 3 | tree | 30 nodes, P(success) 1, LTL holds on 4018 abstract states |
| 4 | mission run | reported after ≈38 s, 0 denials / violations / mismatches; unsafe tree: `b_go_box` denied |
| 5 | GR(1) | 160 game states, realisable, 466-state controller; unrealisable without `GF ¬human_present` |
| 6 | plan | 25 actions, cost 197; HTN 7 primitives / 13 nodes |
| 7 | MDP / POMDP | 12.943 (refine) vs 15.287; look_closer 8.68, belief 0.718, threshold 0.943 |
| 8 | modes | hysteresis 0.5 m, τ_d > 0.347 s |
| 9 | real time | R = 1.2 / 11.6 / 43.8 / 93.8 ms; latency 313 / 155 ms; 47 mm > 15 mm |
| 10 | FTA | 3 cut sets of order 4, 1.1e-12 |
| 11 | STL | ρ = 0.1 |
