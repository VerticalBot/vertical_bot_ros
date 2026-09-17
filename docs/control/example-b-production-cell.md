# Demo guide — example B: a production cell

Example B of the course is a flexible cell: two machines (`M1`, `M2`), a shared robot / station `S`, a shared
zone `Z`, conveyors and an AMR that delivers pallets. Two process types compete for the robot and the zone in
opposite order, which is the classic recipe for a deadlock; the cell must then be made live, its throughput
bounded and scheduled, its coordination protocol verified, its reliability and safety budgeted, and its
acceptance demonstrated with statistics. Every step below is a `B · …` model of *Control › Course examples…*.

## Step 1 — the deadlock as a Petri net (chapter 4)

Select **B · Cell Petri net: siphon, deadlock, monitor** and analyse. Process A takes the robot `rM` then the
zone `rZ`; process B takes the zone then the robot.

```{image} ../_static/screens/44-petri-report.png
:alt: Petri net report with the net view
:class: screenshot
:width: 620px
```

The net as the report draws it (places as circles with their tokens, resources in yellow, idle places in green,
transitions as bars):

```{image} ../_static/screens/ctl-graph-petri.png
:alt: Petri net of the cell fragment
:class: screenshot
:width: 520px
```

Expected report (verdict *issues found* — that is the point):

- *Structure*: 8 places, 6 transitions, 20 arcs; four **P-invariants** (`p1 + a1 + a2 = 1`,
  `a1 + a2 + b2 + rM = 1`, `p2 + b1 + b2 = 1`, `a2 + b1 + b2 + rZ = 1`) and two **T-invariants** (the two process
  cycles); every place covered → structurally bounded.
- *Behaviour*: 6 reachable markings, safe; **not live**, not reversible; **deadlock marking {a1 b1} reached by
  `t1a t1b`** — A holds the robot and waits for the zone, B holds the zone and waits for the robot.
- *Siphons and traps*: 5 minimal siphons; four contain a marked trap (Commoner) and never empty; the siphon
  **{a2, b2, rM, rZ}** (M₀ = 2) **empties at {a1 b1}** — the deadlock cause.
- *Deadlock prevention with GMEC monitors* (Theorem 4.9): one monitor place **V1: a1 + b1 ≤ 1** (`C_V = −lᵀC`,
  `M₀(V) = β − lᵀM₀`) — the controlled net is **live after 1 iteration**. This is the course's "do not let both
  processes hold their first resource at the same time".
- *Timed simulation* (delays 4/5/1 and 5/4/1 s): deadlock at t = 5 s; *CTL* `AG !deadlock` violated with the
  path `p1 p2 rM rZ → a1 p2 rZ → a1 b1`.

Apply the fix by hand: add `place V1 tokens=1 kind=resource`, replace `arc p1, rM -> t1a -> a1` by
`arc p1, rM, V1 -> t1a -> a1`, `arc p2, rZ -> t1b -> b1` by `arc p2, rZ, V1 -> t1b -> b1`, and return the token
after the second resource is taken (`arc a1, rZ -> t2a -> a2, V1`, `arc b1, rM -> t2b -> b2, V1`). Analyse: live,
reversible, `AG !deadlock` holds, and the timed simulation reports the throughput of each process.

## Step 2 — the same cell from process routes (chapter 4, S³PR)

**B · Cell as an S³PR** describes only the resources and the routes:

```text
resource M1 1
resource Z 1
process A jobs=1: take[M1] 4; zone[M1,Z] 5
process B jobs=1: zone[Z] 5; robot[Z,M1] 4
```

```{image} ../_static/screens/44b-s3pr-report.png
:alt: S³PR report
:class: screenshot
:width: 620px
```

Expected: the generated S³PR has the same 8 places / 6 transitions; the deadlock `{A.take B.zone}` after
`A.start_take B.start_zone`, the bad siphon `{M1, Z, A.zone, B.robot}`, and the monitor **A.take + B.zone ≤ 1**
that makes it live. Increase `jobs=2` for one process to see the reachability graph and the monitor coefficients
change; the banker's algorithm and the resource-ordering rule are available in the API (`bankerSafe`,
`resourceOrder`) when monitors are too restrictive.

## Step 3 — throughput before simulating (chapter 5)

**B · Bottleneck, saturation and cycle time**: per-part busy times M1 9 s, S 22 s, M2 9 s, Z 10 s, AMR 6 s and
the operation chain u1…u5 with its resource holdings.

```{image} ../_static/screens/47f-perf.png
:alt: Performance report: bottleneck, saturation, max-plus
:class: screenshot
:width: 620px
```

Expected:

- *Bottleneck bound* (Proposition 5.1): **T_cycle ≥ 22 s at S → Θ ≤ 163.6 parts/h**.
- *Saturation curve* for the capacity of S: 1 → 22 s (163.6/h), 2 → 11 s (327.3/h), **3 → 10 s, the bottleneck
  moves to the zone Z** — adding a fourth station buys nothing.
- *Max-plus cycle time over the resource circuits*: **λ = 32 s** (112.5 parts/h), critical circuit u2 → u3 → u4
  (the station is held for 5 + 22 + 5 s per part) — the course's §5.6 answer; two stations give 16 s, three 10.67 s.
- *Little's law*: WIP 2.5 = 0.0278 parts/s × 90 s.

The GSPN steady state of the machine (π = 0.1546 / 0.6763 / 0.1691, 110.7 parts/h) is in the scenario
*Petri net: siphon → deadlock → GMEC monitor* and in the `petri` analysis when transitions carry `rate=`.

## Step 4 — verifying the zone protocol (chapter 7)

**B · Shared zone: r2 starves under static priority** is the NuSMV-style model of two robots and a zone with a
static priority for r1 (`g2` requires that r1 is not waiting), with the fairness constraints that nobody stays
in the zone forever.

```{image} ../_static/screens/46-smv-counterexample.png
:alt: Model checking: mutual exclusion holds, no-starvation fails with a counterexample
:class: screenshot
:width: 620px
```

Expected: 12 reachable states; **mutual exclusion holds** (Büchi 32 states, product 124); **"no starvation r2"
is VIOLATED** with a lasso counterexample — a short prefix in which r2 starts waiting, then the cycle
`r1 waiting → r1 in_zone → r1 leaving → r1 waiting …` while `r2.state = waiting` forever; CTL `AG EF owner = none`
holds (the system can always recover, yet r2 never gets in — the difference between possibility and inevitability
the course stresses).

**B · Shared zone with alternation (fixed)** adds a `turn` variable that flips when a robot leaves. Expected:
24 states, mutual exclusion and **both no-starvation properties hold**, `AG EF owner = none` and
`EF r2.state = waiting` hold (the premise is reachable, so the liveness result is not vacuous).

## Step 5 — scheduling the cell (chapter 14)

**B · Job shop: 54 s vs LB 44 s**: three jobs (M1 5 s → S1 22 s or S2 26 s / S1 30 s or S2 24 s → M2 5 s).

```{image} ../_static/screens/49-jobshop.png
:alt: Job-shop schedule with bounds, rules and branch and bound
:class: screenshot
:width: 620px
```

Expected: lower bounds LB1 = 34 (machine load), LB2 = 34 (job length), LB3 = 44 (heads + tails), best
assignment 44 → **LB = 44**; dispatch rules SPT / LPT / EDD / FIFO / CR / ATC give 54, MWKR 59; **branch and
bound: makespan 54, optimal** (120 nodes), gap to the bound 22.7 % — "the gap is structural, not a weakness of
the search" (§14.11). The critical path `J1/M1 → J1/S1 → J3/S1 → J3/M2` and the robustness under 15 % duration
noise (55.8 ± 1.7 s, p95 63.5 s) tell where a buffer helps.

## Step 6 — allocating AMRs and resolving a corridor (chapter 13)

**B · Three AMRs, three pallets**: the 3 × 3 cost matrix of §13.7. Expected: **Hungarian total 81** (AMR1 → cell1,
AMR2 → cell2, AMR3 → cell3) — greedy happens to find the same 81 here; the **minimax assignment gives makespan
55** with a different matching (AMR2 → cell3, AMR3 → cell2). The auction and CBBA block allocate three tasks to
two robots at total 9.5 / makespan 7.2 with CBBA converging in 2 iterations.

**B · Corridor: priority planning fails, CBS succeeds**: two robots must swap ends of a corridor with one pocket.

```{image} ../_static/screens/47g-mapf.png
:alt: MAPF report: priority planning, CBS, temporal plan graph
:class: screenshot
:width: 620px
```

Expected: **priority planning is incomplete** (no path for B in that order); **CBS** finds the optimal plan (sum of
costs 8, makespan 5, 8 constraint-tree nodes): A steps into the pocket while B passes; the temporal plan graph
(10 events, 5 priority arcs, acyclic) executes with A delayed by 3 steps and **0 collisions**; the corridor cells
`c1`, `c2` are **critical sections** — a robot failing there disconnects the others, so an evacuation procedure is
required.

## Step 7 — time windows for the delivery (chapter 10)

**B · Delivery to a machine (STNU)**: the AMR starts at 20 s, travels 8–14 s (contingent), the machine frees at
30–40 s (contingent), unloading must follow within 5 s of the machine being free.

Expected: STN **consistent**, earliest / latest times (arrive 28–34, free 30–40, unload 30–45, makespan
30–45); **dynamically controllable** after 2 Morris reduction rounds — a dispatch strategy exists for any travel
and machine times. Change `start => arrive [8,14]` to `[8,16]`: consistent as an STN but **not dynamically
controllable** — the course's point that consistency of the projection is not enough when durations are not
ours to choose.

## Step 8 — reliability and safety budget (chapter 16)

**B · Reliability budget, SSM, PL**: eight components in series with their failure rates, MTTR 4 h, the
ISO/TS 15066 speed-and-separation parameters and the ISO 13849 risk graph.

```{image} ../_static/screens/50-reliability-safety.png
:alt: Reliability, SSM and performance level report
:class: screenshot
:width: 620px
```

Expected:

- λ_sys = 1.58e-3 h⁻¹ → **MTTF 633 h** (79 shifts), **availability 0.9937** (30 min lost per 80-h week); S and the
  AMR carry 57 % of the failure rate.
- Speed and separation: **S_p = 1.54 m** at 1.2 m/s; with 1 m available the admissible speed is **0.311 m/s** → a
  SLOW mode with hysteresis (the mode automaton of example A).
- Performance level: S2 F2 P2 → **PLr = e**; category 3, MTTFd high, DC medium → **PL d — does not meet PLr e**
  (verdict *issues found*); the report reminds that CBF filters, monitors and learned perception do not count
  towards PL — only certified components do. Change `cat=4 dc=0.99` to reach PL e.
- FDIR: ≤ 1 false alarm per day at 100 Hz → p ≤ 1.16e-7 per check: a 5.3σ threshold or 3σ confirmed three times
  (30 ms delay).

## Step 9 — acceptance statistics and traceability (chapter 17)

**B · Acceptance statistics and traceability**: 97 successes in 100 grasps against a 0.95 target, twenty mission
times against a 15 min limit, a sim-to-real comparison, a pairwise test design over five factors and a
requirements table.

```{image} ../_static/screens/47h-acceptance.png
:alt: Acceptance statistics report
:class: screenshot
:width: 620px
```

Expected:

- **97/100 does not demonstrate 0.95**: the Clopper–Pearson 95 % lower bound is 0.924; the rule of three
  (0.03 at 100 failure-free trials) and the required number of trials (**154** with 3 allowed failures) are
  reported.
- Mission time: mean 11.54 min, one-sided 95 % bound 13.67 < 15 → **met**.
- Sim-to-real: bias 6.6 (12.6 %), spread ratio 3.29, Welch t = 7.3 → **significant** — apply a correction and find
  the missing effect.
- Pairwise covering array: **20 tests** cover every pair of factor values (the full factorial would be 240).
- Traceability: R1 (safety, formalised as `G !(base_mv & arm_mv)`, verified by model checking + monitor) passes;
  R4 is open and not formalised — the report flags it.

## Checklist of expected numbers

| Step | Quantity | Expected |
|---|---|---|
| 1 | Petri net | 6 markings, deadlock {a1 b1} via t1a t1b, siphon {a2 b2 rM rZ} empties, monitor a1 + b1 ≤ 1 → live |
| 2 | S³PR | deadlock {A.take B.zone}, monitor A.take + B.zone ≤ 1 |
| 3 | performance | bound 22 s / 163.6 per h at S; 3 stations → Z bottleneck; max-plus 32 s |
| 4 | model checking | 12 states; mutual exclusion holds; r2 starvation counterexample; fixed model 24 states, all hold |
| 5 | job shop | LB 44, optimal 54, gap 22.7 % |
| 6 | MRTA / MAPF | Hungarian 81, minimax 55; priority fails, CBS sum 8 / makespan 5, TPG 0 collisions |
| 7 | STNU | dynamically controllable with [8,14], not with [8,16] |
| 8 | reliability | MTTF 633 h, A 0.9937, S_p 1.54 m, 0.311 m/s, PL d < PLr e |
| 9 | acceptance | lower bound 0.924 < 0.95, 154 trials needed, 20 pairwise tests |
