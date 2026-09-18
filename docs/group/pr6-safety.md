# ПР6 — Group safety with barrier functions (§16.4.3, chapter 13)

A `safety` document runs a group of single integrators to their goals through the decentralised barrier-function
filter of the practicum: pair constraints, an exact planar QP, the go-to-goal nominal law, the keep-right rule
against symmetric deadlocks and the margins for stale information about a neighbour.

```{image} ../_static/screens/69-pr6-safety.png
:alt: Safety report: eight robots swap across a circle with the keep-right rule, and the deadlock without it
:class: screenshot
:width: 900px
```

## Steps 1–2 — pair constraint and the QP filter

For a pair the barrier is h = ‖pᵢ − pⱼ‖² − d² and the condition ḣ ≥ −γh keeps h ≥ 0 forever; with mutual trust each
robot takes half: 2(pᵢ − pⱼ)ᵀuᵢ ≥ −γh/2, i.e. aᵀu ≥ b with a = 2(pᵢ − pⱼ), b = −½γh (Wang, Li, Egerstedt 2017 —
the Robotarium filter). The speed limit |u| ≤ vmax is an inscribed 16-gon. The filter solves min ‖u − u_nom‖²
subject to all half-planes **exactly**: the optimum is u_nom itself, its projection on one line or the intersection
of two lines; the feasible candidate closest to u_nom wins, and zero (always feasible while h ≥ 0) is the fallback.

## Step 3 — the group controller

```text
safety Two robots cross
scenario crossing
params d_safe=0.2 gamma=2 vmax=0.3 dt=0.02 steps=1500 sense=1
```

Each robot filters its nominal velocity k(goal − p) with the constraints of every neighbour closer than `sense`.
The report gives the minimum pair distance over the run (≥ d_safe), the goal error at the end, the arrival time,
the trajectories with the goals, and — *Nominal go-to-goal without the filter* — the minimum distance of the
unfiltered law (5 cm: a collision). `scenario custom` with `robot … at= goal=` lines describes any configuration.

## Step 4 — deadlock and the keep-right rule

```text
safety Antipodal swap of eight robots
scenario antipodal n=8 radius=1
params d_safe=0.2 gamma=2 vmax=0.3 dt=0.02 steps=2500 sense=1
unstuck angle=-45 hold=50
```

Eight robots on a circle must reach the diametrically opposite points. With the plain filter everybody yields
symmetrically and nobody passes — the section *Without the deadlock rule* shows the deadlock (max goal error > 0.5
m). The rule of step 4: a robot far from its goal whose progress (projection of the last velocity on the goal
direction) is below `stuck_speed` rotates its nominal velocity by −45° for `hold` steps; all robots turn the same
way, so the symmetry breaks consistently, and the hysteresis prevents the rule from switching off at the first
sideways step. The verdict: minimum distance ≥ d_safe, no deadlock, arrival time.

## Step 5 — stale data about a neighbour

```text
safety Uncooperative neighbour known with delay
robot r1 at=0,0 goal=0,0
robot r2 at=1.2,0 goal=-3,0 speed=0.1
params d_safe=0.3 gamma=2 vmax=0.3 dt=0.02 steps=2500 sense=2 compare=false
stale lag=25 vmax_j=0.1
uncooperative 2
```

Robot 2 has lost the link and drives straight at robot 1 (`uncooperative` — no filter, `speed` 0.1 m/s); robot 1
knows its position `lag` steps (0.5 s) late. The stale constraint enlarges the safety distance to d + vmax_j·age,
takes the whole responsibility (share 1) and adds the worst-case approach term 2‖pᵢ − pⱼ‖vmax_j; robot 1 backs
away, the minimum distance stays at 0.3 m, and robot 1 returns to its goal once the intruder has passed.

**On the fleet.** **▶ Run on fleet** applies the same filter to the station robots: for `antipodal` the goals are
the mirror images of the current positions through their centroid, for `crossing` the first two robots swap
places, for `custom` the goals of the document (metres in the station frame). The run stops when every robot is
within 5 cm of its goal.

## Acceptance checklist

| Course test | Studio |
|---|---|
| a = (2, 0), b = −0.5·2·(1 − 0.25); filter inactive / single active / corner; matches a brute-force grid on 30 random QPs | `tests/mrs_practicum.test.ts` › ПР6 |
| crossing: min distance ≥ 0.2 − 10⁻³, goals within 5 cm; nominal law collides (< 0.05) | same and *ПР6 · Two robots cross* |
| hysteresis of the rule (hold 3 → 2 → 1, rotation −45°); antipodal swap: deadlock without, ≤ 0.1 m goal error with the rule | same and *ПР6 · Antipodal swap* |
| stale constraint a = (2, 0), b = −2(1 − 0.36) + 0.2; evading an uncooperative neighbour keeps 0.3 | same and *ПР6 · Stale data* |
