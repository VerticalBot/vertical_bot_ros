# ПР4 — Multi-agent path finding on a warehouse grid (chapter 11)

A `gridmapf` document describes a shelf grid, the agents (or a random instance) and what to run: space-time A*
for the individual paths, prioritized planning with a reservation table, conflict-based search, and the execution of
the plan through an action dependency graph under random delays. Cells are `(row, column)`, a move goes to one of
the four neighbours or waits; the cost of a solution is the sum of arrival times.

```{image} ../_static/screens/67-pr4-mapf.png
:alt: MAPF report: CBS paths on the shelf grid and the execution table
:class: screenshot
:width: 900px
```

## Steps 1–2 — space-time A* and prioritized planning

```text
gridmapf Warehouse crossing
map
  ..........
  .##.##.##.
  …
end
agent a start=0,0 goal=6,9
agent b start=6,9 goal=0,0
agent c start=0,9 goal=6,0
agent d start=6,0 goal=0,9
methods prioritized cbs
execute delay=0.3 runs=5 seed=12
```

The *Instance* section gives the lower bound (sum of the individual shortest paths — 15 steps from (0,0) to
(6,9) on the course map). *Prioritized planning* plans the agents in order, each one avoiding the vertices of the
previous paths at every time (parking included) and the swaps; the sum of costs and the makespan are reported, or
the failure — the method is **not complete**: in *ПР4 · Corridor with a pocket* agent a parks on its goal inside
the corridor and walls agent b in.

## Steps 3–4 — conflicts and CBS

*Conflict-based search* expands a constraint tree: the node with the smallest sum of costs is checked for the first
conflict (vertex conflicts of pairs a < b first, then swaps), two children forbid the cell (or the edge) for either
agent and only that agent replans with space-time A*. The report gives the optimal sum of costs, the number of
expanded nodes, the makespan and the paths, and draws them on the grid (starts as hollow circles, goals as
crosses). In the pocket instance CBS finds the plan where a waits in the pocket; on random instances of the course
map CBS is never worse than the best of the 24 priority orders (`tests/mrs_practicum.test.ts`).

## Step 5 — execution with delays

`execute delay=0.3 runs=5 seed=12` builds the **action dependency graph**: the k-th move of robot i into cell c at
planned time t depends on the exits of every robot that was in c earlier. At every tick a robot performs its next
move when its dependencies are done and it is not delayed (probability `delay`). The table lists, per run, the
ticks to completion and the collisions (always 0) against the collisions of a clock-driven execution that follows
the planned times without coordination (Hönig et al. 2019).

**On the fleet.** **▶ Run on fleet** plans with CBS (or prioritized planning when CBS is not in `methods`) for the
robots of the station (named `a`, `b`, … or the first n mobile robots), executes the ADG in continuous time with
random delays of 0.5–1.5 s and moves the robots between cell centres (`map cell=` sets the cell size in metres,
default 1 m; the map's row 0 is at y = 0 and rows go down). The status line counts the moves done and the robots
at their goals. Load the scenario *ПР4* to get four robots and the three documents.

## Acceptance checklist

| Course test | Studio |
|---|---|
| plain shortest path 15 steps; vertex constraint forces a wait; edge constraint; goal blocked later → 5 steps; unreachable → none | `tests/mrs_practicum.test.ts` › ПР4 |
| corridor crossing SOC 5 without conflicts; incompleteness on `....` | same and *ПР4 · Corridor with a pocket* |
| vertex / swap / parked-agent conflicts detected, following allowed | same |
| CBS solves the incomplete case; never worse than the best priority order on random instances | same |
| ADG dependencies of a follow; delayed execution never collides and ends at the goals | same and *ПР4 · Six random agents* |
