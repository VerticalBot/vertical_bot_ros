# Architectures: centralised, decentralised and hybrid group control over configured robots

Chapter 3 of the course contrasts three ways of organising the control of a group: a **centralised** architecture
with one planner that knows everything and commands everyone, a **decentralised** one where every robot decides
from its neighbours, and a **hybrid** one that plans globally and executes locally. The `mission` kind lets you run
*the same mission with the same robots, radio and faults* under each of them and measure the trade-offs, and then
execute the chosen architecture on the configured robots of the station — through their own kinematic controllers,
with the station's zones as goals and obstacles, and under a supervisor and a mode automaton of the control-design
layer.

```{image} ../_static/screens/79-architectures-report.png
:alt: Mission report: the three architectures compared, formation error over time, messages
:class: screenshot
:width: 900px
```

## The mission document

```text
mission Form, move, allocate, return — three architectures
robots r1 r2 r3 r4 r5 r6 box=3 seed=1
architecture compare                        # centralized | decentralized | hybrid | compare
comm radius=4 drop=0.05 period=0.5 lost=3 settle=1
coordinator at=0,0 range=12 fail=60 recover=110
phase form circle r=1.2
phase goto at=6,0 speed=0.3
phase allocate targets=8,2;9,-1;7,-3;10,1;8,-2;9,3
phase gather
phase home
safety d_safe=0.4 gamma=2 sense=1.5
obstacle at=3,0.5 r=0.6
fail r4 at=40
duration 300 dt=0.1 seed=1 vmax=0.5
```

A mission is a sequence of **phases**: `form <shape> r=` (circle, line, wedge, grid), `goto at=x,y | zone=<name>
speed=` (the formation moves), `allocate targets=… | zones=…` (one robot per target), `gather`, `cover area=…`
(Lloyd), `home`, `hold seconds=`. `comm` is the radio between robots (range, loss, period, `lost` — the silence
after which a peer or the coordinator is considered lost, `settle` — the agreement time of the decentralised phase
advance); `coordinator` is the fleet manager with its own radio range and an optional outage window; `safety` is the
barrier-function reflex that every robot runs on its sensed neighbours and on `obstacle`s (and on the station's
no-go zones); `fail` stops a robot silently.

## The three architectures

| | Centralised | Decentralised | Hybrid |
|---|---|---|---|
| Information | the coordinator sees every robot it can reach | each robot sees neighbours within the radio range | both |
| Formation slots | Hungarian assignment (minimum total travel) broadcast to the robots | slot = robot index | Hungarian while the coordinator is up; the table is kept for the fallback |
| Formation law | reference point per robot | consensus on ξᵢ = pᵢ − δᵢ with the neighbours | reference point + local consensus term |
| Task allocation | Hungarian on the free robots and the open targets, reassignment when a robot goes silent | CBBA over the radio graph, finished tasks gossiped, silent winners forgotten after `lost` | Hungarian when reachable, CBBA in fallback |
| Phase advance | decided by the coordinator on global criteria | a robot advances when it and all fresh neighbours are done for `settle` s; the phase counter is gossiped (max) | coordinator decides; fallback robots use the local rule; the coordinator adopts the robots' phase when it returns |
| Coordinator loss | robots without a fresh command **hold** (stalled time) | none to lose | robots switch to the decentralised laws after `lost` s (fallback time), resynchronise on return |
| Radio loss / partition | robots out of the coordinator's range stall | components converge separately | mixed |
| Traffic | 2 messages per robot per period (↑ state, ↓ command) | 2 per link per period | both |
| Safety | local barrier filter in all three (sensors, not radio) | | |

**Analyse** with `architecture compare` runs the three and prints the table of the report: completion time, phase
durations, coordinator and peer messages, stalled and fallback time, distance travelled, minimum distance, tasks
done; the chart of the formation error over time shows the centralised group frozen during the outage while the
hybrid one keeps its shape; the logs list the slot assignments, the reassignments after the failure of r4, the
fallback and the resynchronisation. With the course document the numbers read (seed 1):

| architecture | complete | stalled | fallback | coordinator msgs | peer msgs |
|---|---|---|---|---|---|
| centralised | 132 s | 48 s (the whole outage) | 0 | ≈ 1700 | 0 |
| decentralised | 76 s | 0 | 0 | 0 | ≈ 3400 |
| hybrid | 68 s | 0 | 15 s | ≈ 1300 | ≈ 2700 |

*Architectures · Short radio: robots beyond the coordinator* (radio 3.5 m with 20 % loss, coordinator range 3 m on a
wide start) shows the other side: the robots that the coordinator cannot reach never move in the centralised
variant (the mission is reported incomplete with the robots still in their first phase), while the decentralised
group forms and gathers on its own and the hybrid group forms with the far robots in fallback — the coordinator's
slot table reaches them through their neighbours. The verdict of a report is OK when no architecture violates d_safe and, for a single architecture,
the mission completes.

## Running the mission on the configured robots of the station

```{image} ../_static/screens/80-architectures-station.png
:alt: Mission on the station: four configured robots drive to the dock through their own controllers
:class: screenshot
:width: 900px
```

*Architectures · Station mission over configured robots* is meant for **▶ Run on fleet** — load the scenario
*Architectures* (*Help › Demo scenarios… › Group control*) to get the scene, or build your own:

1. **Robots.** `robots r1 r2 r3 r4` names mobile robots of the station; their positions, homes (`home` of the
   robot item), speed limits, acceleration, yaw rate and drive type come from the configured items. The default
   `drive unicycle` sends the group law's velocity reference to each robot as (v, ω) through the point ahead of
   the axle and integrates it with the robot's kinematic limits — a differential-drive robot turns, an Ackermann
   robot respects its turning radius. `drive pose` writes the poses directly (the fast model view).
2. **Zones.** `phase goto zone=Dock` and `phase allocate zones=Shelf A,Shelf B,…` refer to zones of the station by
   name (their centroids in the station frame); every `nogo` zone becomes an obstacle for the safety reflex, and
   the log says so when the run starts. The keep-out of a no-go zone is its circumscribed circle plus half of
   `d_safe`: place homes, docks and shelves outside it, or the phase that targets them can never complete (the
   robots stop at the barrier). A robot that starts inside a keep-out — or is pushed into one by a body that
   stopped too close — leaves it at the speed limit and the log says so; a robot wedged between a body and a
   keep-out backs out through the escalating turn of the deadlock rule (−45°, −90°, −135°, −180°).
3. **Supervisor.** `supervisor Fleet supervisor` names a `des` model of the station. Every phase asks the
   controllable event `<phase>_start` before it starts and reports `<phase>_done` when it ends; a supervisor that
   disables an event makes the group hold (the status line counts the denials and shows the plant state). The
   scenario's supervisor is the plain mission plant; add a `spec` — for example one that never permits
   `gather_start` — and the group stops at that phase, as `tests/mrs_mission.test.ts` checks. Keep the
   Ramadge–Wonham convention in mind: a spec constrains only the events of its alphabet, so an event you want to
   forbid must be listed in the spec's `events` line (with no transition on it); events the supervisor's
   alphabet does not contain are not gated at all. The last `home_done` is reported in the tick the mission
   completes, so a complete run ends in the plant state after the last `_done` (P10 in the scenario).
4. **Modes.** `modes Fleet modes` names a `hybrid` model; its guards read `dist_human`, `human`, `estop`,
   `battery` (the station environment: an item named *Human*, the e-stop, the lowest battery) and `phase`; the
   `vmax` of the current mode limits the whole fleet — the robots slow to 0.1 m/s when a human is within 2 m.
5. **Architecture.** `architecture hybrid` (or `centralized` / `decentralized`; `compare` runs hybrid on the fleet).
   The status line shows the phase, the coordinator state, the message counters, the fallback time, the minimum
   distance, and per robot its phase (`*` = in fallback, `†` = failed).

The same closed loop is what the analysis simulates, so the report of the document predicts what the station run
shows; the difference is the plant — a kinematic single integrator in the report, the configured robots on the
station.

## Choosing an architecture

- **Centralised** when the coordinator is reliable and always in range (a wired cell, a warehouse with a
  fleet manager): global optimality, simple robots, but a single point of failure and a bottleneck in traffic.
- **Decentralised** when the group is large, the radio is local or the environment is hostile: no single point
  of failure, traffic proportional to the neighbourhood, at the price of suboptimal slots / tasks and agreement
  time; watch the partitions (the `consensus` report gives λ₂ and the robustness of your graph).
- **Hybrid** for most fleets: plan globally, execute locally, degrade gracefully. Set `lost` from the radio's
  worst-case silence, `settle` above the gossip round-trip, and test the outage window with `coordinator fail= recover=`.

The pattern is the one of chapter 3 and of the homework: strategic decisions (allocation, phases) by a
coordinator or a consensus auction, tactical ones (formation keeping, reservation) by local laws, and a reflex
layer (barrier functions) that never depends on communication.
