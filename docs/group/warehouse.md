# Homework — a multi-robot warehouse (stages 1–4)

The homework of the course builds, in four stages, a fleet of four robots that serve a stream of orders in a
16 × 10 warehouse: a warehouse model and a simulator (stage 1), CBBA on the order stream with commitment plus the
executor state machine of a robot (stage 2), distributed reservation of cells with settle / acknowledgement /
priority rules and asymmetric detours (stage 3), and fault tolerance — lost winners, a protective layer, faults
injected by hand (stage 4). The `warehouse` kind contains all of it without ROS: the analysis runs the whole fleet
for the given duration and reports the homework metrics; **Build scene** and **▶ Run on fleet** put the same
simulation on the station robots.

```{image} ../_static/screens/71-warehouse-report.png
:alt: Warehouse report: metrics, robot positions on the map and the deliveries chart
:class: screenshot
:width: 900px
```

## The document

```text
warehouse Homework fleet: four robots, six stations
robots r1,r2,r3,r4 homes=H1,H2,H3,H4 speed=0.3 handle=3 radio=4 drop=0 latency=0.05
orders rate=0.05 reward=10 first=6 pickups=P1,P2,P3 drops=D1,D2,D3
traffic settle=0.3 stale=2 wait=6 lookahead=2 protective=true
cbba capacity=3 discount=0.98 period=0.5 stale=3 commit=2 lost=6
duration 600 dt=0.1 seed=1
```

The map and the stations of `config/warehouse.yaml` (cell 0.5 m, `#` shelves, P1–P3 pickup on the left, D1–D3 drop
on the right, H1–H4 homes at the bottom) are the defaults; `cell`, a `map … end` block and `station NAME r,c
kind=pickup|drop|home` lines describe another warehouse. `order id P D reward= at=` adds fixed orders, `fault
<robot> at= kind=stop|mute|recover` injects faults.

## Stage 1 — model and simulator

`Warehouse` holds the grid, the stations, an A* over cells with a distance cache; the simulator moves the robots
cell by cell at `speed`, loads and unloads for `handle` seconds, generates the first batch of orders at start and
then Poisson arrivals at `rate`, delivers radio messages to robots within `radio` metres with loss probability
`drop`, and keeps for every receiver the last received state of each neighbour (the staleness rules decide whether
it is still usable).

## Stage 2 — CBBA on orders and the executor

Every `period` each robot rebuilds its bundle over the open orders (score Σ R λ^t from the station where it will
be free, delivered orders and orders committed by anyone excluded), merges the fresh states of its neighbours
(committed orders leave the auction; max bid wins with lexicographic tie-break; release from the first lost
order), and forgets the bids of winners silent for longer than `lost` (their orders return to the auction). When
the head of its route has been won for `commit` seconds the robot **claims** the order through the order board —
a second claim of the same order is counted as a **double commit** (the ROS version's order service rejects it the
same way) and the loser drops it.

The executor of a robot is the seven-state machine of the homework: IDLE → TO_PICKUP → LOADING → TO_DROP →
UNLOADING → IDLE, TO_HOME after `idle_home` seconds without orders, FAILED on `stop` (an order not yet loaded is
released, a loaded one is kept and resumed on `reset`); `nav_failed` retries once towards the pickup and forever
towards the drop. `tests/mrs_warehouse.test.ts` replays the transition tests of the homework.

## Stage 3 — cell reservation

A robot holds the cell it stands in and up to `lookahead` reserved cells ahead; it claims the next cell of its
route with a timestamp and may enter when (1) no fresh neighbour holds it, (2) it has the earliest claim (ties by
name), (3) `settle` seconds passed since its claim and (4) every fresh neighbour's last state was sent after the
claim + settle — the acknowledgement that turns "wait and hope" into confirmation. After `wait` seconds (2·wait
when the blocker has a larger name — the younger robot yields first) the robot detours around the held cells.
The two-robot test of the homework never reserves a cell twice and both arrive.

## Stage 4 — faults and the protective layer

*ДЗ · Robot failure and recovery* stops r2 at 120 s and recovers it at 400 s: its unloaded order is released,
its bids are forgotten after `lost` seconds and the others reclaim its orders; while it stands silent on a cell
the **protective layer** (`protective=true`) keeps its last known cells as obstacles for the reservation and stops
any robot whose proximity sensor sees a body on the next cell, detouring after the wait limit. *ДЗ · Short radio
range and packet loss* (2.5 m, 30 % loss) partitions the fleet: conflicting claims appear and the latency grows,
but the fleet keeps delivering.

## Metrics

| Metric | Meaning |
|---|---|
| delivered / created / open, throughput (orders/h) | the order flow |
| mean / max latency | creation → delivery time |
| double commits | conflicting claims of one order |
| path conflicts, near misses, proximity stops | two robots in one cell, within 0.9 cell, ticks stopped by the sensor |
| detours, messages, utilisation, deliveries per robot | traffic and load |

The report draws the robot positions on the map (5 s snapshots) with the stations, the deliveries-over-time
chart and the last 40 lines of the event log (commits, deliveries, faults, reclaims, conflicts).

## On the station

**Build scene** (Control tab, or *Group › Build warehouse scene from the selected model*) creates a map item with the
shelves, one zone per station (loading / unloading / parking) and the robots at their homes with the fleet speed;
**▶ Run on fleet** then runs the simulator in the world loop and moves the robots, the status line showing
deliveries, latency, conflicts and each executor's state and order. The scenario *Homework: multi-robot warehouse*
builds the scene for you.

```{image} ../_static/screens/72-warehouse-fleet-run.png
:alt: The warehouse scene in the 3D view during a fleet run
:class: screenshot
:width: 900px
```
