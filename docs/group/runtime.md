# Fleet runtime: group laws on the station robots

The reports answer *what does the theory predict?*; **▶ Run on fleet** answers *what does it look like on the
robots?*. Six kinds are executable: `consensus`, `swarm`, `coverage`, `safety`, `gridmapf` and `warehouse`. The
runtime (`studio/src/mrs/runtime.ts`) steps the same algorithms as the analysis in the world loop of the studio
(the clock shared with fleets, process components and the mission runtime), reads the robots' positions in metres
(station mm / 1000), computes the group law and writes the poses back every tick — so the 3D view shows the
formation assembling, the flock aligning, the coverage settling, the safety filter bending trajectories, the
MAPF plan being executed or the warehouse fleet serving orders.

## Running

1. Put mobile robots on the station (*Mobile & Fleet › Add mobile robot…*), or *Load into the studio* one of the
   group scenarios (*Help › Demo scenarios…* → *Group control*), which adds the robots and the documents. A
   `warehouse` model creates its own robots: **Build scene**.
2. Select the model in the Control tab and press **▶ Run on fleet** (also *Group › Run selected model on the
   fleet*). The world clock starts; *Simulation › World › Pause* pauses it, **⏹ Stop** ends the run.
3. The status line shows the model-specific state; the run stops when the law's goal is reached (formation error
   below 1 mm, all robots at their goals, the plan executed, the warehouse duration elapsed) or at **Stop**;
   flocks and Vicsek swarms run until stopped. *Export ▾ › Fleet log* saves the log.

Which robots: the `robot <name>` lines of the document when those names exist on the station, otherwise the first
n mobile robots of the station in tree order (n from the document: `graph n=`, `robots n`, `model … n=`, the
number of agents or safety robots). Fewer robots than the model wants: the run uses what is there and says so in
the log. Speed limit: the document's `vmax` for `swarm` and `safety`, otherwise min(0.5 m/s, the robots' kinematic
limit).

## What each kind does on the fleet

| Kind | Law on the robots | Status line | Stops when |
|---|---|---|---|
| `consensus` | formation by offsets (`formation`), connectivity-preserving rendezvous (`rendezvous`), or plain consensus on positions (gather at the centroid); the graph is the document's topology, or a disk graph on the live positions with `comm radius= drop=`; `fail` lines stop a robot at a time | links, formation error / disagreement, failed robots | error < 1 mm |
| `swarm` | `boids`: the Reynolds rules at the document's `dt`; `vicsek`: one step per second on a torus of side `box` around the initial centroid, first η; `pso robots=true`: one PSO step per second, the robots move between way-points with repulsion, noisy measurements at the real positions; ACO / firefly / GWO / bee are analysis-only | polarization and minimum distance; order; best value and distance to the source | never (boids, Vicsek); swarm centre within 0.1 m of the source |
| `coverage` | Lloyd (or limited-range Lloyd) centroids recomputed every second on the document's area and density; robots drive towards their centroids | H and whether the robots still move | robots settled |
| `safety` | the barrier-function filter with the document's parameters; goals: `custom` robots' goals, `antipodal` mirror images through the centroid, `crossing` the first two swap; `unstuck` and `uncooperative` honoured | minimum distance, max goal error | all within 5 cm of their goals |
| `gridmapf` | CBS (or prioritized) plan for the station robots from the document's starts / goals, executed through the ADG with random delays 0.5–1.5 s; cells are `map cell=` metres, row 0 at y = 0 | moves done, robots at their goals | plan complete |
| `warehouse` | the fleet simulator (CBBA, executors, reservation, faults) at the document's `dt`; robots named as in the document (created at their homes when missing) | deliveries, open orders, latency, double commits, path conflicts, executor states | `duration` elapsed |

## Scenes

**Build scene** turns a `warehouse` model into station items: a map (resolution ≤ 100 mm) with the shelves,
zones named after the stations (`loading` for pickups, `unloading` for drops, `parking` for homes), and the robots
at their homes with the fleet speed and a footprint that fits the cell. The origin of the map is the centre of the
top-left cell; x grows with the columns and y decreases with the rows, exactly as the homework's coordinate
convention.

## From the API and in tests

```ts
import { FleetRuntime, buildWarehouseScene } from 'src/mrs/runtime';

const rt = new FleetRuntime({ kind: 'consensus', source, robots: station.itemsOfType(ItemType.MOBILE_ROBOT), station });
for (let t = 0; t < 60 && !rt.done; t += 0.05) rt.tick(0.05);
rt.status();   // '⏹ Formation of six robots — formation: t=9.9 s · 6 links · formation error 0.0010 m'
rt.log;        // the lines that the Log tab shows during a run
```

`tests/mrs_docs.test.ts` runs the formation, the rendezvous, the coverage, the safety crossing, the ADG execution,
a flock and the warehouse (built with `buildWarehouseScene`) on station robots and checks the resulting poses.
