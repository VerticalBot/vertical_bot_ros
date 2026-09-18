# How-to guides: working with the group-control module

Short recipes for the everyday tasks; the practicum pages explain the theory behind each kind.

## 1. From a configured station to a group mission

1. Add the robots (*Mobile & Fleet › Add mobile robot…*), set their kinematics in the properties (max speed,
   acceleration, yaw rate, drive type, footprint) and their homes; add the zones the mission refers to (a dock,
   shelves, a no-go pillar) — *Mobile & Fleet › Zone…*. Keep the goals of the mission (homes, dock, shelves) out
   of the keep-out of the no-go zones (their circumscribed radius plus `d_safe`/2): the safety reflex never lets a
   robot in, so a goal inside one blocks its phase.
2. *Group › New group-control model…* → kind *Group mission*. Write `robots <names>` with the names of your robots,
   the phases (`form`, `goto zone=`, `allocate zones=`, `gather`, `home`), the radio (`comm`) and the safety
   distance (`safety d_safe=`; use the robot footprint plus a margin).
3. **Analyse** with `architecture compare` to see how the three architectures behave with your radio and faults;
   pick one and set `architecture`.
4. **▶ Run on fleet**. Watch the status line (phase, coordinator, messages, minimum distance); *Export ▾ › Fleet
   log* keeps the log; **⏹ Stop** ends the run. The *Simulation* tab's world controls pause the clock.

## 2. Choosing the communication graph and the consensus step

Write a `consensus` document with `graph disk radius=<your radio>` and `robot … at=` lines (or `robots n`) and
read λ₂, 1/Δmax, the optimal step and the r-robustness; `compare path ring star complete` shows what a different
topology would buy. Use `eps` below 1/Δmax in your own consensus loops; use `wmsr F=` when a neighbour may lie or a
sensor may fail.

## 3. Assigning tasks to robots

`allocation` with your `robot`/`task` positions: compare greedy, Hungarian, SSI and CBBA on the same instance; take
Hungarian when a coordinator knows everything (one task per robot), SSI for routes of several tasks with a
coordinator, CBBA when the robots must agree by radio. The `mission` kind uses Hungarian in the centralised
architecture and CBBA in the decentralised one; the `warehouse` kind runs CBBA on a stream of orders with
commitment.

## 4. Planning collision-free paths on a grid and executing them

`gridmapf` with the map (`#` shelves) and the agents: CBS gives the optimal plan; `execute delay=` shows that the
action dependency graph keeps the execution collision-free under delays. **Run on fleet** executes the plan on
the station robots; `map cell=` sets the cell size in metres.

## 5. Covering an area

`coverage` with `area`, a `density` (uniform or a Gaussian hot spot) and `robots n`; `range r` for a sensing radius.
**Run on fleet** drives the robots to their centroids from where they stand.

## 6. Keeping the group safe

Every runnable kind of the module applies the barrier-function reflex from ПР6 with `d_safe` / `gamma` /
`sense` (the `safety` and `mission` kinds expose them directly; `warehouse` uses cell reservation plus the
protective layer). Add `unstuck` when symmetric configurations deadlock, and `stale lag=` when neighbour states
arrive late. On the station, `nogo` zones become obstacles automatically.

## 7. Gating the group with a supervisor and limiting speed with modes

Write a `des` model whose plant lists the phase events (`<phase>_start` controllable, `<phase>_done`
uncontrollable) and the `spec`s of your interlocks; write a `hybrid` model whose modes carry `vmax` and whose
guards read `dist_human`, `estop`, `battery`, `phase`. Name them in the mission: `supervisor <des name>`,
`modes <hybrid name>`. Denials, mismatches and the current mode appear in the status line; the same models can be
analysed (realisability, chattering, dwell time) in the Control tab. A spec constrains only the events of its
alphabet — to forbid a phase, list its `<phase>_start` in the spec's `events` line and give it no transition; a
phase whose events the supervisor does not know is not gated.

## 8. Comparing architectures under faults

`coordinator fail=<t> recover=<t>` and `fail <robot> at=<t>` in a `mission` document; `comm drop=` for packet loss
and `comm radius=` for partitions. Read the stalled / fallback times, the messages and the completion times in the
report table; the formation-error chart shows what each architecture does during the outage.

## 9. Running the fleet homework

`warehouse` — the defaults are the homework map; change `robots`, `orders`, `traffic`, `cbba`, add `fault` lines.
**Build scene** creates the map, the station zones and the robots; **▶ Run on fleet** runs the order flow on
them. The report gives throughput, latency, double commits and path conflicts.

## 10. Testing and automation

- `npm test` runs the module's suites: `mrs_practicum` (the course acceptance tests), `mrs_chapters`,
  `mrs_warehouse`, `mrs_mission`, `mrs_dsl`, `mrs_plots`, `mrs_docs` (templates, examples, runtime on station robots).
- `npm run scenarios` regenerates the scenario report; the group scenarios are the ones with the `grp_` prefix.
- From code: `analyse(kind, source)` returns the report with metrics; `new FleetRuntime({ kind, source, robots,
  station })` runs a model on robots; `compareArchitectures(parseMission(source))` gives the three metric sets.

## 11. Export

*Export ▾*: the report (Markdown), the document (`.ctl.txt`), any chart (SVG), the fleet log. The models are items
of the station file and come back with it; the Python API sees them as `CONTROL_MODEL` items.
