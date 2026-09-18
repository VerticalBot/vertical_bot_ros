# Group control: multi-robot systems as studio features

The **group-control module** brings the course *Управление распределёнными робототехническими системами*
(control of distributed robotic systems, BMSTU) into the studio the way the control-design layer brought the course
on robotic complexes: every method of the seventeen chapters and every step of the six practicum works and the
homework is a model you can write, analyse and — for the group laws — **run on the mobile robots of the station**.
Consensus and formations on communication graphs, swarm behaviour and swarm intelligence, task allocation by
auctions and CBBA, multi-agent path finding on a warehouse grid, area coverage and distributed estimation, group
safety with barrier functions, the multi-robot warehouse of the homework, and the theory examples (games,
reinforcement learning, evolutionary tuning, cellular automata, fuzzy control, Byzantine resilience) are fourteen
document kinds with reports, charts and demo scenarios whose numbers match the course text.

```{image} ../_static/screens/61-group-tab-consensus.png
:alt: Control tab with a consensus model of the group-control module: communication graph, iterations and chart
:class: screenshot
:width: 900px
```

```{admonition} Where to start
:class: tip
- *Group › Course examples…* adds the practicum documents (ПР1–ПР6), the warehouse homework and the chapter
  examples to the station; the guides are {doc}`pr1-consensus`, {doc}`pr2-swarm`, {doc}`pr3-allocation`,
  {doc}`pr4-mapf`, {doc}`pr5-coverage`, {doc}`pr6-safety`, {doc}`warehouse` and {doc}`chapters`.
- {doc}`architectures` shows how to build the group layer over the configured robots of the station and compares
  the centralised, decentralised and hybrid control of the same mission; {doc}`guides` collects the how-to recipes.
- {doc}`dsl` is the reference of the fifteen document languages; {doc}`runtime` explains **▶ Run on fleet**,
  the warehouse scene builder and how the same laws drive the station robots.
- *Help › Demo scenarios…* → group *Group control* (or *Group › Demo scenarios: group control…*) runs the eight
  scenarios headlessly and compares the numbers with the course ({doc}`../reference/scenarios`).
- The module is a separate package of the studio (`studio/src/mrs/`), independent of the renderer; it plugs into
  the same Control tab, station tree and API as the control-design models.
```

## The module in the studio

The documents are control models of the station (tree folder *Control design*, type `CONTROL_MODEL`) whose kind
belongs to the group **Group control (multi-robot systems)**. They share the Control tab with the control-design
kinds: the text editor, **Analyse**, the report with sections / tables / charts / graph views, **Export ▾**, undo /
redo. What is specific to the module:

| Element | What it does |
|---|---|
| **Group** menu | *New group-control model…*, *Course examples: group control…*, *Run selected model on the fleet*, *Stop fleet run*, *Build warehouse scene from the selected model*, *Demo scenarios: group control…* |
| **New ▾ › Group control** | the fifteen kinds, labelled with the practicum (ПР1 … ДЗ) and the chapters of the course |
| **Group examples ▾** | the course documents grouped by practicum / homework / chapter, or *Add all group examples* |
| report charts | time series (states, order parameter, coverage cost, fitness), planar trajectories with goals, sources, stations and shelves, cell grids (Voronoi labels, cellular automata, pheromone fields), bars (Shapley value); every chart exports as SVG with *Export ▾ › Graph view (SVG)* |
| communication graph | drawn as an undirected graph view, pinned to the robot positions when the model gives them |
| **▶ Run on fleet** | executes a `mission`, `consensus`, `swarm`, `coverage`, `safety`, `gridmapf` or `warehouse` model on the mobile robots of the station in the world loop; the status line shows links, errors, distances, deliveries; missions drive the configured robots through their own kinematic controllers ({doc}`architectures`) |
| **Build scene** | for a `warehouse` model: the map with the shelves, one zone per station and the robots at their homes |

```{image} ../_static/screens/62-group-menu.png
:alt: Group menu
:class: screenshot
:width: 560px
```

## From theory to features: the course chapter by chapter

| Chapter | Theory | In the studio | Kind | Course document |
|---|---|---|---|---|
| 1 Complex systems and distributed robotics | emergence, order–disorder transition, the consensus of three agents (§1.7) | the Vicsek phase transition, the consensus iterations of a small graph | `swarm`, `consensus` | ПР2 · Vicsek, ПР1 · Chain P₄ |
| 2 Multi-agent systems | agents, interaction protocols, contract net | contract-net message count next to the auctions, CBBA as a consensus protocol | `allocation` | ПР3 · Nine tasks |
| 3 Architectures | centralised / decentralised / hybrid, radio model | `mission` documents: the same mission under the three architectures (coordinator, radio range / loss, outages, fallback), executed on the configured station robots under a supervisor and a mode automaton ({doc}`architectures`); radio model of the fleet simulator | `mission`, `warehouse`, `consensus` | Architectures · three documents, ДЗ · Short radio range |
| 4 Graphs, consensus, formations | Laplacian, λ₂ (Fiedler), discrete step 1/Δmax, convergence factor ρ, optimal step 2/(λ₂+λₙ), topology table 4.1, formation by offsets, cooperative transport, potential fields, cellular automata (§4.6) | `consensus` documents: spectrum, λ₂, ρ, iterations to 1 %, the three iterations of §4.3.4, topology comparison, formation, transport regimes; `ca` documents: Wolfram rules, Life, pheromone | `consensus`, `ca` | ПР1 · Chain P₄, ПР1 · Formation, Chapter 4 · Rule 90 |
| 5 Swarm intelligence | Reynolds rules, polarization, Vicsek, ACO, PSO (§5.6.4), bee / firefly / grey wolf | `swarm` documents: boids, Vicsek order vs noise, PSO lbest, robot source seeking with repulsion / noise / forgetting, ACO tour, firefly / GWO / bee optimisers | `swarm` | ПР2 · four documents, Chapter 5 · ACO |
| 6 Agents and ontologies | FIPA-style protocols, contract net | contract-net traffic vs CBBA messages in the allocation report | `allocation` | ПР3 |
| 7 Reinforcement learning | Q-learning / SARSA, the corridor of §7.2.4 (table 7.2), independent MARL, CTDE | `marl` documents: table 7.2 reproduced, ε-greedy learning on grids, independent Q-learning of two robots with collision penalties | `marl` | Chapter 7 · Corridor |
| 8 Distributed automata and Petri nets | FSM, statecharts, Petri nets of the group | the executor state machine of the warehouse (IDLE … FAILED) and the control-design kinds `des`, `petri`, `statechart` | `warehouse` (+ ctl) | ДЗ |
| 9 Game theory | dominance, Nash (pure / mixed), Pareto, best response, fictitious play, potential games, Shapley and the core, Vickrey auctions | `game` documents (bimatrix + coalition); second-price auction in `allocation` | `game`, `allocation` | Chapter 9 · Task allocation game |
| 10 Task allocation | Hungarian, greedy, SSI auction, CBBA (bundle / consensus), market methods | `allocation` documents; CBBA on an order stream with commitment and lost winners in `warehouse` | `allocation`, `warehouse` | ПР3, ДЗ |
| 11 Navigation, MAPF, coverage | space-time A*, prioritized planning, CBS, action dependency graph; Voronoi / Lloyd coverage | `gridmapf` and `coverage` documents | `gridmapf`, `coverage` | ПР4, ПР5 |
| 12 Evolutionary computation | GA (§12.2.6), evolution strategies, differential evolution, online adaptation | `evo` documents: the GA generation of the text, DE / ES on benchmarks, ES tuning the consensus step | `evo` | Chapter 12 |
| 13 Hybrid control | fuzzy controller (§13.4.2), switched systems and dwell time, event-triggered control | `fuzzy` documents (Mamdani, table 13.2, closed loop); `switched` line of `resilience`; `event` line of `consensus` | `fuzzy`, `resilience`, `consensus` | Chapter 13 · Fuzzy, Chapter 16, ПР1 · Event-triggered |
| 14 Collaborative robots | speed and separation, contact limits | the barrier-function filter of ПР6 and the control-design `reliability` (SSM) kind | `safety` (+ ctl) | ПР6 |
| 15 Human in the loop | operator interventions, faults injected by hand | `fault` lines of the warehouse (stop / mute / recover) and the fleet runtime log | `warehouse` | ДЗ · Robot failure |
| 16 Safety and resilience | barrier functions (§16.4.3), graceful degradation (§16.5.3), Byzantine generals (§16.8), W-MSR, trust | `safety` documents (pair CBF, QP filter, deadlock rule, stale data); `resilience` documents (OM(1) agreement, trust consensus, degradation); `wmsr` line of `consensus` | `safety`, `resilience`, `consensus` | ПР6, Chapter 16, ПР1 · W-MSR |
| 17 Perspectives | metrics of a fleet, scalability | the metrics of the warehouse simulator (throughput, latency, double commits, path conflicts, messages) and the demo scenarios | `warehouse` | ДЗ |

## Model kinds

| Kind | Document starts with | Practicum | Analyses |
|---|---|---|---|
| `mission` | `mission <name>` | ch. 3 | the same phased mission (form, goto, allocate, gather, cover, home, hold) under the centralised, decentralised and hybrid architectures: completion, phase durations, coordinator / peer messages, stalled and fallback time, travel, minimum distance, tasks; formation-error chart; executable on configured robots with zones, no-go obstacles, a `des` supervisor and a `hybrid` mode automaton |
| `consensus` | `consensus <name>` | ПР1 | graph, Laplacian spectrum, λ₂, 1/Δmax, ρ, iterations, topology table, formation, connectivity-preserving rendezvous, W-MSR (+ robustness), event-triggered updates, cooperative transport |
| `swarm` | `swarm <name>` | ПР2 | boids (polarization, distances, trajectories), Vicsek order vs noise, PSO / robot source seeking, ACO, firefly / GWO / bee |
| `allocation` | `allocation <name>` | ПР3 | greedy vs Hungarian, SSI rounds and bids (with the exhaustive optimum on small instances), CBBA iterations / messages / conflicts, Vickrey payments, contract-net traffic |
| `gridmapf` | `gridmapf <name>` | ПР4 | shortest paths, prioritized planning, CBS, conflicts, ADG execution with delays vs clock-driven execution |
| `coverage` | `coverage <name>` | ПР5 | Voronoi labels, masses, coverage functional per iteration, Lloyd / limited-range Lloyd trajectories |
| `estimation` | `estimation <name>` | ПР5 | information consensus vs plain averaging vs the centralised estimate, covariance intersection |
| `safety` | `safety <name>` | ПР6 | pair CBF constraints and the exact QP filter, minimum distance, goals reached, deadlock detection, keep-right rule, stale-data margins, nominal comparison |
| `warehouse` | `warehouse <name>` | ДЗ | the fleet simulator: deliveries, throughput, latency, double commits, path conflicts, detours, messages, utilisation, event log, trajectories |
| `game` | `game <name>` | ch. 9 | dominance, pure / mixed Nash, Pareto, best-response and fictitious play, potential, replicator dynamics, Shapley value and core |
| `marl` | `marl <name>` | ch. 7 | Q-learning / SARSA, table 7.2, Q vs Q*, greedy path, independent Q-learning of two robots |
| `evo` | `evo <name>` | ch. 12 | GA (roulette table, generations), DE / ES on benchmarks, ES tuning of the consensus step |
| `ca` | `ca <name>` | ch. 4 | Wolfram rules (symmetry, density), Life (period / shift of a glider), pheromone field |
| `fuzzy` | `fuzzy <name>` | ch. 13 | memberships, rule strengths, centroid outputs, response surface, closed-loop avoidance |
| `resilience` | `resilience <name>` | ch. 16 | Byzantine agreement (OM(1), n ≥ 3f+1), trust-weighted consensus, graceful degradation, switched-system dwell time |

## Verdicts

A report is **OK** when the course criterion of the method holds: the graph is connected and the step admissible,
the formation error vanishes, no initial link breaks during the rendezvous, W-MSR keeps the normal agents inside
their initial range, the flock aligns without collisions, the swarm reaches the source, CBBA ends conflict-free,
the plans are collision-free and the ADG execution never collides, the coverage cost decreases monotonically, the
estimates converge to the centralised one, the minimum distance never falls below d_safe and nobody deadlocks, the
fleet delivers with no path conflicts, the honest agents agree. Several course documents are *meant* to report
issues — W-MSR on a 1-robust chain, three agents with one traitor, the antipodal swap without the deadlock rule —
and the sections say why and how the course fixes it.

## Designing your own group

1. **Communication**: write the graph of your fleet (`graph disk radius=` with the robot positions, or a
   topology) and read λ₂, the admissible step and the robustness in the `consensus` report; compare topologies.
2. **Group law**: formation offsets, rendezvous with connectivity maintenance, a flock, a coverage density, goals
   with a safety filter — one document each; **Run on fleet** shows the law on the station robots.
3. **Tasks**: `allocation` for a one-off assignment (compare greedy, Hungarian, SSI, CBBA on your positions);
   `gridmapf` for the conflict-free paths and their execution under delays; `warehouse` for the whole order flow.
4. **Robustness**: `wmsr`, `event`, `fail` lines, `stale` data, radio range / loss / faults of the warehouse,
   Byzantine and trust experiments in `resilience`; the reports say what breaks first.
5. **Learning and tuning**: `marl` and `evo` documents when a parameter (step, gains, policy) should be learned
   rather than designed.

```{image} ../_static/screens/70-group-scenarios.png
:alt: Demo scenarios dialog, group-control group
:class: screenshot
:width: 760px
```
