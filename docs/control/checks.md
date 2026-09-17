# Catalogue of checks

What the analysis verifies for each model kind, how a failure is reported, and what the course recommends when
it fails. Every check produces a report section with a level (✅ ⚠️ ❌ ℹ️); the model verdict is *OK* only when no
❌ section exists.

## Discrete-event models (`des`)

| Check | Reported | When it fails |
|---|---|---|
| **Reachability / blocking** of the composed plant | reachable states and transitions; blocking states; **deadlocks** (no enabled event) with the event trace that reaches them; **livelocks** (strongly connected components without a marked state) with a trace and the cycle | add the missing completion / recovery events, mark the right states, or let a specification exclude the path |
| **Controllability** of each specification | states where the plant enables an uncontrollable event the specification forbids, with an example trace | weaken the specification (forbid the event only in a context), add a controllable means, or accept the supervisor's restriction |
| **Supervisor synthesis** (supremal controllable non-blocking sublanguage) | states of `H = G ‖ E`, states removed per iteration (controllability / blocking), the supervisor size, disabled controllable events per state count, the shortest marked run | *UNREALISABLE* when the result is empty: an uncontrollable event forbidden by a specification cannot be prevented, or every behaviour blocks — the course's E4 |
| **Modular supervisors and non-conflict** | size of each modular supervisor, size of their joint behaviour, non-conflicting or the blocking state of the conjunction | conflicting modular supervisors need a coordinator or a monolithic supervisor |
| **Observer / observability** (with `unobservable`) | observer states, ambiguous cells, observability of the decisions | add sensing (make the event observable) or make the decision conservative in the ambiguous cells |
| **Diagnosability** (with `faults`) | twin-plant search for an indeterminate cycle; the ambiguous cycle | add a sensor or a periodic check (a runtime monitor) so that the fault is eventually detected |
| **LTL / CTL** `check` lines | holds, or a counterexample (LTL: prefix + cycle of plant states; CTL: a path to a violating state or an explanation) | inspect the trace: the atoms are `Component='state'`, `marked`, `deadlock` |

## Petri nets (`petri`, `s3pr`)

| Check | Reported | When it fails |
|---|---|---|
| **Invariants** | P-invariants (place weights), T-invariants; coverage → structural boundedness | an uncovered place may be unbounded (a buffer without a capacity) |
| **Boundedness** | bound per place from the reachability graph (with an exploration limit) | add capacity places / inhibitor arcs |
| **Liveness** | dead transitions, non-live transitions | the deadlock analysis below explains the cause |
| **Reversibility** | whether M₀ is reachable from every marking; terminal components | a process that cannot return to idle |
| **Deadlocks** | markings without enabled transitions with the firing sequence | monitors, banker, resource ordering |
| **Siphons and traps** | minimal siphons, minimal traps, Commoner condition per siphon, the siphon that **empties** and the sequence | the emptying siphon is the deadlock cause |
| **GMEC monitors** | for each bad siphon a monitor place `lᵀM ≤ β` (`C_V = −lᵀC`, `M₀(V) = β − lᵀM₀`), iterated until the controlled net is live | when monitors cannot restore liveness within the budget, the report suggests the banker's algorithm or a resource ordering |
| **Timed simulation** | throughput per transition, busy fraction per resource place, deadlock time | — |
| **GSPN steady state** (with `rate=`) | tangible / vanishing markings, π, throughput, P(place marked) | — |
| **LTL / CTL** `check` lines | as above; atoms are places and marking arithmetic, `deadlock` | — |

## Behavior trees and statecharts (`bt`, `statechart`)

| Check | Reported | When it fails |
|---|---|---|
| **Structure** | unknown bindings, leaves with children, composite nodes without children, retries without memory, actions without `timeout` (a liveness obligation that cannot be monitored), pre / post contracts that reference unknown keys | fix the tree; add timeouts and contracts |
| **Finite-time success** | Monte Carlo estimate of P(success / failure / timeout) and the mean ticks from the leaf models | raise retries, change priorities, model the leaves better |
| **Abstraction + LTL / CTL** | Kripke structure over the non-deterministic leaf outcomes (restricted by `outcomes`), property results with counterexamples over `tick:` / `ok:` / `fail:` / `run:` / `tree:` atoms | reorder priority layers, add a halt handler, restrict outcomes that the world cannot produce |
| **Monitors** | each `monitor` compiled to an LTL₃ automaton (atoms, states) | attached at runtime; `⊥` stops nothing by itself — the tree decides |
| **Statechart flattening** | states, composite states, reachable configurations, flat transitions; LTL / CTL on the flattened automaton | — |

## Verification models (`smv`, `gr1`, `stl`, `hybrid`)

| Check | Reported | When it fails |
|---|---|---|
| **LTL with fairness** (`smv`) | holds, or the shortest lasso counterexample found in the product with the Büchi automaton (prefix then cycle, all variables per state) | the cycle shows who starves or livelocks; add alternation / fairness in the protocol, not only in the assumptions |
| **CTL** (`smv`) | satisfying states, a path witness for violations | `AG EF` (recoverability) vs `AF` (inevitability) — check both |
| **GR(1) realisability** | game states, winning region, controller size (graph view ≤ 60 states) or the **counter-strategy** and a diagnosis (which assumption / guarantee combination is lost) | add the missing assumption, weaken a guarantee, or give the system more moves |
| **STL robustness** | ρ per formula on the signal; falsification result (parameters, ρ, budget used) | ρ < 0 with the violating time and parameters; ρ ≈ 0 means no margin |
| **Mode automaton** | chattering pairs (opposite guards without hysteresis), hysteresis width, missing dwell times, dwell-time bound τ_d = ln μ / λ | add hysteresis / dwell time; slow down switching below the bound |

## Planning and decisions (`pddl`, `stn`, `mdp`, `pomdp`)

| Check | Reported | When it fails |
|---|---|---|
| **Grounding** | schemas, predicates, objects, ground actions, atoms | huge numbers → add types, static predicates |
| **Plan search** | plan with per-action costs, expansions, time, validation | *no plan* with the last reached layer of the relaxed planning graph — a missing connection or an unreachable goal atom |
| **HTN** | decomposition tree, primitive plan | *no decomposition*: no applicable method for a task |
| **STN / STNU** | consistency (negative cycle otherwise), earliest / latest times, dynamic controllability (Morris) | tighten or relax windows; a contingent link cannot be constrained |
| **MDP** | V*, π*, Q values, sweeps | — |
| **POMDP** | lookahead values per action, belief update, α-vectors, decision threshold β | — |

## Coordination and scheduling (`mrta`, `mapf`, `jobshop`)

| Check | Reported | When it fails |
|---|---|---|
| **Assignment** | Hungarian optimum, greedy result and gap, minimax assignment, auction / CBBA allocation and convergence | — |
| **MAPF** | priority planning success per agent, CBS plan (sum of costs, makespan, CT nodes), TPG acyclicity and delayed execution collisions, **critical sections** | priority planning incomplete → CBS; critical sections → evacuation procedures |
| **Job shop** | lower bounds (machine load, job length, heads + tails, assignment), rule table (makespan, ΣC, Lmax, tardy jobs), branch and bound (optimal or best found within the node limit), critical path, robustness | — |

## Engineering evidence (`realtime`, `reliability`, `fta`, `fmea`, `acceptance`)

| Check | Reported | When it fails |
|---|---|---|
| **Rate-monotonic / response-time analysis** | U, Liu–Layland bound, response times with blocking under PCP / PIP, deadlines met | move a heavy task to another core, shorten periods, split tasks |
| **EDF** | schedulability (U ≤ 1 for implicit deadlines) | — |
| **End-to-end latency** | Σ(T+R) asynchronous vs ΣR synchronous, the heaviest stage, the position error at the given speed against the tolerance, the admissible speed | — |
| **Reliability** | λ_sys, MTTF, availability, per-component share | redundancy for the largest shares |
| **ISO/TS 15066 SSM** | S_p decomposition, admissible speed for the available distance | a SLOW mode with hysteresis |
| **ISO 13849 PL** | PLr from the risk graph, achieved PL from category / MTTFd / DC / CCF, meets or not | category 4 architecture, higher DC — only certified components count |
| **FDIR threshold** | per-check false-alarm probability, σ threshold or confirmation count | — |
| **FTA** | minimal cut sets (MOCUS) with order and probability, P(top) (rare event and exact), sensitivity per basic event | attack the dominant cut set |
| **FMEA** | RPN and action priority per row | — |
| **Acceptance** | Clopper–Pearson lower bound vs target, rule of three, trials needed, one-sided bounds on samples, sim-to-real bias / spread / Welch t, pairwise array, traceability with unformalised or open requirements | more trials, correction factors, formalise the requirement |
