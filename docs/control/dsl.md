# Model languages (DSL) reference

Every control model is a plain-text document. The first word names the kind (`des`, `petri`, `bt`, …), the rest
of the first line is the model name; comments start with `#` or `//`; indentation is significant only where a tree
is described (behavior trees, fault trees). Options are written `key=value`; a value with spaces is quoted
(`name="collect one"`). Numbers, `true` / `false` are converted, everything else stays a string. Syntax errors are
reported with the line number in the report.

```{image} ../_static/screens/52-new-model-dialog.png
:alt: New control model dialog
:class: screenshot
:width: 460px
```

Expressions (conditions, guards, invariants, LTL / CTL atoms) share one small language: `&&`/`&`, `||`/`|`, `!`,
`->`, comparisons `= == != < <= > >=`, arithmetic `+ - * /`, a ternary `c ? a : b`, functions `min max abs`,
identifiers with dots (`r1.state`), quoted symbolic values (`loc = 'base'`) and the prime suffix for the next value
(`loc'`). Temporal operators: LTL `G F X U R W`, CTL `AG EF AF EG AX EX A[φ U ψ] E[φ U ψ]`, STL bounded forms
`G[a,b]`, `F[a,b]`, `U[a,b]`.

## `des` — automata and supervisory control (chapters 2–3)

```text
des Two machines and a buffer
automaton M1                 # plant component
  events start1 finish1      # optional: alphabet (events that appear in transitions are added automatically)
  initial I
  marked I
  I -start1-> W
  W -finish1-> I
spec Buffer                  # specification automaton (keyword `spec` or `supervisor`)
  initial E
  marked E
  E -finish1-> F
  F -start2-> E
uncontrollable finish1 finish2
unobservable g_slip          # optional: partial observation → observer, observability check
faults g_slip                # optional: diagnosability of these events (twin plant)
check ltl "name" G(M1='W' -> F M1='I')
check ctl AG EF (M1='I' & M2='I')
```

- Several events on one arrow: `X -g_ok,g_put-> X`. A state without transitions can be declared with `states`.
- The plant is the parallel composition of all `automaton` blocks; specifications are composed with the plant
  (`H = G ‖ E₁ ‖ … ‖ Eₙ`) and the supremal controllable non-blocking sublanguage is computed.
- Atoms in `check` formulas: `Component = 'state'` (or `Component='state'`), `marked`, `deadlock`.

## `petri` — Petri nets (chapters 4–5)

```text
petri Two processes sharing a robot and a zone   horizon=300
place p1 tokens=1 kind=idle
place a1 kind=activity
place rM tokens=1 kind=resource "robot"           # kinds: idle | activity | resource (used by the monitor synthesis)
transition t1a delay=4                            # delay (deterministic) or rate (exponential, GSPN); `immediate`, priority=, weight=
arc p1, rM -> t1a -> a1                           # chains of arcs; weight=2; inhibitor arc: `p -o t`
check ctl AG !deadlock
```

Atoms for `check`: `p` (place marked), `p >= 2`, arithmetic over markings (`a1 + b1 <= 1`), `deadlock`.

## `s3pr` — resource-allocation system (chapter 4)

```text
s3pr Cell
resource M1 1                     # resource and its capacity
resource Z 1
process A jobs=1: take[M1] 4; zone[M1,Z] 5     # steps: name[resources held] duration
process B jobs=1: zone[Z] 5; robot[Z,M1] 4
```

The S³PR net is generated (idle, activity and resource places) and analysed exactly like a `petri` document.

## `perf` — performance bounds (chapter 5)

```text
perf Production cell saturate=S
resource M1 time=9 capacity=1     # busy time per part, servers
resource S time=22 capacity=1
op u1 duration=4 resources=M1     # operations with precedence for the max-plus cycle time
op u2 duration=5 resources=M1,S,Z after=u1
little throughput=0.0278 leadtime=90     # any two of wip / throughput / leadtime
```

## `bt` — behavior trees (chapters 6, 9)

```text
bt Mission
fallback root
  sequence emergency
    condition estop                              # condition: a blackboard key or a quoted expression
    action halt timeout=1
  sequence "collect one" memory                  # memory: resume from the running child
    action goto zone=table event=b_go_table arrive=b_arrive timeout=120
    retry 2
      sequence grasp memory
        action grasp event=g_close ok=g_ok miss=g_miss timeout=12 post="held"
outcomes halt=success,running                    # restrict leaf outcomes for the abstraction
model grasp p=0.8 ticks=3                        # leaf models for the finite-time success estimate
supervisor <des model name>                      # optional: which DES model gates the events at runtime
modes <hybrid model name>                        # optional: mode automaton at runtime
monitor G(held -> (held U placed))               # LTL₃ runtime monitors
check ltl "name" G("ok:estop" -> "tick:halt")    # properties of the abstraction
```

- Composite nodes: `sequence`, `fallback` (selector), `parallel N` (success threshold), `retry N`, `repeat N`,
  `timeout S`, `inverter`; add `memory` to sequences / fallbacks. Names: a quoted word or `name=`.
- Leaves: `action <binding> key=value …` and `condition <expression or key>`. `timeout=` (seconds), `pre=` /
  `post=` (skill contract expressions), `event=` (controllable event asked from the supervisor before the action
  starts), `done=` (event observed when the action succeeds), `arrive= / ok= / miss= / put=` (uncontrollable
  events the world reports), `emit=` for the `event` action.
- Runtime bindings for a station robot: `goto zone=<zone name>`, `halt`, `wait seconds=`, `dock`, `undock`,
  `reach`, `stow`, `grasp p=`, `place`, `set key=value`, `event emit=`, `report`, `select_target`,
  `refine_pose` ({doc}`runtime`).
- Atoms of `check` formulas on the abstraction: `"tick:<leaf>"` (the leaf was ticked), `"run:<leaf>"`,
  `"ok:<leaf>"`, `"fail:<leaf>"`, `"tree:success|failure|running"`.

## `statechart` — hierarchical state machines (chapter 6)

```text
statechart Operation modes
state Off initial
state On
state Auto parent=On initial history
state Idle parent=Auto region=ctl initial     # orthogonal regions
state Busy parent=Auto region=ctl
Off -power_on-> On
Idle -start [battery > 0.2]-> Busy / notify   # event [guard] / action
check ltl G(Fault -> F Auto)
```

## `hybrid` — mode automata (chapter 9)

```text
hybrid Speed modes
var dist_human=10 v=0 human=false estop=false ack=false
initial NORMAL
mode NORMAL invariant="v <= 1.2" vmax=1.2
mode SLOW invariant="v <= 0.3" vmax=0.3
mode STOP vmax=0
NORMAL -> SLOW when dist_human < 2.0 dwell=0.4     # guard, minimum dwell time, optional reset=...
SLOW -> NORMAL when dist_human > 2.5 dwell=0.4
NORMAL -> STOP when human || estop
STOP -> NORMAL when !human && !estop && ack
```

The analysis reports chattering pairs (guards that overlap without hysteresis), the hysteresis width and the
dwell time bound τ_d = ln μ / λ. At runtime `vmax` of the current mode limits the robot speed.

## `smv` — synchronous models for model checking (chapter 7)

```text
smv Shared zone with priority
var r1.state : {idle, waiting, in_zone, leaving}
var owner : {none, one, two}
var busy : boolean
define g1 := (owner = none) | (owner = one)
init r1.state := idle
next r1.state := case
  r1.state = idle : {idle, waiting};        # a set = non-deterministic choice
  r1.state = waiting & g1 : in_zone;
  TRUE : r1.state;
esac
fairness !(r1.state = in_zone)              # justice constraints (GF …) assumed for the LTL checks
ltlspec "mutual exclusion" G !(r1.state = in_zone & r2.state = in_zone)
ctlspec "recoverable" AG EF owner = none
```

## `gr1` — reactive synthesis (chapter 8)

```text
gr1 Patrol with a door
sys loc : a b                 # system variables and domains (`bool` for booleans)
env door : bool               # environment variables
env_init !door                # assumptions: initial, transition (may use primed variables), liveness
env_live door
sys_init loc = 'a'            # guarantees: initial, transition, liveness
sys_trans (loc = 'a' && loc' = 'b') -> door
sys_live loc = 'a'
sys_live loc = 'b'
```

Aliases: `assume_init / assume / assume_live` and `guarantee_init / guarantee / guarantee_live`.

## `pddl` — planning and HTN (chapters 10–11)

The document is ordinary PDDL (domain and problem in one file) plus directive lines outside the s-expressions:

```text
pddl search=gbfs heuristic=hff maxExpansions=20000    # search: gbfs | astar; heuristic: hff | hadd | hmax
(define (domain …) …)
(define (problem …) …)
method deliver(?o ?b) "M1 holding" : (holding ?o) => goto(?bl) place(?o ?b ?bl)   # HTN methods
method goto(?l) "already there" : (at-robot ?l) =>
task deliver(bolt1 bin-bolts)                                                    # HTN root tasks
```

Supported PDDL: `:typing` (type hierarchy), `:negative-preconditions`, `:universal-preconditions` and
`forall` effects, `exists`, `imply`, `:action-costs` (`increase (total-cost) …`, `:metric minimize`), durative
actions for the Gantt view.

## `stn` / `stnu` — temporal networks (chapter 10)

```text
stnu Delivery to a machine
z -> start [20,20]          # requirement link [min,max]
start => arrive [8,14]      # contingent link (duration chosen by nature)
arrive -> unload [0,100]
```

## `mdp` — Markov decision processes (chapter 12)

```text
mdp Grasp strategy gamma=1 minimize          # minimize = costs, otherwise rewards
terminal s2=0 s4=60                          # terminal states with their value
s0 refine 3 : s1 1.0                         # state action cost/reward : successor probability, …
s0 grasp 8 : s2 0.55, s3 0.45
```

## `pomdp`, `mrta`, `mapf`, `fmea`, `acceptance` — JSON documents

The first line is the kind and the name; the body is JSON. Templates (*New ▾*) show every field:

- `pomdp`: `states`, `actions`, `observations`, `terminalActions`, `transitions`, `observation[action][state][obs]`,
  `reward[state][action]`, `belief`, `horizon`, `gamma`.
- `mrta`: `cost` matrix with `robots` / `tasks` names; optional `auction` `{robots:[{id, at:{x,y}}], tasks:[…]}`.
- `mapf`: `nodes`, `edges`, `agents` `[{id, start, goal}]`, optional `delays` for the temporal-plan-graph
  execution.
- `fmea`: an array of rows `{element, failureMode, systemEffect, detection, S, O, D, measure}`.
- `acceptance`: `trials {n, failures, target}`, `samples {name, values, limit, kind}`, `simReal {sim, real}`,
  `pairwise {factor: [levels]}`, `requirements [{id, text, cls, formal, verification, result, components}]`.

## `jobshop` — scheduling (chapter 14)

```text
jobshop Cell with two machines
machines M1 S1 S2 M2
setup A -> B 3                                   # optional sequence-dependent setups (types)
job J1 type=A due=60 : M1 5 ; S1 22 | S2 26 ; M2 5    # operations separated by `;`, alternatives by `|`
```

## `realtime` — task sets and latency (chapter 15)

```text
realtime Onboard computer protocol=pcp speed=0.15 tolerance=0.015
task drive C=1.2 T=5                             # C execution time, T period, optional D deadline, cs=res:len,…
task perception C=30 T=200
chain camera:33/33 image:50/93.8 pose:50/12 mode=async name="visual servoing"   # stage:T/R, async | sync
```

## `fta` — fault trees (chapter 16)

```text
fta Contact with a human
and top "unacceptable contact"
  or undetected "human not detected in time"
    event A p=1e-5 "scanner failure"
    event B p=1e-2 "human outside the field of view"
  and moving "robot moves at a dangerous speed"
    event D p=1 "motion command"
```

## `reliability` — reliability and safety budget (chapter 16)

```text
reliability Production cell
component M1 lambda=2e-4          # or mttf=5000, redundancy=2
mttr 4
ssm v=1.2 tr=0.15 ts=0.35 zd=0.10 zr=0.05 c=0.20 decel=1.5 distance=1.0   # ISO/TS 15066 speed & separation
pl s=2 f=2 p=2 cat=3 mttfd=40 dc=0.95 ccf=70                             # ISO 13849-1 risk graph and achieved PL
fdir rate=100 fa=1                                                        # detection threshold for 1 false alarm/day
weibull eta=2000 beta=2.5 cp=1 cf=10                                       # optimal replacement interval
```

## `stl` — signal temporal logic (chapters 9, 17)

```text
stl Distance to a human
spec "R6" G[0,60] (human -> dist >= 0.8)
falsify dist0=[0.5,3] v=[0,1.5] budget=200     # optional: search a parameter space for the minimum robustness
signal t dist human                            # a trace: header then rows
0 5 false
1 4 false
```

## Programmatic use

The same parsers and analyses are available in code (`studio/src/ctl`): `analyse(kind, source)` returns the
structured report used by the tab, `parseDes` / `parseBt` / … the model objects, `ControlRuntime` runs a
behavior tree against `stationBindings` (a station robot) or `simulatedBindings` (a stand-in world for tests).
The headless demo scenarios (`npm run scenarios`) and the test suites are the reference for the numbers quoted in
the guides.
