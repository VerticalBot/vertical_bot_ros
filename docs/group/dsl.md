# Group-control DSL reference

One document = one kind; the first line is `<kind> <name>`. Lines are `keyword words key=value …`; `#` starts a
comment; numbers, `true` / `false` and quoted strings are recognised; a point is `x,y`, several points are separated
by `;`. Errors are reported with the line number. The templates (*New ▾ › Group control*) contain every line as a
comment.

## `consensus` (ПР1)

| Line | Meaning |
|---|---|
| `graph path|ring|star|complete n=` | topology; `graph edges` + `edge i j [w]` lines (1-based); `graph disk radius=` from the robot positions |
| `robot <name> [at=x,y]`, `robots n [seed= box=]` | robots (positions for disk graphs, formations, rendezvous; random in the box otherwise) |
| `x0 v₁ v₂ …`, `eps ε`, `steps k`, `continuous true` | initial states, discrete step (default 0.9/Δmax), iterations, Euler run of ẋ = −Lx |
| `compare path ring …` | topology table for the same n |
| `formation circle|line|wedge|grid r= gain= steps= dt=` | formation by offsets in the group simulator |
| `rendezvous radius= gain= dt= steps= vmax=` | connectivity-preserving rendezvous over the initial disk graph |
| `wmsr F= eps= steps= malicious=i,… value= pattern=alternate|constant|ramp` | resilient consensus (+ r-robustness for n ≤ 9) |
| `event sigma= abs=` | event-triggered consensus, message count |
| `transport target=x,y m= kp= kd=` | cooperative transport with a PD law |
| `comm radius= drop=`, `fail i at=`, `seed n` | radio model of the simulation, robot failures, random seed |

## `swarm` (ПР2)

One `model` line: `boids n= steps= dt= vmin= vmax= r_sep= r_view= w_sep= w_ali= w_coh= seed=` ·
`vicsek n= box= radius= speed= steps= eta=η₁,η₂,… seed=` · `pso source=x,y sigma= n= steps= seed= range=a,b k=`
with `robots=true vmax= d_min= noise= forget=` for the robot version · `aco n= ants= iterations= alpha= beta=
rho= seed=` (+ `point x,y` lines) · `firefly | gwo | bee`. Extra lines: `distractor at=x,y sigma= amp=`, `robot
<name>` (names for the fleet run).

## `allocation` (ПР3)

`robot <name> at=x,y`, `task <name> at=x,y [reward= value=]` or `random robots= tasks= area= seed=`; `methods
greedy hungarian ssi cbba vickrey`; `cbba capacity= lambda= graph=complete|path|ring|disk radius= speed=`.

## `gridmapf` (ПР4)

`map [cell=m]` … `end` block (`#` shelf, `.` free — rows may start with `#`); `agent <name> start=r,c goal=r,c` or
`random agents= seed=`; `methods prioritized cbs`; `execute delay= runs= seed=`.

## `coverage` and `estimation` (ПР5)

`coverage`: `area xmin xmax ymin ymax res=`, `density gaussian center=x,y sigma= base=` or `density uniform`,
`robots n seed= spawn=a,b` or `robot <name> at=x,y`, `iterations k`, `range r`, `gain k`.
`estimation`: `target x,y`, `robots n seed= radius= graph=disk|ring|path|complete`, `noise vmin vmax`,
`iterations k`, `ci a=x,y A=σ₁²,σ₂² b=x,y B=σ₁²,σ₂²`.

## `safety` (ПР6)

`scenario antipodal n= radius= | crossing | custom`; `robot <name> at=x,y goal=x,y [speed=]`; `params d_safe=
gamma= vmax= dt= steps= sense= gain= compare=`; `unstuck angle=deg hold=steps`; `stale lag=steps vmax_j=`;
`uncooperative i j …` (1-based).

## `warehouse` (homework)

`cell m`; `map` … `end`; `station NAME r,c kind=pickup|drop|home`; `robots r1,r2,… homes=H1,… speed= handle=
radio= drop= latency=`; `orders rate= reward= first= pickups= drops=`; `order id P D [reward= at=]`; `traffic
settle= stale= wait= lookahead= protective=`; `cbba capacity= discount= period= stale= commit= lost=`; `fault
<robot> at= kind=stop|mute|recover`; `duration s dt= seed= idle_home=`. Without `map` / `station` lines the
homework warehouse is used.

## `game`, `marl`, `evo`, `ca`, `fuzzy`, `resilience` (chapters)

- `game`: `players A B`; `strategies r₁ r₂ … ; c₁ c₂ …`; `payoff rᵢ cⱼ u₁ u₂`; `start rᵢ cⱼ`; `rounds n`;
  `replicator p₁ p₂`; `coalition A=v B=v AB=v …` (single-letter players).
- `marl`: `map` … `end` (`S` start, `G` goal, `#` wall; one row = a corridor with left / right moves); `params
  alpha= gamma= epsilon= episodes= step= goal= seed= [path=right] [method=sarsa]`; `agents 2 starts=r,c;r,c
  goals=r,c;r,c penalty= episodes=`.
- `evo`: `method ga bits= pop= generations= pc= pm= elitism= seed= initial=b₁,b₂,… fitness="expr in x"`; `method
  de|es benchmark=sphere|rastrigin|rosenbrock dim= pop= generations= seed= [x0=…]`; `method es
  objective=consensus graph= n= generations=`.
- `ca`: `wolfram rule= width= steps=`; `life steps= size=` + pattern block … `end`; `pheromone size= steps= rho=
  kappa= [path=r,c;r,c;…]`.
- `fuzzy`: `input dL= dF= dR=`; `obstacles x,y x,y … [goal=x,y]`; `goal x,y`; `start x,y`; `rule dL dF dR -> v
  omega` with terms `near | medium | far` → `stop | small | medium | large`, `hardRight | right | straight | left |
  hardLeft`.
- `resilience`: `agent NAME value [A=v B=v …]` (options = the lies told to each recipient); `trust graph= n=
  x0=… liar=i eps= steps= value=`; `degrade graph= n= order=i,j,…`; `switched A1=a,b,c,d A2=… tau=min,max`.
