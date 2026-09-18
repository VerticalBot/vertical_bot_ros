# Theory chapters as documents: games, learning, evolution, automata, fuzzy control, resilience

Beyond the practicum, the worked examples of the theory chapters are documents too. Each reproduces the numbers of
the text and adds the charts.

## Chapter 9 — game theory (`game`)

```text
game Task allocation between two robots (§9.3.4)
players R1 R2
strategies Z1 Z2 ; Z1 Z2
payoff Z1 Z1 4 4
payoff Z1 Z2 10 6
payoff Z2 Z1 6 10
payoff Z2 Z2 2 2
coalition A=10 B=20 C=30 AB=60 AC=70 BC=90 ABC=120
```

The report prints the bimatrix, the strictly dominated strategies, the pure equilibria ((Z1, Z2), (Z2, Z1)), the
mixed equilibrium of a 2 × 2 game (q = 0.8, expected payoff 5.2), the Pareto-optimal outcomes, best-response
dynamics from `start`, fictitious play frequencies over `rounds`, and whether the game is an exact potential game
(with Φ). `coalition` adds the cooperative part: superadditivity, the **Shapley value** (26.67, 41.67, 51.67) and
the **core** check with the blocking coalitions (the equal split is blocked by BC). `replicator p q` runs the
replicator dynamics of a symmetric game (*Chapter 9 · Prisoner's dilemma*: cooperation dies out).

```{image} ../_static/screens/73-ch9-game.png
:alt: Game report
:class: screenshot
:width: 820px
```

## Chapter 7 — reinforcement learning (`marl`)

```text
marl Corridor Q-learning (§7.2.4)
map
  S..G
end
params alpha=0.5 gamma=0.9 epsilon=0.1 episodes=300 step=-1 goal=10 seed=1 path=right
agents 2 starts=0,0;0,3 goals=0,3;0,0 penalty=-5 episodes=600
```

`path=right` (or the corridor `S..G` itself) reproduces **table 7.2**: Q(s₃, →) = 5, 7.5, 8.75 after three episodes
along the fixed path, and the optimum Q* = 6.2 / 8 / 10 from value iteration. The table of the book prints 4.03 for
Q(s₂) after episode 3; the update 1.5 + 0.5·(−1 + 0.9·7.5 − 1.5) gives 3.625, and the studio shows the computed
value. Then ε-greedy Q-learning (or `method=sarsa`) learns the grid for `episodes`, with the moving-average return,
the greedy path and Q(start) vs Q*. `agents 2 …` runs **independent Q-learning** of two robots that must not enter
the same cell (§7.4.2): collisions per episode at the start and at the end of learning, episode length, joint
return.

```{image} ../_static/screens/74-ch7-qlearning.png
:alt: Q-learning report with table 7.2
:class: screenshot
:width: 820px
```

## Chapter 12 — evolutionary tuning (`evo`)

```text
evo Genetic algorithm for x² (§12.2.6) and continuous tuning
method ga bits=5 pop=4 generations=30 pc=0.9 pm=0.05 elitism=true seed=4 initial=01101,11000,01000,10011 fitness="x*x"
method de benchmark=rastrigin dim=2 pop=20 generations=150 seed=5
method es objective=consensus graph=path n=6 generations=40
```

The GA section shows the roulette table of the initial population (169 / 576 / 64 / 361, p = 0.144 / 0.492 /
0.055 / 0.309, expected copies 0.58 / 1.97 / 0.22 / 1.23), the fitness per generation and the optimum x* = 31
(f = 961); `fitness` is any expression in `x`. `de` runs differential evolution and `es` a (μ, λ) evolution
strategy with a self-adaptive step on the benchmarks `sphere`, `rastrigin`, `rosenbrock`; `objective=consensus`
tunes the consensus step ε of a topology by minimising the convergence factor ρ and compares with the analytic
optimum 2/(λ₂ + λₙ) (§12.5: adaptation of a group parameter).

```{image} ../_static/screens/78-ch12-evo.png
:alt: Evolutionary tuning report
:class: screenshot
:width: 820px
```

## §4.6 and §5.5 — cellular automata and stigmergy (`ca`)

```text
ca Rule 90, a glider and a pheromone trail
wolfram rule=90 width=31 steps=15
life steps=12 size=10
  .#.
  ..#
  ###
end
pheromone size=12 steps=40 rho=0.05 kappa=0.1
```

Rule 90 gives the symmetric Sierpiński triangle, rule 30 an aperiodic pattern (the report checks the symmetry and
counts distinct rows); the glider of the Game of Life repeats itself after 4 steps shifted by (1, 1); the pheromone
field of a robot depositing along a path evaporates (ρ) and diffuses (κ) — the shared memory in the environment
that ant algorithms read.

```{image} ../_static/screens/77-ch4-ca.png
:alt: Cellular automata report
:class: screenshot
:width: 820px
```

## §13.4 — fuzzy obstacle avoidance (`fuzzy`)

```text
fuzzy Obstacle avoidance (§13.4.2)
input dL=2.0 dF=0.7 dR=1.6
obstacles 3,0 3,0.4 3,-0.4 goal=6,0
```

The Mamdani controller of table 13.2 over three range sectors: for the input of the text rule 5 (far, near, far)
fires at min(1, 0.4, 0.67) = 0.4 and rule 2 (far, medium, far) at 0.3; the centroid gives a speed between small and
medium (≈ 0.36 of the maximum) and a left turn — a smooth compromise instead of a switch. The report shows the
memberships, the rule table with strengths, the response surface v(d_F) and, with `obstacles` and `goal`, the
closed loop that blends avoidance with go-to-goal (path, minimum obstacle distance). `rule dL dF dR -> v omega`
lines replace the rule base.

```{image} ../_static/screens/75-ch13-fuzzy.png
:alt: Fuzzy controller report
:class: screenshot
:width: 820px
```

## Chapter 16 and §13.5 — resilience and switching (`resilience`)

```text
resilience Byzantine agents, trust, degradation, switching
agent A continue
agent B continue
agent C continue
agent D continue lies A=continue B=evacuate C=evacuate
trust graph=complete n=6 x0=0.1,0.4,0.2,0.9,0.5,0 liar=6 eps=0.1 steps=200
degrade graph=star n=5 order=1,2
switched A1=-0.1,1,-10,-0.1 A2=-0.1,10,-1,-0.1 tau=0.05,3
```

- **Byzantine agreement** (§16.8.1–16.8.2): the two-round oral-messages protocol; with n = 4 and one traitor the
  honest agents agree on "continue" (the evidence table shows what each heard about each); *Chapter 16 · Three
  agents* shows the tied evidence when n < 3f + 1. The bound and the message count of OM(f) are reported.
- **Trust / reputation** (§16.8.4): a neighbour whose value is an outlier against the local median loses trust
  and is not averaged; an alternating ±100 liar ends with zero trust and the honest agents agree inside their range.
- **Graceful degradation** (§16.5.3): robots are removed in `order` from a formation graph; the table gives the
  connectivity and λ₂ after each failure (a star dies with its hub).
- **Switched systems** (§13.5.1): two stable modes, a search for a common quadratic Lyapunov function, the growth
  of ‖x‖ per switching cycle against the dwell time τ — fast switching destabilises, the minimum dwell time is read
  from the sweep (why supervisors need hysteresis and minimum-stay timers).

```{image} ../_static/screens/76-ch16-resilience.png
:alt: Resilience report: switched system growth vs dwell time
:class: screenshot
:width: 820px
```
