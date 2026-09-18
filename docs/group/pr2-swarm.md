# ПР2 — Swarm behaviour and emergence (chapter 5, §1.1)

Four `swarm` documents cover the practicum: the Reynolds rules, the order parameter and the Vicsek phase
transition, PSO with a local topology, and the transition from particles to robots with bodies. One `model` line
per document selects the algorithm and its parameters.

```{image} ../_static/screens/64-pr2-swarm.png
:alt: Swarm report: flock trajectories and the order-parameter chart
:class: screenshot
:width: 900px
```

## Step 1 — Reynolds rules

```text
swarm Flock of twenty robots
model boids n=20 steps=800 dt=0.05 vmin=0.2 vmax=0.3 r_sep=0.15 r_view=0.6 w_sep=0.05 w_ali=5 w_coh=0.1 seed=4
```

Separation Σ(pᵢ − pⱼ)/‖pᵢ − pⱼ‖² over neighbours closer than `r_sep`, alignment v̄ − vᵢ and cohesion p̄ − pᵢ over
neighbours within `r_view`, weighted and integrated synchronously with the speed clamped to [vmin, vmax] (a flock has
no standing members — without the lower bound cohesion collapses the swarm into a motionless clump). The report
plots the **polarization** φ = ‖Σvᵢ‖ / Σ‖vᵢ‖ and the minimum pair distance over time and draws the trajectories;
the verdict requires φ > 0.9 and a final minimum distance above r_sep/2. Nobody knows the global heading: it emerges.

**On the fleet**: **▶ Run on fleet** with nine or more mobile robots turns the station into the flock (the run
continues until you stop it).

## Step 2 — order parameter and the Vicsek transition

```text
swarm Vicsek model on a torus
model vicsek n=100 box=5 radius=1 speed=0.03 steps=300 eta=0.3,1,2,3,4,5,6 seed=5
```

Every particle takes the mean heading of its neighbours within `radius` on the torus plus a uniform noise of width
η. The report tabulates and plots φ(η) averaged over the second half of each run: above 0.8 at η = 0.3 (an ordered
flock), below 0.3 at η = 6 (disorder) — the order–disorder transition that chapter 1 uses to introduce complex
systems. **Run on fleet** runs the first η on the station robots (positions wrapped on the torus).

## Step 3 — PSO with a ring (lbest) topology

```text
swarm PSO lbest on a Gaussian field
model pso source=0.7,-0.4 sigma=0.5 n=12 steps=150 seed=8 range=-2,2
```

vᵢ⁺ = w vᵢ + c₁r₁(pbestᵢ − xᵢ) + c₂r₂(lbestᵢ − xᵢ) with lbest taken over the ring neighbourhood ±k; the best value
never decreases (the swarm remembers), the maximum is found to 0.02. The 1-D example of §5.6.4 (ω = 0.5, c₁ = c₂ =
1.5, r = 1: x₁ = 4 → −6.5 → 9.25) is the textbook illustration of oscillation with too large coefficients; the
document uses the standard constriction values w = 0.72, c₁ = c₂ = 1.49.

## Step 4 — robots with bodies

```text
swarm Robots seek a gas source
model pso robots=true source=2,1.5 sigma=0.8 n=8 steps=400 seed=9 vmax=0.03 d_min=0.1 noise=0.005 range=0,0.5 forget=0.002
distractor at=0.8,1.6 sigma=0.3 amp=0.3
```

`robots=true` switches to the robot version: a step is limited to `vmax`, robots closer than `d_min` repel each
other (Σ gain·(d_min − d)·(xᵢ − xⱼ)/d), the field is measured at the real position with noise, and `forget`
slowly discounts the personal records so that a noise-inflated measurement does not hold a robot forever. A
`distractor` adds a false local maximum. The report shows the trajectories, the source and the distractor, the
true field value at the best record and the swarm centre's distance to the source (< 0.3 m in the course test).
**Run on fleet** performs one PSO step per second and moves the robots between the way-points.

## Chapter 5 beyond the practicum

- `model aco n=10 ants=10 iterations=60 alpha=1 beta=3 rho=0.3` — ant colony optimisation of a closed patrol
  route through `point x,y` lines (or a circle of n points): probabilities τ^α η^β, evaporation ρ, deposits Q/L;
  the tour is drawn and compared with the nearest-neighbour heuristic.
- `model firefly | gwo | bee` — the firefly, grey-wolf and bee-colony optimisers on the same Gaussian field, with
  the history of the best value.

## Acceptance checklist

| Course test | Studio |
|---|---|
| isolated robot → zero acceleration; components (−10, 0), (1, 0), (0.1, 0); flock φ > 0.9, min distance > 0.1 | `tests/mrs_practicum.test.ts` › ПР2 |
| polarization 1 / 0 / 0; torus neighbours average to π/4; φ > 0.8 (η = 0.3), < 0.3 (η = 6) | same and *ПР2 · Vicsek* |
| PSO velocity formula, vmax, maximum within 0.02 with a monotone best | same and *ПР2 · PSO* |
| repulsion values; robot swarm step ≤ 0.03, centre within 0.3 of the source | same and *ПР2 · Robot swarm* |
