# ПР5 — Coverage of an area and distributed estimation (chapters 4, 11)

Two kinds serve the fifth practicum: `coverage` (Voronoi partition, masses and centroids, the coverage functional,
Lloyd's step and its limited-range decentralised variant) and `estimation` (consensus of measurements in the
information form and covariance intersection).

```{image} ../_static/screens/68-pr5-coverage.png
:alt: Coverage report: robots moving to the centroids around a hot spot, the cost chart and the Voronoi labels
:class: screenshot
:width: 900px
```

## Steps 1–2 — Voronoi, centroids, Lloyd

```text
coverage Hot spot at (3, 1)
area 0 4 0 4 res=0.1
density gaussian center=3,1 sigma=0.8 base=0.05
robots 6 seed=1 spawn=0,0.8
iterations 60
```

The area is discretised into cells of side `res`; the density is a background `base` plus a Gaussian hot spot
(or `density uniform`). Every cell is labelled with its nearest robot, the masses Mᵢ = Σφ dA and centroids Cᵢ of
the cells are computed, and the **coverage functional** H(p) = Σ φ(q) minᵢ ‖q − pᵢ‖² dA is evaluated. Lloyd's step
pᵢ⁺ = pᵢ + k(Cᵢ − pᵢ) never increases H (Cortés, Martínez, Karataş, Bullo 2004): the chart of H per iteration is
monotone, the trajectories show the robots leaving the corner and gathering around the hot spot, and the final
Voronoi labels are drawn as a grid. The verdict requires a monotone decrease of more than 10 %.

## Step 3 — limited sensing range

```text
coverage Uniform area, limited sensing
area 0 5 0 5 res=0.1
density uniform
robots 8 seed=2 spawn=2,3
iterations 80
range 0.8
```

With `range r`, a robot's cell is its Voronoi cell cut at distance r: it needs only neighbours closer than 2r, so
the algorithm is decentralised; a robot whose cell is empty stays put (the course test: a robot at (5, 5) does not
move). The limited cost penalises unreachable cells with r²; on a uniform area the eight robots that start bunched
together spread out (pairwise > 0.5 m) while the cost decreases.

**On the fleet.** **▶ Run on fleet** recomputes the centroids once per second on the station robots' positions
(in metres, the station frame) and drives the robots towards them at the speed limit; the status line shows H and
whether the robots have settled.

## Steps 4–5 — distributed estimation

```text
estimation Target position by eight robots
target 2,-1
robots 8 seed=3 radius=0.55
noise 0.01 1.0
iterations 300
ci a=0,0 A=1,4 b=1,1 B=4,1
```

Eight robots measure the target with different accuracies (variances drawn in `[0.01, 1]`) and communicate on a
random geometric graph (`radius`) with Metropolis weights. Each robot averages the **information vector**
yᵢ = Rᵢ⁻¹zᵢ and the **information matrix** Sᵢ = Rᵢ⁻¹ with its neighbours; because the averages of y and S are
preserved, every estimate Sᵢ⁻¹yᵢ converges to the centralised weighted least-squares estimate (max deviation
< 10⁻⁶ after 300 iterations in the chart, log scale). Plain averaging of the measurements ignores the accuracies
and ends farther from the target — both errors are in the report. The `ci` line runs **covariance intersection**
of two estimates with unknown correlation: ω is chosen on a grid to minimise trace P; for the course values
ω = 0.5 and x = (0.2, 0.8), and P is never smaller than the naive independent fusion.

## Acceptance checklist

| Course test | Studio |
|---|---|
| two robots split the square, masses 0.5 / 0.5, centroids (0.25, 0.5), (0.75, 0.5); empty cell keeps the robot | `tests/mrs_practicum.test.ts` › ПР5 |
| cost value 0.1·(1 + 0.5·4); Lloyd monotone, < 30 % of the initial cost, robots near the hot spot | same and *ПР5 · Lloyd* |
| far robot waits; limited cost decreases, robots spread > 0.5 | same and *ПР5 · Limited-range* |
| information consensus → centralised estimate to 10⁻⁶; CI ω = 0.5, consistent, picks the better estimate | same and *ПР5 · Information consensus* |
