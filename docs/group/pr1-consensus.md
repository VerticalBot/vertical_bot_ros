# ПР1 — Consensus, formation and connectivity (chapter 4, §16.8.3)

The first practicum builds the communication graph of a group, checks its algebraic connectivity, runs the
discrete consensus protocol, turns it into a formation controller, keeps the links alive during a rendezvous and
finally protects the protocol against a malicious neighbour with W-MSR. In the studio all five steps are lines of a
`consensus` document; the report holds the numbers of the course text and the charts of the report template.

```{image} ../_static/screens/63-pr1-formation.png
:alt: Formation report: trajectories of six robots assembling a circle and the error chart
:class: screenshot
:width: 900px
```

## Step 1 — Laplacian and λ₂

*Group › Course examples… › ПР1 · Chain P₄* adds

```text
consensus Chain of four robots (§4.3.4)
graph path n=4
x0 0 4 8 12
eps 0.25
steps 30
compare path ring star complete
continuous true
```

**Analyse.** The section *Communication graph* lists the degrees, the diameter, the vertex connectivity, the
Laplacian spectrum `0, 0.5858, 2, 3.4142`, λ₂ = 2 − √2 = 0.586 (the graph is connected — Fiedler's theorem), the
admissible step 1/Δmax = 0.5 and the optimal step 2/(λ₂ + λₙ) = 0.5. The graph view shows the chain. The topology
table reproduces table 4.1 for n = 4: path 0.586, ring 2, star 1, complete 4 — with the number of links, the
diameter, the convergence factor at ε = 0.9/Δmax and the r-robustness of each.

`graph` accepts `path | ring | star | complete`, `edges` (then `edge i j [w]` lines, 1-based) and `disk radius=`
(a geometric graph built from the `robot … at=x,y` positions). `robots n seed= box=` creates n robots at random
positions. The example of §4.1.5 — edges {1,2}, {2,3}, {2,4}, {3,4} — gives the spectrum 0, 1, 3, 4.

## Step 2 — the discrete protocol

The section *Discrete consensus* prints the first three iterations exactly as the text: `(1, 4, 8, 11)`,
`(1.75, 4.25, 7.75, 10.25)`, `(2.375, 4.5, 7.5, 9.625)`; the mean 6 is preserved at every step; ρ = max |1 − ελᵢ| =
0.854 gives ≈ 30 iterations for a 1 % disagreement, and the measured number is shown next to it. The chart plots
xᵢ(k). Try `eps 0.4` (task 2 of the chapter): the report warns that ε ≥ 1/Δmax loses the positive diagonal and the
iterations stop being monotone; `eps 0.6` makes ρ ≥ 1 and the verdict fails. `continuous true` adds the Euler
integration of ẋ = −Lx with the theoretical time ln(100)/λ₂ to 1 %.

## Step 3 — formation by offsets

*ПР1 · Formation of six robots on a ring*:

```text
consensus Formation of six robots
robots 6 seed=3 box=1
graph ring n=6
formation circle r=0.3 gain=1 steps=600 dt=0.05
fail 4 at=10
```

`formation <circle|line|wedge|grid> r= gain= steps= dt=` runs uᵢ = −k Σ aᵢⱼ((pᵢ − δᵢ) − (pⱼ − δⱼ)) in the group
simulator; the shape error (RMS deviation of ξᵢ = pᵢ − δᵢ from their mean) falls below 1 mm, the centre of the
formation is the mean of the initial ξ (not chosen in advance), and a `fail` line removes a robot at a given
time — the survivors keep the shape with the remaining links. The trajectories and the error chart are in the
report. `comm radius= drop=` replaces the static graph by a disk graph with packet loss.

**On the fleet.** Add six mobile robots (or *Load into the studio* the scenario *ПР1*), select the model and press
**▶ Run on fleet**: the robots move from where they stand into the circle; the status line shows the links and the
formation error, the run stops when the error is below 1 mm ({doc}`runtime`).

## Step 4 — rendezvous with connectivity maintenance

*ПР1 · Rendezvous on the edge of range* places five robots 0.95 R apart:

```text
consensus Rendezvous of a chain at the edge of the radio range
robot r1 at=0,0
robot r2 at=0.95,0
…
graph disk radius=1
rendezvous radius=1 gain=0.5 dt=0.002 steps=5000 vmax=0.5
```

The weights w(d) = (2R − d)/(R − d)² (Ji & Egerstedt) grow without bound as a link approaches its range; the
report checks every recorded step: no initial link is ever broken, the group collapses to a 10 cm spread — while
plain consensus over the same links breaks a link within the first steps.

## Step 5 — W-MSR against a malicious agent

*ПР1 · W-MSR against a malicious agent on K₇*:

```text
consensus W-MSR on a complete graph of seven
graph complete n=7
x0 0.1 0.4 0.2 0.9 0.5 0.3 0
eps 0.1
wmsr F=1 eps=0.1 steps=300 malicious=7 value=100 pattern=alternate
```

Agent 7 alternates ±100. Each normal agent drops up to F larger and F smaller neighbour values before averaging;
after 300 steps the normal agents agree to 10⁻³ and their mean lies inside [0, 0.9], whereas plain consensus is
hijacked to a mean above 50. The report computes the **r-robustness** of the graph by brute force (n ≤ 9): K₇ is
3-robust, which is exactly the (2F + 1) that W-MSR needs. *ПР1 · W-MSR fails on a sparse chain* runs the same
attack on P₇ (1-robust): the verdict is *issues found*, as the theory predicts — sparse chains and trees are
defenceless against a single liar.

## Beyond the practicum (chapters 4 and 13)

*ПР1 · Event-triggered consensus and cooperative transport* adds two lines:

- `event sigma=0.15 abs=0.01` — an agent broadcasts its state only when it deviated from the last broadcast value
  by more than 15 % (+0.01); the report counts the messages against the periodic protocol and shows the bounded
  residual disagreement (§13.5.2).
- `transport target=5,2 m=1 kp=4 kd=4` — three robots carry an object whose position is the mean of the grasp
  points with the PD law of §4.5.2; the report classifies the regime (k_d² vs 4 m k_p: overdamped / critical /
  oscillatory with ω), the settling time and the overshoot.

## Acceptance checklist of the practicum

| Course test | Studio |
|---|---|
| L of P₃ = [[1,−1,0],[−1,2,−1],[0,−1,1]]; rows sum to 0 | `tests/mrs_practicum.test.ts` › ПР1 |
| λ₂(K₆) = 6, λ₂(C₈) = 2 − 2cos(2π/8); λ₂ = 0 iff disconnected | same |
| 1/Δmax of a 5-star = 0.25; convergence to the mean; vector states; the λ₂ rate bound | same |
| formation error < 10⁻³, centre invariant | same and *ПР1 · Formation* |
| rendezvous keeps every initial link, spread < 0.1 | same and *ПР1 · Rendezvous* |
| W-MSR on K₇ with F = 1: spread < 10⁻³, mean inside the initial range; plain consensus > 50 | same and *ПР1 · W-MSR* |
