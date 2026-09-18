# ПР3 — Task allocation in a group (chapter 10, §9.7)

An `allocation` document holds robots, tasks (or a random instance) and the list of methods to compare: the
one-task-per-robot assignments (greedy vs Hungarian), the sequential single-item auction with routes, CBBA with its
two phases, the second-price auction and the contract-net traffic.

```{image} ../_static/screens/65-pr3-allocation.png
:alt: Allocation report: SSI rounds table and CBBA routes
:class: screenshot
:width: 900px
```

## Step 1 — greedy vs optimal

```text
allocation Greedy pays for its first choice
robot r1 at=0.05,0
robot r2 at=2,0
task t1 at=1,0 reward=10
task t2 at=-3,0 reward=10
methods greedy hungarian ssi
```

Greedy takes the cheapest free pair first (r1 → t1 at 0.95) and then has to send r2 across the map (5.0): total
5.95; the Hungarian assignment (from the control-design `mrta` machinery) gives 4.05 — a ratio of 1.47. The course
matrix [[1, 2], [1.1, 10]] gives 11.0 vs 3.1 (3.5×); the studio also reports that ratio when you give the
matrix through positions with the same ordering of costs.

## Step 2 — sequential single-item auction (MiniSum)

*ПР3 · Nine tasks for three robots* draws a random instance (`random robots=3 tasks=9 area=10 seed=2`). In every
round each robot bids the increase of its route length at the **best insertion** position of every free task; the
smallest bid wins and the task is inserted. The report shows the rounds table (task, winner, bid), the routes, the
total length and makespan, the count of bids (3 × 45 = 135 for 9 tasks) and, for instances up to two robots and
seven tasks, the exhaustive optimum with the ratio (guaranteed ≤ 2 for MiniSum).

## Steps 3–4 — CBBA

```text
cbba capacity=3 lambda=0.95 graph=complete
```

Every agent builds its **bundle** by adding the task with the largest marginal gain of the discounted score
Σ Rⱼ λ^{tⱼ} (inserted at the best position) as long as the gain outbids the known winner and the bundle has room;
then the **consensus** phase takes, per task, the maximum bid among the agent and its neighbours (ties → the
smaller index; the winner's own word overrides a stale belief), and releases the bundle from the first task that was
lost, resetting the bids of the later tasks. The report gives the iterations until nothing changes, the messages
(iterations × links × 2), the conflict-freeness check (one owner per task and all agents agree), the assigned
tasks and the score, and draws the routes. `graph=path` (*ПР3 · CBBA over a line graph*) shows that agreement
still comes, only later; `graph=disk radius=` builds the graph from the robot positions.

## Auctions and protocols (chapter 9, chapter 6)

`methods … vickrey` adds a sequential **second-price auction**: utility = value − distance, the winner pays the
second-best bid, so truthful bidding is a dominant strategy; the report lists bids, prices and surpluses. The
**contract-net** section counts announce / bid / award messages for the same tasks and robots, next to the
message count of CBBA — the trade-off between a central auctioneer and a consensus auction.

## Acceptance checklist

| Course test | Studio |
|---|---|
| greedy {0:0, 1:1} = 11.0, optimal 3.1; rectangular greedy valid and ≥ optimal | `tests/mrs_practicum.test.ts` › ПР3 |
| best insertion (pos 1, +0); nine tasks once each, 9 rounds, 135 bids; SSI ≤ 2× the exhaustive optimum | same and *ПР3 · Nine tasks* |
| bundle [0, 1] with y₀ = 10·0.95; a better known bid is not outbid; conflict-free on complete and line graphs; release resets later tasks | same and *ПР3 · CBBA over a line graph* |
