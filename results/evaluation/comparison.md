# Navigation evaluation

Archived static-scene evidence; this command does not collect new navigation trials

## Closed-loop navigation

| Planner / repeat | Success | Recovery | Sim time / s | Actual distance / m | Max world goal error / m | Min sampled circle gap / m |
|---|---:|---:|---:|---:|---:|---:|
| astar / 1 | 4/4 | 0 | 106.06 | 12.39 | 0.130 | 0.259 |
| navfn / 1 | 4/4 | 0 | 106.82 | 12.49 | 0.139 | 0.268 |
| navfn / 2 | 4/4 | 0 | 105.08 | 12.41 | 0.147 | 0.237 |
| astar / 2 | 4/4 | 0 | 107.58 | 12.69 | 0.114 | 0.252 |

## Fixed-map planning

| Query | Planner | Requests | Median planning / ms | Median polyline / m | Sampled center gap / m |
|---|---|---:|---:|---:|---:|
| 0 | NavFnDijkstra | 5 | 0.353 | 3.335 | 0.620 |
| 0 | NavFnAStar | 5 | 0.312 | 3.360 | 0.608 |
| 0 | AStar8 | 5 | 0.686 | 3.270 | 0.617 |
| 0 | AStar4 | 5 | 0.562 | 4.032 | 0.617 |
| 0 | AStarUnweighted | 5 | 0.488 | 2.729 | 0.334 |
| 0 | AStarHighWeight | 5 | 0.539 | 3.270 | 0.617 |
| 0 | AStarClearance | 5 | 0.682 | 3.129 | 0.571 |
| 1 | NavFnDijkstra | 5 | 0.451 | 5.140 | 0.753 |
| 1 | NavFnAStar | 5 | 0.187 | 5.136 | 0.743 |
| 1 | AStar8 | 5 | 0.458 | 5.211 | 0.617 |
| 1 | AStar4 | 5 | 0.409 | 7.232 | 0.617 |
| 1 | AStarUnweighted | 5 | 0.418 | 5.123 | 0.530 |
| 1 | AStarHighWeight | 5 | 0.475 | 5.211 | 0.617 |
| 1 | AStarClearance | 5 | 0.918 | 5.181 | 0.571 |
| 2 | NavFnDijkstra | 5 | 0.586 | 4.684 | 0.665 |
| 2 | NavFnAStar | 5 | 0.344 | 4.687 | 0.712 |
| 2 | AStar8 | 5 | 0.833 | 4.865 | 0.617 |
| 2 | AStar4 | 5 | 0.347 | 5.832 | 0.725 |
| 2 | AStarUnweighted | 5 | 0.395 | 4.631 | 0.334 |
| 2 | AStarHighWeight | 5 | 0.673 | 4.865 | 0.617 |
| 2 | AStarClearance | 5 | 1.054 | 4.836 | 0.571 |

## Other acceptance

14 core tests; 105 measured planning requests; 2 invalid goals rejected; 10 gated navigation successes and 2 expected preflight refusals; 2 GUI navigation successes.

## Limits

- Two fresh simulations per planner; goals within a simulation are correlated
- Planner-server planning duration differs from action round-trip and navigation time
- Fixed-map center distance differs from actual robot circle clearance
- Goal-to-world endpoint error includes localization and navigation tolerance
- No statistical superiority, dynamic-obstacle, precision-docking or hardware claim
