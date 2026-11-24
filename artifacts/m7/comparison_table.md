# M7 fixed-map measurements

Lengths describe returned polylines; clearance is sampled center-to-known-lethal-cell-area distance.
Planning time is the planner-server result duration, median of five measured runs after one warm-up.

| Query | Planner | Time (ms) | Length (m) | Center clearance (m) | Expanded |
|---|---|---:|---:|---:|---:|
| 0 | NavFnDijkstra | 0.353 | 3.335 | 0.620 | N/A |
| 0 | NavFnAStar | 0.312 | 3.360 | 0.608 | N/A |
| 0 | AStar8 | 0.686 | 3.270 | 0.617 | 977 |
| 0 | AStar4 | 0.562 | 4.032 | 0.617 | 910 |
| 0 | AStarUnweighted | 0.488 | 2.729 | 0.334 | 349 |
| 0 | AStarHighWeight | 0.539 | 3.270 | 0.617 | 811 |
| 0 | AStarClearance | 0.682 | 3.129 | 0.571 | 614 |
| 1 | NavFnDijkstra | 0.451 | 5.140 | 0.753 | N/A |
| 1 | NavFnAStar | 0.187 | 5.136 | 0.743 | N/A |
| 1 | AStar8 | 0.458 | 5.211 | 0.617 | 298 |
| 1 | AStar4 | 0.409 | 7.232 | 0.617 | 778 |
| 1 | AStarUnweighted | 0.418 | 5.123 | 0.530 | 73 |
| 1 | AStarHighWeight | 0.475 | 5.211 | 0.617 | 292 |
| 1 | AStarClearance | 0.918 | 5.181 | 0.571 | 175 |
| 2 | NavFnDijkstra | 0.586 | 4.684 | 0.665 | N/A |
| 2 | NavFnAStar | 0.344 | 4.687 | 0.712 | N/A |
| 2 | AStar8 | 0.833 | 4.865 | 0.617 | 1027 |
| 2 | AStar4 | 0.347 | 5.832 | 0.725 | 117 |
| 2 | AStarUnweighted | 0.395 | 4.631 | 0.334 | 645 |
| 2 | AStarHighWeight | 0.673 | 4.865 | 0.617 | 911 |
| 2 | AStarClearance | 1.054 | 4.836 | 0.571 | 879 |
