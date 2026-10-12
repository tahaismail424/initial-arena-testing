| template | planner | episodes | success | time (s, median) | robot-person contacts | min clearance (m) | min wall/panel clear. (m) | freeze (s, median) | ped. inside wall (samples) | ped. overlap share | ped. overlaps >= 2 s | stuck peds | speed / desired | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| blind_corner | dsrnn | 3 | 2/3 | 23.8 | 0 | 0.15 | -0.05 | 0.0 | 0 | 1% | 0 | 0 | 1.0 | 0.99 |
| blind_corner | dwb | 3 | 3/3 | 85.3 | 0 | 0.25 | -0.0 | 0.0 | 0 | 1% | 0 | 0 | 1.0 | 0.99 |
| corridor_counterflow | dsrnn | 3 | 0/3 | 140.3 | 0 | 0.46 | -0.19 | 132.6 | 0 | 4% | 2 | 0 | 0.98 | 0.99 |
| corridor_counterflow | dwb | 3 | 3/3 | 105.8 | 0 | 0.27 | 0.22 | 0.0 | 0 | 8% | 9 | 0 | 0.96 | 0.99 |
| door_exit | dsrnn | 3 | 0/3 | 150.4 | 0 | 0.89 | -0.2 | 142.8 | 0 | 6% | 12 | 0 | 0.99 | 0.99 |
| door_exit | dwb | 3 | 3/3 | 77.4 | 0 | 0.26 | 0.04 | 0.0 | 0 | 7% | 13 | 0 | 1.01 | 0.91 |
| gauntlet | dsrnn | 2 | 0/2 | 300.6 | 0 | 0.37 | -0.17 | 269.1 | 0 | 4% | 15 | 0 | 1.49 | 0.67 |
| gauntlet | dwb | 2 | 2/2 | 316.7 | 0 | 0.25 | 0.01 | 0.0 | 0 | 4% | 4 | 0 | 1.5 | 0.66 |
| intersection | dsrnn | 3 | 2/3 | 25.7 | 0 | 0.28 | -0.12 | 0.0 | 0 | 0% | 0 | 0 | 0.99 | 0.97 |
| intersection | dwb | 3 | 3/3 | 66.4 | 0 | 0.39 | 1.61 | 0.0 | 0 | 1% | 0 | 0 | 1.0 | 0.98 |
| plaza_dense | dsrnn | 3 | 3/3 | 26.5 | 0 | 0.34 | 0.66 | 0.0 | 0 | 4% | 0 | 0 | 1.22 | 0.74 |
| plaza_dense | dwb | 3 | 3/3 | 76.4 | 1 | -0.14 | 3.8 | 0.0 | 0 | 4% | 0 | 0 | 1.39 | 0.69 |
