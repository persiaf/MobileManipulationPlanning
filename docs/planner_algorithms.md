# Global planner audit

All wrappers below are dynamically dispatched by `global_planner/run_path_planner.m`. Their source and shared `global_planner/common` dependencies are present, so they are retained.

| Algorithm | Dispatch support | Default benchmark | Main selectable | Source dependencies present | Treatment |
|---|---:|---:|---:|---:|---|
| Convex-corner A* | yes | optional | yes (default) | yes | retain |
| A* | yes | yes | yes | yes | retain |
| JPS | yes | yes | yes | yes | retain |
| GBFS | yes | yes | yes | yes | retain |
| Theta* | yes | yes | yes | yes | retain |
| Dijkstra | yes | extended | yes | yes | retain |
| Lazy Theta* | yes | extended | yes | yes | retain |
| D* | yes | extended | yes | yes | retain |
| D* Lite | yes | extended | yes | yes | retain |
| LPA* | yes | extended | yes | yes | retain |
| Voronoi | yes | extended | yes | yes | retain |
| Hybrid A* | yes | extended | yes | yes | retain |
| RRT | yes | stochastic | yes | yes | retain |
| RRT* | yes | stochastic | yes | yes | retain |
| RRT-Connect | yes | stochastic | yes | yes | retain |
| Informed RRT* | yes | stochastic | yes | yes | retain |
| ACO | yes | stochastic | yes | yes | retain |
| PSO | yes | stochastic | yes | yes | retain |
| GA | yes | stochastic | yes | yes | retain |

The default benchmark intentionally limits runtime. Extended and stochastic methods require explicit configuration and remain subject to the common post-validation and A* fallback path.
