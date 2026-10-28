# Troubleshooting

| Status | Meaning / inspect | Do not change first |
|---|---|---|
| validate_main_robot_config:ConflictingParameter | A robot override or frozen state differs from robot_config.m | Use the main-scene robot parameters; recreate the incompatible state |
| FOOTPRINT_LIMITED | Inflated footprint limits a point-feasible route | Inspect inflation, map resolution, waypoints |
| BLOCKED | Structured infeasible result; inspect plan.reason | Do not interpret as crash |
| Sparse graph disconnected | No finite route in sparse graph | Inspect sparse k and connectivity |
| No supporting candidate | No feasible docking candidate | Inspect reachability/candidate diagnostics |
| MATLAB heap crash / short-lived process | Resource pressure terminated MATLAB | Reduce rendering/diagnostics; use frozen-state smoke paths |

HORIZONTAL_RADIAL_REACH_LIMIT is a diagnostic constraint. Inspect geometry/profile assumptions before changing any radius.
