# Troubleshooting

| Status | Meaning / inspect | Do not change first |
|---|---|---|
| CAFETERIA_REACHABILITY_BLOCKED | Formal reachability rejected tasks; inspect diagnostics/profile geometry | Do not enlarge reach radius blindly |
| CAFETERIA_CANDIDATE_COVERAGE_BLOCKED | Candidate library cannot cover tasks | Do not alter candidate generation casually |
| CAFETERIA_PATH_COLLISION_FAILURE | Route violates map/footprint checks | Do not delete collision checks |
| FOOTPRINT_LIMITED | Inflated footprint limits a point-feasible route | Inspect inflation, map resolution, waypoints |
| BLOCKED | Structured infeasible result; inspect plan.reason | Do not interpret as crash |
| Sparse graph disconnected | No finite route in sparse graph | Inspect sparse k and connectivity |
| No supporting candidate | No feasible docking candidate | Inspect reachability/candidate diagnostics |
| MATLAB heap crash / short-lived process | Resource pressure terminated MATLAB | Reduce rendering/diagnostics; use frozen-state smoke paths |

HORIZONTAL_RADIAL_REACH_LIMIT is a diagnostic constraint. Inspect geometry/profile assumptions before changing any radius.
