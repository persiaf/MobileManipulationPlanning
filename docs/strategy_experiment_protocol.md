# Strategy Experiment Protocol

## Single-Scene Comparison

Run:

```matlab
run('experiments/exp_compare_hierarchical_strategies.m')
```

The script builds the scene once, generates one candidate library, then runs
`bottom_up`, `top_down`, and `bidirectional` with the same global planner.

Outputs:

- `results/raw/strategy_comparison_records.mat`
- `results/tables/strategy_comparison_summary.csv`
- `results/tables/strategy_time_breakdown.csv`
- `results/tables/strategy_plan_differences.csv`
- `results/figures/fig_strategy_route_comparison.png`
- `results/figures/fig_strategy_route_comparison.pdf`
- `results/figures/fig_strategy_tradeoff.png`
- `results/figures/fig_strategy_tradeoff.pdf`
- `results/figures/fig_strategy_time_breakdown.png`
- `results/figures/fig_strategy_time_breakdown.pdf`
- `results/figures/fig_strategy_plan_differences.png`
- `results/figures/fig_strategy_plan_differences.pdf`

## Metrics

The CSV reports completion, dock count, task path length, return path length,
total path length, turn count, planning time, candidate generation time,
strategy timing, estimated execution time, coverage repair count, and the
strategy objective.

`strategy_time_breakdown.csv` decomposes estimated execution time into
navigation, docking, manipulation, arm transition, side switch, turning, lift,
and replanning components. The values are simulated/model-estimated, not
measured robot execution times.

`strategy_plan_differences.csv` compares each strategy against Bottom-Up. It
reports region-order Hamming distance, preferred-visit-order Hamming distance,
pose matches/differences, task-assignment differences, route-edge differences,
dock-count difference, path-length difference, and execution-time difference.
Visit-order and route-edge differences are computed from the final executed
visit order after normalization to local dock indices. Start and return edges
are reported separately. The legacy `route_edge_difference_count` is the
undirected dock-to-dock edge difference.

## Interpretation

Do not infer that Top-Down or Bidirectional is better from a single run. If all
methods reach 100 percent verified completion, compare efficiency metrics. If
one method leaves tasks uncovered, report the lost coverage instead of hiding
it behind path or timing gains.

The current main-scene result source is
`results/tables/strategy_comparison_summary.csv`,
`results/tables/strategy_time_breakdown.csv`, and
`results/tables/strategy_plan_differences.csv` generated on 2026-07-24.
Bottom-Up and Bidirectional both verified 100 percent completion with 27 docks,
57.34 m task path length, 5.95 m return path length, and 63.29 m total path
length. Top-Down also verified 100 percent completion with 27 docks, but its
task path length increased to 63.67 m, total path length increased to 67.08 m,
and estimated mission time increased to 728.57 s. The time-breakdown table
attributes most of the increase to turning time, not to reduced task coverage
or extra docks.

Run random paired and hard-scenario strategy experiments before making broad
claims about superiority across scenario distributions. The single-scene result
is a diagnostic case, not a complete statistical conclusion.
