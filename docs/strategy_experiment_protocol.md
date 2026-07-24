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
- `results/figures/fig_strategy_route_comparison.png`
- `results/figures/fig_strategy_route_comparison.pdf`
- `results/figures/fig_strategy_tradeoff.png`
- `results/figures/fig_strategy_tradeoff.pdf`

## Metrics

The CSV reports completion, dock count, task path length, return path length,
total path length, turn count, planning time, candidate generation time,
strategy timing, estimated execution time, coverage repair count, and the
strategy objective.

## Interpretation

Do not infer that Top-Down or Bidirectional is better from a single run. If all
methods reach 100 percent verified completion, compare efficiency metrics. If
one method leaves tasks uncovered, report the lost coverage instead of hiding
it behind path or timing gains.
