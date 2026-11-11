# Main-style layout and task-distribution comparison

All maps use the actual exported geometry and ordered path samples. They reuse the maintained gray-table / green-task / blue-path / orange-dock / red-start visual encoding. Previous experiment plot scripts are unchanged.

Before outcomes exist, plot the actual frozen inputs only:

```sh
python visualization/task_navigation_robustness/plot_frozen_inputs.py
```

This command reads `scenario_manifest.csv`, immutable `case_definition.json:scene_cfg` and the actual `tasks.csv`. It creates `input_figures/frozen_inputs_overview.{pdf,png}` and an input-only manifest. It does not read, create or fill outcome rows, and the input map cannot establish success or improvement.

After the entire batch is `COMPLETE`:

```sh
python experiments/analyze_task_navigation_robustness.py
python visualization/task_navigation_robustness/run_all.py
```

Both accept an alternate data directory (`folder` for the analysis script; `--data-dir` for the figure script). The completed batch requires the top-level manifest and 108 raw summary rows, plus each case's complete four-method summary and metadata. Maps additionally read tasks, selected docks and actual paths. Top-level and case summaries are crosschecked.

The three primary methods map raw `minimum_seed` to `staged_minimum` (staged selection plus common minimum-cover repair), then `reorder_only` and `joint_local`. Raw `staged` remains auxiliary. Primary paired statistics require all three primary methods to complete the mission and have the same certified finite-pool minimum dock count; both comparator columns use the same scenario subset and denominator. Failures remain in all planned denominators and appear as N/A in cost plots. Win/tie/loss uses an absolute J difference of `1e-7`, not a relative percentage cutoff.

Outputs include paired J heatmaps, actual paired cost points, complete/minimum mission counts, all nine input maps, the predeclared `L0_D2` route comparison and, when different, a rule-selected worst eligible joint-vs-reorder case. The worst rule minimizes observed paired J reduction, allowing negative values; ties retain manifest/mode order. Overriding `--representative-case` is explicitly labelled and does not become a predeclared representative.

Figures are vector PDF and 300 dpi PNG, with captions and input/script/output SHA256 manifests. The heatmap and colorbar remain vector objects. Statistics are descriptive across deterministic factors; no artificial confidence intervals, universal guarantees, energy claims or physical-success claims are produced.
