# MATLAB Project Function Inventory

## Scope and method

This audit covers every current `*.m` file under the repository, including the maintenance helpers and tests added in this change. The machine-readable inventory is `results/tables/project_function_inventory.csv` and contains one row per file with its primary symbol, signature, configuration reads, repository-local calls, callers, experiment/demo usage, tests, status, and recommendation.

Static calls were matched only against primary MATLAB symbols present in the repository. Function handles, `feval`, callbacks, scripts, class methods, and local functions can evade this analysis. Consequently, absence of a static caller is an audit signal, not proof that a file is safe to delete.

Known false positive: the inventory reports `visualization_config -> legacy_visualization_config` because the canonical config comment names the adapter. Runtime source has no such call. The actual dependency is one-way: `legacy_visualization_config -> visualization_config`; therefore no recursion exists.

## Totals

| Measure | Count |
|---|---:|
| Current MATLAB files | 309 |
| Function definitions, including local test/helper functions | 1045 |
| Script entry files | 6 |
| Test files | 76 |
| Experiment files, including `experiments/real` | 41 |
| Visualization files, including the root compatibility visualizer | 33 |

## Layer inventory

| Layer | Files | Primary role |
|---|---:|---|
| config | 8 | Canonical defaults, compatibility adapter, effective resolution |
| pipeline | 6 | Main stage orchestration |
| environment | 7 | Scene construction, grids, task regions |
| docking | 6 | Candidate creation, evaluation, dock selection |
| manipulator | 6 | Geometric reachability and optional IK boundary |
| collision | 3 | Footprint collision and clearance |
| planning_strategy | 30 | Bottom-Up, Top-Down, Bidirectional and shared plan contracts |
| global_planner | 37 | Planner dispatch, algorithm wrappers, route stitching, sparse graph |
| replanning | 3 | P5 change impact and local repair |
| execution | 9 | Execution estimate and simulated verification |
| visualization plus root visualizer | 33 | Stage, experiment, paper, V1/V2/V3 drawing |
| experiments | 41 | Experiment/profile entry points only |
| demos | 3 | V1/V2/V3 user-facing demos |
| integration and calibration | 9 | Offline-to-real interface and measured-data loaders |
| utils | 31 | Shared stateless helpers |
| tests | 76 | MATLAB unit and smoke tests |

## Status interpretation

`ACTIVE_CORE`, `ACTIVE_EXPERIMENT`, `ACTIVE_VISUALIZATION`, and `ACTIVE_TEST` identify maintained paths. `LEGACY_COMPAT` identifies retained interface boundaries. Eight `UNKNOWN` rows have no direct static caller and are reviewed in `docs/deprecation_candidates.md`; none are deleted. Full per-file evidence is in the CSV rather than duplicated in Markdown.

## Inventory contract

The CSV columns are:

`Layer`, `File`, `Primary function`, `Type`, `Purpose`, `Inputs`, `Outputs`, `Main parameters/config fields read`, `Functions called`, `Called by`, `Experiment/demo usage`, `Test coverage`, `Similar/duplicate functions`, `Current status`, and `Recommended action`.

Regenerate the inventory after adding, moving, or deleting MATLAB source. Review all `UNKNOWN` results manually before making deprecation decisions.
