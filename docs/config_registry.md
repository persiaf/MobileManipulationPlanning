# Configuration Registry

The full 234-row field registry is `results/tables/config_registry.csv`. Canonical defaults remain in `scene_config`, `robot_config`, `planner_config`, `visualization_config`, and `real_platform_config`. `resolve_effective_config` adds an optional incremental resolution layer without changing those APIs.

## Effective order

```text
canonical base configs
  -> named scene profile (one fixed robot_config)
  -> profile_override
  -> experiment_override
  -> user/demo override
```

Later layers win recursively for scene/planner settings. Robot parameters must retain robot_config values; validate_main_robot_config rejects conflicting overrides. Formal experiment files remain authoritative for validated planner values. `print_effective_config` reports the resolved state at runtime.

## Configuration conflicts and provenance

| Parameter | Definitions | Default | Override locations | Effective rule | Recommendation |
|---|---|---|---|---|---|
| map size | `scene_config`, C/L/V profiles | 10 x 10 m | C0-C3 remain 10 x 10; L1 12 x 12; L2/V1/V3 15 x 15; L3/V2 20 x 20 | Selected scene profile wins | These are profiles, not duplicate bugs |
| table count | base positions, layout profiles | 16 | L1=20, L2/V1/V3=28, L3/V2=40 | `table_mode=layout` uses `layout.num_tables` | Keep both base and profile values |
| seed | `scene_config.random_seed` | 2026 | profile/experiment/demo argument | Explicit caller seed wins | Always print effective seed |
| strategy | `planner_config.strategy.name` | bottom_up | strategy experiments | Explicit experiment override wins | Keep baseline default |
| global planner | `planner_config.global_planner` | convex_astar | tests/experiments may set A* | Explicit override wins | Comment now matches code |
| routing mode | planner base and P3/P4/P5 overrides | sparse_knn | validation routes and demos | Current base unless explicit experiment value | Do not confuse defaults with historical validation profiles |
| sparse k | planner base | 6 | formal P3/P4/P5 use 4 | Experiment/demo override wins | Keep k=4 as `VALIDATED_EXPERIMENT_OVERRIDE` |
| inflation | `planner_config.inflation_radius_m` | 0.25 m | no formal semantic change | Planner config | Do not merge with scene safety margin |
| augmentation enable | planner base | false | augmentation experiments | Explicit experiment override | `DEFAULT_BASELINE=false` |
| augmentation variant | planner base | joint | formal coverage work uses position_only; earlier strategy experiment uses joint | Only relevant when enabled | Preserve historical matrices |
| candidate prefilter | planner base | false | P2 enables selectively; P3/P4/P5 explicitly false | Explicit experiment override | P2 optimization remains rejected baseline |
| ordinary path-cost cache | `strategy.common.use_path_cost_cache` | true | Top-Down region queries | Local struct cache | Do not merge semantics with shared lazy cache |
| shared lazy cache | `strategy.bidirectional.use_shared_path_cost_cache` | false | cache/P3/P4 experiments | Bidirectional run-local handle context | Retain separate switch |
| return to start | execution and benchmark fields | execution=true; benchmark=false | tests/experiments | Consumer-specific field wins | Align these explicitly before comparing mission and benchmark lengths |
| visualization | formerly duplicated in planner and visualization configs | `visualization_config` | legacy adapter only | canonical config -> adapter | Do not add a third option set |
| real platform | `real_platform_config.enabled` | false | interface experiments only | interface config | Never merge into robot model |
| lift | `real_platform_config.lift.enabled` | false | explicit real-platform request only | interface config | Remains disabled |

## Visualization source

`visualization_config()` is the only definition site. `legacy_visualization_config()` exposes its `legacy` block in the flat shape required by old stage functions. `planner_config().visualization` is a compatibility view, not a second canonical configuration.

## Reachability boundary

| Field family | Role | Semantics |
|---|---|---|
| `robot.single_arm.reach_radius`, `reach_forward`, `reach_backward`, `reach_lateral`, `reach_z_min/max` | hard geometric limits | Reject physically/geometrically outside candidates |
| `planner.reachability.threshold` | score acceptance threshold | Converts continuous reachability score to accepted coverage |
| `robot_config()` | sole maintained robot model | Same parameters as the main entry point |
| `robot_geometry_profile('legacy_validated')` | compatibility name | Returns robot_config() without overrides |

Robot hard limits and the planner score threshold have separate meanings. No robot or reachability-bound value changed during the 2026-10-27 unification. The alternate physical geometry and its scenes were removed; see main_robot_unification.md.

## Cache boundary

`initialize_path_cost_cache/get_cached_path_cost` is a value-struct cost-only cache used by ordinary strategy computations. `SharedPathCostContext/query_path_with_shared_cache` is a directed, scene-validated, path-plus-cost handle cache shared by Bidirectional refinement and stage 3. Their keys and statistics look similar but their lifetime and semantics differ, so this pass documents rather than rewrites them.
