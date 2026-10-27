# MobileManipulationPlanning Code Architecture

## 1 Project Overview

The repository implements offline MATLAB planning for mobile dual-arm manipulation: scene construction, candidate docking generation, hierarchical dock selection, global route planning, execution estimation, P3/P4 sparse routing, P5 local repair, visualization, and JSON exchange with an external real-platform stack. This maintenance pass changes configuration provenance and drawing structure only; P0-P5 planner semantics and historical results are unchanged.

The complete 309-file source table is `results/tables/project_function_inventory.csv`. The readable inventory methodology is in `docs/project_function_inventory.md`.

## 2 Directory Structure

| Layer | Function/File family | Purpose | Main inputs | Main outputs | Key parameters | Called by | Calls | Similar/duplicate | Status |
|---|---|---|---|---|---|---|---|---|---|
| root | `main_mobile_manipulation_planning.m` | default pipeline entry | configs | plan, figures, stats | runtime, benchmark | user | stages 1-5 | experiment entries | ACTIVE_CORE |
| config | five canonical configs, resolver/printer/adapter | defaults and provenance | profiles/overrides | effective structs | all config fields | all entries | profile builders | experiment overrides | ACTIVE_CORE |
| environment | scene/grid/task-region builders | construct scene | scene cfg | scene, grid, regions | map/layout/task fields | stage 1, experiments | coordinate/layout utils | profile builders | ACTIVE_CORE |
| docking/manipulator/collision | candidate feasibility | dock candidates and scores | scene, robot, planner | candidate library | reach/collision fields | stage 2 | geometric helpers | diagnostics/oracles | ACTIVE_CORE |
| planning_strategy | hierarchical selection | select docks/order | candidate library | unified plan | strategy/cache | stage 2, experiments | three strategies | shared wrappers | ACTIVE_CORE |
| global_planner | route planning | costs/order/waypoints | poses, map | path, sparse graph | planner/routing | stage 3 | algorithm wrappers | core vs wrappers | ACTIVE_CORE |
| replanning | P5 repair | local change handling | frozen state/event | repaired state/stats | dynamic event cfg | P5 | collision/sparse helpers | static replanning | ACTIVE_CORE |
| execution | estimates and verification | predict/simulate mission | path/plan | execution stats | speed/time/return | stage 3 | reachability | real execution differs | ACTIVE_CORE |
| visualization | maintained, legacy, paper views | figures/video | scene/plan/stats | graphics | canonical/legacy vis cfg | stages/demos/paper | shared primitives | several intentional families | ACTIVE_VISUALIZATION |
| experiments/demos | run configuration | reproduce studies/examples | profiles/seeds | CSV/MAT/figures | explicit overrides | user | core pipeline | helper duplication | ACTIVE_EXPERIMENT |
| integration/calibration | real boundary | serialize/import/measure | JSON/CSV/calibration | contracts/data | real platform cfg | offline workflow | validators | adapters | LEGACY_COMPAT |
| tests | regression gates | preserve behavior | fixtures | results | test-local overrides | CI/user | production functions | none | ACTIVE_TEST |

Actual files remain in place to avoid destabilizing imports. The recommended future layout is described in section 20.

## 3 Main Pipeline

`stage1_build_scene` loads base configurations and constructs the occupancy scene. `stage2_docking_planning` creates a shared candidate library, dispatches one hierarchical strategy, and returns selected service poses. `stage3_global_planning` computes dense or sparse path costs, chooses/accepts a visit order, stitches actual waypoint paths, estimates execution, and optionally draws legacy figures. Stage 4 benchmarks algorithms; stage 5 produces animation. See `docs/project_call_graph.md`.

## 4 Configuration System

Canonical files are `scene_config.m`, `robot_config.m`, `planner_config.m`, `visualization_config.m`, and `real_platform_config.m`. `resolve_effective_config` applies base -> profile -> experiment -> user/demo overrides recursively. Existing callers may continue using the original five APIs. `print_effective_config` makes final values visible without influencing them. Full fields are in `results/tables/config_registry.csv`.

## 5 Scene Construction

`scene_config` is the 10 x 10 m, 16-table baseline. `scene_complexity_profile_config` produces C0-C3 factor-isolation scenes. `large_scale_profile_config` produces L1-L3 validated scale profiles and V1-V3 presentation profiles. V2's 20 x 20 m/40-table scene and L2's 15 x 15 m/28-table scene are deliberate profiles, not duplicate defaults. `build_scene_from_cfg` bypasses the base loader so experiments can pass a fully resolved scene.

## 6 Robot Model and Reachability

`robot_config` is the planning robot. `robot_geometry_profile` separates `legacy_validated` from `physical_robot`. Radius, directional, lateral, and z bounds are hard geometric constraints; `planner.reachability.threshold` is a score threshold. `real_platform_config` is not robot geometry. No reachability values change here.

## 7 Candidate Generation

`generate_docking_candidates` creates baseline candidates; `build_candidate_library` normalizes their strategy-shared representation; `augment_candidate_library` supplements uncovered tasks only when enabled. Base augmentation remains disabled with variant `joint`. Historical joint experiments remain joint, while the later formal coverage module explicitly selects `position_only`. The candidate pair prefilter is separately disabled by default and enabled only in P2 audit paths.

## 8 Dock Selection

`select_multi_poses` is the legacy/local selector. Bottom-Up, Top-Down, and Bidirectional operate over the shared candidate library and return the `build_unified_plan` contract. Wrappers and diagnostics are not duplicate selection algorithms.

## 9 Planning Strategies

Bottom-Up greedily selects local coverage. Top-Down first orders regions and performs route-aware selection/repair. Bidirectional starts from a base plan and refines docks with route context while requiring nondecreasing coverage. Strategy experiments override `strategy.name`; the base remains `bottom_up`.

## 10 Global Planners

`run_path_planner` is the dispatcher. `run_*_wrapper` files are algorithm adapters, not duplicates. Common grid search utilities and `convert_path_grid2world` support wrappers. `compute_path_cost_matrix` is a dense matrix caller; `stitch_global_path` is a final-route caller. Benchmark wrappers preserve algorithm-specific behavior.

## 11 Sparse Routing

`build_sparse_docking_graph` constructs and validates KNN/region topology and stores validated edge waypoint paths. `stitch_sparse_graph_path` expands a visit order through those stored paths. HEAD's committed base was dense/k6; the pre-existing working tree changed base mode to sparse/k6. Formal P3/P4/P5 routes explicitly use sparse/k4, which is a `VALIDATED_EXPERIMENT_OVERRIDE`, not a reason to silently rewrite every baseline.

## 12 Dynamic Replanning

`apply_dynamic_event` changes the local scene, `analyze_change_impact` identifies affected regions/edges, and `local_replan` repairs only that scope with fallback rules. P5 uses explicit sparse/k4 configuration. Static planner and local-repair semantics remain separate.

## 13 Visualization

`visualization_config` is canonical. `legacy_visualization_config` supplies the flat legacy stage shape. V1/V2/V3 use `visualize_complex_scene`, `visualize_planning_overlay`, and `visualize_dynamic_replan`. Shared primitives now distinguish topology (`Sparse graph connectivity`) from stored path waypoints (`Collision-free route`), deduplicate legends, save consistently, and reuse one oriented rectangle transform. See `docs/visualization_architecture.md`.

## 14 Experiments

Experiment files own seeds, profiles, headless settings, timeout policy, and validated overrides. Repeated local result-record helpers remain because formats and partial-write behavior differ; existing shared helpers include `configure_headless_experiment`, `build_result_record`, `create_empty_result_record`, `align_record_fields`, and `write_result_table`. No experiment framework rewrite or result CSV modification is performed.

## 15 Real-Platform Interface

`real_platform_config` defines schema, frame, tolerance, IO, policy, and lift interface fields. Defaults remain `enabled=false` and `lift.enabled=false`. `integration` imports/exports contracts; `calibration` reads measured artifacts. External ROS/navigation/manipulation performs execution.

## 16 Tests

Tests cover scene determinism, candidate semantics, strategy consistency, cache identity, sparse routing, P5 locality, integration schemas, and visualization. This pass adds effective precedence, visualization single-source, named world/grid roundtrip, scene immutability, and actual-waypoint drawing checks. Validation uses short-lived MATLAB processes to avoid the repository's documented long-session instability.

## 17 Duplicate / Similar Functions

| Functional area | Existing implementations | Difference | Actual callers | Canonical choice | Compatibility action |
|---|---|---|---|---|---|
| Configuration | five configs, experiment/profile overrides | defaults vs scoped overrides | all entries | five configs + optional resolver | retain direct APIs |
| Visualization | V1/V2/V3, legacy stages, paper plots | different output contracts | demos/stages/paper | V1/V2/V3 maintained stack | legacy adapter |
| Scene drawing | complex scene, background, overview, paper scene | detail/style differs | multiple | shared rectangle transform | retain renderers |
| Route drawing | `draw_global_path`, publication plot, collision-free primitive | labels/styles differ; data is waypoints | stage/paper/V2 | collision-free primitive for maintained views | keep legacy path renderer |
| World/grid transform | `world2grid`, `grid2world`, converter wrappers | scalar canonical vs path adapters | map/planners | two utils | retain adapters |
| Collision | footprint collision, clearance, service validator, path validator | pose/path/interface scope | candidate/path/integration | scope-specific helpers | no merge |
| Reachability | single/dual/bimanual/robust/IK boundary | geometric score vs adapter | docking/diagnostics | single/dual geometric core | retain explicit interfaces |
| Candidate generation | baseline generator, library builder, augmentation | creation/normalization/supplement | stage 2/experiments | shared library contract | no algorithm merge |
| Path cost | matrix, region matrix, cached query, shared query | dense/region/cache scopes | strategies/stage 3 | scope-specific wrappers | document |
| Cache | struct cost cache, SharedPathCostContext | cost-only local vs path+cost shared handle | Top-Down/Bidirectional/stage 3 | both | separate switches |
| Sparse graph | build, validate, stitch | topology/validation/path expansion | P3/P4/P5 | current three-part design | no merge |
| CSV/result writer | record builders and several experiment-local writes | schemas/partial recovery differ | experiments | shared record helpers where schemas match | defer framework rewrite |
| Experiment config | scene/large-scale/dynamic profiles and local experiment functions | scene vs run matrix | experiments | named profiles + explicit run overrides | resolver optional |
| Timeout wrapper | P3/P4 timeout row plus experiment guards | recovery schema differs | long experiments | current targeted helpers | do not generalize yet |

Consolidated in this pass: visualization defaults, route/topology/dock primitives, legend construction, V1/V2/V3 saving, and oriented rectangle transforms. Preserved compatibility: original stage visualizers and planner `visualization` field.

## 18 Deprecated / Unused Candidates

Eight files lack direct static callers, chiefly calibration and real-platform boundary APIs. They remain `UNKNOWN`/review candidates rather than deletions. `environment/load_real_scene_json` is a compatibility wrapper around `integration/import_real_scene_json` and has the highest, still only medium, deletion confidence. See `docs/deprecation_candidates.md`.

## 19 Effective Configuration Flow

```text
scene_config + robot_config + planner_config
             + visualization_config + real_platform_config
  -> named scene profile / robot geometry profile
  -> profile override
  -> experiment override
  -> user or demo override
  -> legacy visualization and real-platform compatibility views
  -> print_effective_config (observation only)
```

The printed summary includes profile/seed, map/resolution, tables/tasks/regions/obstacles, robot profile, strategy/planner, routing/k, inflation, augmentation, prefilter, both caches, return-to-start, visualization source, real-platform enable, and lift enable.

## 20 Recommended Future Structure

The actual directory structure should remain stable for P0-P5 reproducibility. Future work can gradually move experiment-local seed/headless/result-writing boilerplate behind existing utils, add explicit external-API deprecation warnings, and migrate old stage visuals onto maintained primitives when figures are intentionally regenerated. Do not move planner wrappers, collapse geometric thresholds, merge `real_platform_config` into `robot_config`, or replace explicit experiment overrides with global defaults.

Recommended target:

```text
config/               canonical configs, resolver, printer, compatibility adapter
utils/                coordinate/geometry and generic record helpers
pipeline/             stages 1-5
planning_strategy/    common plus three strategies
global_planner/       implementations, wrappers, sparse infrastructure
replanning/           local change and repair
visualization/        canonical primitives plus maintained/legacy/paper views
experiments/          experiment entry and run configuration only
demos/                user entry points only
integration/          offline/real contract
tests/                grouped regression gates
```
