> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](docs/main_robot_unification.md)。

# MobileManipulationPlanning Code Pipeline Audit

Audit date: 2026-09-23  
Scope: read-only structural review. No MATLAB source was modified.

## 1. Canonical Pipeline

```text
scene_config / resolve_effective_config
        |
        v
build_scene / build_scene_from_cfg
        |
        v
task_table_map -> build_task_regions
        |
        v
build_candidate_library -> generate_docking_candidates
        |
        v
stage2_docking_planning -> run_planning_strategy
        |
        v
stage3_global_planning -> sparse/dense path costs -> route -> global path
        |
        +--> validate_world_path_collision / footprint validation
        |
        +--> visualization and experiment exports
```

The standard project entry point is `main_mobile_manipulation_planning.m`.
The Cafeteria entry point is `main_cafeteria_mobile_manipulation_planning.m`,
which delegates to `experiments/run_cafeteria_planning.m` and follows a profile-
based configuration path.

## 2. Stage Interface Inventory

| Stage / function | File | Inputs | Outputs | Main callers / callees |
|---|---|---|---|---|
| Scene configuration | `scene_config` | none | `scene_cfg` | `pipeline/stage1_build_scene.m`, `environment/build_scene.m` |
| Effective/profile configuration | `resolve_effective_config` | profile, seed, robot/profile overrides, experiment overrides | `effective.scene`, `effective.robot`, `effective.planner`, `effective.visualization` | Cafeteria and hardware experiments |
| Robot configuration | `robot_config` | none | `robot_cfg` | Stage1, candidate generation, collision validation |
| Planner configuration | `planner_config` | none | `planner_cfg` | Stage1, Stage2, Stage3, global planner |
| Scene construction | `build_scene` | internally loads `scene_config` | `scene` with `grid_map`, `map_info`, `tasks`, `scene_cfg`, `start_pose` | `stage1_build_scene`, legacy main |
| Profile scene construction | `build_scene_from_cfg` | `scene_cfg` | same `scene` contract | `run_cafeteria_planning`, profile experiments |
| Stage 1 | `stage1_build_scene` | none | robot, scene, planner configs; scene; task map; regions | `main_mobile_manipulation_planning` |
| Task-to-table mapping | `assign_tasks_to_tables` | `scene` | `task_table_map` | Stage1, Cafeteria runner |
| Region construction | `build_task_regions` | `scene`, `task_table_map` | `regions` | Stage1, Cafeteria runner |
| Candidate library | `build_candidate_library` | regions, scene, robot, planner config | region candidate library and diagnostics | main, Stage2 |
| Candidate generation | `generate_docking_candidates` | region, scene, robot, planner config | candidates, generation diagnostics | candidate library |
| Cafeteria proxy library | `build_cafeteria_navigation_library` | regions, scene, robot, planner config | navigation-only library | `run_cafeteria_planning` when `navigation_docking_only` |
| Stage 2 | `stage2_docking_planning` | regions, scene, robot, planner, optional candidate library | selected poses, region map, task summary, pose info, plan | main, Cafeteria runner |
| Strategy dispatch | `run_planning_strategy` | candidate library, regions, scene, robot, planner | plan | Stage2; dispatches bottom-up/top-down/bidirectional paths |
| Dock selection | `select_multi_poses` | candidates, region, scene, robot, planner | selected poses and pose info | bottom-up strategy / docking layer |
| Stage 3 | `stage3_global_planning` | scene, selected poses, task summary/map, robot, planner, pose info, timing, plan | updated scene, global path, all poses, route order, executed tasks, path length, stats | main, Cafeteria runner |
| Global path planner | `run_path_planner` | grid map, start/goal poses, map info, planner config | path and fallback info | dense path stitching, sparse graph edge validation |
| Sparse graph | `build_sparse_docking_graph` | all poses, scene map, map info, planner, region IDs | validated adjacency, costs, direct paths, connectivity diagnostics | Stage3 sparse routing |
| Dense stitching | `stitch_global_path` | poses, route order, grid map, map info, planner | global path and segment stats | Stage3 dense routing |
| Sparse stitching | `stitch_sparse_graph_path` | poses, route order, sparse graph | global path and segment stats | Stage3 sparse routing |
| Replanning impact | `analyze_change_impact` | planning state, change event | affected regions/tasks/docks/edges | local replanning |
| Dynamic event | `apply_dynamic_event` | planning state, event type/id | changed state and deterministic event | local replanning, dynamic visualization |
| Local replanning | `local_replan` | planning state, changed state, event, optional impact | local result/status/metrics | dynamic experiment drivers |
| Main overlay visualization | `visualize_planning_overlay` | scene, plan, stats, visualization config, options | figure | complex planning demos |
| Complex scene visualization | `visualize_complex_scene` | scene, visualization config, options | figure | overlay and complex scene demos |
| Cafeteria visualization | `visualize_cafeteria_scene` | scene, visualization config, plan/path/operability options | figure | Cafeteria runner |
| Dynamic visualization | `visualize_dynamic_replan` | before state, event, local result, visualization config | before/after figures | dynamic replan demos |

## 3. Detailed Call Relationships

### Stage 1

`stage1_build_scene` loads `robot_config`, `scene_config`, and
`planner_config`, then calls `build_scene`. `build_scene` creates the grid map,
generates tasks, and returns the scene contract. Stage1 then calls
`validate_scene_config`, `assign_tasks_to_tables`, and `build_task_regions`.

The Cafeteria path bypasses `stage1_build_scene`: `run_cafeteria_planning`
resolves a profile, calls `build_scene_from_cfg`, validates the scene, and then
constructs task mappings and regions locally.

### Stage 2

`stage2_docking_planning` accepts an optional candidate library. If absent, it
creates one through `build_candidate_library`; if present, it validates the
library and does not regenerate it. It then calls `run_planning_strategy`, which
selects bottom-up, top-down, or bidirectional strategy behavior according to
`planner_cfg.strategy`.

The standard candidate path is:

```text
build_candidate_library
  -> generate_docking_candidates
       -> compute_reachability_field
       -> check_footprint_collision
       -> evaluate_dual_arm_reachability
       -> evaluate_docking_candidate
```

The Cafeteria `navigation_docking_only` mode intentionally uses
`build_cafeteria_navigation_library`, which is a separate proxy-support path
and should not be silently treated as equivalent to full manipulation
reachability.

### Stage 3

`stage3_global_planning` prepends the scene start pose to selected docks. It
chooses `sparse_knn` or `dense` routing from `planner_cfg.routing.mode`.

For sparse routing:

```text
build_sparse_docking_graph
  -> build_geometric_edges
  -> query_path_with_shared_cache
       -> SharedPathCostContext.query (when enabled)
       -> run_path_planner
  -> validate_sparse_graph_connectivity
  -> stitch_sparse_graph_path
```

For dense routing, `compute_path_cost_matrix` and `stitch_global_path` call the
same path-planning entry point for each required segment. The planner can apply
footprint-aware map preparation internally, while the original scene map
remains the scene/task map.

## 4. Replanning Boundary

`apply_dynamic_event` creates deterministic changes for `ADD_LOCAL_OBSTACLE`,
`INVALIDATE_DOCK`, and `ADD_TASKS`. `analyze_change_impact` limits the affected
scope to route edges, docks, regions, and tasks. `local_replan` then performs
local bookkeeping and route-cost repair without rerunning Stage1, Stage2, or
Stage3.

This is a deliberate fast-repair interface, not a full replanning pipeline.
Its result contract includes fallback flags and recomputation ratios, but some
event branches update aggregate metrics rather than constructing a new full
candidate library or a new global path object.

## 5. Visualization Boundary

The visualization layer consumes scene/plan/stats objects and should not own
planning decisions. The main reusable overlay is
`visualize_planning_overlay`; it composes `visualize_complex_scene`, selected
docks, sparse graph connectivity, and stored global waypoints.

There are parallel domain-specific visualizers:

- `visualize_complex_scene` for general scene profiles.
- `visualize_cafeteria_scene` for fixed Cafeteria tables/tasks/aisle.
- `visualize_dynamic_replan` for event and affected-edge overlays.
- legacy drawing functions such as `draw_scene`, `draw_global_path`, and
  `draw_docking_result` used by the standard stage functions.

## 6. Repeated or Overlapping Functions

| Area | Overlap | Risk |
|---|---|---|
| Scene construction | `build_scene` and `build_scene_from_cfg` both produce the scene contract | Configuration fields can diverge between legacy defaults and profile overrides |
| Configuration | direct `scene_config`/`planner_config` calls versus `resolve_effective_config` | A caller may accidentally mix raw and effective configs |
| Candidate generation | `build_candidate_library` versus `build_cafeteria_navigation_library` | Candidate counts and reachability semantics differ; result labels must preserve the mode |
| Dock selection | bottom-up, top-down, and bidirectional strategy dispatch | Strategy-specific plan fields and preferred visit order are not interchangeable without validation |
| Global routing | dense `stitch_global_path` versus sparse graph + `stitch_sparse_graph_path` | Different connectivity and caching behavior can change path length/runtime while docks remain fixed |
| Collision checking | `check_footprint_collision`, `validate_world_path_collision`, and footprint-aware map inflation | Centerline, map-cell, and full rectangle semantics must be reported separately |
| Visualization | `draw_scene` family plus complex/Cafeteria scene visualizers | Duplicate rendering paths can show different map/path representations |
| Experiment runners | main scripts, `run_cafeteria_planning`, hardware/profile runners | Outputs and runtime flags are not uniform across entry points |

## 7. Potential Interface Conflicts

1. **Raw versus effective configuration:** Stage1 uses raw config constructors,
   while Cafeteria uses profile merging. A stable interface should pass one
   immutable effective configuration bundle into every downstream stage.
2. **Scene map ownership:** `scene.grid_map` is used both for candidate
   collision filtering and global path planning. Footprint-aware planning adds
   a derived search map; callers should not overwrite the scene map in place.
3. **Candidate-library ownership:** Stage2 can regenerate candidates when the
   optional library is omitted. Experiments requiring exact reproducibility
   should always pass and validate the precomputed library.
4. **Routing-dependent stats:** `stats.sparse_graph` exists only for sparse
   routing; consumers must handle the dense case where it is empty.
5. **Return-to-start semantics:** Stage3 appends node 1 when
   `planner_cfg.execution.return_to_start` is true. Any external route-order
   consumer must distinguish dock order from the full route order.
6. **Replanning state contract:** `local_replan` assumes fields such as
   `plan.selected_poses`, `stats.sparse_graph`, `stats.route_order`, and
   `all_poses`; a generic Stage3 result without these fields is not sufficient.
7. **Validation timing:** Path validity is checked during planner calls and
   again by experiment-level validators. Reports should identify which layer
   produced each collision count.

## 8. Recommended Frozen Interfaces

For a stable Stage1/2/3 experiment API, freeze these contracts:

```matlab
[ctx, stage1] = stage1_build_scene();
[stage2] = stage2_docking_planning(ctx.regions, ctx.scene, ...
    ctx.robot_cfg, ctx.planner_cfg, ctx.candidate_library);
[stage3] = stage3_global_planning(ctx.scene, stage2.selected_poses, ...
    stage2.task_sequence_summary, ctx.task_table_map, ctx.robot_cfg, ...
    ctx.planner_cfg, stage2.pose_info_list, struct(), stage2.plan);
```

The current implementation returns positional values rather than a single
context/result struct. Until a code change is authorized, callers should keep
the existing positional signatures and explicitly preserve the following
objects: `scene`, `candidate_library`, `plan`, `global_path`, `stats`, and
`planner_cfg`.

## 9. Audit Conclusion

The project has a recognizable three-stage pipeline, but it currently contains
two valid scene/configuration paths and two candidate-library semantics. The
most important freeze requirement is therefore interface discipline: profile
experiments must label their mode, candidate libraries must be passed explicitly
when reproducibility matters, and Stage3 consumers must branch on routing mode
when reading graph diagnostics.
