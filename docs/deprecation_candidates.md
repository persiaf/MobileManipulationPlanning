# Deprecation Candidates

No MATLAB file is deleted in this maintenance pass. The following files have no direct repository-local caller in the static primary-symbol graph. Dynamic use and external API use remain possible.

| File | Reason | Last known purpose | Replacement | Safe-to-delete confidence |
|---|---|---|---|---|
| `calibration/load_arm_workspace_map.m` | no static caller | measured workspace import | `load_calibration_file` is its lower-level loader | Low; keep real-interface API |
| `calibration/load_docking_error_model.m` | no static caller | measured docking-error import | `load_calibration_file` is its lower-level loader | Low; keep real-interface API |
| `collision/validate_service_config_collision.m` | no static caller | exported service pose validation | `check_footprint_collision` provides core operation | Low; useful interface adapter |
| `environment/load_real_scene_json.m` | one-line compatibility wrapper | environment-level real scene entry | `integration/import_real_scene_json.m` | Medium; deprecate only after API review |
| `execution/predict_geometrically_reachable_tasks.m` | no static caller | restricted execution prediction | `evaluate_dual_arm_reachability` is lower-level | Low; contract guards global discovery |
| `manipulator/evaluate_bimanual_reachability.m` | no static caller | paired target screening | no exact replacement | Low; future bimanual interface |
| `manipulator/evaluate_ik_feasibility.m` | no static caller | external IK adapter boundary | no replacement | Low; intentional not-implemented boundary |
| `manipulator/evaluate_robust_reachability.m` | no static caller | docking-error robustness sweep | no exact replacement | Low; calibration analysis API |

The only plausible duplicate adapter is `environment/load_real_scene_json` versus `integration/import_real_scene_json`; it remains for compatibility. A future deletion pass should first search downstream scripts outside this repository and add deprecation warnings for one release cycle.
