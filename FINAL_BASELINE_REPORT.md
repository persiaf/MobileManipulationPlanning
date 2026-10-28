> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](docs/main_robot_unification.md)。

# Final Baseline Report

## Git

- HEAD: `4fb8333a725b68778b7e03125bfed3ae566a7ac8`
- Backup: `D:/file_used/backup/MobileManipulationPlanning_round1_20260923_110933`
- Modified files: pre-existing user changes plus the new validation utilities and audit documentation.

## Baseline

- Candidate number: `672` (Cafeteria navigation-only library; default-scene candidate regression not rerun)
- Selected docks: `8`
- Coverage: `12/12`

## Cafeteria

- Task support: `12/12`
- Selected docks: `8`; multi-task docks: `4`
- Coverage: `12/12`; path length: `30.3128 m`; waypoints: `829`
- Centerline path collision: `0`; footprint collision: `271`

## Footprint

- Before: legacy rectangle check; measured footprint collision count `271`.
- After: `utils/check_robot_footprint_collision.m` and `experiments/export_footprint_validation.m` added; the same path was exported to `results/cafeteria/footprint_validation/footprint_collision_report.csv`.
- Remaining: resolve the 271 collisions with an allowed local path-safety change, without changing docks, tasks, or safety parameters.

## Tests

- PASS: backup manifest and restore script created.
- PASS: source audit completed without changing protected parameters.
- PASS: Cafeteria navigation-only rerun: `8` docks, `12/12` coverage, centerline collision `0`.
- FAIL: footprint acceptance target, measured `271` collisions.
- NOT RUN: default 16-table baseline candidate/coverage regression.

`FINAL_STATUS = FOOTPRINT_LIMITED`
