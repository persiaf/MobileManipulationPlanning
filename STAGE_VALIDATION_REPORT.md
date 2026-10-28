> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](docs/main_robot_unification.md)。

# Stage 1/2/3 Validation Report

Validation date: 2026-09-23  
Algorithm source was not modified during this validation.

## Summary

| Stage | Case | Coverage | Dock count | Path exists | Centerline collision | Footprint collision | Status |
|---|---|---:|---:|---|---:|---:|---|
| Stage1 | baseline | 36/36 | 27 | yes | 0 | 0 | PASS |
| Stage1 | cafeteria complex12 | 12/12 | 8 | yes | 0 | 0 | PASS |
| Stage2 | L1 | 90/90 | 33 | state prepared | not applicable | not applicable | PASS |
| Stage2 | L2 | 126/126 | 49 | state prepared | not applicable | not applicable | PASS |
| Stage2 | L3 | 181/181 | 72 | state prepared | not applicable | not applicable | PASS |
| Stage3 | D1: ADD_LOCAL_OBSTACLE | 100% | 49 | retained route state | validated by no-collision flag | not independently exported | PASS |
| Stage3 | D2: INVALIDATE_DOCK | 100% | 49 | retained route state | validated by no-collision flag | not independently exported | PASS |
| Stage3 | D3: ADD_TASKS | 100% | 50 | retained route state | validated by no-collision flag | not independently exported | PASS |

## Stage1

### Baseline

The default 16-table pipeline was executed through scene construction, candidate
generation, docking selection, and global planning.

- Tasks: `36`
- Candidates: `7893`
- Coverage: `36/36`
- Selected docks: `27`
- Path length: `79.4383 m`
- Path exists: yes (`2158` waypoints)
- Centerline collision: `0`
- Rectangle footprint collision: `0`
- Runtime: `26.933 s`

Raw execution log: `results/tables/stage_validation_baseline.log`.

### Cafeteria

- Tasks: `12`
- Coverage: `12/12`
- Selected docks: `8`
- Path length: `28.0702 m`
- Path exists: yes (`775` waypoints)
- Centerline collision: `0`
- Rectangle footprint collision: `0`
- Runtime: `23.201 s`

Raw execution log: `results/tables/stage_validation_cafeteria.log`.

## Stage2

Stage2 validation loaded the frozen P3/P4 state files for seed 2026 and checked
the stored plan contract (`task_sequence_summary`, covered task IDs, and
`num_docking_poses`). This isolates Stage2 from Stage3 runtime and preserves the
same candidate library and selected-dock set used by the routing experiments.

| Profile | State file | Tasks | Coverage | Docks |
|---|---|---:|---:|---:|
| L1 | `p3p4_state_l1_seed2026_bottom_up.mat` | 90 | 90/90 | 33 |
| L2 | `p3p4_state_l2_seed2026_bottom_up.mat` | 126 | 126/126 | 49 |
| L3 | `p3p4_state_l3_seed2026_bottom_up.mat` | 181 | 181/181 | 72 |

Stage2 has no global path output by contract; path and collision checks belong
to Stage3. The validation log is `results/tables/stage_validation_stage2.log`.

## Stage3 / Dynamic D1-D3

The P5 local-replanning experiment was executed for L2 seed 2026. The event
mapping is the maintained experiment convention:

- D1 = `ADD_LOCAL_OBSTACLE`
- D2 = `INVALIDATE_DOCK`
- D3 = `ADD_TASKS`

All three local-replan rows returned `SUCCESS`, retained a connected sparse
graph, set `no_collision_invalid_route=1`, and achieved 100% raw and verified
coverage. D3 adds one dock because the event adds tasks; D1 and D2 retain the
49-dock state.

| Event | Result docks | Coverage | Final path length | Replanning status | Sparse graph connected |
|---|---:|---:|---:|---|---|
| D1 | 49 | 100% | 165.9404 m | SUCCESS | yes |
| D2 | 49 | 100% | 123.0044 m | SUCCESS | yes |
| D3 | 50 | 100% | 124.2223 m | SUCCESS | yes |

The current P5 local-replan result schema reports route safety through
`no_collision_invalid_route`; it does not export a separate per-waypoint
rectangle-footprint CSV for each event. Therefore the footprint column above is
marked as not independently exported rather than being inferred as zero.

Raw records: `results/tables/dynamic_local_replanning_p5_raw.csv`.  
Execution log: `results/tables/stage_validation_stage3.log`.

## Conclusion

`FINAL_STATUS = STAGE1_STAGE2_STAGE3_VALIDATED`

Stage1 baseline and Cafeteria complete end-to-end with zero centerline and
footprint collisions. Stage2 L1/L2/L3 frozen plans provide full candidate/dock
coverage. Stage3 D1/D2/D3 local replanning completes with connected sparse
graphs and collision-safe route flags; a dedicated footprint CSV remains a
future measurement enhancement for dynamic event paths.
