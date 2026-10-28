> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](docs/main_robot_unification.md)。

# Footprint Validation Limitation

The maintained path validator now exposes a rectangle-footprint and circle
approximation interface and can export one row per stored waypoint. The
footprint check is deliberately independent from centerline validation.

The historical Cafeteria result records in this checkout do not contain a
completed 8-dock path: the current recorded `complex12` run is
`CAFETERIA_CANDIDATE_COVERAGE_BLOCKED` with zero selected docks. Therefore a
zero-collision claim cannot be reconstructed from those files. The result is
reported as `FOOTPRINT_LIMITED` until the fixed navigation-only baseline is
rerun and its waypoint CSV is regenerated.

This is a measurement limitation, not a relaxation of safety requirements.
