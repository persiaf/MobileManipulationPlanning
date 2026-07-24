# Windows MATLAB to Ubuntu ROS integration

## Boundary

MATLAB performs offline planning and writes `mission_plan.json`. A separate Ubuntu 20.04/ROS Noetic process reads the plan, resolves each service target through the navigation/manipulation stack, and writes an execution log. MATLAB then imports the log for analysis.

No ROS Toolbox dependency is required on Windows, and no ROS control code belongs in the MATLAB planning modules.

## Files to synchronize

- `integration/schemas/`
- generated `mission_plan.json`
- the matching `real_scene.json`
- an exported snapshot of `config/real_platform_config.m` values
- `docs/data_contracts.md`

## Synchronization checklist

- Use UTF-8 and LF line endings for exchanged text files.
- Use relative data paths; translate host paths at the deployment boundary.
- Preserve the same `scene_id`, schema version, and map version on both machines.
- Confirm `frame_id=map`, map origin, resolution, and yaw convention.
- Set Linux executable permission only on Linux-side scripts; JSON/schema files do not require it.
- Validate the mission against `mission_plan_schema.json` before execution.
- Return measured outcomes; never infer real success from MATLAB geometric reachability.

## Not implemented here

ROS navigation actions, lift control, collision-aware arm IK, coordinated bimanual motion, ACT policy invocation, perception verification, retry policy, and online replanning belong to the Linux execution system. MATLAB adapters report an explicit missing/not-implemented state until those components are configured.
