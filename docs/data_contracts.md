# Data contracts and coordinates

## Units and frames

- World positions use metres.
- Angles use radians.
- The default and currently supported exchange frame is `map`.
- A planar pose is always `[x, y, theta]`.
- Positive `theta` is counter-clockwise about the positive z-axis.
- JSON files are UTF-8.

## Grid convention

MATLAB matrices use row/column indexing. Row 1 represents the highest world-y band, matching the top row of a ROS PGM image. Column 1 represents the lowest world-x band.

For resolution `r`, map height `H`, and origin `[ox, oy]`:

```text
column = floor((x - ox) / r) + 1
row    = floor((H - (y - oy)) / r) + 1

x = ox + (column - 0.5) * r
y = oy + H - (row - 0.5) * r
```

The world-to-grid-to-world round-trip error is bounded by half a grid cell per axis. A non-zero ROS map origin is stored in `map_info.origin_xy`; call the optional-origin form of `world2grid` and `grid2world` when processing imported maps.

## Mission plan

`mission_plan.json` contains a schema version, scene version, planner version, start pose, service visit order, and service configurations. Each service configuration includes base pose, lift height, table/task identifiers, arm assignments, policy identifier, tolerances, and optional predicted metrics.

The route contains service targets, not every grid point in the MATLAB path. A Linux navigation stack remains responsible for safe motion execution.

## Real scene

`real_scene.json` contains versioned map metadata, table/workbench geometry, measured object/task poses, grasp or destination poses when available, and timestamps. Paths inside the document must be relative to the exchanged data bundle.

## Execution log

The execution log returns planned and measured docking poses, position/yaw errors, lift targets and measurements, task timestamps, policy identifiers, success flags, verification flags, failure reasons, retry counts, and replanning status.

## Planning evidence

`separate_execution_status` preserves predicted, commanded, executed and verified task IDs with explicit evidence metadata. A geometric simulation supplies geometric prediction and simulated geometric verification, not measured execution or IK evidence. Unknown evidence produces `NaN` rates in `build_execution_evidence_metrics`; percentages use all scene tasks as the denominator. `geometric_coverage_rate`, `simulated_geometric_completion_rate`, `proxy_coverage_rate`, `ik_feasibility_rate` and `actual_completion_rate` are distinct fields. JSON encodes MATLAB NaN as null; the current MATLAB CSV writer emits `NaN`. A planning success additionally requires a validated complete route.

Paths are SE(2) waypoint sequences. `validate_planned_route` checks their exact concatenation from stored segments, requested docking endpoints, continuity and resampled footprint geometry. A geometric path does not supply controller tracking, dynamics or arm-joint collision evidence. See [repair report](geometry_protocol_evidence_repair.md).
