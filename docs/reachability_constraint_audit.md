# Reachability Constraint Audit

Date: 2026-09-02

Scope: code facts from `manipulator/evaluate_single_arm_reachability.m`,
`manipulator/evaluate_dual_arm_reachability.m`, and `config/robot_config.m`.

## Radial Distance Definition

For each arm, the evaluator first computes the arm base in world coordinates:

```text
arm_base = base_pose_xy + rot2(base_theta) * [arm_base_mount_x, +/- base_spacing/2]
```

Then it computes:

```text
dx = target_x - arm_base_x
dy = target_y - arm_base_y
dz = target_z - arm_base_mount_z
d_horizontal = sqrt(dx^2 + dy^2)
```

The radial hard gate is:

```text
R_min <= d_horizontal <= R_max
```

where:

```text
R_min = 0.01 m
R_max = robot_cfg.single_arm.reach_radius
```

In the current legacy robot configuration:

```text
robot_cfg.single_arm.reach_radius = 0.278 m
```

## Radial Provenance

`R_radial` comes from `robot_config.m` as a manually configured legacy value:

```text
cfg.single_arm.reach_radius = 0.278
```

The value is not derived inside the evaluator from DH parameters, and no
calibration file or measured workspace lookup is used by this code path.

Classification:

```text
C. manually configured legacy value
```

## Constraint Evaluation Order

The single-arm evaluator checks constraints in this order:

1. radial distance hard gate:
   `d_horizontal < R_min || d_horizontal > R_max`
2. vertical hard gate:
   `dz < reach_z_min || dz > reach_z_max`
3. directional hard gate:
   `local_dir(1) < -reach_backward`
4. lateral hard gate:
   `abs(local_dir(2)) > reach_lateral * 1.1`
5. continuous score calculation:
   distance score, center score, z score, forward score, lateral penalty
6. score threshold hard gate:
   `raw_score < threshold`

## Left and Right Arm Calculation

The left and right arms use the same evaluator. Their only difference is the
lateral sign of the base offset:

```text
left  base_offset = [arm_base_mount_x, +base_spacing/2]
right base_offset = [arm_base_mount_x, -base_spacing/2]
```

## Combined Reach Rule

The dual-arm evaluator computes both arm scores and returns:

```text
combined_score = max(left_score, right_score)
```

The task is considered reachable when either arm has a positive score. If both
scores are zero, the assignment is `none`.

## Hard Gates vs Score Terms

Hard gates:

```text
radial
vertical
backward
lateral
score threshold
```

Score terms:

```text
radial distance score
radial center score
vertical center score
forward score
lateral penalty
```

The radial condition appears twice: once as an early hard gate via `R_max`, and
again as softer distance/center score terms if the hard gate passes.
