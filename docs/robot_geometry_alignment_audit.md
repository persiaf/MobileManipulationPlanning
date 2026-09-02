# Robot Geometry Alignment Audit

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

This audit compares the validated planning geometry in `robot_config.m` with
the physical robot measurements used for future alignment. The legacy
configuration must remain the default because the validated large-scale
checkpoint was generated with it.

## Coordinate Semantics In Current Code

The mobile base pose is `[x, y, theta]` in the world frame. Arm base offsets
are expressed in the mobile-base frame and rotated by `rot2(theta)`:

```matlab
arm_base_xy = base_xy + rot2(theta) * [arm_base_mount_x; +/- base_spacing/2]
```

The current code therefore interprets:

- `arm_base_mount_x` as forward offset from mobile-base origin to each arm base
- `dual_arm.base_spacing` as left-right distance between the two arm-base
  origins
- `arm_base_mount_z` as arm-base height in the same vertical frame as task `z`

## Quantity Mapping

| Quantity | Current Code | Physical Measurement | Same Definition? | Action |
|---|---:|---:|---|---|
| Chassis length | `0.423 m` | N/A | Not audited | Keep legacy default |
| Chassis width | `0.366 m` | N/A | Not audited | Keep legacy default |
| Chassis center / base origin | implicit base pose origin | physical reference not fully documented | Unconfirmed | Do not overwrite |
| Arm mount x | `cfg.chassis.length / 2 = 0.2115 m` | `xa = 0.06 m` | Same code role if both are base-frame fore-aft offsets, but value conflicts | Profile only |
| Left arm y | `+0.125 m` | `+0.20 m` if `da = 0.40 m` is center-to-center spacing | Likely same semantic role | Profile only |
| Right arm y | `-0.125 m` | `-0.20 m` if `da = 0.40 m` is center-to-center spacing | Likely same semantic role | Profile only |
| Arm spacing | `0.250 m` | `da = 0.40 m` | Likely same physical quantity | Profile only |
| Arm mounting z | `0.1833 m` | `h0 = 0.40 m` | Likely same vertical role, but physical reference height must be confirmed | Profile only |
| Single-arm reach radius | `0.278 m` | N/A | Not audited | Keep legacy default |
| Forward reach | `0.278 m` | N/A | Not audited | Keep legacy default |
| Backward reach | `0.197 m` | N/A | Not audited | Keep legacy default |
| Lateral reach | `0.278 m` | N/A | Not audited | Keep legacy default |

## Profile Boundary

`utils/robot_geometry_profile.m` adds explicit profiles:

- `legacy_validated`: exact identity with `robot_config()`
- `physical_robot`: maps `da = 0.40 m`, `xa = 0.06 m`, and `h0 = 0.40 m` to
  the same fields used by the current reachability model

The default project behavior is unchanged. The physical profile is
exploratory and must not be used to reinterpret prior validated results.

## Exploratory Candidate-Level Smoke

`experiments/exp_robot_geometry_profile_smoke.m` compared C0 and C2 with seed
2026 using candidate generation only.

| Case | Profile | Candidate Count | Coverage | Uncovered Tasks |
|---|---|---:|---:|---:|
| C0 | `legacy_validated` | 7134 | 100.00% | 0 |
| C0 | `physical_robot` | 7134 | 64.00% | 27 |
| C2 | `legacy_validated` | 3812 | 55.38% | 29 |
| C2 | `physical_robot` | 3812 | 32.31% | 44 |

The candidate count is unchanged because the same base candidate positions
are generated. Coverage changes because reachability is evaluated from
different arm-base offsets and height.

## Audit Conclusion

The physical measurements appear mappable to the current fields, but the
base-frame origin and vertical reference must be confirmed before any formal
physical-robot claims. The legacy geometry must remain the default for all
validated paper results.
