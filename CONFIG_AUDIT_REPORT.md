# Configuration Audit

Audit scope: tracked MATLAB source under `MobileManipulationPlanning`, after the
round-1 backup. No robot, task, table, scene, or safety value was changed.

| Parameter | Current value | Canonical source | Usage locations |
|---|---:|---|---|
| `robot_radius` | not defined as a scalar | chassis rectangle in `config/robot_config.m` (`0.423 x 0.366 m`) | `collision/check_footprint_collision.m`, `utils/check_robot_footprint_collision.m` |
| `inflation_radius_m` | `0.25 m` | `config/planner_config.m` | path-cost cache and planner map inflation |
| `safety_margin_m` | `0.05 m` default; profile overrides are explicit | `config/scene_config.m` and named scene profiles | grid-map construction and scene validation |
| `rho_min` | not present under this name | no active definition found | no active use found |

The repository currently has one planner inflation value and profile-specific
scene safety margins. These are intentionally preserved for this freeze because
moving profile overrides into a new schema would change experiment semantics.
Future changes should pass the effective planner and scene configurations
explicitly rather than redefining values inside functions.

Data flow audit: `scene_config` -> `build_scene_from_cfg` -> grid map -> task
regions/candidate generation -> docking selection -> global path -> centerline
and footprint validation. No temporary path dependency was added in this round.
