# Architecture

The project uses a layered planning architecture:

1. `environment` creates a deterministic simulation scene or imports measured scene/map metadata.
2. `docking` generates collision-free `[x, y, theta]` candidates and evaluates geometric arm reachability.
3. `select_multi_poses` assigns each task to explicit service poses.
4. `global_planner` builds the path-cost matrix, orders visits, stitches paths, and validates collision-free output.
5. `execution` converts assigned tasks into commanded, executed, and verified states.
6. `integration` exports ordered service configurations and imports execution feedback.

The global planning layer consumes only `service_config.pose2d`. Lift height, arm assignment, policy identifiers, and execution tolerances remain in the full service configuration for the execution boundary.

Simulation scene construction and the paper's heuristic cost terms remain in their existing modules. Real-platform code is isolated in `integration`, `calibration`, and `experiments/real` so that ROS-specific implementations do not leak into the MATLAB planning core.
