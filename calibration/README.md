# Calibration artifacts

This directory contains loaders only. Measured calibration data should be kept outside the source repository unless it is a small, versioned test fixture.

- `load_docking_error_model`: position/yaw docking-error samples.
- `load_lift_calibration`: commanded and measured lift heights.
- `load_arm_workspace_map`: measured workspace samples and reachability labels.

Accepted formats are MAT and UTF-8 JSON. Missing files or required fields raise an error; the loaders never synthesize successful calibration data.
