<!--
Problem Alignment Notes:
- Covers the core mismatch between tabletop tasks and base goals.
- Supports contributions on hierarchical planning, candidate scoring, multi-docking, and experiments.
- Evidence: main scene, submission-level baseline, candidate quality.
-->

Multi-table mobile manipulation requires a robot to decide not only how to move between workspaces, but also where to dock so that tabletop tasks are reachable by its manipulators. Treating tabletop task points as navigation goals ignores the mismatch between the mobile-base configuration space and the arm workspace. RCH-MT-MMP partitions tabletop tasks, generates feasible docking poses, scores them by coverage and reachability, selects multiple docking poses when needed, and connects the selected poses by a global planner. The main experiment completes 36/36 tasks, and submission-level baselines show that the full method keeps 100% completion while single-docking and nearest-docking degrade strongly.
