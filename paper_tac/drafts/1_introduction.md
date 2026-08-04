<!--
Problem Alignment Notes:
- Introduces the engineering demand and the feasibility-before-shortest-path thesis.
- Contributions supported: formulation, docking evaluation, multi-docking, experiments.
- Claims not supported: real-robot execution and global optimality are not claimed.
-->

Mobile dual-arm robots in multi-table service environments must decide where to dock before manipulation, not only how to navigate. Tabletop task points are not navigation goals because the base must remain outside obstacles while arms satisfy reachability and side constraints. Existing path planners connect navigation states but do not decide which service states make manipulation feasible. Single docking per table also fails when tasks lie on multiple sides or near reach limits. RCH-MT-MMP addresses these issues by converting tasks into reachable docking-pose coverage decisions and then routing selected poses.
