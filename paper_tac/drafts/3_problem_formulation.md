<!--
Problem Alignment Notes:
- Formalizes tasks, tables, candidates, coverage, local cost, and hierarchical objective.
- Evidence used later: completion and ablations validate the priority on uncovered tasks.
-->

The problem is defined over task set T, table regions R, candidate docking poses P, reachability scores, and coverage sets. A candidate covers a task if it is safe and reaches the task above threshold. The objective penalizes uncovered tasks, local docking cost, and global route length, with uncovered tasks given priority. This formulation separates feasibility from route shortening while keeping both in the same decision problem.
