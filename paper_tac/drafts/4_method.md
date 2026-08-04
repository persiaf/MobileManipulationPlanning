<!--
Problem Alignment Notes:
- Describes candidate generation, scoring, multi-docking selection, and routing.
- Supports the main algorithmic contributions.
-->

The method samples side-aligned docking poses around each table, removes unsafe poses, evaluates reachability and coverage, and assigns a local cost. It then greedily selects candidates that cover remaining tasks with good cost-to-coverage tradeoff. Multiple candidates can be selected for the same table. Finally, selected docking poses are connected by a global planner, enabling planner benchmarking without changing docking decisions.
