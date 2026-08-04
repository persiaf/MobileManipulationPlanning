<!--
Problem Alignment Notes:
- Organizes prior work around global path planning, mobile manipulation, coverage, and hierarchy.
- Keeps the distinction that this paper contributes a docking-selection decision layer.
-->

Global path planners such as Dijkstra, A*, JPS, and Theta* solve routing once navigation goals are known. Mobile manipulation and base placement methods evaluate reachability from candidate base poses. Coverage and viewpoint selection provide a set-cover analogy. Hierarchical planning motivates decoupling local feasibility from global route construction. The present work combines these ideas for multi-table dual-arm tasks by exposing reachability-aware coverage sets to a replaceable routing layer.
