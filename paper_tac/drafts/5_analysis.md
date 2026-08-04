<!--
Problem Alignment Notes:
- Adds TAC-facing feasibility and complexity statements.
- Avoids overclaiming global optimality.
-->

If each task is covered by at least one feasible candidate, and the selection loop always chooses a candidate covering at least one remaining task, the loop terminates with full coverage in at most N successful iterations. Candidate evaluation and selection scale as O(KN), while pairwise path-cost computation scales as O(Q^2 C_planner). The oracle study is framed as a bounded candidate-set reference, not a proof of global optimality.
