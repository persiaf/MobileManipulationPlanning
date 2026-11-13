# Independent reviewer judgment

The broad claim is **partial**. The docking-attribution claim is **supported for all 27 frozen combinations**. The stronger claim that the new allocation is generally an improvement is **not supported**.

## What is established

The original selected-set ATSP baseline closes its bound gap in every combination. The new J route is below the conservative fixed-set lower bound in 27/27 groups. Therefore no permutation of those original docking poses can reproduce J's cost. This resolves the previous study's undersearched-baseline objection for the residual improvement beyond F. Position, top-down grasp and table-wipe median residual reductions are 12.5399%, 9.5451% and 8.8563%, respectively.

The proof is conditional on the frozen directed navigation graph: for any permutation pi of an original set S, `J_new < LB(S) <= OPT(S) <= J(S,pi)`. It neither asserts a globally optimal new docking set nor requires the new heuristic itself to be exact.

The audit passed once without corrections. It checked 414 unchanged input/model files, all 108 results, 3,888 task assignments, 1,260 strictly improving moves, 572,073 path samples and 254,796 unique sampled poses. Each result preserves the existing support for all 36 tasks and the regional finite-pool minimum of 16 or 32 docks. This is reuse of frozen IK evidence, not a new IK experiment. All 54 fixed-set certificates have a feasible Hamiltonian incumbent and closed saved solver bounds. The independent audit verifies graph/route provenance and bound arithmetic; it does not independently re-solve or formally verify HiGHS dual proofs.

## What the negative comparisons establish

Relative to the old joint selected set with its ordering fully optimized, the new route passes the stronger lower-bound test in only 13/27 cases: 4/9 position, 5/9 grasp, 4/9 wipe. The median reductions against that stronger reference are -2.4324%, +1.3369% and -0.5074%. Fourteen failures of this stronger inequality remain visible. Thus the 27/27 original-set result cannot be presented as 27/27 improvement in docking-set quality over the old joint method.

J versus I gives **8 wins, 7 ties and 12 losses**. Position-only has **0 wins, 2 ties and 7 losses**. The new balanced regional quota, screening and final-order package does not demonstrate stable improvement over the old schedule under common strong initialization. Calling this budget change generally successful would contradict the ablation.

J versus N gives **18 wins and 9 losses**, with no ties. Strong initialization helps more often in this fixed grid, but changes the local-search trajectory and can hurt. It is not a universally better initialization. This is the clean route-initialization comparison because J and N share the algorithm, known graph and logical budgets.

I versus the historical old joint result does not isolate only initialization: the historical search charged callback requests for original-set edges, whereas I starts with the complete original-set known-edge mask. Treat that comparison as combined initialization/accounting context. The recorded clarification should accompany the unchanged prospective protocol rather than silently rewriting it.

## Interpretation and routing

Retain J as the predeclared primary method. Do not choose I or N after observing which wins on each case, and do not convert the current result into a best-of-methods result. The useful result is a certified separation of order-only gains and docking-enabled gains, accompanied by a negative result for the proposed general allocation advantage.

The displayed certification heatmap retains the 14 old-set counterexamples; the ablation plot exposes both N and I outperforming J. Figure terminology is consistent with the closed fixed-set gaps. These plots support the narrower interpretation rather than hiding the negative ablations.

The method can reduce modeled navigation cost while retaining each stop's existing task service evidence and minimum stop count. Measured execution time, contact performance and physical task success were not evaluated here. These 27 deterministic development combinations do not establish held-out generalization or a runtime advantage.

**Suggested claim:** On all 27 existing frozen scene/task combinations, the predeclared new joint route preserves task support and minimum docking cardinality while achieving a cost below every ordering of the original docking set. A stronger comparison to fully reordered old joint sets succeeds in 13 cases. Strong initialization is beneficial in 18 cases, while the new allocation package is not uniformly beneficial and loses to the old schedule in 12 cases.

**Routing:** confirm the bounded docking-attribution result; reject general superiority of the allocation revision; preserve all artifacts and negative results. Further method changes require a separate predeclared experiment. No extra tuning or rerun is required to report this experiment honestly.

Confidence is high for the bounded exported-data conclusions. The paper-facing verdict remains provisional because `paper/PAPER_CLAIM_AUDIT.json` is unavailable. This is an independent agent review, not human peer review.
