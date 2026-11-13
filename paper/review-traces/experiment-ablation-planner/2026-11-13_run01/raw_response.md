# Reviewer design: attributing navigation gains to docking selection

Date: 2026-11-13. Reviewer: `/root/attribution_design`. This is a prospective local-code experiment design, not a review of new results.

The 2026-11-11 study shows that the current three-pass fixed-set reordering baseline is insufficient for clean attribution: 16/27 endpoints retain an improving 2-opt neighbor, and the post-hoc converged alternative changes the joint comparison from 24 wins to 21. The redesign must therefore separate order-search gains from the gains enabled by changing docking poses.

## Required controls

| Priority | Control | What it tests | Expected if docking selection matters |
|---|---|---|---|
| 1 | F: fixed original minimum-cover docking set, ATSP solve with feasible upper bound and valid lower bound | Can any ordering of the original docking set explain the joint result? | New joint cost is below the fixed-set lower bound on at least some cases. |
| 1 | J: new joint method initialized from F's feasible incumbent, with minimum-cover replacements and reserved reordering work | Can the expanded docking search improve a sufficiently strong shared starting route while retaining coverage/cardinality? | Positive residual gains remain after the fixed-set ordering opportunity is removed. |
| 2 | I: legacy replacement/2-opt schedule initialized from the same F incumbent, with the same logical query, evaluation and time caps as J | Does the changed allocation/screening improve over initialization alone? | J improves cost or reaches comparable cost with less work than I. |

F is the freeze-docking ablation. Once F has an exact certificate, re-running another fixed-set heuristic from its optimum would be a no-op and adds no evidence. Keep the previously saved historical baseline and old joint results as context; do not relabel them as the new primary control.

## Frozen evidence

Use all 9 existing cases by all 3 task modes, without selection: 27 comparisons. Read the existing model, scene definitions, candidate pools, numerical support libraries, region ownership and minimum-cardinality certificates. Hash these inputs and do not regenerate or overwrite them. No new IK search is needed or counted as fresh reachability evidence. Navigation cost semantics stay directed actual path length plus 0.15 times wrapped service-endpoint yaw difference. Count raw L separately from J.

Every resulting selected set must have the same per-region minimum cardinalities, cover all 36 tasks using the existing support matrix, and have a finite fully priced closed tour. New navigation edges can be queried under the unchanged navigator. A cache miss and an external path computation are distinct counters; loading old costs must not silently grant one search more logical budget.

## Attribution certificate

For the original frozen selected set S0, compute a valid ATSP lower bound LB0 and a feasible upper bound UB0. Exactness means a Hamiltonian tour and a closed certified gap; a disconnected assignment solution is not an ATSP tour. If the cut loop or MILP times out, retain the last valid LB and the best already verified feasible tour. Never label a timeout incumbent optimal.

For a fully feasible joint route of cost JJ:

- `JJ < LB0 - certificate_tolerance` certifies that no reordering of S0 can explain the joint outcome. This is the primary docking-attribution test.
- `JJ < UB0` alone shows improvement over the found route, but cannot exclude insufficient fixed-set search if the gap remains open.
- If F is exact, `J_staged - J_fixed_optimum` is the ordering opportunity; `J_fixed_optimum - JJ` is the docking-enabled residual. Report both rather than crediting their sum to pose selection.
- If F is not exact, use `max(0, LB0 - JJ)` as the certified lower bound on the docking-enabled improvement and report the unresolved gap.

Use a conservative certificate margin, e.g. 1e-6 plus an explicit export/solver tolerance allowance. Keep the earlier absolute 1e-7 descriptive win/tie rule separately if continuity with historical figures is useful. Report LB, UB, gap, status and route per case so the mathematical implication can be audited.

The certificate is for the existing finite graph and the original selected set. It proves neither globally optimal joint selection nor continuous-space optimality. It also does not prove that every new-versus-old-joint gain came from better pose selection: the old joint ordering could be undersearched too. If making that stronger claim, solve/bound ATSP on the old joint selected set S_old and require `JJ < LB_old`. Otherwise explicitly attribute only the residual relative to S0.

## Initialization and budget accounting

Use F's same feasible incumbent as the start of I and J. Charge or separately itemize the shared graph pricing and F solve in both end-to-end totals; do not omit the stronger initialization when discussing computation. The fixed-set solver is a deliberately strong quality reference, not an equal-wall-time speed competitor.

For I versus J, use the same available known-edge mask at entry, same logical edge request cap, same neighbor evaluation cap and same cooperative wall-clock limit. Reserve order-search work before exhausting regional replacements, and retain the best fully priced feasible incumbent on any limit. Record initialization, regional replacement and reorder counters/times separately. If limits differ, label the comparison as a budget sensitivity experiment instead of an isolated allocation ablation.

Geometry can rank trial replacements but may not become an accepted route cost. Early screening must be a stated heuristic or an admissible bound; it must not be reported as exhaustive local optimality after skipped candidates. A monotone incumbent initialized from F guarantees non-worsening by construction; improvement frequency, magnitude and bound margins are the evidence of usefulness, not the absence of losses alone.

## Minimal execution order and outputs

1. Freeze the 27 input records and protocol before outcome inspection.
2. Unit-test the ATSP solver against brute force on small directed graphs, including unreachable edges and interrupted cut loops; test the search preserves the feasible seed under budget exhaustion and prices directed reversal correctly.
3. Solve F on the already exported fixed-set graphs. Existing graphs contain 17 or 33 nodes, so no full 900-node navigation graph is required.
4. Run J, and I if an allocation-specific claim is intended, on all 27 cases. Preserve every failed/unresolved result.
5. Audit only new evidence: frozen hashes, selected-set support/cardinality, accepted move monotonicity, route pricing and certificate inequalities. Do not rerun the historical IK study.
6. Plot per-case original cost, fixed-set optimum/bounds and joint cost; plot certified residual margins; retain the main scene style for fixed `L0_D2` route panels and display all three task modes.

## Coverage assessment and unnecessary work

The primary F/J comparison directly answers the undersearched-baseline criticism. I/J isolates whether search allocation adds beyond stronger initialization. Changing layouts, arm models, task tolerances, cover matrices, candidate generation or task distributions would introduce new confounds and is outside this experiment. Repeating IK or producing p-values for the same deterministic 27-case grid adds no useful evidence. A second heuristic-only fixed-set baseline is unnecessary when the ATSP gap is closed.

Compute is CPU-only. F can reuse existing directed matrices; set a declared per-instance solver time limit and preserve open gaps. Joint work is bounded by declared edge/evaluation/time caps. Report measured preprocessing and solve times after the run instead of claiming an unmeasured runtime advantage.
