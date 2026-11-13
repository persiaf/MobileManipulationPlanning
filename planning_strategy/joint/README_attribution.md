# Fixed-set initialization and budgeted joint selection

The original selector remains `plan_task_navigation_joint_local`. The opt-in
`plan_task_navigation_joint_budgeted` uses the same signed candidate library,
regional minimum-cover enumeration and unified plan representation. It never
evaluates IK or changes task support.

Its required `options` contain `initial_candidate_indices`,
`initial_matrix_visit_order`, `cost_matrix`, `known_edge_mask` and a directed
`edge_cost_callback`. Node 1 is the start; the other nodes follow the unchanged
flattened candidate-library order. The initial selected-set graph must already
be priced. Unknown edges may be Inf in the matrix but must be false in the mask.
Known Inf means an unavailable navigation transition.

The optional strategy dispatcher entry is `task_navigation_joint_budgeted`;
it takes these options from `planner.strategy.joint_budgeted`.

The search distributes candidate trials over regions and reserves directed
edge requests for a complete final selected-set graph. It then applies
best-improvement directed 2-opt and relocate moves until their combined
neighborhood is exhausted or a recorded limit is reached. Geometry ranks trials
only. Accepted changes always strictly decrease the actual directed objective.
`fixed_set_order_converged` refers only to this ordering neighborhood; it does
not certify the whole joint selection problem.

`solve_fixed_set_atsp.py` solves the separate original-set counterfactual with
binary assignment constraints and subtour cuts. It retains a valid lower bound
and a Hamiltonian feasible upper bound separately. If a new joint route has
cost below the conservative lower bound, **no visit order on the fixed original
docks can explain the improvement**. A second certificate on the previous joint
output tests the stronger claim of improved docking-set quality over that output.

The fixed graph and solve are initialization costs, outside the *additional*
1500-request search allowance. CSVs report both quantities. External cache
reuse does not exempt a new logical request from the per-method search allowance.
Wall times are diagnostic, not an end-to-end speed comparison.

The complete study protocol is
`paper/JOINT_ATTRIBUTION_PROTOCOL_20261113.md`. Reproduce from repository root:

```powershell
python experiments/prepare_joint_attribution.py
```

Then run each layout (0, 1, 2) in MATLAB; they write disjoint case directories:

```matlab
addpath('pipeline'); init_paths;
run_joint_attribution(0);
```

After all three workers complete:

```powershell
python experiments/analyze_joint_attribution.py
python experiments/audit_joint_attribution.py
python experiments/report_joint_attribution.py
python visualization/joint_attribution/plot_attribution.py
python visualization/joint_attribution/plot_decomposition.py
python visualization/joint_attribution/plot_routes.py
python visualization/joint_attribution/plot_ablations.py
```

Use a separate output directory for revised protocols. Do not replace the
frozen nine-scene source outputs or interpret replay as newly validated IK.
MILP field semantics are documented in the
[SciPy API](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.milp.html).
