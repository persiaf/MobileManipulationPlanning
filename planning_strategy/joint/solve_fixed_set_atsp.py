"""Directed fixed-dock tour certificate, using assignment MILP + DFJ cuts.

No metric/symmetry assumption; unavailable edges are forbidden variables.
Every intermediate MILP is a relaxation of ATSP. Its dual bound remains a
valid ATSP lower bound even when its integer solution contains subtours.
Only a single Hamiltonian cycle can update the feasible incumbent.
"""
from __future__ import annotations

import time
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import coo_matrix


def tour_cost(matrix, route):
    return float(sum(matrix[a, b] for a, b in zip(route, route[1:] + route[:1])))


def solve_fixed_set_atsp(matrix, seed, time_limit=60.0):
    matrix = np.asarray(matrix, dtype=float)
    n = len(matrix)
    if matrix.shape != (n, n) or n < 2 or np.isnan(matrix).any() or (matrix < 0).any():
        raise ValueError("Nonnegative square directed costs required")
    seed = list(map(int, seed))
    if sorted(seed) != list(range(n)) or seed[0] != 0:
        raise ValueError("Seed must be a permutation starting at node zero")
    best, upper = seed, tour_cost(matrix, seed)
    if not np.isfinite(upper):
        raise ValueError("A finite incumbent is required")
    edge = [(i, j) for i in range(n) for j in range(n) if i != j and np.isfinite(matrix[i, j])]
    c = np.array([matrix[i, j] for i, j in edge])
    row, col, val = [], [], []
    for k, (i, j) in enumerate(edge):
        row.extend((i, n+j)); col.extend((k, k)); val.extend((1., 1.))
    lower_constraints, upper_constraints = [1.]* (2*n), [1.]* (2*n)
    cuts, trace, lower = set(), [], 0.0
    start = time.perf_counter()
    while time.perf_counter()-start < time_limit:
        A = coo_matrix((val, (row, col)), shape=(len(lower_constraints), len(edge))).tocsc()
        result = milp(c, integrality=np.ones(len(edge)), bounds=Bounds(0, 1),
                      constraints=LinearConstraint(A, lower_constraints, upper_constraints),
                      options={"mip_rel_gap": 0., "time_limit": max(.001, time_limit-(time.perf_counter()-start))})
        bound = getattr(result, "mip_dual_bound", None)
        if bound is not None and np.isfinite(bound):
            lower = max(lower, float(bound))
        cycles = []
        if result.x is not None:
            chosen = [edge[k] for k, x in enumerate(result.x) if x > .5]
            succ = dict(chosen)
            if len(chosen) == n and len(succ) == n and len(set(succ.values())) == n:
                remaining = set(range(n))
                while remaining:
                    root = min(remaining); cycle = [root]; nxt = succ[root]
                    while nxt != root:
                        cycle.append(nxt); nxt = succ[nxt]
                    cycles.append(cycle); remaining.difference_update(cycle)
                if len(cycles) == 1:
                    value = tour_cost(matrix, cycles[0])
                    if value < upper:
                        upper, best = value, cycles[0]
        trace.append({"status": int(result.status), "lower_bound": lower,
                      "upper_bound": upper, "cycle_sizes": list(map(len, cycles)),
                      "elapsed_s": time.perf_counter()-start})
        if upper-lower <= 1e-7:
            break
        if not cycles or len(cycles) == 1:
            break  # limit/solver failure; never promote a bound to an incumbent
        new = 0
        for cycle in cycles:
            key = tuple(sorted(cycle))
            if key in cuts:
                continue
            cuts.add(key); new += 1; subset = set(cycle); rid = len(lower_constraints)
            for k, (i, j) in enumerate(edge):
                if i in subset and j in subset:
                    row.append(rid); col.append(k); val.append(1.)
            lower_constraints.append(-np.inf); upper_constraints.append(len(cycle)-1.)
        if not new:
            break
    # Subtract a numerical guard for attribution comparisons; retain raw LB.
    guard = 1e-6 + 1e-9*max(1., abs(lower))
    return {"route_local_zero_based": best, "upper_bound": upper,
            "lower_bound_raw": lower, "lower_bound_conservative": max(0., lower-guard),
            "absolute_gap": max(0., upper-lower), "certified_optimal": upper-lower <= 1e-7,
            "elapsed_s": time.perf_counter()-start, "subtour_cut_count": len(cuts),
            "trace": trace, "solver": "scipy.optimize.milp/HiGHS, binary assignment + DFJ cuts"}
