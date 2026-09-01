# Experiment Result-to-Claim: Large-Scale Coverage and Path-Cost Diagnostics

Date: 2026-09-01
Branch: `research/large-scale-complex-tasks`
Status: provisional, `[pending external review]`
Paper claim audit: unavailable (`paper/PAPER_CLAIM_AUDIT.json` is not present)

## Claim verdict

- Intended claim: the S4 coverage drop and the dominant path-planning cost can be localized well enough to choose the next research direction without changing planner semantics.
- Verdict: `partial`
- Confidence: high for the measured configurations; provisional for broader claims.
- Routing action: supplement before making a general paper claim. Prioritize coverage-preserving candidate generation diagnostics, and separately evaluate a shared lazy path-cost cache. Do not change strategy semantics based on these diagnostics alone.

## What the results support

### Coverage

Across S1-S4 and five seeds, the candidate-library upper bound exactly matches Bottom-Up, Top-Down, and Bidirectional final predicted coverage for every run.

| Scale | Mean tasks | Library | Bottom-Up | Top-Down | Bidirectional |
| --- | ---: | ---: | ---: | ---: | ---: |
| S1 | 36.0 | 100.0% | 100.0% | 100.0% | 100.0% |
| S2 | 64.0 | 100.0% | 100.0% | 100.0% | 100.0% |
| S3 | 87.8 | 100.0% | 100.0% | 100.0% | 100.0% |
| S4 | 120.4 | 74.43% | 74.43% | 74.43% | 74.43% |

S4 per-seed library and strategy coverage is 71.5%, 75.6%, 74.4%, 74.4%, and 76.3%. All 154 uncovered S4 task instances are `NOT_IN_LIBRARY`; there are zero observed `LIBRARY_COVERABLE_NOT_SELECTED` cases.

The loss is spatially structured but not confined to only a few regions. Twelve of 16 region indices lose coverage, with aggregate failure rates from 21.1% to 43.2%; regions 9-12 remain at 100%. Among retained collision-free candidates, uncovered tasks have a maximum observed combined reachability score of zero. The pipeline does not expose pre-collision candidates, so this result cannot separate collision rejection from reachability loss before library insertion.

The approximate greedy requirement for covering the library is 34-36 docks per seed. The summed initial Top-Down budget estimate is 39-45, and all strategies select 35-36 docks while covering every library-coverable task. Dock budget therefore does not explain the missing S4 tasks.

### Path-cost reuse

The S1/S4, seeds 1-3 audit records directed keys and does not assume reverse-path symmetry.

| Metric | Value |
| --- | ---: |
| Logical queries | 41,111 |
| Directed unique pairs | 24,294 |
| Repeated queries | 16,817 (40.91%) |
| Current expensive computations | 36,427 |
| Maximum additional expensive-call reduction | 12,133 (33.31%) |

Of the repeated queries, 9,692 (57.6%) are within the same query source and 7,125 (42.4%) cross query sources. Cross-source opportunities represent 17.3% of all logical queries.

- Bottom-Up: 2,642 queries, 2,558 unique; at most 84 additional calls (3.2%) are avoidable.
- Top-Down: 28,068 queries, 22,590 unique, 23,384 current expensive calls; after its existing cache, at most 794 additional calls (3.4% of current expensive calls) are avoidable within a single strategy run.
- Bidirectional: 10,401 queries, 2,574 unique; 7,827 calls (75.3%) are repeated expensive computations.
- Bidirectional refinement alone issues 7,759 queries over 2,574 unique pairs: 5,185 duplicates (66.8%), with individual directed pairs evaluated up to six times.
- Stage3 recomputes 0/2,475 prior Bottom-Up pairs, 626/2,494 prior Top-Down pairs (25.1%), and 2,475/2,475 prior Bidirectional pairs (100%).

The existing Top-Down cached sources issue 25,406 queries with 4,684 hits, an 18.44% hit ratio consistent with the prior 16-21% range. Region ordering contributes 4,344 entirely first-time queries; anchor selection has a 20.6% repeat ratio. The low hit ratio is therefore mainly caused by first-time pair dominance, while the cache scope still misses stage3 reuse opportunities.

## What the results do not support

- They do not identify the exact pre-library split between raw-candidate absence, collision rejection, and reachability rejection.
- They do not prove that the same ratios hold outside S1/S4, seeds 1-3, the current map generator, or the current global planner.
- The 12,133-call reduction is a conservative directed-key upper bound for the complete three-strategy diagnostic batch. Cross-strategy reuse is unavailable when only one strategy is run.
- The greedy dock count is an approximate set-cover diagnostic, not a minimum-dock proof.
- These measurements do not by themselves establish end-to-end runtime speedup; cache lookup, memory, and invalidation costs were not measured.

## Next evidence

1. Add pre-library counters for raw candidate generation, collision rejection, and task-level reachability rejection without changing thresholds.
2. Prototype and benchmark a shared lazy cache only after defining scene invalidation and directed/symmetric key semantics.
3. Measure wall-clock and memory effects separately for Top-Down and Bidirectional; do not infer speedup directly from query counts.

Evidence files are generated under `results/tables/` and intentionally remain outside the code commit.

## 2026-09-01 S4 pre-filter follow-up

Status: provisional, `[pending external review]`.

The five-seed task trace accounts for all 154 S4 failures:

| Failure category | Count | Share |
| --- | ---: | ---: |
| NO_RAW_SUPPORT | 0 | 0% |
| ALL_RAW_COLLISION_REJECTED | 0 | 0% |
| COLLISION_VALID_BUT_UNREACHABLE | 154 | 100% |
| REMOVED_DURING_LIBRARY_BUILD | 0 | 0% |

The actual pipeline generates raw poses, removes footprint collisions, and only then evaluates task reachability. Consequently, it cannot label a collision-rejected pose with a task-specific reachability result without adding counterfactual computation. The supported statement is narrower: every failed task belongs to a region with raw and collision-valid poses, but none of those retained poses reaches that task at the configured threshold. All collision-valid candidates enter the library, so there is no later library pruning loss.

All failures lie on the table-local center row (`table_local_y = 0`, 0.20 m from the nearest table edge). The outer task rows at `table_local_y = +/-0.16 m` are fully coverable. Regions 9-12 retain collision-valid reach support for center-row tasks in every seed; in the inspected seed 1 scene they retain 4-6 such poses per region, while the other 12 regions retain none. This supports investigating candidate pose geometry and collision-preserving sampling for center-row reach, but it does not support changing reachability thresholds or robot geometry from these results alone.

## 2026-09-01 Bidirectional shared-cache prototype

Status: provisional, `[pending external review]`.

Paired cache-OFF/cache-ON experiments used S1 and S4, seeds 1-5. Each pair used the same generated scene and candidate library. All ten pairs have identical predicted coverage, selected docking poses/count, visit order, total path length, and objective within the stated numerical assertions.

| Scale | Calls OFF | Calls ON | Calls removed | Total time OFF | Total time ON | Speedup | Cache memory |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| S1 | 741.0 | 181.8 | 559.2 | 44.43 s | 15.14 s | 2.94x | 0.55 MB |
| S4 | 2745.6 | 679.6 | 2066.0 | 295.33 s | 79.39 s | 3.72x | 2.62 MB |

The cache removes an average of 369.2 S1 and 1370.4 S4 refinement computations. It serves every stage3 distance-matrix pair from cache: 171 per S1 run and 630-666 per S4 run (658.8 mean). For the earlier S1/S4 seeds 1-3 audit subset, all 2,475 previously identified stage3 pairs are therefore eliminated as expensive calls. Directed final-route edges add 19 S1 and 36-37 S4 hits; reverse-direction edges remain separate by design.

The cache is run-local, in-memory, and attached only to a Bidirectional plan. It stores complete path, raw path cost, and success, validates map/planner identity once per query source, and is disabled by default. The reported wall-clock results were collected as paired runs in four concurrent MATLAB process shards, so absolute times include shared-machine contention. The consistent per-seed speedups support the prototype's value on this benchmark, but should not be generalized to other planners or machines without isolated reruns.
