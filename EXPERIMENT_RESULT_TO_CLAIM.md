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

## 2026-09-01 coverage-preserving candidate augmentation

Status: provisional, `[pending external review]`.

- Verdict: `partial`.
- Confidence: `medium`.
- Paper claim audit: unavailable (`paper/PAPER_CLAIM_AUDIT.json` is not present).
- Scope: S4 seeds 1-5 with the current scene generator, robot geometry, collision checks, reachability threshold, and planner implementation. S1-S3 seeds 1-5 provide no-op checks only.

### Geometry and candidate-library evidence

Across the five S4 scenes, 133 candidate-task support pairs cover center-row tasks, and all 133 come from Regions 9-12. Every supporting pose lies on table side `+Y`, uses the minimum tested normal offset of 0.25 m, has 0.25 m footprint clearance, and is aligned tangentially with the target task (projection error from -0.09 m to 0.09 m). Candidate-to-task distance is 0.454-0.459 m. Heading relative to the task ranges from -0.418 rad to 0.366 rad.

Collision-valid poses in the failed regions remain farther from the table: their minimum normal offset is 0.30 m and median is 0.35 m, while the reference non-support median is 0.40 m. The strongest observed factor is therefore task-aligned position at the minimum collision-valid normal offset. A reasonably task-facing orientation is compatible with success, but the ablation below shows that orientation refinement alone is insufficient and Joint refinement is not necessary on these scenes.

| Variant | Mean library coverage | Minimum coverage | Recovered prior failures | Mean added candidates | Mean supplementation time | Mean total candidate time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Baseline | 74.433% | 71.545% | 0/154 | 0.0 | 0.571 s | 11.684 s |
| Position-only | 100.000% | 100.000% | 154/154 | 25.2 | 7.810 s | 18.923 s |
| Orientation-only | 74.433% | 71.545% | 0/154 | 0.0 | 58.110 s | 69.223 s |
| Joint | 100.000% | 100.000% | 154/154 | 25.2 | 21.148 s | 32.261 s |

Position-only matches Joint coverage and selected-candidate count while requiring substantially less supplementation time. The evidence therefore favors Position-only as the simpler tested variant; it does not support a claim that Joint position-orientation augmentation is necessary or superior.

On S1, S2, and S3 seeds 1-5, baseline coverage is already 100%, coverage remains 100%, and the augmentation selects zero candidates. Observed no-op overhead is approximately 0.05-0.18 s per scene.

### Full-strategy and combined evidence

Joint augmentation was used for the full S4 strategy experiment:

| Strategy | Mean/min coverage | Verified completion | Mean selected docks | Mean greedy requirement | Mean total path | Mean total time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Bottom-Up | 100% / 100% | 100% | 49.8 | 49.2 | 77.689 m | 202.95 s |
| Top-Down | 100% / 100% | 100% | 49.2 | 49.2 | 103.140 m | 521.71 s |
| Bidirectional + cache | 100% / 100% | 100% | 49.8 | 49.2 | 77.689 m | 173.41 s |

The original incomplete-library baseline selected a mean of 35.8 docks. Restoring full coverage raises the greedy requirement to 49.2 and selected docks to approximately 49-50, so the coverage recovery is not free. Path length and objective values from incomplete and full-coverage runs cover different task sets and do not establish route-quality improvement.

Within the augmented Bidirectional experiment, cache OFF and ON produce exactly identical coverage, verified completion, selected docks, total path length, and objective for every seed. The cache reduces mean expensive calls from 5219.6 to 1293.0 and measured mean time from 698.63 s to 173.41 s, a 4.03x within-augmentation speedup.

| Configuration | Mean coverage | Mean expensive calls | Observed mean time |
| --- | ---: | ---: | ---: |
| Original baseline, cache OFF | 74.433% | 2745.6 | 295.33 s |
| Augmentation, cache OFF | 100.000% | 5219.6 | 698.63 s |
| Original baseline, cache ON | 74.433% | 679.6 | 79.394 s |
| Augmentation, cache ON | 100.000% | 1293.0 | 173.41 s |

Relative to the original cache-OFF baseline, the combined configuration reaches 100% coverage with 52.9% fewer expensive calls. Its observed time is 41.3% lower (1.70x), but this timing comparison is provisional: the original and augmented measurements were collected in separate batches, and the augmented runs used three concurrent MATLAB shards. Call counts and the paired augmented cache-OFF/cache-ON semantic identity provide stronger evidence than this cross-batch wall-clock comparison.

### Supported claim and routing

The results support the following scoped statement:

> On the five tested S4 scenes, task-aligned positional augmentation (and Joint augmentation) increased library and verified coverage from a 74.43% mean to 100% without relaxing collision or reachability checks. The independent shared lazy cache preserved all reported planning outputs and reduced augmented expensive calls from 5219.6 to 1293.0. The combined configuration achieved 100% coverage with fewer calls than the original cache-OFF baseline; its observed cross-batch timing advantage remains provisional.

The results do not support generalization to other layouts, scales, planners, or hardware; a claim that Joint augmentation is necessary; a route-quality improvement claim across unequal covered-task sets; or a claim that coverage recovery has no docking or generation cost. Before a general paper claim, run isolated, randomized four-mode timing repeats, compare Position-only and Joint in the full strategy pipeline, extend S4 layouts/seeds, and record augmented-cache memory use.

Evidence files remain generated under `results/tables/` and intentionally outside the code commits.

## 2026-09-01 Position-only final validation

Status: provisional, `[pending external review]`.

- Verdict: `partial`.
- Confidence: `medium`.
- Paper claim audit: unavailable (`paper/PAPER_CLAIM_AUDIT.json` is not present).
- Scope: frozen Position-only parameters; development scenes S4 seeds 1-5; held-out S4 seeds 6-20; current planner and scene generator.

### Full-strategy paired validation

Using the same generated scene, candidate inputs, planner, and robot configuration for each seed, Position-only and Joint were compared in Bottom-Up, Top-Down, and Bidirectional planning. All 30 completed strategy runs (5 seeds x 2 variants x 3 strategies) reached `100%` library coverage and `100%` verified completion. Mean added candidates were 25.2 for both variants. Position-only had lower mean candidate-generation time (`13.836 s` versus `24.884 s`), saving `11.048 s` per strategy-seed pair on average. Mean total time was `244.501 s` for Position-only and `253.573 s` for Joint, but total planning time and path length should not be treated as a universal quality comparison because the selected poses and resulting routes differ.

The per-seed Position-only strategy results were:

| Seed | Bottom-Up | Top-Down | Bidirectional |
| ---: | --- | --- | --- |
| 1 | 100% / 100%, 50 docks | 100% / 100%, 50 docks | 100% / 100%, 50 docks |
| 2 | 100% / 100%, 50 docks | 100% / 100%, 50 docks | 100% / 100%, 50 docks |
| 3 | 100% / 100%, 49 docks | 100% / 100%, 48 docks | 100% / 100%, 49 docks |
| 4 | 100% / 100%, 53 docks | 100% / 100%, 53 docks | 100% / 100%, 53 docks |
| 5 | 100% / 100%, 47 docks | 100% / 100%, 46 docks | 100% / 100%, 47 docks |

Here each entry is `coverage / verified completion, selected docks`. The result satisfies the formal Position-only success criterion on all five development seeds. It does not show a strategy-level coverage gap after augmentation.

### Held-out coverage validation

The frozen Position-only configuration was evaluated on 15 held-out S4 scenes (seeds 6-20), without parameter changes based on held-out outcomes. Baseline coverage ranged from `71.774%` to `79.832%`, with mean `75.421%`. Position-only reached `100%` coverage on all 15 scenes, recovering every baseline-uncovered task in each scene; the minimum held-out coverage was `100%`, and the mean selected supplementary count was `24.8`. No held-out failure geometry was recorded.

This supports the narrower claim that task-aligned positional augmentation consistently recovered candidate-library coverage on these tested held-out random scenes. It does not establish performance on arbitrary layouts, other planners, or other hardware.

### Isolated four-mode timing status

The single-process benchmark entry point implements the requested M0-M3 design, warm-up, deterministic per-seed randomized order, scene reuse, run-local cache lifetime, and per-pair semantic assertions. However, execution was stopped after the first completed pair because the S4 path matrix cost was substantially higher than the other experiments. The completed record is only:

| Seed | Repeat | Mode | Coverage | Expensive calls | Total time |
| ---: | ---: | --- | ---: | ---: | ---: |
| 1 | 1 | Baseline OFF | 71.545% | 2775 | 287.385 s |

No M1-M3 timing aggregate, paired speedup, cache-memory summary, or same-batch M0-versus-M3 runtime claim is available. The earlier cross-batch cache numbers remain provisional and are not upgraded by this partial run. The claim therefore remains `partial / medium` until an isolated batch completes.

### Routing

Adopt Position-only as the primary candidate augmentation variant for subsequent controlled work; retain Joint only as an ablation and Orientation-only as a negative ablation. Keep the shared cache as a separate Bidirectional optimization. The next evidence needed is a completed isolated M0-M3 batch with at least three repeats or a documented, resource-bounded reduced design, plus cache memory and paired speedup statistics. Do not update a general paper claim or claim a definitive combined runtime improvement yet.

## 2026-09-02 resource-bounded isolated four-mode timing

Status: `yes / high` for the two claims within the tested configuration; provisional - no paper claim audit available.

The completed single-compute-process batch used S4 seeds `[1, 3, 5]`, two repeats, and 24 runs. Each seed's scene and Baseline/Position-only candidate libraries were generated once and shared across modes and repeats. Mode order was deterministically randomized per seed-repeat pair, each mode received a new run-local cache, and candidate-generation time was measured once per seed and reused across repeats. Two repeats were used because the measured cache-OFF modes made the originally proposed 36-run design unreasonable under the available resource budget.

| Mode | Coverage mean | Docks mean | Calls mean | Candidate time mean | Total time mean / median |
| --- | ---: | ---: | ---: | ---: | ---: |
| M0 Baseline, cache OFF | 74.058% | 36.0 | 2775.0 | 8.276 s | 298.739 / 290.660 s |
| M1 Baseline, cache ON | 74.058% | 36.0 | 685.3 | 8.276 s | 79.275 / 79.703 s |
| M2 Position-only, cache OFF | 100.000% | 48.7 | 4986.3 | 14.012 s | 562.334 / 561.454 s |
| M3 Position-only, cache ON | 100.000% | 48.7 | 1232.3 | 14.012 s | 151.363 / 151.208 s |

M0/M1 and M2/M3 were identical in all six pairs for coverage, verified completion, selected docks, visit order, total path, and objective. M2 to M3 reduced expensive planner calls by `75.285%` on average and produced a mean/median paired speedup of `3.713x / 3.686x` (`3.600x` to `3.826x`). M1 cache memory was `2,751,590` bytes mean and `2,764,810` bytes maximum; M3 was `4,981,290` bytes mean and `5,190,927` bytes maximum.

Within the same batch, M0 to M3 increased coverage in every pair and reduced wall-clock time in every pair. Mean coverage increased by `25.942` percentage points and mean/median paired speedup was `1.973x / 1.992x` (`1.802x` to `2.121x`). This supports a combined Position-only-plus-cache claim, not a claim that augmentation alone reduces runtime. M0 to M2 added 11-14 docks, 1881-2478 planner calls, 4.967-6.872 s candidate time, and 230.4-283.8 s total time; coverage recovery is therefore not free.

The full query-trace audit caused MATLAB R2022a heap corruption during initial attempts. The completed batch used an aggregate count-only audit that preserved the expensive-call event definition without retaining per-query strings or structures. The result is limited to the tested S4 seeds, current Bidirectional implementation, current `convex_astar`, current hardware/software environment, and six pairs. It does not support path-quality superiority across M0 and M3, general speedup claims, or independent candidate-time repeat statistics.

The scoped supported statement is:

> For the tested S4 scenes with seeds 1, 3, and 5, using the current Bidirectional planner and `convex_astar` on the evaluated hardware, the run-local shared cache preserved all checked planning outputs while reducing expensive planner calls by approximately 75% and yielding about 3.7x mean paired speedup. In the same batch, frozen Position-only augmentation combined with caching achieved 100% coverage and lower observed wall-clock time than the uncached baseline in all six paired trials, with about 2.0x mean speedup.

Routing: retain Position-only as the formal augmentation and the shared cache as a separate optimization. Treat these two narrowly scoped claims as confirmed experimental evidence, but keep downstream paper wording provisional until a paper claim audit verifies manuscript-to-result fidelity. Broader claims require more seeds, independent sessions or hardware, uncertainty intervals, and additional scales.

## 2026-10-30 current-worktree scene and research assessment

- Intended claim: robustness to table-layout changes and defensible master's-level docking/navigation research contributions.
- Independent paper architect/reviewer verdict: `partial`; confidence `medium`; routing `supplement`. No paper/PAPER_CLAIM_AUDIT.json exists; the judgment remains provisional. This is independent AI review, not institutional graduation approval.
- New evidence: 13 targeted MATLAB tests passed; 4/8/16-table mixed-angle scene builds passed. A complete current-worktree four-table case with angles `[0,30,90,-45]` degrees and seed2026 succeeded: 9/9 simulated geometric task acceptance, 2289 candidates, 6 docks, 27.627060257 m including return; sampled rectangular-footprint and endpoint/assembly validation passed. Runner elapsed time was 13.556510 s including result saving, not a repeated timing benchmark.
- Sources: `tmp/user_assessment_20261030/verification.log`; `tmp/user_assessment_20261030/rotated_four_tables/pipeline.log`; associated reproduction scripts and MAT records. The earlier `results/logs/main_scene_repair_20261028/main_scene_repair_summary.log` confirms the repaired default 16-table scene; October 27 default routing failures do not describe that later repaired case.
- Supported: particular current-version layout changes can complete high-level geometric planning. Historical candidate and cache benefits remain restricted to their tested versions. The project provides a viable master's research foundation.
- Not supported: arbitrary-layout guarantees, measured manipulation success, joint-level dual-arm collaboration, general bidirectional improvement, or state-of-the-art superiority. Geometric max(left,right) and the simulated verified field are not IK/physical execution.
- Missing evidence: independent IK/motion/collision validation, current-version held-out layout experiments, fair strong baselines, validated bidirectional improvement, failure-inclusive dynamic recovery costs.
- Routing: keep the topic, narrow the central claim to coverage-constrained multi-docking and navigation, then supplement evidence. This review did not alter production algorithms or launch broad parameter sweeps.
- Detailed code/data/literature mapping: `docs/SCENE_PIPELINE_RESEARCH_REVIEW_20261030.md`. Full review trace: `paper/review-traces/experiment-result-to-claim/2026-10-30_run01/`.

## 2026-11-11 task-conditioned arm, docking and navigation mechanism

- Source: `results/task_constraint_comparison_20261111/`; protocol, raw CSVs, effective task settings, MAT snapshots, vector figures, independent CSV/geometry audit and reproducible analysis are retained together.
- Intended claim: in one frozen VX300s/two-table scene, changing task pose/path requirements changes numerical support, docking choices and navigation cost; a position-only docking set can lose task support under constrained manipulation requirements.
- Independent paper architect/reviewer verdict: **yes for the scoped mechanism; no evidence for paper novelty or general superiority**. Confidence high only for bounded descriptive counts and exported routes. Provisional: `paper/PAPER_CLAIM_AUDIT.json` is unavailable. This is independent AI review, not human peer review or independent IK execution validation.
- Fixed: same arm chain, uncalibrated mounts and TCP, 10 task anchors, 136 footprint-safe candidates (144 raw), 680 same-table candidate–task pairs per mode, A*, nearest-neighbor routing, weights and return-to-start condition.
- Position/grasp/wipe results: pair support **418/680, 96/680, 100/680**; selected docks **2, 4, 4**; each supports and rechecks **10/10** tasks with the same IK evaluator. Round-trip routes are **11.6254834, 14.3183766, 11.7254834 m**, all base-route checks passed. Exact set-cover DP on the exported binary matrices gives minimum dock counts **2, 4, 4**; this is neither a continuous-space nor route-optimality certificate.
- Fixed-reference-base workspace: **183/189, 26/189, 36/189** task anchors supported. A matched 12 cm path control varying only orientation freedom gives **49/55 none, 11/55 axis, 11/55 full**, with no observed numerical nesting violation. It shows an orientation effect but no further full-pose support loss in this sample. Main grasp and wipe paths differ and do not form a strict inclusion chain.
- Plan transfer: the position-only docking set supports **0/10** grasp and **0/10** wipe tasks under the constrained evaluator; the grasp set supports 10/10 wipe tasks; the wipe set supports 8/10 grasp tasks. Counts allow reassignment inside each docking set and are not physical success rates or impossibility proofs.
- Verification: **7/7** new fairness/protocol tests passed. Independent CSV/geometry audit passed for shared candidate identity, pair statistics, residual bounds, coverage, recomputed path lengths, sampled physical base/table separation, return, DP and transfer counts; reviewer delegated crosscheck additionally verified complete unique pairs, exact task assignments and all dock/yaw visits. No independent IK solving or continuous arm-motion validation occurred.
- Limits: one deterministic scene, fixed-order single timing observations, finite IK budget, uncalibrated installation, no arm/self/inter-arm collision, between-sample or inter-task arm motion, contact force, cleaning outcome or measured execution. Do not claim generic efficiency, robustness, workspace volume, new pose-constraint representation or universal route-cost increase.
- Routing: retain this controlled mechanism evidence; supplement with collision-aware independent validation, held-out layouts/heights/path directions, strong equal-budget planning baselines and component ablations before broader contribution claims. For `paper/PAPER_PLAN.md`, this can inform method/task-feasibility and mechanism sections, but it does not complete the existing ROS/lift/recovery experiments or authorize confirmed full-paper claims.
- Full review trace: `paper/review-traces/experiment-result-to-claim/2026-11-11_run01/`. Main explanation: `results/task_constraint_comparison_20261111/RESULTS_ANALYSIS.md`.

## 2026-11-11 joint task-support and navigation selection

- Source: `results/joint_task_navigation_20261111/`. Same VX300s, uncalibrated mounts/TCP, two-table scene, 10 targets, 136 candidates and frozen task-support matrix as the preceding mechanism comparison. No change to IK evidence is used to manufacture a navigation gain.
- Intended claim: within that frozen support pool, joint selection of minimum-count docking covers and their visit order can reduce the existing navigation objective while preserving all task support and the minimum number of docks.
- Independent reviewer verdict: **yes for the bounded weighted-navigation improvement; no evidence for generic superiority or paper novelty**. Confidence high for independently reconstructed matrix/graph/CSV results only. Provisional: no manuscript claim audit is available.
- Compared methods: original staged bottom-up + nearest neighbor; fixed staged candidate-set exact reordering; joint minimum-cover + visit-order enumeration. Objective is `J = L + 0.15 m/rad * sum(abs(wrapped service-endpoint yaw changes))`; it is not integrated path turning effort or energy.
- All nine cases retain **10/10** supported/checked tasks and the certified minimum dock counts **2/4/4**, pass base-route validation and return to the same start.
- Staged -> joint weighted costs: **12.0967223 -> 9.6413929**, **15.2608544 -> 15.0514149**, **12.3014421 -> 10.9360621**; reductions **20.30%, 1.37%, 11.10%**. Raw route lengths: **11.6254834 -> 9.2748737 m**, **14.3183766 -> 14.3183766 m**, **11.7254834 -> 10.4648232 m**. Grasp distance does not decrease; its gain is in the endpoint-heading penalty.
- Reorder-only weighted costs are **12.0967223, 15.2608544, 12.1967223**, with unchanged raw lengths. The joint improvement beyond this control supports a candidate-pose selection effect, not only better route permutation.
- Independent Python audit rebuilt all minimum covers, the **2,292** necessary directed real-path edges, all **512/1,944/18,816** joint tours, fixed-set exact orders, exactly-once legal task/arm assignments, actual path visits and headings, return conditions, sampled physical footprint safety and weighted/raw metrics. Its optima agree with the MATLAB implementation; audit PASS. Base routes are recomputed by the existing Stage 3.
- Tests: **34 distinct targeted checks** passed across runs (17 joint strategy contracts, 7 fixed-protocol checks, 10 existing model contracts); not a full repository suite. The one-time shared real-path graph took **29.061 s**, excluded from the plotted search-only times. No end-to-end speedup, significance or real-time claim is supported.
- Exactness is bounded to the supplied disjoint region-task partition, candidate ownership, binary minimum-cover layer and computed directed navigation graph. Each selected service dock is visited once in a Hamiltonian return tour. No finite minimum-layer tour returns BLOCKED; it does not silently add docks or search repeated-service walks. Budget overflow fails explicitly.
- Limits: single deterministic scene, finite IK support, no independent arm trajectory/collision/transition/contact/physical execution validation; no continuous-space or arbitrary large-scene optimum, scalability or novelty claim. Figures follow the main-scenario style while retaining the same two-table experiment geometry.
- Routing: retain the exact bounded selector as an available strategy/reference; next compare strong navigation-aware heuristics, scalable variants, held-out layouts/path directions and candidate/IK budgets, with full preprocessing accounting and collision-aware validation. Existing paper plans may cite this mechanism/selection evidence; ROS/recovery/hardware studies remain unrun.
- Full trace: `paper/review-traces/experiment-result-to-claim/2026-11-11_run02/`; report/reproduction: `results/joint_task_navigation_20261111/ANALYSIS.md`; independent auditor: `experiments/audit_joint_task_navigation.py`.

## 2026-11-11 full default main-scene task/navigation comparison

- Source: `results/main_joint_task_navigation_20261111/`; the unchanged default 10 x 10 m layout, 16 tables and all 36 seed-2026 tasks. The same optional VX300s model, uncalibrated mounts and TCP are retained. Geometry-only sampling freezes 906 shared candidates from 26,660 raw / 8,692 footprint-valid poses before task IK. This is a budgeted subpool, not the full dense pool.
- Intended claim: in this main scene, jointly changing task-supported dock poses and their visit order can reduce navigation cost while retaining all task support and the regional finite-subpool minimum dock count.
- Independent reviewer: **yes for these scoped observed reductions; no evidence for broad superiority, novelty, physical success or continuous-space optimality**. The claim remains provisional without a manuscript claim audit. Raw-data/geometry/figure review is not independent IK or hardware validation.
- All twelve final cases support and recheck **36/36** tasks, retain **16 docks / subpool minimum 16**, pass base-route validation and return to the same start. The staged seeds already have minimum cardinality, so seed repair contributes no dock-count reduction in this scene. Pair supports are **1,420 / 472 / 493 out of 2,037** for position/grasp/wipe.
- Shared navigation correction is separated from the selection claim: the initial historical mirrored matrix and single NN order fail for grasp and wipe, with **4/12** valid missions overall. That complete run and failures remain in `results/main_joint_task_navigation_20261111_initial_symmetric/`. All final methods use independently queried directed costs and, only if NN1 is invalid, the first finite NN restart in fixed node order. This common recovery preserves the selected set and is not counted as a joint-selection gain. Immutable inputs, support CSVs and library MATs match the initial run by SHA256.
- Against this commonly corrected staged baseline, position/grasp/wipe routes are **53.9655375 -> 42.1909401 m**, **62.3508592 -> 52.8283516 m**, **62.8147529 -> 50.6432203 m**: reductions **21.82%, 15.27%, 19.38%**. Weighted J values are **55.5363338 -> 43.5702945**, **64.3866614 -> 55.2944162**, **65.7388605 -> 53.0245267**: reductions **21.55%, 14.12%, 19.34%**. J uses actual path length plus 0.15 m/rad times wrapped service-endpoint yaw differences, not integrated turning, energy or execution time.
- Fixed-set reorder-only routes are **48.1793939 / 55.9810107 / 55.8436851 m**. Joint routes improve by a further **12.43% / 5.63% / 9.31%**; weighted J improves by **12.28% / 4.69% / 9.77%**. This supports a benefit of allowing pose selection and ordering together in this run, not an isolated main effect or equal-compute superiority proof.
- Independent audit PASS: original scene/targets, geometry subpool construction, all same-table pairs and cached-witness counters, 48 regional DP/cover counts, first-finite recovery from each full directed D, legal exactly-once assignments, all accepted strict-cost-decrease moves, actual route metrics/visits/return and sampled base safety. Five figure sets and their manifest hashes were independently checked; the diagrams use the actual main-scene geometry/style.
- **54 distinct targeted tests** passed, not a full repository suite. Joint search uses at most three passes / 1,500 external callback queries / 10,000 neighbors / 120 s; all three final joint runs stop at max_passes and have no global or local optimum certificate. A callback can hit the shared memo; neither callback counts nor memo edges represent all Stage-3 A* calls. Search-only timings exclude IK preprocessing and route rebuilding and have shared-cache/order bias, so no end-to-end speedup or real-time claim is supported.
- Limits and routing: one deterministic layout/task set and one geometric density; finite sampled IK with current-FK cache witnesses, no calibrated mounts, arm/self/inter-arm/table collision, between-task/continuous arm trajectories, force or measured task execution. Retain the bounded main-scene evidence; supplement with held-out layouts/task paths, candidate/IK/search budgets, stronger equal-budget navigation-aware baselines and independent collision-aware manipulation validation. Existing ROS/recovery/hardware paper studies remain unrun.
- Full trace: `paper/review-traces/experiment-result-to-claim/2026-11-11_run03/`; reproducible analysis and figures: `results/main_joint_task_navigation_20261111/ANALYSIS.md`; entry: `experiments/exp_main_task_navigation.m`; auditor: `experiments/audit_main_task_navigation.py`.

## 2026-11-11 nine-layout/distribution task-navigation stability study

- Source: `results/task_navigation_robustness_20261111/`. Public pre-study backup `fee9967`; execution source `05c630b`. The predeclared 3-layout x 3-distribution grid retains the same VX300s configuration, 16 tables and 36 task IDs/counts. Each layout shares one reference candidate pool across distributions and modes: 906 / 906 / 913 poses.
- Reviewer verdict: **partial**, confidence high for the exported results. Joint improvement over staged planning is widespread; an advantage over reordering is conditional. Full trace: `paper/review-traces/experiment-result-to-claim/2026-11-11_run04/`.
- All **108 raw / 81 primary results** verify 36/36 tasks, pass base-route checks and attain the regional support-matrix minimum of **16 or 32 docks**. All 27 common paired groups are eligible. Raw staged also reaches the minimum in this batch; common cover repair does not reduce its stop count.
- Joint vs staged: **26 wins, 1 tie, 0 losses**. Joint vs 3-pass fixed-set reordering: **24 wins, 0 ties, 3 losses**. Position/grasp/wipe median J reductions are **16.46% / 14.12% / 14.67%** against staged and **14.64% / 8.19% / 9.77%** against reordering.
- The three reordering losses are L0_D2 grasp (**-4.9400%**), L1_D2 wipe (**-0.0043%**), and L2_D2 wipe (**-1.7173%**). Each retains full coverage and 32 minimum docks. L0_D2 grasp stops at 120 s with ten accepted regional replacements and no accepted reorder; L2_D2 wipe spends its 1,500 queries without an accepted move and returns the seed.
- The L0 grasp regional replacement neighborhood grows from **73 to 12,000** options when D0 changes to D2; wipe grows **91 to 10,732**. These observations identify search allocation at larger docking cardinality as a concrete method-development target.
- A separately labeled post-hoc graph diagnostic runs best-improvement directed 2-opt from the common fixed-set seed. Joint wins **21/27**, loses six. It follows a different trajectory and does not dominate the main reordering control in every case. It adds no IK/A* calls or complete-plan rows. Sixteen main reordering endpoints still admit a 2-opt improvement, so the primary comparison does not isolate a pure pose-selection effect.
- Independent result audit PASS covers 108 rows, 3,888 assignments, 545 accepted moves and 637,253 path samples. Seventeen new targeted checks passed. Main-scene-style figures, all cases and derived tables are retained with the analysis.
- Next step: initialize joint search with more complete fixed-set reordering, charge that initialization to the common budget, then screen regional covers and reserve reordering budget. Test this revision against a predeclared fuller reordering control before claiming a pose-selection advantage. Update the task/docking/navigation mechanism and experiment sections; retain the current nine-case results as development evidence.
