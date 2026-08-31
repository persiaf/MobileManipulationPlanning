# Large-Scale Baseline Results (2026-08-31)

## 1. Experiment Configuration

The completed batch contains 4 scales x 5 seeds x 3 strategies = 60 rows. The global planner is the configured `convex_astar`; all tasks use one operating height. No lift planning, lift timing, candidate pruning, lazy path-cost evaluation, or strategy changes were introduced.

Actual task counts were: S1=36, S2=64, S3=87.8 median (83-90 across seeds), and S4=120.4 median (117-123). The requested S2/S3 targets are recorded separately because table-wise random sampling produces nearby actual counts.

## 2. Data Integrity and Success

All 60 strategy runs completed successfully. No failure rows were observed, no timing value was negative, and cache-hit ratios remained in [0, 1]. S1-S3 predicted coverage was 100% for every strategy. S4 coverage was 74.4% for all three strategies, so scalability did not preserve full coverage at the largest tested scale.

## 3. Scale Summary

| Scale | Actual tasks (median) | Candidates (median) | Docks (median) | Bottom-Up T_total mean | Top-Down T_total mean | Bidirectional T_total mean |
|---|---:|---:|---:|---:|---:|---:|
| S1 | 36.0 | 3647 | 18.0 | 13.50 s | 100.46 s | 39.43 s |
| S2 | 64.0 | 7566 | 32.0 | 62.72 s | 358.51 s | 220.57 s |
| S3 | 87.8 | 6688 | 32.0 | 59.72 s | 337.53 s | 214.12 s |
| S4 | 120.4 | 7363 | 35.8 | 72.07 s | 353.81 s | 262.45 s |

Means, medians, and standard deviations are available in `results/tables/large_scale_complex_tasks_summary.csv`.

## 4. Stage Timing

Median timing shares over all scales were:

| Strategy | Candidate | Path cost | Strategy | Route | Scene | Execution simulation |
|---|---:|---:|---:|---:|---:|---:|
| Bottom-Up | 10.9% | 86.6% | 0.3% | ~0% | 0.1% | ~0% |
| Top-Down | 2.0% | 15.4% | 81.9% | ~0% | ~0% | ~0% |
| Bidirectional | 3.1% | 24.0% | 72.2% | ~0% | ~0% | ~0% |

`T_total` is an inclusive wall-clock measure (`scene + candidate + strategy + stage3`). Stage fields are diagnostic components and must not be summed as if they were all exclusive; strategy timing includes its internal path-cost work.

## 5. Path-Cost and Cache Analysis

Median path-cost query counts grew from 342 (S1) to 1318 (S4) for Bottom-Up/Bidirectional, while Top-Down grew from about 2747 to 5696 queries. Top-Down had a cache-hit ratio of 0.165-0.207; Bottom-Up and Bidirectional stage3 measurements report zero cache hits because stage3 uses the existing uncached distance-matrix routine. This means the current cache is effective inside Top-Down, but it does not cover all path-cost consumers.

## 6. Scaling Indicators

Using medians and a descriptive log-log fit over the four scales:

| Strategy | Task growth | Candidate growth | Query growth | T_total growth | alpha | R2 |
|---|---:|---:|---:|---:|---:|---:|
| Bottom-Up | 3.361x | 2.014x | 3.895x | 5.362x | 1.355 | 0.805 |
| Top-Down | 3.361x | 2.014x | 2.094x | 3.528x | 1.032 | 0.743 |
| Bidirectional | 3.361x | 2.014x | 3.895x | 6.716x | 1.547 | 0.810 |

These alpha values are descriptive scalability indicators, not algorithmic complexity proofs.

## 7. Bottleneck Diagnosis

The dominant runtime component differs by strategy: Bottom-Up is dominated by stage3 path-cost computation, while Top-Down and Bidirectional are dominated by their strategy-level work. The fastest-growing measured total time is Bidirectional, followed by Bottom-Up; Top-Down has the largest absolute time at S2-S4. Candidate count grows only about 2.0x from S1 to S4, so candidate volume alone does not explain the full runtime increase. The data supports multiple bottlenecks rather than one universal bottleneck.

The S4 coverage drop is a separate feasibility signal and should not be attributed to runtime without additional evidence.

## 8. Next Optimization Priority (Not Implemented)

1. Profile and reduce repeated stage3/path-cost computation, especially the uncached distance matrix.
2. Evaluate broader incremental/cache reuse across strategy and stage3 consumers.
3. Investigate region-level coarse-to-fine candidate handling while preserving coverage.
4. Bound Bidirectional refinement work with an evidence-based budget.

No optimization was implemented in this stage.
