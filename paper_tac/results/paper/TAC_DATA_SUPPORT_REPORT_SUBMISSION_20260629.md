# TAC Data Support Report (SUBMISSION)

Generated on 2026-07-02 from the current MATLAB experiment pipeline.

## Data Roots

- Paper tables: `results/tables/paper_submission`
- TAC extension tables: `results/tables/tac_submission`
- Evidence index: `results/tables/tac_submission/tac_evidence_index_submission.csv`

## Evidence Index

| Evidence block | Paper target | Rows | OK | Failed | Claim supported |
|---|---:|---:|---:|---:|---|
| Experiment parameters | Table 1 | 12 | - | - | Reproducible setup and robot/task parameters. |
| Main scenario | Table 2, Figure 4 | 1 | - | - | Closed-loop feasibility on the 16-table main scene. |
| Candidate quality | Table 3, Figure 3 | 5 | - | - | Selected docking poses should have stronger coverage and reachability than rejected candidates. |
| Candidate detail | Supplementary statistics | 14527 | - | - | Candidate-level samples for selected/rejected and nearest/top-cost analyses. |
| Hard scenarios | Table 4 | 5 | - | - | Robustness under normal, corner, two-side, reach-boundary, and mixed task distributions. |
| Main ablation | Table 5, Figure 5 | 7 | - | - | Necessity of reachability, coverage cost, and multi-docking in the main scene. |
| Hard ablation | Table 5, Figure 5 | 7 | - | - | Necessity of the proposed components under hard-mixed distributions. |
| Random scale | Table 6 | 3 | - | - | Scalability across small, medium, and large randomly generated scenes. |
| Planner benchmark | Table 7 | 7 | - | - | Modularity of the global routing layer across multiple planners. |
| Baseline comparison | Extended Table 5 | 1120 | 1100 | 20 | Multi-trial comparison against nearest, single-docking, random, set-cover, and partial-cost baselines. |
| Stress boundary | Appendix/Discussion | 17 | - | - | Sensitivity to table spacing, reachability threshold, task density, and candidate density. |
| Runtime scaling | Figure 6 | 9 | - | - | Runtime behavior under scale and candidate-density changes. |
| Noise robustness | Appendix/Discussion | 30 | - | - | Perturbation evidence for task noise and docking pose noise. |
| Oracle gap | Appendix/Discussion | 3 | - | - | Small-scale optimality-gap evidence against exhaustive set-cover/TSP oracle. |

## Key Result Numbers

### Main Scenario

- Tasks completed: 36/36, completion rate 100.00%.
- Selected docking poses: 27; total path length 63.29 m; planning time 28.85 s.

### Candidate Quality

- Candidate-quality summary rows: 5.
- Selected candidates: coverage 0.604, reach 0.310; rejected candidates: coverage 0.094, reach 0.038.

### Main Ablation

- Full method: 100.00% completion with 0 uncovered tasks.
- Single docking per table: 69.44% completion with 11 uncovered tasks.
- Nearest docking: 50.00% completion with 18 uncovered tasks.

### Extended Baselines

- Detail records: 1120.
- OK records: 1100; failed records: 20.

### Stress Boundary

- Stress summary rows: 17.
- Lowest mean completion: 96.92% at reach_threshold = 0.5.

### Runtime Scaling

- Runtime summary rows: 9.
- Planning-time range: 1.92 to 60.02 s.

### Noise Robustness

- Noise summary rows: 30.
- Full method final completion range: 69.03% to 100.00%.
- Nearest-docking baseline final completion range: 0.00% to 0.00%.

### Oracle Gap

- Oracle summary rows: 3.
- Relative gap range: -4.56% to 0.63%.
- Negative gaps indicate that this small-scale oracle is a bounded candidate-set reference, not a formal global optimality proof.


## TAC Writing Guidance

- Main text should emphasize constrained hierarchical decision making: task coverage and dual-arm reachability are feasibility constraints before path shortening.
- The strongest evidence is the completion-rate gap between full multi-docking and nearest/single-docking baselines.
- Runtime and stress scans should be used to discuss computational scaling and constraint sensitivity, not as the central novelty claim.
- LNN results are intentionally excluded from the main evidence chain for this TAC draft.

## Recommended Next Submission Run

For a formal submission table set, run:

```matlab
run_tac_data_support('submission')
```

The current `submission` data are suitable for drafting, figure design, and deciding which claims are already supported.
