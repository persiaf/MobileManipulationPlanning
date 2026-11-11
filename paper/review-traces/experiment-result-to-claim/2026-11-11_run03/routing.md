# Routing decision

- Confirm the scoped main-scene result: 36/36 numerical task coverage and 16/16 subpool-minimum stops in all 12 valid final cases; joint J decreases 21.55%, 14.12%, 19.34% relative to the shared corrected staged baseline and 12.28%, 4.69%, 9.77% relative to fixed-set reorder.
- Retain the initial symmetric run and its eight failed/BLOCKED cases separately. Never attribute common directed-matrix/first-finite-NN feasibility corrections to joint selection, or compute percentage gains against NaN failures.
- Preserve minimum_seed as a separate control. It does not improve stop counts or costs in this run. Preserve the fixed-set reorder control without treating heuristic improvements as guaranteed on every instance.
- State minimum-count scope as the shared 906-candidate regional numerical support subpool. All joint runs terminate at max_passes; claim neither global nor local optimality.
- Report L separately from endpoint-yaw-weighted J. Keep search callback counts separate from actual new lazy queries and all Stage 3 A* calls; no runtime, real-time or equal-compute advantage claim.
- Five main-scene-style PNGs/captions are acceptable. Preserve their actual geometry, support/physical-execution distinction, shared-correction note and timing scope.
- Root should record this scoped finding and its limits in local experiment notes. Paper wording remains provisional because `paper/PAPER_CLAIM_AUDIT.json` is absent. Broader novelty/generalization/physical-success claims require supplementary evidence; no new experiments are required for this bounded user-facing result.
- Reproduction: `python experiments/audit_main_task_navigation.py [RESULT_FOLDER]`. The reviewer wrote only the explicitly authorized auditor, its derived audit JSON and this run03 trace; no execution implementation or raw numerical evidence was changed by the review.
