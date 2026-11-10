# Independent result-to-claim review task

Date: 2026-10-30. Role: independent paper architect/reviewer sub-agent research_evidence.

User questions: whether changing table count/orientation preserves task completion; how scene variables, dual-arm reachability and routes are computed; whether the project is sufficient and valuable as a master's research topic relative to modern mobile manipulation.

Task issued: read current code, experiment raw results and reports; do not accept report self-evaluations as evidence. Evaluate claims about docking/route coupling, footprint safety, candidate generation, and dynamic repair. Return claim_supported, what_results_support, what_results_dont_support, missing_evidence, suggested_claim_revision, next_experiments_needed, confidence. Distinguish master's research foundation from top-tier algorithm novelty. Cite raw CSV/log evidence and code locations. No production-code changes.

Evidence follow-up: current worktree has existing uncommitted navigation repair. Prioritize 2026-10-28 main_scene_repair_summary.log (36 tasks, 27 docks, 90.581503 m, valid footprint/endpoint checks) over obsolete 2026-10-27 default-scene failure. Investigate first-neighbor early exit and stale covered_tasks_predicted in plan_bidirectional.

Literature supplied after primary-source browsing: Reister et al. RA-L2022 (navigation/manipulation costs, docking, grasp and arm choice); MoMa-Pos2024 (inverse reachability/object kinematics); SMMP2025 (robot-scene A-space and whole-body planning); Dual-Arm Whole-Body2025 (overlapping kinematic chains); cuRoboV2 technical report2026; DockAnywhere2026 (docking viewpoint generalization).

Final evidence update: 13 targeted MATLAB tests passed; 4/8/16 mixed-angle scene builds passed. Current-worktree 4-table end-to-end test at [2 2;5 2;2 5;5 5] and [0,30,90,-45] degrees, seed2026, yielded 9/9 simulated geometric acceptance, 2289 candidates, 6 docks, 27.627060257 m including return, 13.556510 s runner time, valid sampled footprint and endpoint checks. Do not generalize one case to arbitrary layouts or physical execution. Save the complete reviewer response and this update without altering business code.
