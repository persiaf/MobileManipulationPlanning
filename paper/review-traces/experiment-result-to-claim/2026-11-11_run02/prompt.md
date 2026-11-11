# Independent result-to-claim and figure review prompt

Role: independent paper architect/reviewer, agent /root/joint_review. The selector was written by /root/arm_audit; the experiment runner by /root; the main-scene-style figures by /root/aloha_sources. Do not delegate the judgment to the implementation author.

User request: add task-constraint/navigation-cost jointly driven docking selection while preserving the same arm, scene and candidates; compare with selecting docks before navigation, preserving full task coverage and minimum docking count; provide visual analysis close to the existing main-scenario style.

Intended scoped claim: on the frozen VX300s/two-table/10-target/136-candidate scene and its fixed numerical IK support, lexicographic coverage -> minimum dock count -> computed directed navigation cost can improve the navigation objective without adding stops. Distinguish selection from visit-order effects with a fixed-set exact-reorder control. Do not assume all three raw path lengths improve.

Evidence: planning_strategy/joint/{enumerate_minimum_task_covers,solve_minimum_cover_route,plan_task_navigation_joint}.m; experiments/{prepare_joint_task_navigation,build_joint_navigation_graph,exp_joint_task_navigation}.m; results/joint_task_navigation_20261111/{metadata,candidate_pool,frozen_pair_support,minimum_covers,navigation_edges,summary,docking_poses,task_assignments,navigation_paths}; all figure PNGs and captions.

Review exactness, regional partition, necessary-edge completeness, directed/Inf cost handling, unchanged candidates/task support, exact assignments, route cost versus distance, fixed-set control, computational overhead, main-scene visual fidelity, claim scope and missing evidence. Independently enumerate all covers/tours in Python; inspect exported real paths and sampled physical base/table separation. Do not rerun large IK experiments or change execution code/raw evidence. The root later authorized a reusable independent CSV auditor at experiments/audit_joint_task_navigation.py; its generated audit is evidence, not independent IK solving.

Report claim_supported, what_results_support, what_results_dont_support, missing_evidence, suggested_claim_revision, next_experiments_needed, confidence and figure quality. No paper claim audit is currently available, so downstream paper wording remains provisional.
