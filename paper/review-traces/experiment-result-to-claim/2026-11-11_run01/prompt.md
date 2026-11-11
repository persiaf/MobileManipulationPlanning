# Independent result-to-claim review prompt

Date: 2026-11-11
Role: paper architect/reviewer, independent of experiment execution.

Judge the intended claim: in this frozen scene, task pose/path constraints change finite-search support, docking decisions and navigation cost; a position-only docking set may lose support under the constrained task definitions. Separately judge whether this establishes paper novelty or general superiority.

Review the exported raw data, protocol, metadata, diagnostics, independent CSV/geometric audit, RESULTS_ANALYSIS and the three rendered figures plus captions in results/task_constraint_comparison_20261111. Inspect the actual images. Do not treat reported verified_tasks or repeated evaluation by the same IK scorer as independent IK or physical execution evidence.

Required fields: claim_supported yes/partial/no; scoped_mechanism_claim_supported; paper_novelty_or_general_superiority_supported; what_results_support; what_results_dont_support; missing_evidence; suggested_claim_revision; next_experiments_needed; confidence; per-figure strengths and limitations.

Expected evidence to verify, not accept blindly: 136 common candidates after 144 raw candidates; 680 same-table pairs per mode; support 418/96/100; full task support 10/10 in all modes; docks 2/4/4; exact finite-pool set-cover minimum 2/4/4; valid round trips 11.6254834/14.3183766/11.7254834 m; workspace 183/26/36 of 189; matched path 49/11/11 of 55; transfer position->grasp/wipe 0/10, grasp->wipe 10/10, wipe->grasp 8/10.

Check key boundaries: one fixed scene and order, uncalibrated mounts, finite IK search, no arm collision or continuous-motion certificate, no measured wiping/force or grasp outcome. The upper workspace row plots anchor height z=0.5 m; grasp paths include points above the table. Grasp and wipe profiles are not nested. The matched control shares path, TCP and tolerances and changes only orientation freedom.

Only write this review trace directory. Do not edit raw data, execution code or project status; root will update EXPERIMENT_RESULT_TO_CLAIM.md. paper/PAPER_CLAIM_AUDIT.json is absent, so label the result provisional. The installed skill's shared-references/review-tracing.md and assurance-contract.md are also absent; preserve full prompt, response, normalized verdict and routing fields directly.

A separate read-only CSV checker was delegated to /root/arm_audit/numerical_crosscheck to recompute counts, uniqueness/completeness, task assignments, exact set cover, route dock visits and transfer/workspace values. The final reviewer judgment remains /root/arm_audit's.
