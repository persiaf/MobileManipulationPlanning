# Result-to-Claim Review Prompt

Evaluate whether the completed large-scale diagnostics support the intended claim.

Intended claim: the S4 coverage drop and dominant path-planning cost can be localized without changing planner semantics, providing a defensible next research direction.

Experiments:

- Coverage diagnosis: S1-S4, seeds 1-5, shared candidate library and all three strategies.
- Path-cost audit: S1 and S4, seeds 1-3, all three strategies, directed scene/planner/pose keys.

Required judgment fields: `claim_supported`, supported and unsupported statements, missing evidence, revised claim, next experiments, confidence, and routing action. Do not inflate the claim.

Known caveats: no pre-library raw/collision/reachability counters; no cache implementation or runtime speedup experiment; no paper claim audit file.
