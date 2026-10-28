# Development History

| Theme | Problem | Change | Current status |
|---|---|---|---|
| Original baseline | Need a reference | Preserved Bottom-Up staged pipeline | Maintained |
| Position-only augmentation | Candidate coverage gaps | Added task-aligned candidates | Large-scale studies |
| Shared Lazy Cache | Repeated path queries | Added run-local cache | Benchmark-specific |
| Parameterized scenes | Fixed scenes limited scale | Added profile-driven scenes | Maintained |
| Operability accounting | Task states conflated | Separated predicted/commanded/executed/verified | Current semantics |
| Sparse routing | Dense graph cost | Added sparse_knn and validation | Current option |
| P5 local replanning | Dynamic changes | Added D1/D2/D3 workflow | Current validation |
| Restaurant / alternate physical model | Different model and coverage semantics | Removed dedicated scenes, proxies, parameters, tests and outputs on 2026-10-27 | Removed; see main_robot_unification.md |
| One main-scene robot | Prevent parameter mixing | All maintained scenes use robot_config; conflicting overrides fail | Maintained |
| Geometry consistency, 2026-10-27 | Rotated raster/footprint axes, candidate arguments, stale scene mapping | Unified table/world transforms, exact rectangle checks, requested-profile source-bound caches and endpoint/SE2 route validation | See geometry_protocol_evidence_repair.md |
| Experiment evidence, 2026-10-27 | Estimated dynamic references and mixed completion/timing definitions | Fixed comparison protocol, separate evidence rates, real paired replans and independently coded geometric audits | Pilot studies preserve all failures |
| Experiment manager | Too many entries | Added registry/builder/runner/GUI | Recommended workflow |
| Unified visualization | Renderer drift | Centralized config/render helpers | Current boundary |
| Footprint-aware navigation | Point paths miss clearance | Added inflation/route validation | Current checks |
| Sparse graph repair | Invalid shortcuts | Added edge validation/repair | Current checks |

Commit/tag attribution is omitted where Git history cannot establish it.
