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
| Cafeteria scene | Need compact layout | Added basic3/complex12 and mode split | Stage 1 extension |
| Experiment manager | Too many entries | Added registry/builder/runner/GUI | Recommended workflow |
| Unified visualization | Renderer drift | Centralized config/render helpers | Current boundary |
| Footprint-aware navigation | Point paths miss clearance | Added inflation/route validation | Current checks |
| Sparse graph repair | Invalid shortcuts | Added edge validation/repair | Current checks |

Commit/tag attribution is omitted where Git history cannot establish it.
