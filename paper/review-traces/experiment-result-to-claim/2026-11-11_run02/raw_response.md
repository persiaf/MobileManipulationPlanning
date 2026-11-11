# Independent judgment

**Scoped navigation-improvement claim: yes. Paper novelty or general superiority: no evidence.** Confidence is high for the frozen exported matrix/graph and the bounded combinatorial statement; this is not physical validation. The paper-facing verdict remains provisional because no manuscript claim audit is available.

Independent standard-library Python audit passed all 9 cases. It reconstructed all 680 same-table candidate-task keys per mode, the unchanged IK CSV and candidate pool, the DP minimum stop counts 2/4/4, all regional minimum covers (16/9/28 alternatives per table), and all 2,292 necessary directed edges. Edge masks omit no Hamiltonian tour on the minimum-cover layer. Independent enumeration covered 512 position, 1,944 grasp and 18,816 wipe tours and matched the joint costs. This particular edge graph happens to have no numerically asymmetric pairs; asymmetric and Inf support is a code property, not a dataset result.

| Condition | Staged cost | Fixed-set reorder cost | Joint cost | Staged -> joint raw distance |
|---|---:|---:|---:|---:|
| Position | 12.096722298 | 12.096722298 | 9.641392877 | 11.625483400 -> 9.274873734 m |
| Downward grasp | 15.260854414 | 15.260854414 | 15.051414904 | 14.318376618 -> 14.318376618 m |
| Wipe | 12.301442053 | 12.196722298 | 10.936062126 | 11.725483400 -> 10.464823228 m |

Costs are m-equivalent: path length + 0.15 m/rad times wrapped endpoint heading changes, including return. Weighted reductions are 20.2975%, 1.3724%, 11.0993%. Raw distance reductions are 20.2195%, 0%, 10.7515%. The grasp improvement is entirely the endpoint-heading term (2*pi -> 4.886921906 rad), not a shorter path. Wipe fixed-set reordering also changes heading cost without shortening raw distance; joint selection then improves beyond that control by 10.3361% weighted cost.

Every case assigns all 10 tasks exactly once to supported candidates with matching stored arm assignment. The selected sets stay at 2/4/4, all actual routes visit dock positions/headings in recorded visit-rank order and return to the common start position/heading. CSV path lengths and weighted costs match the directed graph. Sampled base polygons avoid physical table rectangles and the 0.1 m border. These checks do not solve IK independently or certify swept/arm/contact motion.

The exactness assumptions are now explicit: regions must partition all scene task indices; the strategy searches the binary minimum-cover layer and Hamiltonian closed tours, visiting each selected dock once. If that layer has no finite tour it reports BLOCKED instead of trying extra docks. The certificate is therefore not the optimum over arbitrary continuous placements, repeat-visit walks, or all navigation-feasible covers.

The fixed-set control supports an interpretation of a docking-selection benefit beyond merely running a better route permutation. It does not establish a novel scalable algorithm: exhaustive finite-pool search is expected to dominate a heuristic on its own objective, and it pays additional navigation-oracle work. The shared graph took 29.0613351 s for 2,292 directed queries. The sub-second search bars exclude this and final route construction, so they are not total-runtime or speedup evidence. No repeated timing/significance/generalization claim is justified.

Visual review: main_style_overview and per-mode paired figures reproduce the main scenario's gray tables, green tasks, blue actual routes, orange dock triangles, red start, blue physical footprints/headings, full axes and light grid. Geometry remains the experimental two-table room. Task-assignment links are explicitly not arm paths. Grasp yaw labels make the heading-only improvement visible; the caption correctly says distance did not shrink. The metrics separate raw distance and weighted cost and disclose the omitted graph overhead. No blocking visualization defect was found. Minor publication suggestion: label panel task counts as IK-supported rather than simply tasks, and always retain the computational-scope caption.

Route: confirm this bounded mechanism/result, retain the exact method as a finite-pool reference, and supplement before novelty, scaling, efficiency, robustness or physical-execution claims. Useful next evidence is held-out layouts/heights/path directions, candidate/IK saturation, full-cost repeated timings, equal-budget navigation-aware baselines, a scalable search variant, and collision-aware independent arm/transition validation. These future studies are not required to finish the current requested comparison.
