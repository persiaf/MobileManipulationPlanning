# Visualization Validation Report

Validation used the frozen experiment states and existing Stage1/P5 logs. No
planner, candidate, docking, reachability, or replanning code was modified.
The Stage2 and Stage3 rendering smoke tests wrote their files under
`results/figures/visualization_validation/<case>/`.

| Case | Required outputs | Validation |
|---|---|---|
| baseline | `scene.png`, `docking.png`, `planning.png` | PASS (Stage1 smoke) |
| cafeteria | `scene.png`, `docking.png`, `planning.png` | PASS (end-to-end smoke) |
| L1 | `scene_region.png`, `sparse_graph.png`, `planning.png` | PASS |
| L2 | `scene_region.png`, `sparse_graph.png`, `planning.png` | PASS |
| L3 | `scene_region.png`, `sparse_graph.png`, `planning.png` | PASS |
| D1 | `before_event.png`, `event.png`, `replanning.png`, `comparison.png` | PASS (Stage3 report smoke) |
| D2 | `before_event.png`, `event.png`, `replanning.png`, `comparison.png` | PASS (Stage3 report smoke) |
| D3 | `before_event.png`, `event.png`, `replanning.png`, `comparison.png` | PASS (Stage3 report smoke) |

All generated figures use white backgrounds, English Arial labels, consistent
line widths, shared colors, and a single legend convention. Titles are removed
from the final publication files; stage/event identity is encoded by filename.

The Stage3 report smoke uses the frozen L2 scene/plan contract when no dynamic
`planning_state_before` object is supplied; the dynamic event renderer accepts
that state object when available and adds event markers and local-replan text.

`FINAL_STATUS = VISUALIZATION_VALIDATED`
