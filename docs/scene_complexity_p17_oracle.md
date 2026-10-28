> 历史记录说明（2026-10-27）：餐厅场景、实机匹配场景及另一套机器人几何已移除。本文的旧入口、维护状态与文件数量仅对应当时版本；当前唯一模型及删除范围见 [主场景参数统一说明](main_robot_unification.md)。

# Scene Complexity P1.7-Rapid Oracle

Date: 2026-09-02

Branch: `research/scene-complexity-scaling`

## 1. Oracle Definition

The local reachability feasibility oracle is implemented in:

```text
experiments/exp_local_reachability_oracle.m
```

It is diagnostic only. It does not add poses to the production candidate
library and does not change:

```text
reachability threshold
collision margin
robot geometry
map size
global sampling density
```

The formal robot profile is `legacy_validated`. The exploratory
`physical_robot` profile is not used.

For each baseline-uncovered task, the oracle searches a table-relative local
band around all four table sides. Each pose is checked by the unchanged
footprint collision check and the unchanged legacy reachability evaluator. The
oracle only answers whether a collision-valid and reachability-valid pose
exists.

## 2. F010 Result

Case:

```text
F010 seed 2026
grid layout
discrete_random table orientation
discrete tasks
normal difficulty
10 m by 10 m map
16 tables
legacy_validated robot profile
```

Result:

| Metric | Value |
|---|---:|
| Task count | 75 |
| Baseline candidate count | 4689 |
| Baseline coverage | 64.00% |
| Baseline uncovered tasks | 27 |
| Oracle feasible tasks | 4 |
| Oracle infeasible tasks | 23 |
| Feasible ratio | 14.81% |
| Collision-valid oracle poses | 142520 |

## 3. Feasible/Infeasible Split

The feasible ratio is below the 20% gate threshold. Therefore, candidate
generation is not the primary bottleneck for this sample.

```text
primary bottleneck = REACHABILITY_MODEL_IS_PRIMARY_BOTTLENECK
```

## 4. Missing Geometry for Feasible Tasks

Only 4 of the 27 uncovered tasks were oracle-feasible.

Their missing-geometry classes were:

| Missing geometry class | Task count |
|---|---:|
| MISSING_HEADING | 3 |
| POSITION_HEADING_INTERACTION | 1 |

This is useful for a later narrow candidate-generation repair, but the subset
is too small to justify implementing repair in this round.

## 5. Reachability Rejection for Infeasible Tasks

The dominant rejection condition is:

```text
FAIL_RADIAL
```

The oracle found many collision-valid local poses, but most uncovered tasks
still failed the unchanged reachability model. The maximum combined reach score
observed across the oracle pass was `0.35662`, but this came from the small
feasible subset; the majority of tasks remained model-unreachable.

## 6. Repair Result

No repair was implemented.

Reason:

```text
oracle feasible ratio = 14.81% <= 20%
```

Per the P1.7 gate, this stops the round before modifying supplementary
candidate generation.

## 7. Final Gate

Decision:

```text
REACHABILITY_MODEL_REVIEW_REQUIRED
```

P2 candidate pruning should not start from this branch state. The next research
step should review the reachability model semantics, especially the radial
reach condition under mixed table orientations and nearby-table collision
constraints.
