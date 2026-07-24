# IV. Experiments

## A. Experimental Setup

The proposed RCH-MT-MMP method was evaluated in a simulation-based multi-table service environment. The target problem is mobile manipulation planning for a dual-arm mobile robot, where task points are located on tabletop surfaces and cannot be directly reached by the mobile base. Therefore, instead of planning a path to the task points themselves, the proposed planner first generates table-side docking poses that satisfy reachability, task coverage, and docking feasibility constraints, and then plans a global navigation path over the selected docking poses.

The main case study was conducted in a 10 m x 10 m grid-map environment with a resolution of 0.05 m. The scene contained 16 rectangular tables arranged in a regular multi-table layout. Each table had a size of 1.2 m x 0.4 m and a height of 0.5 m. A total of 36 tabletop task points were generated, and the robot started from the lower-left region of the map. A* was used as the default global path planner, and the global path included the return-to-start segment.

To evaluate robustness and scalability, three groups of experiments were conducted. First, a normal random batch experiment was performed under small, medium, and large scene scales, with 20 random trials for each scale. The small, medium, and large settings contained 4, 9, and 16 tables, respectively. Second, hard scenario experiments were conducted under five task distributions: normal, hard corner, hard two-side, hard reach-boundary, and hard mixed. Each distribution was tested with 20 random trials in the small-scale scene. Third, ablation studies were conducted in both the main 16-table scene and the hard-mixed scenario to analyze the effects of reachability constraints, multi-docking selection, and coverage-aware docking evaluation.

All reported experiments are simulation-based validations. The manipulation feasibility considered in this study is reachability-level feasibility rather than full dynamic arm trajectory execution on a physical robot. Therefore, the results should be interpreted as planning-level validation of docking pose generation, task coverage, and global navigation, rather than real-robot deployment results.

## B. Evaluation Metrics

The primary metric is the task completion rate,

\[
R_c=\frac{N_c}{N_t}\times 100\%,
\]

where \(N_c\) is the number of completed tabletop tasks and \(N_t\) is the total number of tasks. Task completion rate is prioritized over shortest path length because a shorter mobile-base path is not meaningful if the selected docking poses fail to cover the required manipulation tasks.

The remaining metrics are used to characterize the quality and efficiency of the planning result. The number of docking poses reflects how many table-side manipulation poses are selected. The number of uncovered tasks measures tasks not covered by the selected poses. The total path length is the sum of the task-visiting path length and the return path length. Planning time records the computational time of the planning pipeline. Docking cost summarizes the evaluated quality of the selected docking poses, including reachability, coverage, orientation, safety, approach convenience, and maneuverability terms.

Together, these metrics reflect the coupled requirements of mobile manipulation planning: reachability, task coverage, docking feasibility, and global navigation efficiency.

## C. Main Case Study

The main case study validates the complete RCH-MT-MMP pipeline in the 16-table scene. The planner selected 27 docking poses for 36 tabletop tasks and achieved full task completion. The task path length was 57.34 m, and the return path length was 5.95 m, resulting in a total path length of 63.29 m. The generated path contained 1710 path points and 174 turns.

The result demonstrates that the proposed method can convert tabletop task points into feasible table-side docking poses and then connect these poses through a global navigation path. The selected path is not the shortest possible mobile-base route in isolation; instead, it is the result of prioritizing full task coverage under reachability and docking feasibility constraints.

**Table I. Main case study results in the 16-table scene.**

| Metric | Value |
|---|---:|
| Map size | 10 m x 10 m |
| Grid resolution | 0.05 m |
| Number of tables | 16 |
| Number of tasks | 36 |
| Selected docking poses | 27 |
| Completed tasks | 36 / 36 |
| Completion rate | 100.00% |
| Task path length | 57.34 m |
| Return path length | 5.95 m |
| Total path length | 63.29 m |
| Number of path points | 1710 |
| Number of turns | 174 |
| Total planning time | 9.79 s |
| Docking planning time | 9.34 s |
| Global path planning time | 0.08 s |
| Estimated execution time | 789.79 s |

**Fig. 1.** Framework of the proposed RCH-MT-MMP method. The planning pipeline first constructs tabletop task regions, then generates reachability-constrained docking poses, selects multi-docking task coverage poses, and finally plans the global visiting path.

**Fig. 2.** Main 16-table simulation scene. The figure shows tabletop task points, selected table-side docking poses, and the stitched global navigation path.

## D. Normal Random Batch Results

The normal random batch experiment evaluates scalability under increasing scene sizes. Across all 60 trials, the proposed method achieved 100% task completion. The number of tasks, selected docking poses, path length, and planning time increased with the scene scale, which is consistent with the larger number of tables and task points.

In the small-scale scenes, the mean number of tasks was 9.70, and the planner selected 7.20 docking poses on average. In the medium-scale scenes, the mean number of tasks increased to 22.70, with 15.80 selected docking poses. In the large-scale scenes, the mean number of tasks reached 40.80, and 27.95 docking poses were selected on average. Despite the increased scale, the completion rate remained 100% in all trials.

**Table II. Normal random batch results over 20 trials per scale. Values are reported as mean +/- standard deviation.**

| Scale | Tasks | Completion Rate | Docking Poses | Total Path Length | Planning Time |
|---|---:|---:|---:|---:|---:|
| Small | 9.70 +/- 0.80 | 100.00 +/- 0.00% | 7.20 +/- 0.62 | 20.57 +/- 1.64 m | 2.66 +/- 0.17 s |
| Medium | 22.70 +/- 1.42 | 100.00 +/- 0.00% | 15.80 +/- 1.28 | 41.66 +/- 3.56 m | 7.19 +/- 0.35 s |
| Large | 40.80 +/- 2.04 | 100.00 +/- 0.00% | 27.95 +/- 1.57 | 73.87 +/- 5.54 m | 19.34 +/- 1.54 s |

**Fig. 3.** Scalability summary under normal random task distributions. The completion rate remains 100% from small to large scenes, while path length and planning time increase with scene scale.

## E. Hard Scenario Results

The hard scenario experiment evaluates the adaptability of RCH-MT-MMP under challenging task distributions. Compared with the normal distribution, hard corner places task points near table corners, hard two-side distributes tasks on opposite sides of the table, hard reach-boundary places tasks near the reachability boundary, and hard mixed combines these challenging patterns.

The proposed method achieved 100% task completion in all hard scenario trials. No uncovered tasks were observed in any tested distribution. The number of selected docking poses varied with the spatial distribution of tasks. For example, hard reach-boundary required the largest mean number of docking poses, 7.30, because tasks close to the reachability boundary generally require more careful docking pose selection. In contrast, hard two-side used 5.80 docking poses on average.

**Table III. Hard scenario results over 20 trials per distribution. Values are reported as mean +/- standard deviation.**

| Scenario | Completion Rate | Uncovered Tasks | Docking Poses | Total Path Length | Planning Time |
|---|---:|---:|---:|---:|---:|
| Normal | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 6.85 +/- 0.88 | 20.27 +/- 2.05 m | 3.47 +/- 0.37 s |
| Hard corner | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 6.45 +/- 1.10 | 17.34 +/- 1.57 m | 3.33 +/- 0.48 s |
| Hard two-side | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 5.80 +/- 0.95 | 16.74 +/- 1.45 m | 3.19 +/- 0.04 s |
| Hard reach-boundary | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 7.30 +/- 0.73 | 18.40 +/- 1.91 m | 3.11 +/- 0.19 s |
| Hard mixed | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 6.10 +/- 1.12 | 17.83 +/- 2.01 m | 3.00 +/- 0.20 s |

These results indicate that the proposed docking pose generation and selection strategy can handle diverse tabletop task distributions. The key reason is that docking pose selection is not based only on proximity or navigation distance, but also considers reachability, coverage, and docking feasibility.

**Fig. 4.** Comparison between normal and hard task distributions. The hard scenarios change the spatial pattern of tabletop tasks while preserving the same simulation-based evaluation protocol.

**Fig. 5.** Hard scenario summary. The proposed method maintains full task completion under corner, two-side, reach-boundary, and mixed hard task distributions.

## F. Ablation Study in the Main Scene

The main-scene ablation study compares the full RCH-MT-MMP method with six variants: no reachability, single docking per table, nearest docking, no coverage cost, no coverage priority, and no coverage all. The full method achieved 100% completion with 27 docking poses and zero uncovered tasks.

The single-docking-per-table and nearest-docking variants show the most significant degradation. Single docking per table completed only 69.44% of tasks and left 11 tasks uncovered. Nearest docking completed only 50.00% of tasks and left 18 tasks uncovered. Although these two variants produced shorter total paths, the shorter paths were caused by missing tasks rather than better mobile manipulation planning. This confirms that path length should be interpreted only after task coverage and reachability feasibility have been satisfied.

Removing the reachability term did not reduce completion rate in this particular main scene, but it increased the total path length from 63.29 m to 67.71 m. This suggests that reachability-aware docking selection also affects downstream navigation efficiency. Removing individual coverage terms had little effect in the main scene, but the hard ablation study below shows that coverage-related terms can influence docking pose count and path behavior under more challenging task distributions.

**Table IV. Main-scene ablation study.**

| Method | Completion Rate | Docking Poses | Uncovered Tasks | Task Path | Total Path | Planning Time | Docking Cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| Full | 100.00% | 27 | 0 | 57.34 m | 63.29 m | 0.078 s | 8.8151 |
| No reachability | 100.00% | 27 | 0 | 57.58 m | 67.71 m | 0.159 s | 8.6573 |
| Single docking per table | 69.44% | 16 | 11 | 43.51 m | 53.64 m | 0.166 s | 4.6221 |
| Nearest docking | 50.00% | 16 | 18 | 36.72 m | 46.69 m | 0.064 s | 5.8956 |
| No coverage cost | 100.00% | 27 | 0 | 57.34 m | 63.29 m | 0.080 s | 6.6151 |
| No coverage priority | 100.00% | 27 | 0 | 57.34 m | 63.29 m | 0.077 s | 8.8151 |
| No coverage all | 100.00% | 28 | 0 | 57.34 m | 63.29 m | 0.078 s | 6.8840 |

**Fig. 6.** Main-scene ablation results for completion rate and uncovered tasks. The nearest-docking and single-docking variants fail to cover all tasks, demonstrating the necessity of multi-docking coverage selection.

**Fig. 7.** Main-scene ablation results for path length and docking pose count. Shorter paths in incomplete variants should not be interpreted as better results because they are associated with task failures.

## G. Hard-Mixed Ablation Study

To further test the necessity of the docking selection components, the same ablation modes were evaluated under the hard-mixed task distribution. Each mode was tested over 20 random trials. The full method achieved 100% completion with zero uncovered tasks, 7.05 docking poses on average, and a mean total path length of 17.41 m.

The nearest-docking variant completely failed under hard mixed tasks, with a 0.00% completion rate and 10.40 uncovered tasks on average. The single-docking-per-table variant achieved only 75.54% completion and left 2.40 tasks uncovered on average. These results are stronger than the main-scene ablation and clearly show that selecting the closest docking pose or enforcing one docking pose per table is insufficient for mobile manipulation tasks with spatially diverse tabletop targets.

The coverage-related variants retained 100% completion but changed the docking pose count and total path length. Removing all coverage-related terms increased the mean number of docking poses to 7.60 and increased the mean total path length to 18.10 m. This indicates that coverage-aware selection helps avoid unnecessary docking poses while maintaining task coverage.

**Table V. Hard-mixed ablation study over 20 trials per mode. Values are reported as mean +/- standard deviation.**

| Method | Completion Rate | Uncovered Tasks | Docking Poses | Total Path Length | Planning Time | Docking Cost |
|---|---:|---:|---:|---:|---:|---:|
| Full | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 7.05 +/- 0.69 | 17.41 +/- 1.40 m | 2.99 +/- 0.24 s | 2.7498 +/- 0.4387 |
| No reachability | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 6.45 +/- 0.83 | 18.20 +/- 2.40 m | 3.12 +/- 0.17 s | 2.3435 +/- 0.5154 |
| Single docking per table | 75.54 +/- 9.02% | 2.40 +/- 0.88 | 4.00 +/- 0.00 | 15.65 +/- 0.55 m | 2.91 +/- 0.18 s | 1.2382 +/- 0.1726 |
| Nearest docking | 0.00 +/- 0.00% | 10.40 +/- 0.82 | 4.00 +/- 0.00 | 16.20 +/- 0.00 m | 2.97 +/- 0.17 s | 2.2196 +/- 0.0544 |
| No coverage cost | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 6.50 +/- 0.95 | 17.31 +/- 1.52 m | 3.05 +/- 0.17 s | 1.8917 +/- 0.4129 |
| No coverage priority | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 6.75 +/- 0.91 | 17.66 +/- 1.69 m | 3.11 +/- 0.19 s | 2.5187 +/- 0.5655 |
| No coverage all | 100.00 +/- 0.00% | 0.00 +/- 0.00 | 7.60 +/- 0.88 | 18.10 +/- 1.34 m | 3.12 +/- 0.27 s | 2.2312 +/- 0.3859 |

**Fig. 8.** Hard-mixed ablation comparison. The failure of nearest docking and the performance drop of single docking per table confirm that multi-docking and task coverage are necessary under challenging task distributions.

## H. Global Planner Benchmark

The implemented framework also supports a multi-algorithm global planner benchmark after docking pose selection. The benchmark can compare graph-search, sampling-based, and heuristic planners under a shared docking visit order. This design separates two issues: the local mobile manipulation feasibility problem solved by docking selection, and the global path connection problem solved by the navigation planner.

In the current completed result files, the exported CSV summaries focus on the main method, random batch tests, hard scenarios, and ablation studies. A separate numerical global planner benchmark table is not included in the available result files. Therefore, global planner benchmark results should be reported only after exporting the corresponding `benchmark_results` structure to a table. The recommended benchmark metrics are total path length, task path length, return path length, total computation time, fallback count, number of turns, and number of path points.

**Table VI. Recommended global planner benchmark format.**

| Planner | Total Path | Task Path | Return Path | Total Time | Fallback Count | Path Points |
|---|---:|---:|---:|---:|---:|---:|
| A* | To be exported | To be exported | To be exported | To be exported | To be exported | To be exported |
| Dijkstra | To be exported | To be exported | To be exported | To be exported | To be exported | To be exported |
| JPS | To be exported | To be exported | To be exported | To be exported | To be exported | To be exported |
| GBFS | To be exported | To be exported | To be exported | To be exported | To be exported | To be exported |
| Theta* | To be exported | To be exported | To be exported | To be exported | To be exported | To be exported |

## I. Discussion

The experimental results support the main motivation of RCH-MT-MMP: mobile manipulation planning in tabletop service environments cannot be reduced to shortest-path navigation. Since the task points are located on table surfaces, the mobile base must visit feasible docking poses from which the dual-arm system can reach and cover the tasks. A path that is shorter but leaves tasks uncovered is not a successful mobile manipulation plan.

The normal random batch results show that the proposed planner maintains full task completion across increasing scene scales. The hard scenario results show that the method remains effective when task points are concentrated near corners, distributed on opposite sides, or placed near reachability boundaries. The main-scene and hard-mixed ablation studies further demonstrate the necessity of multi-docking selection and coverage-aware docking evaluation. In particular, the nearest-docking strategy fails severely under hard-mixed task distributions, confirming that docking feasibility and task coverage must be considered explicitly.

The results also show that reachability, task coverage, docking feasibility, and global navigation efficiency are coupled. Reachability determines whether a task can be executed from a candidate pose; coverage determines whether all tabletop tasks can be completed; docking feasibility ensures that the mobile base can safely occupy the pose; and global navigation efficiency determines the cost of visiting the selected poses. The proposed hierarchical formulation provides a practical way to combine these factors while preserving a clear separation between local manipulation-oriented docking selection and global path planning.

Several limitations remain. First, the validation is simulation-based and does not include real-robot execution uncertainties such as localization error, perception noise, actuator limits, or dynamic obstacles. Second, manipulation feasibility is evaluated at the reachability level rather than through complete dual-arm motion planning and collision checking. Third, the current benchmark summaries mainly evaluate planning outputs and do not yet include a fully exported numerical comparison of all global planners. Future work will extend the framework to higher-fidelity robot simulation, full arm trajectory validation, dynamic replanning, and physical robot experiments.

