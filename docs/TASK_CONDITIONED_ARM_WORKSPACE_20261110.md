# 任务约束驱动的机械臂可操作空间：第一版实现

日期：2026-11-10。开发分支：`codex/task-conditioned-arm-workspace`。

## 修改前备份

原工作区的未提交修改和未被 Git 忽略的文件已提交为 `6304e79`，并推送至
[`codex/backup-20261110-before-task-constraints`](https://github.com/persiaf/MobileManipulationPlanning/tree/codex/backup-20261110-before-task-constraints)。
包含当时的代码、论文草稿和临时目录中未被忽略的文件；`.gitignore` 排除的原始数据、缓存和日志不属于 Git 备份。公开上传已获用户明确授权。

## 模型事实与实现范围

原 `robot_config.m` 描述五关节、0.278 m 水平半径的简化机械臂，原可达性只计算位置包络和启发式分数。
它不能直接称为完整的 Mobile ALOHA 机械臂。官方 Mobile ALOHA 的
[follower 配置使用 vx300s](https://github.com/MarkFzp/mobile-aloha/blob/main/launch/4arms_teleop.launch)。

本次保留默认代码和原场景几何，新增可选 VX300s 六关节运动学实验模式。模型固定到
[Interbotix URDF 提交 0bb2b0e](https://github.com/Interbotix/interbotix_ros_manipulators/blob/0bb2b0e6d0e619bff02cf74dbd5af5681dcf80c9/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf.xacro)，
并与[官方 POE 模型](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/af96d28bcdf23fccd94a922bd23fb74f8156d914/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/mr_descriptions.py)独立交叉检查。
关节顺序为 `waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate`。
TCP 为 `ee_gripper_link`，工具前进轴为 **+X**。限位统一取固定 URDF，避免混用规格网页中不同的固件限位。

`task_manipulation_config` 中的左右臂安装变换初值继承原规划器的安装位置，**未经实体测量**，不表示完整 Mobile ALOHA 底盘、升降与双臂装配。应以实际平台的 `base_link -> arm_base_link` 变换替换，并另行核验工具 TCP。

## 任务定义

| task_type | 默认姿态要求 | 路径要求 |
|---|---|---|
| `position_only` | 无姿态限制 | 单点；也可显式给同样的路径用于对照 |
| `top_down_grasp` | 工具 +X 朝世界 −Z，允许绕接近方向旋转，容差 10° | 默认上方 4 cm 接近、目标点、撤离；按 2 cm 间距采样 |
| `table_wipe` | 完整 TCP 旋转，工具接近轴对准表面内法向，容差 5° | 必须显式提供至少两个不同的有序世界坐标路径点 |

两种任务都可以朝下，差别不应靠人为指定不同俯仰角制造。抓取保留绕接近方向的自由度；擦拭默认进一步固定面内朝向，并要求整段接触路径可执行。工具不同时，通过 `T_ee_tcp` 与 `tool_axis` 表达工具自身坐标系，而不是修改机械臂长度。

`task.manipulation` 可覆盖以下参数（单位为米、弧度）：

- `orientation_mode`：`none`、`axis` 或 `full`；擦拭必须为 `axis/full`。
- `rotation_world`：完整 TCP 旋转矩阵；`axis_direction_world`：轴约束的目标方向。
- `T_ee_tcp`：`ee_gripper_link -> 任务工具 TCP`；`tool_axis`：TCP 中指向接触面的工具接近轴。
- `waypoints_world`：N×3 有序路径；`sample_spacing_m`：笛卡尔采样间距。
- `position_tolerance_m`、`orientation_tolerance_rad`、`max_joint_step_rad`。
- `arm_side`：`either/left/right`，整条路径只用同一机械臂。
- `surface_normal_world`：表面向外法向；接触平面经过任务 xyz；`contact_height_tolerance_m` 默认 3 mm。
- `approach_distance_m`：沿接近方向反向生成接近/撤离点。

擦拭会同时检查期望路径和每个已求解 FK 位姿的接触高度；工具轴必须指向表面内法向。更换工具轴时，应一起设置正确的 `rotation_world`。本版擦拭接触路径不包含独立的靠近/离开接触阶段，默认 `approach_distance_m=0`；抓取包含接近/撤离。

## 使用方法

在项目根目录运行独立演示，不需要 Robotics System Toolbox：

```matlab
addpath('pipeline'); init_paths;
report = demo_task_conditioned_workspace;
```

接入已有场景和规划链：

```matlab
robot = robot_config();
robot.manipulator = task_manipulation_config(robot);
planner = planner_config();
planner.robot_cfg = robot;

% scene 已由现有场景入口构建。各任务可以使用不同的约束。
scene.tasks(1).task_type = 'top_down_grasp';
scene.tasks(1).manipulation = struct('orientation_tolerance_rad', deg2rad(8));

scene.tasks(2).task_type = 'table_wipe';
p = [scene.tasks(2).x, scene.tasks(2).y, scene.tasks(2).z];
scene.tasks(2).manipulation = struct('waypoints_world', ...
    [p + [0,-0.04,0]; p + [0,0.04,0]]);

% VX300s 需要重新设置候选距离。这里只是实验起点，不是实体标定值。
scene.scene_cfg.docking_min_distance_m = 0.45;
scene.scene_cfg.docking_max_distance_m = 0.80;
task_table_map = assign_tasks_to_tables(scene);
regions = build_task_regions(scene, task_table_map);
[poses, map, summary, info, plan] = stage2_docking_planning( ...
    regions, scene, robot, planner);
```

如使用 `resolve_effective_config`，可通过 `user_override.robot.manipulator` 传入上述可选配置。旧默认模型不变。若在旧模式下显式请求抓取/擦拭约束，将报错，避免静默忽略姿态要求。

## 求解与规划链

`arm_forward_kinematics` 提供 FK 与几何 Jacobian；`solve_arm_task_ik` 用确定性多初值阻尼迭代处理关节限位及三种姿态模式。后续路径点只以上一关节解作初值，并限制相邻关节变化，避免逐点重选机械臂或任意跳到另一组关节解。

`evaluate_task_reachability` 在新模式返回 0/1，含义是“是否找到满足约束的采样路径 IK 解”；不是原启发式质量分或概率。阈值不再把已满足 IK 约束的解额外筛掉。失败原因区分 `ik_not_found`、`joint_step_exceeded`、`contact_height_exceeded` 等；有限数值搜索失败不能证明数学上无解。

候选生成、候选增补、执行预测、模拟验证、局部重规划和重规划状态校验均调用此接口。新模式跳过旧小臂包络预筛。Robust 模式对各停靠误差样本使用同一机械臂，并同步保存该机械臂分配。候选缓存包含任务类型、约束、工具、路径和模型配置；执行状态变化不导致无关的缓存失效。

候选位置仍由 `scene_cfg` 的距离、横向和角度采样决定。旧场景 0.25–0.40 m 距离带可能使更大的 VX300s 过于靠近任务，不能用该有限候选集的失败推断机械臂无解。历史 `radial_operability_audit/local_reachability_oracle` 等位置包络实验保留原语义，不能直接充当新任务模式的 IK oracle。

## 首轮结果与证据边界

同一模型、同一底盘位姿、同一 z=0.45 m 平面上的 30 个采样点，比较位置任务、含接近/撤离的朝下抓取、固定姿态的 8 cm 擦拭路径。输出位于 `results/task_conditioned_workspace/`，含 CSV、MAT 和 PNG。结果是任务约束的联合影响；抓取与擦拭路径不同，不应假设二者的可行集合互相包含，也不能将差异单独归因于末端姿态。

首次可复现运行找到的可行采样数为：位置任务 **30/30**，朝下抓取 **17/30**，擦拭 **20/30**。这是当前参数和数值搜索预算下的有限采样结果，不是完整连续工作空间的体积估计。

MATLAB R2022a 下 **59/59 项定向测试通过**，并跑通新任务模式的停靠选择、导航及 IK 验证链。测试覆盖官方 POE 交叉验证、Jacobian 差分、姿态反向、工具/安装坐标、实际接触高度、法向、关节连续性、robust 固定臂、缓存语义与旧流程回归；不宣称全仓测试通过。复现命令：

```matlab
addpath('pipeline'); init_paths;
summary = run_task_manipulation_validation;
```

逐项结果与配置摘要保存为 `validation_cases.csv` 和 `validation_summary.json`；旧日志中路径未初始化及过近候选带的中间失败不作为最终验证结果。最终权威运行日志是本地 `results/logs/task_manipulation_authoritative_validation_20261110.log`（日志按仓库规则不上传）。

该实验验证机制能够改变可行区域，尚不足以证明论文方法优于基线。后续对照应固定相同的模型、场景、工具和候选集合，依次比较位置约束、轴向约束、完整姿态、整段路径及停靠误差，并统计新增停靠次数、路径代价、IK 求解开销和失败原因。

本版不检查机械臂自碰撞、双臂互碰、桌面/障碍物碰撞，不认证采样点之间的连续运动，不建模关节速度/加速度或接触力。执行结果标记为 `ik_validation`，`measured_completed` 仍未知。实体部署前需接入碰撞感知运动规划、实际安装变换和工具标定；擦拭还需独立接触控制。本次没有实现 Linux 执行器的新约束消费协议，现有 mission-plan JSON 不应直接作为上述任务轨迹的硬件执行文件。
