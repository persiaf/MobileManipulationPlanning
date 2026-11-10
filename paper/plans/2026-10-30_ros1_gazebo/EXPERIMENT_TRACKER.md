# ROS 1 / Gazebo实验追踪表

日期：2026-10-30。对应同目录EXPERIMENT_PLAN.md。**全部TODO，未访问双系统Ubuntu工程、未修改模型、未运行仿真/真机。** 先修关节方向、零位与TCP，再做算法对照；不迁移ROS 2，不叠加旧60次真机。

| ID | 时间/块 | 任务与交付 | 通过条件/计数 | 优先级 | 状态 |
|---|---|---|---|---|---|
| R001 | W1/B1 | 环境、运行模型、launch/控制配置快照；单关节正负小步 | 名称/单位/方向/旋转中心及command/actual可追溯 | MUST | TODO |
| R002 | W1/B1 | 左右臂分别多姿态参考；修origin/axis/零位/TCP | 独立几何参考，不只RViz/Gazebo同URDF一致 | MUST | TODO |
| R003 | W1/B1 | 升降/底盘坐标链、联合无载重复 | J1–J5通过；否则暂停算法比较，不能附着绕过 | MUST | TODO |
| R004 | W2/B2 | 一箱两台、技能域/运输包络、JSON状态机 | 350/660 mm台面与抓取点分开；ID映射和阶段日志正确 | MUST | TODO |
| R005 | W2/B2 | 冻结contact/proxy、导航归因与A/B/C | 同候选/域/控制/恢复；proxy仅限其余验收通过后接触困难 | MUST | TODO |
| R006 | W2/B2 | 机制pilot、扰动来源/残差检查 | B/C有可解释差异；不凭4.3 cm均值造高斯，不只改日志 | MUST | TODO |
| G001 | W2 gate | 冻结协议、模型hash、预算、N与随机顺序 | 默认90或预定180；主张无效先缩减，正式后不换指标 | MUST | TODO |
| T001–T006 | W3/B3 | 3场景×2误差，每条件A/B/C×5种子 | 每批15，共90；每方法30；30区组仅6条件；全部失败保留 | MUST | TODO |
| T007 | W3/B3选做扩展 | 第二对象复制冻结矩阵 | 额外90、总180；仅G001预定且技能稳定时做 | NICE | TODO |
| R010 | W4/B4 | 主表、原始N/区间、候选/恢复及失败分析 | 模拟时间/墙时/RTF分开；删除/简化变体额外运行选做 | MUST | TODO |
| T010 | W4/B5选做真机 | 2案例×2重复×3方法 | 12次，每方法4次仅案例核查，不估总体成功率 | NICE | TODO |
| R011 | W4 | 论文图、模型/配置版本、原始日志与复现说明 | contact/proxy/真机分表，不借用旧PDF成功率 | MUST | TODO |

默认主指标：contact为完整仿真取放转运成功；proxy为替代技能条件下仿真任务执行成功。若pilot触发主张修订，应在正式前书面冻结，始终报告全部成功率。120 s整链仅为pilot起点；若正式采用，90次最多3 h模拟时间，墙时除RTF再加复位/规划/日志开销，不承诺工期。

逐试次保存：`trial/block/method/seed, scene/object, error_source/injection/requested/realized, model/controller/skill/domain_version, execution_mode, selected_service_ids, planned/estimated/ground_truth_pose, lift/joint_command_actual, object_state, planning_wall/task_sim/task_wall/RTF, retry_limits, lift_corrections/redock/retries, success/intervention/collision, failure_stage/reason, bag/video_reference`。正式开始后展开T批次；取消/延期保留原因，计划总数不当已完成样本量。