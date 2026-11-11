# ROS 1 / Gazebo一个月实验追踪表

2026-11-11新增离线多布局任务约束／导航批次，独立方案见 [TASK_NAVIGATION_ROBUSTNESS_EXPERIMENT_PLAN_20261111.md](TASK_NAVIGATION_ROBUSTNESS_EXPERIMENT_PLAN_20261111.md)，实时状态见 `results/task_navigation_robustness_20261111/scenario_progress.csv`。其离线结果不计入下表ROS/实机任务完成数。

日期：2026-10-30。对应同目录 `EXPERIMENT_PLAN.md`。**全部任务为TODO，未访问Ubuntu工程、未修改模型、未执行仿真或真机。**

已确认：用户继续Ubuntu 20.04＋ROS 1＋Gazebo；已有初步场景，关节能动但方向/零位/TCP不对；工程位于本电脑双系统Ubuntu侧，目前不可访问。首周先修模型/映射，不把迁移ROS 2或盲目调PID作为起点。已有真实自主取放不等于仿真控制正确。

| ID | 里程碑 | 目的 | 系统/数据 | 指标/交付 | 优先级 | 状态 | 继续条件与备注 |
|---|---|---|---|---|---|---|---|
| R001 | M0 / 第1–2天 | 环境与模型快照 | 实际ROS/Gazebo版本、launch、运行时URDF/SDF | 可恢复配置、加载来源和模型hash | MUST | TODO | 不在Windows猜测或重写Ubuntu模型 |
| R002 | M0 | 单关节身份与正方向 | 命令、joint_states、控制器状态 | 逐关节正负小步审查表 | MUST | TODO | 按名称匹配；转动rad、平移m；其他关节保持当前值 |
| R003 | M1 / 第3–5天 | origin/axis/零位/TCP | 左臂至少三个姿态及独立参考 | 关节中心、方向、TCP误差、源单位/零位映射 | MUST | TODO | RViz与Gazebo相同不足以证明物理正确 |
| R004 | M1 | 右臂独立验收 | 右臂模型、安装与输入映射 | 右臂多姿态记录、联合无载检查 | MUST | TODO | 不直接复制左臂角度或统一取负 |
| R005 | M1 / 第6–7天 | 升降与底盘坐标链 | 至少两个h与两个底盘姿态 | TCP变换一致性、无重复加高度 | MUST | TODO | 台面/抓取点/升降行程分开 |
| G001 | M1 gate | 是否允许进入规划比较 | J1–J5及小幅重复动作 | 验收阈值、通过证据、未解问题 | MUST | TODO | 未通过就继续修模型，不能靠附着绕过 |
| R010 | M2 / 第2周 | 最小场景与技能 | 一刚性箱、350/660 mm工位 | 目标位姿、技能版本、运输包络 | MUST | TODO | 不假设策略为ACT，固定原有动作或显式仿真模板 |
| R011 | M2 | 接JSON与执行状态机 | MATLAB→rospy/actionlib→Gazebo | 计划ID映射、导航/升降/双臂反馈、完整日志 | MUST | TODO | 路线起点索引与服务ID区分；只执行选中配置 |
| R012 | M2 | 明确路径归因 | 共同ROS导航器或导出MATLAB路径 | 冻结导航/跟踪方案 | MUST | TODO | ROS重新生成路径不得归为MATLAB路径算法成果 |
| R013 | M2 | 冻结执行模式 | contact或attach_proxy | 模式、成功判据、两侧接触/附着前置条件 | MUST | TODO | proxy只限运动学/控制/碰撞正确而接触困难时 |
| R014 | M2 | 技能域与误差开发 | 开发样本、保留核查、误差来源 | supported/unknown、经验样本、真值/估计分离 | MUST | TODO | 4.3 cm均值不作σ或误差界；合成模型标敏感性 |
| R015 | M2 | A/B/C共同执行 | A分离、B名义联合、C预见恢复 | 同候选/域/控制/恢复/预算的可运行方案 | MUST | TODO | 到位门禁与调高/重靠权限完全相同 |
| R016 | M2 | 机制pilot | 少量普通与安全边界例 | 配置变化、恢复成本、失败来源 | MUST | TODO | 不计入主表，B/C无机制差异先收缩主张 |
| R017 | M2 | 扰动真实性检查 | 误差注入位置、阶段与持续时间 | 操作开始时的请求/实际残差 | MUST | TODO | 不能只改日志，或将被理想导航消除的扰动当有效误差 |
| G002 | M2 gate | 冻结正式协议 | 三场景、两误差、五种子、三方法 | 90或180、随机顺序、预算、模型/控制hash | MUST | TODO | 正式前决定，不根据中途胜负追加/停止 |
| T001 | M3 / 第3周 | 场景1名义条件 | A/B/C×5种子 | 15条完整试次及全部失败 | MUST | TODO | 无脚本扰动不等于物理误差为零 |
| T002 | M3 | 场景1有来源扰动 | A/B/C×5种子 | 15条完整试次及实际误差 | MUST | TODO | 真值与估计分开，记录扰动机制 |
| T003 | M3 | 场景2名义条件 | A/B/C×5种子 | 15条完整试次 | MUST | TODO | 独立复位、共同起点、随机方法顺序 |
| T004 | M3 | 场景2有来源扰动 | A/B/C×5种子 | 15条完整试次 | MUST | TODO | 恢复次数与时间预算一致 |
| T005 | M3 | 场景3名义条件 | A/B/C×5种子 | 15条完整试次 | MUST | TODO | 场景是已验证可操作范围，非故意压低基线 |
| T006 | M3 | 场景3有来源扰动 | A/B/C×5种子 | 15条完整试次 | MUST | TODO | 保留失败/碰撞/干预，阶段分类 |
| T007 | M3，可选预定 | 第二对象扩展 | 另1对象×同3场景×2误差×5种子×3方法 | 90条附加试次，总180 | NICE | TODO | 仅G002已预定且对象技能稳定时执行 |
| R020 | M4 / 第4周 | 主表与数据审计 | T001–T006全部结果 | 每方法30条、配对差异、各条件原始数及区间 | MUST | TODO | 30区组但仅6场景—误差条件；不保证统计功效 |
| R021 | M4 | 机制与失败分析 | 候选排序、到位状态、恢复日志 | 无需重靠、调高/重靠、误差/碰撞/接触失败图 | MUST | TODO | 同一评分函数重复调用不算独立验证 |
| R022 | M4，可选 | 简化/删除检查 | 固定裕量或删除调高预测，二选一 | 决策变化、开销、少量关键复核 | NICE | TODO | 运行时恢复不变；次数与条件预先登记 |
| T010 | M4，可选真机 | 代表案例核查 | 2案例×2重复×3方法 | 12条逐次实机记录 | NICE | TODO | 每方法4次仅展示，不估计总体成功率；不能挤占主试验 |
| R023 | M4 | 论文与复现包 | 冻结协议、原始日志、模型/配置版本 | 方法图、主表、失败边界、命令/环境快照 | MUST | TODO | 仿真/实机、contact/proxy分开；不借用旧PDF成功率 |

## 样本、指标与时间口径

- 默认：1对象×3场景×2误差条件×5种子×3方法＝90次，每方法30次，30个配对区组，仅6种场景—误差条件。
- 预定扩展：2对象时总180次，每方法60次。不要将总次数误当各方法样本量。
- 默认主指标：contact模式为完整仿真取放转运成功；attach_proxy模式为替代技能条件下的仿真任务执行成功。二者不合并，不称真实抓取成功率。
- 若pilot发现天花板，可在正式测试前记录主张修订并冻结成本协议；始终报告全部成功率，禁止看过主结果后换指标。
- 每服务配置5 s、整链120 s只是pilot起点。若正式冻结120 s，90次上限3 h模拟时间；墙时约模拟时间/RTF＋规划、复位和日志开销，须实测，不保证完成时间。
- B1控制错误未解决，不启动主算法比较，不以attach_proxy掩盖错误。仅接触困难且其余验证通过时才切换proxy。
- 不叠加旧版60次真机任务；选做真机12次单独记录，取消或延期条目保留原因。

## 原始试次最少字段

`trial_id, block_id, method, seed, scene/version, object/size/mass, error_condition/source/injection_stage, requested_and_realized_perturbation, model_hash, controller_hash, skill/domain_version, execution_mode, selected_service_ids, planned/estimated/ground_truth_pose, lift_command/actual, joint_command/actual, object_state, planning_wall_time, task_sim_time, task_wall_time, real_time_factor, retry_limits, lift_corrections, redock_count, skill_retries, autonomous_success, intervention, collision, failure_stage/reason, bag/video_reference`。

正式开始后逐条展开T批次，以原始记录更新状态。当前所有条目均未执行，不能用计划总数作为已完成样本量。
