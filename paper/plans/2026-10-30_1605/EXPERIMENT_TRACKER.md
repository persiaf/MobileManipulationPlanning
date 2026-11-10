# 实验执行追踪表

日期：2026-10-30。对应 `EXPERIMENT_PLAN.md`。**以下全部为TODO，尚未运行。** 之前的几何测试和4桌案例单独保留在现状核查报告，不冒充本表中的物理验证结果。

硬件/期限未确认；第1–2周取得模型是条件。180 s/次上限及具体候选/求解预算待开发pilot后冻结。表中一个“批次”包含多个运行，执行时展开唯一run_id，并保存全部失败。

| Run ID / 批次 | 阶段 | 目的 | 系统/变体 | 数据划分 | 主要指标/交付 | 优先级 | 状态 | 数量与备注 |
|---|---|---|---|---|---|---|---|---|
| R001 | M0/B1 | 机器人模型与坐标校核 | 统一模型/物理后端 | 开发夹具 | FK误差、限位、工具轴、碰撞几何 | MUST | TODO | 先确认型号、自由度、完整模型来源；无模型不进入下步 |
| R002 | M0/B1 | 完整单站动作 | 统一IK/运动/终检 | 普通开发例 | 入场、接近、执行、退离、q连续 | MUST | TODO | 不能只展示各目标独立IK |
| R003 | M0/B1 | 运输安全 | 收臂姿态+现有导航 | 开发通道夹具 | 底盘与手臂整体包络、端点一致 | MUST | TODO | 最低版本空载；不只验证底盘rectangle |
| R004 | M1/B1 | 公平普通基线 | A/B/C/Ours | 3普通场景 | 全部输出的状态与终检 | MUST | TODO | 方法不能拥有不同碰撞白名单 |
| R005 | M1/B1 | 机制是否存在 | A/B/C/Ours+高预算参考 | 3机制场景 | 点IK/序列过渡/return-home差异 | MUST | TODO | 真实模型；一次未找到不等于物理无解 |
| R006 | M1 | 预算及协议冻结 | A/B/C/Ours | 上述6场景×3种子 | timeout、候选上限、计时、日志 | MUST | TODO | 72次预算，最多3.6 process-hours暂定；R004/R005重合运行复用，不重复计费 |
| G001 | M1决策 | 是否继续规模实验 | 评审门槛 | 开发结果 | 机制差异、公平性、模型可信性 | MUST | TODO | 未通过则不运行R010–R013，不用更多场景掩盖无差异 |
| R007 | M1 | 保留集与方法冻结 | 所有系统 | 30独立场景 | 场景/模型/候选/代码hash与种子表 | MUST | TODO | 3组×2规模×5布局；不看结果调参 |
| R010 | M2/B2 | 主结果A | 停靠先行+统一后验运动验证 | 30场景×3种子 | 固定预算完整计划成功率 | MUST | TODO | 90次；允许普通重试与回home救援 |
| R011 | M2/B2 | 主结果B | 逐点IK/成本联合选择+全量重算 | 同R010 | 同主指标、调用数、失败类型 | MUST | TODO | 90次；状态模型与Ours相同 |
| R012 | M2/B2 | 主结果C | 通用lazy反馈 | 同R010 | 同主指标、时间预算曲线 | MUST | TODO | 90次；是否真实PDDLStream或自实现须明确 |
| R013 | M2/B2 | 主结果Ours | 服务序列反馈协同规划 | 同R010 | 同主指标、改组/修复记录 | MUST | TODO | 90次；四批总360次，最多18 process-hours暂定 |
| R014 | M2/B2 | 配对分析 | A/B/C/Ours | 全部360次，包括失败 | 场景聚类区间、成功数、原因 | MUST | TODO | 统计unit=30场景，3种子是重复 |
| G002 | M2决策 | C1是否得到支持 | Ours对最强C | 保留结果 | 成功率主假设的证据 | MUST | TODO | 不能在看到结果后把次成本升级成原主张 |
| R020 | M3/B3 | 删除序列反馈 | V1：同后端/状态的generic lazy | 预选12场景×3种子 | 主成功率、失败验证次数 | MUST | TODO | 至多36次；若等同C，直接复用R012子集 |
| R021 | M3/B3 | 删除分类局部修复 | V2：完整验证+统一全量重算 | 同R020 | 修复/调用/时间分项 | MUST | TODO | 36次 |
| R022 | M3/B3 | 简化机制强对照 | V3：显式逐任务return-home救援 | 同R020 | 成功率、实际回零轨迹代价 | MUST | TODO | 36次；不将回零禁止以制造失败 |
| R023 | M3/B3 | 消融综合 | 完整Ours复用R013 | 同12场景 | 关键机制差异与简化选择 | MUST | TODO | 新增最多108次=5.4 process-hours暂定 |
| R030 | M4/B4 | 布局适用范围 | A/B/C/Ours | R010–R013分组 | 角度/间距/任务方向分组成功率 | MUST | TODO | 复用主数据，0次新增 |
| R031 | M4/B4 | 小扰动 | Ours/C | 预选6场景×2非零扰动×3种子 | 固定计划通过率、恢复成功率 | MUST | TODO | 72次新增=3.6 process-hours暂定；无实测时明确配置扰动 |
| R032 | M4/B4 | 失败诊断 | Ours/C及终检 | 所有扰动失败 | 固定分配臂、碰撞/过渡/状态失败 | MUST | TODO | 不隐式换臂后称原分配鲁棒 |
| R040 | M5/B5 | 动态局部恢复 | 局部修复/全量重算 | 6有效基准场景×2事件×3种子 | 最终成功率、含回退总耗时 | OPTIONAL | TODO | 72次=3.6 process-hours暂定；核心不足则取消 |
| R041 | M5/B5 | 硬件演示或运动仿真 | 经验证完整系统 | 2–3代表设置 | 连续执行日志、全部试次 | OPTIONAL | TODO | 硬件/人时未定；不能预填实测成功 |
| R050 | 整理 | 可复现报告与论文图 | 冻结全部方法 | 全部有效及失败记录 | 原始表、协议、版本、图表 | MUST | TODO | 图表只从冻结记录生成 |

## 预算核对

- 核心正式批次：360 + 至多108 + 72 = 至多540次；暂定上限27 process-hours。
- 开发pilot：72次，上限3.6 process-hours；R004/R005与pilot重合的记录复用。
- 可选动态：72次，上限3.6 process-hours。
- 含pilot及可选动态总计最多684次，上限34.2 process-hours。该值是180 s计时上限乘运行数，不包含模型调试、场景生成、I/O和外部终检，也不等于CPU core-hours。
- 主指标固定为预算内完整可执行计划成功率。预算与主指标只可在保留集运行前根据pilot调整并留版本记录。

## 每条结果最少保存字段

`run_id, batch_id, scene_id, scene_seed, motion_seed, method, model_hash, source_hash, task_contract_hash, candidate_pool_hash, budget_s, status, failure_reason, planning_wall_s, verification_wall_s, completed_tasks, total_tasks, estimated_execution_s, measured_execution_s, candidate_count, ik_calls, motion_calls, repair_attempts, fallback_time_s, artifact_path`。

`measured_execution_s`无实机日志时为未知；`IK_NOT_FOUND/MOTION_NOT_FOUND/TIMEOUT`保留原样，不能改成已证明不可达。关闭/取消某批次时写明原因，不删除其行。
