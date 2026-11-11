**图：真实主场景的导航结果与搜索开销。** 仅展示 summary.csv 中实际完成的方法组合，未为缺失方法填充数据。(a) 原始路线距离；(b) 加权导航代价，两者使用独立单位的纵轴。加权代价为实际距离加 0.15 m/rad 乘以相邻访问端点的 wrapped 朝向差绝对值总和；朝向项不是沿实际路径逐点转角的积分。(c) 停靠数量；(d) selection_time_s，该阶段耗时不能解释为端到端总规划时间或机器人执行时间，构图、路径查询和最终路径生成的计时范围以原始结果和协议为准。minimum_seed 是对 baseline 停靠覆盖种子的显式修复；若存在 reorder_only，它仅对修复后的最少停靠种子重排访问顺序，用于分离重排与联合选择的影响。joint_local 为有限预算局部搜索，不保证固定候选池全局最优，也不保证达到最少停靠数。所有结果来自一次运行，不绘制未观测的误差条，不声称统计显著性或普遍改进。覆盖不足或导航失败时，该方法四个面板的柱形均留空并标 N/A，缺失的 NaN 代价不按零处理，原始失败记录仍保留。

Position only：Staged：覆盖 36/36，停靠 16，距离 53.966 m，加权代价 55.536 m-equivalent，selection_time_s=0.238 s，route_valid=1；Minimum seed：覆盖 36/36，停靠 16，距离 53.966 m，加权代价 55.536 m-equivalent，selection_time_s=0.084 s，route_valid=1；Reorder only：覆盖 36/36，停靠 16，距离 48.179 m，加权代价 49.672 m-equivalent，selection_time_s=0.879 s，route_valid=1；Joint local search：覆盖 36/36，停靠 16，距离 42.191 m，加权代价 43.570 m-equivalent，selection_time_s=67.281 s，route_valid=1。

数据：summary.csv、metadata.json。
