**图：L0_D2的三方法实际路线对照。** 代表场景在查看结果前固定为原布局的spread分布（L0_D2）；不是按最大改善挑选。 本图使用真实Original Grid布局和Opposite-Side Long-Span Tasks任务分布，各行依次为三种任务约束，各列依次为分阶段+共同最少覆盖修复、仅重排和联合局部搜索。灰桌、绿任务点、蓝色实际路径、橙色停靠三角、红色起点以及蓝色矩形底盘足迹/朝向沿用主场景；少量D编号按真实visit_rank显示，省略其余标签不删停靠。蓝线直接来自navigation_paths.csv的有序样本，并按导出L复算，不连接停靠中心伪造路径。失败或未达最少停靠前提时不绘制完整任务路径、不补齐未导出的位姿，标N/A；保留真实地图和任务点。本图覆盖预先声明的全部 9 个场景，三种任务模式为仅位置、朝下抓取和桌面擦拭。主比较中的 Staged + minimum repair 对应原始导出的 minimum_seed，即分阶段选择后接受共同最少覆盖修复；原始 staged 行仅作辅助记录，不把非最少停靠的原始贪心结果悄悄当作等停靠对照。所有方法使用相同机械臂、同场景候选池、冻结任务支持矩阵、真实有向导航查询和共同有限闭环恢复。最少停靠证书只针对各场景共用的有限候选子池与区域任务归属。联合搜索有固定预算，未声称全局最优。

position_only/minimum_seed: L=50.635891m, J=51.342749, docks=16, minimum=16

position_only/reorder_only: L=49.762237m, J=50.469095, docks=16, minimum=16

position_only/joint_local: L=48.152385m, J=49.094863, docks=16, minimum=16

top_down_grasp/minimum_seed: L=83.833749m, J=89.724236, docks=32, minimum=32

top_down_grasp/reorder_only: L=77.617454m, J=82.644003, docks=32, minimum=32

top_down_grasp/joint_local: L=82.098573m, J=86.726657, docks=32, minimum=32

table_wipe/minimum_seed: L=76.463281m, J=80.154652, docks=32, minimum=32

table_wipe/reorder_only: L=68.369168m, J=71.903460, docks=32, minimum=32

table_wipe/joint_local: L=69.152667m, J=70.935618, docks=32, minimum=32
