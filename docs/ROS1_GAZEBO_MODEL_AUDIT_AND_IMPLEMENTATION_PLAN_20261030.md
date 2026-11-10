# ROS 1 / Gazebo模型审查与实施计划

日期：2026-10-30。对应题目：复杂任务中移动平台与机械臂的协同规划研究。

## 1. 当前事实与判断

用户已有Ubuntu 20.04、ROS 1、Gazebo初步场景，决定继续使用该环境；故障表现是**关节能动，但方向、零位或末端位置不对**。工程位于本电脑双系统的Ubuntu侧，目前不可访问。因此本文是可带到Ubuntu端执行的审查/修改计划，尚未读取该工程的URDF、控制器和日志，不能宣布具体根因或修复成功。

按Ubuntu 20.04常见组合，以ROS Noetic＋Gazebo Classic 11＋MoveIt 1（若已使用）为工作假设，先用实际版本命令确认。沿用已有环境，不把迁移ROS 2作为本月前置任务。Noetic已于2025-05-31结束官方支持，Gazebo Classic已于2025年1月结束支持；本月应保存可运行工程、依赖版本与配置快照，避免无必要地更换软件栈。[S1][S2]

当前症状优先指向**模型运动学、坐标系、角度单位/零位与关节名称映射**。只有在目标与模型关系正确、但实测关节跟踪仍明显错误时，才进入接口与控制增益分支。不是预先判定PID无关，也不应先大范围调PID。

最小控制链：轨迹输入→ROS 1 ros_control控制器→gazebo_ros_control→Gazebo关节→实际joint_states/控制器状态。这里用ROS 1的`gazebo_ros_control`，不混入ROS 2的`gazebo_ros2_control/gz_ros2_control/ros2_control`配置。

## 2. 返回Ubuntu后的第一批资料

先保留旧工程副本或版本提交；本计划未在Windows端修改Ubuntu分区。收集：

1. 真实启动入口launch及其include链；实际ROS、Gazebo、MoveIt版本。
2. 顶层URDF/Xacro、左右臂宏、升降与夹爪模型、mesh引用与缩放。
3. 展开后的URDF，以及**运行时实际加载的robot_description**；若Gazebo单独加载SDF，另保存该SDF。
4. ros_control控制器YAML、transmission、插件参数与命名空间。
5. 若使用MoveIt：SRDF、kinematics/joint_limits/controllers配置、末端link与规划组。
6. 同一动作的命令、实测joint_states、控制器desired/actual/error、TF和Gazebo画面；明确每份数据的时间戳和坐标系。

Ubuntu端只读入口（未由本助手执行）：

```bash
rosversion -d
gazebo --version
rosparam get /use_sim_time
rostopic list
rosservice find controller_manager_msgs/ListControllers
rostopic info /joint_states
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /clock
```

`/joint_states`是示例，若实际有命名空间，应从topic列表使用真实路径。对最后一条重复两次确认仿真时间推进。controller_manager服务通过发现结果调用，不假定一定在根命名空间。`check_urdf`可用于检查展开后的URDF结构；语法通过不证明坐标和参数正确。

## 3. 用一个实验区分四类错误

在**单独的单臂诊断场景**中固定机械臂安装座，保留所有应活动关节；先不启用底盘运动、升降变化、另一臂、物体抓取和MoveIt规划。固定安装座是诊断条件，最终移动平台模型仍须恢复正常机构关系。

从无碰撞、远离限位的姿态出发，其他关节保持当前值，每次只让一个转动关节做小幅正、负运动。例如±0.05 rad是测试起点，不适用于所有关节；幅度须根据实际限位、间隙和机构核定。移动关节使用米。轨迹控制器通常仍需收到它所控制的完整关节向量，不能把“单关节试验”误解成随意省略其余关节。

| 观察 | 优先分支 | 下一步 |
|---|---|---|
| 指定joint A，实际joint B变化 | 关节名/数组顺序/重复名称 | 核对joint_names与positions/velocities每一项，禁止按未确认索引映射 |
| 同一joint的命令与实测角度明显不一致 | 控制接口、限位、资源占用、跟踪 | 核对控制器接口与反馈；用误差曲线再判断是否需要调增益 |
| 实测关节值达到目标，但转向/旋转中心/末端仍不对 | origin、axis、零位、TCP、参考系 | 按第4节逐项审模型 |
| RViz与Gazebo在相同真实关节值下仍姿态不同 | 加载模型不同、旧SDF、TF/状态发布冲突 | 比较运行时描述和实际spawn来源，确认非fake状态 |

RViz和Gazebo一致只能证明内部一致；两者可能使用同一份错误URDF。必须再用CAD尺寸、独立测量或实体已知姿态验证。

## 4. 模型审查：按这个顺序修改

### A. 统一模型来源、单位与关节身份

- 确认Gazebo spawn、robot_state_publisher和MoveIt读取相同版本的模型；修改Xacro后重新展开和重新加载，避免仍运行旧模型。
- Gazebo从URDF转SDF时可能合并固定关节，核对的是关节树与变换的语义一致性，不要求两种文件文本完全相同；同时排查旧工作空间overlay。
- joint/link名称全局唯一；左右臂显式前缀。对每个控制数组按joint_names匹配，不能假定joint_states、控制器、LeRobot输出具有相同排序。
- URDF平移单位米，转动关节与rpy单位弧度；STL/CAD导出尺度独立核对。`mesh scale`修好外观不等于关节origin、碰撞体、惯量也已同步修好。
- 对同一关节确认命令所有者及实际状态来源；不让GUI joint_state_publisher或MoveIt fake execution覆盖Gazebo真实状态。多个发布者管理不相交关节可以成立，应排查的是冲突值而非仅看发布者数量。

### B. 关节origin与axis

对于普通转动关节，可按以下关系理解：

`T_parent_child(q) = T_parent_joint_origin × Rotation(axis_in_joint_frame, q)`。

`origin`描述零位时的父link到关节坐标系变换；`axis`在关节坐标系表达，不是直接照抄世界坐标的转轴。先检查parent/child树，再检查旋转中心、origin的xyz/rpy，最后检查axis方向和右手定则。[S3]

- 检查实际铰链中心是否落在预期轴线上；若连杆绕错误位置画圆，先修origin，不用改控制增益。
- 若关节位置数值正确但转向反了，依据机械坐标定义确定是axis、安装变换还是输入符号错误，避免多处同时取负。
- visual/collision各有自己的origin；只旋转mesh可让静态外观看似正确，却不改变真实关节运动学。
- 每次修改一个根因，再重复正负小步并留存对比。不要一次修改axis、offset、mesh和PID，导致无法归因。
- 安装变换与关节旋转的乘法顺序不能交换，rpy顺序也不能凭视觉随意交换；移动关节使用沿轴平移。

### C. 零位及真实技能到ROS的映射

分别记录：CAD设计零位、URDF零位、Gazebo初始关节值、实体/驱动器标定零位和技能记录的零位。它们不一定相同。

对已知为角度的输入，先转换到弧度，再用有标定依据的映射：

`q_ros = a × q_source_rad + b`。

在双方已使用同一关节角单位且无额外传动比例时，a通常为±1；若输入是编码器计数或归一化动作，必须先确认其量程、标定/解归一化和传动含义，不能直接套角度公式。平移关节使用米。命令和反馈的转换应互相一致，禁止驱动层、技能adapter和URDF重复补偿。

用至少三个不同姿态检查映射，不能只在零位“摆对”。改变URDF坐标约定后，相应更新关节限位、home姿态、命名姿态、技能数据适配和测试记录；不得让新旧角度约定混用。

检查角度环绕、归一化及夹爪mimic关系；被动/耦合关节不应被当作互不相关的主动关节重复驱动。

### D. 左右臂镜像

先独立验证左臂，再独立验证右臂。机械镜像、安装旋转与mesh镜像不是同一回事；几何反射不能直接当作合法旋转变换。不能仅改变安装点y正负、给mesh负缩放，或把所有右臂角度统一取负后就认定正确。

每个关节分别记录安装轴、正方向和零位，并使用右手坐标系。左右臂对称任务需要分别计算目标和关节映射，不假定相同角度数组就产生镜像动作。

### E. 末端TCP与升降链

确认控制的末端究竟是最后一节link、法兰、夹爪基座、夹指接触点还是工具中心TCP。以显式固定变换定义TCP；规划目标与评测必须使用同一工具定义，而不是随意给目标坐标加常量。

整机检查链：

`world/map → base → lift carriage → left/right mount → arm joints → tool/TCP`。

固定两臂关节角，只改变升降h：两侧安装座及TCP应按照升降轴和实际结构一起变化。理想竖直直线机构的位移应与h一致；其他结构使用实际标定关系。再固定h和关节角，只改变底盘姿态：世界坐标应按刚体变换变化，臂基座坐标下的TCP应保持不变。

表面高度350/660 mm、物体接触点高度、升降行程和TCP世界高度分开记录。升降不能既作为URDF prismatic joint计算，又在任务adapter里额外重复加一次高度。

### F. 最后检查动态与控制参数

方向、零位、TCP与坐标链正确后，再检查轨迹是否跟踪：质量/惯量、质心、碰撞体、初始接触、限位、速度/力矩上限、时间尺度与PID。URDF在Gazebo中还涉及SDF转换与物理属性，视觉正确不等于物理模型正确。[S2]

Noetic的DefaultRobotHWSim支持位置、速度和力矩接口，控制器类型与transmission接口必须一致；不要照陈旧教程一律改为effort，也不要看到“No p gain”就直接认定它是方向错误的根因。[S4][S5]

## 5. 控制层、MoveIt与双臂的验收顺序

| 层级 | 验收内容 | 本级不通过时 |
|---|---|---|
| J1 单关节 | 每个关节正负方向、旋转中心、实际数值正确 | 返回模型/映射检查，不接规划算法 |
| J2 单臂 | 多个关节姿态的TCP与独立几何参考一致；小幅轨迹可重复 | 检查运动链与工具坐标，保留完整失败记录 |
| J3 左右臂 | 分别通过，联合无载运动无命名/资源冲突 | 不进入共同夹物实验 |
| J4 升降 | 至少两个已验证h下，安装座和TCP变换正确 | 检查链顺序、轴、零点及重复加高度 |
| J5 底盘 | 位置/朝向变化后目标转换正确，停稳后动作可重复 | 检查map/odom/base变换及实际到位反馈 |
| J6 整链 | 取件—携物—放置各阶段有独立结果与状态 | 分类失败，不能以“指令已发送”判成功 |

验收容差先按任务间隙、模型精度和测量精度确定并记录，再执行；不能看到结果后放宽。无载重复10次可作为工程推进检查，不证明可靠性概率达到某一水平。

若已使用MoveIt 1：底层直接关节轨迹通过后，再检查真实FollowJointTrajectory action与MoveIt controller_list中的name/action_ns/joints是否对应，关闭fake execution用于真实仿真执行。MoveIt能规划不保证Gazebo执行器已接通。[S6]

末端目标还需符合机械臂实际自由度和任务容差；若欠驱动，不默认任意完整六维位姿都存在IK解。双臂规划组或同步发指令也不自动保证对同一物体的闭链与受力约束。

## 6. 单关节/模型审查记录模板

每个主动关节一行，夹爪mimic/passive关节另外标明。下表为空白模板，不预填真实型号或标定结果。

| joint名称 | 侧/类型 | parent→child | origin xyz/rpy | axis局部坐标 | 源单位与零位 | a/b映射 | 限位 | +小步实际方向 | -小步实际方向 | TCP参考与误差 | 结论/证据 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 | TODO |

独立几何核对至少保存：姿态A/B/C的关节向量、安装座坐标、TCP坐标、参考来源（CAD/测量/实体）、模型计算与Gazebo观测。使用同一URDF生成的两套结果只能作内部一致性验证。

## 7. 与现有MATLAB代码的接入及修改优先级

当前仓库`docs/real_platform_integration.md`原本就定义了Ubuntu 20.04/ROS Noetic执行边界；保留MATLAB离线规划，Ubuntu侧新增薄的rospy/actionlib任务执行器。实时伺服仍由现有底盘/关节控制器承担。

`VALIDATE → NAVIGATE → ARRIVAL_CHECK → LIFT → ARM_APPROACH → GRASP/RELEASE → CONFIRM → TRANSPORT/NEXT`；失败进入所有方法共用的有限恢复或明确失败状态。

| 优先级 | 修改处 | 具体要求 |
|---|---|---|
| P0 | Ubuntu模型及技能adapter | 完成第4–5节；不在Windows侧凭PDF猜关节参数 |
| P0 | 显式仿真profile | 固定模型、控制器、技能、零位标定版本，启用真正参与评价的升降；保留旧模型基准 |
| P0 | scene/task生成 | 每工位高度、物体/抓取偏移、成对pick/place及持物状态 |
| P0 | `docking/expand_lift_candidates.m`及选择层 | 高度在可行性/代价评价前展开；导出最终选中配置，而非所有h候选 |
| P0 | `integration/export_mission_plan_json.m`与schema | 路线节点与服务ID显式映射；区分dock_id/candidate_id/selected_service_id，避免重复ID |
| P0 | `build_service_config`与validator | 统一`dual_independent`等枚举；校验单位、引用、范围和状态条件，不只检查字段存在 |
| P0 | Ubuntu执行器 | 解析JSON，等待action结果并复核实际状态；时间戳与use_sim_time统一 |
| P1 | 路径与误差评价 | 携物包络、真实到位复核、相同恢复预算；误差数据来源明确 |
| P1 | 日志导入与分析 | 区分sim/real、contact/attach_proxy、真值/估计；保留阶段、对象状态、失败与版本 |

当前Stage3路线的索引1是起点，服务配置ID也从1开始，二者不能直接等同；展开多个h时当前代码还会复制相同ID。应使用显式映射，不能简单假定所有情况下索引减1就够。

现有JSON只导出服务目标，未包含完整导航路径和可执行双臂轨迹。二选一并固定：①研究服务配置时，三方法使用同一ROS导航器；②研究MATLAB路径算法时，另导出路径段并由共同跟踪器跟随。不能把ROS重新生成路径的效果归因于原MATLAB路径算法。

最小补充字段：mission_id、robot_model_version、action、object_id、target_pose及frame、skill_id/version、pre/postcondition、transport_mode、timeout/retry_budget。JSON传任务，控制器做实时跟踪，不让跨系统文件交换承担高频伺服。

## 8. 一个月计划与证据边界

| 时间 | 工作 | 必须达到的门槛 |
|---|---|---|
| 第1–2天 | 快照、运行模型核对、单臂逐关节诊断 | 确定错误属于关节映射/运动学/跟踪中的哪一类 |
| 第3–5天 | 修origin/axis/零位/TCP；分别验收两臂 | 关节与TCP在多个姿态一致，实际轨迹可重复 |
| 第6–7天 | 升降与底盘坐标链，联合无载动作 | J1–J5通过；否则继续模型修复，暂不启动算法优劣比较 |
| 第2周 | 一箱、两工位完整流程；接JSON；实现A/B/C共同后端 | 路线、状态、碰撞、反馈可追溯；小例能解释配置差异 |
| 第3周 | 冻结参数，完成90次主仿真 | 同候选、同控制、同恢复、同误差协议；所有失败保留 |
| 第4周 | 机制消融、统计、论文图；条件允许做12次代表性真机核查 | 分清仿真与真机、理想抓取与接触抓取，结论不超证据 |

默认主仿真90次＝3场景×2误差条件×5种子×3方法；详细协议见`paper/EXPERIMENT_PLAN.md`。有稳定接口再于正式测试前扩至180次，不把60次真机与新仿真任务机械叠加。

若两臂运动学/控制仍错误，物体附着不能把失败掩盖成有效整机实验。如果两臂、升降和碰撞验证已通过，仅接触抓取难以稳定，可采用有条件附着的任务级替代执行器。必须检查两侧目标关系、碰撞和持物状态；此时主指标称“在替代技能条件下的仿真任务执行成功率”，不称实际自主抓取成功率。

关闭碰撞、无限增大力矩、永久关闭重力、把活动关节改成fixed或直接瞬移末端，均不能作为修复完成的证据。若使用理想化底盘或抓取模型，应在论文中明确其假设及不能支持的结论。

本月不新增大模型训练、通用双臂力控或连续全身控制。实际技能是否为学习策略尚未确认；不能因文档写有ACT就假定用户正在使用ACT。

## 9. 官方与维护者资料

- [S1 ROS Noetic结束支持公告](https://discourse.openrobotics.org/t/ros-noetic-end-of-life-may-31-2025/43160)
- [S2 Gazebo Classic：URDF模型与物理属性](https://classic.gazebosim.org/tutorials?tut=ros_urdf)
- [S3 URDF关节数据结构：axis与parent_to_joint_origin_transform](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/joint.h)
- [S4 Noetic DefaultRobotHWSim实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp)
- [S5 Noetic轨迹控制器类型](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/ros_control_plugins.xml)
- [S6 MoveIt 1 Noetic控制器映射与fake execution](https://moveit.github.io/moveit_tutorials/doc/controller_configuration/controller_configuration_tutorial.html)

本次只生成计划与审查资料，未访问Ubuntu工程、未修改机器人模型、未安装软件、未执行控制命令。
