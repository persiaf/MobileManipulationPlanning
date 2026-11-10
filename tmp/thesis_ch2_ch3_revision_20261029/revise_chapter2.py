import copy
import json
import re
from pathlib import Path

ROOT=Path(__file__).parent
all_blocks=json.loads((ROOT/'baseline_content.json').read_text(encoding='utf-8'))
end=next(i for i,b in enumerate(all_blocks) if b.get('type')=='heading' and b.get('level')==1 and b.get('text','').startswith('第3章'))
B=copy.deepcopy(all_blocks[:end])
def P(text):return {'type':'p','text':text}
def E(label,latex):return {'type':'eq','label':label,'latex':latex}
def T(caption,headers,rows):return {'type':'table','caption':caption,'headers':headers,'rows':rows}
def idx_text(prefix):
    hits=[i for i,b in enumerate(B) if b.get('text','').startswith(prefix)]
    assert len(hits)==1,(prefix,hits)
    return hits[0]
def repl(prefix,text):B[idx_text(prefix)]['text']=text
def after_text(prefix,blocks):
    i=idx_text(prefix);B[i+1:i+1]=blocks
def repl_eq(label,latex):
    hits=[b for b in B if b.get('label')==label];assert len(hits)==1
    hits[0]['latex']=latex
def after_eq(label,blocks):
    i=next(i for i,b in enumerate(B) if b.get('label')==label);B[i+1:i+1]=blocks

B[0]['text']='第2章 移动双臂机器人系统建模与规划问题描述'
repl('其中，$x_b$',r'其中，$x_b$、$y_b$ 为底盘参考点在世界坐标系中的平面坐标，$\theta_b$ 为航向角，$h$ 为升降机构相对于参考高度的位移，$\boldsymbol q_L$、$\boldsymbol q_R$ 分别为左右机械臂关节变量，$n_L$、$n_R$ 为相应关节数。本章保留一般关节维数，以便区分系统构型定义与具体机械臂参数。表2-2列出的五组关节限位对应本文仿真参数模型，不能仅凭限位数量认定最终实机的关节构成；仿真模型与实验平台运动链的对应关系还需统一验证。广义构型用于描述各部分之间的运动学关系，不意味着规划算法同时搜索全部自由度。')
repl('表2-1给出',r'表2-1给出本文几何规划所采用的参数。机械臂几何评价原点位于底盘前部，前向偏移取底盘长度的一半。导航碰撞和停靠碰撞使用相同的矩形底盘外廓，以保持对机器人占据空间的描述一致。这组参数用于构造候选位姿与任务之间的几何关系；工作空间边界属于代理模型参数，尚不能替代依据实际关节构型、工具几何和碰撞条件确定的安全操作范围。')

# Pose parameters and rigid transforms are different objects.
repl_eq('2-2',r'\boldsymbol b=\begin{bmatrix}x_b&y_b&\theta_b\end{bmatrix}^{T}\in\mathcal X_B,\qquad\mathcal X_B=\mathbb R^2\times S^1')
repl('其中，$SE(2)$',r'其中，$S^1$ 表示航向角的周期空间，$\theta_b$ 与 $\theta_b+2\pi$ 表示同一方向，逆时针转动为正。$\boldsymbol b$ 是底盘位姿的坐标表示，而 $SE(2)$ 的元素是平面刚体变换，两者通过下式关联[1]：')
after_text('其中，$S^1$',[
 E('2-A1',r'T_2(\boldsymbol b)=\begin{bmatrix}\cos\theta_b&-\sin\theta_b&x_b\\\sin\theta_b&\cos\theta_b&y_b\\0&0&1\end{bmatrix}\in SE(2)'),
 P(r'下文以 $\boldsymbol b\in\mathcal X_B$ 表示当前底盘位姿坐标，以 $p_k\in\mathcal X_B$ 表示候选停靠位姿；相应的刚体变换分别为 $T_2(\boldsymbol b)$ 与 $T_2(p_k)$。碰撞集合、自由构型空间和路径均定义在坐标空间 $\mathcal X_B$ 中，坐标变换则通过齐次矩阵完成。为嵌入三维机械臂运动链，底盘旋转矩阵和三维齐次变换写为')
])
after_text(r'式中，$\boldsymbol R_{A_\ell}$',[
 P(r'图2-1给出平面几何评价坐标系及双臂安装偏置。图中的机械臂评价轴与底盘轴保持同向，用于计算任务点相对于两侧基座的前后、侧向和竖直距离；实际关节运动学采用的连杆轴仍由机械臂结构确定。'),
 {'type':'figure','path':'FIG_COORD','caption':'图2-1 移动底盘与双臂几何评价坐标系'}
])

# Conditional mecanum model; not a claim about unverified hardware.
repl('该模型允许底盘',r'该模型允许底盘在保持航向不变时进行侧向平移，适合表达靠近桌面、沿桌边调整和从停靠位姿退出等动作。式（2-11）描述底盘刚体速度，不直接确定各车轮转速。轮速映射还取决于驱动轮类型、滚子方向和安装位置，只有在这些条件给定后才能建立具体关系。')
after_text('该模型允许底盘',[
 P(r'以常见的四麦克纳姆轮X型滚子布置为条件模型，设各轮半径均为 $r_w>0$，轮中心相对于底盘中心的纵向、横向距离分别为 $l_x$、$l_y$，并记 $a=l_x+l_y>0$。车轮顺序为左前、右前、左后、右后，轮速正方向取使车轮产生前向运动的转向；在对应滚子方向约定和理想接触条件下，可得'),
 E('2-A2',r'\begin{bmatrix}\omega_{FL}\\\omega_{FR}\\\omega_{RL}\\\omega_{RR}\end{bmatrix}=\frac{1}{r_w}\begin{bmatrix}1&-1&-a\\1&1&a\\1&1&-a\\1&-1&a\end{bmatrix}\begin{bmatrix}v_x^B\\v_y^B\\\omega_b\end{bmatrix}'),
 P(r'记式（2-A2）中的四行三列系数矩阵为 $A$，由于 $A^TA=\operatorname{diag}(4,4,4a^2)$，对轮速观测采用左伪逆可得到对应的底盘速度：'),
 E('2-A3',r'\begin{bmatrix}v_x^B\\v_y^B\\\omega_b\end{bmatrix}=\frac{r_w}{4}\begin{bmatrix}1&1&1&1\\-1&1&1&-1\\-1/a&1/a&-1/a&1/a\end{bmatrix}\begin{bmatrix}\omega_{FL}\\\omega_{FR}\\\omega_{RL}\\\omega_{RR}\end{bmatrix}'),
 P(r'上述关系用于说明一种全向底盘实现的速度分配条件。改变滚子手性、轮速正方向或车轮排列，会相应改变矩阵符号；轮径和半轴距也必须由具体底盘测量确定。本文未以该条件模型认定实验平台的实际轮型，未给未测量的 $r_w$、$l_x$、$l_y$ 赋予数值，后续几何规划仍以式（2-11）的全向刚体模型为基础。')
])
after_eq('2-12',[
 P(r'式（2-12）是航向角局部坐标下的一阶近似。数值更新后应按 $2\pi$ 周期归一化角度，航向差采用周期意义下的最短角差；直接相减未归一化的角度表示可能产生虚假的大幅旋转。')
])
repl('从起点到停靠点的移动路径',r'从起点到停靠点的移动路径写为 $\gamma:[0,1]\rightarrow\mathcal X_B$，其对应刚体运动为 $T_2\circ\gamma$。若要在整个移动过程中保持无碰，应满足')

# DH provenance and task dimension.
repl('机械臂正运动学描述',r'机械臂正运动学描述关节变量到末端位姿的映射。对任意一侧机械臂，采用标准DH形式定义相邻连杆变换[2]；令 $a_k$、$\alpha_k$、$d_k$、$\vartheta_k$ 分别为连杆长度、扭转角、偏距和关节角，则')
repl('末端工具变换',r'末端工具变换 $^{n_\ell}T_{E_\ell}$ 表达末级连杆到工具坐标系的固定关系。将该式代入式（2-8），即可把底盘、升降机构、机械臂和工具统一到同一运动链中。完整的数值DH模型应同时给出每一行 $a_k$、$\alpha_k$、$d_k$、关节变量类型与零位偏移，并明确基座和工具坐标系，仅有若干连杆尺寸与角度偏移不能唯一确定整条运动链。')
after_text('末端工具变换',[
 P(r'本章几何参数与拟采用实机的运动链对应关系尚未完成验证，尤其需要核实有效关节数、关节轴方向、零位定义及工具末端变换。因此，本节保留参数化运动学表达，不将未确认的结构尺寸拼接为实际DH参数表。安装高度 $z_A$ 已计入机械臂几何评价原点的竖直偏移，建立关节运动链时还应重新检查基座原点，避免将同一结构偏距重复计入。后续逆运动学结果只有在上述模型对应关系明确后，才能作为实际操作可行性的证据。')
])
repl('对于规定完整目标位姿',r'对于规定完整目标位姿的任务，逆运动学需要寻找满足下式的关节构型[1]：')
repl('当任务只规定目标位置',r'式（2-20）对末端位置和完整姿态同时施加要求，并不保证给定目标一定有解。对于五自由度机械臂，任意指定六维末端位姿通常超出可独立控制的任务维度。桌面抓取可采用“给定位置与接近轴、允许绕该轴转动”的任务形式，令 $\boldsymbol n_i$ 为单位期望接近方向，$\boldsymbol e_z=[0,0,1]^T$，则允许姿态集合定义为')
after_text('式（2-20）对末端位置',[
 E('2-A4',r'\mathcal A_i=\{R\in SO(3):R\boldsymbol e_z=\boldsymbol n_i\},\qquad\|\boldsymbol n_i\|_2=1'),
 E('2-A5',r'\boldsymbol p_{E_\ell}^{W}(\boldsymbol b,h,\boldsymbol q_\ell)=\boldsymbol p_i^W,\qquad R_{E_\ell}^{W}(\boldsymbol b,h,\boldsymbol q_\ell)\in\mathcal A_i'),
 P(r'固定一个单位方向通常对应两个独立方向约束，加上三个位置约束，式（2-A5）在一般非退化情形下包含五个独立任务约束。这一维数匹配并不意味着五自由度机械臂对所有目标均可解：连杆几何、关节限位、奇异构型与碰撞条件仍可能导致无解。若任务只要求位置，可进一步放宽姿态集合；若要求完整姿态，则应按式（2-20）判断，不能以位置可达替代姿态可达。无论采用哪种任务表达，关节变量首先须位于关节限位集合中：')
])

# Distinguish limit workspace, static safety workspace, and path safety.
repl('表2-2中的关节范围',r'表2-2给出的关节范围构成本文参数模型的允许域。首先不考虑碰撞，只对关节限位允许域取末端位置像，得到关节限位下的位置工作空间：')
repl_eq('2-22',r'\mathcal W_\ell^{\mathrm{lim}}(\boldsymbol b,h)=\{\boldsymbol p_{E_\ell}^{W}(\boldsymbol b,h,\boldsymbol q_\ell)\mid\boldsymbol q_\ell\in\mathcal Q_\ell\}')
repl('由此可见，目标落在位置工作空间',r'式（2-22）未包含机械臂碰撞，不能称为安全工作空间。为明确静态安全条件，令 $\bar\ell$ 表示另一侧机械臂，$\boldsymbol q_{\bar\ell}^{\mathrm{park}}$ 为其已指定的停车构型，$\mathcal O_3$ 为三维环境几何。定义 $\operatorname{CF}_3$ 在整个机器人构型与环境无碰、无自碰且两臂无互碰时取1，否则取0。则单臂静态安全构型集合及其位置工作空间为')
after_text('式（2-22）未包含机械臂碰撞',[
 E('2-A6',r'\begin{aligned}\mathcal Q_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3)=\{\boldsymbol q_\ell\in\mathcal Q_\ell:\quad\\\operatorname{CF}_3(\boldsymbol b,h,\boldsymbol q_\ell,\boldsymbol q_{\bar\ell}^{\mathrm{park}};\mathcal O_3)=1\}.\end{aligned}'),
 E('2-A7',r'\begin{aligned}\mathcal W_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3)=\{\boldsymbol p_{E_\ell}^{W}(\boldsymbol b,h,\boldsymbol q_\ell):\quad\\\boldsymbol q_\ell\in\mathcal Q_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3)\}.\end{aligned}'),
 P(r'在相同底盘状态和高度下，$\mathcal W_\ell^{\mathrm{safe}}\subseteq\mathcal W_\ell^{\mathrm{lim}}$。另一侧机械臂的停车构型改变时，互碰关系和安全工作空间也可能改变，因此该构型必须作为安全条件的参数，不能隐去后笼统声称单臂无碰。若两臂同时运动，则应在联合构型空间 $\mathcal Q_L\times\mathcal Q_R$ 中检查碰撞，静止另一臂的模型不再适用。'),
 P(r'静态安全工作空间仍只表示存在安全末端构型。给定单臂起始构型 $\boldsymbol q_{\ell,0}$ 和满足任务约束的目标构型 $\boldsymbol q_{\ell,g}$，还需存在连接二者的连续关节路径 $\boldsymbol\zeta_\ell$，使其整个过程均位于同一安全构型集合内：'),
 E('2-A8',r'\begin{aligned}\boldsymbol\zeta_\ell(0)&=\boldsymbol q_{\ell,0},\qquad\boldsymbol\zeta_\ell(1)=\boldsymbol q_{\ell,g},\\\boldsymbol\zeta_\ell(s)&\in\mathcal Q_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3),\quad\forall s\in[0,1].\end{aligned}'),
 P(r'安全构型集合可能具有多个不连通分支，故目标静态无碰不足以保证式（2-A8）成立。路径通过几何验证后，仍需满足速度、加速度和执行控制约束。第三章的几何代理不直接计算上述三维碰撞集合与关节路径，其输出只能作为候选筛选和覆盖规划的依据，不能替代完整逆运动学及动作验证。')
])
repl(r'其中，$\boldsymbol p_i^W$',r'其中，$\boldsymbol p_i^W$ 为目标位置，$\mathcal A_i\subseteq SO(3)$ 为允许的末端姿态集合，$\kappa_i$ 为任务类型，$j_i$ 为工作区域编号。采用 $\mathcal A_i$ 表示姿态集合，以区别于工作区域集合。要求接近方向的任务可采用式（2-A4），一般位姿任务可令该集合仅包含指定旋转。第三章的点任务评价以位置和区域归属为核心，尚未将接近轴约束作为经过验证的几何评分条件，因此其覆盖结果不包含指定抓取姿态已经可行的结论。')

# Current grid convention is valid when stated, and differs from lower-inclusive on lines.
repl('本文以二维栅格地图',r'本文以二维栅格地图表示底盘移动环境。设栅格边长为 $r_g$，世界坐标参考原点为 $(x_0,y_0)$，地图列数、行数分别为 $N_x$、$N_y$。以下采用 $W=N_xr_g$、$H=N_yr_g$ 的完整栅格矩形；若物理地图尺寸不是分辨率的整数倍，应另外保留实际边界并裁剪末端单元，不能把取整扩大的范围视为真实自由空间。环境矩阵的零值表示自由区域，非零值表示底盘不可进入的区域；桌面投影可保留不同语义值，但均纳入二值占据掩码：')
repl('栅格矩阵的行号自上向下增加',r'栅格矩阵的行号自上向下增加，而世界坐标 $y$ 自下向上增加。本文已有仿真采用横向左含右不含、纵向下不含上含的半开约定，即有效索引域为 $x\in[x_0,x_0+W)$、$y\in(y_0,y_0+H]$。与这一约定对应的第 $(g_i,g_j)$ 个单元为')
after_text('栅格矩阵的行号自上向下增加',[
 E('2-A9',r'\begin{aligned}\mathcal G_{g_i,g_j}={}&[x_0+(g_j-1)r_g,\ x_0+g_jr_g)\\&\times(y_0+H-g_ir_g,\ y_0+H-(g_i-1)r_g].\end{aligned}'),
 P(r'在有效索引域内，采用一基行列索引，连续坐标到单元的映射为')
])
repl('其中 $g_i$ 为行号',r'其中 $g_i$ 为行号、$g_j$ 为列号，结果必须满足 $1\le g_i\le N_y$、$1\le g_j\le N_x$。在上述约定下，$y=y_0$ 被排除，$y=y_0+H$ 映射到首行；这属于单元边界归属的选择，不能脱离有效域把下边界的越界索引当作有效单元。索引域包含某个地图边缘点，也不意味着底盘可以停在那里，底盘外廓仍须严格位于地图内部。反向转换取对应单元中心：')
repl('连续坐标映射到同一栅格',r'连续坐标映射到同一单元后会丢失单元内部的位置差异，因此往返转换得到的是栅格中心，而非原连续点。另一种常见约定是纵向下含上不含，此时在 $y\in[y_0,y_0+H)$ 内应使用 $g_i^{\mathrm{lower}}=N_y-\lfloor(y-y_0)/r_g\rfloor$。两种约定对于非水平网格线上的点给出相同行号，但在 $y=y_0+kr_g$ 的内部网格线上，式（2-28）给出 $N_y-k+1$，下含上不含约定给出 $N_y-k$，二者并非无条件等价。')
after_text('连续坐标映射到同一单元',[
 P(r'本章明确沿用已有仿真的上含下不含纵向约定，第三章报告的已有结果亦保持该映射，没有将更换边界约定后的结果与原结果混用。数值计算中对接近整数的比值进行浮点对齐，只用于消除舍入误差，不能替代有效域与索引范围检查；越界点不应通过钳制行列号强行归入边界单元。碰撞检测还须按完整单元几何处理接触关系，不以单点索引是否有效替代底盘外廓校验。')
])
repl_eq('2-32',r'\begin{aligned}\mathcal O(t)&=\mathcal O_s\cup\mathcal O_d(t),\\\mathcal C_{\mathrm{free}}(t)&=\{p\in\mathcal X_B:\mathcal F(p)\cap\mathcal O_{\mathrm{col}}(t)=\varnothing,\\&\hspace{5em}\mathcal F(p)\subset\operatorname{int}(\mathcal D)\}.\end{aligned}')
repl_eq('2-33',r'\mathcal P_j=\{p_{j1},\ldots,p_{jK_j}\},\qquad p_{jk}=(x_{jk},y_{jk},\theta_{jk})\in\mathcal X_B')
repl(r'其中，$\Omega_{\mathrm{free}}$',r'上述集合均为 $\mathcal X_B$ 的子集。其中，$\Omega_{\mathrm{free}}=\mathcal C_{\mathrm{free}}$ 由当前地图和底盘碰撞模型定义，$\Omega_{\mathrm{reach},j}$ 描述任务几何覆盖条件，$\Omega_{\mathrm{orient},j}$ 表示候选采样考虑的朝向范围。若另外设置硬安全间隙，可再与相应安全域求交；只设置软安全代价时则不应添加并未执行的硬约束。')

# Dimensionless objective; measured computation time is a metric, not an oracle cost.
after_text('第一个约束要求每个任务只分配一次',[
 P(r'仅考虑选择最少候选以覆盖全部任务时，该问题具有集合覆盖的基本结构[5]。将覆盖选择与障碍环境中的访问顺序和路径连接共同优化后，不能直接把单独集合覆盖问题的贪心性能结论推广为整体规划的最优性或近似保证。')
])
repl_eq('2-39',r'\begin{aligned}\min_{\boldsymbol X,\boldsymbol y,\sigma,\Gamma}\quad&\alpha\frac{N_{\mathrm{dock}}}{N_0}+\beta\frac{L_{\mathrm{nav}}}{L_0}+\gamma\frac{J_{\mathrm{op}}}{J_0},\\\mathrm{s.t.}\quad&N_{\mathrm{dock}}=\sum_{k=1}^{K}y_k,\quad\text{constraints in (2-37)},\\&\gamma_e([0,1])\subseteq\mathcal C_{\mathrm{free}},\quad\forall\gamma_e\in\Gamma.\end{aligned}')
repl(r'式中，$J_{\mathrm{op}}$',r'式中，$J_{\mathrm{op}}$ 汇总所选停靠位姿的操作相关代价；$N_0>0$、$L_0>0$、$J_0>0$ 分别为停靠数量、路径长度和操作代价的固定参考尺度，使三个比值均无量纲。参考尺度应在同一实验组比较前统一指定，例如从固定问题规模、任务区域尺寸或共同基准方案确定，不随某次候选解改变；即使基准操作代价为零，也必须预先指定正的 $J_0$。无量纲权重满足 $\alpha,\beta,\gamma\ge0$ 且 $\alpha+\beta+\gamma=1$，用于表达相对偏好。')
after_text(r'式中，$J_{\mathrm{op}}$',[
 P(r'式（2-39）给出统一的问题定义，并非对已有算法内部权重的重新拟合。任务分配仍受式（2-37）约束，无可行覆盖或无可行路径时，不能靠降低其他代价将不可行解变为可行。后续分层算法根据这一问题结构构造解，其数值评价应说明实际采用的参考尺度和权重，不据此宣称获得联合问题的全局最优解。')
])
repl_eq('2-42',r'\min_{\Pi_{t+1}}\quad\lambda_L\frac{L_{\mathrm{rem}}(\Pi_{t+1})}{L_{\mathrm{ref}}}+\lambda_D\frac{D_{\mathrm{edit}}(\Pi_{t+1},\Pi_t)}{D_{\mathrm{ref}}}')
repl(r'其中，$L_{\mathrm{rem}}$',r'其中，$L_{\mathrm{rem}}$ 为从机器人当前位姿出发的剩余移动代价，$D_{\mathrm{edit}}$ 为新旧方案之间的修改代价，可用停靠替换、任务重新分配和访问边变化等量定义。$L_{\mathrm{ref}}>0$ 与 $D_{\mathrm{ref}}>0$ 是一次变化事件比较中预先固定的正参考尺度，$\lambda_L,\lambda_D\ge0$ 且 $\lambda_L+\lambda_D=1$。新计划的路径首端必须对应当前位姿，已完成任务保持完成状态，仅对剩余任务重新验证覆盖与连接关系。')
after_text(r'其中，$L_{\mathrm{rem}}$',[
 P(r'实测重规划时间 $T_{\mathrm{replan}}$ 只有在规划过程执行后才能获得，若缺少事先可评价的耗时预测器，就不能把它当作任意待选方案的已知目标函数值。因此，式（2-42）不包含实测计算时间；该时间作为独立效率指标，与修复成功率、路径变化和计划修改量共同报告。需要限制响应时间时，可在算法层设置计算预算，超时则返回明确状态，而不将预算误写成已经获得的最优计算耗时。')
])
repl('本章建立了移动双臂机器人的统一建模框架',r'本章建立了移动双臂机器人系统与规划问题的统一表达。首先区分位姿坐标空间与刚体变换群，通过多坐标系变换描述固定升降高度下底盘位置、航向和双臂安装偏置之间的关系，利用矩形外廓与占据栅格定义底盘碰撞条件。随后从关节限位工作空间、静态安全工作空间和关节路径安全三个层次说明机械臂操作可行性，并明确方向任务的维数条件不等于逆运动学解必然存在。')
repl('在此基础上，本章将多桌面任务组织',r'在此基础上，本章明确栅格单元的半开边界约定，将多桌面任务组织为区域化任务集合，依次定义可达停靠生成、多停靠任务与路径规划以及环境变化下在线调整问题。联合目标使用正参考尺度统一量纲，实测重规划时间单独作为效率指标。上述模型为候选覆盖和执行计划提供一致接口；实际运动链、完整DH参数及三维碰撞模型仍须与实验平台对应后，才能支持超出几何代理范围的操作结论。下一章据此构建候选生成与几何评价方法。')

# Avoid English cross-reference prose embedded in a display equation;
# constraints are explicitly listed to make renumbering independent of LaTeX text.
repl_eq('2-39',r'\begin{aligned}\min_{\boldsymbol X,\boldsymbol y,\sigma,\Gamma}\quad&\alpha\frac{\sum_k y_k}{N_0}+\beta\frac{L_{\mathrm{nav}}}{L_0}+\gamma\frac{J_{\mathrm{op}}}{J_0},\\\mathrm{s.t.}\quad&\sum_kx_{ik}=1,\quad x_{ik}\le C_{ik}y_k,\quad x_{ik},y_k\in\{0,1\},\\&\gamma_e([0,1])\subseteq\mathcal C_{\mathrm{free}},\quad\forall\gamma_e\in\Gamma.\end{aligned}')

assert len({b['label'] for b in B if b['type']=='eq'})==sum(b['type']=='eq' for b in B)
assert not any(ord(c)<32 for b in B for c in b.get('text',''))
assert all(b.get('text','').count('$')%2==0 for b in B)
out=ROOT/'chapter2_revised.json'
out.write_text(json.dumps(B,ensure_ascii=False,indent=2),encoding='utf-8')
print(out)
print('blocks',len(B),'equations',sum(b['type']=='eq' for b in B),'Chinese chars',len(re.findall(r'[\u4e00-\u9fff]',json.dumps(B,ensure_ascii=False))))
print('SE2 uses:',[(b.get('label'),b.get('text',b.get('latex'))) for b in B if 'SE(2)' in b.get('text',b.get('latex',''))])
