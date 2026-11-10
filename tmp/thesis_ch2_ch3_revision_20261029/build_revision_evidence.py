import csv
import hashlib
import json
import math
import struct
from pathlib import Path
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
THESIS=Path(r'D:/file_used/西柚/实验室_任务/硕士论文相关')
ROS=THESIS/'小论文/ros代码调试/1'
inventory=json.loads((OUT/'urdf_inventory.json').read_text(encoding='utf-8'))

def vec(text): return np.array([float(v) for v in text.split()])
def origin_matrix(origin):
    xyz=vec(origin.get('xyz','0 0 0')); roll,pitch,yaw=vec(origin.get('rpy','0 0 0'))
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    T=np.eye(4);T[:3,:3]=rz@ry@rx;T[:3,3]=xyz
    return T

static=[]
for inv in inventory:
    bychild={j['child']['link']:j for j in inv['joints']}
    def chain_zero(link):
        chain=[]
        while link!='base_link':
            j=bychild[link];chain.insert(0,j);link=j['parent']['link']
        T=np.eye(4)
        for j in chain:T=T@origin_matrix(j.get('origin',{}))
        return T,[j['name'] for j in chain]
    positions={};chains={}
    for link in ['left_base','right_base','left_link1','right_link1']:
        T,chain=chain_zero(link);positions[link]=T[:3,3].tolist();chains[link]=chain
    l=np.array(positions['left_base']);r=np.array(positions['right_base'])
    arm_joints=[j for j in inv['joints'] if j['type']=='revolute' and ('left_joint' in j['name'] or 'right_joint' in j['name'])]
    assert len(arm_joints)==12
    assert all(float(j['limit']['lower'])==0 and float(j['limit']['upper'])==0 for j in arm_joints)
    static.append({'file':inv['file'],'sha256':inv['sha256'],
      'frame':'base_link; raw URDF reference/zero configuration; no hardware calibration assumed',
      'reference_positions_m':positions,'origin_chains':chains,
      'arm_base_origin_distance_3d_m':float(np.linalg.norm(l-r)),
      'arm_base_origin_distance_xy_m':float(np.linalg.norm((l-r)[:2])),
      'revolute_joints_per_side':6,'all_arm_joint_limits_lower_upper_zero':True,
      'vertical_move_limits':next(j['limit'] for j in inv['joints'] if j['name']=='vertical_move'),
      'wheel_joints':[{k:j[k] for k in ['name','type','origin']} for j in inv['joints'] if 'wheel' in j['name']]})

api=ROS/'gazebo_world_sources/AlohaMini/examples/pi0.5_openpi/evaluate_bi.py'
api_lines=api.read_text(encoding='utf-8').splitlines()
api_hits=[{'line':i+1,'text':s.strip()} for i,s in enumerate(api_lines) if any(x in s for x in ['arm_left_shoulder','arm_left_elbow','arm_left_wrist','arm_left_gripper'])]

pdf=THESIS/'小论文/论文稿/paper_tac/main.pdf'
pdftext=(OUT/'main_historical_pdf.txt').read_text(encoding='utf-8')
pages=pdftext.split('\f')
pdf_records=[]
for i,p in enumerate(pages):
    matches=[s.strip() for s in p.splitlines() if any(x in s for x in ['36/36','Total route length','Planning time','Arm mounting height','Forward arm offset','Dual-arm spacing','Overall size','Lifting stroke','End-effector working height','Table height','Base positioning error','of 4.3 cm','rather than repeated hardware'])]
    if matches:pdf_records.append({'pdf_page':i+1,'lines':matches})

e=ROOT/'results/repair_20261027/evidence_20261027_221823'
current_rows=list(csv.DictReader((e/'main/main_raw.csv').open(encoding='utf-8-sig')))
columns=['seed','task_count','selected_docks','candidate_count','candidate_coverage_percent','geometric_coverage_percent','simulated_geometric_completion_percent','ik_feasibility_percent','actual_completion_percent','status','failure_reason','global_planner','routing_mode','threshold','grid_resolution_m','implementation_signature','task_geometry_signature']

review={
 'scope':'只读审查已有文件；未修改 MATLAB/ROS 代码，未运行新实验、IK、控制或仿真；仅计算已有 URDF 的参考安装变换，不创建模型。',
 'review_attachment':r'C:/Users/Administrator/.codex/attachments/0eeb0a0e-5f6d-4818-9c91-6aadd5ec54da/已粘贴的文本.txt',
 'urdf_static_inventory':static,
 'hardware_findings':[
  {'finding':'四份 URDF 均定义每侧 left/right_joint1 至 joint6 六个 revolute 关节，但所有关节限位、速度和力矩字段为零；升降关节亦零行程。',
   'supported':'可确认存在这些关节节点及参考装配链。','not_supported':'不可把这些零范围文件视为经过标定、可用于运动范围验证的真实 IK 模型；不能直接确认六个独立定位自由度。'},
  {'finding':'同一 AlohaMini 源目录的机器人接口按每臂 shoulder_pan、shoulder_lift、elbow_flex、wrist_flex、wrist_roll、gripper 排列，明确为五个臂关节加夹爪通道。',
   'source':str(api),'lines':api_hits,
   'supported':'审查材料所说六自由度与 MATLAB 五个关节限位的差异，可能包含是否计入夹爪的口径差异，不能仅按 URDF 名称个数下结论。',
   'unknown':'尚未得到 joint1…6 与硬件电机/API 通道的一对一标定映射；不得断言第6个URDF关节已被验证为夹爪。'},
  {'finding':'原始 Aloha URDF 是三轮模型；两份 mecanum 变体定义四个轮子，但轮子为 fixed，导航由 libgazebo_ros_planar_move.so 插件提供。',
   'supported':'可证明存在四轮外观的导航模型版本，及四轮麦克纳姆平台的历史设计意图。',
   'not_supported':'不能由固定轮外观和 planar_move 插件证明实际轮速映射、滚子方向、真实轮半径、无滑移或驱动动力学。'},
  {'finding':'URDF两臂base坐标原点三维间距约0.313859m，原始base_link下安装高度约0.75221m和0.752908m；front_arm变体只增加整机绕Z轴90度的参考方向变换，并未将安装改为MATLAB几何。',
   'supported':'这些安装变换与当前MATLAB的0.250m间距、0.2115m前偏和0.1833m固定高度不能直接对应。',
   'unknown':'尚无实物测量、升降零位、底盘参考点和工具坐标标定将多个模型统一；不据此判定它们必为不同物理机器人。'},
  {'finding':'URDF提供origin、rpy、axis运动链，不提供与现有MATLAB字段逐项对应的标准DH表。',
   'supported':'可以据明确的关节轴约定推导某份URDF自身的等价运动学描述。',
   'not_supported':'不能把该URDF数值拼入MATLAB既有d1/a2/a3/d4/d5并称为当前模型的完整DH参数；本轮不执行这种转换。'}],
 'matlab_current_model':{'source':str(ROOT/'config/robot_config.m'),'sha256':hashlib.sha256((ROOT/'config/robot_config.m').read_bytes()).hexdigest(),
   'chassis_m':[0.423,0.366,0.127],'arm_spacing_m':0.250,'arm_forward_offset_m':0.2115,'arm_base_height_m':0.1833,
   'joint_limit_entries_per_arm':5,'horizontal_reach_radius_m':0.278,'relative_height_bounds_m':[-0.142,0.334],
   'interpretation':'论文应将上述数字标为本章固定几何计算模型参数；不是由这四份URDF及实测共同证实的统一硬件参数。'},
 'historical_pdf':{'path':str(pdf),'sha256':hashlib.sha256(pdf.read_bytes()).hexdigest(),'extraction_tool':'D:/app/texlive/2026/bin/windows/pdftotext.exe -layout',
   'confirmed_claims':pdf_records,'physical_table':{'mount_height_m':0.40,'forward_offset_m':0.06,'arm_spacing_m':0.40,'overall_size_mm':[400,460,1100],'lifting_stroke_mm':600,'working_height_mm':[400,1200],'table_height_m':0.70,'base_positioning_error_cm':4.3},
   'main_claim':{'tables':16,'tasks':36,'docks':27,'completed_tasks':36,'route_length_m':63.29,'planning_time_s':28.85},
   'unknown':['PDF未提供可将主场景绑定到当前快照的种子、源码哈希和连续任务/地图签名。','Table III是实机参数，不能仅由此推出PDF主仿真的全部几何参数就是该组数值。','4.3cm仅有测得误差的文字陈述，无原始测量记录、样本数、误差维度及均值/最大值/分位数定义。','代表性实机展示与可重复定量实机实验在PDF末段已作区分。']},
 'current_main_snapshot':{'path':str(e/'main/main_raw.csv'),'rows':[{k:r[k] for k in columns} for r in current_rows],
   'comparison':'当前seed2026同样为36任务/27停靠，但阶段3稀疏图连通性验证失败；因此不能将差异简单解释为不同种子。缺少旧PDF的运行身份信息，无法定位全部差异来源。保留当前可审计失败结论，不与旧完成率并作同一版本结果。'},
 'proposal_interim':json.loads((OUT/'proposal_interim_extract.json').read_text(encoding='utf-8')),
 'mathematical_review':{
   'grid':{
    'current_code':str(ROOT/'utils/world2grid.m'),
    'conditions':'为保持表达清晰，可限定矩形网格H=N_y r_g、W=N_x r_g，并区分数值索引约定与底盘可行域。',
    'current_convention':'列 j=floor((x-x0)/r)+1对应x方向[左,右)；行 i=floor((H-(y-y0))/r)+1对应y方向(下,上]。因此y=y0产生N_y+1而y=y0+H产生1，是该半开单元约定的边界选择。',
    'reviewer_alternative':'i=N_y-floor((y-y0)/r)改为y方向[下,上)，不仅改变外边界，还改变每条水平内部网格线的单元归属。两者在非网格线内部点上相同；不能只改论文公式并声称仍是当前代码。',
    'valid_domain':'当前坐标函数不自行裁剪：x0<=x<x0+W、y0<y<=y0+H可得到有效索引；接触地图边界的实体底盘仍由footprint检测判为不可行。',
    'floating_point':'代码取整前对距最近整数不超过4*eps(max(abs(value),1))的值进行对齐。',
    'recommended_wording':'本章沿用实验实现的上边界归属约定，边界位姿不依靠索引裁剪补救；采用另一种半开单元规范时，应同步修改映射、测试及实验身份。本轮未修改实现。'},
   'robust_fixed_arm':{
    'definition':r'B_m=1[BaseFree(p_k^{(m)})], A_{ell,m}=1[rho_ell(p_k^{(m)},t_i)>=tau_r]; R_fixed=max_ell min_m(B_m A_{ell,m}); R_switch=min_m max_ell(B_m A_{ell,m}).',
    'inequality':'R_fixed <= R_switch；固定臂的存在量词在所有扰动量词之前。',
    'simple_counterexample':'两个样本分别只有左臂/右臂可达且底盘均无碰，则R_switch=1而R_fixed=0。该例仅解释数学语义，不作为仿真实验结果。',
    'current_implementation':'当前离散扰动仅重算max(left,right)几何评分，允许换臂，不检查扰动后底盘碰撞；配置q_min=1，对15个样本均通过作判定。',
    'not_implemented':'包含扰动后碰撞及固定执行臂的扩展可作为规范或待验证模型给出，不能写成已实现/已实验证实。',
    'samples_not_bound':'平移样本为原点及四轴向±0.02m，和0/±0.03rad做笛卡尔积，不覆盖二维±0.02m方框角点或连续集合。4.3cm历史误差的统计定义未知，不能把它自动当最坏界或直接改扰动范围。'},
   'score_and_height':{'lateral_gate':'在R_lat=R_max=0.278m时，|v|<=r<=R_max已蕴含|v|<=1.1R_lat，因此该侧向硬条件当前冗余。',
      'score_range':'在当前硬范围内s_l=1-|v|/R_lat也属于[0,1]；一般配置仅要求|v|<=1.1R_lat则s_l可能低至-0.1，因此应限当前配置或依靠总分clip保证最终[0,1]。',
      'height_margin_m':0.0173,'height_component':1-abs((0.50-0.1833)-((-0.142+0.334)/2))/((0.334+0.142)/2),
      'height_component_weighted':0.2*(1-abs((0.50-0.1833)-((-0.142+0.334)/2))/((0.334+0.142)/2)),
      'interpretation':'17.3mm和高度评分值只来自固定几何模型；不是IK或物理安全裕量。'}},
 'ready_to_use_revision_paragraphs':[
 '本章采用固定几何计算模型开展候选停靠与任务覆盖研究，参数统一取自本次实验配置。现有ROS模型提供了双臂参考装配链，但其关节限位与升降行程字段均设为零，且安装间距、参考高度与本章配置尚未建立一致的标定映射。因此，本文保留通用关节运动学表达，不将这些URDF直接作为本章几何模型已完成逆运动学验证的依据。',
 '关节自由度按末端定位关节与夹爪开合分别统计。本章计算模型保存五个臂关节的限位；AlohaMini接口同样列出五个臂关节通道及一个夹爪通道。URDF中每侧六个命名转动关节与上述通道的对应关系仍待核实，故不能仅按关节节点数量将末端定位模型改写为六自由度。',
 '历史小论文记载了另一组实机安装参数及4.3cm底盘定位误差，但未给出该误差的样本规模与统计定义。本章±0.02m离散平移扰动为配置敏感性设置，不能将其表述为经该实测误差覆盖验证的鲁棒范围，也不由此宣称连续误差条件下的安全保证。',
 '历史小论文报告16桌、36任务、27停靠及36/36任务完成；本次可审计快照中，2026种子同样对应36任务和27停靠，但在候选全部覆盖后因稀疏导航图不连通而终止。两份记录尚缺完整的运行身份对齐，本文不将历史完成率并入当前实验结论，保留本次几何覆盖与后续导航失败的分别记录。'
 ],
 'unresolved_inputs':['最终采用的实体机器人与URDF版本','关节名称—驱动电机—夹爪通道的标定对应','非零关节限位与升降零位/行程的实测或厂家参数','底盘参考点、两臂基座及工具坐标的统一标定','4.3cm误差的原始日志及统计口径','历史36/36实验的冻结地图/源码/参数与执行证据']
}

(OUT/'revision_evidence.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
lines=['硬件与历史结果只读核查','本轮未修改代码、未补实验、未生成URDF、未运行IK。','']
for i,x in enumerate(review['hardware_findings'],1):
    lines.append(f'{i}. {x["finding"]}')
    for key in ['supported','not_supported','unknown']:
        if key in x:lines.append(f'   {key}: {x[key]}')
lines+=['','可直接采用的修订说明：',*review['ready_to_use_revision_paragraphs'],'','数学核查：']
for key in ['grid','robust_fixed_arm','score_and_height']:
    lines.append(key+': '+json.dumps(review['mathematical_review'][key],ensure_ascii=False))
lines+=['','URDF参考安装数据：',json.dumps(static,ensure_ascii=False,indent=2),'','未知项：',*review['unresolved_inputs'],
 '', '完整来源、SHA-256、PDF页码及当前CSV逐行信息见同目录revision_evidence.json。']
(OUT/'revision_evidence.txt').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps({'json':str(OUT/'revision_evidence.json'),'text':str(OUT/'revision_evidence.txt'),
 'urdf_mounts':[{'name':Path(x['file']).name,'positions':x['reference_positions_m'],'spacing':x['arm_base_origin_distance_3d_m']} for x in static]},ensure_ascii=False,indent=2))
