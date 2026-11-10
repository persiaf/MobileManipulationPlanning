from pathlib import Path
import json, re, subprocess, hashlib, copy
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_TAB_ALIGNMENT
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.enum.style import WD_STYLE_TYPE
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent/'thesis_ch2_ch3_20261029'/'第二章与第三章初稿_20261029.docx'
FORMULAS = Path(r'D:\file_used\西柚\实验室_任务\硕士论文相关\大论文\chatgpt生成文件\第二章_MathType_纯公式代码.txt')
PANDOC = Path(r'D:\tools\anocaonda\Scripts\pandoc.exe')
OUT = ROOT/'第二章与第三章修订稿_20261029.docx'

def font_style(style, east='宋体', size=12, bold=False):
    style.font.name='Times New Roman'
    style.font.size=Pt(size)
    style.font.bold=bold
    style.font.color.rgb=RGBColor(0,0,0)
    rp=style.element.get_or_add_rPr()
    f=rp.find(qn('w:rFonts'))
    if f is None:
        f=OxmlElement('w:rFonts'); rp.insert(0,f)
    for a in list(f.attrib):
        if 'Theme' in a or 'theme' in a: del f.attrib[a]
    f.set(qn('w:eastAsia'),east)
    f.set(qn('w:ascii'),'Times New Roman'); f.set(qn('w:hAnsi'),'Times New Roman')
    for c in rp.findall(qn('w:color')):
        for a in list(c.attrib):
            if a!=qn('w:val'): del c.attrib[a]

def clear_char_indents(ppr):
    if ppr is None: return
    ind=ppr.find(qn('w:ind'))
    if ind is not None:
        for k in ['firstLineChars','leftChars','rightChars','hangingChars']:
            ind.attrib.pop(qn('w:'+k),None)

def set_zero(p):
    f=p.paragraph_format
    f.first_line_indent=Pt(0); f.left_indent=Pt(0); f.right_indent=Pt(0)
    clear_char_indents(p._p.pPr)

def make_reference():
    d=Document(SOURCE)
    # Retain the reference's page geometry and style definitions in the new file.
    for child in list(d._element.body):
        if child.tag!=qn('w:sectPr'): d._element.body.remove(child)
    for s in d.sections:
        s.different_first_page_header_footer=False
        for hf in (s.header,s.footer,s.first_page_header,s.first_page_footer,s.even_page_header,s.even_page_footer):
            for p in hf.paragraphs: p.clear()
        p=s.footer.paragraphs[0]; set_zero(p)
        p.alignment=WD_ALIGN_PARAGRAPH.CENTER
        r=p.add_run(); r.font.size=Pt(10.5)
        fld=OxmlElement('w:fldSimple'); fld.set(qn('w:instr'),'PAGE')
        rr=OxmlElement('w:r'); tt=OxmlElement('w:t'); tt.text='1'; rr.append(tt); fld.append(rr); p._p.append(fld)
    for st in d.styles:
        if st.type==1: clear_char_indents(st.element.pPr)
    norm=d.styles['Normal']; font_style(norm)
    pf=norm.paragraph_format
    pf.first_line_indent=Pt(24); pf.line_spacing=1.25
    pf.space_after=Pt(0); pf.space_before=Pt(0); pf.widow_control=True
    for name in ['Body Text','First Paragraph']:
        if name in d.styles:
            st=d.styles[name]; st.base_style=norm; font_style(st)
            st.paragraph_format.first_line_indent=Pt(24)
            st.paragraph_format.line_spacing=1.25
            st.paragraph_format.space_after=Pt(0)
    for level,size in [(1,16),(2,14),(3,12)]:
        st=d.styles[f'Heading {level}']; font_style(st,'黑体',size)
        pf=st.paragraph_format
        pf.first_line_indent=Pt(0); pf.space_before=Pt(14 if level==1 else 8)
        pf.space_after=Pt(8 if level==1 else 4); pf.keep_with_next=True
        pf.keep_together=True; pf.line_spacing=1.25
        pf.alignment=WD_ALIGN_PARAGRAPH.CENTER if level==1 else WD_ALIGN_PARAGRAPH.LEFT
        pf.page_break_before=level==1
    if 'Caption' not in d.styles:
        d.styles.add_style('Caption',WD_STYLE_TYPE.PARAGRAPH)
    for name in ['Title','Caption','Table Caption','Image Caption','Compact']:
        if name in d.styles:
            font_style(d.styles[name],size=10.5 if 'Caption' in name else 12)
    d.settings.element.append(OxmlElement('w:updateFields'))
    d.settings.element[-1].set(qn('w:val'),'true')
    d.save(ROOT/'reference.docx')

def load_blocks():
    blocks=json.loads((ROOT/'chapter2_revised.json').read_text(encoding='utf-8-sig'))+json.loads((ROOT/'chapter3_revised.json').read_text(encoding='utf-8-sig'))
    pictures={'FIG_COORD':'fig2_1_coordinates.png','FIG_PROXY':'fig3_1_proxy.png','FIG_C2':'fig3_2_c2.png'}
    for b in blocks:
        if b['type']=='figure':
            b['path']=(ROOT/'figures'/pictures[b.get('path',b.get('id'))]).as_posix()
            if '图2-1' in b['caption']:
                b['caption']='图2-1 移动底盘与双臂几何评价坐标系示意图'
            elif '图3-1' in b['caption']:
                b['caption']='图3-1 固定任务高度0.50 m下的双臂择一几何代理评分分布'
            elif '图3-2' in b['caption']:
                b['caption']='图3-2 C2场景原候选覆盖与有限加密搜索结果'
        if b['type']=='eq':
            # Pandoc 2.12 accepts overset but not xrightarrow.
            b['latex']=re.sub(r'\\xrightarrow\{([^}]+)\}',lambda m:r'\overset{'+m.group(1)+r'}{\longrightarrow}',b['latex'])
    # Keep tool frame E consistent between chapters; F denotes the flange only.
    for b in blocks:
        if b.get('text','').startswith('设 $G$ 为工具作用坐标系'):
            b['text']=r'沿用第2章的工具末端坐标系 $E_\ell$，另以 $F_\ell$ 表示机械臂末端法兰坐标系，工具固定变换为 ${}^{F_\ell}T_{E_\ell}$。目标工具位置为 $\boldsymbol p_i^g$，期望接近方向为单位向量 $\boldsymbol n_i$，允许绕接近轴的转角自由。工具位姿由'
        if b.get('label')=='3-A15':
            b['latex']=r'{}^WT_{E_\ell}(p,h,\boldsymbol q_\ell)={}^WT_{A_\ell}(p,h)\,{}^{A_\ell}T_{F_\ell}(\boldsymbol q_\ell)\,{}^{F_\ell}T_{E_\ell}'
        if b.get('label')=='3-A16':
            b['latex']=b['latex'].replace('G_\\ell','E_\\ell')
        if b.get('text','').startswith('确定。目标位置必须'):
            b['text']=r'确定。其中 ${}^{A_\ell}T_{F_\ell}$ 仅包含臂基座至法兰的关节链，工具固定变换只计入一次。目标位置必须由物体参考点、抓取位置和工具偏移一致定义。以下沿用第2章的安全构型集合 $\mathcal Q_\ell^{\mathrm{safe}}$，其底盘位姿、升降高度、另一臂停放构型及三维障碍环境由每个验证案例指定，并在公式中省略这些固定参数。可定义目标构型存在性的理想判据'
    wide={
      '2-A6':r'\begin{aligned}&\mathcal Q_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3)\\&\quad=\{\boldsymbol q_\ell\in\mathcal Q_\ell:\\\ &\qquad\operatorname{CF}_3(\boldsymbol b,h,\boldsymbol q_\ell,\boldsymbol q_{\bar\ell}^{\mathrm{park}};\mathcal O_3)=1\}.\end{aligned}',
      '2-A7':r'\begin{aligned}&\mathcal W_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3)\\&\quad=\{\boldsymbol p_{E_\ell}^{W}(\boldsymbol b,h,\boldsymbol q_\ell):\\&\qquad\boldsymbol q_\ell\in\mathcal Q_\ell^{\mathrm{safe}}(\boldsymbol b,h;\boldsymbol q_{\bar\ell}^{\mathrm{park}},\mathcal O_3)\}.\end{aligned}',
      '3-A9':r'\begin{aligned}R_{ik}^{\mathrm{fix}}&=\max_{\ell\in\{L,R\}}\min_{\delta\in\Delta}(A_{\ell\delta}B_\delta),\\R_{ik}^{\mathrm{switch}}&=\min_{\delta\in\Delta}\max_{\ell\in\{L,R\}}(A_{\ell\delta}B_\delta).\end{aligned}',
      '3-A10':r'\begin{aligned}R_{ik}^{\mathrm{fix}}&\le R_{ik}^{\mathrm{switch}}\le R_{ik}^{\mathrm{reach}},\\R_{ik}^{\mathrm{switch}}&=\left(\min_{\delta\in\Delta}B_\delta\right)R_{ik}^{\mathrm{reach}}.\end{aligned}'
    }
    for b in blocks:
        if b.get('label') in wide: b['latex']=wide[b['label']]
    figure_notes={
      'FIG_COORD':'图中左、右臂坐标系为几何评价原点，虚线表示任务点相对各原点的位置。侧视图采用固定底盘参考高度和0.50 m任务高度。该图用于说明坐标关系，各面板未按统一比例绘制，示意线不代表实际连杆。',
      'FIG_PROXY':'图中固定底盘位姿为(0, 0, 0)、任务高度为0.50 m，评分取左右臂评分的较大值，轮廓线对应0.30阈值。白色区域表示当前代理函数得分为零，不能据此认定物理不可达。底盘虚线轮廓仅供位置参考，图中未执行环境或连杆碰撞检查；颜色也不表示操作成功概率。',
      'FIG_C2':'三个种子的任务总数分别为65、79和76。柱高为任务比例，数字为任务数量；原候选覆盖、额外搜索见证和有限搜索未找到见证分别标出。新增见证仅通过本章几何判据，尚不能解释为实际抓取成功。'
    }
    expanded=[]
    for b in blocks:
        expanded.append(b)
        if b['type']=='figure':
            key=b.get('id') or ('FIG_COORD' if 'fig2_1' in b['path'] else None)
            if key in figure_notes: expanded.append({'type':'p','text':figure_notes[key]})
    blocks=expanded
    emap={}; tmap={}; ec={2:0,3:0}; tc={2:0,3:0}
    for b in blocks:
        if b['type']=='eq':
            old=b['label']; c=int(old[0]); ec[c]+=1; emap[old]=f'{c}-{ec[c]}'
        if b['type']=='table':
            old=re.match(r'表([23]-(?:A\d+|\d+))',b['caption']).group(1)
            c=int(old[0]); tc[c]+=1; tmap[old]=f'{c}-{tc[c]}'
    def renumber(v):
        if isinstance(v,str):
            v=re.sub(r'(?<=[（(])([23]-(?:A\d+|\d+))(?=[）)])',lambda m:emap.get(m.group(1),m.group(1)),v)
            return re.sub(r'表([23]-(?:A\d+|\d+))',lambda m:'表'+tmap.get(m.group(1),m.group(1)),v)
        if isinstance(v,list): return [renumber(x) for x in v]
        if isinstance(v,dict): return {k:renumber(x) for k,x in v.items()}
        return v
    blocks=renumber(blocks)
    for b in blocks:
        if b['type']=='eq': b['label']=emap[b['label']]
    refs=[
      '[1] LYNCH K M, PARK F C. Modern Robotics: Mechanics, Planning, and Control[M]. Cambridge: Cambridge University Press, 2017. [出版社 DOI](https://doi.org/10.1017/9781316661239).',
      '[2] DENAVIT J, HARTENBERG R S. A Kinematic Notation for Lower-Pair Mechanisms Based on Matrices[J]. Journal of Applied Mechanics, 1955, 22(2): 215–221. [DOI](https://doi.org/10.1115/1.4011045).',
      '[3] ZACHARIAS F, BORST C, HIRZINGER G. Capturing Robot Workspace Structure: Representing Robot Capabilities[C]//2007 IEEE/RSJ International Conference on Intelligent Robots and Systems. 2007: 3229–3236. [作者机构记录](https://elib.dlr.de/51226/).',
      '[4] VAHRENKAMP N, ASFOUR T, DILLMANN R. Robot Placement Based on Reachability Inversion[C]//2013 IEEE International Conference on Robotics and Automation. 2013: 1970–1975. [DOI](https://doi.org/10.1109/ICRA.2013.6630839).',
      '[5] CHVÁTAL V. A Greedy Heuristic for the Set-Covering Problem[J]. Mathematics of Operations Research, 1979, 4(3): 233–235. [DOI](https://doi.org/10.1287/moor.4.3.233).'
    ]
    # References follow order of first appearance in these two chapters.
    citation_order=[]
    for b in blocks:
        for match in re.finditer(r'\[([1-5])\]',b.get('text','')):
            old=match.group(1)
            if old not in citation_order: citation_order.append(old)
    cmap={old:str(i+1) for i,old in enumerate(citation_order)}
    for b in blocks:
        if 'text' in b: b['text']=re.sub(r'\[([1-5])\]',lambda m:'['+cmap[m.group(1)]+']',b['text'])
    sorted_refs=[re.sub(r'^\[\d\]', '['+cmap[old]+']',refs[int(old)-1]) for old in citation_order]
    blocks += [{'type':'heading','level':1,'text':'本稿参考文献'}]+[{'type':'p','text':x} for x in sorted_refs]
    blocks += [
      {'type':'heading','level':1,'text':'修订说明'},
      {'type':'p','text':'本节为初稿修订记录，不计入论文正文。本次修订以原第二、三章及其对应的MATLAB仿真模型为基础，完成可由现有材料支持的数学与文字修正；原始实验数值保持不变，没有新增IK、实机或算法对比实验。'},
      {'type':'p','text':'主要修订包括：区分位姿参数空间与刚体变换群，补全栅格边界约定和安全构型条件，明确目标函数归一化；推导停靠距离的几何依据，分析评分项的取值范围、垂向裕量及代价相关性；区分固定执行臂与可切换执行臂的离散鲁棒条件。新增三幅图分别表达坐标关系、现有几何代理函数和已有C2审计结果。'},
      {'type':'p','text':'IK验证、离线可达空间及逆可达补采样在正文中列为扩展验证方案。数值IK失败不能直接用作物理不可达标签；新的覆盖率、误判率或成功率须在模型核定并实际验证后报告。现有几何一致性试验仅证明实现之间的一致，不能替代运动学真值检验。'},
      {'type':'p','text':'平台资料仍需对齐。当前MATLAB模型列出五个臂关节限位。所查Aloha URDF虽然含六个命名转动关节，但关节限位和驱动限制均为零，且接口又区分五个臂关节与夹爪；仅凭命名数量不能确定实机自由度。这些URDF的安装变换与本章模型也未建立一致性。故本稿未编造完整实机DH表，也未把未经校准的URDF作为IK标签来源。'},
      {'type':'p','text':'历史小论文的实机参数与当前模型分别记录了不同的臂间距、安装偏移、安装高度和桌面高度，尚不能证明二者对应同一校准平台。其4.3 cm定位误差未给出足以确定最坏误差界的统计口径，因此不能直接据此设定鲁棒扰动的确定边界。应补充平台编号、工具变换、有效关节范围以及定位误差原始记录。'},
      {'type':'p','text':'历史小论文中的36/36完成记录与当前快照的路线连接失败均予保留，但不合并为同一组性能证据：两者缺少可逐项对应的源码、地图和验证口径。即使任务数与停靠数相同，也不足以证明状态相同。本文实验分析采用可追溯的当前原始结果。'},
      {'type':'p','text':'定稿前尚需补齐的材料为：实际平台的完整运动链与工具坐标定义、与实物一致的有效URDF或DH表、关节及轮系标定数据、几何代理与IK的双侧抽样对照，以及补采样与联合扰动碰撞验证结果。参考文献编号为本稿临时编号，合并整篇学位论文时应统一排序。'}
    ]
    (ROOT/'numbering_map.json').write_text(json.dumps({'equations':emap,'tables':tmap},ensure_ascii=False,indent=2),encoding='utf-8')
    (ROOT/'final_content.json').write_text(json.dumps(blocks,ensure_ascii=False,indent=2),encoding='utf-8')
    return blocks

def to_md(blocks):
    lines=[]
    for i,b in enumerate(blocks):
        t=b['type']
        if t=='heading': lines.append('#'*b['level']+' '+b['text'])
        elif t=='p': lines.append(b['text'])
        elif t=='eq':
            lines.extend(['$$\n'+b['latex']+'\n$$', 'EQLABEL_'+b['label'].replace('-','_')])
        elif t=='table':
            lines.append('TABLECAPTION '+b['caption'])
            heads=[str(x) for x in b['headers']]
            lines.append('| '+' | '.join(heads)+' |\n| '+' | '.join(['---']*len(heads))+' |\n'+'\n'.join('| '+' | '.join(str(x).replace('|',r'\|') for x in row)+' |' for row in b['rows']))
        elif t=='algorithm':
            lines.append('ALGORITHMTITLE '+b['title'])
            lines.extend('ALGORITHMSTEP '+str(j+1)+'．'+v for j,v in enumerate(b['steps']))
        elif t=='figure':
            lines.append('![]('+b['path']+'){width=13.5cm}')
            lines.append('FIGCAPTION '+b['caption'])
        else: raise ValueError(t)
    return '\n\n'.join(lines)+'\n'

def fix_doc():
    d=Document(ROOT/'pandoc.docx')
    width=d.sections[0].page_width-d.sections[0].left_margin-d.sections[0].right_margin
    paragraphs=list(d.paragraphs)
    for i,p in enumerate(paragraphs):
        text=p.text
        if text.startswith('EQLABEL_'):
            label=text[len('EQLABEL_'):].replace('_','-')
            eq=paragraphs[i-1]
            maths=eq._p.xpath('./m:oMathPara/m:oMath | ./m:oMath')
            if not maths: raise RuntimeError('Equation absent: '+label)
            eq.clear(); set_zero(eq)
            eq.paragraph_format.line_spacing=1.0
            eq.paragraph_format.space_before=Pt(5); eq.paragraph_format.space_after=Pt(5)
            eq.paragraph_format.keep_together=True; eq.paragraph_format.keep_with_next=False
            eq.paragraph_format.tab_stops.add_tab_stop(int(width/2), WD_TAB_ALIGNMENT.CENTER)
            eq.paragraph_format.tab_stops.add_tab_stop(width, WD_TAB_ALIGNMENT.RIGHT)
            eq.add_run('\t')
            for m in maths: eq._p.append(m)
            eq.add_run('\t（'+label+'）').font.size=Pt(10.5)
            p._p.getparent().remove(p._p)
        elif text.startswith(('TABLECAPTION ','FIGCAPTION ','ALGORITHMTITLE ')):
            prefix=text.split(' ',1)[0]+' '
            # Prefix is a plain first run; remove it without destroying inline equations.
            for r in p.runs:
                if prefix in r.text: r.text=r.text.replace(prefix,'',1); break
            p.style=d.styles['Caption']; set_zero(p)
            p.alignment=WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.space_before=Pt(8); p.paragraph_format.space_after=Pt(5)
            p.paragraph_format.keep_with_next=prefix!='FIGCAPTION '
        elif text.startswith('ALGORITHMSTEP '):
            for r in p.runs:
                if 'ALGORITHMSTEP ' in r.text: r.text=r.text.replace('ALGORITHMSTEP ','',1); break
            set_zero(p); p.paragraph_format.left_indent=Pt(12)
            p.paragraph_format.space_after=Pt(3)
        elif re.match(r'^\[[1-5]\] ',text):
            set_zero(p); p.paragraph_format.left_indent=Pt(18); p.paragraph_format.first_line_indent=Pt(-18)
            p.paragraph_format.space_after=Pt(7); p.paragraph_format.line_spacing=1.1
            for r in p.runs: r.font.size=Pt(10.5)
        elif p.style.name.startswith('Heading'):
            set_zero(p)
        else:
            if p._p.xpath('.//w:drawing'):
                set_zero(p); p.alignment=WD_ALIGN_PARAGRAPH.CENTER
                p.paragraph_format.keep_with_next=True
            elif p.text.strip():
                p.style=d.styles['Normal']; clear_char_indents(p._p.pPr)
                p.paragraph_format.first_line_indent=Pt(24)
                p.paragraph_format.space_before=Pt(0); p.paragraph_format.space_after=Pt(0)
                # Keep short introductory sentences with their displayed definition.
                if i+1<len(paragraphs) and paragraphs[i+1]._p.xpath('./m:oMathPara | ./m:oMath') and len(p.text)<80:
                    p.paragraph_format.keep_with_next=True
        # Native equations use a consistently explicit math font and size.
        for mr in p._p.xpath('.//m:r'):
            wr=mr.find(qn('w:rPr'))
            if wr is None: wr=OxmlElement('w:rPr'); mr.insert(0,wr)
            for tag in ['w:rFonts','w:sz','w:szCs']:
                old=wr.find(qn(tag))
                if old is not None: wr.remove(old)
            f=OxmlElement('w:rFonts'); f.set(qn('w:ascii'),'Cambria Math'); f.set(qn('w:hAnsi'),'Cambria Math'); wr.append(f)
            sz=OxmlElement('w:sz'); sz.set(qn('w:val'),'22'); wr.append(sz)
    for ti,table in enumerate(d.tables):
        table.alignment=WD_TABLE_ALIGNMENT.CENTER; table.autofit=False
        n=len(table.columns)
        if not n:
            n=len(table._tbl.tr_lst[0].tc_lst)
            for ci in range(n):
                col=OxmlElement('w:gridCol'); col.set(qn('w:w'),str(round(width/635/n))); table._tbl.tblGrid.append(col)
        # Allow explanatory columns to carry more text than short numeric ones.
        weights={2:[0.40,0.60],3:[0.32,0.25,0.43],4:[0.30,0.25,0.23,0.22],5:[0.25,0.17,0.18,0.20,0.20],6:[0.20,0.16,0.16,0.16,0.16,0.16]}.get(n,[1/n]*n)
        headers='|'.join(''.join(tc.itertext()) for tc in table._tbl.tr_lst[0].tc_lst)
        if '随机种子' in headers and n==4: weights=[.18,.27,.27,.28]
        elif '场景' in headers and n==4: weights=[.13,.29,.27,.31]
        elif '信息类别' in headers and n==3: weights=[.18,.45,.37]
        elif '变更设置' in headers and n==3: weights=[.40,.30,.30]
        elif '关节' in headers and n==3: weights=[.34,.33,.33]
        elif '问题' in headers and n==3: weights=[.28,.34,.38]
        pr=table._tbl.tblPr
        tw=pr.find(qn('w:tblW')); tw.set(qn('w:type'),'dxa'); tw.set(qn('w:w'),str(round(width/635)))
        borders=pr.find(qn('w:tblBorders'))
        if borders is None: borders=OxmlElement('w:tblBorders'); pr.append(borders)
        for side in ['top','left','bottom','right','insideH','insideV']:
            el=OxmlElement('w:'+side); el.set(qn('w:val'),'single'); el.set(qn('w:sz'),'4'); el.set(qn('w:color'),'D9D9D9'); borders.append(el)
        mar=OxmlElement('w:tblCellMar')
        for side,num in [('top',70),('bottom',70),('left',90),('right',90)]:
            el=OxmlElement('w:'+side); el.set(qn('w:w'),str(num)); el.set(qn('w:type'),'dxa'); mar.append(el)
        pr.append(mar)
        for ci,c in enumerate(table.columns): c.width=int(width*weights[ci])
        for ri,row in enumerate(table.rows):
            trpr=row._tr.get_or_add_trPr()
            cant=OxmlElement('w:cantSplit'); trpr.append(cant)
            if ri==0:
                rep=OxmlElement('w:tblHeader'); trpr.append(rep)
            for ci,cell in enumerate(row.cells):
                cell.width=int(width*weights[ci]); cell.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
                tcpr=cell._tc.get_or_add_tcPr()
                if ri==0:
                    sh=OxmlElement('w:shd'); sh.set(qn('w:fill'),'EDEDED'); tcpr.append(sh)
                for p in cell.paragraphs:
                    set_zero(p); p.paragraph_format.line_spacing=1.1
                    p.paragraph_format.space_before=Pt(0); p.paragraph_format.space_after=Pt(0)
                    p.paragraph_format.keep_with_next=ri==0
                    p.alignment=WD_ALIGN_PARAGRAPH.CENTER if ri==0 or ci>0 else WD_ALIGN_PARAGRAPH.LEFT
                    for r in p.runs:
                        r.font.size=Pt(10.5); r.font.bold=ri==0
                        r.font.color.rgb=RGBColor(0,0,0)
    d.core_properties.title='移动双臂机器人系统建模与几何可达停靠规划 第二章与第三章修订稿'
    d.core_properties.subject='固定升降高度下的移动操作建模与候选停靠评价'
    d.core_properties.author=''
    d.core_properties.comments=''
    d.save(OUT)
    return d

if __name__=='__main__':
    blocks=load_blocks()
    labels=[b['label'] for b in blocks if b['type']=='eq']
    assert len(set(labels))==len(labels), 'Duplicate equation labels'
    make_reference()
    (ROOT/'assembled.md').write_text(to_md(blocks),encoding='utf-8')
    proc=subprocess.run([str(PANDOC),str(ROOT/'assembled.md'),'-f','markdown+tex_math_dollars','-t','docx','--reference-doc='+str(ROOT/'reference.docx'),'-o',str(ROOT/'pandoc.docx')],capture_output=True,text=True,encoding='utf-8')
    (ROOT/'pandoc.log').write_text(proc.stdout+'\n'+proc.stderr,encoding='utf-8')
    if proc.returncode or '[WARNING]' in proc.stderr: raise RuntimeError(proc.stderr)
    d=fix_doc()
    stats={'blocks':len(blocks),'equations':len(labels),'tables':len(d.tables),'chinese_chars':len(re.findall(r'[\u4e00-\u9fff]',json.dumps(blocks,ensure_ascii=False))),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'formulas_sha256':hashlib.sha256(FORMULAS.read_bytes()).hexdigest(),'output':str(OUT)}
    (ROOT/'build_stats.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(stats,ensure_ascii=False,indent=2))
