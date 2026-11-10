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
SOURCE = Path(r'D:\file_used\西柚\实验室_任务\硕士论文相关\大论文\初稿.docx')
FORMULAS = SOURCE.parent/'chatgpt生成文件'/'第二章_MathType_纯公式代码.txt'
PANDOC = Path(r'D:\tools\anocaonda\Scripts\pandoc.exe')
OUT = ROOT/'第二章与第三章初稿_20261029.docx'

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
    a=json.loads((ROOT/'chapter2.json').read_text(encoding='utf-8-sig'))
    b=json.loads((ROOT/'chapter3.json').read_text(encoding='utf-8-sig'))
    e=json.loads((ROOT/'chapter3_experiments.json').read_text(encoding='utf-8-sig'))
    start=next(i for i,x in enumerate(b) if x['type']=='heading' and x['text'].startswith('3.7'))
    end=next(i for i,x in enumerate(b) if x['type']=='heading' and x['text'].startswith('3.8'))
    b=b[:start]+e+b[end:]
    blocks=a+b
    # Unify region membership and symbol meanings across independently drafted chapters.
    for x in blocks:
        if x.get('label')=='2-36':
            x['latex']=r'\begin{aligned}\mathcal C(p_k)&=\{t_i\in\mathcal T_{j(k)}:\rho(p_k,t_i)\ge\rho_{\min}\},\\C_{ik}&=\begin{cases}1,&t_i\in\mathcal C(p_k),\\0,&\text{otherwise}.\end{cases}\end{aligned}'
        if x.get('text','').startswith('令二值变量'):
            x['text']='其中，$j(k)$ 表示候选 $p_k$ 所属的工作区域，跨区域的覆盖矩阵元素置零；各区域内的覆盖集合合并后形成全局覆盖关系。'+x['text']
        if x.get('text','').startswith('分母始终取全部任务'):
            x['text']=r'式中，$\mathcal C(p)$ 即候选所属区域内的覆盖集合 $\mathcal C_j(p)$。'+x['text']
        if x.get('text','').startswith('以 $d_s=0.25'):
            x['text']=x['text'].replace('d_s=',r'd_{\mathrm{ref}}=')
        if x.get('label')=='3-27':
            x['latex']=x['latex'].replace('/d_s',r'/d_{\mathrm{ref}}')
        if x.get('label')=='2-44':
            x['latex']=x['latex'].replace(r'\xrightarrow{P_1}',r'\overset{P_1}{\longrightarrow}').replace(r'\xrightarrow{P_2}',r'\overset{P_2}{\longrightarrow}').replace(r'\xrightarrow{P_3}',r'\overset{P_3}{\longrightarrow}')
        if x.get('text','').startswith('以 $\\tau_r=0.30'):
            x['text']=x['text'].replace(r'\tau_r=0.30',r'\tau_r=\rho_{\min}=0.30')
        if '真实轮廓间隙' in x.get('text',''):
            x['text']=x['text'].replace('真实轮廓间隙','相对于占据地图的轮廓间隙')
        if '名义可达性评价需要处理' in x.get('text',''):
            x['text']=x['text'].replace('名义可达性评价需要处理','名义可达性评价至多处理')
        if x['type']=='algorithm':
            x['steps']=[v.replace('若启用预筛，则先跳过两臂均违反硬条件的组合。','名义模式启用预筛时，先跳过两臂均违反硬条件的组合；鲁棒扩展保持该预筛关闭。') for v in x['steps']]
    # Split long displays at mathematical boundaries, preserving editable equations.
    wide={
      '2-1':r'\begin{aligned}\boldsymbol\xi&=\begin{bmatrix}x_b&y_b&\theta_b&h&\boldsymbol q_L^T&\boldsymbol q_R^T\end{bmatrix}^{T},\\n_\xi&=4+n_L+n_R.\end{aligned}',
      '2-6':r'\begin{aligned}{}^{H}T_{A_\ell}&=\begin{bmatrix}\boldsymbol R_{A_\ell}&\boldsymbol d_\ell\\\boldsymbol0^{T}&1\end{bmatrix},\\\boldsymbol d_L&=\begin{bmatrix}d_x\\d_a/2\\0\end{bmatrix},\qquad\boldsymbol d_R=\begin{bmatrix}d_x\\-d_a/2\\0\end{bmatrix}.\end{aligned}',
      '2-9':r'\begin{aligned}\begin{bmatrix}\boldsymbol p_i^{A_\ell}\\1\end{bmatrix}&=\left({}^{W}T_{A_\ell}\right)^{-1}\begin{bmatrix}\boldsymbol p_i^W\\1\end{bmatrix},\\\boldsymbol p_i^{A_\ell}&=\boldsymbol R_{WA_\ell}^{T}(\boldsymbol p_i^W-\boldsymbol p_{WA_\ell}).\end{aligned}',
      '2-11':r'\begin{aligned}\boldsymbol v_B&=\begin{bmatrix}v_x^B&v_y^B&\omega_b\end{bmatrix}^{T},\\\dot{\boldsymbol b}&=\begin{bmatrix}\cos\theta_b&-\sin\theta_b&0\\\sin\theta_b&\cos\theta_b&0\\0&0&1\end{bmatrix}\boldsymbol v_B.\end{aligned}',
      '2-17':r'\begin{aligned}\gamma(0)&=\boldsymbol b_s,\qquad\gamma(1)=\boldsymbol b_g,\\\mathcal F(\gamma(s))\cap\mathcal O_{\mathrm{col}}&=\varnothing,\quad\mathcal F(\gamma(s))\subset\operatorname{int}(\mathcal D),\quad\forall s\in[0,1].\end{aligned}',
      '2-32':r'\begin{aligned}\mathcal O(t)&=\mathcal O_s\cup\mathcal O_d(t),\\\mathcal C_{\mathrm{free}}(t)&=\{p\in SE(2):\mathcal F(p)\cap\mathcal O_{\mathrm{col}}(t)=\varnothing,\\&\hspace{5em}\mathcal F(p)\subset\operatorname{int}(\mathcal D)\}.\end{aligned}',
      '2-35':r'\begin{aligned}\rho(p,t_i)&=\max\{\rho_L(p,t_i),\rho_R(p,t_i)\},\\\mathcal B_i&=\{p\in\mathcal P_{j_i}\cap\Omega_{j_i}:\rho(p,t_i)\ge\rho_{\min}\}.\end{aligned}',
      '2-39':r'\begin{aligned}\min_{\boldsymbol X,\boldsymbol y,\sigma,\Gamma}\quad&\alpha\sum_{k=1}^{K}y_k+\beta L_{\mathrm{nav}}+\gamma J_{\mathrm{op}},\\&\gamma_e([0,1])\subseteq\mathcal C_{\mathrm{free}},\quad\forall\gamma_e\in\Gamma.\end{aligned}',
      '2-43':r'\begin{aligned}\mathcal T_{\mathrm{rem}}(t+1)&\subseteq\bigcup_{p\in\mathcal S_{t+1}}\mathcal C_{t+1}(p),\\\gamma_e([0,1])&\subseteq\mathcal C_{\mathrm{free}}(t+1),\quad\forall\gamma_e\in\Gamma_{t+1}.\end{aligned}',
      '3-3':r'\begin{aligned}\boldsymbol u_1&=\begin{bmatrix}s\\W_T/2+d\end{bmatrix},&\boldsymbol u_2&=\begin{bmatrix}s\\-W_T/2-d\end{bmatrix},\\\boldsymbol u_3&=\begin{bmatrix}-L_T/2-d\\s\end{bmatrix},&\boldsymbol u_4&=\begin{bmatrix}L_T/2+d\\s\end{bmatrix}.\end{aligned}',
      '3-30':r'\begin{aligned}J_{\mathrm{dock}}(p_k)&=w_rJ_r+w_cJ_c+w_\theta J_\theta\\&\quad+w_sJ_s+w_aJ_a+w_mJ_m,\qquad\sum_\nu w_\nu=1.\end{aligned}'
    }
    for x in blocks:
        if x.get('label') in wide: x['latex']=wide[x['label']]
    # Sequential numbering after merging the experimental section.
    for x in blocks:
        if x.get('label') in ['3-50','3-51']:
            x['label']={'3-50':'3-31','3-51':'3-32'}[x['label']]
        for key in ['text','caption']:
            if key in x:
                x[key]=re.sub(r'表3-([567])',lambda m:'表3-'+str(int(m.group(1))-3),x[key])
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
        weights={1:[0.34,0.33,0.33],2:[0.28,0.34,0.38],3:[0.18,0.45,0.37],4:[0.13,0.29,0.27,0.31],5:[0.18,0.27,0.27,0.28],6:[0.40,0.30,0.30]}.get(ti,weights)
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
    d.core_properties.title='移动双臂机器人基础理论与操作可达性建模 第二章与第三章初稿'
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
    if proc.returncode: raise RuntimeError(proc.stderr)
    d=fix_doc()
    stats={'blocks':len(blocks),'equations':len(labels),'tables':len(d.tables),'chinese_chars':len(re.findall(r'[\u4e00-\u9fff]',json.dumps(blocks,ensure_ascii=False))),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'formulas_sha256':hashlib.sha256(FORMULAS.read_bytes()).hexdigest(),'output':str(OUT)}
    (ROOT/'build_stats.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(stats,ensure_ascii=False,indent=2))
