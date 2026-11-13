"""Render all predeclared comparisons and record their actual data sources."""
import hashlib
import json
from common import ROOT,SOURCE,OUT,FIG
from plot_attribution import main as gains
from plot_decomposition import main as decomposition
from plot_routes import main as routes
from plot_ablations import main as ablations

def main():
    for draw in (gains,decomposition,routes,ablations):draw()
    record_manifest()

def record_manifest():
    sources=[OUT/'paired_attribution.csv',OUT/'summary.csv']
    for base in (SOURCE,OUT):
        sources += [base/'cases/L0_D2'/name for name in ('summary.csv','navigation_paths.csv','docking_poses.csv')]
    sources += [SOURCE/'cases/L0_D2'/name for name in ('metadata.json','tasks.csv')]
    sources += sorted((ROOT/'visualization/joint_attribution').glob('*.py'))
    sources += [ROOT/'visualization/main_joint_task_navigation/main_common.py',ROOT/'visualization/joint_task_navigation/common.py']
    outputs=sorted(p for p in FIG.iterdir() if p.suffix in ('.png','.pdf','.md'))
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest={'source_sha256':{str(p.relative_to(ROOT)).replace('\\','/'):digest(p) for p in sources},
              'output_sha256':{p.name:digest(p) for p in outputs},'png_dpi':300,
              'representative_case':'L0_D2','style':'inherited main scene; actual exported routes',
              'selection_rule':'all 27 groups; no winner-only filtering or post-hoc N/J substitution'}
    (FIG/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    includes=[]
    for stem in ('certified_docking_gains','order_and_docking_decomposition','main_scene_attribution_routes','initialization_budget_ablations'):
        includes.append('\\begin{figure*}[t]\n\\centering\n\\includegraphics[width=0.95\\textwidth]{'+stem+'.pdf}\n'
                        '% See '+stem+'_caption.md for the evidence-scoped caption.\n\\end{figure*}\n')
    (FIG/'latex_includes.tex').write_text('\n'.join(includes),encoding='utf-8')

if __name__=='__main__':main()
