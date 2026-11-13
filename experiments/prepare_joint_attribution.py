"""Freeze existing evidence and certify original fixed docking sets (no IK)."""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'results/task_navigation_robustness_20261111'
OUTPUT = ROOT/'results/joint_attribution_20261113'
MODES = ('position_only', 'top_down_grasp', 'table_wipe')
sys.path.insert(0, str(ROOT/'planning_strategy/joint'))
from solve_fixed_set_atsp import solve_fixed_set_atsp


def read_rows(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    freeze = OUTPUT/'frozen_inputs.json'
    if not freeze.exists():
        files = sorted(p for p in SOURCE.rglob('*') if p.is_file())
        protected = [ROOT/'planning_strategy/joint/plan_task_navigation_joint_local.m']
        protected += sorted((ROOT/'manipulator').glob('*.m')) + sorted((ROOT/'config').glob('*vx300*.m'))
        write_json(freeze, {'source_commit': 'b4e859f', 'source_directory': str(SOURCE.relative_to(ROOT)),
                           'sha256': {str(p.relative_to(ROOT)).replace('\\','/'): hashlib.sha256(p.read_bytes()).hexdigest() for p in files+protected}})
    for folder in sorted((SOURCE/'cases').iterdir()):
        for mode in MODES:
            dest = OUTPUT/'cases'/folder.name/f'fixed_{mode}.json'
            if dest.exists():
                continue
            rows = read_rows(folder/f'seed_navigation_costs_{mode}_staged.csv')
            nodes = sorted({int(r['from_node']) for r in rows})
            index = {n:i for i,n in enumerate(nodes)}
            D = np.full((len(nodes), len(nodes)), np.inf)
            for row in rows:
                D[index[int(row['from_node'])],index[int(row['to_node'])]] = float(row['navigation_cost'])
            docks = [r for r in read_rows(folder/'docking_poses.csv') if r['mode']==mode and r['method']=='minimum_seed']
            seed_nodes = [1]+[int(r['candidate_id'])+1 for r in sorted(docks,key=lambda r:int(r['visit_rank']))]
            assert set(seed_nodes)==set(nodes)
            result = solve_fixed_set_atsp(D, [index[n] for n in seed_nodes], 60.)
            result.update(case_id=folder.name, mode=mode, nodes=nodes,
                          route_nodes=[nodes[i] for i in result['route_local_zero_based']],
                          dense_directed_edges=len(nodes)*(len(nodes)-1),
                          source_matrix=str((folder/f'seed_navigation_costs_{mode}_staged.csv').relative_to(ROOT)))
            write_json(dest,result)
            print(folder.name,mode,'fixed',round(result['upper_bound'],6),'gap',result['absolute_gap'],flush=True)


if __name__ == '__main__':
    main()
