"""Fixed-set counterfactual certificates, coverage replay audit and analysis."""
from __future__ import annotations
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'results/task_navigation_robustness_20261111'
OUT=ROOT/'results/joint_attribution_20261113'
MODES=('position_only','top_down_grasp','table_wipe')
METHODS=('fixed_optimum','legacy_strong_init','balanced_raw_init','balanced_strong_init')
sys.path.insert(0,str(ROOT/'planning_strategy/joint'))
from solve_fixed_set_atsp import solve_fixed_set_atsp, tour_cost


def rows(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))


def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path,obj):path.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


def csv_write(path,data):
    with path.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)


def certify_old_set(folder,mode):
    dest=folder/f'old_joint_fixed_{mode}.json'
    if dest.exists():return read(dest)
    graph=rows(folder/f'old_joint_graph_{mode}.csv');nodes=sorted({int(r['from_node']) for r in graph});idx={n:i for i,n in enumerate(nodes)}
    D=np.full((len(nodes),len(nodes)),np.inf)
    for r in graph:D[idx[int(r['from_node'])],idx[int(r['to_node'])]]=float(r['navigation_cost'])
    od=[r for r in rows(SOURCE/'cases'/folder.name/'docking_poses.csv') if r['mode']==mode and r['method']=='joint_local']
    seed=[1]+[int(r['candidate_id'])+1 for r in sorted(od,key=lambda r:int(r['visit_rank']))]
    cert=solve_fixed_set_atsp(D,[idx[n] for n in seed],60.)
    cert.update(case_id=folder.name,mode=mode,nodes=nodes,route_nodes=[nodes[i] for i in cert['route_local_zero_based']])
    write(dest,cert);return cert


def main():
    freeze=read(OUT/'frozen_inputs.json')
    for name,expected in freeze['sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected, f'Frozen input changed: {name}'
    paired=[];all_rows=[];assignment_count=0;move_count=0
    for folder in sorted((OUT/'cases').iterdir()):
        src=SOURCE/'cases'/folder.name
        assert read(folder/'run_state.json')['status']=='COMPLETE'
        summary=rows(folder/'summary.csv');assert len(summary)==12
        all_rows+=summary
        lookup={(r['mode'],r['method']):r for r in summary}
        old={(r['mode'],r['method']):r for r in rows(src/'summary.csv')}
        docks=rows(folder/'docking_poses.csv');assignments=rows(folder/'task_assignments.csv')
        candidates={int(r['candidate_id']):r for r in rows(src/'candidate_pool.csv')}
        nav={(int(r['from_node']),int(r['to_node'])):float(r['navigation_cost']) for r in rows(src/'navigation_edges.csv')}
        nav.update({(int(r['from_node']),int(r['to_node'])):float(r['navigation_cost']) for r in rows(folder/'new_navigation_edges.csv')})
        for mode in MODES:
            for r in rows(src/f'seed_navigation_costs_{mode}_staged.csv'):
                nav[int(r['from_node']),int(r['to_node'])]=float(r['navigation_cost'])
            for i in range(len(candidates)+1):nav[i+1,i+1]=0.
            fixed=read(folder/f'fixed_{mode}.json')
            old_cert=certify_old_set(folder,mode)
            support={(int(r['candidate_id']),int(r['task_id'])):r for r in rows(src/f'pair_support_{mode}.csv')}
            min_rows=[r for r in rows(src/'minimum_cover_certificates.csv') if r['mode']==mode]
            min_counts={int(r['region_id']):int(r['minimum_docks']) for r in min_rows}
            selected={}
            for method in METHODS:
                r=lookup[mode,method];assert r['route_valid']=='1' and int(r['covered_tasks'])==36
                assert int(r['docks'])==int(r['minimum_docks'])==sum(min_counts.values())
                assert int(r['new_directed_requests'])<=1500 and int(r['neighbor_evaluations'])<=10000
                ds=[d for d in docks if d['mode']==mode and d['method']==method]
                ds.sort(key=lambda d:int(d['visit_rank']));ids=[int(d['candidate_id']) for d in ds];selected[method]=set(ids)
                assert len(ids)==len(set(ids))==int(r['docks'])
                for region,k in min_counts.items():assert sum(int(candidates[c]['table_id'])==region for c in ids)==k
                for d in ds:
                    original=candidates[int(d['candidate_id'])]
                    assert all(abs(float(d[key])-float(original[key]))<1e-10 for key in ('x_m','y_m','yaw_rad'))
                route=[1]+[c+1 for c in ids];cost=sum(nav[a,b] for a,b in zip(route,route[1:]+[1]))
                assert abs(cost-float(r['navigation_cost']))<1e-7
                assigned=[a for a in assignments if a['mode']==mode and a['method']==method]
                assert sorted(int(a['task_id']) for a in assigned)==list(range(1,37))
                for a in assigned:
                    c,t=int(a['candidate_id']),int(a['task_id']);p=support[c,t]
                    assert c in ids and p['feasible']=='1' and p['assigned_arm']==a['arm']
                assignment_count+=len(assigned)
            F=float(lookup[mode,'fixed_optimum']['navigation_cost']);J=float(lookup[mode,'balanced_strong_init']['navigation_cost'])
            I=float(lookup[mode,'legacy_strong_init']['navigation_cost']);N=float(lookup[mode,'balanced_raw_init']['navigation_cost'])
            S=float(old[mode,'minimum_seed']['navigation_cost']);C=float(old[mode,'joint_local']['navigation_cost'])
            assert abs(F-fixed['upper_bound'])<1e-7
            assert J<=F+1e-7 and I<=F+1e-7
            paired.append(dict(case_id=folder.name,mode=mode,minimum_docks=int(lookup[mode,'fixed_optimum']['docks']),
                staged_J=S,old_joint_J=C,fixed_optimum_J=F,legacy_strong_J=I,balanced_raw_J=N,new_joint_J=J,
                original_fixed_LB=fixed['lower_bound_conservative'],original_fixed_gap=fixed['absolute_gap'],
                old_joint_fixed_J=old_cert['upper_bound'],old_joint_fixed_LB=old_cert['lower_bound_conservative'],old_joint_fixed_gap=old_cert['absolute_gap'],
                gain_vs_fixed_pct=100*(F-J)/F,gain_vs_old_joint_pct=100*(C-J)/C,
                gain_vs_old_joint_fixed_pct=100*(old_cert['upper_bound']-J)/old_cert['upper_bound'],
                order_gain=S-F,docking_gain=F-J,allocation_gain=I-J,initialization_gain=N-J,
                certified_vs_original_set=J<fixed['lower_bound_conservative']-1e-7,
                certified_vs_old_joint_set=J<old_cert['lower_bound_conservative']-1e-7,
                changed_docks=len(selected['balanced_strong_init']-selected['fixed_optimum'])))
        for item in read(folder/'search_events.json'):
            j=item['search'];ev=j['accepted_moves'];ev=[ev] if isinstance(ev,dict) else ev
            for event in ev:
                before,after=event['previous_matrix_visit_order'],event['matrix_visit_order']
                for route,key in ((before,'previous_cost'),(after,'new_cost')):
                    value=sum(nav[a,b] for a,b in zip(route,route[1:]+[1]))
                    assert abs(value-event[key])<1e-7
                assert event['new_cost']<event['previous_cost']-1e-9
                move_count+=1
    csv_write(OUT/'summary.csv',all_rows);csv_write(OUT/'paired_attribution.csv',paired)
    aggregates=[]
    for mode in MODES:
        subset=[r for r in paired if r['mode']==mode]
        aggregates.append(dict(mode=mode,cases=len(subset),certified_vs_original=sum(r['certified_vs_original_set'] for r in subset),
            certified_vs_old_joint=sum(r['certified_vs_old_joint_set'] for r in subset),
            gain_vs_fixed_median_pct=float(np.median([r['gain_vs_fixed_pct'] for r in subset])),
            gain_vs_old_joint_median_pct=float(np.median([r['gain_vs_old_joint_pct'] for r in subset])),
            gain_vs_old_joint_fixed_median_pct=float(np.median([r['gain_vs_old_joint_fixed_pct'] for r in subset])),
            allocation_wins=sum(r['allocation_gain']>1e-7 for r in subset),allocation_ties=sum(abs(r['allocation_gain'])<=1e-7 for r in subset),
            allocation_losses=sum(r['allocation_gain']< -1e-7 for r in subset),
            init_wins=sum(r['initialization_gain']>1e-7 for r in subset),init_ties=sum(abs(r['initialization_gain'])<=1e-7 for r in subset),
            init_losses=sum(r['initialization_gain']< -1e-7 for r in subset)))
    csv_write(OUT/'aggregate_statistics.csv',aggregates)
    audit=dict(status='PASS',unchanged_input_files=len(freeze['sha256']),rows=len(all_rows),groups=len(paired),
               assignments=assignment_count,strict_priced_moves=move_count,coverage='frozen task support, no new IK',
               original_set_certified=sum(r['certified_vs_original_set'] for r in paired),
               old_joint_set_certified=sum(r['certified_vs_old_joint_set'] for r in paired))
    write(OUT/'audit.json',audit);print(json.dumps(audit,indent=2));print(json.dumps(aggregates,indent=2))


if __name__=='__main__':
    if '--certify-available' in sys.argv:
        for folder in sorted((OUT/'cases').iterdir()):
            for mode in MODES:
                if (folder/f'old_joint_graph_{mode}.csv').exists():
                    cert=certify_old_set(folder,mode)
                    print(folder.name,mode,cert['upper_bound'],cert['absolute_gap'],flush=True)
    else:main()
