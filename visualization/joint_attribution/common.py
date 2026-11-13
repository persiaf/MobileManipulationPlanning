"""Immutable input adapter and inherited main-scene drawing style."""
from pathlib import Path
import sys
import csv
import json
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'visualization/main_joint_task_navigation'))
from main_common import style,save,caption,draw_map,draw_base,scene_legend,MODE_LABELS,PATH_COLOR,TASK_FACE,TASK_EDGE,START_COLOR
SOURCE=ROOT/'results/task_navigation_robustness_20261111'
OUT=ROOT/'results/joint_attribution_20261113'
FIG=OUT/'figures'
MODES=('position_only','top_down_grasp','table_wipe')
def rows(p):
    with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def panel(ax,label):ax.text(0,1.02,label,transform=ax.transAxes,ha='left',va='bottom',fontsize=10)
