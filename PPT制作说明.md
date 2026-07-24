# PPT制作说明

## 1. 交付文件

- `代鹏_研究内容阶段汇报_5页.pptx`：5页可编辑PowerPoint文件。
- `代鹏_研究内容阶段汇报_5页.pdf`：由Microsoft PowerPoint导出的5页PDF预览。
- `preview/slide_01.png` 至 `preview/slide_05.png`：逐页PNG预览。

## 2. 项目材料检查

已确认并使用以下材料：

- 模板：`D:\file_used\西柚\实验室_任务\硕士论文相关\中期检查\中期答辩-代鹏.pptx`
- 开题答辩：`D:\file_used\西柚\实验室_任务\硕士论文相关\开题答辩-代鹏.pptx`
- 中期报告：`D:\file_used\西柚\实验室_任务\硕士论文相关\中期检查\代鹏中期检查报告 (1).pdf`
- 英文小论文：`D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\论文稿\paper_tac\build\A Constrained Hierarchical Planning Framework for Reachability-Aware Multi-Task Mobile Manipulation.pdf`
- MATLAB项目：`D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning`

相关项目结构：

```text
MobileManipulationPlanning/
|-- main_mobile_manipulation_planning.m
|-- config/
|-- environment/
|-- collision/
|-- manipulator/
|-- docking/
|-- global_planner/
|-- pipeline/
|-- experiments/
|-- visualization/
|-- results/
|   |-- figures/
|   |-- tables/
|   |-- raw/
|   `-- logs/
|-- preview/
|-- 代鹏_研究内容阶段汇报_5页.pptx
`-- 代鹏_研究内容阶段汇报_5页.pdf
```

## 3. 已核验研究流程

场景与任务输入 -> 任务区域划分 -> 候选停靠位姿生成 -> footprint碰撞检测 -> 安全距离筛选 -> 左右机械臂联合可达性评价 -> 候选位姿综合代价 -> 覆盖感知多停靠选择 -> 两两路径代价矩阵 -> 最近邻访问排序 -> 可选2-opt优化 -> 全局路径拼接 -> 碰撞复核 -> 结果保存与可视化。

## 4. PPT页面标题

1. 研究问题与总体研究思路
2. RCH-MT-MMP分层规划框架
3. 可达性约束的停靠评价与多停靠覆盖方法
4. MATLAB实验结果与方法对比
5. 阶段性研究成果总结

## 5. 数据与图片来源

第1页和第4页使用：

- `results/figures/fig_main_scenario_publication.png`

第4页对比数据使用：

- `results/tables/ablation_summary.csv`

采用的最新可追溯数据：

- 地图尺寸：10 m x 10 m
- 桌子数量：16
- 桌面任务点：36
- 完整方法任务完成：36/36
- 选定停靠位姿：27
- A*总路径长度：63.29 m
- 完整方法完成率：100.00%
- 单停靠方法完成率：69.44%
- 最近停靠方法完成率：50.00%

## 6. 数据差异说明

任务说明中给出的73.73%和25.59%来自论文中的另一组投稿级汇总口径；当前项目最新的`results/tables/ablation_summary.csv`记录为69.44%和50.00%。本PPT遵循“优先使用当前results目录最新且可追溯结果”的要求，未混用两套数据。

## 7. 图片处理记录

未修改MATLAB算法或实验数据。第1页从原始`fig_main_scenario_publication.png`等比例裁切出上部任务区和下部任务区，去除了图题与图例空白，并保留路径、桌子、任务点、停靠位姿及机器人起点。第4页直接使用原始主场景图；柱状图为PowerPoint原生可编辑图表。

## 8. 质量检查

- PowerPoint打开检查：通过，页数为5。
- 页面标题检查：通过。
- 禁用内容检查：未发现“未来工作”“下一步”“深度强化学习”“DDPG/DDQN”等内容。
- 占位符检查：最终PPTX的5个slide XML中空占位符数量为0。
- PDF导出：完成，页数为5。
- PNG渲染：完成，5页均已逐页检查。
- 字体与中文显示：未发现乱码或字体丢失。
- 公式、图表、页眉、校徽、校训、页码：逐页可见且未发现遮挡。
- 自动溢出工具提示5页母版对象越界；复核表明这是测试扩展画布时母版未同步平移造成的误报。最终PPT、PNG和PDF中未发现可见内容越界。
- 模板一致性工具将已在`template-frame-map.json`中明确删除的旧占位内容误判为被新内容覆盖；最终XML确认旧占位符已删除，不存在隐藏遮挡。

## 9. 人工确认项

如最终汇报必须与论文投稿级汇总表完全同口径，需要作者确认是否改用73.73%和25.59%。当前版本采用项目`results`中的69.44%和50.00%，这是按任务要求确定的默认口径。
