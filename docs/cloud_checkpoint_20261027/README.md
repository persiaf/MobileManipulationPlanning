# GitHub 工作区保存点：2026-10-27

此保存点用于保存当前已有代码和文件状态，没有进行算法修改或清理实验数据。

- 仓库：<https://github.com/persiaf/MobileManipulationPlanning>（私有）。
- 代码分支：`research/cafeteria-scene-planning`。
- 保存点标签：`checkpoint-20261027-095303`。
- 完整附件入口：<https://github.com/persiaf/MobileManipulationPlanning/releases/tag/checkpoint-20261027-095303>。

普通 Git 提交保存当前源代码、配置、测试、文档、审计清单、图表和较小实验结果，也记录当前已有删除。保留当前分支，不改写 main 或其他分支历史。

大型冻结状态 `results/tables/p3p4_state_*.mat` 与 `position_only_resource_bounded_seed*_state.mat` 共 11 个文件保留原位，新增精确忽略规则，避免超大数据直接进入普通 Git 历史。它们与原先被忽略的实验原始数据、历史模型、日志和媒体一起，保存在完整 workspace ZIP 附件中。

附件包括：

1. `MobileManipulationPlanning-checkpoint-20261027-095303-workspace.zip`：当前项目全部文件的原始字节，以及空目录，排除 `.git`。
2. `MobileManipulationPlanning-checkpoint-20261027-095303.bundle`：当前分支的 Git 历史、HEAD 和保存点标签。
3. `workspace_manifest.json`：每个项目文件的 SHA-256、大小、时间和 tracked/ignored 状态。
4. `SHA256SUMS.txt`：附件校验和。
5. `RESTORE.md`：恢复说明。

完整项目快照的范围是当前 MobileManipulationPlanning 目录。项目外的历史备份及 Codex 配置备份仅包含此前生成的路径/元数据审计清单；其原始内容属于外部目录。

恢复时先校验附件，在一个新目录克隆 Git bundle，再把 workspace ZIP 覆盖到该新克隆中，保留它的 `.git`。这样既能恢复版本历史，也能恢复被忽略的原始数据和精确文件字节。GitHub 自动生成的 Source code ZIP 只包含 Git 跟踪文件，完整恢复应使用上面单独上传的 workspace ZIP。

本轮验证保存范围、文件数、ZIP CRC、Git bundle 和上传附件 SHA-256；不运行会产生或覆盖实验结果的 MATLAB 流程。已有 MATLAB 行为没有在本次保存中重新认证。原有 `config/scene_config.m` 有 17 处注释行尾空格，`git diff --check` 会报告这些既有格式问题，本轮按当前状态保存。

`cloud_snapshot.py` 使用系统 Git credential helper 获取临时凭据，仅在内存中供 GitHub API 请求使用，不把凭据写入项目或附件。其 prepare/upload 输出目录必须在项目树外，便于防止快照包含自身。
