# 清理执行报告（2026-10-27）

用户已明确授权删除无用备份和实验残留。本次已执行清理，并保留云端完整保存点和去重历史归档。

清理前保存点：[75cc128](https://github.com/persiaf/MobileManipulationPlanning/releases/tag/checkpoint-20261027-095303)。所有 5 个云端附件在删除前核对了大小和 GitHub 返回的 SHA-256。

## 清理结果

| 类别 | 数量 | 原逻辑体积 | 处置 |
| --- | --- | --- | --- |
| 旧备份和散落实验输出 | 74 个位置 / 14,332 个文件 | 26.70 GiB | 原散落目录/压缩包已删除；全部内容可由去重历史归档和云端基线重建 |
| 项目内实验残留 | 188 个文件 | 821.32 MiB | 已删除；每个文件均有完全相同的云端保存点副本 |
| 过期 Codex 配置备份 | 4 个 | 14,915 B | 已删除；保留最近配置回滚点、代理修复前环境文件和状态库回滚点 |
| 失效备份路径指针 | 1 个 | 154 B | 原指向已归档的旧备份目录，已删除 |
| 已上传的本地临时快照 | 7 个文件 | 842.42 MiB | 已删除；云端附件保留 |
| 集中保留的独有历史归档 | 1788 个去重内容对象 | 2.16 GiB | 保留 history_unique.zip；减少重复占用 |

共移除 14,532 个原文件（目录删除按完整文件清单计数）。扣除历史归档 ZIP 后，净减少逻辑数据约 **26.17 GiB**；另有少量清单和报告文件。磁盘可用空间变化见 verification.json，受文件系统分配及其他进程写入影响。

| 项目内已删除类别 | 文件数 | 体积 |
| --- | --- | --- |
| HISTORICAL_LOG | 50 | 3.76 MiB |
| LEGACY_LEARNING_PARTIAL | 9 | 3.93 MiB |
| LEGACY_LNN_EXPERIMENT | 84 | 699.79 MiB |
| LEGACY_PAPER_CACHE | 1 | 47.66 MiB |
| PROJECT_ARCHIVE | 2 | 54.29 MiB |
| REDUNDANT_PARTIAL_CONFIRMED | 5 | 0.02 MiB |
| SMOKE_VISUAL_OUTPUT | 37 | 11.88 MiB |

项目内删除包括旧 LNN 数据/模型/导出及对应缓存与 9 份旧学习实验 partial，37 份 GUI/渲染试跑图片，5 份被正式结果覆盖的 partial CSV，50 份历史日志和两个重复的本地项目/论文 ZIP。正式 raw/summary 表、论文成品、源代码和冻结复现输入保留。

## 保留项与验证

- 383 个原 MATLAB 源文件均存在且 SHA-256 与清理前一致；所有 778 个未列入删除清单的原项目文件均逐个核对，无内容变化。

- 8 个 P3/P4 冻结状态、3 个 Position-only seed state、resource_bounded 续跑 CSV，以及两份没有重复覆盖依据的中断/部分结果均保留并核对哈希。

- 当前 Codex config.toml/.env 未作为删除目标；保留最近 config.toml 备份、今天代理修复前 .env 备份、旧状态库备份及应用自动状态备份。

- 没有运行会重写结果的 MATLAB 实验；本次验证覆盖删除范围、源文件哈希、复现依赖、ZIP 完整性和恢复材料。

## 独有历史记录如何恢复

归档目录：`D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\codex_backups\retained_history_20261027`。

- [history_unique.zip](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/retained_history_20261027/history_unique.zip>)（2.16 GiB）

- [完整原路径与内容映射](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/retained_history_20261027/history_manifest.json>)；[归档 SHA-256](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/retained_history_20261027/SHA256SUMS.txt>)；[恢复工具](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/codex_backups/retained_history_20261027/restore_history.py>)

归档保存 14,332 个旧文件的完整路径映射；其中 10,053 个文件的内容已经存在于云端完整基线中，其余内容以去重对象保存。旧 .git 历史、补丁、报告和较早压缩包的独有内容也在此范围内。

恢复时从清理前 Release 下载自定义 workspace ZIP，并准备好 history_unique.zip。使用新的空目录：

```powershell
python restore_history.py --history history_unique.zip --baseline MobileManipulationPlanning-checkpoint-20261027-095303-workspace.zip --destination D:\recovered_history_new
```

恢复工具逐文件校验 SHA-256，不写回原备份位置。历史归档依赖云端基线，二者共同构成恢复材料；该历史归档当前保存在本机，并未上传其独有内容。

## 尚未删除的权限例外

`D:\file_used\西柚\实验室_任务\硕士论文相关\小论文\MobileManipulationPlanning_removed_python_cache_20260720` 中的旧 `.pytest_cache` 保留原位。原审计记录为 6 个文件、1,259 字节。Administrator 无法列出该缓存的内容，限定在此缓存目录的所有权修正也被 Windows 拒绝，因此本轮未对它实施删除。该限制来自 Windows 文件权限，并非自动审批拒绝。其他清理已完成。

## 明细

- [逐文件删除清单（含原路径、大小、SHA-256 和恢复来源）](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/planned_deletions.csv>)

- [实际删除日志](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/deletion_log.csv>)

- [精确执行计划](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/cleanup_plan.json>)；[清理后验证结果](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/verification.json>)

- [删除前云端保存点验证](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/cloud_verification.json>)；[云端基线文件清单](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/baseline_workspace_manifest.json>)

- [历史归档和恢复来源检查](<D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning/docs/cleanup_execution_20261027/preparation_verification.json>)

此前 docs/cleanup_audit_20261027/ 是删除前的盘点快照，其中原备份路径属于历史记录；本报告反映执行后的状态。
