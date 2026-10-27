param([string]$PlanPath = (Join-Path $PSScriptRoot 'cleanup_plan.json'), [switch]$ValidateOnly)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# One PowerShell process performs all deletions through native literal-path
# cmdlets. No target is interpolated into another shell command.
$plan = Get-Content -LiteralPath $PlanPath -Raw | ConvertFrom-Json
$workspace = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..')).TrimEnd('\')
$parent = [IO.Directory]::GetParent($workspace).FullName
$codexRoot = 'C:\Users\Administrator\.codex'
$expectedTemporaryRoot = 'C:\Users\Administrator\AppData\Local\Temp\MobileManipulationPlanning_github_checkpoint_20261027_095303'
$expectedArchiveRoot = Join-Path $parent 'codex_backups\retained_history_20261027'
if ($plan.workspace_root -ne $workspace -or $plan.parent_root -ne $parent -or $plan.codex_root -ne $codexRoot) {
    throw 'Plan workspace roots do not match this script location.'
}
function Full-Existing([string]$Path) {
    return (Resolve-Path -LiteralPath $Path -ErrorAction Stop).ProviderPath.TrimEnd('\')
}
function Assert-Contained([string]$Target, [string]$Container) {
    $t = [IO.Path]::GetFullPath($Target).TrimEnd('\')
    $c = [IO.Path]::GetFullPath($Container).TrimEnd('\')
    if (-not $t.StartsWith($c + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Target is outside intended container: $t"
    }
}
function Assert-Metadata($Item) {
    $p = Full-Existing $Item.absolute_path
    $file = Get-Item -LiteralPath $p -Force
    if ($file.PSIsContainer -or ($file.Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw "Not a plain file: $p" }
    if ($file.Length -ne [long]$Item.bytes -or $file.LastWriteTimeUtc.Ticks -ne [long]$Item.mtime_ticks) {
        throw "File changed after archive preparation: $p"
    }
}
function Assert-Hash($Item) {
    Assert-Metadata $Item
    if ((Get-FileHash -LiteralPath $Item.absolute_path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $Item.sha256) {
        throw "File hash changed before deletion: $($Item.absolute_path)"
    }
}
function Assert-ExternalRoot($Root) {
    $p = Full-Existing $Root.absolute_path
    if ($p -eq $workspace -or $p.StartsWith($workspace + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'External target overlaps workspace.' }
    $base = [IO.Path]::GetDirectoryName($p)
    $leaf = [IO.Path]::GetFileName($p)
    $families = @((Join-Path $parent 'codex_backups\MobileManipulationPlanning'), (Join-Path $parent 'codex_backups\paper_tac'))
    $allowed = ($base -in $families) -or
        ($base -eq $parent -and ($leaf.StartsWith('MobileManipulationPlanning') -or $leaf -in @('results','stage_output_custom_scene','stage_output_custom_scene_v2'))) -or
        ($leaf -eq 'MobileManipulationPlanning_cleanup_20260831_181820' -and $base -eq [IO.Path]::GetDirectoryName($expectedTemporaryRoot))
    if (-not $allowed) { throw "External root is not an explicitly audited backup/output location: $p" }
    if ($p -eq $expectedArchiveRoot -or $p.StartsWith($expectedArchiveRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Retained archive was selected for deletion.' }
    return $p
}

Assert-Contained $plan.retained_archive $expectedArchiveRoot
Assert-Contained $plan.history_manifest $expectedArchiveRoot
$archive = Full-Existing $plan.retained_archive
$manifestPath = Full-Existing $plan.history_manifest
if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $plan.retained_archive_sha256) { throw 'Retained archive hash failed.' }
if ((Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $plan.history_manifest_sha256) { throw 'Retained history manifest hash failed.' }
$history = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
if ($history.files.Count -ne $plan.summary.external_files) { throw 'History file coverage differs from the deletion plan.' }

# Preflight every target before any deletion. All recursive roots have a
# complete archived inventory and contain no reparse points or new files.
foreach ($root in $plan.external_roots) {
    $p = Assert-ExternalRoot $root
    $item = Get-Item -LiteralPath $p -Force
    if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse root: $p" }
    $expectedFiles = @($history.files | Where-Object { $_.root_id -eq $root.id })
    if ($root.is_directory) {
        $entries = @(Get-ChildItem -LiteralPath $p -Force -Recurse -ErrorAction Stop)
        foreach ($entry in $entries) { if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Reparse entry: $($entry.FullName)" } }
        $actualFiles = @($entries | Where-Object { -not $_.PSIsContainer })
        $actualDirs = @($entries | Where-Object { $_.PSIsContainer })
        if ($actualFiles.Count -ne $expectedFiles.Count -or $actualDirs.Count -ne $root.directories.Count) { throw "Backup membership changed: $p" }
    }
    foreach ($f in $expectedFiles) {
        if ($root.is_directory) { Assert-Contained $f.absolute_path $p } elseif ($f.absolute_path -ne $p) { throw 'File root member mismatch.' }
        Assert-Metadata $f
    }
}
foreach ($f in $plan.project_files) {
    Assert-Contained $f.absolute_path $workspace
    if ($f.absolute_path -match '[\\/]\.git([\\/]|$)' -or [IO.Path]::GetExtension($f.absolute_path) -eq '.m') { throw 'Source or Git metadata in project deletion plan.' }
    Assert-Hash $f
}
$allowedCodexNames = @('config.toml.bak-20260701-154201','config.toml.bak-before-code-skit-20260724','config.toml.bak-before-medium-20260903-0948','config.toml.bak-before-stream-fix-20260720-2155','tmp_large_scale_backup_path.txt')
foreach ($f in $plan.codex_files) {
    if ([IO.Path]::GetDirectoryName($f.absolute_path) -ne $codexRoot -or [IO.Path]::GetFileName($f.absolute_path) -notin $allowedCodexNames) { throw 'Unexpected Codex target.' }
    Assert-Hash $f
}
foreach ($name in @('config.toml','.env','config.toml.bak-too-many-pending-20260915-1345','.env.bak-20261027-093020-proxy-fix','state_5.sqlite.bak-before-gpt55-low-20260903-0945')) {
    if (-not (Test-Path -LiteralPath (Join-Path $codexRoot $name) -PathType Leaf)) { throw "Required retained config missing: $name" }
}
foreach ($f in $plan.protected_project_files) {
    $p = Join-Path $workspace $f.path
    if ((Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash.ToLowerInvariant() -ne $f.sha256) { throw "Protected experiment input changed: $p" }
}
if ((Full-Existing $plan.temporary_root) -ne $expectedTemporaryRoot) { throw 'Unexpected uploaded temporary snapshot path.' }
$temporaryEntries = @(Get-ChildItem -LiteralPath $expectedTemporaryRoot -Force -Recurse)
if (@($temporaryEntries | Where-Object { -not $_.PSIsContainer }).Count -ne $plan.temporary_files.Count) { throw 'Temporary snapshot membership changed.' }
foreach ($entry in $temporaryEntries) { if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Temporary snapshot contains reparse point.' } }
foreach ($f in $plan.temporary_files) { Assert-Contained $f.absolute_path $expectedTemporaryRoot; Assert-Metadata $f }

Write-Output "Preflight passed. Retained archive: $archive"
Write-Output "Validated external roots: $($plan.external_roots.Count); project artifact files: $($plan.project_files.Count); obsolete config/pointer files: $($plan.codex_files.Count)"
if ($ValidateOnly) { Write-Output 'Validation-only run: no files deleted.'; return }
$logPath = Join-Path $PSScriptRoot 'deletion_log.csv'
'scope,path,status' | Set-Content -LiteralPath $logPath -Encoding utf8
function Record-Removal([string]$Scope, [string]$Path) {
    [pscustomobject]@{scope=$Scope; path=$Path; status='REMOVED'} | ConvertTo-Csv -NoTypeInformation | Select-Object -Skip 1 | Add-Content -LiteralPath $logPath -Encoding utf8
}
foreach ($root in $plan.external_roots) {
    # Re-resolve the final absolute target immediately before recursive delete.
    $p = Assert-ExternalRoot $root
    if ($root.is_directory) { Remove-Item -LiteralPath $p -Recurse -Force -ErrorAction Stop }
    else { Remove-Item -LiteralPath $p -Force -ErrorAction Stop }
    Record-Removal 'EXTERNAL_BACKUP' $p
}
foreach ($f in $plan.project_files) {
    Assert-Contained (Full-Existing $f.absolute_path) $workspace
    Remove-Item -LiteralPath $f.absolute_path -Force -ErrorAction Stop
    Record-Removal 'PROJECT_DERIVED_DATA' $f.absolute_path
}
foreach ($f in $plan.codex_files) {
    Remove-Item -LiteralPath (Full-Existing $f.absolute_path) -Force -ErrorAction Stop
    Record-Removal 'OBSOLETE_CODEX_CONFIG' $f.absolute_path
}
foreach ($p in $plan.project_empty_directory_candidates) {
    Assert-Contained $p $workspace
    if (Test-Path -LiteralPath $p -PathType Container) {
        if (@(Get-ChildItem -LiteralPath $p -Force).Count -eq 0) { Remove-Item -LiteralPath $p -Force; Record-Removal 'EMPTY_ARTIFACT_DIRECTORY' $p }
    }
}
foreach ($p in @((Join-Path $parent 'codex_backups\MobileManipulationPlanning'),(Join-Path $parent 'codex_backups\paper_tac'))) {
    if ((Test-Path -LiteralPath $p -PathType Container) -and @(Get-ChildItem -LiteralPath $p -Force).Count -eq 0) { Remove-Item -LiteralPath $p -Force; Record-Removal 'EMPTY_BACKUP_CONTAINER' $p }
}
if ((Full-Existing $plan.temporary_root) -ne $expectedTemporaryRoot) { throw 'Temporary root changed before removal.' }
Remove-Item -LiteralPath $expectedTemporaryRoot -Recurse -Force -ErrorAction Stop
Record-Removal 'UPLOADED_TEMP_SNAPSHOT' $expectedTemporaryRoot
Write-Output 'Cleanup completed. Detailed removal log written next to this script.'
