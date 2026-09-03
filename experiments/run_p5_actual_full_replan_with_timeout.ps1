param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("ADD_LOCAL_OBSTACLE", "INVALIDATE_DOCK", "ADD_TASKS")]
    [string]$EventType,

    [int]$Seed = 2026,
    [int]$TimeoutSeconds = 300,
    [string]$OutputCsv = "results\tables\p5_actual_full_replan_paired.csv",
    [string]$SummaryCsv = "results\tables\dynamic_local_replanning_p5_summary.csv",
    [switch]$ResetRaw
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$eventExpr = "@{'ADD_LOCAL_OBSTACLE','INVALIDATE_DOCK','ADD_TASKS'}"
$resetExpr = if ($ResetRaw) { "true" } else { "false" }
$matlabExpr = @"
project_root=pwd;
addpath(fullfile(project_root,'pipeline'));
init_paths(project_root);
addpath(fullfile(project_root,'experiments'));
options=struct();
options.seeds=$Seed;
options.events={'$EventType'};
options.run_full_replan=true;
options.raw_csv='$OutputCsv';
options.summary_csv='$SummaryCsv';
options.reset_raw=$resetExpr;
exp_dynamic_local_replanning_p5(options);
"@

$process = Start-Process -FilePath "matlab" -ArgumentList @("-batch", $matlabExpr) -WorkingDirectory $repoRoot -WindowStyle Hidden -PassThru
if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
    Stop-Process -Id $process.Id -Force
    exit 124
}
exit $process.ExitCode
