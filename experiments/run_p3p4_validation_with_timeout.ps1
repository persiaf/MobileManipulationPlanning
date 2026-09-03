param(
    [Parameter(Mandatory = $true)]
    [string]$StateFile,

    [Parameter(Mandatory = $true)]
    [string]$RoutingMode,

    [Parameter(Mandatory = $true)]
    [int]$K,

    [Parameter(Mandatory = $true)]
    [bool]$CacheOn,

    [Parameter(Mandatory = $true)]
    [string]$ModeLabel,

    [string]$OutputCsv = "results\tables\p3p4_final_validation_raw.csv",

    [int]$TimeoutSeconds = 300
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$resolvedState = (Resolve-Path -LiteralPath $StateFile).Path
$resolvedOutput = Join-Path $repoRoot $OutputCsv
$cacheFlag = if ($CacheOn) { "true" } else { "false" }

$runExpr = "addpath(fullfile(pwd,'pipeline')); init_paths(pwd); addpath(fullfile(pwd,'experiments')); run_p3p4_validation_route('$($resolvedState.Replace("'","''"))','$RoutingMode',$K,$cacheFlag,'$($ModeLabel.Replace("'","''"))','$($resolvedOutput.Replace("'","''"))');"
$process = Start-Process -FilePath "matlab" -ArgumentList @("-batch", $runExpr) -WorkingDirectory $repoRoot -WindowStyle Hidden -PassThru

if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
    Stop-Process -Id $process.Id -Force
    $timeoutExpr = "addpath(fullfile(pwd,'pipeline')); init_paths(pwd); addpath(fullfile(pwd,'experiments')); record_p3p4_timeout_row('$($resolvedState.Replace("'","''"))','$RoutingMode',$K,$cacheFlag,'$($ModeLabel.Replace("'","''"))','$($resolvedOutput.Replace("'","''"))',$TimeoutSeconds);"
    $timeoutProcess = Start-Process -FilePath "matlab" -ArgumentList @("-batch", $timeoutExpr) -WorkingDirectory $repoRoot -WindowStyle Hidden -PassThru -Wait
    exit 124
}

exit $process.ExitCode
