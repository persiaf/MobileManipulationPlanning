[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$paperRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
Push-Location $paperRoot
try {
    & latexmk -c -outdir=build main.tex
    if ($LASTEXITCODE -ne 0) {
        throw "latexmk cleanup failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
