[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$paperRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
Push-Location $paperRoot
try {
    New-Item -ItemType Directory -Force -Path 'build' | Out-Null
    & latexmk -pdf -interaction=nonstopmode -synctex=1 -file-line-error -outdir=build main.tex
    if ($LASTEXITCODE -ne 0) {
        throw "latexmk failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
