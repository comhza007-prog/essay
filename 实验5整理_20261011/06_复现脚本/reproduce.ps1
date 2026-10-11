param(
    [switch]$Full,
    [string]$Node = 'C:\Users\ele\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe',
    [string]$Python = 'C:\Users\ele\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [string]$Modules = 'C:\Users\ele\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\node_modules',
    [string]$ChineseFont = 'C:\Windows\Fonts\simsun.ttc'
)
$ErrorActionPreference = 'Stop'
$env:PYTHONIOENCODING = 'utf-8'
$packageRoot = Split-Path -Parent $PSScriptRoot
$resultDir = Join-Path $packageRoot ('复算结果_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
if (Test-Path -LiteralPath $resultDir) { throw "Output exists; do not overwrite: $resultDir" }
New-Item -ItemType Directory -Path $resultDir | Out-Null
Push-Location -LiteralPath $packageRoot
try {
    & $Node (Join-Path $PSScriptRoot 'analyze.mjs') $resultDir | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Analysis failed' }
    & $Python (Join-Path $PSScriptRoot 'build_report.py') $resultDir --verify-only
    if ($LASTEXITCODE -ne 0) { throw 'Independent verification failed' }
    $baseline = Get-Content -LiteralPath (Join-Path $packageRoot '03_统计汇总\analysis.json') -Raw -Encoding utf8 | ConvertFrom-Json
    $actual = Get-Content -LiteralPath (Join-Path $resultDir 'analysis.json') -Raw -Encoding utf8 | ConvertFrom-Json
    if ($actual.selected.Count -ne 18) { throw 'Wrong sample count' }
    foreach ($trial in $baseline.selected) {
        $match = @($actual.selected | Where-Object id -EQ $trial.id)
        if ($match.Count -ne 1 -or $match[0].sourceSHA256 -ne $trial.sourceSHA256) { throw "Source differs: $($trial.id)" }
        foreach ($metric in $trial.metrics.PSObject.Properties) {
            if ([Math]::Abs([double]$metric.Value - [double]$match[0].metrics.($metric.Name)) -gt 1e-10) { throw "Metric differs: $($trial.id) $($metric.Name)" }
        }
    }
    if ($Full) {
        $moduleLink = Join-Path $PSScriptRoot 'node_modules'
        if (-not (Test-Path -LiteralPath $moduleLink)) {
            if (-not (Test-Path -LiteralPath $Modules)) { throw 'Install artifact-tool and sharp, or specify -Modules' }
            New-Item -ItemType Junction -Path $moduleLink -Target $Modules | Out-Null
        }
        & $Node (Join-Path $PSScriptRoot 'build_tables.mjs') $resultDir
        if ($LASTEXITCODE -ne 0) { throw 'Workbook generation failed' }
        & $Node (Join-Path $PSScriptRoot 'build_figures.mjs') $resultDir
        if ($LASTEXITCODE -ne 0) { throw 'Figure generation failed' }
        & $Python (Join-Path $PSScriptRoot 'build_report.py') $resultDir $ChineseFont
        if ($LASTEXITCODE -ne 0) { throw 'Report generation failed' }
        Write-Output 'New report/chart layout must be reviewed if dependency versions or font changed.'
    }
    Write-Output "PASS: all 18 metric records match. Fresh output: $resultDir"
} finally { Pop-Location }
