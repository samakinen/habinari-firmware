# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen

param(
    [string]$InputXml = "ets_export/habinari_tp1_ets.knxprod.xml",
    [string]$OutputKnxprod = "ets_export/habinari_tp1_ets.knxprod"
)

$producer = Get-Command OpenKNXproducer -ErrorAction SilentlyContinue
if (-not $producer) {
    Write-Error "OpenKNXproducer not found in PATH. Install it on the ETS host first."
    exit 1
}

$resolvedInput = Resolve-Path $InputXml -ErrorAction Stop
$outputDir = Split-Path -Parent $OutputKnxprod
if ($outputDir) {
    New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
}

& $producer.Source knxprod -o $OutputKnxprod $resolvedInput.Path
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host "Created $OutputKnxprod"