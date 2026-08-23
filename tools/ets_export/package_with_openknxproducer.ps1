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

# The exporter writes a self-contained product XML: it carries a MasterData
# section so the file can be validated and read back without ETS. A .knxprod
# must not. OpenKNXproducer splits the input into Catalog.xml, Hardware.xml and
# the application program file, and it only ever *removes* the elements it
# knows about — MasterData is copied into all three. ETS then signs them, and
# its hasher walks every element name through its registration-relevance table,
# where MasterData does not exist because master data is not manufacturer data:
#
#   System.ArgumentException: Naming pair for MasterData cannot be found.
#
# So strip it here. ETS supplies its own knx_master.xml for the archive.
$inputDir = Split-Path -Parent $resolvedInput.Path
$signingName = (Split-Path -Leaf $resolvedInput.Path) -replace '\.xml$', '.signing.xml'
$signingXml = Join-Path $inputDir $signingName

$doc = New-Object System.Xml.XmlDocument
$doc.PreserveWhitespace = $true
$doc.Load($resolvedInput.Path)
$master = $doc.DocumentElement.SelectSingleNode("*[local-name()='MasterData']")
if ($master) {
    $doc.DocumentElement.RemoveChild($master) | Out-Null
}
$doc.Save($signingXml)

try {
    # -N: skip the XSD auto-search. It looks for an <?xml-model?> processing
    # instruction, which this file does not carry, and then reports the working
    # directory as a missing schema file. Nothing was ever validated by it.
    & $producer.Source knxprod -N -o $OutputKnxprod $signingXml
    $producerExit = $LASTEXITCODE
} finally {
    Remove-Item -Path $signingXml -ErrorAction SilentlyContinue
}

# OpenKNXproducer writes the archive and only then prints a success banner, for
# which it re-reads the application program id with a hardcoded regex. A crash
# there (System.ArgumentOutOfRangeException out of Program.ExportKnxprod) says
# nothing about the archive, which is already on disk — and the producer deletes
# any previous output before it starts, so the file existing means it was
# written by this run.
if ($producerExit -ne 0) {
    if (Test-Path $OutputKnxprod) {
        Write-Warning ("OpenKNXproducer exited with $producerExit after writing " +
            "$OutputKnxprod. The archive is there; check the output above for what " +
            "it tripped over.")
    } else {
        exit $producerExit
    }
}

Write-Host "Created $OutputKnxprod"
