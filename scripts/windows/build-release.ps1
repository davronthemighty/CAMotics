# SPDX-License-Identifier: GPL-2.0-or-later
param(
  [Parameter(Mandatory)] [string]$BundleDir,
  [string]$ArtifactDir = "",
  [string]$Msys2Root = ""
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$bundle = (Resolve-Path $BundleDir).Path
if (-not $ArtifactDir) { $ArtifactDir = Join-Path $root "dist" }
$artifacts = [IO.Path]::GetFullPath($ArtifactDir)
New-Item -ItemType Directory -Force -Path $artifacts | Out-Null

$metadata = Get-Content -Raw (Join-Path $root "package.json") | ConvertFrom-Json
$parts = $metadata.version.Split('.')
$releaseTag = "v{0:D4}.{1:D2}.{2}" -f [int]$parts[0], [int]$parts[1], [int]$parts[2]
$artifactBase = "CAMotics-Fast-$releaseTag-windows-x86_64"
$zip = Join-Path $artifacts "$artifactBase.zip"
$installer = Join-Path $artifacts "$artifactBase-setup.exe"

if (-not $Msys2Root) { $Msys2Root = $env:MSYS2_ROOT }
if (-not $Msys2Root) { $Msys2Root = "C:\msys64" }
$zipTool = Join-Path $Msys2Root "usr\bin\zip.exe"
if (-not (Test-Path -LiteralPath $zipTool)) {
  throw "MSYS2 zip is required: $zipTool"
}

$epoch = $env:SOURCE_DATE_EPOCH
if (-not $epoch) {
  $epoch = (& git -C $root show -s --format=%ct HEAD).Trim()
  if ($LASTEXITCODE) { throw "Could not determine SOURCE_DATE_EPOCH" }
}
$stamp = [DateTimeOffset]::FromUnixTimeSeconds([long]$epoch).UtcDateTime
Get-ChildItem $bundle -Recurse -Force | ForEach-Object {
  $_.LastWriteTimeUtc = $stamp
}
if (Test-Path -LiteralPath $zip) { Remove-Item -LiteralPath $zip }
Push-Location $bundle
try {
  & $zipTool -X -q -r $zip .
  if ($LASTEXITCODE) { throw "Portable ZIP creation failed" }
} finally {
  Pop-Location
}

$makensis = Get-Command makensis.exe -ErrorAction SilentlyContinue
$makensisPath = if ($makensis) { $makensis.Source } else { "" }
if (-not $makensisPath) {
  $candidate = "C:\Program Files (x86)\NSIS\makensis.exe"
  if (Test-Path -LiteralPath $candidate) { $makensisPath = $candidate }
}
if (-not $makensisPath) {
  throw "NSIS 3.11 is required to build the installer"
}
$nsisVersion = (& $makensisPath /VERSION).Trim()
if ($nsisVersion -ne "v3.11") {
  throw "Expected pinned NSIS v3.11, found $nsisVersion"
}

$nsisArgs = @(
  "/V2"
  "/DBUNDLE_DIR=$bundle"
  "/DOUTPUT_FILE=$installer"
  "/DPRODUCT_VERSION=$($metadata.version)"
  (Join-Path $PSScriptRoot "installer.nsi")
)
& $makensisPath @nsisArgs
if ($LASTEXITCODE -or -not (Test-Path -LiteralPath $installer)) {
  throw "NSIS installer creation failed"
}

$checksumFile = Join-Path $artifacts "SHA256SUMS.windows"
@($zip, $installer) | ForEach-Object {
  $hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $_).Hash.ToLowerInvariant()
  "$hash  $([IO.Path]::GetFileName($_))"
} | Set-Content -Encoding ascii $checksumFile

Write-Host "Built $zip"
Write-Host "Built $installer"
