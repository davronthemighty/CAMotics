# SPDX-License-Identifier: GPL-2.0-or-later
param(

  [ValidateRange(1, 64)]
  [int]$Jobs = 16,

  [string]$BundleDir = "",

  [string]$Msys2Root = "",

  [string]$CbangDir = "",

  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceDateEpoch = $env:SOURCE_DATE_EPOCH
if (-not $sourceDateEpoch) {
  $sourceDateEpoch = (& git -C $repoRoot show -s --format=%ct HEAD).Trim()
  if ($LASTEXITCODE) { throw "Could not determine SOURCE_DATE_EPOCH" }
}
if ($sourceDateEpoch -notmatch "^\d+$") {
  throw "SOURCE_DATE_EPOCH must be an integer"
}
$env:SOURCE_DATE_EPOCH = $sourceDateEpoch

$msysRoot = $Msys2Root
if (-not $msysRoot) { $msysRoot = $env:MSYS2_ROOT }
if (-not $msysRoot) { $msysRoot = "C:\msys64" }
$msysRoot = [IO.Path]::GetFullPath($msysRoot)

$bash = Join-Path $msysRoot "usr\bin\bash.exe"
$msysScons = Join-Path $msysRoot "usr\bin\scons"
$ucrtBin = Join-Path $msysRoot "ucrt64\bin"
$scons = Join-Path $ucrtBin "scons.exe"
$deployQt = Join-Path $ucrtBin "windeployqt-qt5.exe"
$objdump = Join-Path $ucrtBin "objdump.exe"
$cbangDir = $CbangDir
if (-not $cbangDir) { $cbangDir = $env:CBANG_HOME }
if (-not $cbangDir) { $cbangDir = Join-Path $repoRoot "build-deps\cbang" }
$cbangDir = [IO.Path]::GetFullPath($cbangDir)
$cbangCommit = "62bd9aa11938236ac3f1568e8bfdeaa160c14eac"

foreach ($tool in @($bash, $msysScons, $scons, $deployQt, $objdump)) {
  if (-not (Test-Path -LiteralPath $tool)) {
    throw "Missing MSYS2 build tool: $tool"
  }
}

if (-not (Test-Path -LiteralPath (Join-Path $cbangDir ".git"))) {
  New-Item -ItemType Directory -Force -Path (Split-Path $cbangDir) | Out-Null
  & git clone --depth=1 https://github.com/CauldronDevelopmentLLC/cbang.git $cbangDir
  if ($LASTEXITCODE) { throw "Cbang clone failed" }
}

$cbangStatus = & git -C $cbangDir status --porcelain
if ($LASTEXITCODE -or $cbangStatus) {
  throw "The local Cbang checkout is not clean: $cbangDir"
}

$cbangHead = & git -C $cbangDir rev-parse HEAD
if ($LASTEXITCODE) { throw "Could not read the Cbang revision" }
if ($cbangHead.Trim() -ne $cbangCommit) {
  & git -C $cbangDir fetch --depth=1 origin $cbangCommit
  if ($LASTEXITCODE) { throw "Could not fetch pinned Cbang revision" }
  & git -C $cbangDir checkout --detach $cbangCommit
  if ($LASTEXITCODE) { throw "Could not select pinned Cbang revision" }
}

function Invoke-Ucrt64 {
  param(
    [Parameter(Mandatory)] [string]$WorkingDirectory,
    [Parameter(Mandatory)] [string]$Command
  )

  $oldMsystem = $env:MSYSTEM
  $oldChere = $env:CHERE_INVOKING
  $oldArch = $env:TARGET_ARCH
  try {
    $env:MSYSTEM = "UCRT64"
    $env:CHERE_INVOKING = "1"
    $env:TARGET_ARCH = "x86_64"
    Push-Location $WorkingDirectory
    try {
      & $bash -lc $Command
      if ($LASTEXITCODE) { throw "UCRT64 command failed: $Command" }
    } finally {
      Pop-Location
    }
  } finally {
    $env:MSYSTEM = $oldMsystem
    $env:CHERE_INVOKING = $oldChere
    $env:TARGET_ARCH = $oldArch
  }
}

$common = @(
  "optimize=1", "debug=0", "harden=0", "strict=0", "ccache=0",
  "platform=win32", "compiler=gnu", "python=0",
  "cc=gcc", "cxx=g++", "ar=ar", "ranlib=ranlib",
  "ccflags=-D_WIN32_WINNT=0x0601", "cxxflags=-fpermissive"
) -join " "

if (-not $SkipBuild) {
  # Cbang's static archive has enough objects to exceed Windows' command-line
  # limit under native SCons.  MSYS SCons uses its POSIX spawn path while the
  # UCRT64 shell still resolves gcc, g++, ar, and ranlib to native tools.
  Invoke-Ucrt64 $cbangDir "/usr/bin/scons -j$Jobs $common with_openssl=0"
  if ($cbangDir.Contains("'")) {
    throw "The Cbang path cannot contain a single quote: $cbangDir"
  }
  $cbangUnix = (& $bash -lc "cygpath -u '$cbangDir'").Trim()
  if ($LASTEXITCODE -or -not $cbangUnix) {
    throw "Could not convert the Cbang path for MSYS2: $cbangDir"
  }
  $camoticsCommand =
    "export CBANG_HOME=`"$cbangUnix`"; " +
    "/ucrt64/bin/scons -j$Jobs $common with_tpl=0 with_gui=1 cross_mingw=1"
  Invoke-Ucrt64 $repoRoot $camoticsCommand
}

if (-not $BundleDir) {
  $revision = (& git -C $repoRoot rev-parse --short=8 HEAD).Trim()
  if ($LASTEXITCODE) { throw "Could not read the CAMotics revision" }
  $BundleDir = Join-Path $repoRoot "cross_test\windows-camotics-$revision-msys2"
} elseif (-not [IO.Path]::IsPathRooted($BundleDir)) {
  $BundleDir = Join-Path $repoRoot $BundleDir
}

if (Test-Path -LiteralPath $BundleDir) {
  throw "Bundle directory already exists: $BundleDir"
}

New-Item -ItemType Directory -Path $BundleDir | Out-Null
$executables = Get-ChildItem $repoRoot -File -Filter "*.exe"
if (-not ($executables.Name -contains "camotics.exe")) {
  throw "camotics.exe was not built"
}
$executables | Copy-Item -Destination $BundleDir
Copy-Item -Recurse -LiteralPath (Join-Path $repoRoot "machines") `
  -Destination $BundleDir

$oldPath = $env:PATH
try {
  $env:PATH = "$ucrtBin;$oldPath"
  & $deployQt --release --no-compiler-runtime --no-opengl-sw --no-translations `
    --dir $BundleDir (Join-Path $BundleDir "camotics.exe")
  $deployExit = $LASTEXITCODE
} finally {
  $env:PATH = $oldPath
}

$qtDlls = @(
  "Qt5Core.dll", "Qt5Gui.dll", "Qt5Network.dll", "Qt5WebSockets.dll",
  "Qt5Widgets.dll"
)
$missingQtDlls = $qtDlls | Where-Object {
  -not (Test-Path -LiteralPath (Join-Path $BundleDir $_))
}
if ($missingQtDlls) {
  throw "windeployqt omitted required DLLs: $($missingQtDlls -join ', ')"
}
if ($deployExit) {
  Write-Warning (
    "windeployqt returned $deployExit because MSYS2 Qt has no ANGLE " +
    "libGLESv2.dll; required desktop-OpenGL Qt DLLs were deployed"
  )
}

$platformDir = Join-Path $BundleDir "platforms"
New-Item -ItemType Directory -Force -Path $platformDir | Out-Null
Copy-Item -LiteralPath `
  (Join-Path $msysRoot "ucrt64\share\qt5\plugins\platforms\qwindows.dll") `
  -Destination $platformDir

# MSYS2's windeployqt deploys Qt DLLs but not their non-Qt UCRT64 dependency
# closure.  Resolve every executable and plugin recursively with objdump.
$queue = [Collections.Generic.Queue[string]]::new()
Get-ChildItem $BundleDir -Recurse -File |
  Where-Object { $_.Extension -in ".exe", ".dll" } |
  ForEach-Object { $queue.Enqueue($_.FullName) }
$seen = @{}
while ($queue.Count) {
  $file = $queue.Dequeue()
  $key = $file.ToLowerInvariant()
  if ($seen.ContainsKey($key)) { continue }
  $seen[$key] = $true

  foreach ($line in (& $objdump -p $file 2>$null)) {
    if ($line -notmatch "DLL Name:\s*(.+)$") { continue }
    $name = $Matches[1].Trim()
    $present = Get-ChildItem $BundleDir -Recurse -File -Filter $name |
      Select-Object -First 1
    if ($present) { continue }

    $source = Join-Path $ucrtBin $name
    if (Test-Path -LiteralPath $source) {
      $destination = Join-Path $BundleDir $name
      Copy-Item -LiteralPath $source -Destination $destination
      $queue.Enqueue($destination)
    }
  }
}

& (Join-Path $BundleDir "camsim.exe") --version
if ($LASTEXITCODE) { throw "Packaged camsim runtime check failed" }

$size = Get-ChildItem $BundleDir -Recurse -File | Measure-Object Length -Sum
Write-Host "Windows CAMotics bundle: $BundleDir"
Write-Host ("Files: {0}; bytes: {1}" -f $size.Count, $size.Sum)
