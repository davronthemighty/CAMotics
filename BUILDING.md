# Building CAMotics Fast

The release configuration uses C++17, Qt 5, SCons, and Cbang.  Cbang is pinned
to commit `62bd9aa11938236ac3f1568e8bfdeaa160c14eac` so local and hosted builds
use the same dependency source.  The legacy TPL/V8 subsystem is disabled in
published binaries with `with_tpl=0`.

## Linux and WSL

These package names are for current Debian and Ubuntu releases:

```sh
sudo apt update
sudo apt install -y \
  build-essential git scons pkg-config python3-dev \
  qtbase5-dev libqt5opengl5-dev libqt5websockets5-dev qttools5-dev-tools \
  libgl1-mesa-dev libglu1-mesa-dev \
  zlib1g-dev libbz2-dev liblz4-dev libexpat1-dev libevent-dev \
  libre2-dev libsqlite3-dev libyaml-dev
```

Build the pinned dependency and CAMotics:

```sh
git clone https://github.com/CauldronDevelopmentLLC/cbang.git build-deps/cbang
git -C build-deps/cbang checkout 62bd9aa11938236ac3f1568e8bfdeaa160c14eac
scons -C build-deps/cbang -j"$(nproc)" with_openssl=0

export CBANG_HOME="$PWD/build-deps/cbang"
scons -j"$(nproc)" with_gui=1 with_tpl=0
```

The executables are written to the repository root.  Start the GUI with
`./camotics`; use `./camsim --help` for the simulator CLI.

WSL uses the same commands.  A WSL GUI additionally needs WSLg or another
working X/Wayland server.  Check the toolchain before building:

```sh
moc -v
pkg-config --modversion Qt5Core Qt5Gui Qt5OpenGL Qt5Widgets Qt5WebSockets
glxinfo -B
```

`llvmpipe` is adequate for automated GUI checks but is software OpenGL.  It is
not representative of native Windows rendering performance.

## Headless build

For servers and geometry tests that do not need Qt rendering:

```sh
CBANG_HOME="$PWD/build-deps/cbang" \
  scons -j"$(nproc)" camsim camsim-path camsim-region-plan \
  with_gui=0 with_tpl=0
```

Run a compact smoke test:

```sh
bash scripts/perf/smoke_dexel_simulation.sh ./camsim
```

## Native Windows with MSYS2 UCRT64

Install the normal 64-bit MSYS2 release, open an MSYS2 terminal once to update
it, then install the UCRT64 toolchain:

```sh
pacman -Syu
pacman -S --needed \
  git scons make zip \
  mingw-w64-ucrt-x86_64-toolchain \
  mingw-w64-ucrt-x86_64-scons \
  mingw-w64-ucrt-x86_64-qt5-base \
  mingw-w64-ucrt-x86_64-qt5-tools \
  mingw-w64-ucrt-x86_64-qt5-websockets \
  mingw-w64-ucrt-x86_64-python \
  mingw-w64-ucrt-x86_64-zlib \
  mingw-w64-ucrt-x86_64-bzip2 \
  mingw-w64-ucrt-x86_64-lz4 \
  mingw-w64-ucrt-x86_64-expat \
  mingw-w64-ucrt-x86_64-libevent \
  mingw-w64-ucrt-x86_64-re2 \
  mingw-w64-ucrt-x86_64-sqlite3 \
  mingw-w64-ucrt-x86_64-libyaml
```

Run the build from PowerShell, not from the MSYS2 terminal:

```powershell
Set-Location C:\path\to\CAMotics
.\scripts\windows\build-msys2.ps1 -Jobs 16 -Msys2Root C:\msys64
```

`-CbangDir C:\path\to\cbang` overrides the dependency location.  The same
values can be supplied as `MSYS2_ROOT` and `CBANG_HOME`.  The script checks out
the pinned Cbang revision, builds all native programs, runs `windeployqt`,
copies the UCRT64 DLL closure, and executes packaged `camsim.exe --version`.

## Reproducible release inputs

Release scripts honor `SOURCE_DATE_EPOCH`.  Set it to the commit timestamp:

```sh
export SOURCE_DATE_EPOCH="$(git show -s --format=%ct HEAD)"
```

Linux packages:

```sh
scripts/release/build-linux-packages.sh
```

Windows portable ZIP and installer, after the MSYS2 bundle is built:

```powershell
.\scripts\windows\build-release.ps1 -BundleDir C:\path\to\bundle
```

Published package builders write checksums and `BUILDINFO.json` next to the
artifacts.  Release binaries are not built with `-march=native`; they target a
portable x86-64 baseline.  Local `-march=native` experiments are not suitable
for redistribution.

## Cleaning and troubleshooting

Clean objects when changing compiler, sanitizer, or Cbang configuration:

```sh
scons -c with_gui=1 with_tpl=0
rm -rf .sconf_temp .sconsign.dblite config.log
```

Common problems:

- `Cbang headers or libraries not found`: set `CBANG_HOME` to the pinned Cbang
  checkout and build it first.
- `QT5DIR variable is not defined`: if Qt is otherwise detected through
  `pkg-config`, this message is harmless.  Set `QT5DIR` only when Qt is in a
  nonstandard prefix.
- Windows Qt platform plugin error: verify that `platforms/qwindows.dll` is in
  the portable bundle and start the packaged executable, not the build-tree
  executable copied by itself.
- WSL window opens but renders through `llvmpipe`: use the native Windows build
  for interactive work.
- Build stops on warnings from an older system dependency: use the documented
  toolchain first.  `strict=0` is a diagnostic workaround, not a release
  configuration.
