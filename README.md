# CAMotics Fast

![CAMotics logo](images/camotics-logo.png)

CAMotics Fast is a performance-focused fork of
[CAMotics](https://github.com/CauldronDevelopmentLLC/CAMotics), the open-source
3-axis G-code simulator created by Joseph Coffland and its contributors.  It
keeps CAMotics' full marching-cubes simulator as the correctness reference and
adds faster paths for the jobs that can be handled without changing the
meaning of the cut.

Accuracy comes before speed.  Every accelerated backend has explicit
eligibility checks, reports why it was rejected, and falls back to full
marching cubes when its assumptions do not hold.

## Status

| Feature | Status | Notes |
| --- | --- | --- |
| Full marching cubes | Stable reference | Default CLI backend and fallback |
| ToolSweep spatial index | Stable | Exact candidate pruning; no geometry change |
| Z-dexel simulation | Experimental | Fast 2.5D cutting for supported 3-axis jobs |
| Sparse surface extraction | Experimental | Marches toolpath-adjacent regions and stitches analytic stock |
| Safe mesh reduction | Advanced opt-in | Applies only after deviation and topology checks |
| Checkpointed GUI playback | Experimental | Exact retained Dexel replay with coalesced display updates |
| Live Dexel height map | Experimental | 8-bit grayscale view and CLI PNG export |

## Downloads

Stable Windows and Linux builds are attached to the
[latest release](https://github.com/davronthemighty/CAMotics/releases/latest).
Windows executables are not code-signed and may trigger a SmartScreen warning.

Release files include a portable Windows ZIP, a Windows installer, a Linux
AppImage, a Debian package, checksums, and build information.  The Debian
package is named `camotics-fast` and deliberately conflicts with the upstream
`camotics` package because both install the same commands and desktop files.

## Quick build

The normal public build omits the legacy TPL/V8 subsystem:

```sh
git clone https://github.com/davronthemighty/CAMotics.git
cd CAMotics
git clone https://github.com/CauldronDevelopmentLLC/cbang.git build-deps/cbang
git -C build-deps/cbang checkout 62bd9aa11938236ac3f1568e8bfdeaa160c14eac
scons -C build-deps/cbang -j"$(nproc)" with_openssl=0
CBANG_HOME="$PWD/build-deps/cbang" scons -j"$(nproc)" with_tpl=0
./camsim --version
```

See [BUILDING.md](BUILDING.md) for Linux, WSL, headless, GUI, and MSYS2
instructions.

## Using the accelerated paths

Run the full reference simulation in the usual way:

```sh
./camsim --threads 16 project.camotics result.stl
```

Try the Dexel backend, retaining automatic fallback:

```sh
./camsim --dexel --threads 16 --profile run.json \
  project.camotics result.stl
```

Inspect a job before rendering:

```sh
./camsim --perf-advice project.camotics
./camsim --dexel-eligibility-only --profile eligibility.json project.camotics
```

The accelerated options and their limits are documented in:

- [Accelerated simulation](ACCELERATED_SIMULATION.md)
- [Dexel simulation](DEXEL_SIMULATION.md)
- [Sparse surface extraction](SPARSE_SURFACE_EXTRACTION.md)
- [Safe mesh reduction](SAFE_MESH_REDUCTION.md)
- [GUI playback](GUI_PLAYBACK.md)
- [Benchmarks and reproducibility](BENCHMARKS.md)

## Important limits

- CAMotics is a stock-removal preview, not a machine-dynamics or collision
  certification system.
- The Dexel backend is a single-height 2.5D model.  It cannot represent
  undercuts, caves, or separate vertical intervals in one XY column.
- Rotary and auxiliary-axis motion is rejected by Dexel simulation.
- Machine profiles describe geometry, travel, and published operating limits;
  they do not model backlash, runout, flex, servo following error, or cutting
  forces.
- Simulation is not a substitute for reviewing G-code, workholding, tools,
  feeds, limits, and clearance on the real machine.

## Project lineage and license

CAMotics Fast is distributed under the GNU General Public License, version 2
or later.  See [COPYING](COPYING).  Existing CAMotics copyrights and
attribution are retained; new fork code is also attributed to
`davronthemighty` in its source headers.

AI-assisted development tools were used during this work. The released code and documentation were reviewed and validated by the maintainer.
