# CAMotics Fast v2026.08.0

This is the first CAMotics Fast release.  It is based on CAMotics and retains
full marching cubes as the reference simulator and fallback.

Highlights:

- opt-in Z-dexel simulation for supported 3-axis stock removal;
- exact retained playback checkpoints and live Dexel height maps;
- ToolSweep spatial indexing for the full marching-cubes path;
- experimental sparse surface extraction with analytic stock stitching;
- topology-checked planar mesh reduction with explicit rejection accounting;
- improved Tool Table, reference frames, persistent playback status, tool
  filtering, End action, and trackpad navigation;
- Carvera Air visualization and published-envelope profile;
- portable Windows and Linux packages with public geometry gates.

Read the backend limitations before using accelerated output in production.
Dexel simulation cannot represent undercuts or multiple solid intervals in one
XY column.  Machine profiles do not model backlash, flex, runout, controller
following error, or cutting forces.

Windows executables are unsigned and may trigger Microsoft SmartScreen.  The
release includes SHA-256 checksums and GitHub build-provenance attestations,
but no Authenticode signature.
