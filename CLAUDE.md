# rescue-bird

Drone software stack + edge-SoC sizing tool. Two purposes side-by-side:
**measure** workload via instrumented SITL (ROS 2 + Gazebo + PX4) and
**project** workload via an analytical model (33 sliders → 15 KPIs). The
goal is producing defensible numbers for an edge-SoC partitioning
conversation (NPU TOPS, ISP/DSP/VPU sizing, memory bandwidth, BF16 vs
FP16, LLM headroom).

## Read these first

- `DESIGN.md` — high-level architecture map
- `docs/decisions/` — 11 ADRs capturing the *why* behind key choices.
  Read at minimum 001 (two-source model), 003 (asymmetric multi-camera),
  009 (NPU efficiency factor — the most-likely-wrong constant), and
  011 (test harness).
- `docs/testing.md` — six layers of testing strategy
- `docs/whatif_kpis.md` — slider catalog and worked examples

## Current focus

Calibrating `npu_efficiency_factor` (currently 0.55, see ADR 009) against
real measurements. The next concrete task: replace the `_StubModel` in
`ros2_ws/src/drone_perception/drone_perception/perception_node.py` with a
TensorRT-converted EdgeTAM, run `test_cruise_straight` and
`test_aerobatic_forest`, compare measured vs projected perception
latency, refine the constant in
`instrumentation/analysis/whatif/workload_model.py`.

## Commands

```bash
# Run all tests (92 unit + 10 golden, should all pass)
pytest tests/ -v

# Regenerate golden snapshots after intentional model changes
pytest tests/golden --update-goldens

# Point evaluation: does the default chip pass at default workload?
python -m instrumentation.analysis.whatif.whatif_cli point

# 1D sweep over a slider
python -m instrumentation.analysis.whatif.whatif_cli sweep --slider npu_tops_bf16 --steps 5

# 2D Pareto across two sliders
python -m instrumentation.analysis.whatif.whatif_cli pareto --x npu_tops_bf16 --y memory_channels

# Run a SITL mission (requires Docker + GPU)
./scripts/run_mission.sh test_aerobatic_forest --target rescue_bird_a720
```

## Architecture pointers

- `instrumentation/analysis/whatif/` — pure-Python analytical model
- `instrumentation/analysis/profiles/*.yaml` — SoC profiles (a720, imx95, snapdragon)
- `instrumentation/probes/` — Parquet-emitting probes (op, GPU, NVENC, g2g)
- `instrumentation/trajectories/` — deterministic flight profile generators
- `instrumentation/pilots/` — measurement-only pilot flyability model
- `ros2_ws/src/drone_*/` — per-subsystem ROS 2 nodes (each maps to a silicon engine)
- `missions/test_*.yaml` — the four trajectory-driven test scenarios

## Gotchas

- **Cross-platform line endings**: this repo expects LF. If editing on
  Windows, set `git config core.autocrlf false` and `git config
  core.fileMode false` in your local clone before committing.
- **Don't add a `requirements.txt`** — dependencies are listed in
  `pyproject.toml`. Use `pip install -e .` for dev install.
- **Stub models in perception/radar nodes are deliberate** — they exist
  so the SITL plumbing can be validated before real model weights are
  wired in. When swapping in real models, keep the stub path as a
  fallback for CI.
- **ROS 2 hop latency overstates real silicon by 5-10×** (see ADR 002).
  Subtract DDS overhead before reporting on-chip latency in the
  competitive deck.