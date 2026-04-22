# Cube-fall benchmarks

Measure Isaac Sim performance under five configurations to locate the source of slowdowns — in particular, whether the PX4 MAVLink 250 Hz lockstep loop is the bottleneck the team suspects.

Each script times a 0.5 m cube falling 10 m onto a ground plane, plus a 5 s steady-state sample after landing. The cube is a fixed "probe" workload; the drone (scenarios 2–5) is the load under test.

## Scripts

| # | Script | Pegasus | Scene | Drone backend |
|---|---|---|---|---|
| 1 | `1_cube_no_pegasus.py` | No | default ground plane | — |
| 2 | `2_cube_pegasus_flat_no_px4.py` | Yes | Flat Plane | Python (in-process) |
| 3 | `3_cube_pegasus_flat_px4.py` | Yes | Flat Plane | PX4 MAVLink (250 Hz lockstep) |
| 4 | `4_cube_pegasus_complex_no_px4.py` | Yes | Full Warehouse | Python (in-process) |
| 5 | `5_cube_pegasus_complex_px4.py` | Yes | Full Warehouse | PX4 MAVLink (250 Hz lockstep) |
| 6 | `6_cube_no_pegasus_complex.py` | No | Full Warehouse | — |
| 7 | `7_cube_no_pegasus_250hz.py` | No (physics_dt=1/250) | default ground plane | — |
| 8 | `8_cube_no_pegasus_complex_250hz.py` | No (physics_dt=1/250) | Full Warehouse | — |

Scripts 1 and 6 use Isaac's default `World()` (physics_dt ≈ 1/60). Scripts 2–5 use Pegasus's `px4` world settings (physics_dt = 1/250). Scripts 7 and 8 are copies of 1 and 6 configured with physics_dt = 1/250 so they're directly comparable to the Pegasus scripts.

Pairing for interpretation:

- **1 vs 6** isolates **complex-scene cost** at 60 Hz physics.
- **7 vs 8** isolates **complex-scene cost** at 250 Hz physics.
- **1 vs 7** and **6 vs 8** isolate the **cost of 250 Hz physics** itself (no Pegasus either side).
- **7 vs 2** isolates **Pegasus + drone cost** on a flat scene at matched physics_dt.
- **8 vs 4** isolates **Pegasus + drone cost** on Full Warehouse at matched physics_dt.
- **2 vs 3** and **4 vs 5** isolate the **PX4 MAVLink cost** (the team's primary hypothesis).

## Running

Single script:

```bash
isaac_run benchmarking/1_cube_no_pegasus.py            # headless (default)
isaac_run benchmarking/1_cube_no_pegasus.py --no-headless
```

All five + aggregate plot:

```bash
python3 benchmarking/run_all.py                        # run all 5, print table, write summary.png
python3 benchmarking/run_all.py --scripts 1,3,5        # subset
python3 benchmarking/run_all.py --skip-run             # re-plot from existing JSONs
python3 benchmarking/run_all.py --output custom.png    # custom output path
```

`run_all.py` invokes each script via `$ISAACSIM_PYTHON` (falls back to `$ISAACSIM_PATH/python.sh`, then `~/isaacsim/python.sh`).

## Metrics

Headline metric: **real-time factor (RTF) = simulated_s / wallclock_s** (RTF > 1 = faster than real time).

Each run writes `results/<script_stem>.json` with:
- `startup_sim_app_s` — `SimulationApp(...)` construction
- `startup_world_and_scene_s` — imports, World, scene/drone/cube, `world.reset()`
- `startup_total_s` — sum
- `fall_*` — wallclock, simulated seconds, steps, RTF during the ~1.4 s fall
- `steady_*` — same breakdown over 5 s of simulated time after landing (more reliable sample)
- `rolling_rtf` — 1 s-window RTF samples during steady-state (reveals jitter from e.g. PX4 lockstep stalls)
- `physics_dt`, `rendering_dt`, `config`

`run_all.py` additionally writes `results/summary.json` and `results/summary.png` (2×2 figure: startup breakdown, steady RTF, fall vs steady RTF, rolling-RTF jitter).

## Caveats

- First SimulationApp launch after a reboot includes shader compile / asset cache warm-up — second-run numbers are more representative.
- `--no-headless` adds rendering overhead; only compare headless-to-headless across scripts.
- Scripts 3 and 5 require PX4 installed at `pg.px4_path` (see `pegasus.simulator/config/configs.yaml`); they fail early if it isn't. After these exit, confirm with `ps aux | grep px4` that PX4 has shut down.
- A single run per script gives one sample — run `run_all.py` 3–5 times to eyeball variance (each invocation overwrites the JSON; `--skip-run` replots the latest).
