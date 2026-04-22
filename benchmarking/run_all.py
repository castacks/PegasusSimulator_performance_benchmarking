#!/usr/bin/env python3
"""Orchestrator: run all cube-fall benchmarks and plot the results.

Runs each benchmarking/<N>_*.py script as a subprocess via Isaac Sim's python.sh
(sequentially to avoid GPU/CPU contention), collects their per-script JSON
outputs, and produces:

  benchmarking/results/summary.json   - aggregated metrics
  benchmarking/results/summary.png    - 2x2 comparison figure

Invocation:
  python3 benchmarking/run_all.py                       # run all 5 and plot
  python3 benchmarking/run_all.py --scripts 1,3,5       # run a subset
  python3 benchmarking/run_all.py --skip-run            # re-plot existing JSONs
  python3 benchmarking/run_all.py --output path.png     # custom output

Environment:
  ISAACSIM_PYTHON is used to invoke each benchmark (set in .bashrc / .zshrc).
  Falls back to $ISAACSIM_PATH/python.sh or ~/isaacsim/python.sh.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

BENCH_DIR = Path(__file__).resolve().parent
RESULTS_DIR = BENCH_DIR / "results"

SCRIPTS = {
    1: "1_cube_no_pegasus.py",
    2: "2_cube_pegasus_flat_no_px4.py",
    3: "3_cube_pegasus_flat_px4.py",
    4: "4_cube_pegasus_complex_no_px4.py",
    5: "5_cube_pegasus_complex_px4.py",
    6: "6_cube_no_pegasus_complex.py",
    7: "7_cube_no_pegasus_250hz.py",
    8: "8_cube_no_pegasus_complex_250hz.py",
}


def _find_isaac_python() -> str:
    env = os.environ.get("ISAACSIM_PYTHON")
    if env and Path(env).is_file():
        return env
    base = os.environ.get("ISAACSIM_PATH") or str(Path.home() / "isaacsim")
    candidate = Path(base) / "python.sh"
    if candidate.is_file():
        return str(candidate)
    raise FileNotFoundError(
        "Could not locate Isaac Sim's python.sh. Set ISAACSIM_PYTHON or ISAACSIM_PATH."
    )


def _run_one(script_num: int, isaac_python: str, extra_args: list[str]) -> bool:
    script = BENCH_DIR / SCRIPTS[script_num]
    print(f"\n[run_all] >>> running {script.name}", flush=True)
    t0 = time.perf_counter()
    proc = subprocess.run([isaac_python, str(script), *extra_args])
    elapsed = time.perf_counter() - t0
    if proc.returncode != 0:
        print(f"[run_all] !!! {script.name} exited {proc.returncode} after {elapsed:.1f}s", flush=True)
        return False
    print(f"[run_all] <<< {script.name} ok ({elapsed:.1f}s wall)", flush=True)
    return True


def _load_results(script_nums: list[int]) -> dict[int, dict]:
    results: dict[int, dict] = {}
    for n in script_nums:
        stem = Path(SCRIPTS[n]).stem
        path = RESULTS_DIR / f"{stem}.json"
        if not path.is_file():
            print(f"[run_all] warn: missing {path}")
            continue
        with open(path) as f:
            results[n] = json.load(f)
    return results


def _print_summary_table(results: dict[int, dict]) -> None:
    print("\n" + "=" * 96)
    print(f"{'#':<3} {'script':<40} {'startup_total_s':>15} {'fall_rtf':>10} {'steady_rtf':>12}")
    print("-" * 96)
    for n in sorted(results.keys()):
        r = results[n]
        print(
            f"{n:<3} {r['script']:<40} "
            f"{r['startup_total_s']:>15.3f} "
            f"{r['fall_rtf']:>10.3f} "
            f"{r['steady_rtf']:>12.3f}"
        )
    print("=" * 96)


def _plot(results: dict[int, dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    nums = sorted(results.keys())
    labels = [f"{n}\n{results[n]['config']['scene'][:14]}" for n in nums]

    def _color(n: int) -> str:
        cfg = results[n]["config"]
        if not cfg.get("pegasus"):
            return "#1f77b4"  # blue: baseline
        if cfg.get("drone_backend") == "px4_mavlink":
            return "#d62728"  # red: PX4
        return "#2ca02c"      # green: no PX4

    colors = [_color(n) for n in nums]
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Isaac Sim + Pegasus cube-fall benchmark", fontsize=14, fontweight="bold")

    # Top-left: startup time breakdown (stacked)
    ax = axes[0, 0]
    sim_app = [results[n]["startup_sim_app_s"] for n in nums]
    world = [results[n]["startup_world_and_scene_s"] for n in nums]
    x = np.arange(len(nums))
    ax.bar(x, sim_app, color="#7f7f7f", label="SimulationApp")
    ax.bar(x, world, bottom=sim_app, color="#bcbd22", label="World + scene")
    for i, n in enumerate(nums):
        ax.text(i, sim_app[i] + world[i], f"{sim_app[i] + world[i]:.1f}s",
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("seconds")
    ax.set_title("Startup time")
    ax.legend(loc="upper left")
    ax.grid(axis="y", alpha=0.3)

    # Top-right: steady-state RTF
    ax = axes[0, 1]
    steady = [results[n]["steady_rtf"] for n in nums]
    bars = ax.bar(x, steady, color=colors)
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1, alpha=0.6, label="real-time (1.0)")
    for bar, v in zip(bars, steady):
        ax.text(bar.get_x() + bar.get_width() / 2, v, f"{v:.2f}",
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("RTF = simulated_s / wall_s")
    ax.set_title("Steady-state real-time factor (headline metric)")
    ax.legend(loc="upper right")
    ax.grid(axis="y", alpha=0.3)

    # Bottom-left: fall RTF vs steady RTF grouped
    ax = axes[1, 0]
    fall = [results[n]["fall_rtf"] for n in nums]
    w = 0.38
    ax.bar(x - w/2, fall, w, color="#ff7f0e", label="fall (transient)")
    ax.bar(x + w/2, steady, w, color=colors, label="steady")
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1, alpha=0.6)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("RTF")
    ax.set_title("Fall RTF vs steady-state RTF")
    ax.legend(loc="upper right")
    ax.grid(axis="y", alpha=0.3)

    # Bottom-right: rolling RTF over simulated time
    ax = axes[1, 1]
    for n in nums:
        series = results[n].get("rolling_rtf", [])
        if not series:
            continue
        xs = [s["sim_time_s"] for s in series]
        ys = [s["rtf"] for s in series]
        ax.plot(xs, ys, marker="o", linewidth=1.5, color=_color(n),
                label=f"{n}: {results[n]['config']['scene'][:18]}")
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1, alpha=0.6)
    ax.set_xlabel("simulated time since landing (s)")
    ax.set_ylabel("rolling RTF (1 s windows)")
    ax.set_title("Steady-state RTF jitter")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout(rect=(0, 0, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output, dpi=120)
    print(f"[run_all] wrote {output}")


def _parse_scripts(arg: str | None) -> list[int]:
    if not arg:
        return sorted(SCRIPTS.keys())
    nums: list[int] = []
    for tok in arg.split(","):
        tok = tok.strip()
        if not tok:
            continue
        n = int(tok)
        if n not in SCRIPTS:
            raise SystemExit(f"Unknown script number: {n}. Valid: {sorted(SCRIPTS)}")
        nums.append(n)
    return nums


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scripts", default=None, help="Comma-separated script numbers (default: all)")
    parser.add_argument("--skip-run", action="store_true", help="Skip execution, just re-plot existing JSONs")
    parser.add_argument("--output", default=str(RESULTS_DIR / "summary.png"), help="Output PNG path")
    parser.add_argument("--no-headless", dest="headless", action="store_false", default=True,
                        help="Pass --no-headless through to each benchmark (default: headless)")
    args = parser.parse_args()

    nums = _parse_scripts(args.scripts)

    if not args.skip_run:
        isaac_python = _find_isaac_python()
        print(f"[run_all] using {isaac_python}")
        extra = [] if args.headless else ["--no-headless"]
        failures: list[int] = []
        for i, n in enumerate(nums):
            ok = _run_one(n, isaac_python, extra)
            if not ok:
                failures.append(n)
            if i < len(nums) - 1:
                time.sleep(2)
        if failures:
            print(f"[run_all] warning: {len(failures)}/{len(nums)} script(s) failed: {failures}")

    results = _load_results(nums)
    if not results:
        print("[run_all] no results to plot")
        return 1

    _print_summary_table(results)

    summary_json = RESULTS_DIR / "summary.json"
    summary_json.parent.mkdir(parents=True, exist_ok=True)
    with open(summary_json, "w") as f:
        json.dump({str(k): v for k, v in results.items()}, f, indent=2)
    print(f"[run_all] wrote {summary_json}")

    _plot(results, Path(args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
