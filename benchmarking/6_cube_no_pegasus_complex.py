#!/usr/bin/env python
"""Benchmark 6: cube fall with pure Isaac Sim, Full Warehouse scene, no Pegasus.

Loads the same Full Warehouse USD used by scripts 4 and 5 directly via Isaac Sim's
nucleus API, without importing pegasus.simulator and without spawning a drone.
Paired with script 1, this isolates the cost of the complex scene itself from any
Pegasus extension overhead.

Caveats:
  - Uses default World() physics settings (physics_dt ~1/60), same as script 1.
    Scripts 4/5 use Pegasus's px4 world settings (physics_dt=1/250), so 6-vs-4
    and 6-vs-5 RTF comparisons are not apples-to-apples on step rate; 1-vs-6 is.
  - First SimulationApp launch after a reboot includes shader compile / asset cache
    warm-up; second-run numbers are more representative.
  - --no-headless adds rendering overhead; only compare headless-to-headless across scripts.

Run:
  ./python.sh benchmarking/6_cube_no_pegasus_complex.py            # headless (default)
  ./python.sh benchmarking/6_cube_no_pegasus_complex.py --no-headless
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.bench_timer import (
    BenchTimer,
    CUBE_SIZE,
    CUBE_SPAWN_Z,
    parse_common_args,
    report,
    run_cube_fall_and_steady,
    script_stem,
)

args = parse_common_args(__doc__)

timer = BenchTimer()
timer.start("startup_sim_app")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})
timer.stop("startup_sim_app")

timer.start("startup_world_and_scene")
import numpy as np
import omni.timeline
import isaacsim.storage.native as nucleus
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

timeline = omni.timeline.get_timeline_interface()
world = World()

assets_root = nucleus.get_assets_root_path()
if assets_root is None:
    raise RuntimeError("Could not resolve NVIDIA assets root via nucleus.get_assets_root_path()")
warehouse_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
layout_prim = world.stage.DefinePrim("/World/layout")
if not layout_prim.GetReferences().AddReference(warehouse_usd):
    raise RuntimeError(f"Failed to load USD: {warehouse_usd}")

cube = world.scene.add(DynamicCuboid(
    prim_path="/World/benchmark_cube",
    name="benchmark_cube",
    position=np.array([0.0, 0.0, CUBE_SPAWN_Z]),
    size=CUBE_SIZE,
    color=np.array([1.0, 0.0, 0.0]),
))
world.reset()
timer.stop("startup_world_and_scene")

physics_dt = world.get_physics_dt()
rendering_dt = world.get_rendering_dt()

timeline.play()
runtime = run_cube_fall_and_steady(
    world=world,
    cube=cube,
    physics_dt=physics_dt,
    render=True,
    is_running=simulation_app.is_running,
)
timeline.stop()

report(
    script_stem=script_stem(__file__),
    config={
        "pegasus": False,
        "scene": "Full Warehouse",
        "drone_backend": None,
        "headless": args.headless,
    },
    timer=timer,
    runtime=runtime,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

simulation_app.close()
