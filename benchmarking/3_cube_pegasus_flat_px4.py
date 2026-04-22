#!/usr/bin/env python
"""Benchmark 3: cube fall with Pegasus loaded, Flat Plane scene, drone with PX4 MAVLink backend.

Same as benchmark 2 but the Iris drone uses PX4MavlinkBackend with px4_autolaunch=True.
This activates the 250 Hz lockstep MAVLink loop that the team suspects is the
bottleneck. PX4 is launched from pg.px4_path and shuts down when the script exits.

Caveats:
  - First SimulationApp launch after a reboot includes shader compile / asset cache
    warm-up; second-run numbers are more representative.
  - --no-headless adds rendering overhead; only compare headless-to-headless across scripts.
  - Script will fail at startup if PX4 is not installed at pg.px4_path (see configs.yaml).

Run:
  ./python.sh benchmarking/3_cube_pegasus_flat_px4.py            # headless (default)
  ./python.sh benchmarking/3_cube_pegasus_flat_px4.py --no-headless
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
from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)

timeline = omni.timeline.get_timeline_interface()

pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world

pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])

multirotor_config = MultirotorConfig()
mavlink_config = PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,
})
multirotor_config.backends = [PX4MavlinkBackend(mavlink_config)]
Multirotor(
    "/World/quadrotor",
    ROBOTS["Iris"],
    0,
    [2.0, 2.0, 0.07],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=multirotor_config,
)

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
        "pegasus": True,
        "scene": "Flat Plane",
        "drone_backend": "px4_mavlink",
        "headless": args.headless,
    },
    timer=timer,
    runtime=runtime,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

simulation_app.close()
