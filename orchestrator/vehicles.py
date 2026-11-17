#!/usr/bin/env python3
"""Registry of vehicle types an agent can be (`agents[].vehicle` in the experiment YAML; default `x500`).

A vehicle type is described once per physics simulator, because the two change independently:

  PX4 side        which airframe (PX4_SYS_AUTOSTART) and model name (PX4_SIM_MODEL) the PX4 SITL in the agent container runs.
  Simulator side  which asset the simulator spawns for it. For Gazebo that is the model directory PX4 spawns itself
                  (`PX4_SIM_MODEL` without its `gz_` prefix), named `<model>_<px4_instance>` in the world.

`kind` is what the control code switches on (a multirotor takes off and flies 3-D setpoints; a rover drives 2-D position
setpoints and never leaves the ground). It does not depend on the simulator.

Only `gazebo` entries exist, plus one `isaac` entry (`iris`, spike). The airframes below (`4001_gz_x500`, `4009_gz_r1_rover`) are PX4's Gazebo-bridge airframes; another
simulator needs its own `autostart`/`sim_model` and asset reference under its own key (docs/CONTAINERS.md, "Adding a physics
simulator"). Asking for a vehicle on a simulator with no entry stops with a message instead of guessing.
"""
DEFAULT = "x500"

VEHICLES = {
    "x500": {
        "kind": "multirotor",
        "gazebo": {"autostart": "4001", "sim_model": "gz_x500", "model": "x500"},
    },
    # 3DR Iris as shipped with Pegasus Simulator (Isaac Sim extension, multirotors only). PX4 runs the generic external-simulator airframe
    # 10016_none_iris (PX4_SIM_MODEL=none_iris) and reaches Isaac Sim over the simulator MAVLink interface (TCP 4560 + px4_instance), not gz-transport.
    # `asset` names the Pegasus ROBOTS key. Isaac Sim spike: experimental, one vehicle, no rover (docs/CONTAINERS.md).
    "iris": {
        "kind": "multirotor",
        "isaac": {"sim_model": "none_iris", "asset": "Iris"},
    },
    # RC Cessna: PX4 airframe 4003_gz_rc_cessna (fixed-wing) and PX4's `rc_cessna` Gazebo model, both in the pinned PX4 commit (px4/Dockerfile).
    # A fixed-wing cannot hover: it takes off along its spawn heading and circles (loiters) instead of holding a point, so it needs its own flight code.
    "rc_cessna": {
        "kind": "fixedwing",
        # `px4_params` are set at PX4 start-up through PX4's own PX4_PARAM_<name> environment override (rcS in the pinned commit). The airframe's default
        # NAV_DLL_ACT=2 fails the preflight check with "No connection to the GCS" (observed 2026-11-10): there is no ground station in this stack. 0 disables that
        # failsafe, as px4/Dockerfile already does for the x500 airframe.
        # NAV_LOITER_RAD is the radius of the circle PX4 flies in hold mode (airframe default 80 m); `loiter_radius_m` is the same number for the planner
        # (orchestrator/mesh_plan.py). 50 m needs about 30 degrees of bank at the 16.7 m/s ground speed seen in the first flights (atan(v^2 / (g r))).
        "loiter_radius_m": 50.0,
        "gazebo": {"autostart": "4003", "sim_model": "gz_rc_cessna", "model": "rc_cessna", "px4_params": {"NAV_DLL_ACT": "0", "NAV_LOITER_RAD": "50"}},
    },
    # Aion Robotics R1 rover: PX4 airframe 4009_gz_r1_rover (differential drive) and PX4's `r1_rover` Gazebo model, both in the
    # pinned PX4 commit (px4/Dockerfile). The airframe sets the rover's tuning; the real vehicle's own parameters will differ.
    "r1_rover": {
        "kind": "rover",
        "gazebo": {"autostart": "4009", "sim_model": "gz_r1_rover", "model": "r1_rover"},
    },
}


def name(agent):
    n = agent.get("vehicle", DEFAULT)
    if n not in VEHICLES:
        raise SystemExit(f"agent {agent['name']}: unknown vehicle '{n}'. Available: {', '.join(sorted(VEHICLES))}")
    return n


def kind(agent):
    return VEHICLES[name(agent)]["kind"]


def for_simulator(agent, simulator_type):
    """The PX4 and asset settings of this agent's vehicle on `simulator_type`."""
    n = name(agent)
    v = VEHICLES[n].get(simulator_type)
    if v is None:
        have = sorted(k for k in VEHICLES[n] if k != "kind")
        raise SystemExit(f"agent {agent['name']}: vehicle '{n}' has no definition for simulator '{simulator_type}' (it has: {', '.join(have)}). "
                         "Add one in orchestrator/vehicles.py.")
    return v


def check(cfg):
    """Fail before anything starts when an agent's vehicle and its Gazebo model name disagree. The pose bridge follows the model named
    in `px4_gz_model`; PX4 spawns `<model>_<px4_instance>`, so a mismatch would run the flight with ns-3 never seeing the vehicle move."""
    sim = cfg["simulator"]["type"]
    for a in cfg["agents"]:
        v = for_simulator(a, sim)
        if a.get("camera") and sim != "isaac":
            raise SystemExit(f"agent {a['name']}: `camera` is only implemented for simulator.type isaac (this is {sim})")
        if a.get("camera"):
            for key in ("width", "height", "rate_hz"):
                val = a["camera"].get(key)
                if not isinstance(val, (int, float)) or isinstance(val, bool) or val <= 0:
                    raise SystemExit(f"agent {a['name']}: `camera.{key}` must be a positive number (got {val!r}); see docs/CONFIGURATION.md, `camera`")
        if sim == "gazebo":
            want = f"{v['model']}_{a['px4_instance']}"
            if a.get("px4_gz_model") != want:
                raise SystemExit(f"agent {a['name']}: vehicle '{name(a)}' is spawned as '{want}' but px4_gz_model is '{a.get('px4_gz_model')}'")
