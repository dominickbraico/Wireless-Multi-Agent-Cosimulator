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
        if sim == "gazebo":
            want = f"{v['model']}_{a['px4_instance']}"
            if a.get("px4_gz_model") != want:
                raise SystemExit(f"agent {a['name']}: vehicle '{name(a)}' is spawned as '{want}' but px4_gz_model is '{a.get('px4_gz_model')}'")
