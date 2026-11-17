# Docker deployment

How to run the whole stack under Docker Compose, with one container per agent, one for ns-3 and one for the physics
simulator. This document also describes how to add a physics simulator, a vehicle, a network technology or a world.

## Contents

- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Operations](#operations)
- [Live demo](#live-demo)
- [Containers](#containers)
- [Container networks](#container-networks)
- [Physics simulators](#physics-simulators)
- [Isaac Sim (experimental)](#isaac-sim-experimental)
- [Vehicle types](#vehicle-types)
- [Network technology](#network-technology)
- [Scenarios](#scenarios)
- [Worlds](#worlds)
- [Results](#results)
- [Limitations](#limitations)

## Requirements

The host needs only the Docker daemon with Compose. It needs no `sudo`, ROS 2, PX4, Gazebo or ns-3.

Tested with Docker 29.7 and Compose 5.5 on Ubuntu 24.04. The native deployment in [SETUP.md](SETUP.md) reads the same
experiment YAML and is unchanged.

## Quick start

Build the images once. The first command compiles PX4; the second compiles ns-3:

```bash
docker build -t cosim-px4-runtime px4/
python3 orchestrator/cosim.py build
```

Run the network-only test with two agents, then a flight with five agents:

```bash
tests/containers_static_link.sh
tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi.yaml
```

## Operations

| Command | Effect |
|---|---|
| `python3 orchestrator/cosim.py build [service...]` | Builds the images |
| `python3 orchestrator/cosim.py up --config YAML --out DIR [--mode flight\|net]` | Starts a stack that logs to `DIR` |
| `python3 orchestrator/cosim.py down --out DIR` | Removes the stack and returns ownership of `DIR` to the user |
| `python3 orchestrator/cosim.py fetch-world NAME` | Downloads the Fuel models of a world ahead of time |
| `docker compose -p cosim-<experiment name> down` | Removes a stack when the log directory is unknown |

- The tests call `up` and `down` themselves and tear the stack down on exit and on interrupt.
- After an interrupted run, call `down` with that run's log directory.
- The agent and simulator images build on `cosim-px4-runtime` and share its layers. That base is about 12.5 GB because
  it keeps the PX4 build tree. Override it with `docker compose build --build-arg PX4_RUNTIME_IMAGE=<image>`.
- Rebuild the agent image after a change under `orchestrator/` or `ros2_ws/`.
- `tests/containers_leader_follower.sh` honours `SIDE`, `ALT`, `SPEED`, `OFFSET`, `PX4_WORLD` and `OUT`, like the native
  test.

## Live demo

Run a flight with the Gazebo GUI and a live signal map on the host. The default configuration is
`five_agent_wifi.yaml`:

```bash
demo/run_container_demo.sh [--config YAML] [--no-gui] [--no-map] [--no-markers] [--show-plan] [--halos] [--no-prompt]
```

The simulation runs in the containers. The script starts the stack, holds the vehicles on the ground, opens two
windows, frames an overview camera, and starts the flight when Enter is pressed.

| Window | Content |
|---|---|
| Gazebo GUI | The host's `gz` attached to the simulator container |
| Live 2D map | `tools/plot_comm_map.py --live`: every agent pair coloured from dark green (strong) to red (weak) |

| Flag | Effect |
|---|---|
| `--no-gui`, `--no-map` | Omits that window |
| `--no-prompt` | Starts the flight after 5 s without waiting for Enter |
| `--show-plan` | Draws each agent's planned path (`mesh_dropout` configurations) |
| `--halos` | Draws a translucent sphere on each vehicle |
| `--no-markers` | Omits trails, plans and halos |

Requirements on the host: a desktop session, `matplotlib` with Tk (`python3-tk`), Gazebo Harmonic and PX4's Gazebo
models. See [SETUP.md](SETUP.md).

Details:

- The map redraws about twice a second from `ns3/pair_rssi.csv`, which ns-3 computes from its own loss chain. It
  follows any change of layout, agent count, transmit power or loss model in the YAML. Pairs below the receiver floor
  have no line. See [SIGNAL_MODEL.md](SIGNAL_MODEL.md).
- The GUI draws a trail behind each vehicle in that agent's colour (`demo/path_markers.py --trace`, from the poses ns-3
  applied). `orchestrator/agent_colors.py` assigns the colours in YAML order. The map uses the same colour for the
  agent's node.
- Trails, plans and halos are Gazebo GUI markers. They exist only in the host GUI and are absent from physics and
  sensors.
- The host's `gz` joins the `cosim` Gazebo partition from the Docker bridge's gateway address on `mgmt` (`10.20.0.254`
  by default). The agents' management firewall ignores that address.
- Attaching the GUI pauses the simulator briefly on the ground (about 0.4 s in the runs in [Results](#results)). The
  real-time-factor check measures from take-off, so the pause is reported as a `note` and is not part of the criterion.
  A pause during the flight still fails the check.

## Containers

| Container | Image | Runs | Count |
|---|---|---|---|
| `simulator` | `cosim-sim-gazebo` | Gazebo Harmonic server (headless) and the pose adapter | 1 |
| `ns3` | `cosim-ns3` | The ns-3 scenario of the chosen network technology, and a bridge and a TAP per agent | 1 |
| `agent1` ... `agentN` | `cosim-agent` | PX4 SITL, Micro XRCE-DDS Agent, the radio node and the flight node | One per `agents` entry |

`orchestrator/compose_gen.py` writes the Compose project from the experiment YAML to
`<log dir>/compose/docker-compose.yml`. Adding an agent to the YAML adds a container. The generated file is plain
Compose. Do not edit it; change the YAML and regenerate.

The simulator has its own container because it is the part that changes. Gazebo and Isaac Sim need different base
images and GPU runtimes, and the rest of the system touches the simulator through two interfaces only: the PX4 link
and the pose feed. One world holds every vehicle, so there is one simulator container. The pose adapter runs inside
it, because it subscribes to the simulator's own API.

## Container networks

Three kinds of connection exist. No other path exists between containers.

```text
   +-------------+   pose feed (UDP/JSON)    +------------+
   |  simulator  +-------------------------->|    ns3     |      mgmt network (not emulated):
   | Gazebo+pose |        mgmt               |   ns-3 +   |      simulator, ns3 and every agent
   |   adapter   |                           | bridge/TAP |
   +------^------+                           | per agent  |
          | gz-transport (mgmt)              +--+------+--+
          |                         link-agent1 |      | link-agent2     private networks, one per agent:
   +------+------+                              |      |                 radio frames only, emulated by ns-3
   |   agent1    +---- radio0 ------------------+      |
   | PX4, ROS 2  |                                     |
   +-------------+                                     |
   +-------------+                                     |
   |   agent2    +---- radio0 -------------------------+
   | PX4, ROS 2  |
   +-------------+
```

| Connection | Network | Carries | Emulated |
|---|---|---|---|
| agent to ns3 | `link-<agent>`, one private network per agent | Ethernet frames of the agent's `radio0` | Yes |
| agent to simulator | `mgmt` | gz-transport: sensors, motor commands, model spawn | No |
| simulator to ns3 | `mgmt` | Pose feed: UDP/JSON datagrams to port 5600 | No |

| Topic | Description |
|---|---|
| Agent to agent | An agent reaches another agent only through ns-3. A frame on `link-<agent>` reaches only the `ns3` container, where a bridge hands it to a TAP. |
| Management isolation | Every agent is on `mgmt`, but the nftables table `cosim_mgmt` in each agent drops all management traffic except to and from the simulator. The tests check its counters. |
| Inside `ns3` | Each link interface joins a Linux bridge with a TAP device `tap-<agent>`, which the ns-3 TapBridge opens. The source-MAC guard is installed here. |
| MAC addresses | The MAC of each `radio0` derives from the agent's position in the YAML, so both sides agree. ns-3 preserves it end to end. |
| `radio0` | Docker numbers interfaces in no guaranteed order. `orchestrator/container_net.py` finds the radio interface by its address, renames it to `radio0`, sets the MAC and `radio_ip`, and installs the radio firewall. |
| Addressing | `orchestrator/containers.py` derives all addresses from the YAML. Link networks come from `containers.link_subnet_base` (default `10.30.0.0/16`, one /24 per agent). |
| Logs | The run's log directory is mounted at `/logs` in every container. File names match the native scripts, so the analysis tools read a container run unchanged. |
| Start signal | The agents wait for the file `/logs/go`. The test driver creates it once the simulator, ns-3 and every agent are healthy and every PX4 is ready. |

## Physics simulators

Gazebo has the supported container driver. Isaac Sim has an experimental one. A new simulator needs four pieces.
ns-3, the agents' ROS 2 code and the swarm protocol do not change.

| Piece | What to provide |
|---|---|
| Image | `docker/sim-<name>.Dockerfile` |
| Registry entry | One entry in `SIMULATORS` in `orchestrator/simulators.py`, keyed by `simulator.type` |
| Pose adapter | A process in the simulator container that sends the pose feed |
| Vehicle link | The environment PX4 needs to attach to the simulator, returned by the entry's `px4_env()` |

The registry entry has four functions:

| Function | Returns |
|---|---|
| `world()` | The world or stage to load |
| `service()` | Extra Compose settings, such as `gpus`, the health check and the environment |
| `px4_env()` | How each agent's PX4 attaches |
| `processes()` | What the simulator container runs |

### Pose feed contract

The pose feed is the whole interface to ns-3. The adapter sends one UDP datagram to `network.pose.listen_addr:port` at
`network.pose.rate_hz` and repeats the last pose when nothing changes:

```json
{"seq": 123, "t_sim": 45.678, "t_wall": 1790000000.123, "agents": {"agent1": [x, y, z], "agent2": [x, y, z]}}
```

| Field | Meaning |
|---|---|
| `seq` | Sequence number, rising by 1 |
| `t_sim` | Simulation time in seconds |
| `t_wall` | The sender's wall clock in seconds; ns-3 reports staleness from it |
| `agents` | Antenna position of each agent in metres, East-North-Up, in the origin ns-3 uses |

`adapters/gazebo/pose_bridge.py` is the reference implementation. Its `encode()` isolates the encoding, and it adds
`antenna_offset`.

### Vehicle link

The vehicle link is simulator-specific. Gazebo's gz-transport reaches PX4 directly over `mgmt` (`PX4_GZ_STANDALONE`).
The Isaac Sim driver uses PX4's MAVLink simulator interface as implemented by Pegasus Simulator: TCP, one port per
instance, 4560 + `px4_instance`.

### Steps

1. Add the Dockerfile.
2. Add the registry entry, including the `px4_env()` that attaches PX4.
3. Write the adapter so that it satisfies the pose feed contract.
4. Set `simulator.type` in a copy of an experiment YAML.
5. Run `python3 orchestrator/cosim.py build` and a test.

`agents[].px4_gz_model` names a Gazebo model. Another simulator needs its own mapping from scene object to agent name;
the adapter can hold it and emit only agent names.

## Isaac Sim (experimental)

`simulator.type: isaac` runs Isaac Sim 5.1.0 headless with Pegasus Simulator v5.1.0 (commit
`644da37e9d5268e5f9a34e78bdcfd57a8bab82b4`) in the simulator container. The pinned PX4 in each agent container
connects over the MAVLink simulator interface. The driver is a spike that establishes what Isaac Sim needs. It is not
a supported simulator.

| Implemented | Location |
|---|---|
| Driver and image | `orchestrator/simulators.py`, `docker/sim-isaac.Dockerfile` |
| Scene script, one vehicle type (`iris`) | `adapters/isaac_sim/vehicle_sim.py` |
| Pose feed, a thread inside the Isaac process | `adapters/isaac_sim/pose_feed.py` |
| Real-time factor from the pose feed | `tools/analyze_isaac.py`, `tools/timehealth.py` |
| Camera stream over the management network | `adapters/isaac_sim/camera_stream.py`, `orchestrator/camera_sink.py` |

Isaac runs are judged by `tools/analyze_isaac.py`, not `tools/analyze_formation.py`. The camera frames are raw, not a
ROS 2 topic. `COSIM_ISAAC_GUI=1`, set by `demo/run_container_demo.sh`, opens Isaac Sim's window for demonstrations. The
tests are listed as Isaac Sim stages A to F in [EXPERIMENTS.md](EXPERIMENTS.md).

### Requirements

Measured on the development machine: Ubuntu 24.04, RTX 5060 Ti, driver 580.178.04.

| Requirement | Detail |
|---|---|
| NVIDIA GPU and the NVIDIA Container Toolkit | `docker info` must list an `nvidia` runtime. The generator stops with a message when it is missing. |
| Base image `nvcr.io/nvidia/isaac-sim:5.1.0` | 22.9 GB unpacked. It needs an NVIDIA NGC account and `docker login nvcr.io`. |
| Licence acceptance | Set `COSIM_ACCEPT_ISAAC_EULA=Y` yourself for `cosim.py build`, `up` and the tests. Nothing sets it for you. |
| Privacy setting | `COSIM_ISAAC_PRIVACY_CONSENT` (default `N`) is passed to Isaac Sim as `PRIVACY_CONSENT`. |

With Docker Desktop installed, `docker context ls` may point at its separate engine instead of the host engine.

### Implementation facts

- PX4 attaches with `PX4_SIM_MODEL=none_iris` and `PX4_SIM_HOST_ADDR=<simulator IP>`. It connects out to TCP 4560 +
  `px4_instance`.
- Pegasus listens with `connection_ip` set to `0.0.0.0`. Its default, `localhost`, is unreachable from another
  container.
- The simulator container has no internet. Pegasus' `params.py` and Isaac's default ground plane both query NVIDIA's
  asset server at start-up and fail offline. The scene script answers the first lookup with an empty path and uses a
  fixed collision box as ground.
- Pegasus runs from `PYTHONPATH`. It is not installed with `pip install -e`, because its setup script edits Isaac Sim's
  kit application files.
- Some materials are hosted on NVIDIA's asset server and fail to compile offline. The scene script adds a light and four
  coloured posts when an agent has a camera.
- The first start compiles shaders, about 160 s to the ready state in one run. Nothing is cached between runs.

## Vehicle types

`agents[].vehicle` selects what each agent is. `orchestrator/vehicles.py` holds the registry. A vehicle type is defined
per physics simulator, because the PX4 airframe and the simulator asset change independently.

| Vehicle | Kind | Simulator | PX4 airframe |
|---|---|---|---|
| `x500` (default) | `multirotor` | Gazebo | `4001_gz_x500` |
| `r1_rover` | `rover` | Gazebo | `4009_gz_r1_rover` (Aion R1, differential drive) |
| `iris` | `multirotor` | Isaac Sim (experimental) | `none_iris` |
| `rc_cessna` (work in progress) | `fixedwing` | Gazebo | `4003_gz_rc_cessna` |

- Gazebo spawns a vehicle as `<model>_<px4_instance>`.
- Control code switches on `kind`, which does not depend on the simulator.
- The ns-3 side, the radio and the swarm DDS domain do not depend on the vehicle.
- `vehicles.check()` stops with a message for a vehicle that has no entry for the chosen simulator.

To add a vehicle, add one entry to `VEHICLES` per simulator that can run it, and control code for its `kind`.

| Scenario | Vehicles accepted |
|---|---|
| `leader_follower` | Multirotors only: it flies 3-D position setpoints |
| `mesh_dropout`, mode `roam` | Mixed: a multirotor runs `mesh_flight`, a rover runs `mesh_drive` |
| `rover_drive` | Rovers only |

## Network technology

The experiment YAML selects the network technology at two levels. Neither requires editing the orchestrator.

### Wi-Fi standard, rates and receiver model

`network.wifi.standard` selects the 802.11 standard and `band` selects the band. `data_mode` and `control_mode` pass to
ns-3 unchanged, so they must belong to the standard:

| `standard` | `data_mode` example | `control_mode` example |
|---|---|---|
| `80211a` | `OfdmRate12Mbps` | `OfdmRate6Mbps` |
| `80211g` | `ErpOfdmRate12Mbps` | `ErpOfdmRate6Mbps` |
| `80211n` | `HtMcs4` | `OfdmRate6Mbps` |
| `80211ac` | `VhtMcs4` | `OfdmRate6Mbps` |
| `80211ax` | `HeMcs4` | `OfdmRate6Mbps` |

Revisit `network.channel.loss` when the band changes. The 1 m reference loss depends on the frequency: about 46.7 dB at
5.2 GHz and 40.0 dB at 2.4 GHz.

Run the 802.11n example. The test checks that ns-3 reports the standard and rate the YAML asks for:

```bash
tests/containers_static_link.sh --config config/experiments/two_agent_wifi_80211n.yaml
```

### Another technology

A technology is an ns-3 scenario program plus one registry line. `network.technology` names it; the default is `wifi`.

1. Write the scenario under `ns3/scenarios/<name>/`, using `wifi_adhoc_tap` as the template. It must meet the
   requirements in the table below.
2. Build it into the image. Add the target to the `ns3 build` line in `docker/ns3.Dockerfile` and link its binary under
   `/opt/cosim/scenarios/<name>`. For native runs, add it to `ns3/build_ns3.sh`. Add any further ns-3 modules, for
   example `lte`, to `--enable-modules`.
3. Register it in `orchestrator/technologies.py`:

   ```python
   "lorawan": {"scenario": "lorawan_tap", "config_section": "lorawan", "summary": "..."},
   ```

4. Select it in a copy of an experiment YAML:

   ```yaml
   network:
     technology: lorawan
     lorawan: {...}          # whatever the scenario reads
   ```

| Requirement on the scenario | Detail |
|---|---|
| Arguments | Accept `--config=<yaml> --log-dir=<dir> --duration=<s>` and read parameters from `network.<section>` |
| Nodes | Create one ghost node per `agents` entry, in YAML order, with no IP stack |
| TapBridge | Attach one TapBridge per node, in `UseLocal` mode, to the TAP named by that agent's `tap` |
| Simulator | Use `RealtimeSimulatorImpl` and enable checksums |
| Positions | Apply positions from the pose feed; the listener in `wifi_adhoc_tap.cc` is reusable |
| Signal log | Write `pair_rssi.csv` (`t_s,tx,rx,distance_m,rssi_dbm,below_floor`) for the signal map |
| Health | Write `summary.json` (`ns3_max_lag_ms`, `valid_ns3_lag`) and `lag.csv` (`t_s,lag_ms`), and stop cleanly on SIGTERM |

Notes:

- The `ns3` container is healthy once `lag.csv` has a data row.
- Without `pair_rssi.csv`, the map falls back to a log-distance estimate from the YAML and says so on the figure.
- `network.wifi` is required only when the technology is `wifi`.
- The radio firewall allows only the swarm DDS UDP ports. That stays correct for any technology that carries Ethernet
  frames.
- A technology that cannot carry multicast or large frames changes how the swarm DDS domain behaves.
- `ns3/patches/0001-wifi-mac-sync-link-address-on-SetAddress.patch` is specific to Wi-Fi devices.

## Scenarios

The agent containers run the leader-follower flight by default. `scenario.type` in the experiment YAML selects another
scenario. `orchestrator/agent_main.py` starts PX4, waits until it is ready, and then runs the scenario; nothing else in
the stack changes.

| `scenario.type` | Driver | Test |
|---|---|---|
| `leader_follower` | `orchestrator/agent_main.py` | `tests/containers_leader_follower.sh` |
| `mesh_dropout` | `orchestrator/scenario_mesh.py` | `tests/containers_mesh_dropout.sh` |
| `rover_drive` | `orchestrator/scenario_rover.py` | `tests/containers_rover.sh` |

[MESH_DROPOUT.md](MESH_DROPOUT.md) describes the mesh-dropout scenario.

## Worlds

A world is `worlds/<name>.sdf`. The file name must equal the `<world name="...">` inside it, because the pose topic and
the health check use that name. Select a world with `simulator.px4_world_name`, `--world <name>` or `PX4_WORLD=<name>`.
`worlds/` is mounted into the simulator container, so a new world needs no image rebuild.

| Model source | How it works |
|---|---|
| PX4's Gazebo models (`x500`, ...) | Baked into the simulator image at build time |
| Your own models | Put them in `worlds/models/<model name>/` and refer to them as `model://<model name>` |
| Gazebo Fuel (a `<uri>` under `https://fuel.gazebosim.org/`) | Downloaded once into `~/.gz/fuel` on the host, which is mounted read-only into the simulator |

Fuel models are handled automatically. `cosim.py up` checks whether the cache holds the models of the chosen world.
If not, it starts a throw-away container with internet access, loads the world once and records a marker tied to the
hash of the `.sdf`.

Fetch a world ahead of time, or show what is cached:

```bash
python3 orchestrator/cosim.py fetch-world baylands
python3 orchestrator/fuel.py status baylands
```

- The cache is `~/.gz/fuel`, the cache native Gazebo uses. The simulator container mounts it at the same absolute path
  and runs with the user's `HOME`. A connected GUI receives absolute file paths from the server, so those paths must
  also exist on the host.
- `baylands` takes about 410 MB.
- `COSIM_NO_FETCH=1` forbids downloading. `up` then fails and prints the command to run.
- Each run writes `fuel_models.txt` to its log directory, with the hash of the world file and the cached version of each
  model the world names. Fuel serves the latest version for a URI that names none.
- Editing the `.sdf` invalidates the marker, and the next `up` fetches again.

Baylands needs a configuration with matching spawn heights, because its terrain rises with y:

```bash
demo/run_container_demo.sh --config config/experiments/five_agent_wifi_baylands.yaml --world baylands
PX4_WORLD=baylands tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi_baylands.yaml
```

## Results

Single runs on the development machine, `five_agent_wifi_baylands.yaml`, offline simulator container, 2026-10-21.
They are not statistics.

| Command | Runs | Outcome |
|---|---|---|
| `PX4_WORLD=baylands tests/containers_leader_follower.sh --config ...` (headless) | 1 | All checks pass. Formation error p95 1.90 m to 1.94 m, swarm loss 0 %, ns-3 lag at most 0.148 ms, worst 5 s real-time-factor window 0.993 |
| `demo/run_container_demo.sh --config ... --world baylands` (GUI and map) | 3 | 25 of 25 checks pass in each. Worst real-time-factor window 0.981 to 0.998 |

In the GUI runs, the GUI listed `park`, `coast_waves` and `x500_0` to `x500_4`, and the map updated during the flight.

Wi-Fi standards: the 802.11a, 802.11g and 802.11n examples pass `tests/containers_static_link.sh` (single runs).

## Limitations

- Isaac Sim is an experimental spike. Not implemented: sensors other than one camera, a ROS 2 camera publisher, pacing
  the simulation to wall time, rovers, NVIDIA environments.
- Pegasus Simulator supports multirotors only (its README and vehicle page, checked 2026-11-03), so no rover exists for
  Isaac Sim.
- Isaac Sim's vehicle link holds real time for one multirotor only. Four multirotors flew the mesh-dropout scenario in
  one run at a real-time factor of about 0.6, so that run is not valid. See [EXPERIMENTS.md](EXPERIMENTS.md).
- The fixed-wing `rc_cessna` in the mesh-dropout scenario is work in progress. It has no passing run and is not
  documented further.
- Not implemented: a rover leader-follower scenario, terrain-aware spawn heights, ground-level obstacle checks.
- 802.11ac and 802.11ax are accepted by the scenario and not tested. `tools/link_budget.py` predicts range for 802.11a
  only.
- The Gazebo GUI is not inside the stack. The host's GUI attaches to it. The texture quality of the 3-D view is not
  inspected.
- A world with a model in `worlds/models/` loads and lists that model. No flight is tested on such a world.
- `fuel_models.txt` covers the models a world names directly, not the models those depend on.
- PX4's Gazebo models are fetched from the main branch of the `PX4-gazebo-models` repository at image build. They are
  not pinned.
- Containers run as root and keep `NET_ADMIN`, which wiring the interfaces needs.
- The host's Docker bridge for each link network also receives the frames on that link. The source-MAC guard keeps
  host-originated frames off the emulated channel.
- The agents' flight DDS path uses the address of `radio0` over loopback. See [ARCHITECTURE.md](ARCHITECTURE.md). The
  effect of stopping the `ns3` container mid-flight is not tested.
