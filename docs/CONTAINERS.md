# Docker deployment

The whole stack can run under Docker Compose: **one container per agent, one for ns-3, one for the physics simulator.**
It needs only the Docker daemon: no `sudo`, no host network namespaces, no ROS 2, PX4, Gazebo or ns-3 on the host.

```
python3 orchestrator/cosim.py build                                             # once: build the images
tests/containers_static_link.sh                                                 # network only (two agents)
tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi.yaml    # full flight, five agents
```

Tested with Docker 29.7 and Compose 5.5 on Ubuntu 24.04. The native deployment (network namespaces on the host, see
[SETUP.md](SETUP.md)) still works and is unchanged; both read the same experiment YAML.

## Live demo: Gazebo GUI and live signal map

```
demo/run_container_demo.sh [--config YAML] [--no-gui] [--no-map] [--no-markers] [--show-plan] [--halos] [--no-prompt]      # default config: five_agent_wifi.yaml
```

The simulation runs in the containers; two windows open on the host: the **Gazebo GUI**, attached to the simulator container, and a **live 2D map**
(`tools/plot_comm_map.py --live`) that redraws about twice a second from what ns-3 is logging, with every agent pair coloured dark green (strong) to red (weak); pairs below the receiver floor have no line, and the figure shows each radio's transmit power and where the signal reaches the dropout floor. The signal of each pair is read from
ns-3 itself (`ns3/pair_rssi.csv`: the scenario evaluates its own propagation loss chain for every ordered pair on the current node positions), so it follows any
change of layout, agent count, transmit power or loss model in the YAML without touching the tool.
The script starts the stack and holds the vehicles on the ground, opens the windows and frames an overview camera, and flies when you press Enter
(`--no-prompt`: after 5 s). No sudo. It needs a desktop session, matplotlib with Tk (`python3-tk`), and Gazebo Harmonic and PX4's models on the host for the GUI
(see [SETUP.md](SETUP.md)); `--no-gui` and `--no-map` drop either window.
The GUI also shows a trail that grows behind each vehicle as it flies, in that agent's colour (`demo/path_markers.py --trace`, which follows `ns3/pose_applied.csv`, the poses ns-3 applied; one leg per metre moved). The same colour is used for the agent's node and track on the signal map (`orchestrator/agent_colors.py`: the first categorical slots of the dataviz palette in YAML order: blue, orange, aqua, yellow, magenta, ...). `--show-plan` adds each agent's planned path (`mesh_dropout` configs), `--halos` a translucent yellow sphere on each vehicle, `--no-markers` omits all of it. These are Gazebo GUI markers: drawn on the host GUI only, not part of the simulated world, absent from physics and sensors. They stay in that GUI until it closes (the trail is cleared when the next run starts drawing).

How the GUI reaches the container: the host's `gz` joins the `cosim` Gazebo partition from the Docker bridge's gateway address on `mgmt` (`10.20.0.254` by default,
`mgmt.gateway` in `containers.py`). The agents' management firewall ignores it (only the simulator is allowed), and the GUI is not part of the experiment.

Known effect: attaching the GUI makes the simulator pause briefly (about 0.4 s in the runs below) while it sends the scene. That happens on the ground before
take-off. The real-time-factor check therefore measures from the `go` signal (take-off) to the end of the poses; the whole-run figure, including the pause, is
printed as a `note` line but is not part of the criterion (see `docs/EXPERIMENTS.md`, "Run validity"). A pause or slowdown *during* the flight still fails the check.

## Containers

| Container | Image | Runs | Count |
|---|---|---|---|
| `simulator` | `cosim-sim-gazebo` | Gazebo Harmonic server (headless) and the pose adapter that feeds ground-truth positions to ns-3 | 1 |
| `ns3` | `cosim-ns3` | The ns-3 scenario for the chosen network technology, a bridge and a TAP per agent | 1 |
| `agent1` ... `agentN` | `cosim-agent` | PX4 SITL, Micro XRCE-DDS Agent, the swarm node and the flight node | one per `agents:` entry |

`orchestrator/compose_gen.py` writes the Compose project from the experiment YAML (`<log dir>/compose/docker-compose.yml`), so
adding an agent to the YAML adds a container. The generated file is plain Compose and can be used directly:
`docker compose -f <log dir>/compose/docker-compose.yml ps`. Do not edit it; change the YAML and regenerate.

## How the containers communicate

There are exactly three kinds of connection. Anything not listed here does not exist.

```
   ┌─────────────┐   pose feed (UDP/JSON)    ┌────────────┐
   │  simulator  ├──────────────────────────►│    ns3     │      mgmt network (not emulated):
   │ Gazebo+pose │        mgmt               │   ns-3 +   │      simulator, ns3 and every agent
   │   adapter   │                           │ bridge/TAP │
   └──────▲──────┘                           │ per agent  │
          │ gz-transport (mgmt)              └──┬──────┬──┘
          │                         link-agent1 │      │ link-agent2     private networks, one per agent:
   ┌──────┴──────┐ ┌─────────────┐              │      │                 radio frames only, emulated by ns-3
   │   agent1    ├─┤  (radio0)   ├──────────────┘      │
   │ PX4, ROS 2  │ └─────────────┘                     │
   └─────────────┘                                     │
   ┌─────────────┐ ┌─────────────┐                     │
   │   agent2    ├─┤  (radio0)   ├─────────────────────┘     ... and so on for every agent
   │ PX4, ROS 2  │ └─────────────┘
   └─────────────┘
   Every agent also reaches the simulator over gz-transport on mgmt. No agent has any path to another agent except through ns3.
```

| Connection | Network | Carries | Emulated |
|---|---|---|---|
| agent ↔ ns3 | `link-<agent>`, one private network per agent (only that agent and ns3 are on it) | Ethernet frames of the agent's `radio0` | **Yes**: the frames enter ns-3 through a TAP and leave through the ghost node's 802.11 device |
| agent ↔ simulator | `mgmt` | gz-transport (PX4 to Gazebo: sensors, motor commands, model spawn) | No |
| simulator → ns3 | `mgmt` | Pose feed: UDP/JSON datagrams to `ns3:5600` | No |

- **Agent to agent.** An agent has no network path to another agent except through ns-3. Each agent sits on its own
  `link-<agent>` network, so a frame from agent 1 can only reach the `ns3` container, where it is bridged to a TAP and handed to ns-3.
  The same Ethernet frames come out of ns-3 on the receiving agent's link network, with the MAC address preserved.
  Every agent is also on `mgmt`, but an nftables policy inside each agent (`cosim_mgmt`) drops all management traffic to and from anything but
  the simulator. Its counters show the attempts it blocked; the tests check them.
- **Inside the `ns3` container** each link interface is joined to a Linux bridge together with a TAP device (`tap-<agent>`), which the ns-3
  TapBridge opens, exactly as `wire.py` does on the host. The source-MAC guard (an nftables rule that lets only that agent's `radio0` MAC
  reach its TAP) is installed here too; the MAC of every `radio0` is derived from the agent's position in the YAML so that both sides agree.
- **`radio0` is renamed inside the agent.** Docker numbers interfaces in no guaranteed order, so `container_net.py` finds the radio interface
  by its address, then renames it to `radio0`, sets the planned MAC and the YAML's `radio_ip`, and installs the radio firewall
  (domain-42 swarm ports only). The swarm DDS domain, the PX4 domain and every other process see the same `radio0` as in the native setup.
- **Addressing.** All of it comes from the YAML through `orchestrator/containers.py`: `network.mgmt.subnet`, each agent's `mgmt_ip` and
  `radio_ip`, and the optional `containers:` section ([CONFIGURATION.md](CONFIGURATION.md)). The simulator takes `network.mgmt.bridge_ip`.
  Link networks come from a block (default `10.30.0.0/16`, one /24 per agent). Change it if it collides with an existing Docker network.
- **Logs and synchronisation.** The run's log directory is mounted at `/logs` in every container; file names are those of the native scripts, so
  `tools/analyze_formation.py` reads a container run unchanged. The agents wait for a file `/logs/go` before flying; the test driver creates it
  once every agent reports PX4 ready (Compose healthchecks) and the simulator, ns-3 and the agents are up.

## Should the physics simulator be in its own container?

Yes, and it is. The reasons are specific to this project:

1. **It is the part that changes.** Everything else (ns-3, ROS 2, PX4, the swarm protocol) is simulator-independent. Gazebo and Isaac Sim need different
   base images, Python versions and GPU runtimes; putting the simulator behind a container boundary keeps those out of the agents and out of ns-3.
2. **Its requirements differ.** Isaac Sim needs the NVIDIA container runtime and a specific CUDA/driver stack; the agents need none of it.
   A separate container means only that one service asks for `gpus:`.
3. **The interface is already narrow.** The rest of the system touches the simulator through two things only: how PX4 talks to it, and the pose feed to ns-3.
   Both already cross the `mgmt` network, so moving the simulator into a container changed no protocol.
4. **It is shared, not per agent.** One world holds every agent's vehicle, so it is one container, not one per agent.

What is *not* separated: the pose adapter runs inside the simulator container, because it must subscribe to the simulator's own API. The
simulator container is therefore "physics engine + its pose adapter", and its contract with the rest of the system is below.

## Adding a physics simulator (Isaac Sim)

Only Gazebo has a container driver today. A new simulator needs these four pieces; nothing in ns-3, the agents' ROS 2 code or the swarm protocol changes.

| Piece | What to provide |
|---|---|
| **Image** | `docker/sim-<name>.Dockerfile`, with whatever the simulator needs (for Isaac Sim the NVIDIA base image) |
| **Registry entry** | One entry in `SIMULATORS` in `orchestrator/simulators.py`, keyed by `simulator.type` in the YAML |
| **Pose adapter** | A process in the simulator container that sends the pose feed (below) |
| **Vehicle link** | The environment PX4 needs to attach to this simulator, returned by the entry's `px4_env()` |

The registry entry has four functions: `world()` (which world or stage to load), `service()` (extra Compose settings, such as `gpus`, the
health check, environment), `px4_env()` (how each agent's PX4 attaches) and `processes()` (what the simulator container runs).

**The pose feed contract** (the whole interface to ns-3). One UDP datagram to `network.pose.listen_addr:port` (the `ns3` container) at
`network.pose.rate_hz`, repeating the last pose when nothing changed:

```json
{"seq": 123, "t_sim": 45.678, "t_wall": 1790000000.123, "agents": {"agent1": [x, y, z], "agent2": [x, y, z]}}
```

Positions are the antenna position in metres in the simulator's world frame, taken as East-North-Up with the same origin ns-3 uses (the pose
bridge adds `antenna_offset`). `t_sim` is simulation time, `t_wall` the sender's wall clock (ns-3 reports staleness from it). `adapters/gazebo/pose_bridge.py`
is the reference; the encoding is isolated in its `encode()`.

**The vehicle link** is simulator-specific and is the part that needs thought. Gazebo's gz-transport reaches PX4 directly over `mgmt`
(`PX4_GZ_STANDALONE`). Isaac Sim does not use gz-transport. A likely route (not verified here) is PX4's MAVLink simulator interface (lockstep over TCP, one port
per instance) as used by Isaac Sim extensions such as Pegasus Simulator, which would need a different PX4 start-up (a non-Gazebo `PX4_SIM_MODEL`
and the simulator's TCP port) in `px4_env()`. The agent containers, the swarm node and ns-3 are unaffected. This has not been implemented or tested.

Steps: add the Dockerfile, add the registry entry (and the `px4_env()` that attaches PX4), write the adapter so it satisfies the pose contract, set
`simulator.type: isaac` in a copy of an experiment YAML, then run `python3 orchestrator/cosim.py build` and a test.
Two limitations to plan for: the `agents[].px4_gz_model` field names a Gazebo model, so another simulator needs its own mapping from scene object to agent name
(the adapter can do this itself and only emit agent names); and Gazebo-specific names in `simulators.py` (`px4_flat`, `x500`) are the Gazebo driver's
business only.

## Changing the networking technology

"Network technology" is chosen at two levels, both from the experiment YAML. Neither requires editing the orchestrator.

### 1. Change the Wi-Fi standard, rates or receiver model (YAML only)

`network.wifi` selects the 802.11 flavour. Supported values of `standard`: `80211a` (default), `80211g`, `80211n`, `80211ac`, `80211ax`.
`band` (`5GHZ` or `2_4GHZ`; default `5GHZ`, `2_4GHZ` for `80211g`) selects the band. `data_mode` and `control_mode` are passed to ns-3 unchanged, so they must
belong to the standard:

| `standard` | `data_mode` example | `control_mode` example |
|---|---|---|
| `80211a` | `OfdmRate12Mbps` | `OfdmRate6Mbps` |
| `80211g` | `ErpOfdmRate12Mbps` | `ErpOfdmRate6Mbps` |
| `80211n` | `HtMcs4` | `OfdmRate6Mbps` |
| `80211ac` | `VhtMcs4` | `OfdmRate6Mbps` |
| `80211ax` | `HeMcs4` | `OfdmRate6Mbps` |

Also revisit `network.channel.loss`: the 1 m reference loss depends on the frequency (about 46.7 dB at 5.2 GHz, 40.0 dB at 2.4 GHz). Examples:
`config/experiments/two_agent_wifi_80211g.yaml` and `two_agent_wifi_80211n.yaml` differ from `two_agent_wifi.yaml` only in those lines. Run it:

```
tests/containers_static_link.sh --config config/experiments/two_agent_wifi_80211n.yaml
```

The test checks that ns-3 reports the standard and rate the YAML asked for. The 802.11g and 802.11n examples were run this way (and 802.11a).
`ac` and `ax` are accepted by the scenario but have not been run. `tools/link_budget.py` predicts range for 802.11a only.

### 2. Add another technology (new ns-3 scenario)

A technology is an ns-3 scenario program plus one registry line. `network.technology` in the YAML names it (default `wifi`).

1. **Write the scenario** under `ns3/scenarios/<name>/` (copy `wifi_adhoc_tap` as the template). To plug into the rest of the system it must:
   - take `--config=<yaml> --log-dir=<dir> --duration=<s>` and read its parameters from `network.<section>` of the YAML;
   - create one ghost node per entry of `agents:` (in YAML order) and attach one `TapBridge` per node, in `UseLocal` mode, to the TAP named
     by that agent's `tap:`; give the nodes **no** IP stack, since addresses live on the Linux side;
   - use ns-3's real-time simulator (`RealtimeSimulatorImpl`) and enable checksums;
   - apply positions from the pose feed (UDP/JSON to `network.pose`; the listener in `wifi_adhoc_tap.cc` can be reused as is);
   - write `pair_rssi.csv` (`t_s,tx,rx,distance_m,rssi_dbm,below_floor`: the power a frame from `tx` would reach `rx` with, from the technology's own loss model, per
     `network.link_log_interval_s`) if you want the signal map to work; without it the tool falls back to a LogDistance estimate from the YAML and says so on the figure;
   - write `summary.json` (`ns3_max_lag_ms`, `valid_ns3_lag`) and `lag.csv` (`t_s,lag_ms`, one row per simulated second) into `--log-dir`, and stop cleanly on SIGTERM.
     The `ns3` container is healthy once `lag.csv` has a data row.
2. **Build it into the image.** Scenarios under `ns3/scenarios/*/` are copied into the image and built; add the new target to the `ns3 build` line
   in `docker/ns3.Dockerfile` and symlink its binary under `/opt/cosim/scenarios/<name>` as that file does for `wifi_adhoc_tap`. For native runs add it to
   `ns3/build_ns3.sh`. If the technology needs ns-3 modules not yet enabled (for example `lte`), add them to `--enable-modules` in the Dockerfile.
3. **Register it** in `orchestrator/technologies.py`:
   ```python
   "lorawan": {"scenario": "lorawan_tap", "config_section": "lorawan", "summary": "..."},
   ```
4. **Select it** in a copy of an experiment YAML:
   ```yaml
   network:
     technology: lorawan
     lorawan: {...}          # whatever the scenario reads
   ```
   `network.wifi` is only required when the technology is `wifi`. The agents' own `radio_ip`, TAP and bridge names, the isolation and the swarm protocol are unchanged.

Notes: the radio firewall allows only the swarm DDS UDP ports, which stays correct for any technology that carries Ethernet frames. A technology
that cannot carry multicast or large frames will change how the swarm DDS domain behaves; the tests report that. `ns3/patches/0001-...` is specific to
Wi-Fi devices.

## Scenarios

The agent containers run the leader-follower flight by default. `scenario.type: mesh_dropout` in the experiment YAML swaps it for the mesh-dropout demonstration (agents flying seeded random paths whose links drop and recover, with a heartbeat on the radio): `orchestrator/scenario_mesh.py` is called from `agent_main.py` after PX4 is ready, and nothing else in the stack changes. Run it with
`tests/containers_mesh_dropout.sh` or `demo/run_container_demo.sh --config config/experiments/four_agent_mesh_dropout_baylands.yaml`; see [MESH_DROPOUT.md](MESH_DROPOUT.md).

## Worlds, including custom ones and worlds that download models

A world is `worlds/<name>.sdf`. The file name must equal the `<world name="...">` inside it (the pose topic and the health check use
that name). Select it with `simulator.px4_world_name` in the YAML or `--world <name>` / `PX4_WORLD=<name>`. `worlds/` is mounted into the
simulator container, so a new world needs no image rebuild.

Models the world uses can come from three places:

| Source | How it works |
|---|---|
| PX4's Gazebo models (`x500`, ...) | Baked into the simulator image at build time. |
| Your own models | Put them in `worlds/models/<model name>/` (with `model.config` and `model.sdf`) and refer to them as `model://<model name>`. That directory is on the simulator's resource path. |
| Gazebo Fuel (a world whose `<uri>` is `https://fuel.gazebosim.org/...`, such as `baylands`) | The stack has no internet, so the models are downloaded once into Gazebo's own cache on the host, `~/.gz/fuel`, which is mounted read-only into the simulator at the same absolute path. |

Fuel models are handled automatically. `cosim.py up` checks whether the cache holds the models for the chosen world; if not, it starts a
throw-away container with internet access, loads the world once (so models that other models depend on are fetched too) and records a
marker that is tied to the hash of the `.sdf`. You can do this ahead of time with `python3 orchestrator/cosim.py fetch-world baylands`
(or `python3 orchestrator/fuel.py status baylands` to see what is cached).

- Cache location: `~/.gz/fuel`, the cache native Gazebo uses, owned by you. The simulator container mounts it at the same absolute path and runs with your
  `HOME`. This matters for the GUI: the server tells a connected GUI the absolute file paths of the meshes and textures it resolved, so those paths must exist on the host
  too. With the cache inside the container as `/root/.gz/fuel` the host GUI crashed on `Permission denied` (observed 2026-10-21). `baylands` takes about 410 MB.
- `COSIM_NO_FETCH=1` forbids downloading and makes `up` fail with the command to run instead (offline or locked-down runs).
- Each run writes `fuel_models.txt` in its log directory with the world file's hash and the cached version of each model it names. Fuel serves
  the latest version for a URI that names none, so a cache filled later can differ from an earlier one. The list covers the models the
  world names directly, not the ones those models depend on.
- Editing the `.sdf` invalidates the marker, and the next `up` fetches again.

Baylands needs a config with matching spawn heights, because its terrain rises with y:

```
demo/run_container_demo.sh --config config/experiments/five_agent_wifi_baylands.yaml --world baylands
PX4_WORLD=baylands tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi_baylands.yaml
```

Tested (2026-10-21, single runs, `five_agent_wifi_baylands.yaml`, offline simulator container): headless, all checks passed (formation error p95 1.90 to 1.94 m,
swarm loss 0 %, ns-3 lag max 0.148 ms, RTF worst 5-s window 0.993). With the host Gazebo GUI and the live map attached (`demo/run_container_demo.sh` as above,
three runs after the path fix), 25 of 25 checks passed in each (RTF worst window 0.981 to 0.998); the GUI listed `park`, `coast_waves` and `x500_0` to `x500_4` and the map
updated during the flight (screenshot taken mid-flight). The GUI's 3-D view was not inspected for texture quality. A world with a model in `worlds/models/` loaded and showed
that model in `gz model --list`; no flight was run on it.

## Operations

- `python3 orchestrator/cosim.py build [service...]` builds the images (`docker compose build` on a generated project). First build: ns-3 compiles (a few
  minutes on 16 cores); the agent and simulator images are built on the PX4 runtime image (`docker build -t cosim-px4-runtime px4/`, see
  [px4/README.md](../px4/README.md)) and share its layers, so they add little disk space. The base is large (about 12.5 GB on disk) because it keeps the PX4 build tree.
  Override it with `docker compose build --build-arg PX4_RUNTIME_IMAGE=<image>`.
- `python3 orchestrator/cosim.py up --config YAML --out DIR [--mode flight|net]` starts a stack, `down --out DIR` removes it and hands `DIR` back to you
  (the containers write as root). The tests call these and tear down on exit and on interrupt (checked with SIGTERM).
- After an interrupted run: `python3 orchestrator/cosim.py down --out <the run's log dir>`, or `docker compose -p cosim-<experiment name> down`.
- `tests/containers_leader_follower.sh` honours `SIDE`, `ALT`, `SPEED`, `OFFSET`, `PX4_WORLD` and `OUT` like its native counterpart.

## Limitations

- The Gazebo GUI is not inside the stack (the containers are headless); `demo/run_container_demo.sh` attaches the host's GUI to it (see above).
- Worlds that reference Fuel models need a one-time download on the host (see "Worlds"); the default world, `px4_flat`, needs none.
- PX4's Gazebo models are fetched from the `PX4-gazebo-models` repository's main branch when the simulator image is built; they are not pinned.
- Containers run as root and keep `NET_ADMIN` (needed to wire the interfaces; the simulator container uses it for one multicast route).
- The host's Docker bridge for each link network also receives the frames on that link. The source-MAC guard keeps host-originated frames (IPv6 neighbour
  discovery, IGMP) off the emulated channel.
- The agents' flight DDS path still uses `radio0`'s address over loopback ([ARCHITECTURE.md](ARCHITECTURE.md), Limitations). That limitation is documented
  for the native setup; it has not been tested what stopping the `ns3` container mid-flight does to the agents.
- Isaac Sim is a documented extension, not an implemented one.
