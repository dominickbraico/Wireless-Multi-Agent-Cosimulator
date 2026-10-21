# Mesh-dropout scenario

A second control stack, separate from the leader-follower flight, whose purpose is to show links between agents **dropping and recovering**. One agent flies a seeded
random path that includes an excursion beyond radio range; the others hover. Every agent broadcasts a heartbeat on the emulated radio and logs which neighbours it hears,
so a dropout is visible three ways: the modelled signal in ns-3, the live map, and the agents' own message reception. Runs in the Docker deployment
([CONTAINERS.md](CONTAINERS.md)); nothing of the leader-follower stack is changed or reused except PX4's offboard base class and the vehicle-state reader.

## Run it

```
python3 orchestrator/cosim.py build                                  # once (images: docs/CONTAINERS.md)
tests/mesh_plan.sh                                                   # offline checks of the flight plan, seconds, no Docker
tests/containers_mesh_dropout.sh --config config/experiments/four_agent_mesh_dropout.yaml            # flat world, headless, with pass/fail checks
demo/run_container_demo.sh --config config/experiments/four_agent_mesh_dropout_baylands.yaml          # Baylands, Gazebo GUI + live signal map
MESH_SEED=11 tests/containers_mesh_dropout.sh                         # another seed without editing the YAML
```

The demo picks the flight from the YAML's `scenario.type`. About 2.5 minutes per run. Outputs in the log directory: `mesh_dropout.png` (modelled signal and heartbeat
events over time, distances, paths), `comm_map_dropout.png` (the signal map before, at the deepest dropout, and after), `mesh_report.json`, `mesh_plan.json` (the plan flown),
`mesh/` (per-agent heartbeat logs) and the usual ns-3 and flight logs.

## What happens

- **Layout** (`four_agent_mesh_dropout*.yaml`): an equilateral triangle with 10 m sides, `agent1` to `agent3` at its corners, `agent4` at its centroid. `agent4` is the wanderer,
  the other three are anchors that take off and hover at their spawn points (distinct altitudes: agent *i* flies at 8 m + 3 m x *i*, so paths cannot collide).
- **Wanderer path** (`orchestrator/mesh_plan.py`, a pure function of the YAML including the seed, so every container computes the same plan): 3 random points inside a 20 m
  radius (every link stays up there), an outbound point on the excursion heading, the **excursion point** where it hovers 10 s, back through the outbound point, 3 more random points,
  home, land. The excursion point is chosen so that exactly *k* anchor links (*k* = 1 or 2, drawn from the seed) end more than `margin_m` (3 m) beyond the dropout range *D* and
  the other links more than `margin_m` inside it. *D* is where the YAML's own signal model reaches the receiver floor (about 51.4 m for the default; [SIGNAL_MODEL.md](SIGNAL_MODEL.md)), in 3-D.
- **Return:** the wanderer ends at its spawn point and lands; the anchors land once every wanderer has landed.
- **Heartbeat** (`ros2_ws/src/mesh_control`, `mesh_msgs/PeerState`): each agent publishes id, sequence number, send time, position and velocity at 5 Hz on `/mesh/peer_state`
  (domain 42, over `radio0`) and subscribes to everyone's. A neighbour not heard for `link_timeout_s` (1.5 s) is **LOST**; heard again, **UP**. The radio firewall and the isolation checks are
  unchanged: only domain-42 traffic crosses the channel.

## YAML (`scenario`, see also [CONFIGURATION.md](CONFIGURATION.md))

`scenario.type: mesh_dropout` selects the stack; leader-follower remains the default when the key is absent. Keys under `scenario.mesh_dropout`, all optional:
`seed`, `wanderers` (default: the last agent), `altitude_base_m`, `altitude_step_m`, `speed_mps` (3.0), `random_waypoints` (3, before and after), `roam_radius_m` (20),
`excursion_links` (`[1, 2]`), `margin_m` (3), `dwell_s` (10), `heading_deg` (fixed heading) or `heading_sectors_deg` (draw the heading only from these `[from, to]` arcs, ENU degrees),
`heartbeat_hz` (5), `link_timeout_s` (1.5). The heartbeat QoS is `swarm.qos.peer_state` (default best_effort, volatile, **depth 10**). The plan is rejected with a message if it
cannot be built (unknown agent, a margin that no direction allows, ...).

## Pass criteria (fixed before the first run; `tools/analyze_mesh_dropout.py`)

| Group | Criterion |
|---|---|
| Flights | every agent reached its altitude, landed and disarmed; ended within 2.0 m (horizontal) of its spawn point |
| Modelled | links to the planned dropped anchors go below the receiver floor and later recover; every other link never goes below it (from ns-3's `pair_rssi.csv`) |
| Measured | in both directions of each dropped link: LOST then UP; LOST within [-1.0 s, `link_timeout_s` + 2.0 s] of the modelled drop; UP within [-1.0 s, 10 s] of the modelled recovery; every LOST event of the run falls where the model has the signal below the floor |
| Validity | ns-3 lag within `time.max_lag_ms`; Gazebo real-time factor over 5 s at least `time.min_rtf` from take-off ([ARCHITECTURE.md](ARCHITECTURE.md), "Run validity") |
| Isolation | no domain-0, XRCE or PX4-topic traffic on the channel; the swarm domain and the mesh topic are present |

## Results (single runs, headless unless stated; 2026-10-21)

| Run | Config, seed | Planned drops | Detected after modelled drop | Back after modelled recovery | Checks |
|---|---|---|---|---|---|
| Flat world | `four_agent_mesh_dropout.yaml`, 7 | agent4 - agent1, agent4 - agent3 (excursion 53 m) | +1.0 to +1.5 s | -0.1 to 0.0 s | all pass; ns-3 lag max 0.229 ms; RTF worst window 0.997 |
| Baylands | `four_agent_mesh_dropout_baylands.yaml`, 7 | agent4 - agent2, agent4 - agent3 (excursion 52.8 m, heading 223 degrees) | +0.9 to +1.4 s | 0.0 to +0.5 s | all pass; lag max 0.182 ms; RTF 1.000 |
| Baylands, GUI + live map (`demo/run_container_demo.sh`) | same, 7 | same | +1.3 to +1.5 s | -0.1 to +0.3 s | 32 of 32; RTF worst window 0.992; GUI and map did not crash |

Heartbeat latency while links were up: median about 1.2 ms, 99th percentile about 2 ms (flat run, 7879 heartbeats). These are single runs on one machine; they are not statistics.
`tests/mesh_plan.sh` passes for both configs: over 40 seeds every plan is repeatable, every excursion drops exactly *k* links with at least the margin, and (Baylands) every heading lies in the allowed arcs.

## Choices and findings worth knowing

- **Heartbeat history depth.** `PeerState` has no key, so a subscriber keeps one shared keep-last history for all senders. With depth 1 the first run saw agent4 miss 1 to 3 s of agent1's heartbeats
  throughout the flight, at 6 m range (the earliest sender in each burst was overwritten by the later ones), which looked like random link loss. With depth 10 the only LOST events were the planned
  dropouts. A topic with several publishers needs a history deeper than the number of senders.
- **Discovery speed.** `config/dds/cyclonedds_mesh.xml` sets the participant announcement interval to 1 s (Cyclone's default is 30 s). With the default, a neighbour out of range for longer than the
  10 s lease would only be found again at its next announcement, up to 30 s after the signal returned. The shorter interval means the measured recovery follows the radio; it also sends about 30 times
  more discovery announcements, so it is used only by this scenario. Recovery times in the table are with this setting and are not what an unmodified DDS would show.
- **Detection is delayed by design**: LOST fires `link_timeout_s` after the last heartbeat, so about 1 to 1.5 s after the signal falls below the floor.
- **Baylands terrain.** The world has structures (trees, buildings) up to roughly 15 to 25 m high at headings 300 to 340 degrees and about 14 m at 120 to 170 degrees within 75 m of the spawn area; the wanderer
  flies at 17 m. The arcs in `heading_sectors_deg` are the headings whose ground and objects stay below 5 m, found by reading the heights from the world's mesh files (accuracy about 1 m, measured against the
  spawn height, not against Gazebo's collision geometry). The plan flew without incident in the runs above; a different world needs its own check.
- **Flat-ground and constant altitude.** Flight altitude is constant per agent and the planner's distances assume the same flat reference as the rest of the framework.
- **Shutdown artefact.** When the harness stops ns-3 after the last landing every link "drops"; the analyzer ignores events after the last agent has landed.

## Limits

Only one wanderer was run (more are supported by the planner and untested). The scenario uses the Wi-Fi model of the YAML unchanged, so the dropout range is the one set by the log-distance model and the receiver
floor, not a measured one. Not run: other seeds in the containers (the planner is checked for 40, but a flight was run only for seed 7), a wanderer in a different world, native (non-Docker) execution.
