# Mesh-dropout scenario

A second control stack, separate from the leader-follower flight, whose purpose is to show links between agents **dropping and recovering**. The agents fly seeded random paths
and return to their spawn points; whenever two of them are far enough apart the link between them drops, and it comes back when they close up again. Every agent broadcasts a
heartbeat on the emulated radio and logs which neighbours it hears, so a dropout is visible three ways: the modelled signal in ns-3, the live map, and the agents' own message
reception. Runs in the Docker deployment ([CONTAINERS.md](CONTAINERS.md)); nothing of the leader-follower stack is changed or reused except PX4's offboard base class and the
vehicle-state reader.

Two modes (`scenario.mesh_dropout.mode`):

- **`roam` (default): every agent flies.** Four agents in a triangle (10 m sides, the fourth at the centroid), each on its own random path.
- **`excursion`: one agent flies, the others hover** (the first version of this scenario): a wanderer makes an excursion that drops exactly 1 or 2 links to the hovering anchors.

## Run it

```
python3 orchestrator/cosim.py build                                  # once (images: docs/CONTAINERS.md)
tests/mesh_plan.sh                                                   # offline checks of the flight plans, seconds, no Docker
tests/containers_mesh_dropout.sh --config config/experiments/four_agent_mesh_dropout.yaml            # flat world, headless, with pass/fail checks
demo/run_container_demo.sh --config config/experiments/four_agent_mesh_dropout_baylands.yaml          # Baylands, Gazebo GUI + live signal map
MESH_SEED=11 tests/containers_mesh_dropout.sh                         # another seed without editing the YAML
```

The demo picks the flight from the YAML's `scenario.type`. About 2.5 minutes per run. Outputs in the log directory: `mesh_dropout.png` (modelled signal and heartbeat events over time,
distances, paths), `comm_map_dropout.png` (the signal map before, at the deepest dropout, and after), `mesh_report.json`, `mesh_plan.json` (the plan flown), `mesh/` (per-agent heartbeat
logs) and the usual ns-3 and flight logs.

## What happens in `roam` mode

- **Plan** (`orchestrator/mesh_plan.py`, a pure function of the YAML including the seed, so every container computes the same plan): each agent draws random waypoints inside `arena_radius_m` (40 m) of the
  centre of the spawn points, until its path lasts at least `min_flight_s` (default 70 s; the shipped configs set 40 s; at `speed_mps`, 3 m/s), then returns to its spawn point. A candidate set of four paths is kept only if a simulation of the whole
  flight shows **(1)** never more than `max_dropped_links` (default 2; the shipped configs use 3) links at risk at once, where "at risk" means a 3-D distance above *D* minus `margin_m` (3 m), and **(2)** at least `min_drop_events` (2) clear
  drops, where "clear" means a distance above *D* plus the margin for at least `min_drop_s` (6 s). *D* is where the YAML's own signal model reaches the receiver floor (about 51.4 m for the default;
  [SIGNAL_MODEL.md](SIGNAL_MODEL.md)). The simulation lets each vehicle follow its setpoint `tracking_lag_s` (1 s) late. Candidates are drawn from the seed until one qualifies (a median of 10 candidates over
  40 seeds, at most 103; `tests/mesh_plan.sh`). Agent *i* flies at 8 m + 3 m x *i* above its arming point, so paths cannot collide.
- **Shared timeline.** Each agent takes off, touches `/logs/<agent>.airborne` and hovers; when all four have, the first agent's supervisor creates `/logs/mesh_start` and every agent starts its path at that moment.
  Setpoints are a function of the time since then, never of where the vehicle is, so the planned timeline holds for every agent. At the planned end time (all paths done plus `end_hold_s`, 5 s) every agent is hovering
  over its spawn point and lands there.
- **Heartbeat** (`ros2_ws/src/mesh_control`, `mesh_msgs/PeerState`): each agent publishes id, sequence number, send time, position and velocity at 5 Hz on `/mesh/peer_state` (domain 42, over `radio0`) and subscribes to
  everyone's. A neighbour not heard for `link_timeout_s` (1.5 s) is **LOST**; heard again, **UP**. The radio firewall and the isolation checks are unchanged: only domain-42 traffic crosses the channel.
- **Terrain.** `heading_sectors_deg` restricts where the paths may go: waypoints, and the straight legs between them, beyond `roam_radius_m` + 3 m (23 m) of the centre must lie in these arcs. The Baylands YAML uses it
  (below).

## What happens in `excursion` mode

`wanderers` (default: the last agent) fly; every other agent is an anchor that hovers at its spawn point. The wanderer flies `random_waypoints` random points where every link is up, an outbound point just outside
`roam_radius_m` on the excursion heading, the **excursion point** where it hovers `dwell_s` (10 s), back through the outbound point, more random points, home, land. The excursion point is chosen so that exactly
*k* anchor links (1 or 2, from `excursion_links` and the seed) end more than `margin_m` beyond *D* and the others more than `margin_m` inside it. The anchors land once every wanderer has landed.

## YAML (`scenario`, see also [CONFIGURATION.md](CONFIGURATION.md))

`scenario.type: mesh_dropout` selects the stack; leader-follower remains the default when the key is absent. Keys under `scenario.mesh_dropout`, all optional, defaults in `orchestrator/mesh_plan.py`
(the shipped YAMLs list them with comments): `mode`, `seed`, `altitude_base_m`, `altitude_step_m`, `speed_mps`, `heartbeat_hz`, `link_timeout_s`, `margin_m`; roam mode: `arena_radius_m`, `max_dropped_links`, `min_isolation_s`,
`min_drop_events`, `min_drop_s`, `min_flight_s`, `end_hold_s`, `tracking_lag_s`, `max_attempts`; excursion mode: `wanderers`, `random_waypoints`, `roam_radius_m`, `excursion_links`, `dwell_s`, `heading_deg`;
both: `heading_sectors_deg`. The heartbeat QoS is `swarm.qos.peer_state` (default best_effort, volatile, **depth 10**). A plan that cannot be built is rejected with a message (unknown agent or mode, a margin no
direction allows, no plan with the requested drops within `max_attempts`, ...).

## Pass criteria (fixed before the first run; `tools/analyze_mesh_dropout.py`)

| Group | Criterion |
|---|---|
| Flights | every agent reached its altitude, landed and disarmed; was within 2.0 m (horizontal) of its spawn point when its landing began; ended within 5.0 m of it after touchdown |
| Modelled, roam | at least one link falls below the receiver floor for at least 3 s and recovers; every link is above the floor at the end; never more than `max_dropped_links` links below it at once (from ns-3's `pair_rssi.csv`) |
| Measured, roam, when `min_isolation_s` > 0 | one agent had every neighbour LOST at the same time, from its own heartbeat log (added 2026-10-21 with the isolation option; fixed before its first run) |
| Modelled, excursion | links to the planned dropped anchors go below the floor and later recover; every other link never goes below it |
| Measured (heartbeats) | for every modelled drop lasting at least `link_timeout_s` + 1 s (roam), or every planned dropped link (excursion), in both directions: LOST within [-1.0 s, `link_timeout_s` + 2.0 s] of the modelled drop, then UP within [-1.0 s, 10 s] of the modelled recovery; and every LOST event of the run falls where the model has the signal below the floor |
| Validity | ns-3 lag within `time.max_lag_ms`; Gazebo real-time factor over 5 s at least `time.min_rtf` from take-off ([ARCHITECTURE.md](ARCHITECTURE.md), "Run validity") |
| Isolation | no domain-0, XRCE or PX4-topic traffic on the channel; the swarm domain and the mesh topic are present |

The end-position limit was 2.0 m until 2026-10-21. A Baylands run then failed it by 0.17 m although the agent had hovered 0.10 m from its spawn point when landing began: after touchdown the vehicle was bumped 2.2 m by the
ground contact. The check is now split as in the table (the 2.0 m limit applies at the start of landing; 5.0 m is only a sanity bound for a crash or fly-away after touchdown). This was changed after seeing the result.

## Results (single runs, 2026-10-21, seed 7)

| Run | Mode, config | Modelled drops (planned clear drops) | Heartbeat detection after modelled drop | Back after modelled recovery | Checks |
|---|---|---|---|---|---|
| Flat world, headless | roam, `four_agent_mesh_dropout.yaml` | 6 (2); never more than 2 links down at once | +1.0 to +1.4 s | -0.5 to +0.7 s | all pass; 14 LOST events, all explained; ns-3 lag max 0.092 ms; RTF worst window 0.996 |
| Baylands, headless, run twice | roam, `..._baylands.yaml` | 2 (2) | +1.0 to +1.2 s | -0.1 to +0.7 s | all pass both times; RTF 0.997 and 0.999 |
| Baylands, GUI + live map | roam, same | 2 (2) | +1.2 to +1.4 s | -0.4 to +0.4 s | 29 of 29 with the current criteria (28 of 29 with the earlier end-position limit, see above); RTF 0.994; GUI and map did not crash |
| Flat world, headless | excursion, seed 7 | 2 planned links | +1.0 to +1.5 s | -0.1 to 0.0 s | all pass |
| Baylands, headless and with GUI + map | excursion, seed 7 | 2 planned links | +0.9 to +1.5 s | -0.1 to +0.5 s | all pass (36 and 32 checks) |

Hover accuracy when landing began: 0.01 to 0.13 m for every agent in the roam runs. In the Baylands roam runs the plan predicted the drops well: the planned clear drops at plan times 10 to 19 s and 12 to 18 s appeared
in the model at ns-3 times 49 to 59 s and 50 to 58 s, a constant offset of about 39 s (the shared start comes after take-off). In the flat roam run the model shows 6 drops where the plan guarantees 2: links that cross
*D* without exceeding *D* plus the margin are not counted as "clear" by the planner but are real drops.
Heartbeat latency: median about 1.2 ms; the 99th percentile was 2.0 to 2.4 ms in the Baylands runs and 11.3 ms in the flat roam run, whose links spent more time near the floor (frames at about -80 dBm need
retransmissions); that is an observation, not a tested explanation.
`tests/mesh_plan.sh` passes for both configs, both modes, 40 seeds each: repeatable; an independent re-simulation from the plan files confirms the at-risk limit, the clear drops, paths that start and end at the spawn
point at the planned speed and stay inside the allowed area; excursions drop exactly *k* links with the margin.
These are single runs on one machine; they are not statistics.

### Full isolation of one drone (`min_isolation_s`, 2026-10-21)

The shipped configs (`four_agent_mesh_dropout*.yaml`) now set `min_isolation_s: 12`: the planner also requires that at least one agent is beyond *D* + `margin_m` of **all** the
others at once for that long (all three of its links down), and allows up to 3 links at risk (`max_dropped_links: 3`). To make that reachable inside the 40 m arena the shipped
configs also lower the transmit power from 16 to 10 dBm, so *D* is 32.4 m instead of 51.4 m (same chain: 10 dBm - 46.68 dB - 30 log10(d) = -82 dBm). The noise floor was not changed:
the cutoff is the -82 dBm preamble-detection threshold, which sits above noise + 4 dB (-90 dBm), so a lower noise floor would not add disconnections and a higher one would only
matter above about -86 dBm. Excursion mode needs the larger range; run it with `tx_power_dbm: 16.0` (the offline test does).

| Run | Config, command | Result |
|---|---|---|
| Single run, seed 7, 2026-10-21 | `four_agent_mesh_dropout_baylands.yaml`, `demo/run_container_demo.sh --config ... --no-prompt` (GUI, live map) | 36 of 36 checks; plan: agent3 isolated at plan t = 8-21 s (146 candidate plans); measured: agent3 had all three neighbours LOST at once for 14.2 s |

Offline, `tests/mesh_plan.sh` re-simulates 40 seeds: every plan has an isolation of at least 12 s; the planner needed a median of 446 and at most 2157 candidate plans (single-run
seed 7: 146 on Baylands). The flat config's plan was checked offline only, not flown in containers with these settings.

## Choices and findings worth knowing

- **Heartbeat history depth.** `PeerState` has no key, so a subscriber keeps one shared keep-last history for all senders. With depth 1 the first run saw agent4 miss 1 to 3 s of agent1's heartbeats throughout the flight,
  at 6 m range (the earliest sender in each burst was overwritten by the later ones), which looked like random link loss. With depth 10 the only LOST events were real dropouts. A topic with several publishers needs a
  history deeper than the number of senders.
- **Discovery speed.** `config/dds/cyclonedds_mesh.xml` sets the participant announcement interval to 1 s (Cyclone's default is 30 s). With the default, a neighbour out of range for longer than the 10 s lease would
  only be found again at its next announcement, up to 30 s after the signal returned. The shorter interval means the measured recovery follows the radio; it also sends about 30 times more discovery announcements, so it
  is used only by this scenario. Recovery times above are with this setting and are not what an unmodified DDS would show.
- **Detection is delayed by design**: LOST fires `link_timeout_s` after the last heartbeat, so about 1 to 1.5 s after the signal falls below the floor.
- **Baylands terrain.** The world has structures (trees, buildings) up to roughly 15 to 25 m high at headings 300 to 340 degrees and about 14 m at 120 to 170 degrees within 75 m of the spawn area; the agents fly at 8 to 17 m.
  The arcs in `heading_sectors_deg` are the headings whose ground and objects stay below 5 m out to 46 m, found by reading heights from the world's mesh files (accuracy about 1 m, measured against the spawn height,
  not against Gazebo's collision geometry); no allowed cell was higher than 3.2 m. The plans flew without incident in the runs above; a different world needs its own check.
- **Landing on the Baylands slope.** The ground near the triangle falls by a few tenths of a metre toward the east. In the Baylands roam runs the agent spawned at (5, 0) was bumped 0.6, 1.7 and 2.2 m after touchdown
  (the vehicle was already disarmed), in three identical runs; the other three agents stayed within 0.2 m. This is simulator contact behaviour, not the control stack, and it is not repeatable. A later Baylands run with the GUI and the path markers had agent2 end 17.18 m from its spawn point (the 5.0 m after-touchdown bound failed; it was within 0.08 m when landing began). Two repeat runs, one with and one without markers, passed all 33 checks. The markers are GUI-only, so no link to them is expected, but with single runs it is not ruled out.
- **Constant altitude, flat reference.** Flight altitude is constant per agent and distances assume the same flat reference as the rest of the framework.
- **Shutdown artefact.** When the harness stops ns-3 after the last landing every link "drops"; the analyzer ignores events after the last agent has landed.

## Limits

Seed 7 is the only seed flown in the containers (the planners are checked for 40 seeds per mode). Excursion mode was flown with one wanderer. The scenario uses the Wi-Fi model of the YAML unchanged, so the dropout range is the
one set by the log-distance model and the receiver floor, not a measured one. Not run: a world other than the flat one and Baylands, native (non-Docker) execution, more than four agents.
