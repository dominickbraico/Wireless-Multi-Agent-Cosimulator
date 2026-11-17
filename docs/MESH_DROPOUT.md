# Mesh-dropout scenario

A scenario that shows links between agents dropping and recovering. The agents move along seeded random paths and
return to their spawn points. A link drops when two agents are far enough apart and recovers when they close up.

Every agent broadcasts a heartbeat on the emulated radio and logs which neighbours it hears. A dropout is therefore
visible in three places: the modelled signal in ns-3, the live map, and the agents' own message reception. The scenario
runs in the Docker deployment only ([CONTAINERS.md](CONTAINERS.md)).

## Contents

- [Quick start](#quick-start)
- [Modes](#modes)
- [Roam mode](#roam-mode)
- [Excursion mode](#excursion-mode)
- [Full isolation of one agent](#full-isolation-of-one-agent)
- [Mixed fleet](#mixed-fleet)
- [Configuration keys](#configuration-keys)
- [Pass criteria](#pass-criteria)
- [Results](#results)
- [Design notes](#design-notes)
- [Limitations](#limitations)

## Quick start

Build the images once, as described in [CONTAINERS.md](CONTAINERS.md):

```bash
python3 orchestrator/cosim.py build
```

Check the flight plans offline. This takes seconds and needs no Docker:

```bash
tests/mesh_plan.sh
```

Fly the scenario on the flat world, headless, with pass and fail checks. A run takes about 2.5 minutes:

```bash
tests/containers_mesh_dropout.sh --config config/experiments/four_agent_mesh_dropout.yaml
```

Fly it in Baylands with the Gazebo GUI and the live signal map:

```bash
demo/run_container_demo.sh --config config/experiments/four_agent_mesh_dropout_baylands.yaml
```

Fly another seed without editing the YAML:

```bash
MESH_SEED=11 tests/containers_mesh_dropout.sh
```

Outputs in the log directory:

| File | Contents |
|---|---|
| `mesh_dropout.png` | Modelled signal and heartbeat events over time, distances, paths |
| `comm_map_dropout.png` | The signal map before, at the deepest dropout and after |
| `mesh_report.json` | All numbers and check results |
| `mesh_plan.json` | The plan flown |
| `mesh/` | Per-agent heartbeat logs `mesh_rx_<agent>.csv` and `mesh_events_<agent>.csv` |

## Modes

`scenario.mesh_dropout.mode` selects the mode:

| Mode | Behaviour |
|---|---|
| `roam` (default) | Every agent moves on its own random path |
| `excursion` | One agent, the wanderer, flies out of range of 1 or 2 hovering agents; the others hover |

The scenario reuses two parts of the leader-follower stack: PX4's offboard base class and the vehicle-state reader.

## Roam mode

### Plan

`orchestrator/mesh_plan.py` computes the plan. It is a pure function of the YAML, including the seed, so every
container computes the same plan.

1. Each agent draws random waypoints inside `arena_radius_m` of the centre of the spawn points until its path lasts at
   least `min_flight_s` at `speed_mps`. It then returns to its spawn point.
2. The planner simulates the whole flight, with each vehicle following its setpoint `tracking_lag_s` late.
3. The planner keeps the candidate only if both conditions in the table below hold. Otherwise it draws the next
   candidate from the seed.

| Condition | Definition |
|---|---|
| At-risk limit | Never more than `max_dropped_links` links at risk at once. A link is at risk when its 3-D distance exceeds *D* - `margin_m`. |
| Clear drops | At least `min_drop_events` clear drops. A drop is clear when the distance exceeds *D* + `margin_m` for at least `min_drop_s`. |

*D* is the distance at which the YAML's signal model reaches the receiver floor: about 51.4 m for the default radio.
See [SIGNAL_MODEL.md](SIGNAL_MODEL.md).

Agent *i* flies at `altitude_base_m` + *i* x `altitude_step_m` above its arming point, so paths cannot collide.

`heading_sectors_deg` restricts where paths may go. Waypoints, and the straight legs between them, that lie beyond
`roam_radius_m` + 3 m of the centre must fall inside the listed arcs. The Baylands configuration uses it to avoid
terrain.

### Shared timeline

1. Each agent takes off, creates `/logs/<agent>.airborne` and hovers.
2. When every agent has done so, the first agent's supervisor creates `/logs/mesh_start`.
3. Every agent starts its path at that moment. Setpoints are a function of the time since the start, never of the
   vehicle's position, so the planned timeline holds for every agent.
4. At the planned end time, which is the end of all paths plus `end_hold_s`, every agent hovers over its spawn point
   and lands.

### Heartbeat

Each agent publishes `mesh_msgs/PeerState` on `/mesh/peer_state` at `heartbeat_hz` and subscribes to everyone's. The
message holds the sender id, sequence number, send time, position and velocity. The node is `mesh_node` in
`ros2_ws/src/mesh_control`.

| Event | Definition |
|---|---|
| `LOST` | A neighbour is not heard for `link_timeout_s` |
| `UP` | A neighbour is heard for the first time, or again after `LOST` |

The topic uses DDS domain 42 over `radio0`. The radio firewall and the isolation checks are unchanged.

## Excursion mode

The agents in `wanderers` (default: the last agent) fly. Every other agent is an anchor that hovers at its spawn point.

A wanderer flies this sequence:

1. `random_waypoints` random points where every link is up.
2. An outbound point just outside `roam_radius_m`, on the excursion heading.
3. The excursion point, where it hovers for `dwell_s`.
4. Back through the outbound point, then `random_waypoints` more random points, then home, then landing.

The excursion point is chosen so that exactly *k* anchor links end more than `margin_m` beyond *D*, and the others stay
more than `margin_m` inside it. *k* is drawn from `excursion_links` with the seed. The anchors land once every wanderer
has landed.

## Full isolation of one agent

In roam mode, `excursion_agent` and `min_isolation_s` make one drone lose every link at once. The shipped
configurations `four_agent_mesh_dropout.yaml` and `four_agent_mesh_dropout_baylands.yaml` use `excursion_agent: agent3`,
`min_isolation_s: 12`, `max_dropped_links: 3` and `arena_radius_m: 8`.

- The planner requires the excursion agent to be beyond *D* + `margin_m` of all other agents for at least
  `min_isolation_s`.
- The other agents roam within `arena_radius_m` of the centre. With 8 m, two agents are at most 16 m apart in the plane,
  against *D* - `margin_m` = 17.5 m, so their mutual links stay up.
- The excursion agent flies `excursion_pre_waypoints` random points inside the arena. It then flies straight to one
  point
  at *D* + `margin_m` + `excursion_extra_m` from the centre, holds there for `min_isolation_s` + 4 s and flies home.
- The planner keeps a candidate only if the simulation of everyone's lagged positions confirms the isolation.
- Legs shorter than 3 m are rejected. Plan times are rounded to 0.01 s, so a 1 m leg would carry a speed error above
  3 %.

The shipped configurations lower the transmit power from 16 dBm to 4 dBm to keep the trip short. *D* is then 20.5 m
instead of 51.4 m, from the same equation: 4 dBm - 46.68 dB - 30 log10(d) = -82 dBm. The excursion point is 25.5 m from
the centre for seed 7.

The noise floor is unchanged. The cutoff is the -82 dBm preamble-detection threshold, which lies above the noise floor
plus 4 dB (-90 dBm).

Excursion mode needs the larger range. Run it with `tx_power_dbm: 16.0` and without `excursion_agent`, as
`tests/mesh_plan.sh` does.

## Mixed fleet

An agent with `vehicle: r1_rover` is a ground vehicle, the Aion R1 in Gazebo. See "Vehicle types" in
[CONTAINERS.md](CONTAINERS.md). Rovers require `mode: roam`. The planner treats a rover differently from a drone:

| Aspect | Rover behaviour |
|---|---|
| Speed | `rover_speed_mps` (default 1.0) |
| Arena | Waypoints within `rover_arena_radius_m` (default 6.0) of its own spawn point, with the same 3 m minimum leg and `min_flight_s` |
| Altitude and terrain arcs | Altitude 0; `heading_sectors_deg` is ignored |
| Excursion | A rover cannot be the `excursion_agent`: at 1 m/s the trip would take about 25 s each way |
| Control | `mesh_drive` in `ros2_ws/src/mesh_control` follows the planned path on the shared timeline |
| Ready signal | It creates `<agent>.airborne` when it is armed and ready, then waits for `mesh_start` |
| Setpoint | PX4's generic offboard position setpoint, moving at the planned speed |
| Distances | 3-D in the model, with the rover at its spawn height |

`config/experiments/drone_rover_mesh_baylands.yaml` is the shipped example: a drone (agent1, the excursion agent) and a
rover (agent2) in Baylands, 7.1 m apart, with a transmit power of 4 dBm.

`config/experiments/two_drone_two_rover_mesh_baylands.yaml` is a fleet of four with the same radio settings: two drones
(agent1, the excursion agent, and agent2) and two rovers (agent3, agent4) in a diamond.

- The planner has no collision avoidance on the ground, so the layout keeps the rovers apart. `rover_arena_radius_m` is
  4.0 and the rovers' spawn points are 10 m apart, which leaves 2 m between their discs. Each drone's spawn point is
  2.4 m from both discs.
- A rover disc must be larger than the planner's 3 m minimum leg. With `rover_arena_radius_m: 3.0`, no path can be
  drawn and the planner stops with an error.

## Configuration keys

`scenario.type: mesh_dropout` selects the scenario. All keys under `scenario.mesh_dropout` are optional. The defaults
are in `orchestrator/mesh_plan.py`. An unknown key, or a plan that cannot be built, is rejected with a message before
anything starts.

| Key | Default | Mode | Meaning |
|---|---|---|---|
| `mode` | `roam` | Both | `roam` or `excursion` |
| `seed` | 1 | Both | Seed of every random draw |
| `altitude_base_m`, `altitude_step_m` | 8.0, 3.0 | Both | Agent *i* flies at base + *i* x step |
| `speed_mps` | 3.0 | Both | Speed of multirotors |
| `heartbeat_hz` | 5.0 | Both | Heartbeat rate |
| `link_timeout_s` | 1.5 | Both | Silence after which a neighbour is `LOST` |
| `margin_m` | 3.0 | Both | Margin around *D* |
| `heading_sectors_deg` | None | Both | Allowed `[from, to]` arcs in degrees, ENU: 0 is east, 90 is north |
| `arena_radius_m` | 40.0 | Roam | Radius within which waypoints are drawn |
| `max_dropped_links` | 2 | Roam | Most links at risk at once |
| `min_drop_events`, `min_drop_s` | 2, 6.0 | Roam | Number and duration of clear drops |
| `min_flight_s` | 70.0 | Roam | Minimum path duration |
| `end_hold_s` | 5.0 | Roam | Hover at home before landing |
| `tracking_lag_s` | 1.0 | Roam | How late a vehicle follows its setpoint in the planner's simulation |
| `max_attempts` | 50000 | Roam | Candidate plans tried before giving up |
| `excursion_agent` | None | Roam | Agent that is cut off from all others |
| `excursion_extra_m`, `excursion_pre_waypoints` | 2.0, 2 | Roam | Extra distance of the excursion point; random points before the trip |
| `min_isolation_s` | 0.0 | Roam | Required duration of full isolation; 0 disables it |
| `rover_speed_mps`, `rover_arena_radius_m` | 1.0, 6.0 | Roam | Speed and arena of rovers |
| `wanderers` | Last agent | Excursion | Agents that fly |
| `random_waypoints` | 3 | Excursion | Random points before and after the excursion |
| `roam_radius_m` | 20.0 | Excursion | Radius of the random points |
| `excursion_links` | `[1, 2]` | Excursion | Possible numbers of links to drop |
| `dwell_s` | 10.0 | Excursion | Hover time at the excursion point |
| `heading_deg` | From the seed | Excursion | Fixed excursion heading |
| `max_excursion_m` | 250.0 | Excursion | Largest excursion distance |

The heartbeat QoS is `swarm.qos.peer_state`. See [CONFIGURATION.md](CONFIGURATION.md).

## Pass criteria

`tools/analyze_mesh_dropout.py` applies the criteria. They were fixed before the first run, except for the changes in
the second table.

| Group | Criterion |
|---|---|
| Flights | Every agent reaches its altitude, lands and disarms |
| Flights | Every agent is within 2.0 m (horizontal) of its spawn point when its landing begins, and within 5.0 m after touchdown |
| Flights, rover | A rover arms in offboard mode, completes its route and disarms, with the same 2.0 m and 5.0 m limits at the end of its route and after it disarms |
| Modelled, roam | At least one link falls below the receiver floor for at least 3 s and recovers |
| Modelled, roam | Every link is above the floor at the end, and never more than `max_dropped_links` links are below it at once |
| Measured, roam, `min_isolation_s` > 0 | One agent has every neighbour `LOST` at the same time, in its own heartbeat log |
| Modelled, excursion | Links to the planned dropped anchors go below the floor and recover; every other link stays above it |
| Measured | `LOST` falls within [-1.0 s, `link_timeout_s` + 2.0 s] of each modelled drop, in both directions |
| Measured | `UP` falls within [-1.0 s, 10 s] of each modelled recovery, in both directions |
| Measured | Every `LOST` event of the run falls where the model has the signal below the floor |
| Validity | ns-3 lag within `time.max_lag_ms`; the simulator's real-time factor over 5 s at least `time.min_rtf` from take-off |
| Isolation | No domain-0, XRCE or PX4-topic traffic on the channel; the swarm domain and the mesh topic are present |

The real-time factor is Gazebo's own `/stats` factor. For Isaac Sim (experimental) it comes from the pose feed; see
[EXPERIMENTS.md](EXPERIMENTS.md). A run without a real-time-factor source fails the check.

The measured criteria apply to every modelled drop that lasts at least `link_timeout_s` + 1 s (roam) or to every planned
dropped link (excursion). The modelled values come from ns-3's `pair_rssi.csv`.

| Date | Criterion change | Reason |
|---|---|---|
| 2026-10-21 | The end-position check is split: 2.0 m when landing begins, 5.0 m after touchdown. It was 2.0 m after touchdown. | A Baylands run failed by 0.17 m although the agent hovered 0.10 m from its spawn point when landing began; ground contact bumped it 2.2 m after touchdown. This change was made after seeing the result. |
| 2026-10-21 | The full-isolation criterion was added with the `min_isolation_s` option | Fixed before the first run that used the option |

## Results

All results are single runs with seed 7 on the development machine, in Docker. They are not statistics.

### Roam and excursion modes, 2026-10-21

These runs predate the full-isolation settings of the shipped configurations.

| World, display | Mode | Modelled drops (planned clear drops) | `LOST` after modelled drop | `UP` relative to modelled recovery | Checks |
|---|---|---|---|---|---|
| Flat, headless | Roam | 6 (2) | +1.0 s to +1.4 s | -0.5 s to +0.7 s | All pass |
| Baylands, headless, two runs | Roam | 2 (2) | +1.0 s to +1.2 s | -0.1 s to +0.7 s | All pass in both |
| Baylands, GUI and map | Roam | 2 (2) | +1.2 s to +1.4 s | -0.4 s to +0.4 s | 29 of 29 |
| Flat, headless | Excursion | 2 planned links | +1.0 s to +1.5 s | -0.1 s to 0.0 s | All pass |
| Baylands, headless; GUI and map | Excursion | 2 planned links | +0.9 s to +1.5 s | -0.1 s to +0.5 s | All pass (36 and 32 checks) |

Further measurements from those runs:

| Quantity | Value |
|---|---|
| Flat roam run | Never more than 2 links down at once; 14 `LOST` events, all explained; ns-3 lag at most 0.092 ms; worst real-time-factor window 0.996 |
| Baylands roam runs, worst real-time-factor window | 0.997 and 0.999 headless; 0.994 with the GUI |
| Baylands roam run with the GUI | 28 of 29 under the earlier end-position limit; see [Pass criteria](#pass-criteria) |
| Hover accuracy when landing began, roam runs | 0.01 m to 0.13 m for every agent |
| Heartbeat latency, median | About 1.2 ms |
| Heartbeat latency, 99th percentile | 2.0 ms to 2.4 ms in Baylands; 11.3 ms in the flat roam run |

- In the Baylands roam runs, the planned clear drops at plan times 10 s to 19 s and 12 s to 18 s appeared in the model
  at ns-3 times 49 s to 59 s and 50 s to 58 s. The offset of about 39 s is constant, because the shared start follows
  take-off.
- In the flat roam run, the model shows 6 drops where the plan guarantees 2. Links that cross *D* without exceeding
  *D* + `margin_m` are real drops that the planner does not count as clear.
- The links of the flat roam run spent more time near the floor, where frames at about -80 dBm need retransmissions.
  That is an observation about its higher latency, not a tested explanation.

### Full isolation, 2026-10-21

| Configuration | Command | Outcome |
|---|---|---|
| `four_agent_mesh_dropout_baylands.yaml` | `demo/run_container_demo.sh --config ... --no-prompt`, two runs with the GUI, one with the live map | 36 of 36 checks in each |

Plan: agent3 isolated at plan times 14 s to 27 s, found after 109 candidates; 3 clear drops; peak pair distance 32 m;
flight 48 s. Measured: agent3 had all three neighbours `LOST` at once for 17.0 s and 17.5 s.

### Drone and rover in Baylands, 2026-11-02

| Configuration | Command | Outcome |
|---|---|---|
| `drone_rover_mesh_baylands.yaml` | `tests/containers_mesh_dropout.sh --config ...`, headless | All checks pass |

| Quantity | Value |
|---|---|
| Plan | First candidate; flight 47 s; the one link drops at plan times 13 s to 34 s, peak 34 m |
| Drone disconnected | 21.9 s |
| `LOST` after the modelled drop | 1.1 s and 1.2 s |
| `UP` relative to the modelled recovery | -0.1 s and +0.1 s |
| Distance from spawn at the end | Drone 0.05 m, rover 0.41 m |
| ns-3 lag; worst 5 s real-time-factor window | At most 0.44 ms; 1.000 |
| Drone path, from ground truth | 67 m flown (planned 65.6 m), 22.7 m from its spawn point at most |
| Rover path, from ground truth | 32 m driven of 42.4 m planned, 5.2 m from its spawn point at most |

The rover cuts corners: the setpoint moves along the plan and the vehicle's own path controller smooths it. The
comparison with the planned length is not a criterion.

### Two drones and two rovers in Baylands, 2026-11-10

| Configuration | Command | Outcome |
|---|---|---|
| `two_drone_two_rover_mesh_baylands.yaml` | `tests/containers_mesh_dropout.sh --config ...`, headless | All checks pass |

| Quantity | Value |
|---|---|
| Plan | Seventh candidate; flight 54 s; agent1's three links drop at plan times 10 s to 29 s, peaks 28 m to 33 m |
| agent1 disconnected | 18.3 s |
| `LOST` after the modelled drop, six directions | 0.9 s to 1.3 s |
| `UP` after the modelled recovery | 0.1 s to 0.6 s |
| Distance from spawn at the end | Drones 0.02 m and 0.04 m; rovers 0.51 m and 0.45 m |
| ns-3 lag; worst 5 s real-time-factor window | At most 0.52 ms; 0.998 (worst sample 0.91) |
| Closest planned approach of the two rovers | 6.1 m, computed from the plan's waypoints |

No `LOST` event fell outside a modelled drop. The driven separation of the rovers is not measured.

### Offline plan checks

`tests/mesh_plan.sh` re-simulates the plans from the plan files, independently of the planner, for 40 seeds per mode.

| Configuration | Outcome |
|---|---|
| Both `four_agent_mesh_dropout` files, both modes | Pass: plans are repeatable and differ per seed; the at-risk limit, clear drops, speed, start and end points and allowed area are confirmed |
| The same files, full isolation | Every plan isolates a drone for at least 12 s; the planner needs a median of 75 to 81 and at most 324 candidates |
| The same files, excursion mode | Every plan drops exactly *k* links with the margin |
| `drone_rover_mesh_baylands.yaml` | Pass for 40 seeds; excursion mode is skipped for rovers |
| `two_drone_two_rover_mesh_baylands.yaml` | Pass for 40 seeds; a median of 9 and at most 47 candidates |

Before the full-isolation settings were added to the shipped configurations, the planner needed a median of 10 and at
most 103 candidates over 40 seeds.

## Design notes

| Topic | Note |
|---|---|
| Heartbeat history depth | `PeerState` has no key, so a subscriber keeps one keep-last history for all senders. With depth 1, simultaneous heartbeats overwrite each other; depth 10 is the default. |
| Discovery speed | `config/dds/cyclonedds_mesh.xml` sets the participant announcement interval to 1 s. Cyclone's default is 30 s. |
| Detection delay | `LOST` fires `link_timeout_s` after the last heartbeat, so about 1 s to 1.5 s after the signal falls below the floor. |
| Altitude | Each agent flies at a constant altitude. Distances assume the same flat reference as the rest of the framework. |
| Shutdown | When the harness stops ns-3 after the last landing, every link drops. The analyzer ignores events after the last agent lands. |

Heartbeat history depth: in the first run with depth 1, agent4 missed 1 s to 3 s of agent1's heartbeats throughout the
flight at 6 m range, which looked like random link loss. A topic with several publishers needs a history deeper than
the number of senders.

Discovery speed: with the default interval, a neighbour that is out of range for longer than the 10 s lease is found
again only at its next announcement, up to 30 s after the signal returns. The shorter interval makes the measured
recovery follow the radio. It also sends about 30 times more discovery announcements, so only this scenario uses it.
The recovery times above are not what an unmodified DDS configuration shows.

Baylands terrain: the world has structures up to roughly 15 m to 25 m high at headings 300 to 340 degrees, and about
14 m at 120 to 170 degrees, within 75 m of the spawn area. The agents fly at 8 m to 17 m. The arcs in
`heading_sectors_deg` are the headings whose ground and objects stay below 5 m out to 46 m. They were found by reading
heights from the world's mesh files, with an accuracy of about 1 m against the spawn height. No allowed cell is higher
than 3.2 m.

## Limitations

- Seed 7 is the only seed flown. The planners are checked offline for 40 seeds per mode.
- Excursion mode is flown with one wanderer only.
- The flat configuration is not flown with the full-isolation settings.
- The dropout range is the one set by the log-distance model and the receiver floor. It is not a measured range.
- Not tested: a world other than the flat one and Baylands, native execution, more than four agents, more than two
  rovers, two rovers whose discs overlap, a rover in the demo with the Gazebo GUI.
- A fixed-wing agent (`rc_cessna`, node `mesh_plane`) is work in progress. It has no passing run.
- The planner has no ground-level obstacle map. Whether the ground in a rover's arena is free of obstacles is not
  known; a run fails if the rover stops short.
- The Baylands terrain check uses mesh heights, not Gazebo's collision geometry. A different world needs its own check.
- Landing on the Baylands slope is not repeatable. In three identical roam runs, the agent spawned at (5, 0) was bumped
  0.6 m, 1.7 m and 2.2 m after touchdown, when it was already disarmed; the others stayed within 0.2 m.
- In one later Baylands run with the GUI and path markers, agent2 ended 17.18 m from its spawn point and failed the
  5.0 m bound; it was within 0.08 m when landing began. Two repeat runs, one with markers and one without, passed all
  33 checks. The markers exist only in the GUI, so no link is expected, but single runs do not rule one out.
