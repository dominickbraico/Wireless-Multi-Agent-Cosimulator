# Signal strength model

How the signal strength drawn by `tools/plot_comm_map.py` is calculated, with every parameter, its value, where it is set, and where the value comes from.
Versions: ns-3 3.48 (with the local patch in `ns3/patches/`), `ns3::LogDistancePropagationLossModel`, `ns3::ThresholdPreambleDetectionModel`.

## 1. What the map shows

For every ordered pair of agents (A to B and B to A) the ns-3 scenario computes **the power a frame sent by A would arrive with at B**, at the agents'
current positions. The map draws one line per pair, coloured by that power. It is a prediction of the propagation model, not a measurement of traffic: it exists
for pairs that never exchange a message, because all agents share one channel. It excludes contention, collisions, antenna gains and fading (section 7).

## 2. The equation

For one log-distance model, with `d` the 3-D distance between the two antennas in metres:

```
P_rx(d) [dBm] = P_tx - L0 - 10 * n * log10(d / d0)      for d > d0
P_rx(d) [dBm] = P_tx - L0                                for d <= d0
```

| Symbol | Meaning | Unit |
|---|---|---|
| `P_tx` | transmit power (`network.wifi.tx_power_dbm`) | dBm |
| `L0` | reference loss at the reference distance (`ref_loss_db`) | dB |
| `d0` | reference distance (`ref_distance_m`) | m |
| `n` | path-loss exponent (`exponent`) | dimensionless |
| `d` | distance between the two antennas (agent position + `antenna_offset`) | m |

This is the body of `LogDistancePropagationLossModel::DoCalcRxPower` in ns-3 3.48 (`src/propagation/model/propagation-loss-model.cc`). If
`network.channel.loss` lists several models, ns-3 applies them in order, each taking the previous output as its transmit power
(`PropagationLossModel::CalcRxPower`), so the losses add in dB. Transmit and receive antenna gains are ns-3's defaults (0 dB) and are not applied.

**With the default configuration** (`P_tx` = 16.0 dBm, `L0` = 46.6777 dB, `d0` = 1.0 m, `n` = 3.0):

```
P_rx(d) = -30.678 - 30 * log10(d)   dBm      (d in metres, d > 1 m)
```

| d (m) | 1 | 2 | 4.4 | 8 | 16 | 24 | 32 | 40 | 51.4 | 60 | 100 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| P_rx (dBm) | -30.68 | -39.71 | -49.98 | -57.77 | -66.80 | -72.08 | -75.83 | -78.74 | -82.01 | -84.02 | -90.68 |

Computed from the equation above (arithmetic, not a simulation result).

## 3. Where the code is

| Step | Location |
|---|---|
| Build the loss chain from `network.channel.loss` | `ns3/scenarios/wifi_adhoc_tap/wifi_adhoc_tap.cc`, `BuildLossChain()` |
| Give the same chain to the Wi-Fi channel | same file, `yansChannel->SetPropagationLossModel(lossChain)` |
| Evaluate every ordered pair every `network.link_log_interval_s` (default 0.5 s simulated) | same file, `LinkLogTick()`, the call `loss->CalcRxPower(txPowerDbm, a, b)` |
| Output | `<log dir>/ns3/pair_rssi.csv`: `t_s, tx, rx, distance_m, rssi_dbm, below_floor` |
| Read and draw | `tools/plot_comm_map.py`: `PairLog` (reads the file), `make_rssi_of()` (weaker direction of a pair), `draw_panel()` (lines, colours) |
| Fallback when a run has no `pair_rssi.csv` | `rx_power_dbm()` in `tools/plot_comm_map.py`: the same equation from the YAML; the figure title says "YAML-formula estimate" |

Because the real frames and the log use the same loss object, a change to `network.channel.loss` or to the agents' positions changes both.

## 4. Parameters, defaults and their origin

| Parameter (YAML key) | Value used | Origin | Calibrated to hardware? |
|---|---|---|---|
| `network.wifi.tx_power_dbm` | 16.0 dBm | We set it. ns-3's default is 16.0206 dBm (40 mW), so the difference is 0.02 dB | No |
| `network.channel.loss[].ref_loss_db` (`L0`) | 46.6777 dB | ns-3 default. Its documentation calls it "Friis at 1 m with 5.15 GHz". Friis free-space loss at 1 m is 46.68 dB at 5.15 GHz and 46.73 dB at 5.18 GHz (channel 36), so the 0.05 dB difference is not material | No |
| `...loss[].ref_distance_m` (`d0`) | 1.0 m | ns-3 default | n/a |
| `...loss[].exponent` (`n`) | 3.0 | ns-3 default. Free space is 2; obstructed environments are higher. **This value is an assumption, not a measurement of any environment** | No |
| Antenna gains (Tx, Rx) | 0 dB | ns-3 defaults (`TxGain`, `RxGain`); not in the YAML | No |
| `network.wifi.phy.preamble_detection_min_rssi_dbm` | -82 dBm | ns-3 default (`ThresholdPreambleDetectionModel::MinimumRssi`) | No |
| `network.wifi.phy.preamble_detection_snr_db` | 4 dB | ns-3 default (`...::Threshold`) | No |
| `network.wifi.phy.rx_noise_figure_db` | 7 dB | ns-3 default (`RxNoiseFigure`) | No |
| `network.wifi.phy.cca_ed_threshold_dbm` | -62 dBm | ns-3 default (`CcaEdThreshold`) | No |
| `network.wifi.phy.rx_sensitivity_dbm` | -101 dBm | ns-3 default (`RxSensitivity`) | No |

The YAML writes the receiver values out explicitly so they are visible and changeable; they were not tuned. All of `L0`, `d0`, `n` and the receiver
values being ns-3 defaults means the numbers describe ns-3's default radio, not a particular device. Calibrating them needs measurements of the target
hardware (RSSI versus distance gives `L0` and `n`; receiver datasheet or measurement gives the PHY values).

## 5. Link detectability (pairs with no line)

The map draws no line for a pair that is below the floor, that is when `P_rx < preamble_detection_min_rssi_dbm` (-82 dBm by default). That is a **PHY** mechanism: the receiver's
`ThresholdPreambleDetectionModel` never detects a frame weaker than that, whatever else is true, and also needs SNR of at least 4 dB. The MAC layer is
separate (retries, backoff, rate adaptation); it reacts to the missing acknowledgement.

- Noise floor: `N [dBm] = 10 log10(k T B) + 30 + NF`, with k = 1.3803e-23 J/K, T = 290 K, B = 20 MHz (802.11a), NF = 7 dB, giving -93.97 dBm (as in `tools/link_budget.py`).
- Distance at which the default floor is reached: `d_floor = d0 * 10^((P_tx - L0 - P_floor) / (10 n))` = 10^((16 - 46.6777 + 82) / 30) = **51.4 m**.
  `tools/link_budget.py` predicts UDP loss going from 1 % to 99 % at 51.5 m for this configuration, limited by the RSSI floor; an earlier distance sweep (not repeated since the script layout changed) found a step at
  the same place (recorded in `EXPERIMENTS.md`). Beyond it the link is dead; inside it, at the fixed 12 Mbit/s rate, the link works (SNR there is about 11.9 dB).

## 6. How the colours are chosen

`plot_comm_map.py` maps `P_rx` linearly onto a colour scale clipped at both ends: red at the receiver floor (-82 dBm) through orange and yellow to dark green at -50 dBm.
With the default configuration -50 dBm is reached at 4.4 m, so dark green appears only when two agents are within about 4 m; agents 8 to 32 m apart appear yellow to orange-red.
`--rssi-range LO HI` changes the scale; this is a display choice and does not change the simulation.

The figure also states the transmit power (in the saved figures' caption and next to each agent's name) and the dropout rule. A pair whose signal is below the floor has **no line**:
the map shows only links that can carry frames. The caption gives the distance at which the signal reaches the floor, found by solving the same chain of loss models
numerically (`dropout_range_m()` in `tools/plot_comm_map.py`); for the default configuration it prints 51 m, matching the closed form in section 5. The data in
`pair_rssi.csv` keeps every pair, with `below_floor` = 1 for the ones the map leaves out.

## 7. Assumptions and limitations

- **One deterministic model.** Received power depends only on distance: no shadowing, multipath fading, obstacles or antenna patterns. Real radios fluctuate and degrade gradually; this model has a hard edge at the floor.
- **No traffic effects.** Contention, collisions and hidden nodes are not in `pair_rssi.csv`.
- **PHY floor only.** `below_floor` uses the RSSI floor, not the SNR threshold or the error-rate model.
- **Fixed rate.** `ConstantRate` at OfdmRate12Mbps; no rate adaptation, which would extend the range at lower rates in a real radio.
- **Two directions.** The map draws the weaker of the two directions; with equal power and gains they are equal. Only `LogDistance` loss models are supported by the scenario.
- **Timing.** Samples are taken every 0.5 s of simulated time; positions are updated from Gazebo at 50 Hz. A label can lag the drawn geometry by up to one sample interval.
- **Ground assumed flat.** Altitude conversion uses ground at z = 0 (see ARCHITECTURE.md).

## 8. Reproducing

```
python3 tools/link_budget.py config/experiments/five_agent_wifi.yaml          # noise floor and predicted range
tests/containers_static_link.sh                                              # checks pair_rssi.csv against the formula and against the signal the receivers measured
python3 tools/plot_comm_map.py <log dir>                                     # draw the map from a run
```

`tests/containers_static_link.sh` (two agents, 2026-10-20, single run) found the logged -60.678 dBm equal to the YAML formula and to the signal measured on real frames.

## 9. References

- H. T. Friis, "A Note on a Simple Transmission Formula," Proc. IRE 34(5), 254-256, 1946 (free-space loss).
- T. S. Rappaport, *Wireless Communications: Principles and Practice*, 2nd ed., Prentice Hall, 2002 (log-distance path-loss model; typical exponents for different environments). Check the chapter and table numbers before citing.
- ns-3 3.48 source and documentation: `src/propagation` (`LogDistancePropagationLossModel`), `src/wifi` (`WifiPhy`, `ThresholdPreambleDetectionModel`, `YansWifiChannel`).
