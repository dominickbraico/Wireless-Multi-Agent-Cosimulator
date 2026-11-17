# Signal strength model

How the received signal strength between two agents is calculated: the equation, every parameter, where each value is
set and where it comes from. `tools/plot_comm_map.py` draws this quantity.

Versions: ns-3 3.48 with the local patch in `ns3/patches/`, `ns3::LogDistancePropagationLossModel`,
`ns3::ThresholdPreambleDetectionModel`.

## Contents

- [Quantity](#quantity)
- [Equation](#equation)
- [Code locations](#code-locations)
- [Parameters and their origin](#parameters-and-their-origin)
- [Link detectability](#link-detectability)
- [Map colours](#map-colours)
- [Usage](#usage)
- [Results](#results)
- [Limitations](#limitations)
- [References](#references)

## Quantity

For every ordered pair of agents, the ns-3 scenario computes the power with which a frame sent by A would arrive at B,
at the agents' current positions. The map draws one line per pair, coloured by that power.

The value is a prediction of the propagation model, not a measurement of traffic. It exists for pairs that never
exchange a message, because all agents share one channel.

## Equation

For one log-distance model, with `d` the 3-D distance between the two antennas:

```text
P_rx(d) [dBm] = P_tx - L0 - 10 * n * log10(d / d0)      for d > d0
P_rx(d) [dBm] = P_tx - L0                                for d <= d0
```

| Symbol | Meaning | YAML key | Unit |
|---|---|---|---|
| `P_tx` | Transmit power | `network.wifi.tx_power_dbm` | dBm |
| `L0` | Loss at the reference distance | `network.channel.loss[].ref_loss_db` | dB |
| `d0` | Reference distance | `network.channel.loss[].ref_distance_m` | m |
| `n` | Path-loss exponent | `network.channel.loss[].exponent` | none |
| `d` | Distance between the antennas (agent position plus `antenna_offset`) | | m |

The equation is the body of `LogDistancePropagationLossModel::DoCalcRxPower` in ns-3 3.48
(`src/propagation/model/propagation-loss-model.cc`). When `network.channel.loss` lists several models, ns-3 applies
them in order and the losses add in dB. Antenna gains are ns-3's defaults (0 dB).

With the default configuration (`P_tx` = 16.0 dBm, `L0` = 46.6777 dB, `d0` = 1.0 m, `n` = 3.0) the equation becomes:

```text
P_rx(d) = -30.678 - 30 * log10(d)   dBm      (d in metres, d > 1 m)
```

Values computed from that equation (arithmetic, not a simulation result):

| d (m) | 1 | 2 | 4.4 | 8 | 16 | 24 | 32 | 40 | 51.4 | 60 | 100 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| P_rx (dBm) | -30.68 | -39.71 | -49.98 | -57.77 | -66.80 | -72.08 | -75.83 | -78.74 | -82.01 | -84.02 | -90.68 |

## Code locations

| Step | Location |
|---|---|
| Build the loss chain from `network.channel.loss` | `BuildLossChain()` in `ns3/scenarios/wifi_adhoc_tap/wifi_adhoc_tap.cc` |
| Give the same chain to the Wi-Fi channel | `yansChannel->SetPropagationLossModel(lossChain)`, same file |
| Evaluate every ordered pair | `LinkLogTick()`, same file, every `network.link_log_interval_s` (default 0.5 s simulated) |
| Write the log | `<log dir>/ns3/pair_rssi.csv`: `t_s, tx, rx, distance_m, rssi_dbm, below_floor` |
| Read and draw | `PairLog`, `make_rssi_of()` and `draw_panel()` in `tools/plot_comm_map.py` |
| Fallback without `pair_rssi.csv` | `rx_power_dbm()` in `tools/plot_comm_map.py` |

The real frames and the log use the same loss object, so a change to `network.channel.loss` or to the positions changes
both. The fallback evaluates the same equation from the YAML, and the figure title then says "YAML-formula estimate".

## Parameters and their origin

No parameter is calibrated to hardware.

| YAML key | Value | Origin |
|---|---|---|
| `network.wifi.tx_power_dbm` | 16.0 dBm | Set by this project. The ns-3 default is 16.0206 dBm (40 mW). |
| `network.channel.loss[].ref_loss_db` | 46.6777 dB | ns-3 default, documented there as "Friis at 1 m with 5.15 GHz". |
| `network.channel.loss[].ref_distance_m` | 1.0 m | ns-3 default. |
| `network.channel.loss[].exponent` | 3.0 | ns-3 default. An assumption, not a measurement of any environment. |
| Antenna gains (not in the YAML) | 0 dB | ns-3 defaults `TxGain` and `RxGain`. |
| `network.wifi.phy.preamble_detection_min_rssi_dbm` | -82 dBm | ns-3 default `ThresholdPreambleDetectionModel::MinimumRssi`. |
| `network.wifi.phy.preamble_detection_snr_db` | 4 dB | ns-3 default `ThresholdPreambleDetectionModel::Threshold`. |
| `network.wifi.phy.rx_noise_figure_db` | 7 dB | ns-3 default `RxNoiseFigure`. |
| `network.wifi.phy.cca_ed_threshold_dbm` | -62 dBm | ns-3 default `CcaEdThreshold`. |
| `network.wifi.phy.rx_sensitivity_dbm` | -101 dBm | ns-3 default `RxSensitivity`. |

Friis free-space loss at 1 m is 46.68 dB at 5.15 GHz and 46.73 dB at 5.18 GHz (channel 36). The 0.05 dB difference is
not material. A path-loss exponent of 2 is free space; obstructed environments are higher.

The YAML states the receiver values explicitly so that they are visible and changeable. The numbers describe ns-3's
default radio, not a particular device. Calibration needs measurements of the target hardware: RSSI against distance
gives `L0` and `n`, and a receiver datasheet or measurement gives the PHY values.

## Link detectability

A pair is below the floor when `P_rx < preamble_detection_min_rssi_dbm` (-82 dBm by default). The receiver's
`ThresholdPreambleDetectionModel` never detects a weaker frame. It also needs an SNR of at least 4 dB. This is a PHY
mechanism; the MAC layer reacts separately to the missing acknowledgement.

The noise floor is:

```text
N [dBm] = 10 * log10(k * T * B) + 30 + NF = -93.97 dBm
k = 1.3803e-23 J/K,  T = 290 K,  B = 20 MHz (802.11a),  NF = 7 dB
```

The distance at which the default floor is reached is:

```text
d_floor = d0 * 10^((P_tx - L0 - P_floor) / (10 * n)) = 10^((16 - 46.6777 + 82) / 30) = 51.4 m
```

Beyond `d_floor` the link carries no frames. Inside it, at the fixed 12 Mbit/s rate, the link works; the SNR at
`d_floor` is about 11.9 dB. `tools/link_budget.py` computes the same noise floor and predicts UDP loss rising from 1 %
to 99 % at 51.5 m for this configuration.

## Map colours

`tools/plot_comm_map.py` maps `P_rx` linearly onto a colour scale that is clipped at both ends: red at the receiver
floor (-82 dBm), through orange and yellow, to dark green at -50 dBm. With the default configuration, -50 dBm is reached
at 4.4 m. Agents 8 m to 32 m apart therefore appear yellow to orange-red.

- `--rssi-range LO HI` changes the scale. It is a display choice and does not change the simulation.
- A pair below the floor has no line. `pair_rssi.csv` keeps every pair and marks those with `below_floor` = 1.
- The caption states the transmit power and the distance at which the signal reaches the floor. `dropout_range_m()`
  solves the same loss chain numerically; for the default configuration it prints 51 m.

## Usage

Print the noise floor and the predicted range:

```bash
python3 tools/link_budget.py config/experiments/five_agent_wifi.yaml
```

Check `pair_rssi.csv` against the equation and against the signal the receivers measure:

```bash
tests/containers_static_link.sh
```

Draw the map from a run:

```bash
python3 tools/plot_comm_map.py <log dir>
```

## Results

| Date | Command | Configuration | Result |
|---|---|---|---|
| 2026-10-20, single run | `tests/containers_static_link.sh` | `two_agent_wifi.yaml`, two agents 10 m apart | Logged -60.678 dBm, equal to the equation and to the signal measured on real frames |

An earlier distance sweep found the loss step at the distance predicted in [Link detectability](#link-detectability).
It is reported in [EXPERIMENTS.md](EXPERIMENTS.md) and has not been repeated with the current scripts.

## Limitations

- Received power depends only on distance. Shadowing, multipath fading, obstacles and antenna patterns are not modelled.
- The model has a hard edge at the floor. Real radios fluctuate and degrade gradually.
- `pair_rssi.csv` contains no traffic effects: contention, collisions and hidden nodes are absent.
- `below_floor` uses the RSSI floor only, not the SNR threshold or the error-rate model.
- The rate is fixed (`ConstantRate`, `OfdmRate12Mbps`). Rate adaptation is not used.
- The scenario supports only `LogDistance` loss models.
- The map draws the weaker of the two directions of a pair. With equal power and gains they are equal.
- Samples are taken every 0.5 s of simulated time, so a label can lag the drawn geometry by one sample interval.
- Altitude conversion assumes ground at z = 0. See [ARCHITECTURE.md](ARCHITECTURE.md).

## References

- H. T. Friis, "A Note on a Simple Transmission Formula," Proc. IRE 34(5), 254-256, 1946. Free-space loss.
- T. S. Rappaport, *Wireless Communications: Principles and Practice*, 2nd ed., Prentice Hall, 2002. Log-distance
  path-loss model and typical exponents. Chapter and table numbers are not verified here.
- ns-3 3.48 source and documentation: `src/propagation` (`LogDistancePropagationLossModel`) and `src/wifi` (`WifiPhy`,
  `ThresholdPreambleDetectionModel`, `YansWifiChannel`).
