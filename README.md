# High Speed Data Acquisition Controller

A multi-channel biosignal acquisition controller implemented in SystemVerilog, designed for bedside patient monitoring (ECG/EEG/EMG). Verified with SystemVerilog Assertions, constrained-random testbenches, and functional coverage using Questa Sim.

---

## Overview

| Item | Detail |
|---|---|
| Language | SystemVerilog |
| Channels | 8 (ECG ×4, EEG ×2, EMG ×2) |
| Arbitration | Round-Robin / Priority / Weighted / Dynamic |
| Output Interface | SPI (bidirectional, 1 MHz) |
| System Clock | 100 MHz |
| FIFO Depth | 16 entries, 2-level watermark |
| Simulation Tool | Questa Sim |

---

## Module Summary

| Module | Role |
|---|---|
| `configurable_arbiter` | Selects which channel to sample next. Supports 4 modes, configurable via SPI registers. |
| `single_fifo` | 16-entry FIFO with `almost_full` flag (threshold=12) to decouple ADC sampling from SPI output. |
| `derivative_threshold_engine` | Detects signal anomalies (e.g., sudden ECG amplitude spikes) using a sliding window false-positive filter. |
| `performance_monitor` | Tracks throughput (samples/sec), per-channel latency, and FIFO utilization. Asserts warning flags at thresholds. |
| `high_speed_daq_controller` | Top-level wrapper. Integrates all modules, exposes SPI slave interface for configuration and data readout. |

### Arbitration Modes

| Mode | Encoding | Behavior |
|---|---|---|
| Round-Robin | `2'b00` | Cycles through enabled channels in order. Guarantees max wait = N−1 cycles (starvation-free). |
| Priority | `2'b01` | Always selects the highest-priority enabled channel. Risk of starvation for low-priority channels. |
| Weighted | `2'b10` | ECG channels (weight=2) selected ~2× more often than EEG/EMG (weight=1). Default mode. |
| Dynamic | `2'b11` | Falls back to Weighted; immediately promotes any channel with `channel_urgent=1` asserted. |

**Design rationale**: ECG sampling rate (500 Hz) is 2× EEG/EMG (250 Hz), so `weight=2` for ECG naturally encodes this clinical priority in hardware.

### FIFO Design

Depth=16 is chosen to cover one full weighted-round (4×ECG + 2×EEG + 2×EMG = 12 slots minimum) plus margin. `almost_full` at count≥12 signals backpressure before overflow. A 2-flop synchronizer converts the SPI-domain read-enable pulse to a single-cycle pulse in the 100 MHz domain.

---

## Verification

### Unit Testbenches

#### `tb_single_fifo.sv` — `single_fifo` unit test

| Item | Result |
|---|---|
| Scoreboard checks | 279 / 279 pass |
| Overflow attempts observed | 5 |
| Underflow attempts observed | 42 |
| Simultaneous R/W cycles | 133 |
| SVA assertions | 8 / 8 pass |
| Functional coverage | **100%** |

**Scenarios**: overflow (write beyond depth), underflow (read from empty), simultaneous read/write (20-cycle steady-state), reset with inputs asserted, constrained-random (500 cycles).

**SVA (8 assertions)**:
- A1: `count` always in [0, FIFO_DEPTH]
- A2: `wr_full ↔ (count == FIFO_DEPTH)`
- A3: `rd_empty ↔ (count == 0)`
- A4: `almost_full ↔ (count >= 12)`
- A5: No overflow — count unchanged when `wr_full` and no simultaneous read
- A6: No underflow — count unchanged when `rd_empty` and no simultaneous write
- A7: Count changes by at most ±1 per cycle
- A8: After reset, `count == 0`, `rd_empty == 1`, `wr_full == 0`

#### `tb_configurable_arbiter.sv` — `configurable_arbiter` unit test

| Item | Result |
|---|---|
| All directed scenarios | PASS |
| SVA assertions | 9 / 9 pass |
| Functional coverage | **100%** |

**Scenarios**: reset with all inputs asserted, all-8-channel simultaneous request (across all 4 modes), all 16 mode-transition combinations, Dynamic mode urgent priority, `adc_busy` blocking, constrained-random (300 cycles).

**SVA (9 assertions)**:
- A1: `selected_channel` always in [0, NUM_CHANNELS−1]
- A2: `channel_valid=1` → selected channel is `enable && ready`
- A3: `adc_busy=1` → `channel_valid=0`
- A4: No enabled+ready channel → `channel_valid=0`
- A5: After reset, `rr_counter == 0`
- A6: After reset, all `weight_accumulator == 0`
- A7: In Round-Robin mode, `rr_counter` increments by 1 (mod N) on each `channel_accept`
- A8: In Dynamic mode with urgent channels, selected channel must have `channel_urgent=1`
- A9: In Priority mode, `channel_priority[selected_channel] == max` over all enabled+ready channels

#### `tb_top_spi.sv` — End-to-end SPI datapath test

| Item | Result |
|---|---|
| Scoreboard checks | 16 / 16 pass |
| SPI write (config) | Verified: `channel_enable`, `arbiter_mode` registers |
| SPI read (FIFO) | Verified: 16 entries match expected `{channel, counter}` format |

Tests SPI write→register→ADC→FIFO→SPI read full datapath, including CDC boundary.

### Coverage Report

Generated with Questa Sim `vcover`:

```
Covergroup Coverage:   100.00%
Assertion Coverage:    100.00%  (17 assertions, 0 failures)
Statement Coverage:     91.91%  (untouched branches are error paths never triggered — expected)
```

Untouched branches (scoreboard mismatch, FAIL outputs) are intentionally unreachable in a bug-free design.

---

## Running the Simulation

### Requirements

- Questa Sim

### FIFO unit test

```tcl
vlog -sv high_speed_daq_controller.sv
vlog -sv +cover=bcestf tb_single_fifo.sv
vsim -voptargs="+acc" -coverage tb_single_fifo
coverage save -onexit tb_single_fifo.ucdb
run -all
```

```bash
vcover report tb_single_fifo.ucdb -details -output fifo_coverage.txt
```

### Arbiter unit test

```tcl
vlog -sv high_speed_daq_controller.sv
vlog -sv +cover=bcestf tb_configurable_arbiter.sv
vsim -voptargs="+acc" -coverage tb_configurable_arbiter
coverage save -onexit tb_configurable_arbiter.ucdb
run -all
```

```bash
vcover report tb_configurable_arbiter.ucdb -details -output arbiter_coverage.txt
```

### SPI test

```tcl
vlog -sv high_speed_daq_controller.sv
vlog -sv tb_top_spi.sv
vsim -voptargs="+acc" tb_top_spi
run -all
```
