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
| MAC Array | 8×4 channel-parallel dot product, INT12×INT8→INT32 |
| Alert | `mac_alert` pulse when anomaly score > `mac_threshold` |
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
## MAC Array — Real-time Anomaly Detection

### Overview

A channel-parallel MAC array is integrated directly into the acquisition pipeline to compute per-channel anomaly scores in real time, without involving the external MCU.

| Item | Detail |
|---|---|
| Input bit-width | 12 bit (unsigned) — matches `ADC_WIDTH` exactly, no conversion |
| Weight bit-width | 8 bit (signed) — pre-trained anomaly pattern coefficients |
| Output bit-width | 32 bit (signed) — per-channel anomaly score |
| Array size | 8 × 4 (= `NUM_CHANNELS` × `WINDOW_SIZE`) |
| Operation | Per-channel independent dot product (not systolic, not full matrix multiply) |
| Latency | 6 cycles from `valid_in` to `valid_out` (1 clear + 4 accumulate + 1 registered) |

### Why not systolic array

Each channel's dot product is fully independent — no data is shared between channels, and the pre-trained weights are fixed (no weight reuse across different input windows in the systolic sense). A systolic array's PE-to-PE forwarding only pays off when one weight must reach many PEs over time, which does not apply here. A parallel MAC bank achieves the same result with lower wiring complexity and lower latency.

The FIFO datapath and SPI interface are completely unchanged. The MAC array taps the FIFO write path passively — every time a new ADC sample is written to the FIFO, the corresponding channel's sliding window is updated. When a channel's window fills to 4 samples, `mac_valid_in` is asserted and the array computes a fresh dot product for all 8 channels simultaneously.

### New ports

| Port | Direction | Width | Description |
|---|---|---|---|
| `mac_weight` | input | `[8][4]` signed 8-bit | Anomaly pattern weights (held fixed, e.g. loaded via SPI config write at startup) |
| `mac_threshold` | input | 32-bit | Alert threshold for signed anomaly score comparison |
| `mac_alert` | output | 1-bit | 1-cycle pulse when any channel's score exceeds `mac_threshold` |

### Verification

Python golden model (`golden_model.py`) generates 1000 random `(input, weight)` pairs, computes INT32 dot products as reference, and writes them to `golden.hex`. The RTL testbench reads the same hex files, drives the MAC array, and compares results:

```
MAC array test: checks=8000 mismatches=0
MAC ARRAY TEST PASSED (1000/1000 test vectors, 0 mismatches)
```

**Key design decisions verified by overflow analysis** (see `golden_model.py`):
- Max single product: 4,095 × 127 = 520,065
- Max accumulated (4 taps): 2,080,260
- INT32 range: ±2,147,483,648 → **INT32 accumulator safe** 

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
