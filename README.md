# High-Speed Data Acquisition Controller

**VLSI Design Project | Fall 2024**

A fully synthesizable, multi-channel data acquisition system designed for real-time biomedical signal processing with advanced arbitration, buffering, and trigger detection capabilities.

---

## Project Overview

This project implements a configurable 16-channel data acquisition controller optimized for high-speed sampling applications. The design features a sophisticated arbitration system, intelligent buffering with backpressure control, derivative-based trigger detection, and comprehensive performance monitoring.

**Target Application:** Real-time biomedical signal acquisition and processing  
**Technology:** Verilog/SystemVerilog, fully simulation-verified  
**Verification Coverage:** 70%+ across all modules

---

## Design Goals

- **High Throughput:** Support 16 simultaneous ADC channels at configurable sampling rates
- **Flexible Arbitration:** Four arbitration modes (Round-Robin, Priority, Weighted, Dynamic/Urgent)
- **Intelligent Buffering:** 672-entry FIFO with adaptive flow control and level-based warnings
- **Smart Triggering:** Derivative threshold engine with false-positive filtering
- **Real-time Monitoring:** Performance metrics tracking (throughput, latency, utilization)
- **Low Latency:** Optimized datapath with minimal cycle overhead
- **Configurability:** SPI-based runtime reconfiguration

---
## System Architecture

---

## Module Descriptions

### 1. Configurable Arbiter (`configurable_arbiter.sv`)
**Coverage: 71%**

Selects the next ADC channel based on configurable arbitration policy.

**Features:**
- **Mode 0 (Round-Robin):** Fair sequential channel selection
- **Mode 1 (Priority-Based):** Highest priority channel wins, tie-breaking logic
- **Mode 2 (Weighted):** Accumulated weight comparison with automatic reset
- **Mode 3 (Dynamic/Urgent):** Urgent channels preempt, fallback to weighted

**Key Signals:**
- `arbiter_mode[1:0]`: Selects arbitration algorithm
- `channel_priority[3:0][15:0]`: Per-channel priority values (Mode 1)
- `channel_weight[7:0][15:0]`: Per-channel weights (Mode 2/3)
- `channel_urgent[15:0]`: Urgent channel mask (Mode 3)

### 2. Multi-Level FIFO (`single_fifo.sv`)
**Coverage: 70%+**

Adaptive buffering with three warning levels and backpressure control.

**Features:**
- **Depth:** 672 entries (16-bit data)
- **Level Detection:** L1 (<32), L2 (32-160), L3 (>160)
- **Backpressure:** Activates at 90% capacity
- **Overflow Protection:** Level-specific overflow flags

**Performance:**
- Single-cycle write/read when not full/empty
- Automatic backpressure signaling prevents data loss

### 3. Derivative Threshold Engine (`derivative_threshold_engine.sv`)
**Coverage: 70%+**

Detects rapid signal changes using derivative analysis.

**Features:**
- **Dual Detection:** Amplitude threshold + derivative threshold
- **Per-Channel History:** Tracks previous samples for all 16 channels
- **False Positive Filter:** Configurable trigger rate limiting
- **Confidence Calculation:** 8-bit confidence score based on amplitude and derivative magnitude
- **Overflow Handling:** Saturating arithmetic prevents spurious triggers

**Configuration:**
- `CFG_THRESHOLD_LOW`: Amplitude threshold
- `CFG_THRESHOLD_HIGH`: Derivative threshold
- `CFG_FILTER_WINDOW`: Max triggers per time window

### 4. Performance Monitor (`performance_monitor.sv`)
**Coverage: 70%+**

Real-time system metrics and health monitoring.

**Metrics Tracked:**
- **Throughput:** Samples per second (SPS)
- **Latency:** Average and maximum (nanoseconds)
- **FIFO Utilization:** Percentage fullness
- **Trigger Rate:** Parts per million (ppm)

**Warning Flags:**
- Low throughput, High latency
- High FIFO usage, Overflow detected
- Abnormal trigger rate
- ADC timeout, System overload

### 5. ADC Interface Controller (`high_speed_daq_controller.sv`)

Main controller with SPI configuration interface.

**Features:**
- **SPI Slave:** Configuration registers with CDC (Clock Domain Crossing)
- **State Machine:** Manages ADC conversion flow with settling time
- **Serial Output:** Bit-serial data transmission
- **Integration:** Connects all submodules

**SPI Register Map:**
- `0x0000`: System enable
- `0x0004`: Channel enable mask
- `0x0008`: Arbiter mode
- `0x000C`: Urgent channel mask
- `0x0020-0x003C`: Channel priorities (Mode 1)
- `0x0040-0x005C`: Channel weights (Mode 2/3)

---
