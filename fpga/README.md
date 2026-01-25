# 🔧 Chronos FPGA Firmware

<p align="center">
  <img src="https://img.shields.io/badge/Device-CrossLink--NX-ED1C24?style=flat-square" alt="Device">
  <img src="https://img.shields.io/badge/Tool-Radiant%203.2+-blue?style=flat-square" alt="Tool">
  <img src="https://img.shields.io/badge/Language-SystemVerilog-orange?style=flat-square" alt="Language">
</p>

## Overview

This directory contains the FPGA firmware for the Chronos multi-camera synchronization system. The design aggregates four MIPI CSI-2 camera streams into a single output using virtual channel multiplexing.

## 📁 Directory Structure

```
fpga/
├── rtl/                        # RTL Source Files
│   ├── chronos_top.sv          # Top-level integration
│   ├── csi2_rx.sv              # CSI-2 receiver (×4 instances)
│   ├── csi2_tx.sv              # CSI-2 transmitter
│   ├── frame_buffer.sv         # SRAM line buffers
│   ├── tx_arbiter.sv           # Virtual channel multiplexer
│   ├── trigger_generator.sv    # Precision sync pulse generator
│   ├── chronos_pll.sv          # Clock generation
│   ├── i2c_slave.sv            # Configuration interface
│   └── config_regs.sv          # Register bank
│
├── constraints/                # Design Constraints
│   └── chronos_pinout.pdc      # Pin assignments & timing
│
└── tb/                         # Testbenches
    └── (simulation files)
```

## 🏗️ Architecture

```
        ┌─────────────────────────────────────────────────────┐
        │              CrossLink-NX FPGA                       │
        │                                                      │
CSI-2   │  ┌────────┐   ┌──────────┐   ┌────────────┐        │  CSI-2
Cam0 ───┼─▶│ RX[0]  │──▶│          │   │            │        │  Output
Cam1 ───┼─▶│ RX[1]  │──▶│  Frame   │──▶│    TX      │────────┼──▶
Cam2 ───┼─▶│ RX[2]  │──▶│  Buffers │   │  Arbiter   │        │  (VC0-3)
Cam3 ───┼─▶│ RX[3]  │──▶│          │   │            │        │
        │  └────────┘   └──────────┘   └────────────┘        │
        │                                                      │
        │  ┌────────────────┐     ┌─────────────────────────┐ │
        │  │    Trigger     │────▶│  To FSIN (×4) + IMU     │ │
        │  │   Generator    │     └─────────────────────────┘ │
        │  └───────┬────────┘                                 │
        │          │                                          │
        │  ┌───────▼────────┐     ┌──────────────┐           │
        │  │  Config Regs   │◀───▶│  I2C Slave   │◀──────────┼── I2C
        │  └────────────────┘     └──────────────┘           │
        │                                                      │
        └─────────────────────────────────────────────────────┘
```

## 🔑 Key Modules

| Module | Description | Resources |
|--------|-------------|-----------|
| `csi2_rx` | MIPI CSI-2 receiver with D-PHY | Hard IP |
| `frame_buffer` | 4K-entry SRAM FIFO | ~16KB SRAM |
| `tx_arbiter` | Round-robin VC mux | ~500 LUTs |
| `csi2_tx` | MIPI CSI-2 transmitter | Hard IP |
| `trigger_generator` | PLL-based sync pulse | ~200 LUTs |
| `config_regs` | 256-byte register bank | ~300 LUTs |

## 📋 Register Map

| Address | Name | Access | Description |
|---------|------|--------|-------------|
| `0x00` | CTRL | R/W | Control (enable, reset) |
| `0x01` | FRAME_RATE | R/W | Target fps (1-120) |
| `0x02-03` | PULSE_WIDTH | R/W | Trigger width |
| `0x04` | CAM_ENABLE | R/W | Camera enable mask |
| `0x10-13` | TRIG_DELAY[0-3] | R/W | Per-camera delays |
| `0x20` | STATUS | R | PLL lock, sync status |
| `0x21` | ERROR | R | Error flags |
| `0x30-37` | FRAME_CNT[0-3] | R | Frame counters |
| `0xF0` | VERSION | R | Firmware version |
| `0xFE-FF` | DEVICE_ID | R | Device ID (0xC405) |

## 🛠️ Building

### Prerequisites

- Lattice Radiant 3.2 or later
- CrossLink-NX device support package

### Build Steps

```bash
# Open project in Radiant GUI
radiantc chronos.rdf

# Or command-line synthesis
radiantc -t "CrossLink-NX" -d "LIFCL-40" \
         -p "QFN72" -s "7" \
         chronos_top.sv

# Generate bitstream
radiantc --bitstream chronos.bit
```

### Programming

```bash
# Program via JTAG
pgrcmd -infile chronos.bit
```

## 📊 Resource Utilization

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUTs | ~8,000 | 39,000 | 21% |
| Registers | ~4,000 | 39,000 | 10% |
| SRAM | 64 KB | 200 KB | 32% |
| D-PHY | 5 | 8 | 63% |
| PLL | 1 | 4 | 25% |

## ⚡ Timing

- System clock: 200 MHz
- MIPI data rate: 800 Mbps/lane
- Trigger skew: < 100 ns (20 cycles)

## 🧪 Simulation

```bash
cd tb
vsim -do run_tests.do
```

## 📝 Notes

- D-PHY IP is hardened in CrossLink-NX (no soft implementation)
- Use Radiant IP Catalog for DPHY_RX and DPHY_TX instantiation
- PLL configuration via IP Catalog (PLL_CORE primitive)
