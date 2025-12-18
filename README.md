# Buck_SMC_ElectroThermal_FullModel

## Overview

This repository contains a **high-fidelity MATLAB simulation of a Buck DC–DC converter** incorporating:

- **Digital Sliding Mode Control (SMC)** with boundary layer for chattering reduction
- **Full loss modeling**:
  - Inductor copper losses (including ESR)
  - MOSFET conduction and switching losses
  - Diode conduction losses
- **Electro-thermal modeling**:
  - MOSFET and Diode junction temperature using RC thermal model
  - Real-time and steady-state junction temperature computation
- **Comprehensive performance analysis**:
  - Output voltage regulation and ripple
  - Inductor current and RMS values
  - Power loss breakdown
  - Overall converter efficiency
  - Maximum junction temperatures

This simulation provides an **industrial-grade platform** suitable for research, R&D, embedded system prototyping, and academic publications.

---

## Features

- Standalone MATLAB implementation (no additional toolboxes required)
- Accurate modeling of non-idealities in power electronics components
- Electro-thermal co-simulation for component stress and lifetime analysis
- Full performance reporting with professional plots and console output
- Scalable for MPC, thermal management studies, and DSP/FPGA deployment

---

## File Description

| File | Description |
|------|-------------|
| `Buck_SMC_ElectroThermal_FullModel.m` | Complete MATLAB simulation of Buck converter with SMC, losses, and electro-thermal modeling |

---

## System Specifications

- Input voltage: `Vin = 24 V`
- Output voltage reference: `Vref = 12 V`
- Load resistance: `Rload = 10 Ω`
- Inductor: `L = 150 µH` with ESR `RL = 0.08 Ω`
- Output capacitor: `C = 220 µF`
- MOSFET: `Rds(on) = 0.04 Ω`
- Diode forward voltage: `Vd = 0.7 V`
- Switching frequency: `fsw = 50 kHz`
- SMC coefficient: `λ = 800`
- Boundary layer thickness: `η = 0.01`
- Thermal resistances: `Rth_jc_MOS = 0.5 °C/W`, `Rth_jc_D = 2 °C/W`
- Thermal capacitances: `Cth_MOS = 5 mJ/°C`, `Cth_D = 2 mJ/°C`
- Ambient temperature: `T_ambient = 25 °C`

---

## Simulation Details

1. **Sliding Mode Control (SMC)**:
   - Digital SMC implemented for regulating the output voltage
   - Boundary layer used to reduce chattering in the switching signal

2. **Buck Converter Dynamics**:
   - Continuous-time Euler integration for inductor current and output voltage
   - Non-idealities included: inductor ESR, MOSFET Rds(on), diode voltage drop

3. **Loss Calculations**:
   - Inductor copper loss: `P_L = I_L^2 * RL`
   - MOSFET conduction loss: `P_MOS = D*I_L^2*Rds`
   - Diode conduction loss: `P_D = (1-D)*I_out*Vd`
   - Switching loss: `P_sw = 0.5 * Vin * I_L * (tr + tf) * fsw`

4. **Electro-Thermal Modeling**:
   - Junction temperatures computed using RC thermal network
   - Dynamic temperature update per timestep
   - Steady-state and maximum junction temperatures reported

---

## Usage

1. Clone the repository:

```bash
git clone https://github.com/yourusername/Buck_SMC_ElectroThermal_FullModel.git
