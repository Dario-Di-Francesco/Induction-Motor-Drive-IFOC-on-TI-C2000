# Induction Motor Drive (IFOC) — MATLAB/Simulink + TI C2000 F28379D

This repository contains a MATLAB/Simulink project implementing an **induction motor drive** supplied by a **VSI inverter** and controlled via **Indirect Field Oriented Control (IFOC)** with **rotor-flux orientation**.  
The workflow includes a **simulation-only model** for validation and a **Host/Target setup** structured to match TI C2000 (F28379D) constraints.

> Project results and validation plots are included directly in this README (no standalone report).

---

## Repository Overview

### Simulink Models
This project includes **three Simulink models**:

- **SIM-only model**  
  Full drive simulation (plant + IFOC) used for validation, plotting, and tuning.  
  **Not** intended for deployment to the control board.

- **HOST model**  
  Generates the **speed reference** and **enable signal**, then **transmits them via SCIA** at **100 Hz**.

- **TARGET model**  
  Receives **speed reference + enable** via SCIA and runs the control algorithm with the required **ADC/PWM/ISR timing** (ADC EOC ISR, SOC from PWM1, etc.).

### MATLAB Scripts & Functions
- **Nameplate parameter back-calculation**: `scripts/Motor_data_IM_R.m`  
  Computes motor equivalent-circuit parameters + controller gains from rated/nameplate data (with assumptions).

- **Reference profile generation**:
  - `scripts/make_ref_profile.m` → builds the full reference sequence used by SIM and/or HOST
  - `scripts/w_a_profile_quintic_soft_2.m` → support function generating a **quintic smooth speed/acceleration profile** (called by `make_ref_profile.m`)

---

## Project Goals

- Implement **indirect rotor-flux-oriented control (IFOC)** with:
  - **rotor flux magnitude and phase estimation**
  - **decoupled d/q-axis current control** with PI regulators
  - **carrier-based PWM** with **min/max injection**
- Enable **field-weakening** for operation above nominal speed
- Ensure **start-up sequence** activates torque control **only after motor fluxing**
- Validate the system through the required speed transients and load law

---

## Required Simulation Scenarios

Run (in simulation) the following transients:

1. **Start-up:** 0 → nominal speed  
2. **Acceleration:** nominal speed → **2× nominal speed**  
3. **Speed inversion:** **2× nominal speed** → nominal speed with **opposite sign**

### Load Torque Law
Apply a load torque:
- proportional to the **cube of speed**: `T_load ∝ ω³`
- sized so the motor develops **rated power** when operating at **2× nominal speed**

---

## Parameter Identification (from nameplate)

Motor and controller parameters are derived from nameplate data using `scripts/Motor_data_IM_R.m`, assuming:
- negligible iron losses
- negligible mechanical losses
- realistic values for:
  - nominal efficiency
  - nominal magnetizing current
  - breakdown (pull-out) torque

### Model Initialization (Simulink Callback)

The parameter script `scripts/Motor_data_IM_R.m` is executed **automatically** when the Simulink model is initialized.

This is done via:  
**Model Properties → Callbacks → InitFcn**  
where the initialization command is:
`Motor_data_IM_R;`

![Simulink InitFcn callback calling Motor_data_IM_R.m](docs/figures/simulink_initFcn.png)

As a result, motor parameters, controller gains, and scaling constants are loaded into the MATLAB workspace before the simulation starts.

**Outputs (typical):**
- Equivalent-circuit parameters: `Rs`, `Rr'`, `Lm`, `Lσs`, `Lσr`
- Control gains (PI): current (d/q), flux/estimator loop, speed loop
- Limits and scalings (ADC/PWM, RMS vs peak conventions)

---

## Project Data

- Rated power: **33 kW**
- Rated line voltage: **400 V**
- Rated frequency: **50 Hz**
- Rated speed: **2950 rpm**
- Inertia: **0.05 kg·m²**
- DC-link voltage: **600 V**

---

## Control Overview (High-Level)

Typical signal flow:

1. Speed controller → generates `i_q*` (torque-producing current reference)
2. Flux / Field-Weakening logic → generates `i_d*` (flux current reference)
3. Current measurement → Clarke/Park transforms → `i_d, i_q`
4. Decoupled PI current controllers → `v_d*, v_q*`
5. Inverse transforms → three-phase voltage references
6. Carrier-based PWM + min/max injection → VSI gate commands

---

## TI C2000 F28379D Target Constraints (Model Integration)

The control algorithm is prepared for transfer to a **target Simulink model** for the **TI F28379D** with the following constraints.

### PWM Mapping (VSI Legs)
- **PWM1** → inverter leg 1  
- **PWM2** → inverter leg 2  
- **PWM3** → inverter leg 3  

### ADC Mapping (Measurements + Scaling)
- **ADCINA2:** phase-1 current (0 → -100 A, 4095 → 100 A)
- **ADCINB2:** phase-2 current (0 → -100 A, 4095 → 100 A)
- **ADCINC2:** speed (0 → -6000 rpm, 4095 → 6000 rpm)

### Timing / Triggering
- Execute the main control loop inside an **ISR** triggered by **ADC End of Conversion (EOC)**
- Generate ADC **Start of Conversion (SOC)** from **PWM1** at **counter = 0**
- Configure PWM in **up/down mode** to achieve **10 kHz sampling frequency**

### Digital Inputs via SCIA
- The ISR must receive two inputs:
  - **speed reference**
  - **control enable**
- Acquire these inputs via serial communication using **SCIA** in a **100 Hz synchronous task**

---

## Suggested Repository Structure

Recommended layout (rename files to match your project):
