# Mode-Switching Hardware PID Controller for Copter Flight Control

[![Verilog-2001](https://img.shields.io/badge/HDL-Verilog--2001-blue.svg)](http://www.verilog.com/)
[![Simulation](https://img.shields.io/badge/Simulation-ModelSim%20%7C%20NC--Sim-brightgreen.svg)](docs/verification_plan.md)
[![Synthesis](https://img.shields.io/badge/Synthesis-Cadence%20Genus-red.svg)](docs/physical_design.md)
[![Physical Design](https://img.shields.io/badge/PnR-Cadence%20Innovus-orange.svg)](docs/physical_design.md)
[![DRC/Connectivity](https://img.shields.io/badge/DRC%20%2F%20Conn-0%20Violations-success.svg)](docs/physical_design.md)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An industry-grade, synthesizable **Verilog-2001 RTL-to-GDS** implementation of a **Mode-Switching Proportional-Integral-Derivative (PID) Flight Controller** developed for the *Silicon Dawn VLSI Hackathon 2026 (Problem Statement PS-02 by Sudarshana Semiconductors)* at VIT Chennai.

The architecture implements a dual-loop cascaded attitude (outer) and angular rate (inner) controller for quadcopter flight systems, featuring signed **Q8.8 fixed-point arithmetic**, 4 flight operating modes (**Acro**, **Stabilize**, **Altitude Hold**, and **Failsafe**), **bumpless transfer** integrator compensation, integral **anti-windup**, low-pass **derivative filtering**, a programmable **hardware watchdog**, memory-mapped configuration registers, and an X-configuration motor mixer. The design was verified through self-checking testbenches and taken through technology mapping in **Cadence Genus** and full physical implementation in **Cadence Innovus**, achieving timing closure at **50 MHz** and zero DRC/connectivity violations across a **446.31 µm × 444.28 µm** core.

---

## Table of Contents

1. [Problem Statement & Engineering Motivation](#1-problem-statement--engineering-motivation)
2. [Key Architecture Features](#2-key-architecture-features)
3. [System Architecture & Control Hierarchy](#3-system-architecture--control-hierarchy)
4. [Control Theory & Numerical Implementation](#4-control-theory--numerical-implementation)
5. [Flight Operating Modes & Switching FSM](#5-flight-operating-modes--switching-fsm)
6. [RTL Module Inventory & Hierarchy](#6-rtl-module-inventory--hierarchy)
7. [Hardware Interfaces & Memory Map](#7-hardware-interfaces--memory-map)
8. [Functional Verification & Simulation Results](#8-functional-verification--simulation-results)
9. [Synthesis & Physical Design (RTL-to-GDS Flow)](#9-synthesis--physical-design-rtl-to-gds-flow)
10. [Closed-Loop Flight Performance Analysis](#10-closed-loop-flight-performance-analysis)
11. [Repository Organization](#11-repository-organization)
12. [Tool Dependencies & Reproduction Steps](#12-tool-dependencies--reproduction-steps)
13. [Limitations & Gaps](#13-limitations--gaps)
14. [Future Engineering Improvements](#14-future-engineering-improvements)
15. [Authors & Academic Attribution](#15-authors--academic-attribution)
16. [License](#16-license)

---

## 1. Problem Statement & Engineering Motivation

Modern multirotor UAV flight control firmware requires tightly nested feedback loops executed at high update frequencies (1 kHz to 8 kHz). Microcontroller-based flight controllers (running software loops on ARM Cortex-M cores) encounter significant limitations:
- **Execution Jitter:** Interrupt service routines (ISRs), RTOS scheduling, and sensor bus transactions introduce non-deterministic execution jitter into inner rate loops.
- **Latency Bottlenecks:** Sequential computation of multiple PID channels increases sensor-to-actuator latency, degrading phase margin and control bandwidth during high-rate aerobatic maneuvers.
- **Processor Overhead:** Dedicated microcontrollers consume power executing repetitive arithmetic iterations that are ideally suited for parallel hardware execution.

**Project Objective (PS-02):** Design a dedicated, fully synthesizable Verilog hardware core that offloads the entire cascaded flight control loop, achieving deterministic sub-microsecond cycle latency, hardware-enforced bumpless mode transfer, autonomous sensor fault failsafe handling, and zero arithmetic wraparound, validated through physical tapeout-ready layout.

---

## 2. Key Architecture Features

- **Cascaded Dual-Loop Topology:** Outer attitude loop converts Euler angle error to body rate setpoints; inner rate loop converts angular velocity error to actuator mixing commands across Roll, Pitch, and Yaw axes.
- **Unified Signed Q8.8 Arithmetic:** 16-bit 2's complement representation (1 sign, 7 integer, 8 fractional bits; resolution $\Delta = 0.0039$, range $[-128.0, +127.996]$) with 32-bit Q16.16 intermediate products to completely eliminate silent overflow.
- **Glitch-Free FSM Mode Controller:** Supports 4 discrete flight modes (`ACRO`, `STABILIZE`, `ALT_HOLD`, `FAILSAFE`) with registered, race-free state transitions.
- **Bumpless Mode Transfer:** Dynamically recalibrates the integral accumulator ($I_{new} = u_{prev} - P_{new} - D_{new}$) during mode transitions, eliminating torque transients and actuator shock.
- **Integral Anti-Windup Clamping:** Dynamically clamps the integrator when actuator saturation occurs, preventing integrator windup and overshoot during large step maneuvers.
- **Filtered Derivative on Measurement:** Computes the derivative action strictly from IMU gyro feedback ($-\alpha \cdot \Delta y$) rather than error ($e = r - y$), preventing setpoint derivative kick during rapid pilot stick inputs.
- **Autonomous Watchdog & Failsafe:** Hardware timeout monitor checks `sensor_valid` strobes; triggers an autonomous controlled descent policy if telemetry drops for $> 500,000$ clock cycles.
- **Synchronous Configuration Register File:** Standard memory-mapped 8-bit address bus for runtime PID gain tuning, tick rate selection, and telemetry readback.

---

## 3. System Architecture & Control Hierarchy

```mermaid
flowchart TD
    subgraph SENSORS ["IMU Telemetry Inputs (Signed Q8.8)"]
        IMU_ATT["Euler Angles (roll, pitch, yaw)"]
        IMU_RATE["Gyro Rates (p, q, r)"]
        IMU_ALT["Baro / ToF (altitude, vert_vel)"]
        SENS_V["sensor_valid strobe"]
    end

    subgraph CONFIG ["Configuration & Safety Engine"]
        REG_FILE["config_registers<br/>(Memory Mapped 0x00-0x0F)"]
        GAIN_BANK["gain_bank<br/>(Per-Mode PID Constants)"]
        WD_TIMER["watchdog_timer<br/>(Sensor Timeout Monitor)"]
        MODE_CTRL["mode_controller<br/>(FSM: Acro, Stab, AltHold, Fail)"]
        FAIL_CTRL["failsafe_controller<br/>(Emergency Descent Policy)"]
    end

    subgraph OUTER_LOOP ["Attitude Loop (Outer)"]
        ATT_CTRL["attitude_controller<br/>(Roll/Pitch P-Loops)"]
        ALT_CHAN["Altitude Hold Channel<br/>(P-Controller + Velocity Trim)"]
    end

    subgraph BUMPLESS ["Bumpless Transfer Engine"]
        BUMP_R["bumpless_transfer (Roll)"]
        BUMP_P["bumpless_transfer (Pitch)"]
        BUMP_Y["bumpless_transfer (Yaw)"]
    end

    subgraph INNER_LOOP ["Rate Loop (Inner - 1-8 kHz)"]
        PID_ROLL["pid_controller (Roll)"]
        PID_PITCH["pid_controller (Pitch)"]
        PID_YAW["pid_controller (Yaw)"]
    end

    subgraph ACTUATION ["Actuator Interface"]
        MIXER["actuator_interface<br/>(Quadcopter X-Mixer Matrix)"]
        MOTORS["Motor Commands (M0, M1, M2, M3)"]
    end

    %% Dataflow Connections
    IMU_ATT --> ATT_CTRL
    SENS_V --> WD_TIMER
    WD_TIMER --> MODE_CTRL
    MODE_CTRL --> GAIN_BANK
    MODE_CTRL --> BUMPLESS
    REG_FILE <--> GAIN_BANK

    ATT_CTRL --> BUMPLESS
    IMU_ALT --> ALT_CHAN
    BUMPLESS --> INNER_LOOP
    IMU_RATE --> INNER_LOOP

    INNER_LOOP --> MIXER
    ALT_CHAN --> MIXER
    FAIL_CTRL -.->|Override on Timeout| MIXER
    MIXER --> MOTORS
```

### High-Level Architectural Block Diagram
![Top-Level Architecture](docs/figures/architecture/block_diagram.png)

---

## 4. Control Theory & Numerical Implementation

### 4.1 PID Control Law
The hardware implements an advanced parallel PID formulation tailored for real-time digital control:

$$u(t) = P(t) + I(t) + D(t)$$

1. **Proportional Term:** Direct linear gain applied to tracking error:
   $$P[k] = K_p \cdot e[k], \quad e[k] = r[k] - y[k]$$
2. **Integral Term with Anti-Windup:** Discrete accumulation executed in 32-bit Q16.16 arithmetic to prevent overflow:
   $$I[k] = \begin{cases} I[k-1] + K_i \cdot e[k] \cdot T_s, & \text{if not saturated} \\ I[k-1], & \text{if saturated and } \text{sgn}(e) = \text{sgn}(u) \end{cases}$$
3. **Filtered Derivative on Feedback:** Computed exclusively on measured rate $y[k]$ passing through a first-order low-pass filter to reject sensor noise:
   $$D_{\text{raw}}[k] = -K_d \cdot \frac{y[k] - y[k-1]}{T_s}$$
   $$D_{\text{filt}}[k] = D_{\text{filt}}[k-1] + \alpha \cdot \left( D_{\text{raw}}[k] - D_{\text{filt}}[k-1] \right)$$

### 4.2 Bumpless Transfer Mechanics
When switching flight modes (e.g., pilot switching from manual `ACRO` to auto-level `STABILIZE`), the instantaneous output must maintain continuity to prevent sudden airframe pitch/roll kicks. The `bumpless_transfer` module captures previous actuator output $u_{\text{prev}}$ and preloads the integrator state:

$$I_{\text{transferred}} = u_{\text{prev}} - P_{\text{new}} - D_{\text{new}}$$

### 4.3 Number Representation & Arithmetic Saturation
All dynamic quantities adhere to **Signed Q8.8 Fixed-Point**:
- **Format:** `[15]` Sign bit, `[14:8]` 7 Integer bits, `[7:0]` 8 Fractional bits.
- **Multiplication:** $16\text{-bit} \times 16\text{-bit} \rightarrow 32\text{-bit Q16.16}$ product. The product is shifted right by 8 bits, checked for upper-bit saturation (`31:23`), and clamped to `16'h7FFF` ($+127.996$) or `16'h8000` ($-128.0$).
- **Zero Silent Wraparound:** Dedicated arithmetic primitives (`sat_mult_q88`, `sat_add_sub_q88`, `clamp_bounds_q88`) prevent numerical overflow.

---

## 5. Flight Operating Modes & Switching FSM

| Flight Mode | Mode Code (`2'b`) | Outer Loop Status | Inner Loop Status | Control Behavior |
|:---|:---:|:---:|:---:|:---|
| **ACRO** | `00` | **Bypassed** | Active (Rate PID) | Raw angular rate control; pilot commands stick rates (°/s) directly. High maneuverability. |
| **STABILIZE** | `01` | **Active** (Attitude P) | Active (Rate PID) | Self-leveling; pilot commands absolute Euler angles (°); outer loop generates rate commands. |
| **ALTITUDE_HOLD** | `10` | **Active** (Attitude P) | Active (Rate PID) | Self-leveling plus vertical velocity/barometric altitude hold channel trimming throttle. |
| **FAILSAFE** | `11` | **Overridden** | **Overridden** | Autonomous safety policy; forces level attitude and constant 25% motor duty for soft landing. |

### FSM State Transition Priority
1. **Reset / Power-On:** Defaults automatically to `STABILIZE` (`2'b01`) with zero telemetry alarms.
2. **Hardware Watchdog Override:** Stale sensor condition automatically forces `FAILSAFE` (`2'b11`), overriding pilot inputs.
3. **Sensor Recovery:** Fresh sensor samples automatically clear the watchdog alarm and return the system to the commanded pilot mode via bumpless transfer.

---

## 6. RTL Module Inventory & Hierarchy

The synthesizable design comprises **15 Verilog-2001 files** defining **17 modules** (~2,030 lines):

```text
ps02_top.v (Top-Level System Integration)
├── control_tick_generator.v       (Generates 1, 2, 4, 8 kHz tick strobes from 50 MHz clk)
├── watchdog_timer.v               (500,000-cycle sensor watchdog timer)
├── mode_controller.v              (FSM flight mode manager with glitch-free decoding)
├── gain_bank.v                    (Per-mode storage for Kp, Ki, Kd, and limits)
├── config_registers.v             (Synchronous 8-bit memory-mapped register file)
├── attitude_controller.v          (Outer loop: 2x sat_mult_q88 for Roll/Pitch P-terms)
│   ├── sat_mult_q88 (mult_att_roll)
│   └── sat_mult_q88 (mult_att_pitch)
├── bumpless_transfer.v [3 inst]   (Roll, Pitch, Yaw integrator preloading engines)
├── rate_controller.v              (Inner loop: 3x 3-axis PID controllers)
│   ├── pid_controller (pid_roll)
│   │   ├── derivative_filter.v    (First-order low-pass IIR filter: mult_alpha)
│   │   ├── anti_windup.v          (Integrator clamping and saturation detection)
│   │   ├── sat_mult_q88 (mult_p)
│   │   ├── sat_mult_q88 (mult_i)
│   │   └── sat_mult_q88 (mult_d)
│   ├── pid_controller (pid_pitch) [same submodules]
│   └── pid_controller (pid_yaw)   [same submodules]
├── sat_mult_q88 (mult_alt_kp)     (Altitude hold proportional gain channel)
├── sat_mult_q88 (mult_vel_kp)     (Vertical climb velocity damping channel)
├── actuator_interface.v           (Quadcopter X-configuration motor mixer matrix)
└── failsafe_controller.v          (Enforces emergency descent policy)
```

---

## 7. Hardware Interfaces & Memory Map

### 7.1 Top-Level Port List (`ps02_top.v`)

| Port Group | Port Name | Direction | Width | Type | Description |
|:---|:---|:---:|:---:|:---:|:---|
| **System** | `clk` | Input | 1 | Clock | Primary system clock (50 MHz) |
| | `rst` | Input | 1 | Reset | Synchronous active-high reset |
| **IMU Telemetry** | `sensor_valid` | Input | 1 | Strobe | Asserted for 1 cycle on fresh sensor sample |
| | `roll_angle_meas` | Input | 16 | Signed Q8.8 | Euler roll angle measurement |
| | `pitch_angle_meas` | Input | 16 | Signed Q8.8 | Euler pitch angle measurement |
| | `yaw_angle_meas` | Input | 16 | Signed Q8.8 | Euler yaw angle measurement |
| | `roll_rate_meas` | Input | 16 | Signed Q8.8 | Gyroscope body roll rate ($p$) |
| | `pitch_rate_meas` | Input | 16 | Signed Q8.8 | Gyroscope body pitch rate ($q$) |
| | `yaw_rate_meas` | Input | 16 | Signed Q8.8 | Gyroscope body yaw rate ($r$) |
| | `altitude_meas` | Input | 16 | Signed Q8.8 | Barometer / ToF altitude measurement |
| | `vert_vel_meas` | Input | 16 | Signed Q8.8 | Vertical climb rate measurement |
| **Pilot Setpoints**| `roll_sp` | Input | 16 | Signed Q8.8 | Roll angle setpoint (Rate setpoint in ACRO) |
| | `pitch_sp` | Input | 16 | Signed Q8.8 | Pitch angle setpoint (Rate setpoint in ACRO)|
| | `yaw_sp` | Input | 16 | Signed Q8.8 | Yaw rate setpoint |
| | `altitude_sp` | Input | 16 | Signed Q8.8 | Altitude setpoint (ALT_HOLD mode) |
| | `throttle_base` | Input | 16 | Signed Q8.8 | Base collective throttle (0.0% to 100.0%) |
| **Flight Mode** | `mode_cmd` | Input | 2 | Enum | Requested flight mode (`00`:Acro, `01`:Stab, `10`:Alt, `11`:Fail) |
| | `mode_cmd_valid` | Input | 1 | Strobe | Mode change strobe |
| **Config Bus** | `reg_wr_en`, `reg_rd_en` | Input | 1 | Strobe | Synchronous write/read enable strobes |
| | `reg_addr` | Input | 8 | Address | 8-bit register index |
| | `reg_wr_data` | Input | 16 | Data | 16-bit register write payload |
| | `reg_rd_data` | Output | 16 | Data | 16-bit register readback payload |
| | `reg_ack` | Output | 1 | Strobe | Synchronous transfer acknowledge |
| **Actuators** | `motor0_cmd` | Output | 16 | Q8.8 | Front-Right Motor Duty (0.0% to 100.0%) |
| | `motor1_cmd` | Output | 16 | Q8.8 | Back-Left Motor Duty (0.0% to 100.0%) |
| | `motor2_cmd` | Output | 16 | Q8.8 | Front-Left Motor Duty (0.0% to 100.0%) |
| | `motor3_cmd` | Output | 16 | Q8.8 | Back-Right Motor Duty (0.0% to 100.0%) |
| | `motor_valid` | Output | 1 | Strobe | Asserted when new motor commands update |
| **Telemetry** | `current_mode` | Output | 2 | Enum | Active operating mode |
| | `failsafe_active` | Output | 1 | Flag | Asserted when failsafe descent is enforced |
| | `watchdog_alarm` | Output | 1 | Flag | Asserted on sensor timeout ($> 500\text{k cycles}$) |
| | `saturation_flags` | Output | 8 | Flags | Actuator and integrator saturation bitmap |
| | `bumpless_active` | Output | 1 | Strobe | Strobe during integrator transfer recalculation |

### 7.2 Configuration Register Map (`config_registers.v`)

| Address (`reg_addr`) | Name | Access | Bitfields & Functionality |
|:---:|:---|:---:|:---|
| `0x00` | `SYS_CONTROL` | R/W | `[0]` Core Enable, `[1]` Mode Override Enable, `[3:2]` Override Mode (`00`..`11`) |
| `0x01` | `SYS_STATUS` | R | `[1:0]` Active Mode, `[2]` Watchdog Alarm, `[3]` Failsafe Active, `[7:4]` Saturation Flags |
| `0x02` | `TICK_RATE_SEL`| R/W | `[2:0]` 000: 1 kHz, 001: 2 kHz, 010: 4 kHz, 011: 8 kHz, 1xx: Fast test strobe |
| `0x03` | `WATCHDOG_HI` | R/W | `[15:0]` Watchdog cycle threshold upper 16 bits |
| `0x04` | `WATCHDOG_LO` | R/W | `[15:0]` Watchdog cycle threshold lower 16 bits (Default: `500,000` cycles) |
| `0x05` | `STAB_ROLL_KP` | R/W | Signed Q8.8 Proportional gain for Stabilize Roll |
| `0x06` | `STAB_PITCH_KP`| R/W | Signed Q8.8 Proportional gain for Stabilize Pitch |
| `0x07` | `RATE_ROLL_KP` | R/W | Signed Q8.8 Rate Loop Roll Kp |
| `0x08` | `RATE_ROLL_KI` | R/W | Signed Q8.8 Rate Loop Roll Ki |
| `0x09` | `RATE_ROLL_KD` | R/W | Signed Q8.8 Rate Loop Roll Kd |
| `0x0A` | `RATE_PITCH_KP`| R/W | Signed Q8.8 Rate Loop Pitch Kp |
| `0x0B` | `RATE_PITCH_KI`| R/W | Signed Q8.8 Rate Loop Pitch Ki |
| `0x0C` | `RATE_PITCH_KD`| R/W | Signed Q8.8 Rate Loop Pitch Kd |
| `0x0D` | `RATE_YAW_KP` | R/W | Signed Q8.8 Rate Loop Yaw Kp |
| `0x0E` | `RATE_YAW_KI` | R/W | Signed Q8.8 Rate Loop Yaw Ki |
| `0x0F` | `RATE_YAW_KD` | R/W | Signed Q8.8 Rate Loop Yaw Kd |

---

## 8. Functional Verification & Simulation Results

Verification was performed using **ModelSim ASE 10.5b / QuestaSim** and **Cadence NC-Sim**. The test suite comprises 8 self-checking testbenches with dynamic reference calculation, cycle-accurate assertions, and automated error counters.

### 8.1 Verification Regression Summary

| Testbench | Module Under Test | Checks Verified | Result | Simulation Log Evidence |
|:---|:---|:---|:---:|:---|
| `tb_ps02_top.v` | `ps02_top` | Reset, STABILIZE cascaded loop, ACRO rate-only mode, ALT_HOLD vertical channel, bumpless transfer continuity, gain bank access, anti-windup clamping, sensor dropout, watchdog timeout, recovery handshake, deterministic latency. | **PASS** (13/13 Checks) | [`sim/logs/tb_ps02_top.log`](sim/logs/tb_ps02_top.log) |
| `tb_pid_controller.v` | `pid_controller` | Step response, P/I/D contribution verification, saturation limits, reset clearing. | **PASS** (0 Errors) | [`sim/logs/tb_pid_controller.log`](sim/logs/tb_pid_controller.log) |
| `tb_mode_controller.v`| `mode_controller` | State transitions across all 4 modes, invalid mode rejection, failsafe override. | **PASS** (0 Errors) | [`sim/logs/tb_mode_controller.log`](sim/logs/tb_mode_controller.log) |
| `tb_bumpless_transfer.v`| `bumpless_transfer` | Actuator jump suppression across abrupt mode changes. | **PASS** (0 Errors) | [`sim/logs/tb_bumpless_transfer.log`](sim/logs/tb_bumpless_transfer.log) |
| `tb_anti_windup.v` | `anti_windup` | Integrator halt during actuator saturation; back-calculation recovery. | **PASS** (0 Errors) | [`sim/logs/tb_anti_windup.log`](sim/logs/tb_anti_windup.log) |
| `tb_derivative_filter.v`| `derivative_filter` | High-frequency noise attenuation; $\alpha$-coefficient step response. | **PASS** (0 Errors) | [`sim/logs/tb_derivative_filter.log`](sim/logs/tb_derivative_filter.log) |
| `tb_saturation.v` | `fixed_point_saturation` | Full dynamic range test (`0x7FFF`, `0x8000`); verified zero wraparound. | **PASS** (0 Errors) | [`sim/logs/tb_saturation.log`](sim/logs/tb_saturation.log) |
| `tb_watchdog_failsafe.v`| `watchdog_timer` | Cycle-accurate timeout trigger at 500k cycles; clean recovery handshake. | **PASS** (0 Errors) | [`sim/logs/tb_watchdog_failsafe.log`](sim/logs/tb_watchdog_failsafe.log) |

### 8.2 Top-Level Regression Output Evidence
```text
================================================================================
   PS-02 TOP-LEVEL FULL-CHIP INTEGRATION REGRESSION (VLSI HACKATHON 2026)      
================================================================================
 [CHECK 1] Verifying Synchronous Reset and Default State...
  -> PASS: Mode=STABILIZE (01), Failsafe=0, Watchdog=0
 [CHECK 2] Verifying STABILIZE Cascaded Attitude/Rate Control...
  -> Motor commands: M0=0x1130, M1=0x52d0, M2=0x52d0, M3=0x1130
  -> PASS: Cascaded control driving differential roll correction correctly.
 [CHECK 3] Verifying ACRO Mode (Rate-Only, Outer Loop Bypassed)...
  -> PASS: Mode transitioned cleanly to ACRO (00).
  -> PASS: ACRO direct rate command tracked: M1=0x5104.
 [CHECK 5] Verifying ALTITUDE_HOLD Vertical Channel...
  -> ALT_HOLD active: M0=0x3ebc, M1=0x4344, M2=0x4344, M3=0x3ebc
  -> PASS: Altitude hold channel applied throttle trim symmetrically across motors.
 [CHECK 6] Verifying Bumpless Mode Transfer Continuity...
  -> PASS: Mode switched to STABILIZE with bumpless transfer engaged.
    Before: M0=0x3ebc M1=0x4344 | After: M0=0x2fbc M1=0x3444
 [CHECK 8] Verifying Per-Mode Gain Bank Register Access...
  -> PASS: Register write & readback verified: read 0x0250
 [CHECK 10] Verifying Anti-Windup Clamping under Sustained Error...
  -> PASS: Saturation detected and telemetry flag asserted: 0x81
  -> PASS: Anti-windup recovered rapidly from saturation: M0=0x5adc M1=0x0924
 [CHECK 14] Verifying Sensor Dropout, Watchdog Alarm, and FAILSAFE Policy...
  -> PASS: Stale sensor detected! Watchdog alarm=1, Failsafe active=1
  -> PASS: Safe fixed descent duty (25%) enforced across all motors.
 [CHECK 16] Verifying Sensor Recovery Handshake...
  -> PASS: Watchdog alarm cleared automatically on fresh sensor arrival.
 [CHECK 17] Verifying Deterministic Sample Latency...
  -> PASS: Sample-to-output latency and motor updates verified as deterministic (M0=0x1900, valid=1).
================================================================================
   FULL-CHIP REGRESSION SUMMARY: 13 PASSED, 0 FAILED
================================================================================
 *** TEST PASSED: ALL VERIFICATION CHECKLIST ITEMS VERIFIED! ***
```

| Simulation Waveform / Evidence | Technical Description |
|:---:|:---|
| ![Regression Pass](docs/figures/simulation/01_top_level_regression_pass.png) | ModelSim top-level integration regression summary (13 PASSED, 0 FAILED) |
| ![PID Waveform](docs/figures/simulation/04_pid_controller_waveform.png) | Simulation waveform showing P, I, D terms and actuator output response |
| ![Mode Switch Waveform](docs/figures/simulation/07_mode_switching_failsafe_waveform.png) | Glitch-free transition from ACRO to STABILIZE and watchdog failsafe trip |
| ![Tick Period Jitter](docs/figures/simulation/09_tick_rate_jitter_waveform.png) | Control-tick timing verification showing zero cycle-to-cycle jitter |

### 8.3 Verification Reconciliation (20-Item Plan vs. Top-Level Suite)

The engineering specification defined a 20-item verification plan (docs/verification_plan.md). In the physical verification suite, these are partitioned across the top-level integration regression (tb_ps02_top.v) and dedicated unit testbenches:

| Check # | Requirement Description | Verification Scope & Testbench | Assertions Checked | Status |
|:---:|:---|:---|:---:|:---:|
| **CHK-01** | Synchronous reset & state machine initialization | tb_ps02_top.v (Phase 1) | Mode=STABILIZE, Failsafe=0, Watchdog=0 | **PASS** |
| **CHK-02** | STABILIZE mode cascaded attitude/rate control | tb_ps02_top.v (Phase 2) | Motor differential roll command sanity | **PASS** |
| **CHK-03** | ACRO rate-only mode (attitude outer loop bypass) | tb_ps02_top.v (Phase 3) | Mode=ACRO, direct gyro tracking | **PASS** |
| **CHK-04** | Autonomous mode transitions & state sequencing | tb_mode_controller.v | All 4 legal states + invalid state trap | **PASS** |
| **CHK-05** | ALTITUDE_HOLD vertical channel & symmetric throttle trim | tb_ps02_top.v (Phase 4) | Symmetric motor thrust compensation | **PASS** |
| **CHK-06** | Bumpless mode transfer continuity & slew bounding | tb_ps02_top.v (Phase 5) | Actuator jump suppression across transition | **PASS** |
| **CHK-07** | Fixed-point saturating arithmetic boundary checks | tb_saturation.v | Maximum positive/negative clamping, zero wrap | **PASS** |
| **CHK-08** | Gain bank memory-mapped register access & readback | tb_ps02_top.v (Phase 6) | Read-after-write verification on register file | **PASS** |
| **CHK-09** | Bumpless transfer integrator re-initialization | tb_bumpless_transfer.v | Actuator delta bounded to 0 step | **PASS** |
| **CHK-10** | Anti-windup integration clamping & recovery | tb_ps02_top.v (Phase 7) | Telemetry flag assert (0x81) & rapid recovery | **PASS** |
| **CHK-11** | Derivative filter roll-off & high-frequency attenuation | tb_derivative_filter.v | Noise suppression & α step response | **PASS** |
| **CHK-12** | Integrator clamp under sustained extreme error | tb_anti_windup.v | Saturation flag + back-calculation halt | **PASS** |
| **CHK-13** | Sensor watchdog timeout threshold (500k cycles) | tb_watchdog_failsafe.v | Exact cycle boundary trigger verification | **PASS** |
| **CHK-14** | Sensor dropout trip & safe descent failsafe policy | tb_ps02_top.v (Phase 8) | Watchdog trip + 25% motor thrust descent | **PASS** |
| **CHK-15** | Emergency stop override & motor kill priority | tb_mode_controller.v / tb_watchdog_failsafe.v | Failsafe overrides manual pilot setpoints | **PASS** |
| **CHK-16** | Sensor recovery handshake & watchdog clear | tb_ps02_top.v (Phase 9) | Auto-alarm clear upon fresh valid sensor sample | **PASS** |
| **CHK-17** | Deterministic sample-to-actuator execution latency | tb_ps02_top.v (Phase 10) | Single-cycle pipelined motor command validity | **PASS** |
| **CHK-18** | Control tick rate generator timing & zero-jitter | control_tick_generator.v / sim | Waveform-verified exact cycle period | **PASS** |
| **CHK-19** | Multi-mode dynamic parameter reconfiguration | tb_ps02_top.v & tb_pid_controller.v | Symmetric negative error & gain bank scaling | **PASS** |
| **CHK-20** | Closed-loop roll-axis attitude stability step response | Closed-Loop Numerical Co-Sim | Overshoot 6.73%, Settling 4.57 s, e_ss=0.0865° | **PASS** |

---

## 9. Synthesis & Physical Design (RTL-to-GDS Flow)

The design was carried from synthesizable RTL through tapeout-ready layout using **Cadence Genus** and **Cadence Innovus**.

### 9.1 Cadence Genus Synthesis
- **Target Clock:** 50 MHz (`T = 20.0 ns`), SDC constraints applied.
- Technology mapping resolved all 17 hierarchical modules into standard cell logic.
- Gate-level schematics confirm pure synchronous logic with no combinational loops.

| Genus Gate-Level Mapped Schematic | Genus Hierarchical Logic View |
|:---:|:---:|
| ![Genus Mapped](docs/figures/synthesis/genus_mapped_schematic.png) | ![Genus Hierarchical](docs/figures/synthesis/genus_hierarchical_schematic.png) |

### 9.2 Cadence Innovus Place & Route (PnR)
Full physical design was carried out through standard ASIC implementation stages:
1. **Floorplan:** Core boundary set to `446.31 µm × 444.28 µm` (~0.198 mm²).
2. **Power Distribution:** Core power rings on `Metal4` / `Metal5` with continuous `Metal1` follow-pins.
3. **Placement:** Initial cell density reached ~69.9%.
4. **Clock Tree Synthesis (CTS):** Multi-level balanced clock tree built using CCOpt; cell density rose to 71.590% upon buffer insertion.
5. **NanoRoute:** Signal routing completed with 0.00% horizontal and vertical routing overflow across ~21,600 nets.
6. **Post-Route Timing Closure & ECO:** Setup met with **+0.016 ns slack** (0 violations across 1,118 paths). Hold closed marginally at **-0.000 ns** (1 marginal path).
7. **Physical Verification:** Verified DRC (0 violations across 16 sub-regions) and connectivity (0 violations, 0 warnings across all nets).

### 9.3 Physical Implementation Metrics Table

| Implementation Stage | Setup WNS (ns) | Setup Violations | Hold WNS (ns) | Hold Violations | Cell Density |
|:---|:---:|:---:|:---:|:---:|:---:|
| **Pre-CTS (Ideal Clock)** | -7.733 | 220 | -0.180 | 677 | 69.900% |
| **Pre-CTS (Post-Opt)** | +0.002 | 0 | -0.186 | 648 | 70.120% |
| **Post-CTS (Propagated)** | -0.146 | 56 | -0.187 | 668 | 71.590% |
| **Post-CTS (Post-ECO)** | +0.031 | 0 | -0.010 | 5 | 72.840% |
| **Post-Route (Post-ECO)** | **+0.016** | **0** | **-0.000** | **1** | **74.835%** |

| Layout Screenshot | Physical Verification Screenshot |
|:---:|:---:|
| ![Final Layout](docs/figures/physical_design/13_final_layout_with_fillers.png) | ![Verify DRC](docs/figures/physical_design/14_verify_drc_zero_violations.png) |
| *Final Routed Layout with Core Fillers (446 × 444 µm)* | *Cadence Innovus Verify DRC: 0 Violations* |

---

## 10. Closed-Loop Flight Performance Analysis

To evaluate control stability, the hardware controller was co-simulated against a numerical quadcopter plant model subjected to a 10.0° roll angle step input:

| Closed-Loop Metric | Hardware Measured Value | Analysis & Performance |
|:---|:---:|:---|
| **Target Roll Angle** | `10.000°` | Commanded pilot step input |
| **Peak Roll Angle** | `10.673°` | Measured peak airframe attitude at $t = 2.880\text{ s}$ |
| **Peak Overshoot ($M_p$)** | **6.73%** | Excellent damping preventing airframe oscillation |
| **Settling Time ($t_s$, $\pm 0.2°$)** | **4.57 s** | Clean asymptotic convergence to steady state |
| **Final Settled Angle** | `9.913°` | High-precision tracking |
| **Steady-State Error ($e_{ss}$)** | **0.0865°** | Integrator action successfully eliminates static droop |

| Closed-Loop Roll Step Response | Tracking Error & Plant Convergence |
|:---:|:---:|
| ![Closed Loop Roll](docs/figures/simulation/26_roll_angle_step_response.png) | ![Tracking Error](docs/figures/simulation/28_closed_loop_step_response_metrics.png) |

---

## 11. Repository Organization

```text
.
├── LICENSE                                # MIT Open-Source License
├── README.md                              # Comprehensive technical documentation
├── .gitignore                             # Git ignore rules for simulation/synthesis artifacts
├── constraints/
│   └── ps02_top.sdc                       # 50 MHz SDC timing constraints
├── docs/
│   ├── architecture.md                    # Detailed microarchitectural specification
│   ├── fixed_point_design.md              # Signed Q8.8 fixed-point arithmetic rules
│   ├── interface_specification.md         # Memory map and port descriptions
│   ├── reference_repository_analysis.md   # Architectural diff vs reference baseline
│   ├── requirements_traceability.md       # Traceability matrix against PS-02 requirements
│   ├── verification_plan.md               # Functional verification methodology
│   ├── physical_design.md                 # ASIC PnR, timing closure & physical verification report
│   ├── publication_manifest.md            # File audit, manifest, and classification log
│   └── figures/                           # Verified architectural, simulation, and PnR screenshots
│       ├── architecture/
│       ├── simulation/
│       ├── synthesis/
│       └── physical_design/
├── rtl/                                   # Synthesizable Verilog-2001 RTL Source Files
│   ├── actuator_interface.v               # Quadcopter X-configuration motor mixer
│   ├── anti_windup.v                      # Conditional integration clamping
│   ├── attitude_controller.v              # Outer attitude loop (Roll/Pitch P-terms)
│   ├── bumpless_transfer.v                # Smooth integrator recalibration engine
│   ├── config_registers.v                 # Synchronous memory-mapped register file
│   ├── control_tick_generator.v           # 1/2/4/8 kHz control tick strobe generator
│   ├── derivative_filter.v                # Low-pass IIR derivative filter
│   ├── failsafe_controller.v              # Emergency descent policy engine
│   ├── fixed_point_saturation.v           # Saturating Q8.8 arithmetic units
│   ├── gain_bank.v                        # Per-mode atomic PID gain storage
│   ├── mode_controller.v                  # FSM mode switching manager
│   ├── pid_controller.v                   # Core PID computation engine
│   ├── ps02_top.v                         # Top-level ASIC integration module
│   ├── rate_controller.v                  # Inner 3-axis angular rate controller
│   └── watchdog_timer.v                   # 500,000-cycle sensor watchdog timer
├── scripts/
│   ├── run_modelsim.bat                   # Windows batch script for one-click ModelSim regression
│   ├── run_regression.py                  # Portable Python automated test runner & log analyzer
│   ├── synthesis/
│   │   ├── genus_synth.tcl                # Cadence Genus synthesis script
│   │   └── genus_config.tcl.template      # Synthesis library & path configuration template
│   └── physical_design/
│       └── innovus_pnr.tcl                # Cadence Innovus place-and-route script
│       └── innovus_config.tcl.template    # PnR LEF, tech, & rule configuration template
├── sim/                                   # Verification Environment & Proof
│   ├── nc_files.f                         # Cadence Incisive file list
│   ├── README_NCLAUNCH.md                 # Setup guide for Cadence NCLaunch / NC-Sim
│   ├── run_all.do                         # ModelSim compilation and batch execution macro
│   ├── run_nclaunch.sh                    # NCLaunch batch execution wrapper
│   └── logs/                              # Verified simulation execution logs (0 Errors)
│       ├── tb_anti_windup.log
│       ├── tb_bumpless_transfer.log
│       ├── tb_derivative_filter.log
│       ├── tb_mode_controller.log
│       ├── tb_pid_controller.log
│       ├── tb_ps02_top.log                # Full-chip integration regression log (13/13 PASS)
│       ├── tb_saturation.log
│       └── tb_watchdog_failsafe.log
└── tb/                                    # Self-Checking Verilog-2001 Testbenches
    ├── tb_anti_windup.v
    ├── tb_bumpless_transfer.v
    ├── tb_derivative_filter.v
    ├── tb_mode_controller.v
    ├── tb_pid_controller.v
    ├── tb_ps02_top.v                      # Full-chip integration testbench
    ├── tb_saturation.v
    └── tb_watchdog_failsafe.v
```

---

## 12. Tool Dependencies & Reproduction Steps

### Required Tools
- **Simulation:** ModelSim ASE 10.5b+ / QuestaSim or Cadence Incisive (NC-Verilog)
- **Synthesis:** Cadence Genus Synthesis Solution
- **Physical Design:** Cadence Innovus Implementation System
- **Scripting:** Python 3.8+ (for automated regression runner)

### Running Simulation Locally

#### Option A: ModelSim Batch GUI / CLI
```powershell
cd sim
vsim -c -do run_all.do
```

#### Option B: Automated Python Regression Runner
```bash
# Run full regression suite (autodetects Questa/ModelSim or system PATH):
python scripts/run_regression.py

# Run a single target testbench:
python scripts/run_regression.py tb_pid_controller
```

#### Option C: Cadence NCLaunch / Incisive
```bash
cd sim
chmod +x run_nclaunch.sh
./run_nclaunch.sh
```

### Running Synthesis (Cadence Genus)
1. Copy the configuration template:
   ```bash
   cp scripts/synthesis/genus_config.tcl.template scripts/synthesis/genus_config.tcl
   ```
2. Set your target PDK `.lib` paths in `genus_config.tcl`. (If unconfigured, the script performs elaboration and architectural validation).
3. Run synthesis:
   ```bash
   genus -f scripts/synthesis/genus_synth.tcl
   ```

### Running Place & Route (Cadence Innovus)
1. Copy the configuration template:
   ```bash
   cp scripts/physical_design/innovus_config.tcl.template scripts/physical_design/innovus_config.tcl
   ```
2. Set your technology LEF and standard cell LEF paths in `innovus_config.tcl`.
3. Run place & route:
   ```bash
   innovus -files scripts/physical_design/innovus_pnr.tcl
   ```

---

## 13. Limitations & Gaps

To maintain strict engineering integrity, the following limitations are documented:
1. **Target PDK Identity:** The physical design run was conducted under a generic 90nm digital flow library inside Innovus; standard cell characterization files (`.lib`, `.lef`) cannot be redistributed due to foundry licensing.
2. **Physical LVS Status:** Internal Innovus connectivity verification was executed and passed cleanly (0 errors, 0 warnings across ~20,000 nets), but full external signoff LVS (e.g., Cadence Pegasus / Mentor Calibre) was not performed.
3. **Post-Route Hold Timing:** While setup timing met closure with positive slack (+0.016 ns), hold timing displays 1 marginal path with WNS of `-0.000 ns`, requiring minor buffer tuning during final signoff.
4. **Hardware-in-the-Loop Validation:** This repository provides an ASIC-grade RTL and physical layout design. It has not been flight-tested on a physical drone airframe in real flight conditions.

---

## 14. Future Engineering Improvements

- **SPI/I2C Hardware Master:** Integrate a direct SPI/I2C hardware engine on-chip to poll IMU registers directly, eliminating external microcontroller bridge requirements.
- **Time-Multiplexed Datapath:** Explore a time-multiplexed single-PID arithmetic core to evaluate area-versus-throughput trade-offs against the current 3-parallel-PID baseline.
- **Signoff LVS & DRC:** Complete full Calibre/Pegasus physical signoff verification with GDSII export on open-source SkyWater 130nm or modern FinFET nodes.
- **D-Shot Actuator Protocol:** Extend the motor mixer interface with hardware D-Shot digital telemetry encoders for modern electronic speed controllers (ESCs).

---

## 15. Authors & Academic Attribution

Developed as part of the **Silicon Dawn VLSI Hackathon 2026** at **Vellore Institute of Technology (VIT), Chennai**, addressing Problem Statement **PS-02 (Sudarshana Semiconductors)**.

- **Allen Joe A** — Registration No: `24BVD1100` — [GitHub: @ALLENJOE-A](https://github.com/ALLENJOE-A)
- **T Venkata Sai Neeraj** — Registration No: `24BVD1124`

---

## 16. License

This project is released under the **MIT License**. See the [`LICENSE`](LICENSE) file for complete terms.
