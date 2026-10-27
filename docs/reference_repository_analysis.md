# Reference Repository Analysis & Architectural Audit
**Project:** PS-02 Mode-Switching Hardware PID Controller (VLSI Hackathon 2026)  
**Reference Examined:** `HiL-Flight-Controller-Core-Verilog`  
**Author:** Senior ASIC RTL Architect & Control Engineer  
**Date:** October 2026  
**Status:** Approved Reference Audit  

---

## 1. Executive Summary

A comprehensive architectural and RTL audit of `HiL-Flight-Controller-Core-Verilog` was conducted. The reference repository implements a basic, open-loop-in-time quadcopter flight controller core in Verilog using signed Q8.8 fixed-point arithmetic, simulated using Icarus Verilog and a cocotb Python testbench.

While the reference design provides useful foundational concepts (such as basic Q8.8 multiplication truncation and an X-frame motor mixer), it falls significantly short of the VLSI Hackathon 2026 **PS-02 Specification**. It cannot be used as-is for an ASIC tapeout targeting Cadence Genus/Innovus.

---

## 2. Reference Module-by-Module Audit

| Reference Module | Primary Responsibility | Architectural Observation | Reuse Assessment |
|:---|:---|:---|:---|
| `flight_controller_top.v` | Structural integration of gain registers, 3 PIDs, 3 saturation guards, mixer, watchdog, and 4 PWM generators. | Single-tier flat architecture. No cascaded loops (only single rate/error loop). No flight mode switching. No altitude channel. Direct wiring without registered handshakes. | **Do Not Reuse Directly.** Re-architect into `ps02_top.v` supporting cascaded attitude/rate loops, mode manager, and register interfaces. |
| `pid_controller.v` | Discrete PID with P, I, D terms. | **Critical Flaw 1:** Derivative is calculated on setpoint error (`error_diff = error_signed - prev_error`), which causes violent derivative kick on step setpoints. FR-7 mandates derivative on *measurement*.<br>**Critical Flaw 2:** Integrator accumulates every single raw clock cycle (`50 MHz`) without sample-valid timing control.<br>**Critical Flaw 3:** Integrator clamping function treats the 32-bit accumulator as a 16-bit register with upper bits discarded, failing to provide fractional precision.<br>**Critical Flaw 4:** No bumpless transfer reload port. | **Substantial Redesign Required.** Completely rewrite as modular, parameterized `pid_controller.v` with derivative on feedback measurement, sample-tick gating, and state reload ports for bumpless transfer. |
| `saturation_guard.v` | Output clamping and anti-windup `integrator_hold` generation. | **Critical Flaw:** Sits downstream of `pid_controller.v`. However, `pid_controller.v` already clamps its output (`pid_out <= pid_sum_clamped`). Thus `raw_in > MAX_OUT` is strictly never true, and `integrator_hold` is **never asserted** in closed-loop operation. | **Redesign & Integrate.** Replace with standalone `anti_windup.v` implementing true conditional integration (clamping) and output saturation. |
| `gain_regs.v` | 9-register file (Kp, Ki, Kd for roll, pitch, yaw) with write bus. | Only stores 1 set of gains. No per-mode banks. Write-only (no readback). Writes take effect immediately mid-calculation, risking catastrophic arithmetic glitches during an active control step. | **Replace with `gain_bank.v` and `config_registers.v`.** Support 4 per-mode banks, atomic bank switching at cycle boundaries, and readback. |
| `mixer.v` | X-frame quadcopter mixing matrix. | Standard X-frame logic with wide sum and output clamping `[0, 100.0]`. Combinational, clean. | **Reuse Conceptually.** Reusable with parameterization in `actuator_interface.v`. |
| `watchdog_timer.v` | Monitors `error_valid` and disables motors after timeout. | Counter width calculation uses `$clog2`. Disarms motors (`motor_enable = 0`) on timeout. Good baseline. | **Enhance into `watchdog_timer.v` & `failsafe_controller.v`.** Parameterized timeout, configurable failsafe policies (safe fixed vs. hold last), and formal fault status reporting. |
| `pwm_gen.v` | Clock-agnostic counter-based PWM generator (50 Hz). | Pure counter-comparator. Suitable for hobby ESCs, but for digital flight control ASIC, digital motor command words (or high-speed DShot/OneShot) are primary. | **Retain as optional peripheral.** Focus functional stage on digital motor commands and deterministic latency. |
| `sim/tb_pid.v` | Standalone Verilog testbench for PID and saturation guard. | Uses `$display` without comprehensive assertions. Does not test bumpless transfer, cascaded loops, or mode switching. | **Replace with thorough self-checking Verilog-2001 testbenches.** |

---

## 3. Requirement-by-Requirement Gap Analysis (PS-02 vs. Reference)

| Requirement ID | Problem Statement Requirement | Reference Implementation Status | Gap Identified |
|:---|:---|:---|:---|
| **FR-01** | Three control axes (Roll, Pitch, Yaw) with modular parallel baseline. | **Partially Present** (Roll, Pitch, Yaw exist as flat parallel blocks). | No architectural modularity for future time-shared comparison. Hardcoded connections. |
| **FR-02** | Cascaded attitude (outer) and rate (inner) loops with configurable 1–8 kHz update rate. | **ABSENT.** Only single error input per axis. Updates every raw clock cycle without timing scheduler. | No outer loop. No rate setpoint generation. No configurable 1–8 kHz clock-enable generator. Control cadence is tied to simulation clock. |
| **FR-03** | Four selectable flight modes (ACRO, STABILIZE, ALTITUDE_HOLD, FAILSAFE). | **ABSENT.** | No mode controller. Zero concept of flight modes. Cannot switch control laws or signal paths. |
| **FR-04** | Per-mode gain banks with atomic loading at cycle boundaries. | **ABSENT.** Only a single set of 9 registers. | No independent mode banks. No atomic loading mechanism. Coefficients update asynchronously. |
| **FR-05** | Bumpless mode transfer ($I_{new} = u_{prev} - P_{new} - D_{new}$). | **ABSENT.** | Integrator state is not recalculated or preserved across transitions. Mode switches will cause violent actuator steps. |
| **FR-06** | Anti-windup (conditional integration / back-calculation / clamping). | **BROKEN / INEFFECTIVE.** | `saturation_guard` receives already-clamped data, making `integrator_hold` inoperative. No conditional integration based on error sign. |
| **FR-07** | Filtered derivative on measurement (feedback signal, not error). | **INCORRECT.** Filtered derivative is implemented on *error*, causing derivative kick. | Must switch input of derivative filter from error $e[n]$ to feedback measurement $y_{meas}[n]$ with configurable filter coefficient $\alpha$. |
| **FR-08** | Saturating arithmetic (explicit signed fixed-point, no overflow wrap). | **Partially Present** (simple saturation function in `pid_controller`). | Lacks unified saturation library. Integrator accumulator logic has scaling ambiguities. Intermediate signs and widths need formal specification. |
| **FR-09** | Watchdog and stale sensor data handling with configurable failsafe. | **Minimal.** Basic watchdog shuts off motors to 0. | Missing configurable failsafe policy (safe fixed hover vs. hold-last-value), fault telemetry, and controlled recovery handshake. |
| **FR-10** | Documented register interface for IMU, setpoints, gains, watchdog, and telemetry. | **Minimal.** Only 4-bit write-only bus for 9 gain registers. | No readback, no IMU registers, no setpoint registers, no status/fault registers. |

---

## 4. Key Engineering Decisions & Next Steps

1. **Clean-Room Baseline:** All new RTL and verification collateral will be developed inside `PS02_Mode_Switching_PID_Controller/`, keeping the reference repository completely intact.
2. **Unified Saturating Arithmetic Library (`fixed_point_saturation.v`):** Synthesizable signed saturation functions and modules to guarantee zero wraparound on any multiplier or accumulator overflow.
3. **Dedicated Cascaded Controller Architecture:**
   - Outer loop (`attitude_controller.v`): Computes desired body rates from Euler angle errors ($\Delta \phi, \Delta \theta, \Delta \psi$).
   - Inner loop (`rate_controller.v`): Fast 1–8 kHz loop with rate error PIDs driving motor mixers.
   - Bypassed in `ACRO` mode; cascaded in `STABILIZE` and `ALTITUDE_HOLD`.
4. **Bumpless Transfer Engine (`bumpless_transfer.v`):** Deterministic integrator back-calculation executed in exactly 1 clock cycle upon mode transition strobe.
5. **Deterministic Timing Scheduler (`control_tick_generator.v`):** Parameterized timing engine providing synchronized sample-valid strobes for 1 kHz, 2 kHz, 4 kHz, and 8 kHz operation from a 50 MHz ASIC system clock.
