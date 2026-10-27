# Requirements Traceability Matrix
**Project:** PS-02 Mode-Switching Hardware PID Controller (VLSI Hackathon 2026)  
**Standard:** Synthesizable Verilog-2001 ASIC Flow (GPDK45 / GPDK90 / GPDK180 Target)  

| Req ID | Requirement Description | Target RTL Module | Target Testbench | Implementation Status | Verification Evidence / Log |
|:---|:---|:---|:---|:---|:---|
| **FR-01** | Three control axes (Roll, Pitch, Yaw) with parallel baseline modular for time-shared trade-off. | `rtl/ps02_top.v`<br>`rtl/rate_controller.v`<br>`rtl/attitude_controller.v` | `tb/tb_ps02_top.v` | PARTIALLY VERIFIED | `sim/logs/tb_ps02_top.log` (Pitch/Yaw missing explicit assertions) |
| **FR-02** | Cascaded attitude (outer) and rate (inner) loops; configurable 1–8 kHz inner loop rate; deterministic cycle latency. | `rtl/attitude_controller.v`<br>`rtl/rate_controller.v`<br>`rtl/control_tick_generator.v` | `tb/tb_ps02_top.v`<br>`tb/tb_pid_controller.v` | PARTIALLY VERIFIED | `sim/logs/tb_ps02_top.log` (1-8kHz range & cycle-exact latency missing explicit fail checks) |
| **FR-03** | Four selectable flight modes (ACRO, STABILIZE, ALTITUDE_HOLD, FAILSAFE) with registered glitch-free mode manager. | `rtl/mode_controller.v` | `tb/tb_mode_controller.v`<br>`tb/tb_ps02_top.v` | PARTIALLY VERIFIED | `sim/logs/tb_mode_controller.log` (Not all transition pairs tested) |
| **FR-04** | Per-mode gain banks with atomic loading at control cycle boundaries, readback, and reset defaults. | `rtl/gain_bank.v`<br>`rtl/config_registers.v` | `tb/tb_mode_controller.v`<br>`tb/tb_ps02_top.v` | PARTIALLY VERIFIED | `sim/logs/tb_mode_controller.log` (Atomic boundaries lacking explicit checks) |
| **FR-05** | Bumpless mode transfer via integrator back-calculation ($I_{new} = u_{prev} - P_{new} - D_{new}$) across mode switches. | `rtl/bumpless_transfer.v`<br>`rtl/pid_controller.v` | `tb/tb_bumpless_transfer.v`<br>`tb/tb_ps02_top.v` | PARTIALLY VERIFIED | `sim/logs/tb_bumpless_transfer.log` (Not checked for all mode boundaries) |
| **FR-06** | Integral anti-windup (conditional integration / clamping / accumulator limit) preventing wraparound. | `rtl/anti_windup.v`<br>`rtl/pid_controller.v` | `tb/tb_anti_windup.v`<br>`tb/tb_pid_controller.v` | VERIFIED | `sim/logs/tb_anti_windup.log` (100% PASS) |
| **FR-07** | Low-pass-filtered derivative computed on measurement feedback to eliminate setpoint derivative kick. | `rtl/derivative_filter.v`<br>`rtl/pid_controller.v` | `tb/tb_derivative_filter.v` | VERIFIED | `sim/logs/tb_derivative_filter.log` (100% PASS) |
| **FR-08** | Saturating signed fixed-point arithmetic with explicit sign extension, overflow detection, and bounds checking. | `rtl/fixed_point_saturation.v` | `tb/tb_saturation.v` | VERIFIED | `sim/logs/tb_saturation.log` (100% PASS) |
| **FR-09** | Watchdog timer detecting stale sensor data with programmable timeout and configurable failsafe policy. | `rtl/watchdog_timer.v`<br>`rtl/failsafe_controller.v` | `tb/tb_watchdog_failsafe.v` | VERIFIED | `sim/logs/tb_watchdog_failsafe.log` (100% PASS) |
| **FR-10** | Documented synchronous register interface for IMU, setpoints, vertical channel, mode, telemetry, and status. | `rtl/config_registers.v`<br>`rtl/ps02_top.v` | `tb/tb_ps02_top.v` | VERIFIED | `sim/logs/tb_ps02_top.log` (100% PASS) |

---

## Traceability Update Policy
- **PLANNED:** Module designed and scheduled for implementation.
- **IMPLEMENTED:** RTL written and compiled cleanly under Verilog-2001 compiler without syntax errors.
- **VERIFIED:** Self-checking testbench executed with zero errors and logged evidence.
