# Functional Verification Plan
**Project:** PS-02 Mode-Switching Hardware PID Controller (VLSI Hackathon 2026)  
**Document ID:** DOC-PS02-VERIF-001  
**Author:** Senior ASIC Verification Engineer  

---

## 1. Verification Strategy and Principles

In strict compliance with Hackathon rules and ASIC tapeout standards:
1. **100% Verilog-2001 Compliance:** All testbenches are authored in synthesizable/simulation-standard Verilog-2001. No SystemVerilog constructs (`logic`, `always_ff`, classes, covergroups) are permitted.
2. **Self-Checking Architecture:** Every testbench calculates reference expectations dynamically, checks actual DUT responses, counts errors, and prints an explicit `*** TEST PASSED ***` or `*** TEST FAILED ***` summary banner.
3. **Hard Watchdog Timeouts:** Every testbench contains an independent timeout thread using `$finish` to guarantee the simulator will never hang indefinitely.
4. **Zero Silent Overflow:** Arithmetic boundary testing verifies that every signed multiplication and accumulation handles extreme inputs (`16'h7FFF`, `16'h8000`) with zero wraparound.
5. **Exact Latency Measurement:** The sample-valid to motor-update latency is checked with cycle-accurate assertions.

---

## 2. Testbench Suite Specification

| Testbench File | Target Module | Scope & Key Verification Items |
|:---|:---|:---|
| `tb/tb_saturation.v` | `fixed_point_saturation.v` | Positive/negative overflow, boundary values `+127.99` and `-128.0`, signed range clamping. |
| `tb/tb_derivative_filter.v` | `derivative_filter.v` | Derivative calculated on measurement (not error); step setpoint verification showing **zero derivative kick**; filter response under noise. |
| `tb/tb_anti_windup.v` | `anti_windup.v` | Positive saturation clamping; negative saturation clamping; integration allowed when error acts to un-saturate; recovery time. |
| `tb/tb_pid_controller.v` | `pid_controller.v` | Isolated P, I, D step responses; integrator accumulator precision; state reload for bumpless transfer; deterministic 1-cycle latency. |
| `tb/tb_bumpless_transfer.v` | `bumpless_transfer.v` | Back-calculation $I_{new} = u_{prev} - P_{new} - D_{new}$; bounded output discontinuity across gain steps; limit clamping. |
| `tb/tb_mode_controller.v` | `mode_controller.v` | 4-mode FSM transitions; atomic gain bank switching at cycle boundaries; invalid mode encoding recovery. |
| `tb/tb_watchdog_failsafe.v` | `watchdog_timer.v`<br>`failsafe_controller.v` | Timeout at exact boundary cycle; safe fixed vs hold-last failsafe policies; recovery handshake when valid samples resume. |
| `tb/tb_ps02_top.v` | `ps02_top.v` | Full-chip end-to-end regression covering all 20 functional test requirements. |

---

## 3. Top-Level Regression Checklist (20 Mandatory Verification Items)

1. [ ] **Reset and Initial State:** DUT resets synchronously; all motor commands zero; mode initializes to configured default.
2. [ ] **Three-Axis Independence:** Roll, pitch, and yaw respond independently without cross-axis coupling.
3. [ ] **ACRO Mode Rate Control:** Outer loop bypassed; rate setpoints drive inner rate loop directly.
4. [ ] **STABILIZE Mode Cascaded Control:** Outer attitude loop computes target rates; inner rate loop tracks gyro.
5. [ ] **ALTITUDE_HOLD Mode:** Vertical channel calculates climb rate and adjusts throttle trim within configured clamp.
6. [ ] **Full Mode Transition Matrix:** All directed transitions (ACRO $\leftrightarrow$ STABILIZE $\leftrightarrow$ ALT_HOLD $\leftrightarrow$ FAILSAFE).
7. [ ] **Bumpless Output Continuity:** Discontinuity $|u_{new} - u_{prev}|$ strictly verified across mode switches.
8. [ ] **Per-Mode Gain Banks:** Gains and limits reload atomically from active bank on mode change.
9. [ ] **Bipolar Error Response:** Symmetric response to positive and negative errors on all axes.
10. [ ] **Anti-Windup Clamping:** Integrator freezes when saturated; recovers immediately upon error reversal.
11. [ ] **No Derivative Kick:** Setpoint step creates zero derivative pulse; measurement step creates low-pass filtered derivative.
12. [ ] **Output Saturation:** Actuator commands clamped strictly to configured $[MIN\_OUT, MAX\_OUT]$.
13. [ ] **Arithmetic Overflow Protection:** Extreme product/sum values saturate cleanly to $16'h7FFF$ / $16'h8000$.
14. [ ] **Watchdog Sensor Timeout:** Stale sensor data triggers watchdog alarm at exact timeout cycle.
15. [ ] **Failsafe Policy Enforcement:** Policy 0 enforces safe fixed descent duty; Policy 1 enforces hold-last-value.
16. [ ] **Sensor Recovery Handshake:** Resumption of valid samples clears watchdog and restores normal control safely.
17. [ ] **Deterministic Cycle Latency:** Input sample to motor update takes exactly 3 clock cycles.
18. [ ] **Configurable Tick Rates:** Inner loop timing verified at 1 kHz, 4 kHz, and 8 kHz.
19. [ ] **Back-to-Back Mode Switching:** Rapid mode changes handled without state corruption or arithmetic glitches.
20. [ ] **Fault and Recovery Reset:** Reset behaves predictably during normal flight, saturation, and failsafe.
