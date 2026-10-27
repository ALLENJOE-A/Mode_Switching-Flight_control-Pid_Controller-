# Architecture Specification: PS-02 Mode-Switching Hardware PID Controller
**Project:** PS-02 Mode-Switching Hardware PID Controller (VLSI Hackathon 2026)  
**Document ID:** DOC-PS02-ARCH-001  
**Author:** Senior ASIC RTL Architect & Control Engineer  

---

## 1. Top-Level System Architecture

```
                                    +-------------------------------------------------------------+
                                    |                       ps02_top                              |
                                    |                                                             |
   reg_wr/rd/addr/data ------------>|---> [config_registers] <=======> [gain_bank]                |
                                    |              |                       |                      |
   mode_cmd ----------------------->|---> [mode_controller] <-------------+ (Active Gains)       |
                                    |              |                                              |
   sensor_valid ------------------->|---> [watchdog_timer] ---> [failsafe_controller]             |
                                    |              |                                              |
                                    |     +--------+--------+                                     |
                                    |     |                 |                                     |
   Angle Meas & Setpoints --------->|---> [attitude_ctrl]   | (Bypassed in ACRO)                  |
                                    |           |           |                                     |
                                    |           v           |                                     |
   Rate Meas & Setpoints ---------->|---> [rate_ctrl] <-----+                                     |
                                    |      (3x PID)                                               |
                                    |           |                                                 |
   Alt Meas & Setpoint ------------>|---> [alt_ctrl]                                              |
                                    |           |                                                 |
                                    |           v                                                 |
                                    |     [bumpless_transfer] <--- u_prev, P_new, D_new           |
                                    |           |                                                 |
                                    |           v                                                 |
                                    |     [actuator_interface / mixer]                            |
                                    |           |                                                 |
   motor0..3_cmd <------------------+-----------+                                                 |
                                    +-------------------------------------------------------------+
```

---

## 2. Module Responsibilities and Boundaries

| Module | Description & Responsibility | Input Dependencies | Key Outputs |
|:---|:---|:---|:---|
| `ps02_top.v` | Top-level ASIC module integrating control pipelines, registers, safety watchdogs, and actuator interfaces. | System clk, rst, IMU sensors, setpoints, register bus. | Motor duty commands, telemetry status, fault flags. |
| `mode_controller.v` | FSM managing transitions among ACRO, STABILIZE, ALT_HOLD, and FAILSAFE. Coordinates synchronous bank switching. | Mode request, watchdog fault, tick enable. | Active mode ID, mode transition pulse, bank select. |
| `attitude_controller.v` | Outer control loop. Evaluates Euler angle errors ($\Delta \phi, \Delta \theta, \Delta \psi$) to generate target body rates. Active in STABILIZE and ALT_HOLD. | Angle setpoints, angle measurements, outer loop gains. | Desired angular rates ($p_{sp}, q_{sp}, r_{sp}$). |
| `rate_controller.v` | Inner fast control loop (1–8 kHz). Executes parallel PID control on roll ($p$), pitch ($q$), and yaw ($r$) angular rate errors. | Rate setpoints, gyro measurements, rate loop gains. | Unmixed torque corrections ($u_{roll}, u_{pitch}, u_{yaw}$). |
| `pid_controller.v` | Parameterized core PID engine with saturating math, derivative on measurement, and integrator reload port for bumpless transfer. | Error, measurement feedback, gains, reload value, tick. | PID correction output, saturation indicators. |
| `gain_bank.v` | Multi-bank storage holding independent gain and limit configurations for all 4 flight modes. | Register configuration bus, active mode select. | Active $K_p, K_i, K_d, \alpha, MAX, MIN$ to datapaths. |
| `bumpless_transfer.v` | Calculates new integrator state upon mode switch: $I_{new} = \text{clamp}(u_{prev} - P_{new} - D_{new}, MIN_I, MAX_I)$. | Previous actuator outputs, new P and D contributions, limits. | Integrator reload value and strobe. |
| `anti_windup.v` | Conditional integration (clamping) engine. Inhibits integration when saturated in the direction that worsens saturation. | Integrator increment, output saturation flags, error sign. | Gated integrator increment, clamped output. |
| `derivative_filter.v` | First-order discrete low-pass filter on measurement feedback: $d_{filt}[n] = d_{filt}[n-1] + \alpha (-(y[n] - y[n-1]) - d_{filt}[n-1])$. | Sensor measurement, filter coefficient $\alpha$, tick. | Filtered derivative term without setpoint kick. |
| `fixed_point_saturation.v` | Reusable signed saturation functions and verification wrappers. | Wide signed products / sums. | Clamped Q8.8 results with overflow flags. |
| `control_tick_generator.v` | Clock divider generating deterministic inner loop (1–8 kHz) and outer loop enable pulses from system clock. | Master system clock (50 MHz), rate divider config. | `inner_tick`, `outer_tick`. |
| `watchdog_timer.v` | Monostable safety counter measuring inter-arrival time of fresh sensor samples. Flags stale data on timeout. | `sensor_valid`, timeout limit register. | `watchdog_timeout` pulse and alarm level. |
| `failsafe_controller.v` | Safety executive enforcing configured emergency policy (safe descent fixed duty vs. hold-last-value). | Watchdog alarm, manual failsafe command, failsafe config. | Failsafe override commands, motor disarm. |
| `actuator_interface.v` | Quadcopter X-frame mixer translating throttle and roll/pitch/yaw torques into 4 motor duty cycles with `[0, 100.0%]` clamps. | Throttle, roll/pitch/yaw corrections, failsafe override. | Four motor duty commands (`motor0..3_cmd`). |
| `config_registers.v` | Synchronous memory-mapped register file providing read/write access to gain banks and system settings. | Address, write data, write enable, read enable. | Read data, acknowledge, configuration busses. |

---

## 3. Flight Mode Data Flow

### 3.1 ACRO Mode (`2'b00`)
- **Outer Loop:** Completely bypassed.
- **Inner Loop:** Roll/pitch/yaw setpoint inputs are fed directly into the inner rate loop ($p_{sp} = roll\_sp, q_{sp} = pitch\_sp, r_{sp} = yaw\_sp$).
- **Altitude:** Inactive; base throttle feeds motor mixer directly.
- **Target Dynamics:** High-agility acrobatic flight; pilot controls rotational rates directly.

### 3.2 STABILIZE Mode (`2'b01`)
- **Outer Loop:** Active. Computes angle error $e_\theta = \theta_{sp} - \theta_{meas}$ and scales by attitude $K_{p,att}$ to produce rate setpoint $\omega_{sp}$.
- **Inner Loop:** Active. Fast rate loop tracks $\omega_{sp}$ against gyro measurements.
- **Altitude:** Inactive; base throttle feeds mixer directly.
- **Target Dynamics:** Self-leveling flight. Sticks command bank angles.

### 3.3 ALTITUDE_HOLD Mode (`2'b10`)
- **Outer Loop & Inner Loop:** Same as STABILIZE for roll and pitch; yaw rate controlled.
- **Altitude Channel:** Outer loop calculates climb rate setpoint $\dot{z}_{sp} = K_{p,alt} \cdot (z_{sp} - z_{meas})$. Inner loop calculates throttle trim $\Delta \text{throttle} = \text{PID}(\dot{z}_{sp} - \dot{z}_{meas})$.
- **Mixer:** Net throttle = `throttle_base` + $\Delta \text{throttle}$.
- **Target Dynamics:** Stable autonomous hover at desired barometric / sensor altitude.

### 3.4 FAILSAFE Mode (`2'b11`)
- **Trigger:** Watchdog timeout (stale sensor data) or register command.
- **Policy 0 (`SAFE_FIXED`):** Roll/Pitch/Yaw torques zeroed (level attitude). Throttle commands set to calibrated safe descent duty (`0x1900` = 25.0%).
- **Policy 1 (`HOLD_LAST`):** Freezes all actuator duty commands at their exact values prior to failure.

---

## 4. Bumpless Transfer Sequence and Mathematics

When switching from Mode $A$ to Mode $B$ at sample time $k$:
1. Mode controller asserts `bumpless_strobe` for 1 clock cycle.
2. The active gains are atomically swapped to Mode $B$'s bank: $K_{p,B}, K_{i,B}, K_{d,B}$.
3. The new proportional and derivative terms are calculated:
   $$P_B[k] = K_{p,B} \cdot e_B[k]$$
   $$D_B[k] = -K_{d,B} \cdot d_{filt}[k]$$
4. The previous total control command is $u_{prev} = u_A[k-1]$.
5. To enforce $u_B[k] \approx u_A[k-1]$, the required integral term is:
   $$I_{target} = u_{prev} - P_B[k] - D_B[k]$$
6. The target integral term is clamped to Bank $B$'s configured integral limits:
   $$I_{new} = \text{clamp}(I_{target}, MIN\_I_B, MAX\_I_B)$$
7. `bumpless_transfer.v` loads $I_{new}$ into the PID accumulator simultaneously with the gain swap.
8. The resulting output discontinuity $|u_B[k] - u_A[k-1]|$ is bounded strictly to 0 (unless limited by $MAX\_I_B$).

---

## 5. Architectural Trade-Off: Parallel Baseline vs. Time-Shared Datapath

| Metric / Consideration | Baseline: 3 Parallel Datapaths | Future: 1 Time-Shared Datapath |
|:---|:---|:---|
| **RTL Multipliers (DSP / Cell)** | 9 Multipliers (3 per axis: P, I, D) | 1 Multiplier (time-multiplexed) |
| **Silicon Area (NAND-2 Equivalent)** | High (~3.2× arithmetic area) | Minimal (~60% arithmetic area savings) |
| **Cycle Latency** | 3 Clock Cycles (Deterministic) | 12–16 Clock Cycles (Sequencer latency) |
| **Control Timing Simplicity** | Ultra-simple; all axes update simultaneously | Requires multi-state FSM and state RAM/registers |
| **Maximum Inner Loop Rate** | Exceeds 500 kHz at 50 MHz clock | Limited to ~3 MHz maximum |
| **Verification Complexity** | Clean, decoupled, concurrent | Complex scheduling and hazard checking |

*Conclusion:* The parallel baseline is implemented first for functional perfection and safety verification. The module interfaces are strictly isolated to allow drop-in substitution of a time-shared ALU in future synthesis benchmarks.
