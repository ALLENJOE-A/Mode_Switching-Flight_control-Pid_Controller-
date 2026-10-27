# Interface Specification & Memory Map
**Project:** PS-02 Mode-Switching Hardware PID Controller  
**Document ID:** DOC-PS02-IF-001  
**Author:** Senior ASIC RTL Architect  

---

## 1. Top-Level Port List (`ps02_top.v`)

```verilog
module ps02_top #(
    parameter CLK_FREQ_HZ       = 50_000_000,
    parameter DEFAULT_TICK_RATE = 3'b010,       // 000: 1kHz, 001: 2kHz, 010: 4kHz, 011: 8kHz, 1xx: Direct test tick
    parameter DEFAULT_WATCHDOG  = 500_000       // Watchdog timeout in clock cycles
)(
    // Clock and Reset
    input  wire        clk,
    input  wire        rst,                     // Synchronous active-high reset

    // IMU Sensor Sample Inputs (Signed Q8.8)
    input  wire        sensor_valid,            // Strobe indicating fresh IMU sample
    input  wire [15:0] roll_angle_meas,         // Attitude Euler roll angle
    input  wire [15:0] pitch_angle_meas,        // Attitude Euler pitch angle
    input  wire [15:0] yaw_angle_meas,          // Attitude Euler yaw angle
    input  wire [15:0] roll_rate_meas,          // Gyroscope body roll rate (p)
    input  wire [15:0] pitch_rate_meas,         // Gyroscope body pitch rate (q)
    input  wire [15:0] yaw_rate_meas,           // Gyroscope body yaw rate (r)
    input  wire [15:0] altitude_meas,           // Barometer / ToF altitude
    input  wire [15:0] vert_vel_meas,           // Vertical velocity (climb rate)

    // Setpoint Inputs (Signed Q8.8)
    input  wire [15:0] roll_sp,                 // Desired roll angle (or rate in ACRO)
    input  wire [15:0] pitch_sp,                // Desired pitch angle (or rate in ACRO)
    input  wire [15:0] yaw_sp,                  // Desired yaw rate
    input  wire [15:0] altitude_sp,             // Desired altitude (ALT_HOLD)
    input  wire [15:0] throttle_base,           // Base pilot throttle (0.0 to +100.0%)

    // External Flight Mode Request
    input  wire [1:0]  mode_cmd,                // 00: ACRO, 01: STABILIZE, 10: ALT_HOLD, 11: FAILSAFE
    input  wire        mode_cmd_valid,          // Mode change request strobe

    // Synchronous Configuration Register Bus
    input  wire        reg_wr_en,               // Register write strobe
    input  wire        reg_rd_en,               // Register read strobe
    input  wire [7:0]  reg_addr,                // 8-bit register address
    input  wire [15:0] reg_wr_data,             // 16-bit write data (Q8.8 or config)
    output wire [15:0] reg_rd_data,             // 16-bit read data
    output wire        reg_ack,                 // Read/write acknowledge strobe

    // Actuator Command Outputs (Q8.8, 0 to +100.0%)
    output wire [15:0] motor0_cmd,              // Front-Right motor
    output wire [15:0] motor1_cmd,              // Back-Left motor
    output wire [15:0] motor2_cmd,              // Front-Left motor
    output wire [15:0] motor3_cmd,              // Back-Right motor
    output wire        motor_valid,             // New motor command valid strobe

    // Status and Telemetry Flags
    output wire [1:0]  current_mode,            // Active flight mode
    output wire        failsafe_active,         // Asserted when system is in FAILSAFE
    output wire        watchdog_alarm,          // Asserted on sensor timeout
    output wire [7:0]  saturation_flags,        // Axis/motor saturation telemetry
    output wire        bumpless_active          // Strobe when bumpless transfer is active
);
```

---

## 2. Register Memory Map

The register file contains 64 addressable 16-bit registers organized into 4 per-mode banks and system configuration registers.

### 2.1 Mode Encodings
- `2'b00`: **ACRO** (Rate-only control, attitude loop bypassed)
- `2'b01`: **STABILIZE** (Attitude outer loop + rate inner loop)
- `2'b10`: **ALTITUDE_HOLD** (Attitude/rate control + vertical channel)
- `2'b11`: **FAILSAFE** (Safe fixed output or hold-last-value)

### 2.2 Register Table

| Address (Hex) | Register Name | Mode / Domain | Access | Default (Hex) | Physical Equivalent |
|:---|:---|:---|:---:|:---:|:---|
| **0x00** | `SYS_CONTROL` | System | R/W | `0x0001` | Bit 0: Module Enable, Bit 1: Soft Reset, Bits [3:2]: Mode Override |
| **0x01** | `SYS_STATUS` | System | R | `0x0000` | Telemetry: [1:0] Current Mode, [2] Watchdog Alarm, [3] Failsafe, [11:4] Sat Flags |
| **0x02** | `SYS_TICK_DIV` | Timing | R/W | `0x0002` | Sample Rate Select: 0=1kHz, 1=2kHz, 2=4kHz, 3=8kHz, 4=Direct |
| **0x03** | `WATCHDOG_LIMIT_HI` | Watchdog | R/W | `0x0007` | Upper 16 bits of timeout cycle count |
| **0x04** | `WATCHDOG_LIMIT_LO` | Watchdog | R/W | `0xA120` | Lower 16 bits of timeout cycle count (500,000 cycles = 10 ms @ 50MHz) |
| **0x05** | `FAILSAFE_CONFIG` | Safety | R/W | `0x0000` | Bit 0: Policy (0=Safe Fixed, 1=Hold Last), Bits [15:8]: Safe Throttle |
| **0x06–0x0F** | *Reserved* | - | - | `0x0000` | Reserved |
| **0x10** | `ACRO_ROLL_KP` | Bank 0 (ACRO) | R/W | `0x0180` | Roll Rate Kp = 1.5 |
| **0x11** | `ACRO_ROLL_KI` | Bank 0 (ACRO) | R/W | `0x0008` | Roll Rate Ki = 0.03125 |
| **0x12** | `ACRO_ROLL_KD` | Bank 0 (ACRO) | R/W | `0x0080` | Roll Rate Kd = 0.5 |
| **0x13** | `ACRO_PITCH_KP` | Bank 0 (ACRO) | R/W | `0x0180` | Pitch Rate Kp = 1.5 |
| **0x14** | `ACRO_PITCH_KI` | Bank 0 (ACRO) | R/W | `0x0008` | Pitch Rate Ki = 0.03125 |
| **0x15** | `ACRO_PITCH_KD` | Bank 0 (ACRO) | R/W | `0x0080` | Pitch Rate Kd = 0.5 |
| **0x16** | `ACRO_YAW_KP` | Bank 0 (ACRO) | R/W | `0x0100` | Yaw Rate Kp = 1.0 |
| **0x17** | `ACRO_YAW_KI` | Bank 0 (ACRO) | R/W | `0x0008` | Yaw Rate Ki = 0.03125 |
| **0x18** | `ACRO_YAW_KD` | Bank 0 (ACRO) | R/W | `0x0040` | Yaw Rate Kd = 0.25 |
| **0x19** | `ACRO_D_ALPHA` | Bank 0 (ACRO) | R/W | `0x0080` | Derivative filter alpha = 0.5 |
| **0x1A** | `ACRO_MAX_OUT` | Bank 0 (ACRO) | R/W | `0x5000` | Output limit = +80.0 |
| **0x1B** | `ACRO_MIN_OUT` | Bank 0 (ACRO) | R/W | `0xB000` | Output limit = -80.0 |
| **0x1C** | `ACRO_MAX_I` | Bank 0 (ACRO) | R/W | `0x2000` | Integrator clamp = +32.0 |
| **0x1D** | `ACRO_MIN_I` | Bank 0 (ACRO) | R/W | `0xE000` | Integrator clamp = -32.0 |
| **0x20** | `STAB_ATT_KP` | Bank 1 (STABILIZE) | R/W | `0x0280` | Attitude Loop Angle Kp = 2.5 |
| **0x21** | `STAB_RATE_KP` | Bank 1 (STABILIZE) | R/W | `0x0100` | Rate Loop Kp = 1.0 |
| **0x22** | `STAB_RATE_KI` | Bank 1 (STABILIZE) | R/W | `0x000A` | Rate Loop Ki = 0.039 |
| **0x23** | `STAB_RATE_KD` | Bank 1 (STABILIZE) | R/W | `0x0060` | Rate Loop Kd = 0.375 |
| **0x24** | `STAB_YAW_KP` | Bank 1 (STABILIZE) | R/W | `0x0100` | Yaw Kp = 1.0 |
| **0x25** | `STAB_YAW_KI` | Bank 1 (STABILIZE) | R/W | `0x0008` | Yaw Ki = 0.031 |
| **0x26** | `STAB_YAW_KD` | Bank 1 (STABILIZE) | R/W | `0x0040` | Yaw Kd = 0.25 |
| **0x27** | `STAB_D_ALPHA` | Bank 1 (STABILIZE) | R/W | `0x0080` | Derivative filter alpha = 0.5 |
| **0x28** | `STAB_MAX_OUT` | Bank 1 (STABILIZE) | R/W | `0x4000` | Output limit = +64.0 |
| **0x29** | `STAB_MIN_OUT` | Bank 1 (STABILIZE) | R/W | `0xC000` | Output limit = -64.0 |
| **0x2A** | `STAB_MAX_I` | Bank 1 (STABILIZE) | R/W | `0x1800` | Integrator clamp = +24.0 |
| **0x2B** | `STAB_MIN_I` | Bank 1 (STABILIZE) | R/W | `0xE800` | Integrator clamp = -24.0 |
| **0x30** | `ALT_KP` | Bank 2 (ALT_HOLD) | R/W | `0x0180` | Altitude outer Kp = 1.5 |
| **0x31** | `ALT_VEL_KP` | Bank 2 (ALT_HOLD) | R/W | `0x0100` | Vertical velocity Kp = 1.0 |
| **0x32** | `ALT_VEL_KI` | Bank 2 (ALT_HOLD) | R/W | `0x0006` | Vertical velocity Ki = 0.023 |
| **0x33** | `ALT_VEL_KD` | Bank 2 (ALT_HOLD) | R/W | `0x0040` | Vertical velocity Kd = 0.25 |
| **0x34** | `ALT_THROTTLE_CLAMP`| Bank 2 (ALT_HOLD) | R/W | `0x1E00` | Max vertical throttle adjust = $\pm 30.0\%$ |
| **0x38** | `SAFE_MOTOR_DUTY` | Bank 3 (FAILSAFE) | R/W | `0x1900` | Safe descent motor duty = $25.0\%$ |
| **0x39** | `FAILSAFE_RAMP_RATE`| Bank 3 (FAILSAFE) | R/W | `0x0020` | Ramp down step size per tick |

---

## 3. Handshake & Timing Specification

1. **Sensor Ingestion:** When `sensor_valid` pulses high for 1 clock cycle, fresh IMU telemetry is latched into the input pipeline.
2. **Watchdog Activity:** Every assertion of `sensor_valid` resets the watchdog counter to 0. If the counter reaches `WATCHDOG_LIMIT`, `watchdog_alarm` and `failsafe_active` are asserted.
3. **Control Update Cadence:** The internal control tick generator divides `clk` to produce the selected sample rate (1–8 kHz). When enabled by `inner_tick`, the cascaded arithmetic pipeline executes deterministically.
4. **Latency:** Input sample to motor output update occurs in exactly **3 clock cycles** (Deterministic Latency = 60 ns at 50 MHz).
5. **Mode Switching:** If `mode_cmd_valid` pulses, the mode manager latches `mode_cmd` and coordinates with `bumpless_transfer.v` to perform an atomic coefficient and state handover at the next tick boundary.
