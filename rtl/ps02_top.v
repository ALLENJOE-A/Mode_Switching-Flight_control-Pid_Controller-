`timescale 1ns / 1ps
//============================================================================
// Module:      ps02_top
// Project:     PS-02 Mode-Switching Hardware PID Controller (VLSI Hackathon 2026)
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Top-level flight controller integrating:
//              1. Cascaded outer attitude loop and inner fast rate loop (1–8 kHz).
//              2. 4 Selectable flight modes (ACRO, STABILIZE, ALT_HOLD, FAILSAFE).
//              3. Multi-bank per-mode gain register storage with atomic loading.
//              4. Bumpless mode transfer engine via integrator back-calculation.
//              5. Conditional anti-windup clamping with zero integral drift.
//              6. Measurement-feedback filtered derivative (zero setpoint kick).
//              7. Hardware safety watchdog detecting stale sensor telemetry.
//              8. Configurable failsafe policy executive (Safe Fixed vs Hold Last).
//              9. Quadcopter X-frame actuator mixing with 0–100% duty clamps.
//             10. Synchronous memory-mapped register configuration interface.
//============================================================================

module ps02_top #(
    parameter CLK_FREQ_HZ       = 50_000_000,
    parameter DEFAULT_TICK_RATE = 3'b010,       // 000: 1kHz, 001: 2kHz, 010: 4kHz, 011: 8kHz, 1xx: Fast test tick
    parameter DEFAULT_WATCHDOG  = 500_000       // Watchdog timeout in clock cycles
)(
    // Clock and Reset
    input  wire        clk,
    input  wire        rst,                     // Synchronous active-high reset

    // IMU Sensor Telemetry Inputs (Signed Q8.8)
    input  wire        sensor_valid,            // Strobe on fresh IMU sample
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
    output reg         motor_valid,             // New motor command valid strobe

    // Status and Telemetry Flags
    output wire [1:0]  current_mode,            // Active flight mode
    output wire        failsafe_active,         // Asserted when system is in FAILSAFE
    output wire        watchdog_alarm,          // Asserted on sensor timeout
    output wire [7:0]  saturation_flags,        // Axis/motor saturation telemetry
    output wire        bumpless_active          // Strobe when bumpless transfer is active
);

    // ================================================================
    // 1. Control Tick Generator (Timing Scheduler)
    // ================================================================
    wire       inner_tick;
    wire       outer_tick;
    wire [2:0] tick_rate_sel;

    control_tick_generator #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ)
    ) tick_gen_inst (
        .clk          (clk),
        .rst          (rst),
        .rate_sel     (tick_rate_sel),
        .sensor_valid (sensor_valid),
        .inner_tick   (inner_tick),
        .outer_tick   (outer_tick)
    );

    // ================================================================
    // 2. Safety Watchdog Timer
    // ================================================================
    wire [31:0] watchdog_limit;
    wire        timeout_pulse;

    watchdog_timer watchdog_inst (
        .clk            (clk),
        .rst            (rst),
        .sensor_valid   (sensor_valid),
        .timeout_limit  (watchdog_limit),
        .watchdog_alarm (watchdog_alarm),
        .timeout_pulse  (timeout_pulse)
    );

    // ================================================================
    // 3. Flight Mode Manager
    // ================================================================
    wire [1:0] prev_mode;
    wire       mode_switch_strobe;
    wire [1:0] mode_override;
    wire       mode_override_en;
    wire       failsafe_trigger;

    mode_controller mode_ctrl_inst (
        .clk                (clk),
        .rst                (rst),
        .control_tick       (inner_tick),
        .mode_cmd           (mode_cmd),
        .mode_cmd_valid     (mode_cmd_valid),
        .mode_override      (mode_override),
        .mode_override_en   (mode_override_en),
        .failsafe_trigger   (failsafe_trigger),
        .current_mode       (current_mode),
        .prev_mode          (prev_mode),
        .mode_switch_strobe (mode_switch_strobe),
        .bumpless_active    (bumpless_active)
    );

    // ================================================================
    // 4. Multi-Bank Per-Mode Gain Storage
    // ================================================================
    wire        bank_wr_en;
    wire [7:0]  bank_wr_addr;
    wire [15:0] bank_wr_data;
    wire [7:0]  bank_rd_addr;
    wire [15:0] bank_rd_data;

    wire signed [15:0] active_att_kp;
    wire signed [15:0] active_rate_kp_rp;
    wire signed [15:0] active_rate_ki_rp;
    wire signed [15:0] active_rate_kd_rp;
    wire signed [15:0] active_rate_kp_yaw;
    wire signed [15:0] active_rate_ki_yaw;
    wire signed [15:0] active_rate_kd_yaw;
    wire signed [15:0] active_d_alpha;
    wire signed [15:0] active_max_out;
    wire signed [15:0] active_min_out;
    wire signed [15:0] active_max_i;
    wire signed [15:0] active_min_i;

    wire signed [15:0] active_alt_kp;
    wire signed [15:0] active_alt_vel_kp;
    wire signed [15:0] active_alt_vel_ki;
    wire signed [15:0] active_alt_vel_kd;
    wire signed [15:0] active_alt_throttle_clamp;

    wire signed [15:0] active_safe_motor_duty;
    wire               active_failsafe_policy;

    gain_bank gain_bank_inst (
        .clk                       (clk),
        .rst                       (rst),
        .wr_en                     (bank_wr_en),
        .wr_addr                   (bank_wr_addr),
        .wr_data                   (bank_wr_data),
        .rd_addr                   (bank_rd_addr),
        .rd_data                   (bank_rd_data),
        .mode_sel                  (current_mode),
        .active_att_kp             (active_att_kp),
        .active_rate_kp_rp         (active_rate_kp_rp),
        .active_rate_ki_rp         (active_rate_ki_rp),
        .active_rate_kd_rp         (active_rate_kd_rp),
        .active_rate_kp_yaw        (active_rate_kp_yaw),
        .active_rate_ki_yaw        (active_rate_ki_yaw),
        .active_rate_kd_yaw        (active_rate_kd_yaw),
        .active_d_alpha            (active_d_alpha),
        .active_max_out            (active_max_out),
        .active_min_out            (active_min_out),
        .active_max_i              (active_max_i),
        .active_min_i              (active_min_i),
        .active_alt_kp             (active_alt_kp),
        .active_alt_vel_kp         (active_alt_vel_kp),
        .active_alt_vel_ki         (active_alt_vel_ki),
        .active_alt_vel_kd         (active_alt_vel_kd),
        .active_alt_throttle_clamp (active_alt_throttle_clamp),
        .active_safe_motor_duty    (active_safe_motor_duty),
        .active_failsafe_policy    (active_failsafe_policy)
    );

    // ================================================================
    // 5. Configuration Registers & Bus Interface
    // ================================================================
    wire module_en;
    wire [5:0] rate_sat_flags;
    wire [3:0] motor_sat_flags;

    assign saturation_flags = {motor_sat_flags[1:0], rate_sat_flags};

    config_registers #(
        .DEFAULT_TICK_RATE (DEFAULT_TICK_RATE),
        .DEFAULT_WATCHDOG  (DEFAULT_WATCHDOG)
    ) cfg_regs_inst (
        .clk                       (clk),
        .rst                       (rst),
        .reg_wr_en                 (reg_wr_en),
        .reg_rd_en                 (reg_rd_en),
        .reg_addr                  (reg_addr),
        .reg_wr_data               (reg_wr_data),
        .reg_rd_data               (reg_rd_data),
        .reg_ack                   (reg_ack),
        .module_en                 (module_en),
        .mode_override             (mode_override),
        .mode_override_en          (mode_override_en),
        .tick_rate_sel             (tick_rate_sel),
        .watchdog_limit            (watchdog_limit),
        .telemetry_mode            (current_mode),
        .telemetry_watchdog_alarm  (watchdog_alarm),
        .telemetry_failsafe_active (failsafe_active),
        .telemetry_sat_flags       (saturation_flags),
        .bank_wr_en                (bank_wr_en),
        .bank_wr_addr              (bank_wr_addr),
        .bank_wr_data              (bank_wr_data),
        .bank_rd_addr              (bank_rd_addr),
        .bank_rd_data              (bank_rd_data)
    );

    // ================================================================
    // 6. Outer Attitude (Angle) Loop
    // ================================================================
    wire signed [15:0] target_roll_rate;
    wire signed [15:0] target_pitch_rate;
    wire signed [15:0] target_yaw_rate;

    attitude_controller att_ctrl_inst (
        .clk               (clk),
        .rst               (rst),
        .outer_tick        (outer_tick),
        .current_mode      (current_mode),
        .roll_sp           (roll_sp),
        .pitch_sp          (pitch_sp),
        .yaw_sp            (yaw_sp),
        .roll_meas         (roll_angle_meas),
        .pitch_meas        (pitch_angle_meas),
        .att_kp            (active_att_kp),
        .target_roll_rate  (target_roll_rate),
        .target_pitch_rate (target_pitch_rate),
        .target_yaw_rate   (target_yaw_rate)
    );

    // ================================================================
    // 7. Bumpless Transfer Engines (Roll, Pitch, Yaw)
    // ================================================================
    wire signed [15:0] roll_torque;
    wire signed [15:0] pitch_torque;
    wire signed [15:0] yaw_torque;

    wire signed [15:0] roll_p_term, roll_d_term;
    wire signed [15:0] pitch_p_term, pitch_d_term;
    wire signed [15:0] yaw_p_term, yaw_d_term;

    wire               reload_roll_i, reload_pitch_i, reload_yaw_i;
    wire signed [15:0] reload_roll_val, reload_pitch_val, reload_yaw_val;
    wire signed [15:0] disc_roll, disc_pitch, disc_yaw;

    bumpless_transfer bump_roll (
        .clk                (clk),
        .rst                (rst),
        .mode_switch_strobe (mode_switch_strobe),
        .u_prev             (roll_torque),
        .new_p_term         (roll_p_term),
        .new_d_term         (roll_d_term),
        .max_i              (active_max_i),
        .min_i              (active_min_i),
        .reload_strobe      (reload_roll_i),
        .new_i_state        (reload_roll_val),
        .discontinuity      (disc_roll)
    );

    bumpless_transfer bump_pitch (
        .clk                (clk),
        .rst                (rst),
        .mode_switch_strobe (mode_switch_strobe),
        .u_prev             (pitch_torque),
        .new_p_term         (pitch_p_term),
        .new_d_term         (pitch_d_term),
        .max_i              (active_max_i),
        .min_i              (active_min_i),
        .reload_strobe      (reload_pitch_i),
        .new_i_state        (reload_pitch_val),
        .discontinuity      (disc_pitch)
    );

    bumpless_transfer bump_yaw (
        .clk                (clk),
        .rst                (rst),
        .mode_switch_strobe (mode_switch_strobe),
        .u_prev             (yaw_torque),
        .new_p_term         (yaw_p_term),
        .new_d_term         (yaw_d_term),
        .max_i              (active_max_i),
        .min_i              (active_min_i),
        .reload_strobe      (reload_yaw_i),
        .new_i_state        (reload_yaw_val),
        .discontinuity      (disc_yaw)
    );

    // ================================================================
    // 8. Inner Fast Rate Loop (3x Parallel PIDs)
    // ================================================================
    wire rate_overflow;

    rate_controller rate_ctrl_inst (
        .clk              (clk),
        .rst              (rst),
        .inner_tick       (inner_tick),
        .target_roll_rate (target_roll_rate),
        .target_pitch_rate(target_pitch_rate),
        .target_yaw_rate  (target_yaw_rate),
        .roll_rate_meas   (roll_rate_meas),
        .pitch_rate_meas  (pitch_rate_meas),
        .yaw_rate_meas    (yaw_rate_meas),
        .rate_kp_rp       (active_rate_kp_rp),
        .rate_ki_rp       (active_rate_ki_rp),
        .rate_kd_rp       (active_rate_kd_rp),
        .rate_kp_yaw      (active_rate_kp_yaw),
        .rate_ki_yaw      (active_rate_ki_yaw),
        .rate_kd_yaw      (active_rate_kd_yaw),
        .d_alpha          (active_d_alpha),
        .max_out          (active_max_out),
        .min_out          (active_min_out),
        .max_i            (active_max_i),
        .min_i            (active_min_i),
        .reload_roll_i    (reload_roll_i),
        .reload_roll_val  (reload_roll_val),
        .reload_pitch_i   (reload_pitch_i),
        .reload_pitch_val (reload_pitch_val),
        .reload_yaw_i     (reload_yaw_i),
        .reload_yaw_val   (reload_yaw_val),
        .roll_torque      (roll_torque),
        .pitch_torque     (pitch_torque),
        .yaw_torque       (yaw_torque),
        .roll_p_term      (roll_p_term),
        .roll_d_term      (roll_d_term),
        .pitch_p_term     (pitch_p_term),
        .pitch_d_term     (pitch_d_term),
        .yaw_p_term       (yaw_p_term),
        .yaw_d_term       (yaw_d_term),
        .rate_sat_flags   (rate_sat_flags),
        .overflow         (rate_overflow)
    );

    // ================================================================
    // 9. Altitude Control Channel (Active in ALT_HOLD mode)
    // ================================================================
    reg signed [15:0] alt_trim_throttle;

    // Altitude error: altitude_sp - altitude_meas
    wire signed [16:0] alt_err_wide = {altitude_sp[15], altitude_sp} - {altitude_meas[15], altitude_meas};
    wire signed [15:0] alt_err = (alt_err_wide > 17'sh07FFF) ? 16'sh7FFF :
                                 (alt_err_wide < -17'sh08000) ? 16'sh8000 : alt_err_wide[15:0];

    wire signed [15:0] target_climb_rate;
    sat_mult_q88 mult_alt_kp (
        .a        (active_alt_kp),
        .b        (alt_err),
        .result   (target_climb_rate),
        .overflow ()
    );

    // Vertical velocity error: target_climb_rate - vert_vel_meas
    wire signed [16:0] vel_err_wide = {target_climb_rate[15], target_climb_rate} - {vert_vel_meas[15], vert_vel_meas};
    wire signed [15:0] vel_err = (vel_err_wide > 17'sh07FFF) ? 16'sh7FFF :
                                 (vel_err_wide < -17'sh08000) ? 16'sh8000 : vel_err_wide[15:0];

    wire signed [15:0] calc_alt_trim;
    sat_mult_q88 mult_vel_kp (
        .a        (active_alt_vel_kp),
        .b        (vel_err),
        .result   (calc_alt_trim),
        .overflow ()
    );

    // Clamp altitude throttle trim to active_alt_throttle_clamp
    wire signed [15:0] clamped_alt_trim;
    assign clamped_alt_trim = (calc_alt_trim > active_alt_throttle_clamp)  ? active_alt_throttle_clamp :
                              (calc_alt_trim < -active_alt_throttle_clamp) ? -active_alt_throttle_clamp :
                              calc_alt_trim;

    always @(posedge clk) begin
        if (rst) begin
            alt_trim_throttle <= 16'sh0000;
        end else if (current_mode == 2'b10) begin // ALT_HOLD
            if (inner_tick) begin
                alt_trim_throttle <= clamped_alt_trim;
            end
        end else begin
            alt_trim_throttle <= 16'sh0000;
        end
    end

    // Net Throttle = throttle_base + alt_trim_throttle
    wire signed [16:0] net_throttle_wide = {throttle_base[15], throttle_base} + {alt_trim_throttle[15], alt_trim_throttle};
    wire signed [15:0] net_throttle = (net_throttle_wide > 17'sh07FFF) ? 16'sh7FFF :
                                      (net_throttle_wide < 17'sh00000) ? 16'sh0000 : // No negative throttle
                                      net_throttle_wide[15:0];

    // ================================================================
    // 10. Actuator Mixer Matrix (X-Frame)
    // ================================================================
    wire [15:0] calc_m0, calc_m1, calc_m2, calc_m3;

    actuator_interface mixer_inst (
        .throttle        (net_throttle),
        .roll_torque     (roll_torque),
        .pitch_torque    (pitch_torque),
        .yaw_torque      (yaw_torque),
        .motor0_calc     (calc_m0),
        .motor1_calc     (calc_m1),
        .motor2_calc     (calc_m2),
        .motor3_calc     (calc_m3),
        .motor_sat_flags (motor_sat_flags)
    );

    // ================================================================
    // 11. Failsafe Safety Policy Executive
    // ================================================================
    failsafe_controller failsafe_inst (
        .clk              (clk),
        .rst              (rst),
        .current_mode     (current_mode),
        .watchdog_alarm   (watchdog_alarm),
        .failsafe_policy  (active_failsafe_policy),
        .safe_motor_duty  (active_safe_motor_duty),
        .calc_m0          (calc_m0),
        .calc_m1          (calc_m1),
        .calc_m2          (calc_m2),
        .calc_m3          (calc_m3),
        .final_m0         (motor0_cmd),
        .final_m1         (motor1_cmd),
        .final_m2         (motor2_cmd),
        .final_m3         (motor3_cmd),
        .failsafe_active  (failsafe_active),
        .failsafe_trigger (failsafe_trigger)
    );

    // Motor command valid strobe follows inner loop execution
    always @(posedge clk) begin
        if (rst) begin
            motor_valid <= 1'b0;
        end else begin
            motor_valid <= inner_tick;
        end
    end

endmodule
