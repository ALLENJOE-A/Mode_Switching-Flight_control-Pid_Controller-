`timescale 1ns / 1ps
//============================================================================
// Module:      rate_controller
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Inner rate loop implementing 3 parallel PID controllers for
//              roll, pitch, and yaw body angular rates. Operates at the fast
//              configurable rate (1–8 kHz) with bumpless state reload support.
//============================================================================

module rate_controller (
    input  wire               clk,
    input  wire               rst,
    input  wire               inner_tick,        // Inner loop execution strobe (1-8 kHz)

    // Desired rate setpoints (from attitude_controller or pilot sticks)
    input  wire signed [15:0] target_roll_rate,
    input  wire signed [15:0] target_pitch_rate,
    input  wire signed [15:0] target_yaw_rate,

    // Gyroscope feedback measurements (Signed Q8.8)
    input  wire signed [15:0] roll_rate_meas,
    input  wire signed [15:0] pitch_rate_meas,
    input  wire signed [15:0] yaw_rate_meas,

    // Active tuning gains from gain_bank
    input  wire signed [15:0] rate_kp_rp,
    input  wire signed [15:0] rate_ki_rp,
    input  wire signed [15:0] rate_kd_rp,
    input  wire signed [15:0] rate_kp_yaw,
    input  wire signed [15:0] rate_ki_yaw,
    input  wire signed [15:0] rate_kd_yaw,
    input  wire signed [15:0] d_alpha,
    input  wire signed [15:0] max_out,
    input  wire signed [15:0] min_out,
    input  wire signed [15:0] max_i,
    input  wire signed [15:0] min_i,

    // Bumpless transfer reload interfaces (one per axis)
    input  wire               reload_roll_i,
    input  wire signed [15:0] reload_roll_val,
    input  wire               reload_pitch_i,
    input  wire signed [15:0] reload_pitch_val,
    input  wire               reload_yaw_i,
    input  wire signed [15:0] reload_yaw_val,

    // Torque correction outputs (Signed Q8.8)
    output wire signed [15:0] roll_torque,
    output wire signed [15:0] pitch_torque,
    output wire signed [15:0] yaw_torque,

    // Prospective P and D terms for bumpless transfer engine
    output wire signed [15:0] roll_p_term,
    output wire signed [15:0] roll_d_term,
    output wire signed [15:0] pitch_p_term,
    output wire signed [15:0] pitch_d_term,
    output wire signed [15:0] yaw_p_term,
    output wire signed [15:0] yaw_d_term,

    // Saturation telemetry
    output wire [5:0]         rate_sat_flags,    // [0]: roll_high, [1]: roll_low, [2]: pitch_high, etc.
    output wire               overflow
);

    // Rate errors: target - measured
    wire signed [16:0] roll_err_w  = {target_roll_rate[15], target_roll_rate} - {roll_rate_meas[15], roll_rate_meas};
    wire signed [16:0] pitch_err_w = {target_pitch_rate[15], target_pitch_rate} - {pitch_rate_meas[15], pitch_rate_meas};
    wire signed [16:0] yaw_err_w   = {target_yaw_rate[15], target_yaw_rate} - {yaw_rate_meas[15], yaw_rate_meas};

    wire signed [15:0] roll_err  = (roll_err_w > 17'sh07FFF) ? 16'sh7FFF :
                                   (roll_err_w < -17'sh08000) ? 16'sh8000 : roll_err_w[15:0];
    wire signed [15:0] pitch_err = (pitch_err_w > 17'sh07FFF) ? 16'sh7FFF :
                                   (pitch_err_w < -17'sh08000) ? 16'sh8000 : pitch_err_w[15:0];
    wire signed [15:0] yaw_err   = (yaw_err_w > 17'sh07FFF) ? 16'sh7FFF :
                                   (yaw_err_w < -17'sh08000) ? 16'sh8000 : yaw_err_w[15:0];

    wire roll_sat_h, roll_sat_l, roll_ovf;
    wire pitch_sat_h, pitch_sat_l, pitch_ovf;
    wire yaw_sat_h, yaw_sat_l, yaw_ovf;

    // Roll Rate PID Instance
    pid_controller pid_roll (
        .clk            (clk),
        .rst            (rst),
        .sample_tick    (inner_tick),
        .Kp             (rate_kp_rp),
        .Ki             (rate_ki_rp),
        .Kd             (rate_kd_rp),
        .d_alpha        (d_alpha),
        .max_out        (max_out),
        .min_out        (min_out),
        .max_i          (max_i),
        .min_i          (min_i),
        .error_in       (roll_err),
        .meas_in        (roll_rate_meas),
        .reload_i_state (reload_roll_i),
        .reload_i_value (reload_roll_val),
        .pid_out        (roll_torque),
        .p_term_out     (roll_p_term),
        .i_term_out     (),
        .d_term_out     (roll_d_term),
        .sat_high       (roll_sat_h),
        .sat_low        (roll_sat_l),
        .overflow       (roll_ovf)
    );

    // Pitch Rate PID Instance
    pid_controller pid_pitch (
        .clk            (clk),
        .rst            (rst),
        .sample_tick    (inner_tick),
        .Kp             (rate_kp_rp),
        .Ki             (rate_ki_rp),
        .Kd             (rate_kd_rp),
        .d_alpha        (d_alpha),
        .max_out        (max_out),
        .min_out        (min_out),
        .max_i          (max_i),
        .min_i          (min_i),
        .error_in       (pitch_err),
        .meas_in        (pitch_rate_meas),
        .reload_i_state (reload_pitch_i),
        .reload_i_value (reload_pitch_val),
        .pid_out        (pitch_torque),
        .p_term_out     (pitch_p_term),
        .i_term_out     (),
        .d_term_out     (pitch_d_term),
        .sat_high       (pitch_sat_h),
        .sat_low        (pitch_sat_l),
        .overflow       (pitch_ovf)
    );

    // Yaw Rate PID Instance
    pid_controller pid_yaw (
        .clk            (clk),
        .rst            (rst),
        .sample_tick    (inner_tick),
        .Kp             (rate_kp_yaw),
        .Ki             (rate_ki_yaw),
        .Kd             (rate_kd_yaw),
        .d_alpha        (d_alpha),
        .max_out        (max_out),
        .min_out        (min_out),
        .max_i          (max_i),
        .min_i          (min_i),
        .error_in       (yaw_err),
        .meas_in        (yaw_rate_meas),
        .reload_i_state (reload_yaw_i),
        .reload_i_value (reload_yaw_val),
        .pid_out        (yaw_torque),
        .p_term_out     (yaw_p_term),
        .i_term_out     (),
        .d_term_out     (yaw_d_term),
        .sat_high       (yaw_sat_h),
        .sat_low        (yaw_sat_l),
        .overflow       (yaw_ovf)
    );

    assign rate_sat_flags = {yaw_sat_l, yaw_sat_h, pitch_sat_l, pitch_sat_h, roll_sat_l, roll_sat_h};
    assign overflow       = roll_ovf | pitch_ovf | yaw_ovf;

endmodule
