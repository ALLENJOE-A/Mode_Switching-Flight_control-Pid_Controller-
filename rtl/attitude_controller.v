`timescale 1ns / 1ps
//============================================================================
// Module:      attitude_controller
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Outer attitude (angle) loop of the cascaded flight controller.
//              - In ACRO mode: Bypassed; setpoints pass directly as body rates.
//              - In STABILIZE / ALT_HOLD modes: Converts Euler angle tracking errors
//                into target angular rates (p_sp, q_sp) via proportional gain.
//============================================================================

module attitude_controller (
    input  wire               clk,
    input  wire               rst,
    input  wire               outer_tick,        // Outer loop cadence strobe
    input  wire [1:0]         current_mode,      // Active flight mode

    // Angle setpoints and measurements (Signed Q8.8)
    input  wire signed [15:0] roll_sp,           // Angle setpoint (or rate in ACRO)
    input  wire signed [15:0] pitch_sp,          // Angle setpoint (or rate in ACRO)
    input  wire signed [15:0] yaw_sp,            // Desired yaw rate
    input  wire signed [15:0] roll_meas,         // Measured Euler roll angle
    input  wire signed [15:0] pitch_meas,        // Measured Euler pitch angle

    // Tuning parameters from active bank
    input  wire signed [15:0] att_kp,            // Attitude P-gain (Q8.8)

    // Generated target angular rates for inner rate loop
    output reg  signed [15:0] target_roll_rate,
    output reg  signed [15:0] target_pitch_rate,
    output reg  signed [15:0] target_yaw_rate
);

    localparam [1:0] MODE_ACRO = 2'b00;

    // Angle errors: setpoint - measurement
    wire signed [16:0] roll_err_wide  = {roll_sp[15], roll_sp} - {roll_meas[15], roll_meas};
    wire signed [16:0] pitch_err_wide = {pitch_sp[15], pitch_sp} - {pitch_meas[15], pitch_meas};

    wire signed [15:0] roll_err = (roll_err_wide > 17'sh07FFF) ? 16'sh7FFF :
                                  (roll_err_wide < -17'sh08000) ? 16'sh8000 :
                                  roll_err_wide[15:0];

    wire signed [15:0] pitch_err = (pitch_err_wide > 17'sh07FFF) ? 16'sh7FFF :
                                   (pitch_err_wide < -17'sh08000) ? 16'sh8000 :
                                   pitch_err_wide[15:0];

    // Compute target rates: Kp_att * error
    wire signed [15:0] calc_roll_rate;
    wire signed [15:0] calc_pitch_rate;
    wire               ovf_r, ovf_p;

    sat_mult_q88 mult_att_roll (
        .a        (att_kp),
        .b        (roll_err),
        .result   (calc_roll_rate),
        .overflow (ovf_r)
    );

    sat_mult_q88 mult_att_pitch (
        .a        (att_kp),
        .b        (pitch_err),
        .result   (calc_pitch_rate),
        .overflow (ovf_p)
    );

    // Rate limits (Q8.8, e.g., max +/- 64.0 deg/s)
    localparam signed [15:0] MAX_RATE_CMD = 16'sh4000;
    localparam signed [15:0] MIN_RATE_CMD = -16'sh4000;

    wire signed [15:0] clamped_target_roll = (calc_roll_rate > MAX_RATE_CMD) ? MAX_RATE_CMD :
                                             (calc_roll_rate < MIN_RATE_CMD) ? MIN_RATE_CMD :
                                             calc_roll_rate;

    wire signed [15:0] clamped_target_pitch = (calc_pitch_rate > MAX_RATE_CMD) ? MAX_RATE_CMD :
                                              (calc_pitch_rate < MIN_RATE_CMD) ? MIN_RATE_CMD :
                                              calc_pitch_rate;

    always @(posedge clk) begin
        if (rst) begin
            target_roll_rate  <= 16'sh0000;
            target_pitch_rate <= 16'sh0000;
            target_yaw_rate   <= 16'sh0000;
        end else if (current_mode == MODE_ACRO) begin
            // In ACRO mode, pass pilot stick commands straight to rate loop
            target_roll_rate  <= roll_sp;
            target_pitch_rate <= pitch_sp;
            target_yaw_rate   <= yaw_sp;
        end else if (outer_tick) begin
            // In STABILIZE / ALT_HOLD, drive rates from attitude loop
            target_roll_rate  <= clamped_target_roll;
            target_pitch_rate <= clamped_target_pitch;
            target_yaw_rate   <= yaw_sp;
        end
    end

endmodule
