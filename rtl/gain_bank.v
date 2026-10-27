`timescale 1ns / 1ps
//============================================================================
// Module:      gain_bank
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Multi-bank storage array for per-mode PID coefficients, limits,
//              and derivative filter constants.
//              Bank 0: ACRO (Rate only)
//              Bank 1: STABILIZE (Attitude + Rate)
//              Bank 2: ALTITUDE_HOLD (Attitude + Rate + Vertical)
//              Bank 3: FAILSAFE (Safe fixed / hold-last constants)
//              Active gains are multiplexed atomically by mode_sel[1:0].
//============================================================================

module gain_bank (
    input  wire               clk,
    input  wire               rst,

    // Register write interface
    input  wire               wr_en,
    input  wire [7:0]         wr_addr,
    input  wire signed [15:0] wr_data,

    // Register read interface
    input  wire [7:0]         rd_addr,
    output reg  signed [15:0] rd_data,

    // Active mode selector
    input  wire [1:0]         mode_sel,

    // Active outputs to Attitude Outer Loop
    output reg  signed [15:0] active_att_kp,

    // Active outputs to Inner Rate Loop (Roll/Pitch/Yaw)
    output reg  signed [15:0] active_rate_kp_rp,
    output reg  signed [15:0] active_rate_ki_rp,
    output reg  signed [15:0] active_rate_kd_rp,
    output reg  signed [15:0] active_rate_kp_yaw,
    output reg  signed [15:0] active_rate_ki_yaw,
    output reg  signed [15:0] active_rate_kd_yaw,
    output reg  signed [15:0] active_d_alpha,
    output reg  signed [15:0] active_max_out,
    output reg  signed [15:0] active_min_out,
    output reg  signed [15:0] active_max_i,
    output reg  signed [15:0] active_min_i,

    // Active outputs to Altitude Channel
    output reg  signed [15:0] active_alt_kp,
    output reg  signed [15:0] active_alt_vel_kp,
    output reg  signed [15:0] active_alt_vel_ki,
    output reg  signed [15:0] active_alt_vel_kd,
    output reg  signed [15:0] active_alt_throttle_clamp,

    // Active Failsafe Parameters
    output reg  signed [15:0] active_safe_motor_duty,
    output reg                active_failsafe_policy
);

    // ================================================================
    // Register Storage Declarations with Default Reset Values
    // ================================================================

    // --- Bank 0: ACRO Registers (0x10 - 0x1D) ---
    reg signed [15:0] acro_rp_kp;
    reg signed [15:0] acro_rp_ki;
    reg signed [15:0] acro_rp_kd;
    reg signed [15:0] acro_yaw_kp;
    reg signed [15:0] acro_yaw_ki;
    reg signed [15:0] acro_yaw_kd;
    reg signed [15:0] acro_d_alpha;
    reg signed [15:0] acro_max_out;
    reg signed [15:0] acro_min_out;
    reg signed [15:0] acro_max_i;
    reg signed [15:0] acro_min_i;

    // --- Bank 1: STABILIZE Registers (0x20 - 0x2B) ---
    reg signed [15:0] stab_att_kp;
    reg signed [15:0] stab_rp_kp;
    reg signed [15:0] stab_rp_ki;
    reg signed [15:0] stab_rp_kd;
    reg signed [15:0] stab_yaw_kp;
    reg signed [15:0] stab_yaw_ki;
    reg signed [15:0] stab_yaw_kd;
    reg signed [15:0] stab_d_alpha;
    reg signed [15:0] stab_max_out;
    reg signed [15:0] stab_min_out;
    reg signed [15:0] stab_max_i;
    reg signed [15:0] stab_min_i;

    // --- Bank 2: ALTITUDE_HOLD Registers (0x30 - 0x34) ---
    reg signed [15:0] alt_kp;
    reg signed [15:0] alt_vel_kp;
    reg signed [15:0] alt_vel_ki;
    reg signed [15:0] alt_vel_kd;
    reg signed [15:0] alt_throttle_clamp;

    // --- Bank 3: FAILSAFE Registers (0x38 - 0x39, 0x05) ---
    reg signed [15:0] safe_motor_duty;
    reg               failsafe_policy; // 0: Safe fixed, 1: Hold last

    // ================================================================
    // Synchronous Write Process
    // ================================================================
    always @(posedge clk) begin
        if (rst) begin
            // Bank 0 (ACRO) defaults
            acro_rp_kp   <= 16'sh0180; // 1.5
            acro_rp_ki   <= 16'sh0008; // 0.03125
            acro_rp_kd   <= 16'sh0080; // 0.5
            acro_yaw_kp  <= 16'sh0100; // 1.0
            acro_yaw_ki  <= 16'sh0008; // 0.03125
            acro_yaw_kd  <= 16'sh0040; // 0.25
            acro_d_alpha <= 16'sh0080; // 0.5
            acro_max_out <= 16'sh5000; // +80.0
            acro_min_out <= -16'sh5000;// -80.0
            acro_max_i   <= 16'sh2000; // +32.0
            acro_min_i   <= -16'sh2000;// -32.0

            // Bank 1 (STABILIZE) defaults
            stab_att_kp  <= 16'sh0280; // 2.5
            stab_rp_kp   <= 16'sh0100; // 1.0
            stab_rp_ki   <= 16'sh000A; // 0.039
            stab_rp_kd   <= 16'sh0060; // 0.375
            stab_yaw_kp  <= 16'sh0100; // 1.0
            stab_yaw_ki  <= 16'sh0008; // 0.031
            stab_yaw_kd  <= 16'sh0040; // 0.25
            stab_d_alpha <= 16'sh0080; // 0.5
            stab_max_out <= 16'sh4000; // +64.0
            stab_min_out <= -16'sh4000;// -64.0
            stab_max_i   <= 16'sh1800; // +24.0
            stab_min_i   <= -16'sh1800;// -24.0

            // Bank 2 (ALTITUDE_HOLD) defaults
            alt_kp             <= 16'sh0180; // 1.5
            alt_vel_kp         <= 16'sh0100; // 1.0
            alt_vel_ki         <= 16'sh0006; // 0.023
            alt_vel_kd         <= 16'sh0040; // 0.25
            alt_throttle_clamp <= 16'sh1E00; // +/- 30.0%

            // Bank 3 (FAILSAFE) defaults
            safe_motor_duty <= 16'sh1900;    // 25.0%
            failsafe_policy <= 1'b0;         // 0: Safe fixed
        end else if (wr_en) begin
            case (wr_addr)
                // Bank 0 (ACRO)
                8'h10: acro_rp_kp   <= wr_data;
                8'h11: acro_rp_ki   <= wr_data;
                8'h12: acro_rp_kd   <= wr_data;
                8'h13: acro_yaw_kp  <= wr_data;
                8'h14: acro_yaw_ki  <= wr_data;
                8'h15: acro_yaw_kd  <= wr_data;
                8'h19: acro_d_alpha <= wr_data;
                8'h1A: acro_max_out <= wr_data;
                8'h1B: acro_min_out <= wr_data;
                8'h1C: acro_max_i   <= wr_data;
                8'h1D: acro_min_i   <= wr_data;

                // Bank 1 (STABILIZE)
                8'h20: stab_att_kp  <= wr_data;
                8'h21: stab_rp_kp   <= wr_data;
                8'h22: stab_rp_ki   <= wr_data;
                8'h23: stab_rp_kd   <= wr_data;
                8'h24: stab_yaw_kp  <= wr_data;
                8'h25: stab_yaw_ki  <= wr_data;
                8'h26: stab_yaw_kd  <= wr_data;
                8'h27: stab_d_alpha <= wr_data;
                8'h28: stab_max_out <= wr_data;
                8'h29: stab_min_out <= wr_data;
                8'h2A: stab_max_i   <= wr_data;
                8'h2B: stab_min_i   <= wr_data;

                // Bank 2 (ALT_HOLD)
                8'h30: alt_kp             <= wr_data;
                8'h31: alt_vel_kp         <= wr_data;
                8'h32: alt_vel_ki         <= wr_data;
                8'h33: alt_vel_kd         <= wr_data;
                8'h34: alt_throttle_clamp <= wr_data;

                // Bank 3 (FAILSAFE)
                8'h05: failsafe_policy <= wr_data[0];
                8'h38: safe_motor_duty <= wr_data;
                default: ; // Ignore undefined writes
            endcase
        end
    end

    // ================================================================
    // Combinational Readback
    // ================================================================
    always @(*) begin
        case (rd_addr)
            8'h05: rd_data = {15'b0, failsafe_policy};
            8'h10: rd_data = acro_rp_kp;
            8'h11: rd_data = acro_rp_ki;
            8'h12: rd_data = acro_rp_kd;
            8'h13: rd_data = acro_yaw_kp;
            8'h14: rd_data = acro_yaw_ki;
            8'h15: rd_data = acro_yaw_kd;
            8'h19: rd_data = acro_d_alpha;
            8'h1A: rd_data = acro_max_out;
            8'h1B: rd_data = acro_min_out;
            8'h1C: rd_data = acro_max_i;
            8'h1D: rd_data = acro_min_i;

            8'h20: rd_data = stab_att_kp;
            8'h21: rd_data = stab_rp_kp;
            8'h22: rd_data = stab_rp_ki;
            8'h23: rd_data = stab_rp_kd;
            8'h24: rd_data = stab_yaw_kp;
            8'h25: rd_data = stab_yaw_ki;
            8'h26: rd_data = stab_yaw_kd;
            8'h27: rd_data = stab_d_alpha;
            8'h28: rd_data = stab_max_out;
            8'h29: rd_data = stab_min_out;
            8'h2A: rd_data = stab_max_i;
            8'h2B: rd_data = stab_min_i;

            8'h30: rd_data = alt_kp;
            8'h31: rd_data = alt_vel_kp;
            8'h32: rd_data = alt_vel_ki;
            8'h33: rd_data = alt_vel_kd;
            8'h34: rd_data = alt_throttle_clamp;

            8'h38: rd_data = safe_motor_duty;
            default: rd_data = 16'sh0000;
        endcase
    end

    // ================================================================
    // Active Bank Multiplexing based on mode_sel
    // ================================================================
    always @(*) begin
        case (mode_sel)
            2'b00: begin // ACRO
                active_att_kp             = 16'sh0000; // Bypassed
                active_rate_kp_rp         = acro_rp_kp;
                active_rate_ki_rp         = acro_rp_ki;
                active_rate_kd_rp         = acro_rp_kd;
                active_rate_kp_yaw        = acro_yaw_kp;
                active_rate_ki_yaw        = acro_yaw_ki;
                active_rate_kd_yaw        = acro_yaw_kd;
                active_d_alpha            = acro_d_alpha;
                active_max_out            = acro_max_out;
                active_min_out            = acro_min_out;
                active_max_i              = acro_max_i;
                active_min_i              = acro_min_i;
                active_alt_kp             = 16'sh0000;
                active_alt_vel_kp         = 16'sh0000;
                active_alt_vel_ki         = 16'sh0000;
                active_alt_vel_kd         = 16'sh0000;
                active_alt_throttle_clamp = 16'sh0000;
                active_safe_motor_duty    = safe_motor_duty;
                active_failsafe_policy    = failsafe_policy;
            end

            2'b01: begin // STABILIZE
                active_att_kp             = stab_att_kp;
                active_rate_kp_rp         = stab_rp_kp;
                active_rate_ki_rp         = stab_rp_ki;
                active_rate_kd_rp         = stab_rp_kd;
                active_rate_kp_yaw        = stab_yaw_kp;
                active_rate_ki_yaw        = stab_yaw_ki;
                active_rate_kd_yaw        = stab_yaw_kd;
                active_d_alpha            = stab_d_alpha;
                active_max_out            = stab_max_out;
                active_min_out            = stab_min_out;
                active_max_i              = stab_max_i;
                active_min_i              = stab_min_i;
                active_alt_kp             = 16'sh0000;
                active_alt_vel_kp         = 16'sh0000;
                active_alt_vel_ki         = 16'sh0000;
                active_alt_vel_kd         = 16'sh0000;
                active_alt_throttle_clamp = 16'sh0000;
                active_safe_motor_duty    = safe_motor_duty;
                active_failsafe_policy    = failsafe_policy;
            end

            2'b10: begin // ALTITUDE_HOLD
                active_att_kp             = stab_att_kp;
                active_rate_kp_rp         = stab_rp_kp;
                active_rate_ki_rp         = stab_rp_ki;
                active_rate_kd_rp         = stab_rp_kd;
                active_rate_kp_yaw        = stab_yaw_kp;
                active_rate_ki_yaw        = stab_yaw_ki;
                active_rate_kd_yaw        = stab_yaw_kd;
                active_d_alpha            = stab_d_alpha;
                active_max_out            = stab_max_out;
                active_min_out            = stab_min_out;
                active_max_i              = stab_max_i;
                active_min_i              = stab_min_i;
                active_alt_kp             = alt_kp;
                active_alt_vel_kp         = alt_vel_kp;
                active_alt_vel_ki         = alt_vel_ki;
                active_alt_vel_kd         = alt_vel_kd;
                active_alt_throttle_clamp = alt_throttle_clamp;
                active_safe_motor_duty    = safe_motor_duty;
                active_failsafe_policy    = failsafe_policy;
            end

            default: begin // FAILSAFE (2'b11) and invalid encodings
                active_att_kp             = 16'sh0000;
                active_rate_kp_rp         = 16'sh0000;
                active_rate_ki_rp         = 16'sh0000;
                active_rate_kd_rp         = 16'sh0000;
                active_rate_kp_yaw        = 16'sh0000;
                active_rate_ki_yaw        = 16'sh0000;
                active_rate_kd_yaw        = 16'sh0000;
                active_d_alpha            = 16'sh0080;
                active_max_out            = 16'sh6400;
                active_min_out            = 16'sh0000;
                active_max_i              = 16'sh1000;
                active_min_i              = -16'sh1000;
                active_alt_kp             = 16'sh0000;
                active_alt_vel_kp         = 16'sh0000;
                active_alt_vel_ki         = 16'sh0000;
                active_alt_vel_kd         = 16'sh0000;
                active_alt_throttle_clamp = 16'sh0000;
                active_safe_motor_duty    = safe_motor_duty;
                active_failsafe_policy    = failsafe_policy;
            end
        endcase
    end

endmodule
