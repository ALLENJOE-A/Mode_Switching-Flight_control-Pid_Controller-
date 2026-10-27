`timescale 1ns / 1ps
//============================================================================
// Module:      bumpless_transfer
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Bumpless mode transfer calculation engine.
//              On a mode switch, calculates the new integrator state to preserve
//              actuator continuity:
//                I_target = u_previous - P_new - D_new
//                I_new    = clamp(I_target, min_i, max_i)
//              Generates reload strobe to atomically initialize PID state.
//============================================================================

module bumpless_transfer (
    input  wire               clk,
    input  wire               rst,
    input  wire               mode_switch_strobe,// Mode transition trigger pulse

    // Prior and prospective terms (Signed Q8.8)
    input  wire signed [15:0] u_prev,            // Actuator command prior to switch
    input  wire signed [15:0] new_p_term,        // Proportional term with new gain
    input  wire signed [15:0] new_d_term,        // Derivative term with new gain

    // Integral limits of newly selected mode
    input  wire signed [15:0] max_i,             // New mode upper integral limit
    input  wire signed [15:0] min_i,             // New mode lower integral limit

    // Outputs
    output reg                reload_strobe,     // Strobe to reload PID integrator
    output reg  signed [15:0] new_i_state,       // Clamped I_new state for PID
    output wire signed [15:0] discontinuity      // Absolute discontinuity magnitude
);

    // Wide intermediate calculation: u_prev - new_p_term - new_d_term
    wire signed [17:0] target_i_wide;
    assign target_i_wide = {{2{u_prev[15]}}, u_prev} -
                           {{2{new_p_term[15]}}, new_p_term} -
                           {{2{new_d_term[15]}}, new_d_term};

    // Saturate to 16-bit Q8.8
    wire signed [15:0] target_i_sat;
    assign target_i_sat = (target_i_wide > 18'sh07FFF) ? 16'sh7FFF :
                          (target_i_wide < -18'sh08000) ? 16'sh8000 :
                          target_i_wide[15:0];

    // Clamp to configured integral limits [min_i, max_i]
    wire signed [15:0] target_i_clamped;
    assign target_i_clamped = (target_i_sat > max_i) ? max_i :
                              (target_i_sat < min_i) ? min_i :
                              target_i_sat;

    // Discontinuity calculation: u_prev - (new_p_term + target_i_clamped + new_d_term)
    wire signed [17:0] actual_new_u_wide;
    assign actual_new_u_wide = {{2{new_p_term[15]}}, new_p_term} +
                               {{2{target_i_clamped[15]}}, target_i_clamped} +
                               {{2{new_d_term[15]}}, new_d_term};

    wire signed [17:0] disc_diff_wide;
    assign disc_diff_wide = {{2{u_prev[15]}}, u_prev} - actual_new_u_wide;

    wire signed [15:0] disc_signed;
    assign disc_signed = (disc_diff_wide > 18'sh07FFF) ? 16'sh7FFF :
                         (disc_diff_wide < -18'sh08000) ? 16'sh8000 :
                         disc_diff_wide[15:0];

    // Absolute value of discontinuity
    assign discontinuity = (disc_signed < 16'sh0000) ? -disc_signed : disc_signed;

    // Synchronous registered output
    always @(posedge clk) begin
        if (rst) begin
            reload_strobe <= 1'b0;
            new_i_state   <= 16'sh0000;
        end else begin
            reload_strobe <= mode_switch_strobe;
            if (mode_switch_strobe) begin
                new_i_state <= target_i_clamped;
            end
        end
    end

endmodule
