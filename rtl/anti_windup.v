`timescale 1ns / 1ps
//============================================================================
// Module:      anti_windup
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Conditional integration (anti-windup clamping) engine.
//              Inhibits integral accumulation when the controller output is
//              saturated and the error acts to drive it further into saturation.
//              Permits immediate integration as soon as the error reverses,
//              enabling fast recovery from saturation without integral lag.
//============================================================================

module anti_windup (
    // Inputs from PID datapath
    input  wire signed [15:0] error,             // Signed Q8.8 error
    input  wire signed [15:0] i_increment,       // Signed Q8.8 Ki * error term
    input  wire signed [15:0] raw_pid_out,       // Unclamped P + I + D sum
    input  wire signed [15:0] integrator_state,  // Current clamped integrator value

    // Configurable limits from active gain bank
    input  wire signed [15:0] max_out,           // Upper output clamp
    input  wire signed [15:0] min_out,           // Lower output clamp
    input  wire signed [15:0] max_i,             // Upper integrator clamp
    input  wire signed [15:0] min_i,             // Lower integrator clamp

    // Outputs
    output wire signed [15:0] clamped_pid_out,   // Output strictly clamped to [min_out, max_out]
    output wire signed [15:0] allowed_increment, // Increment gated by anti-windup logic
    output wire               output_sat_high,   // Output saturated at upper limit
    output wire               output_sat_low,    // Output saturated at lower limit
    output wire               integrator_sat_high,// Integrator at upper clamp
    output wire               integrator_sat_low  // Integrator at lower clamp
);

    // Output saturation detection
    assign output_sat_high = (raw_pid_out >= max_out);
    assign output_sat_low  = (raw_pid_out <= min_out);

    // Integrator saturation detection
    assign integrator_sat_high = (integrator_state >= max_i);
    assign integrator_sat_low  = (integrator_state <= min_i);

    // Conditional integration rules:
    // 1. Inhibit positive integration if output is saturated high OR integrator is at max_i
    wire inhibit_positive = (output_sat_high || integrator_sat_high) && (i_increment > 16'sh0000);

    // 2. Inhibit negative integration if output is saturated low OR integrator is at min_i
    wire inhibit_negative = (output_sat_low || integrator_sat_low) && (i_increment < 16'sh0000);

    // Gated increment fed to the accumulator
    assign allowed_increment = (inhibit_positive || inhibit_negative) ? 16'sh0000 : i_increment;

    // Final saturated PID output
    assign clamped_pid_out = output_sat_high ? max_out :
                             output_sat_low  ? min_out :
                             raw_pid_out;

endmodule
