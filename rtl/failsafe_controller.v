`timescale 1ns / 1ps
//============================================================================
// Module:      failsafe_controller
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Failsafe policy executive.
//              Enforces configured safety behavior when entering FAILSAFE:
//                Policy 0 (SAFE_FIXED): Applies safe descent duty (25%) with level torques.
//                Policy 1 (HOLD_LAST):  Holds last known valid motor commands.
//============================================================================

module failsafe_controller (
    input  wire               clk,
    input  wire               rst,
    input  wire [1:0]         current_mode,        // Active flight mode
    input  wire               watchdog_alarm,      // Sensor timeout alarm
    input  wire               failsafe_policy,     // 0: Safe Fixed, 1: Hold Last
    input  wire signed [15:0] safe_motor_duty,     // Configured safe descent duty

    // Normal calculated motor commands from mixer
    input  wire [15:0]        calc_m0,
    input  wire [15:0]        calc_m1,
    input  wire [15:0]        calc_m2,
    input  wire [15:0]        calc_m3,

    // Safety-enforced motor commands to actuators
    output reg  [15:0]        final_m0,
    output reg  [15:0]        final_m1,
    output reg  [15:0]        final_m2,
    output reg  [15:0]        final_m3,
    output wire               failsafe_active,     // Level flag
    output wire               failsafe_trigger     // Trigger to mode controller
);

    localparam [1:0] MODE_FAILSAFE = 2'b11;

    assign failsafe_active  = (current_mode == MODE_FAILSAFE) || watchdog_alarm;
    assign failsafe_trigger = watchdog_alarm;

    // Registers to latch last known valid motor commands
    reg [15:0] last_valid_m0;
    reg [15:0] last_valid_m1;
    reg [15:0] last_valid_m2;
    reg [15:0] last_valid_m3;

    always @(posedge clk) begin
        if (rst) begin
            last_valid_m0 <= 16'h0000;
            last_valid_m1 <= 16'h0000;
            last_valid_m2 <= 16'h0000;
            last_valid_m3 <= 16'h0000;
            final_m0      <= 16'h0000;
            final_m1      <= 16'h0000;
            final_m2      <= 16'h0000;
            final_m3      <= 16'h0000;
        end else begin
            // Update last valid state whenever system is healthy
            if (!failsafe_active) begin
                last_valid_m0 <= calc_m0;
                last_valid_m1 <= calc_m1;
                last_valid_m2 <= calc_m2;
                last_valid_m3 <= calc_m3;
                final_m0      <= calc_m0;
                final_m1      <= calc_m1;
                final_m2      <= calc_m2;
                final_m3      <= calc_m3;
            end else begin
                // In Failsafe mode: enforce configured policy
                if (failsafe_policy == 1'b1) begin
                    // Policy 1: Hold Last Value
                    final_m0 <= last_valid_m0;
                    final_m1 <= last_valid_m1;
                    final_m2 <= last_valid_m2;
                    final_m3 <= last_valid_m3;
                end else begin
                    // Policy 0: Safe Fixed Descent Duty
                    final_m0 <= safe_motor_duty;
                    final_m1 <= safe_motor_duty;
                    final_m2 <= safe_motor_duty;
                    final_m3 <= safe_motor_duty;
                end
            end
        end
    end

endmodule
