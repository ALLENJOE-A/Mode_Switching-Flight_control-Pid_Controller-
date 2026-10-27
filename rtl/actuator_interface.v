`timescale 1ns / 1ps
//============================================================================
// Module:      actuator_interface
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Quadcopter X-frame actuator mixing matrix and output clamp.
//              Translates throttle command and 3-axis torque corrections
//              into 4 individual motor duty commands strictly clamped to
//              [0.0%, 100.0%] (16'h0000 to 16'h6400).
//============================================================================

module actuator_interface (
    input  wire signed [15:0] throttle,          // Net throttle (Base + Alt trim)
    input  wire signed [15:0] roll_torque,       // Roll torque correction (Q8.8)
    input  wire signed [15:0] pitch_torque,      // Pitch torque correction (Q8.8)
    input  wire signed [15:0] yaw_torque,        // Yaw torque correction (Q8.8)

    // Calculated 4-motor duty commands (Signed Q8.8, 0 to 100.0%)
    output wire [15:0]        motor0_calc,       // Front-Right (FR)
    output wire [15:0]        motor1_calc,       // Back-Left (BL)
    output wire [15:0]        motor2_calc,       // Front-Left (FL)
    output wire [15:0]        motor3_calc,       // Back-Right (BR)
    output wire [3:0]         motor_sat_flags    // High if any motor clamped at 100%
);

    // Motor floor (0%) and ceiling (100.0% in Q8.8 = 100 * 256 = 25600 = 16'h6400)
    localparam signed [15:0] MOTOR_MAX = 16'sh6400;
    localparam signed [15:0] MOTOR_MIN = 16'sh0000;

    // Wide intermediate sums to prevent arithmetic overflow
    wire signed [31:0] m0_raw = {{16{throttle[15]}}, throttle} -
                                {{16{roll_torque[15]}}, roll_torque} +
                                {{16{pitch_torque[15]}}, pitch_torque} +
                                {{16{yaw_torque[15]}}, yaw_torque};

    wire signed [31:0] m1_raw = {{16{throttle[15]}}, throttle} +
                                {{16{roll_torque[15]}}, roll_torque} -
                                {{16{pitch_torque[15]}}, pitch_torque} +
                                {{16{yaw_torque[15]}}, yaw_torque};

    wire signed [31:0] m2_raw = {{16{throttle[15]}}, throttle} +
                                {{16{roll_torque[15]}}, roll_torque} +
                                {{16{pitch_torque[15]}}, pitch_torque} -
                                {{16{yaw_torque[15]}}, yaw_torque};

    wire signed [31:0] m3_raw = {{16{throttle[15]}}, throttle} -
                                {{16{roll_torque[15]}}, roll_torque} -
                                {{16{pitch_torque[15]}}, pitch_torque} -
                                {{16{yaw_torque[15]}}, yaw_torque};

    // Clamp function to [0, +100.0%]
    function [15:0] clamp_motor;
        input signed [31:0] val;
        begin
            if (val > $signed({{16{1'b0}}, MOTOR_MAX}))
                clamp_motor = MOTOR_MAX;
            else if (val < 32'sd0)
                clamp_motor = MOTOR_MIN;
            else
                clamp_motor = val[15:0];
        end
    endfunction

    assign motor0_calc = clamp_motor(m0_raw);
    assign motor1_calc = clamp_motor(m1_raw);
    assign motor2_calc = clamp_motor(m2_raw);
    assign motor3_calc = clamp_motor(m3_raw);

    assign motor_sat_flags[0] = (m0_raw >= $signed({{16{1'b0}}, MOTOR_MAX}));
    assign motor_sat_flags[1] = (m1_raw >= $signed({{16{1'b0}}, MOTOR_MAX}));
    assign motor_sat_flags[2] = (m2_raw >= $signed({{16{1'b0}}, MOTOR_MAX}));
    assign motor_sat_flags[3] = (m3_raw >= $signed({{16{1'b0}}, MOTOR_MAX}));

endmodule
