`timescale 1ns / 1ps
//============================================================================
// Module:      control_tick_generator
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Synthesizable timing enable generator.
//              Divides system clock (50 MHz) to produce deterministic single-cycle
//              strobe pulses for inner rate loop (1–8 kHz) and outer attitude loop.
//              Includes direct test mode for high-speed simulation.
//============================================================================

module control_tick_generator #(
    parameter CLK_FREQ_HZ = 50_000_000
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [2:0] rate_sel,          // 000: 1kHz, 001: 2kHz, 010: 4kHz, 011: 8kHz, 1xx: Fast sim (32 cycles)
    input  wire       sensor_valid,      // Sensor input valid pulse
    output reg        inner_tick,        // Inner rate loop execution strobe (1-8 kHz)
    output reg        outer_tick         // Outer attitude loop execution strobe (500 Hz - 2 kHz)
);

    // Division modulus constants for 50 MHz system clock
    localparam [15:0] DIV_8KHZ = 16'd6250;
    localparam [15:0] DIV_4KHZ = 16'd12500;
    localparam [15:0] DIV_2KHZ = 16'd25000;
    localparam [15:0] DIV_1KHZ = 16'd50000;
    localparam [15:0] DIV_TEST = 16'd32;     // Fast simulation tick

    reg [15:0] target_count;
    reg [15:0] counter;
    reg [2:0]  outer_divider_cnt;

    // Dynamically select division modulus
    always @(*) begin
        case (rate_sel)
            3'b000:  target_count = DIV_1KHZ;
            3'b001:  target_count = DIV_2KHZ;
            3'b010:  target_count = DIV_4KHZ;
            3'b011:  target_count = DIV_8KHZ;
            default: target_count = DIV_TEST;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            counter           <= 16'd0;
            inner_tick        <= 1'b0;
            outer_tick        <= 1'b0;
            outer_divider_cnt <= 3'd0;
        end else begin
            inner_tick <= 1'b0;
            outer_tick <= 1'b0;

            if (counter >= target_count - 1'b1) begin
                counter    <= 16'd0;
                inner_tick <= 1'b1;

                // Outer loop runs at 1/2 or 1/4 the rate of the inner loop (e.g., 1/2)
                if (outer_divider_cnt >= 3'd1) begin
                    outer_divider_cnt <= 3'd0;
                    outer_tick        <= 1'b1;
                end else begin
                    outer_divider_cnt <= outer_divider_cnt + 1'b1;
                end
            end else begin
                counter <= counter + 1'b1;
            end
        end
    end

endmodule
