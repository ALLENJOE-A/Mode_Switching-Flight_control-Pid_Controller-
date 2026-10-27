`timescale 1ns / 1ps
//============================================================================
// Module:      watchdog_timer
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Safety watchdog timer for detecting stale or missing IMU data.
//              Maintains a cycle counter reset by fresh sensor_valid pulses.
//              Asserts watchdog_alarm and triggers failsafe transition if
//              no fresh sensor data arrives within the programmable timeout.
//============================================================================

module watchdog_timer (
    input  wire        clk,
    input  wire        rst,
    input  wire        sensor_valid,       // Strobe on fresh sensor sample
    input  wire [31:0] timeout_limit,      // Programmable timeout in clock cycles
    output reg         watchdog_alarm,     // Latched alarm level
    output reg         timeout_pulse       // Single-cycle strobe on exact timeout cycle
);

    reg [31:0] counter;

    always @(posedge clk) begin
        if (rst) begin
            counter        <= 32'd0;
            watchdog_alarm <= 1'b0;
            timeout_pulse  <= 1'b0;
        end else if (sensor_valid) begin
            // Valid sample resets counter and clears alarm
            counter        <= 32'd0;
            watchdog_alarm <= 1'b0;
            timeout_pulse  <= 1'b0;
        end else begin
            timeout_pulse <= 1'b0;
            if (counter >= timeout_limit) begin
                watchdog_alarm <= 1'b1;
                // Counter saturates to prevent wraparound
                counter <= counter;
            end else begin
                counter <= counter + 1'b1;
                if (counter == timeout_limit - 1'b1) begin
                    timeout_pulse  <= 1'b1;
                    watchdog_alarm <= 1'b1;
                end
            end
        end
    end

endmodule
