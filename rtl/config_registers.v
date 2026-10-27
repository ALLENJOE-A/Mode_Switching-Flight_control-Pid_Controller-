`timescale 1ns / 1ps
//============================================================================
// Module:      config_registers
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Synchronous memory-mapped register interface.
//              Manages system control/status registers, watchdog limits,
//              and routes gain bank writes/reads.
//============================================================================

module config_registers #(
    parameter [2:0]  DEFAULT_TICK_RATE = 3'b010,
    parameter [31:0] DEFAULT_WATCHDOG  = 32'd500_000
)(
    input  wire        clk,
    input  wire        rst,

    // Bus interface
    input  wire        reg_wr_en,
    input  wire        reg_rd_en,
    input  wire [7:0]  reg_addr,
    input  wire [15:0] reg_wr_data,
    output reg  [15:0] reg_rd_data,
    output reg         reg_ack,

    // System registers to internal blocks
    output reg         module_en,
    output reg  [1:0]  mode_override,
    output reg         mode_override_en,
    output reg  [2:0]  tick_rate_sel,
    output reg  [31:0] watchdog_limit,

    // Telemetry status inputs for readback
    input  wire [1:0]  telemetry_mode,
    input  wire        telemetry_watchdog_alarm,
    input  wire        telemetry_failsafe_active,
    input  wire [7:0]  telemetry_sat_flags,

    // Interface to gain_bank
    output wire        bank_wr_en,
    output wire [7:0]  bank_wr_addr,
    output wire [15:0] bank_wr_data,
    output wire [7:0]  bank_rd_addr,
    input  wire [15:0] bank_rd_data
);

    // Pass-through to gain_bank for addresses >= 0x05
    assign bank_wr_en   = reg_wr_en && (reg_addr >= 8'h05);
    assign bank_wr_addr = reg_addr;
    assign bank_wr_data = reg_wr_data;
    assign bank_rd_addr = reg_addr;

    // Internal system control registers
    reg [15:0] sys_control_reg;
    reg [15:0] watchdog_hi_reg;
    reg [15:0] watchdog_lo_reg;

    always @(posedge clk) begin
        if (rst) begin
            sys_control_reg  <= 16'h0001; // Module enable = 1
            module_en        <= 1'b1;
            mode_override    <= 2'b00;
            mode_override_en <= 1'b0;
            tick_rate_sel    <= DEFAULT_TICK_RATE;
            watchdog_hi_reg  <= DEFAULT_WATCHDOG[31:16];
            watchdog_lo_reg  <= DEFAULT_WATCHDOG[15:0];
            watchdog_limit   <= DEFAULT_WATCHDOG;
            reg_ack          <= 1'b0;
            reg_rd_data      <= 16'h0000;
        end else begin
            reg_ack <= reg_wr_en | reg_rd_en;

            // Handle register writes
            if (reg_wr_en) begin
                case (reg_addr)
                    8'h00: begin
                        sys_control_reg  <= reg_wr_data;
                        module_en        <= reg_wr_data[0];
                        mode_override_en <= reg_wr_data[1];
                        mode_override    <= reg_wr_data[3:2];
                    end
                    8'h02: begin
                        tick_rate_sel <= reg_wr_data[2:0];
                    end
                    8'h03: begin
                        watchdog_hi_reg <= reg_wr_data;
                        watchdog_limit[31:16] <= reg_wr_data;
                    end
                    8'h04: begin
                        watchdog_lo_reg <= reg_wr_data;
                        watchdog_limit[15:0] <= reg_wr_data;
                    end
                    default: ; // Gain bank handles 0x05 and above
                endcase
            end

            // Handle register reads
            if (reg_rd_en) begin
                case (reg_addr)
                    8'h00: reg_rd_data <= sys_control_reg;
                    8'h01: reg_rd_data <= {4'h0, telemetry_sat_flags, telemetry_failsafe_active, telemetry_watchdog_alarm, telemetry_mode};
                    8'h02: reg_rd_data <= {13'h0, tick_rate_sel};
                    8'h03: reg_rd_data <= watchdog_hi_reg;
                    8'h04: reg_rd_data <= watchdog_lo_reg;
                    default: reg_rd_data <= bank_rd_data;
                endcase
            end
        end
    end

endmodule
