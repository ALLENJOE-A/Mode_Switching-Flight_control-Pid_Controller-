`timescale 1ns / 1ps
//============================================================================
// Module:      mode_controller
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Central flight mode manager FSM.
//              Supports 4 modes:
//                2'b00: ACRO (Rate only)
//                2'b01: STABILIZE (Attitude outer + Rate inner)
//                2'b10: ALTITUDE_HOLD (Attitude + Rate + Vertical)
//                2'b11: FAILSAFE (Emergency policy)
//              Enforces atomic mode transitions at control cycle boundaries,
//              generating a synchronous strobe for bumpless transfer.
//============================================================================

module mode_controller (
    input  wire        clk,
    input  wire        rst,
    input  wire        control_tick,       // Control cycle boundary strobe

    // External mode requests
    input  wire [1:0]  mode_cmd,           // External mode command
    input  wire        mode_cmd_valid,     // External mode command strobe
    input  wire [1:0]  mode_override,      // Register override mode
    input  wire        mode_override_en,   // Register override enable

    // Safety inputs
    input  wire        failsafe_trigger,   // Emergency trigger from watchdog/safety

    // Outputs
    output reg  [1:0]  current_mode,       // Currently active flight mode
    output reg  [1:0]  prev_mode,          // Mode prior to last transition
    output reg         mode_switch_strobe, // 1-cycle pulse on mode change
    output reg         bumpless_active     // High during bumpless handover window
);

    // Mode encodings
    localparam [1:0] MODE_ACRO      = 2'b00;
    localparam [1:0] MODE_STABILIZE = 2'b01;
    localparam [1:0] MODE_ALT_HOLD  = 2'b10;
    localparam [1:0] MODE_FAILSAFE  = 2'b11;

    // Registered pending mode
    reg [1:0] pending_mode;
    reg       switch_pending;

    // Mode request arbitration
    wire [1:0] requested_mode;
    assign requested_mode = failsafe_trigger ? MODE_FAILSAFE :
                            mode_override_en ? mode_override :
                            mode_cmd;

    wire request_strobe;
    assign request_strobe = failsafe_trigger || (mode_override_en ? 1'b1 : mode_cmd_valid);

    always @(posedge clk) begin
        if (rst) begin
            current_mode       <= MODE_STABILIZE; // Safe level default out of reset
            prev_mode          <= MODE_STABILIZE;
            pending_mode       <= MODE_STABILIZE;
            switch_pending     <= 1'b0;
            mode_switch_strobe <= 1'b0;
            bumpless_active    <= 1'b0;
        end else begin
            mode_switch_strobe <= 1'b0;

            // 1. Detect and latch new mode requests
            if (failsafe_trigger && current_mode != MODE_FAILSAFE) begin
                // High-priority emergency trigger enters immediately or at boundary
                pending_mode   <= MODE_FAILSAFE;
                switch_pending <= 1'b1;
            end else if (request_strobe && (requested_mode != current_mode)) begin
                // Sanitize requested mode encoding
                case (requested_mode)
                    MODE_ACRO, MODE_STABILIZE, MODE_ALT_HOLD, MODE_FAILSAFE: begin
                        pending_mode   <= requested_mode;
                        switch_pending <= 1'b1;
                    end
                    default: begin
                        // Invalid encoding rejected, maintain current mode
                        switch_pending <= 1'b0;
                    end
                endcase
            end

            // 2. Execute transition at control cycle boundary
            if (control_tick && switch_pending) begin
                prev_mode          <= current_mode;
                current_mode       <= pending_mode;
                switch_pending     <= 1'b0;
                mode_switch_strobe <= 1'b1;
                bumpless_active    <= 1'b1;
            end else if (control_tick) begin
                bumpless_active <= 1'b0;
            end
        end
    end

endmodule
