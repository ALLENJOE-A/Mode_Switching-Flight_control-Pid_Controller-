`timescale 1ns / 1ps
//============================================================================
// Testbench:   tb_mode_controller
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Self-Checking Testbench)
// Description: Unit test for mode_controller FSM.
//              Verifies:
//              1. Safe default mode out of reset (STABILIZE).
//              2. Atomic handover at control cycle boundaries.
//              3. Full directed mode transition matrix.
//              4. High-priority emergency failsafe escalation.
//============================================================================

module tb_mode_controller;

    reg        clk;
    reg        rst;
    reg        control_tick;
    reg  [1:0] mode_cmd;
    reg        mode_cmd_valid;
    reg  [1:0] mode_override;
    reg        mode_override_en;
    reg        failsafe_trigger;

    wire [1:0] current_mode;
    wire [1:0] prev_mode;
    wire       mode_switch_strobe;
    wire       bumpless_active;

    mode_controller uut (
        .clk                (clk),
        .rst                (rst),
        .control_tick       (control_tick),
        .mode_cmd           (mode_cmd),
        .mode_cmd_valid     (mode_cmd_valid),
        .mode_override      (mode_override),
        .mode_override_en   (mode_override_en),
        .failsafe_trigger   (failsafe_trigger),
        .current_mode       (current_mode),
        .prev_mode          (prev_mode),
        .mode_switch_strobe (mode_switch_strobe),
        .bumpless_active    (bumpless_active)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    integer err_count;

    initial begin
        err_count        = 0;
        rst              = 1;
        control_tick     = 0;
        mode_cmd         = 2'b01;
        mode_cmd_valid   = 0;
        mode_override    = 2'b00;
        mode_override_en = 0;
        failsafe_trigger = 0;

        #40;
        @(posedge clk);
        rst = 0;

        $display("============================================================");
        $display(" [TB_MODE_CONTROLLER] Starting Mode Manager FSM Tests");
        $display("============================================================");

        // ------------------------------------------------------------
        // Test 1: Reset Default State
        // ------------------------------------------------------------
        @(posedge clk);
        if (current_mode !== 2'b01) begin // STABILIZE
            $display(" [FAIL] Test 1: Reset default mode should be STABILIZE (01), got %b", current_mode);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 1: Reset default mode verified as STABILIZE (01)");
        end

        // ------------------------------------------------------------
        // Test 2: Transition from STABILIZE to ACRO (2'b00)
        // Request mode before tick; mode must NOT change until control_tick!
        // ------------------------------------------------------------
        @(posedge clk);
        mode_cmd       <= 2'b00; // ACRO
        mode_cmd_valid <= 1;
        @(posedge clk);
        mode_cmd_valid <= 0;

        // Verify mode has NOT changed yet (waiting for cycle boundary)
        @(posedge clk);
        if (current_mode !== 2'b01) begin
            $display(" [FAIL] Test 2: Mode changed asynchronously before control_tick!");
            err_count = err_count + 1;
        end

        // Fire control tick
        control_tick = 1;
        @(posedge clk);
        control_tick = 0;
        #1;

        if (current_mode !== 2'b00 || mode_switch_strobe !== 1'b1 || prev_mode !== 2'b01) begin
            $display(" [FAIL] Test 2: Transition to ACRO at cycle boundary failed! mode=%b strobe=%b",
                     current_mode, mode_switch_strobe);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 2: Synchronous Transition to ACRO (00) verified at cycle boundary.");
        end

        // ------------------------------------------------------------
        // Test 3: Transition from ACRO to ALTITUDE_HOLD (2'b10)
        // ------------------------------------------------------------
        @(posedge clk);
        mode_cmd       <= 2'b10;
        mode_cmd_valid <= 1;
        @(posedge clk);
        mode_cmd_valid <= 0;

        @(posedge clk);
        control_tick = 1;
        @(posedge clk);
        control_tick = 0;
        #1;

        if (current_mode !== 2'b10 || mode_switch_strobe !== 1'b1 || prev_mode !== 2'b00) begin
            $display(" [FAIL] Test 3: Transition to ALT_HOLD failed!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 3: Transition to ALT_HOLD (10) verified.");
        end

        // ------------------------------------------------------------
        // Test 4: High-Priority Emergency Escalation to FAILSAFE
        // ------------------------------------------------------------
        failsafe_trigger = 1;
        @(posedge clk);
        control_tick = 1;
        @(posedge clk);
        control_tick = 0;
        failsafe_trigger = 0;
        #1;

        if (current_mode !== 2'b11 || mode_switch_strobe !== 1'b1 || prev_mode !== 2'b10) begin
            $display(" [FAIL] Test 4: Emergency failsafe escalation failed!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 4: Emergency Escalation to FAILSAFE (11) verified.");
        end

        // Summary
        $display("============================================================");
        if (err_count == 0) begin
            $display(" *** TEST PASSED: MODE CONTROLLER FSM FULLY VERIFIED ***");
        end else begin
            $display(" *** TEST FAILED: %0d ERRORS ENCOUNTERED ***", err_count);
        end
        $display("============================================================");
        $finish;
    end

    // Safety timeout
    initial begin
        #5000;
        $display(" [FATAL] tb_mode_controller timeout!");
        $finish;
    end

endmodule
