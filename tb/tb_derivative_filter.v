`timescale 1ns / 1ps
//============================================================================
// Testbench:   tb_derivative_filter
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Self-Checking Testbench)
// Description: Unit test for derivative_filter module.
//              Verifies:
//              1. Zero derivative kick on setpoint step changes.
//              2. Filtered response to measurement step inputs.
//              3. Smooth exponential decay under configurable alpha.
//============================================================================

module tb_derivative_filter;

    reg               clk;
    reg               rst;
    reg               sample_tick;
    reg  signed [15:0] meas;
    reg  signed [15:0] setpoint;
    reg  signed [15:0] alpha;
    reg               init_state;

    wire signed [15:0] d_filt;
    wire signed [15:0] d_raw;

    derivative_filter uut (
        .clk         (clk),
        .rst         (rst),
        .sample_tick (sample_tick),
        .meas        (meas),
        .alpha       (alpha),
        .init_state  (init_state),
        .d_filt      (d_filt),
        .d_raw       (d_raw)
    );

    // Clock generator: 20ns period (50 MHz)
    initial clk = 0;
    always #10 clk = ~clk;

    integer err_count;

    initial begin
        err_count   = 0;
        rst         = 1;
        sample_tick = 0;
        meas        = 16'sh0000;
        setpoint    = 16'sh0000;
        alpha       = 16'sh0080; // alpha = 0.5 (Q8.8)
        init_state  = 0;

        #40;
        @(posedge clk);
        rst = 0;

        $display("============================================================");
        $display(" [TB_DERIVATIVE_FILTER] Starting Self-Checking Filter Tests");
        $display("============================================================");

        // ------------------------------------------------------------
        // Test 1: Settle initial state
        // ------------------------------------------------------------
        meas = 16'sh0500; // Constant +5.0 deg
        init_state = 1;
        @(posedge clk);
        init_state = 0;
        repeat (5) begin
            @(posedge clk);
            sample_tick = 1;
            @(posedge clk);
            sample_tick = 0;
        end

        // ------------------------------------------------------------
        // Test 2: Setpoint Step from 0 to +45.0 deg (meas unchanged)
        // Must produce EXACTLY ZERO derivative kick!
        // ------------------------------------------------------------
        $display(" Testing Setpoint Step: Changing setpoint from 0 to +45 deg...");
        setpoint = 16'sh2D00; // +45.0 deg
        // Meas remains at +5.0 deg

        repeat (5) begin
            @(posedge clk);
            sample_tick = 1;
            @(posedge clk);
            sample_tick = 0;

            if (d_filt !== 16'sh0000 || d_raw !== 16'sh0000) begin
                $display(" [FAIL] Derivative Kick detected! d_filt=0x%04h, d_raw=0x%04h", d_filt, d_raw);
                err_count = err_count + 1;
            end
        end

        if (err_count == 0) begin
            $display(" [PASS] Test 2: Zero Derivative Kick Verified! d_filt strictly 0x0000.");
        end

        // ------------------------------------------------------------
        // Test 3: Measurement Step from +5.0 deg to +15.0 deg (+10.0 step)
        // d_raw should equal -(15.0 - 5.0) = -10.0 deg (0xF600)
        // With alpha=0.5:
        // Tick 1: d_filt = 0 + 0.5 * (-10.0) = -5.0 deg (0xFB00)
        // Tick 2: d_raw = 0 (meas steady), d_filt = -5.0 + 0.5 * (0 - (-5.0)) = -2.5 deg (0xFD80)
        // ------------------------------------------------------------
        $display(" Testing Measurement Step: meas jumps from +5.0 to +15.0 deg...");
        meas = 16'sh0F00; // +15.0 deg

        // Tick 1
        @(posedge clk);
        sample_tick = 1;
        @(posedge clk);
        sample_tick = 0;
        #1;

        $display("   Tick 1: d_filt=0x%04h (exp 0xFB00)", d_filt);
        if (d_filt !== -16'sh0500) begin
            $display(" [FAIL] Test 3 Tick 1: d_filt mismatch! Got 0x%04h, expected 0x%04h", d_filt, -16'sh0500);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 3 Tick 1: Filter filtered step correctly (-5.0 deg)");
        end

        // Tick 2 (meas constant at +15.0 deg -> d_raw = 0)
        @(posedge clk);
        sample_tick = 1;
        @(posedge clk);
        sample_tick = 0;
        #1;

        $display("   Tick 2: d_filt=0x%04h (exp 0xFD80)", d_filt);
        if (d_filt !== -16'sh0280) begin
            $display(" [FAIL] Test 3 Tick 2: Smooth decay mismatch, got 0x%04h (exp 0xFD80)!", d_filt);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 3 Tick 2: Smooth decay verified (-2.5 deg)");
        end

        // Summary
        $display("============================================================");
        if (err_count == 0) begin
            $display(" *** TEST PASSED: DERIVATIVE FILTER VERIFIED ***");
        end else begin
            $display(" *** TEST FAILED: %0d ERRORS ENCOUNTERED ***", err_count);
        end
        $display("============================================================");
        $finish;
    end

    // Safety timeout
    initial begin
        #10000;
        $display(" [FATAL] tb_derivative_filter timeout!");
        $finish;
    end

endmodule
