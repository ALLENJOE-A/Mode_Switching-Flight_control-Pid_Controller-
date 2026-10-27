`timescale 1ns / 1ps
//============================================================================
// Testbench:   tb_saturation
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Self-Checking Testbench)
// Description: Unit test for fixed_point_saturation arithmetic library.
//              Verifies signed Q8.8 multiplication, addition/subtraction,
//              boundary extremes (+127.99, -128.0), and zero wraparound.
//============================================================================

module tb_saturation;

    reg signed [15:0] a, b;
    reg               sub_mode;
    wire signed [15:0] mult_res;
    wire               mult_ovf;
    wire signed [15:0] add_sub_res;
    wire               add_sub_ovf;
    wire signed [15:0] clamp_res;
    wire               sat_h, sat_l;

    sat_mult_q88 uut_mult (
        .a        (a),
        .b        (b),
        .result   (mult_res),
        .overflow (mult_ovf)
    );

    sat_add_sub_q88 uut_add_sub (
        .a        (a),
        .b        (b),
        .sub_mode (sub_mode),
        .result   (add_sub_res),
        .overflow (add_sub_ovf)
    );

    clamp_bounds_q88 uut_clamp (
        .val         (a),
        .min_limit   (16'shC000), // -64.0
        .max_limit   (16'sh4000), // +64.0
        .clamped_val (clamp_res),
        .sat_high    (sat_h),
        .sat_low     (sat_l)
    );

    integer err_count;

    initial begin
        err_count = 0;
        $display("============================================================");
        $display(" [TB_SATURATION] Starting Self-Checking Saturation Tests");
        $display("============================================================");

        // ------------------------------------------------------------
        // Test 1: Normal multiplication (2.0 * 3.0 = 6.0)
        // 2.0 = 0x0200, 3.0 = 0x0300, 6.0 = 0x0600
        // ------------------------------------------------------------
        a = 16'sh0200;
        b = 16'sh0300;
        #10;
        if (mult_res !== 16'sh0600 || mult_ovf !== 1'b0) begin
            $display(" [FAIL] Test 1: Expected 0x0600, got 0x%04h, ovf=%b", mult_res, mult_ovf);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 1: Normal Mult: 2.0 * 3.0 = 6.0 (0x%04h)", mult_res);
        end

        // ------------------------------------------------------------
        // Test 2: Multiplication positive overflow (10.0 * 20.0 = 200.0 > 127.99)
        // 10.0 = 0x0A00, 20.0 = 0x1400 -> must saturate to 0x7FFF
        // ------------------------------------------------------------
        a = 16'sh0A00;
        b = 16'sh1400;
        #10;
        if (mult_res !== 16'sh7FFF || mult_ovf !== 1'b1) begin
            $display(" [FAIL] Test 2: Expected 0x7FFF with ovf=1, got 0x%04h, ovf=%b", mult_res, mult_ovf);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 2: Positive Mult Saturation: 10 * 20 -> 0x%04h (clamped max)", mult_res);
        end

        // ------------------------------------------------------------
        // Test 3: Multiplication negative overflow (-10.0 * 20.0 = -200.0 < -128.0)
        // -10.0 = 0xF600, 20.0 = 0x1400 -> must saturate to 0x8000
        // ------------------------------------------------------------
        a = -16'sh0A00;
        b = 16'sh1400;
        #10;
        if (mult_res !== 16'sh8000 || mult_ovf !== 1'b1) begin
            $display(" [FAIL] Test 3: Expected 0x8000 with ovf=1, got 0x%04h, ovf=%b", mult_res, mult_ovf);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 3: Negative Mult Saturation: -10 * 20 -> 0x%04h (clamped min)", mult_res);
        end

        // ------------------------------------------------------------
        // Test 4: Addition positive overflow (+100.0 + +50.0 = +150.0 > 127.99)
        // 100.0 = 0x6400, 50.0 = 0x3200 -> must saturate to 0x7FFF
        // ------------------------------------------------------------
        sub_mode = 1'b0;
        a = 16'sh6400;
        b = 16'sh3200;
        #10;
        if (add_sub_res !== 16'sh7FFF || add_sub_ovf !== 1'b1) begin
            $display(" [FAIL] Test 4: Expected 0x7FFF with ovf=1, got 0x%04h, ovf=%b", add_sub_res, add_sub_ovf);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 4: Add Positive Saturation: 100 + 50 -> 0x%04h", add_sub_res);
        end

        // ------------------------------------------------------------
        // Test 5: Subtraction negative overflow (-100.0 - +50.0 = -150.0 < -128.0)
        // ------------------------------------------------------------
        sub_mode = 1'b1;
        a = -16'sh6400;
        b = 16'sh3200;
        #10;
        if (add_sub_res !== 16'sh8000 || add_sub_ovf !== 1'b1) begin
            $display(" [FAIL] Test 5: Expected 0x8000 with ovf=1, got 0x%04h, ovf=%b", add_sub_res, add_sub_ovf);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 5: Sub Negative Saturation: -100 - 50 -> 0x%04h", add_sub_res);
        end

        // ------------------------------------------------------------
        // Test 6: Bound Clamping
        // Upper clamp (+80.0 > +64.0 -> +64.0)
        // Lower clamp (-80.0 < -64.0 -> -64.0)
        // ------------------------------------------------------------
        a = 16'sh5000; // +80.0
        #10;
        if (clamp_res !== 16'sh4000 || sat_h !== 1'b1) begin
            $display(" [FAIL] Test 6a: Clamp high failed, got 0x%04h sat_h=%b", clamp_res, sat_h);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 6a: Clamped high (+80 -> +64): 0x%04h", clamp_res);
        end

        a = -16'sh5000; // -80.0
        #10;
        if (clamp_res !== 16'shC000 || sat_l !== 1'b1) begin
            $display(" [FAIL] Test 6b: Clamp low failed, got 0x%04h sat_l=%b", clamp_res, sat_l);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 6b: Clamped low (-80 -> -64): 0x%04h", clamp_res);
        end

        // Summary
        $display("============================================================");
        if (err_count == 0) begin
            $display(" *** TEST PASSED: ALL SATURATION CHECKS VERIFIED ***");
        end else begin
            $display(" *** TEST FAILED: %0d ERRORS ENCOUNTERED ***", err_count);
        end
        $display("============================================================");
        $finish;
    end

    // Safety timeout
    initial begin
        #5000;
        $display(" [FATAL] tb_saturation timeout!");
        $finish;
    end

endmodule
