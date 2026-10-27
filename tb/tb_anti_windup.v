`timescale 1ns / 1ps
//============================================================================
// Testbench:   tb_anti_windup
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Self-Checking Testbench)
// Description: Unit test for anti_windup conditional integration module.
//              Verifies:
//              1. Inhabitation of positive integration when output saturated high.
//              2. Inhabitation of negative integration when output saturated low.
//              3. Immediate recovery when error reverses sign.
//              4. Accumulator clamping to [min_i, max_i].
//============================================================================

module tb_anti_windup;

    reg  signed [15:0] error;
    reg  signed [15:0] i_increment;
    reg  signed [15:0] raw_pid_out;
    reg  signed [15:0] integrator_state;
    reg  signed [15:0] max_out;
    reg  signed [15:0] min_out;
    reg  signed [15:0] max_i;
    reg  signed [15:0] min_i;

    wire signed [15:0] clamped_pid_out;
    wire signed [15:0] allowed_increment;
    wire               output_sat_high;
    wire               output_sat_low;
    wire               integrator_sat_high;
    wire               integrator_sat_low;

    anti_windup uut (
        .error               (error),
        .i_increment         (i_increment),
        .raw_pid_out         (raw_pid_out),
        .integrator_state    (integrator_state),
        .max_out             (max_out),
        .min_out             (min_out),
        .max_i               (max_i),
        .min_i               (min_i),
        .clamped_pid_out     (clamped_pid_out),
        .allowed_increment   (allowed_increment),
        .output_sat_high     (output_sat_high),
        .output_sat_low      (output_sat_low),
        .integrator_sat_high (integrator_sat_high),
        .integrator_sat_low  (integrator_sat_low)
    );

    integer err_count;

    initial begin
        err_count = 0;
        max_out = 16'sh4000;  // +64.0
        min_out = -16'sh4000; // -64.0
        max_i   = 16'sh2000;  // +32.0
        min_i   = -16'sh2000; // -32.0

        $display("============================================================");
        $display(" [TB_ANTI_WINDUP] Starting Anti-Windup Clamping Tests");
        $display("============================================================");

        // ------------------------------------------------------------
        // Test 1: Normal linear operation (not saturated)
        // ------------------------------------------------------------
        raw_pid_out      = 16'sh1000; // +16.0
        integrator_state = 16'sh0800; // +8.0
        i_increment      = 16'sh0100; // +1.0
        error            = 16'sh0500;
        #10;
        if (allowed_increment !== 16'sh0100 || clamped_pid_out !== 16'sh1000) begin
            $display(" [FAIL] Test 1: Normal linear operation failed!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 1: Normal Linear Operation: allowed_inc = 0x%04h", allowed_increment);
        end

        // ------------------------------------------------------------
        // Test 2: Output saturated high (+80.0 > +64.0) with positive error
        // Integration MUST be inhibited (allowed_increment = 0)
        // ------------------------------------------------------------
        raw_pid_out = 16'sh5000; // +80.0
        i_increment = 16'sh0200; // +2.0 (wants to integrate further positive)
        error       = 16'sh1000;
        #10;
        if (allowed_increment !== 16'sh0000 || output_sat_high !== 1'b1 || clamped_pid_out !== 16'sh4000) begin
            $display(" [FAIL] Test 2: Anti-windup failed to inhibit positive integration when saturated high!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 2: Positive Saturation: Output clamped to 0x%04h, integration gated to 0.", clamped_pid_out);
        end

        // ------------------------------------------------------------
        // Test 3: Output saturated high, but error REVERSES (negative increment)
        // Integration MUST BE PERMITTED immediately to un-saturate!
        // ------------------------------------------------------------
        raw_pid_out = 16'sh5000;  // Still saturated at +80.0
        i_increment = -16'sh0200; // Negative increment (-2.0)
        error       = -16'sh1000; // Error has reversed
        #10;
        if (allowed_increment !== -16'sh0200) begin
            $display(" [FAIL] Test 3: Failed to permit negative integration during high saturation!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 3: Saturation Recovery: Negative integration allowed (0x%04h) to un-saturate.", allowed_increment);
        end

        // ------------------------------------------------------------
        // Test 4: Output saturated low (-80.0 < -64.0) with negative error
        // Integration MUST be inhibited (allowed_increment = 0)
        // ------------------------------------------------------------
        raw_pid_out = -16'sh5000; // -80.0
        i_increment = -16'sh0200; // -2.0
        error       = -16'sh1000;
        #10;
        if (allowed_increment !== 16'sh0000 || output_sat_low !== 1'b1 || clamped_pid_out !== -16'sh4000) begin
            $display(" [FAIL] Test 4: Anti-windup failed to inhibit negative integration when saturated low!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 4: Negative Saturation: Output clamped to 0x%04h, integration gated to 0.", clamped_pid_out);
        end

        // ------------------------------------------------------------
        // Test 5: Integrator at max limit (+32.0)
        // Further positive integration inhibited
        // ------------------------------------------------------------
        raw_pid_out      = 16'sh2000; // Output is +32.0 (not saturated)
        integrator_state = 16'sh2000; // Integrator is at max_i (+32.0)
        i_increment      = 16'sh0100; // Positive increment
        #10;
        if (allowed_increment !== 16'sh0000 || integrator_sat_high !== 1'b1) begin
            $display(" [FAIL] Test 5: Failed to clamp integrator at max_i limit!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 5: Integrator Accumulator Clamp: Clamped at max_i, increment gated to 0.");
        end

        // Summary
        $display("============================================================");
        if (err_count == 0) begin
            $display(" *** TEST PASSED: ANTI-WINDUP LOGIC FULLY VERIFIED ***");
        end else begin
            $display(" *** TEST FAILED: %0d ERRORS ENCOUNTERED ***", err_count);
        end
        $display("============================================================");
        $finish;
    end

    // Safety timeout
    initial begin
        #5000;
        $display(" [FATAL] tb_anti_windup timeout!");
        $finish;
    end

endmodule
