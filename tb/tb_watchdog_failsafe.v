`timescale 1ns / 1ps
//============================================================================
// Testbench:   tb_watchdog_failsafe
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Self-Checking Testbench)
// Description: Unit test for watchdog_timer and failsafe_controller.
//              Verifies:
//              1. Heartbeat monitoring under periodic valid sensor samples.
//              2. Exact cycle boundary timeout assertion.
//              3. Safe fixed descent duty policy enforcement.
//              4. Hold-last-value policy enforcement.
//              5. Auto-recovery when sensor data stream resumes.
//============================================================================

module tb_watchdog_failsafe;

    reg         clk;
    reg         rst;
    reg         sensor_valid;
    reg  [31:0] timeout_limit;
    reg  [1:0]  current_mode;
    reg         failsafe_policy;
    reg  signed [15:0] safe_motor_duty;
    reg  [15:0] calc_m0, calc_m1, calc_m2, calc_m3;

    wire        watchdog_alarm;
    wire        timeout_pulse;
    wire [15:0] final_m0, final_m1, final_m2, final_m3;
    wire        failsafe_active;
    wire        failsafe_trigger;

    watchdog_timer uut_wd (
        .clk            (clk),
        .rst            (rst),
        .sensor_valid   (sensor_valid),
        .timeout_limit  (timeout_limit),
        .watchdog_alarm (watchdog_alarm),
        .timeout_pulse  (timeout_pulse)
    );

    failsafe_controller uut_fs (
        .clk             (clk),
        .rst             (rst),
        .current_mode    (current_mode),
        .watchdog_alarm  (watchdog_alarm),
        .failsafe_policy (failsafe_policy),
        .safe_motor_duty (safe_motor_duty),
        .calc_m0         (calc_m0),
        .calc_m1         (calc_m1),
        .calc_m2         (calc_m2),
        .calc_m3         (calc_m3),
        .final_m0        (final_m0),
        .final_m1        (final_m1),
        .final_m2        (final_m2),
        .final_m3        (final_m3),
        .failsafe_active (failsafe_active),
        .failsafe_trigger(failsafe_trigger)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    integer err_count;

    initial begin
        err_count       = 0;
        rst             = 1;
        sensor_valid    = 0;
        timeout_limit   = 32'd20; // 20 clock cycles for fast unit testing
        current_mode    = 2'b01;  // STABILIZE
        failsafe_policy = 1'b0;   // Policy 0: Safe Fixed
        safe_motor_duty = 16'sh1900; // 25.0%
        calc_m0         = 16'h3200;  // 50.0%
        calc_m1         = 16'h3200;
        calc_m2         = 16'h3200;
        calc_m3         = 16'h3200;

        #40;
        @(posedge clk);
        rst = 0;

        $display("============================================================");
        $display(" [TB_WATCHDOG_FAILSAFE] Starting Safety & Fault Tests");
        $display("============================================================");

        // ------------------------------------------------------------
        // Test 1: Periodic Heartbeat (Normal Operation)
        // ------------------------------------------------------------
        repeat (5) begin
            repeat (10) @(posedge clk);
            sensor_valid = 1;
            @(posedge clk);
            sensor_valid = 0;
        end

        if (watchdog_alarm !== 1'b0 || failsafe_active !== 1'b0 || final_m0 !== calc_m0) begin
            $display(" [FAIL] Test 1: False alarm triggered during normal sensor stream!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 1: Normal Operation: Watchdog quiet, motors tracking calculated commands.");
        end

        // ------------------------------------------------------------
        // Test 2: Sensor Dropout & Exact Cycle Boundary Timeout
        // Cease sensor_valid and count exactly 20 cycles
        // ------------------------------------------------------------
        $display(" Ceasing sensor_valid to simulate sensor disconnection...");
        repeat (19) @(posedge clk);

        // Cycle 19: should NOT have timed out yet
        if (watchdog_alarm !== 1'b0) begin
            $display(" [FAIL] Test 2: Premature timeout before cycle limit!");
            err_count = err_count + 1;
        end

        // Cycle 20: exact boundary
        @(posedge clk);
        #1;
        if (watchdog_alarm !== 1'b1 || failsafe_trigger !== 1'b1) begin
            $display(" [FAIL] Test 2: Watchdog failed to trigger alarm at exact boundary cycle!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 2: Watchdog triggered alarm and failsafe at exact 20-cycle boundary.");
        end

        // ------------------------------------------------------------
        // Test 3: Failsafe Policy 0 (Safe Fixed Descent Duty = 25%)
        // ------------------------------------------------------------
        @(posedge clk);
        #1;
        if (final_m0 !== 16'sh1900 || final_m1 !== 16'sh1900) begin
            $display(" [FAIL] Test 3: Policy 0 failed! Expected safe duty 0x1900, got 0x%04h", final_m0);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 3: Failsafe Policy 0 Verified: All motors clamped to safe duty 25.0%% (0x1900).");
        end

        // ------------------------------------------------------------
        // Test 4: Sensor Recovery Handshake
        // Resuming sensor_valid must clear alarm and restore control
        // ------------------------------------------------------------
        $display(" Resuming fresh sensor stream...");
        @(posedge clk);
        sensor_valid = 1;
        @(posedge clk);
        sensor_valid = 0;
        @(posedge clk);
        #1;

        if (watchdog_alarm !== 1'b0 || failsafe_active !== 1'b0 || final_m0 !== calc_m0) begin
            $display(" [FAIL] Test 4: Failed to recover from failsafe after valid sensor sample!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 4: Sensor Recovery Verified: Alarm cleared, normal motor control restored.");
        end

        // ------------------------------------------------------------
        // Test 5: Failsafe Policy 1 (Hold Last Value)
        // ------------------------------------------------------------
        failsafe_policy = 1'b1; // Hold Last Value
        calc_m0 = 16'h3A00;     // 58.0%
        calc_m1 = 16'h3600;     // 54.0%
        calc_m2 = 16'h3400;     // 52.0%
        calc_m3 = 16'h3800;     // 56.0%
        @(posedge clk);
        @(posedge clk);

        // Induce timeout again
        repeat (22) @(posedge clk);
        #1;
        if (watchdog_alarm !== 1'b1) begin
            $display(" [FAIL] Test 5: Second timeout failed to assert!");
            err_count = err_count + 1;
        end

        // Change calculated motors during failure; final outputs MUST HOLD PREVIOUS VALUES!
        calc_m0 = 16'h7000;
        calc_m1 = 16'h7000;
        @(posedge clk);
        #1;
        if (final_m0 !== 16'h3A00 || final_m1 !== 16'h3600) begin
            $display(" [FAIL] Test 5: Policy 1 Hold-Last failed! Got m0=0x%04h (exp 0x3A00)", final_m0);
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 5: Failsafe Policy 1 (Hold Last Value) Verified: Outputs held frozen at last valid values.");
        end

        // Summary
        $display("============================================================");
        if (err_count == 0) begin
            $display(" *** TEST PASSED: WATCHDOG & FAILSAFE LOGIC FULLY VERIFIED ***");
        end else begin
            $display(" *** TEST FAILED: %0d ERRORS ENCOUNTERED ***", err_count);
        end
        $display("============================================================");
        $finish;
    end

    // Safety timeout
    initial begin
        #10000;
        $display(" [FATAL] tb_watchdog_failsafe timeout!");
        $finish;
    end

endmodule
