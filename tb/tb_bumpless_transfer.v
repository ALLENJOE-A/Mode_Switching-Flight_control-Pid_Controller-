`timescale 1ns / 1ps
//============================================================================
// Testbench:   tb_bumpless_transfer
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Self-Checking Testbench)
// Description: Unit test for bumpless_transfer engine.
//              Verifies:
//              1. Exact back-calculation: I_new = u_prev - P_new - D_new.
//              2. Output continuity guarantee across mode transitions.
//              3. Clamping to new mode's integral limits [min_i, max_i].
//              4. Discontinuity telemetry measurement.
//============================================================================

module tb_bumpless_transfer;

    reg               clk;
    reg               rst;
    reg               mode_switch_strobe;
    reg  signed [15:0] u_prev;
    reg  signed [15:0] new_p_term;
    reg  signed [15:0] new_d_term;
    reg  signed [15:0] max_i;
    reg  signed [15:0] min_i;

    wire               reload_strobe;
    wire signed [15:0] new_i_state;
    wire signed [15:0] discontinuity;

    bumpless_transfer uut (
        .clk                (clk),
        .rst                (rst),
        .mode_switch_strobe (mode_switch_strobe),
        .u_prev             (u_prev),
        .new_p_term         (new_p_term),
        .new_d_term         (new_d_term),
        .max_i              (max_i),
        .min_i              (min_i),
        .reload_strobe      (reload_strobe),
        .new_i_state        (new_i_state),
        .discontinuity      (discontinuity)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    integer err_count;

    initial begin
        err_count          = 0;
        rst                = 1;
        mode_switch_strobe = 0;
        u_prev             = 16'sh0000;
        new_p_term         = 16'sh0000;
        new_d_term         = 16'sh0000;
        max_i              = 16'sh2000; // +32.0
        min_i              = -16'sh2000;// -32.0

        #40;
        @(posedge clk);
        rst = 0;

        $display("============================================================");
        $display(" [TB_BUMPLESS_TRANSFER] Starting Bumpless Transfer Tests");
        $display("============================================================");

        // ------------------------------------------------------------
        // Test 1: Ideal Bumpless Transfer
        // u_prev = +30.0 (0x1E00), new_P = +10.0 (0x0A00), new_D = +2.0 (0x0200)
        // I_target = 30.0 - 10.0 - 2.0 = +18.0 (0x1200)
        // Discontinuity should be exactly 0x0000!
        // ------------------------------------------------------------
        u_prev     = 16'sh1E00; // +30.0
        new_p_term = 16'sh0A00; // +10.0
        new_d_term = 16'sh0200; // +2.0

        @(posedge clk);
        mode_switch_strobe = 1;
        @(posedge clk);
        mode_switch_strobe = 0;
        #1;

        $display(" Test 1: u_prev=+30, P_new=+10, D_new=+2 -> new_I = 0x%04h (exp 0x1200 = +18.0), disc = 0x%04h",
                 new_i_state, discontinuity);

        if (new_i_state !== 16'sh1200 || discontinuity !== 16'sh0000 || reload_strobe !== 1'b1) begin
            $display(" [FAIL] Test 1: Bumpless transfer calculation or discontinuity mismatch!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 1: Exact Bumpless Transfer Verified! Discontinuity = 0.0.");
        end

        // ------------------------------------------------------------
        // Test 2: Transfer with Integrator Clamping
        // u_prev = +50.0 (0x3200), new_P = +5.0 (0x0500), new_D = 0
        // I_target = 50.0 - 5.0 = +45.0 (0x2D00)
        // But max_i = +32.0 (0x2000) -> new_i_state must clamp to 0x2000!
        // Discontinuity should be |45.0 - 32.0| = +13.0 (0x0D00)
        // ------------------------------------------------------------
        u_prev     = 16'sh3200; // +50.0
        new_p_term = 16'sh0500; // +5.0
        new_d_term = 16'sh0000;

        @(posedge clk);
        mode_switch_strobe = 1;
        @(posedge clk);
        mode_switch_strobe = 0;
        #1;

        $display(" Test 2: u_prev=+50, P_new=+5 -> I_target=+45 clamped to max_i=0x%04h, disc=0x%04h (exp 0x0D00 = +13.0)",
                 new_i_state, discontinuity);

        if (new_i_state !== 16'sh2000 || discontinuity !== 16'sh0D00) begin
            $display(" [FAIL] Test 2: Clamping or bounded discontinuity mismatch!");
            err_count = err_count + 1;
        end else begin
            $display(" [PASS] Test 2: Clamping and Bounded Discontinuity Verified correctly.");
        end

        // Summary
        $display("============================================================");
        if (err_count == 0) begin
            $display(" *** TEST PASSED: BUMPLESS TRANSFER LOGIC VERIFIED ***");
        end else begin
            $display(" *** TEST FAILED: %0d ERRORS ENCOUNTERED ***", err_count);
        end
        $display("============================================================");
        $finish;
    end

    // Safety timeout
    initial begin
        #5000;
        $display(" [FATAL] tb_bumpless_transfer timeout!");
        $finish;
    end

endmodule
