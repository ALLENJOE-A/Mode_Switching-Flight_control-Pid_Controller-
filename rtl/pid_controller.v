`timescale 1ns / 1ps
//============================================================================
// Module:      pid_controller
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Core PID controller engine with:
//              1. Low-pass filtered derivative computed on measurement feedback.
//              2. Anti-windup conditional integration and bounds clamping.
//              3. Integrator state reload port for bumpless mode transfer.
//              4. Signed Q8.8 fixed-point arithmetic with zero wraparound.
//============================================================================

module pid_controller (
    input  wire               clk,
    input  wire               rst,
    input  wire               sample_tick,       // Control execution strobe (1-8 kHz)

    // Dynamic gains and parameters from active gain bank
    input  wire signed [15:0] Kp,                // Proportional gain (Q8.8)
    input  wire signed [15:0] Ki,                // Integral gain (Q8.8)
    input  wire signed [15:0] Kd,                // Derivative gain (Q8.8)
    input  wire signed [15:0] d_alpha,           // Derivative filter alpha (Q8.8)
    input  wire signed [15:0] max_out,           // Maximum output clamp (Q8.8)
    input  wire signed [15:0] min_out,           // Minimum output clamp (Q8.8)
    input  wire signed [15:0] max_i,             // Maximum integrator clamp (Q8.8)
    input  wire signed [15:0] min_i,             // Minimum integrator clamp (Q8.8)

    // Tracking signals
    input  wire signed [15:0] error_in,          // Setpoint - Feedback (Q8.8)
    input  wire signed [15:0] meas_in,           // Feedback measurement for derivative

    // Bumpless transfer state reload interface
    input  wire               reload_i_state,    // Load new integrator state strobe
    input  wire signed [15:0] reload_i_value,    // Back-calculated integral value

    // Controller outputs
    output reg  signed [15:0] pid_out,           // Final clamped control output (Q8.8)
    output wire signed [15:0] p_term_out,        // Proportional contribution (Q8.8)
    output wire signed [15:0] i_term_out,        // Integral contribution (Q8.8)
    output wire signed [15:0] d_term_out,        // Derivative contribution (Q8.8)
    output wire               sat_high,          // Asserted when output is clamped high
    output wire               sat_low,           // Asserted when output is clamped low
    output wire               overflow           // Internal overflow flag
);

    // ----------------------------------------------------------------
    // 1. Proportional Term (Kp * error)
    // ----------------------------------------------------------------
    wire signed [15:0] p_term;
    wire               p_overflow;

    sat_mult_q88 mult_p (
        .a        (Kp),
        .b        (error_in),
        .result   (p_term),
        .overflow (p_overflow)
    );
    assign p_term_out = p_term;

    // ----------------------------------------------------------------
    // 2. Filtered Derivative Path (Kd * d_filt(meas))
    // ----------------------------------------------------------------
    wire signed [15:0] d_filt;
    wire signed [15:0] d_raw;

    derivative_filter d_filter_inst (
        .clk         (clk),
        .rst         (rst),
        .sample_tick (sample_tick),
        .meas        (meas_in),
        .alpha       (d_alpha),
        .init_state  (reload_i_state),
        .d_filt      (d_filt),
        .d_raw       (d_raw)
    );

    wire signed [15:0] d_term;
    wire               d_overflow;

    sat_mult_q88 mult_d (
        .a        (Kd),
        .b        (d_filt),
        .result   (d_term),
        .overflow (d_overflow)
    );
    assign d_term_out = d_term;

    // ----------------------------------------------------------------
    // 3. Integral Increment Calculation (Ki * error)
    // ----------------------------------------------------------------
    wire signed [15:0] i_increment;
    wire               i_overflow;

    sat_mult_q88 mult_i (
        .a        (Ki),
        .b        (error_in),
        .result   (i_increment),
        .overflow (i_overflow)
    );

    // ----------------------------------------------------------------
    // 4. Integrator Accumulator (Q16.16 32-bit register)
    // ----------------------------------------------------------------
    reg signed [31:0] integrator_acc;

    // Current integrator value in Q8.8
    wire signed [15:0] current_i_q88;
    assign current_i_q88 = integrator_acc[23:8];
    assign i_term_out    = current_i_q88;

    // ----------------------------------------------------------------
    // 5. Raw Sum: P + I + D
    // ----------------------------------------------------------------
    wire signed [31:0] raw_sum_wide;
    assign raw_sum_wide = {{16{p_term[15]}}, p_term} +
                          {{16{current_i_q88[15]}}, current_i_q88} +
                          {{16{d_term[15]}}, d_term};

    wire signed [15:0] raw_pid_out;
    assign raw_pid_out = (raw_sum_wide > 32'sh00007FFF) ? 16'sh7FFF :
                         (raw_sum_wide < -32'sh00008000) ? 16'sh8000 :
                         raw_sum_wide[15:0];

    // ----------------------------------------------------------------
    // 6. Anti-Windup Clamping and Conditional Integration
    // ----------------------------------------------------------------
    wire signed [15:0] allowed_increment;
    wire signed [15:0] final_pid_out;
    wire               i_sat_high;
    wire               i_sat_low;

    anti_windup aw_inst (
        .error               (error_in),
        .i_increment         (i_increment),
        .raw_pid_out         (raw_pid_out),
        .integrator_state    (current_i_q88),
        .max_out             (max_out),
        .min_out             (min_out),
        .max_i               (max_i),
        .min_i               (min_i),
        .clamped_pid_out     (final_pid_out),
        .allowed_increment   (allowed_increment),
        .output_sat_high     (sat_high),
        .output_sat_low      (sat_low),
        .integrator_sat_high (i_sat_high),
        .integrator_sat_low  (i_sat_low)
    );

    // Calculate next integrator accumulator
    wire signed [31:0] next_acc_candidate;
    // Shift increment into Q16.16 domain
    assign next_acc_candidate = integrator_acc + {{8{allowed_increment[15]}}, allowed_increment, 8'h00};

    // Limits in 32-bit Q16.16 domain
    wire signed [31:0] max_i_acc = {{8{max_i[15]}}, max_i, 8'h00};
    wire signed [31:0] min_i_acc = {{8{min_i[15]}}, min_i, 8'h00};

    wire signed [31:0] clamped_next_acc;
    assign clamped_next_acc = (next_acc_candidate > max_i_acc) ? max_i_acc :
                              (next_acc_candidate < min_i_acc) ? min_i_acc :
                              next_acc_candidate;

    // Overflow indicator
    assign overflow = p_overflow | d_overflow | i_overflow | (raw_sum_wide > 32'sh00007FFF) | (raw_sum_wide < -32'sh00008000);

    // ----------------------------------------------------------------
    // 7. Synchronous State Update
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            integrator_acc <= 32'sh0000_0000;
            pid_out        <= 16'sh0000;
        end else if (reload_i_state) begin
            // Atomic state reload for bumpless transfer
            integrator_acc <= {{8{reload_i_value[15]}}, reload_i_value, 8'h00};
            pid_out        <= final_pid_out;
        end else if (sample_tick) begin
            integrator_acc <= clamped_next_acc;
            pid_out        <= final_pid_out;
        end
    end

endmodule
