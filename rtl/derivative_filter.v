`timescale 1ns / 1ps
//============================================================================
// Module:      derivative_filter
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Low-pass filtered derivative computed on measurement feedback.
//              Eliminates setpoint derivative kick by computing rate of change
//              directly from sensor feedback:
//                d_raw[n]  = -(meas[n] - prev_meas[n-1]) = prev_meas - meas
//                d_filt[n] = d_filt[n-1] + alpha * (d_raw[n] - d_filt[n-1])
//              Filter coefficient alpha is signed Q8.8 (1.0 = 16'h0100).
//============================================================================

module derivative_filter (
    input  wire               clk,
    input  wire               rst,
    input  wire               sample_tick,       // Control update strobe
    input  wire signed [15:0] meas,              // Measurement feedback (Q8.8)
    input  wire signed [15:0] alpha,             // Filter coefficient (Q8.8, 0.0 to 1.0)
    input  wire               init_state,        // Force re-initialization (e.g., on mode change)
    output reg  signed [15:0] d_filt,            // Filtered derivative output (Q8.8)
    output wire signed [15:0] d_raw              // Raw derivative for diagnostics
);

    // Past state registers
    reg signed [15:0] prev_meas;
    reg signed [15:0] d_filt_reg;

    // Difference: - (meas[n] - prev_meas) = prev_meas - meas
    // Use 17-bit intermediate subtraction to prevent premature overflow
    wire signed [16:0] raw_diff_wide;
    assign raw_diff_wide = {prev_meas[15], prev_meas} - {meas[15], meas};

    // Saturate raw difference to 16-bit Q8.8
    assign d_raw = (raw_diff_wide > 17'sh07FFF) ? 16'sh7FFF :
                   (raw_diff_wide < -17'sh08000) ? 16'sh8000 :
                   raw_diff_wide[15:0];

    // Filter error: d_raw - d_filt_reg
    wire signed [16:0] filter_diff_wide;
    assign filter_diff_wide = {d_raw[15], d_raw} - {d_filt_reg[15], d_filt_reg};

    wire signed [15:0] filter_diff;
    assign filter_diff = (filter_diff_wide > 17'sh07FFF) ? 16'sh7FFF :
                         (filter_diff_wide < -17'sh08000) ? 16'sh8000 :
                         filter_diff_wide[15:0];

    // Filter adjustment = alpha * (d_raw - d_filt_reg)
    wire signed [15:0] filter_adj;
    wire               adj_overflow;

    sat_mult_q88 mult_alpha (
        .a        (alpha),
        .b        (filter_diff),
        .result   (filter_adj),
        .overflow (adj_overflow)
    );

    // Next filtered state: d_filt_reg + filter_adj
    wire signed [16:0] next_d_filt_wide;
    assign next_d_filt_wide = {d_filt_reg[15], d_filt_reg} + {filter_adj[15], filter_adj};

    wire signed [15:0] next_d_filt;
    assign next_d_filt = (next_d_filt_wide > 17'sh07FFF) ? 16'sh7FFF :
                         (next_d_filt_wide < -17'sh08000) ? 16'sh8000 :
                         next_d_filt_wide[15:0];

    // Sequential process
    always @(posedge clk) begin
        if (rst) begin
            prev_meas   <= 16'sh0000;
            d_filt_reg  <= 16'sh0000;
            d_filt      <= 16'sh0000;
        end else if (init_state) begin
            prev_meas   <= meas;
            d_filt_reg  <= 16'sh0000;
            d_filt      <= 16'sh0000;
        end else if (sample_tick) begin
            prev_meas   <= meas;
            d_filt_reg  <= next_d_filt;
            d_filt      <= next_d_filt;
        end
    end

endmodule
