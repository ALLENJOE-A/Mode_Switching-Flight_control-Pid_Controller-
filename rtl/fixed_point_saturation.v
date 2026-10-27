`timescale 1ns / 1ps
//============================================================================
// Module:      fixed_point_saturation
// Project:     PS-02 Mode-Switching Hardware PID Controller
// Standard:    Verilog-2001 (Synthesizable ASIC RTL)
// Description: Library of synthesizable saturating arithmetic components for
//              signed Q8.8 fixed-point operations. Guarantees zero wraparound
//              under any numerical overflow condition.
//============================================================================

//----------------------------------------------------------------------------
// Submodule: sat_mult_q88
// Multiplies two signed 16-bit Q8.8 operands.
// Extracts bits [23:8] with 9-bit guard-band overflow check.
//----------------------------------------------------------------------------
module sat_mult_q88 (
    input  wire signed [15:0] a,
    input  wire signed [15:0] b,
    output wire signed [15:0] result,
    output wire               overflow
);

    wire signed [31:0] prod;
    assign prod = a * b;

    wire pos_overflow;
    wire neg_overflow;

    // Check upper guard bits [31:23]
    assign pos_overflow = (prod[31] == 1'b0) && (prod[31:23] != 9'b0_0000_0000);
    assign neg_overflow = (prod[31] == 1'b1) && (prod[31:23] != 9'b1_1111_1111);

    assign overflow = pos_overflow | neg_overflow;

    assign result = pos_overflow ? 16'sh7FFF :
                    neg_overflow ? 16'sh8000 :
                    prod[23:8];

endmodule

//----------------------------------------------------------------------------
// Submodule: sat_add_sub_q88
// Performs saturating signed 16-bit Q8.8 addition or subtraction.
// sub_mode = 0: a + b
// sub_mode = 1: a - b
//----------------------------------------------------------------------------
module sat_add_sub_q88 (
    input  wire signed [15:0] a,
    input  wire signed [15:0] b,
    input  wire               sub_mode,
    output wire signed [15:0] result,
    output wire               overflow
);

    wire signed [16:0] wide_res;
    assign wide_res = sub_mode ? ({a[15], a} - {b[15], b}) :
                                 ({a[15], a} + {b[15], b});

    wire pos_overflow;
    wire neg_overflow;

    assign pos_overflow = (wide_res > 17'sh07FFF);
    assign neg_overflow = (wide_res < -17'sh08000);
    assign overflow     = pos_overflow | neg_overflow;

    assign result = pos_overflow ? 16'sh7FFF :
                    neg_overflow ? 16'sh8000 :
                    wide_res[15:0];

endmodule

//----------------------------------------------------------------------------
// Submodule: clamp_bounds_q88
// Clamps signed 16-bit Q8.8 value to [min_limit, max_limit].
//----------------------------------------------------------------------------
module clamp_bounds_q88 (
    input  wire signed [15:0] val,
    input  wire signed [15:0] min_limit,
    input  wire signed [15:0] max_limit,
    output wire signed [15:0] clamped_val,
    output wire               sat_high,
    output wire               sat_low
);

    assign sat_high = (val > max_limit);
    assign sat_low  = (val < min_limit);

    assign clamped_val = sat_high ? max_limit :
                         sat_low  ? min_limit :
                         val;

endmodule
