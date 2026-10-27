# Fixed-Point Arithmetic & Numerical Design Specification
**Project:** PS-02 Mode-Switching Hardware PID Controller  
**Document ID:** DOC-PS02-FP-001  
**Author:** Senior Digital Control Architect  

---

## 1. Overview and Engineering Rationale

Digital flight control ASICs require deterministic, real-time arithmetic without floating-point units to minimize silicon area, power consumption, and cycle latency. This design employs a unified **Signed Q8.8 Fixed-Point Format** (16 bits) for signals and gains, paired with **Signed Q16.16 (32 bits)** for products and accumulator states.

This ensures:
1. Exact two's complement arithmetic synthesizable directly to Cadence Genus / GPDK standard cells.
2. Zero silent arithmetic wraparound.
3. High fractional precision ($0.0039$) with sufficient dynamic range ($\pm 128$).
4. Standardized bit-slice extraction and rounding across all controller axes.

---

## 2. Signal Types, Scaling, and Units

| Signal Category | Physical Meaning | Physical Range | Fixed-Point Format | Hex Range | Resolution |
|:---|:---|:---|:---|:---|:---|
| **Attitude Angles** ($\phi, \theta, \psi$) | Roll, Pitch, Yaw Euler Angles | $-90.0^\circ$ to $+90.0^\circ$ | Signed Q8.8 (16-bit) | `0xA600` to `0x5A00` | $0.00390625^\circ$ ($1/256^\circ$) |
| **Angular Rates** ($p, q, r$) | Gyroscope body rates (normalized) | $-100.0^\circ/\text{s}$ to $+100.0^\circ/\text{s}$ | Signed Q8.8 (16-bit) | `0x9C00` to `0x6400` | $0.00390625^\circ/\text{s}$ |
| **Altitude & Vertical Vel** ($z, \dot{z}$) | Altitude error & climb rate | $-50.0\text{ m}$ to $+50.0\text{ m}$ | Signed Q8.8 (16-bit) | `0xCE00` to `0x3200` | $0.00390625\text{ m}$ |
| **PID Gains** ($K_p, K_i, K_d$) | Controller tuning coefficients | $0.0$ to $+16.0$ | Signed Q8.8 (16-bit) | `0x0000` to `0x1000` | $1/256$ |
| **Filter Alpha** ($\alpha$) | Derivative LPF pole coefficient | $0.0$ to $1.0$ | Signed Q8.8 (16-bit) | `0x0000` to `0x0100` | $1/256$ ($1.0 = \text{0x0100}$) |
| **Actuator Outputs** ($u$) | Motor commands / duty cycles | $0.0\%$ to $+100.0\%$ | Signed Q8.8 (16-bit) | `0x0000` to `0x6400` | $1/256\%$ |

---

## 3. Bit Layouts

### 16-Bit Signed Q8.8 Format
```
Bit:   15   14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
     [Sign|   Integer bits (7)        |     Fractional bits (8)       ]
```
- **Sign Bit (Bit 15):** 0 for positive, 1 for negative (two's complement).
- **Integer Value (Bits 14:8):** Range $-128$ to $+127$.
- **Fractional Value (Bits 7:0):** Scaled by $2^{-8} = 1/256$.
- **Maximum Positive:** `16'h7FFF` = $+127.99609375$
- **Minimum Negative:** `16'h8000` = $-128.00000000$

### 32-Bit Signed Intermediate Format (Q16.16)
When two Q8.8 operands are multiplied:
$$\text{Q8.8} \times \text{Q8.8} = \text{Signed Q16.16 (32 bits)}$$
```
Bit:   31 ......... 24 | 23 ......... 8 | 7 ......... 0
     [ Overflow Guard  | Q8.8 Core Term | Sub-fraction  ]
```
To extract the resulting Q8.8 value:
- Truncate / round bits `[7:0]`.
- Output Q8.8 core is located at bits `[23:8]`.
- Check upper bits `[31:23]` (9 guard bits):
  - If result is positive ($Product[31] == 0$), bits `[31:23]` must be `9'b0_0000_0000`. If any bit is 1, **positive overflow occurred** $\rightarrow$ saturate to `16'h7FFF`.
  - If result is negative ($Product[31] == 1$), bits `[31:23]` must be `9'b1_1111_1111`. If any bit is 0, **negative overflow occurred** $\rightarrow$ saturate to `16'h8000`.

---

## 4. Arithmetic Operations & Saturation Functions

### Multiplication with Saturation
In Verilog-2001:
```verilog
function signed [15:0] mult_sat_q88;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [31:0] prod;
    reg pos_overflow, neg_overflow;
    begin
        prod = a * b;
        pos_overflow = (prod[31] == 1'b0) && (prod[31:23] != 9'h000);
        neg_overflow = (prod[31] == 1'b1) && (prod[31:23] != 9'h1FF);
        if (pos_overflow)
            mult_sat_q88 = 16'sh7FFF;
        else if (neg_overflow)
            mult_sat_q88 = 16'sh8000;
        else
            mult_sat_q88 = prod[23:8];
    end
endfunction
```

### Addition / Subtraction with Bounds Saturation
For summing $P + I + D$ or adding to the accumulator:
```verilog
function signed [15:0] add_sat_q88;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [16:0] sum;
    begin
        sum = {a[15], a} + {b[15], b};
        if (sum > 17'sh07FFF)
            add_sat_q88 = 16'sh7FFF;
        else if (sum < -17'sh08000)
            add_sat_q88 = 16'sh8000;
        else
            add_sat_q88 = sum[15:0];
    end
endfunction
```

### Range Clamping (`clamp_range`)
Given arbitrary bounds $[MIN, MAX]$:
```verilog
function signed [15:0] clamp_range;
    input signed [15:0] val;
    input signed [15:0] min_limit;
    input signed [15:0] max_limit;
    begin
        if (val > max_limit)
            clamp_range = max_limit;
        else if (val < min_limit)
            clamp_range = min_limit;
        else
            clamp_range = val;
    end
endfunction
```

---

## 5. Integrator Accumulator Dynamics & Precision

The discrete-time integrator equation is:
$$I[n] = I[n-1] + K_i \cdot e[n] \cdot \Delta t$$

In traditional fixed-step sampled controllers where $K_i$ is tuned for the sampling cadence $\Delta t$:
$$\Delta I[n] = K_i \cdot e[n]$$

To eliminate sub-fractional drift:
- The accumulator `integrator_acc` is stored as **Signed 32-bit Q16.16**.
- The increment $\Delta I = K_i \cdot e[n]$ (computed as a 32-bit product with lower 8 fractional bits retained) is added directly into the 32-bit accumulator.
- The effective Q8.8 integral term output is `integrator_acc[23:8]`, clamped strictly to the per-mode configured $[I_{min}, I_{max}]$.
- This architecture maintains zero truncation bias over hours of flight time.
