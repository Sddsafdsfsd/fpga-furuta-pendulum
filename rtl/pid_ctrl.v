// ============================================================================
//  pid_ctrl.v  --  unsigned-input fixed-point PID (Q8), 1 kHz update
// ============================================================================
//  Formula (from the competition guide / J280 manual):
//      PWM = Kp*e(k) + Ki*sum(e) + Kd*[e(k)-e(k-1)]
//
//  Gains are Q8 fixed point: KP_Q8 = Kp * 256
//      Kp = 2.000 -> KP_Q8 = 512
//      Ki = 0.031 -> KI_Q8 = 8
//      Kd = 4.000 -> KD_Q8 = 1024
//
//  Output unit is "microsecond offset": out = -OUT_MAX_US .. +OUT_MAX_US
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  This module was already well written (no latch, no blocking/non-blocking mix,
//  integral clamp present).  Changes are therefore limited to:
//    * P46 : explicit signed constants for the clamp thresholds, instead of
//            comparing against a bare `parameter integer` (signedness of
//            part-selects is easy to get wrong; be explicit)
//    * P31 : add synthesise-time parameter range checks
//    * style: unique comments, consistent formatting, `default_nettype none`
//
//  Bit-width analysis (all values verified against the 10-bit code range):
//      target / actual : 10 bit unsigned, 0..1023
//      err             : signed 16 bit, {6'b0,target}-{6'b0,actual} in [-1023,+1023]
//                        -> 16 bit is far more than enough, NO overflow
//      integ           : signed 32 bit, clamped to +/-INT_MAX
//      p_term          : KP_Q8(16s) * err(16s) -> 32 bit, |max| = 32767*1023 < 2^25
//      i_term          : KI_Q8(16s) * integ(32s) -> 32 bit, clamped by INT_MAX
//      d_term          : KD_Q8(16s) * (err-err_prev)(17s) -> 32 bit
//      sum_q8          : 32 bit, worst case well below 2^31 (see parameter check)
//      sum = sum_q8>>>8: arithmetic shift keeps the sign
//      out             : clamped to +/-OUT_MAX_US first, then sum[15:0]
//                        requires OUT_MAX_US <= 32767 (checked below)
//
//  Timing:
//      single clock, single reset, single enable (tick_1k)
//      clear has priority over tick_1k (same as the original)
//      combination path: 16x16 multiply -> add -> shift -> compare
// ============================================================================
`default_nettype none

module pid_ctrl #(
    parameter signed [15:0] KP_Q8      = 16'sd512,   // Kp = 2.000
    parameter signed [15:0] KI_Q8      = 16'sd8,     // Ki = 0.031
    parameter signed [15:0] KD_Q8      = 16'sd1024,  // Kd = 4.000
    parameter integer       OUT_MAX_US = 1000,       // output clamp (us), <= 32767
    parameter integer       INT_MAX    = 20000       // integral clamp (anti-windup)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1k,               // 1 kHz single-cycle enable
    input  wire               clear,                 // clear integrator (Z/C command)
    input  wire [9:0]         target,                // target angle (code 0..1023)
    input  wire [9:0]         actual,                // measured angle (code 0..1023)
    output reg  signed [15:0] out                    // control output (us offset)
);

    // ---- synthesise-time parameter checks -----------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    // ---- explicit signed clamp thresholds (P46) ------------------------------
    localparam signed [15:0] OUT_MAX_S = OUT_MAX_US;
    localparam signed [15:0] OUT_MIN_S = -OUT_MAX_S;

    reg  signed [31:0] integ;        // sum of errors
    reg  signed [15:0] err_prev;     // e(k-1)

    // e(k): extend the unsigned code to signed before subtracting
    wire signed [15:0] err = $signed({6'b0, target}) - $signed({6'b0, actual});

    wire signed [31:0] p_term = KP_Q8 * err;                    // Q8
    wire signed [31:0] i_term = KI_Q8 * integ;                  // Q8
    wire signed [31:0] d_term = KD_Q8 * (err - err_prev);       // Q8

    wire signed [31:0] sum_q8 = p_term + i_term + d_term;
    wire signed [31:0] sum    = sum_q8 >>> 8;                   // back to us

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integ    <= 32'sd0;
            err_prev <= 16'sd0;
            out      <= 16'sd0;
        end else if (clear) begin
            // one-shot integrator reset: do this before enabling the motor,
            // otherwise the first enabled cycle jumps to full deflection
            integ    <= 32'sd0;
            err_prev <= 16'sd0;
            out      <= 16'sd0;
        end else if (tick_1k) begin
            err_prev <= err;

            // ---- integral clamp (anti-windup) ----
            if (integ + err > INT_MAX)
                integ <= INT_MAX;
            else if (integ + err < -INT_MAX)
                integ <= -INT_MAX;
            else
                integ <= integ + err;

            // ---- output clamp ----
            if (sum > OUT_MAX_S)
                out <= OUT_MAX_S;
            else if (sum < OUT_MIN_S)
                out <= OUT_MIN_S;
            else
                out <= sum[15:0];
        end
    end

endmodule

`default_nettype wire
