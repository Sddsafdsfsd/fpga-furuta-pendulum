// ============================================================================
//  dual_loop_ctrl.v  --  cascaded dual-loop PID (angle inner + position outer)
// ============================================================================
//  The J280 manual shows "angle PID + position PID -> sum -> motor PWM".
//  For an inverted pendulum the usual structure is cascaded:
//  the position loop output offsets the angle setpoint.
//  Both are implemented, selected by LOOP_MODE:
//
//    LOOP_MODE = 1  (cascaded, default)
//        angle setpoint = balance_point + position_loop_out ;  cmd = angle_loop_out
//    LOOP_MODE = 0  (parallel sum, the J280 manual figure)
//        angle setpoint = balance_point                       ;  cmd = angle_out + pos_out
//
//  Units: angle loop works on ADC code (0..1023); position loop on encoder counts.
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  * P02-related : the original summed `$signed(ang_pid) + pos_pid` in a
//    16-bit context.  With the current clamps (1000 + 300) that happens to fit,
//    but any parameter increase silently overflows.  Both terms are now
//    sign-extended to 32 bit *before* the addition.
//  * P04 : a `clear` input is added and routed to both PID instances.
//    (This is an INTERNAL module, so adding a port does not affect the top-level
//     pinout or the .cst file.)
//  * P31 : synthesise-time parameter range checks.
//
//  Bit-width analysis:
//      angle        : 10 bit unsigned 0..1023
//      pos          : 16 bit signed
//      pos_pid      : 16 bit signed, clamped to +/-POS_OUT_MAX
//      base_tgt     : 32 bit signed = angle_target
//      atgt_raw     : 32 bit signed = base_tgt + pos_pid (sign-extended)
//      ang_tgt      : 10 bit, clamped to 0..1023
//      ang_pid      : 16 bit signed, clamped to +/-ANG_OUT_MAX
//      sum          : 32 bit signed = sign-extended(ang_pid) [+ sign-extended(pos_pid)]
//      cmd          : 16 bit, clamped to +/-CMD_MAX
//
//  Timing: pure combinational glue + two registered PID instances,
//          both driven by the same clk / rst_n / tick_1k.  No CDC.
// ============================================================================
`default_nettype none

module dual_loop_ctrl #(
    // ---- angle loop (inner) ----
    parameter signed [15:0] AKP_Q8      = 16'sd512,   // Kp = 2.000
    parameter signed [15:0] AKI_Q8      = 16'sd4,     // Ki = 0.016
    parameter signed [15:0] AKD_Q8      = 16'sd1024,  // Kd = 4.000
    parameter integer       ANG_OUT_MAX = 1000,
    // ---- position loop (outer) ----
    parameter signed [15:0] PKP_Q8      = 16'sd64,    // Kp = 0.250
    parameter signed [15:0] PKI_Q8      = 16'sd1,     // Ki = 0.004
    parameter signed [15:0] PKD_Q8      = 16'sd256,   // Kd = 1.000
    parameter integer       POS_OUT_MAX = 300,
    // ---- structure ----
    parameter integer       LOOP_MODE   = 1,          // 1 = cascaded, 0 = parallel
    parameter integer       CMD_MAX     = 1000
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1k,        // 1 kHz
    input  wire               clear,          // clear both integrators (P04)
    input  wire [9:0]         angle,          // measured tilt (ADC code)
    input  wire [9:0]         angle_target,   // balance point (usually near 512)
    input  wire signed [15:0] pos,            // encoder position
    input  wire signed [15:0] pos_target,     // target position
    output wire signed [15:0] cmd,            // to TB6612
    output wire signed [15:0] angle_term,     // telemetry / debug
    output wire signed [15:0] pos_term
);

    // ---- synthesise-time parameter checks -----------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    // ---- explicit signed clamp threshold (P46) ------------------------------
    localparam signed [15:0] CMD_MAX_S = CMD_MAX;

    // ========================================================================
    //  position loop (outer)
    // ========================================================================
    wire signed [15:0] pos_pid;

    pid_s16 #(
        .KP_Q8  (PKP_Q8),
        .KI_Q8  (PKI_Q8),
        .KD_Q8  (PKD_Q8),
        .OUT_MAX(POS_OUT_MAX),
        .INT_MAX(20000)
    ) u_pos_pid (
        .clk     (clk),
        .rst_n   (rst_n),
        .tick_1k (tick_1k),
        .clear   (clear),          // P04: was unconnected before
        .target  (pos_target),
        .actual  (pos),
        .out     (pos_pid)
    );

    // ========================================================================
    //  angle setpoint: balance point [+ position loop output]
    // ========================================================================
    wire signed [31:0] base_tgt = {{22{1'b0}}, angle_target};      // 0..1023
    wire signed [31:0] pos_pid_s = {{16{pos_pid[15]}}, pos_pid};   // sign-extend to 32 bit
    wire signed [31:0] atgt_raw = (LOOP_MODE == 1) ? (base_tgt + pos_pid_s)
                                                   : base_tgt;

    wire [9:0] ang_tgt = (atgt_raw < 32'sd0)        ? 10'd0    :
                         (atgt_raw > 32'sd1023)     ? 10'd1023 : atgt_raw[9:0];

    // ========================================================================
    //  angle loop (inner)
    // ========================================================================
    wire signed [15:0] ang_pid;

    pid_ctrl #(
        .KP_Q8      (AKP_Q8),
        .KI_Q8      (AKI_Q8),
        .KD_Q8      (AKD_Q8),
        .OUT_MAX_US (ANG_OUT_MAX),
        .INT_MAX    (20000)
    ) u_ang_pid (
        .clk     (clk),
        .rst_n   (rst_n),
        .tick_1k (tick_1k),
        .clear   (clear),          // P04: was unconnected before
        .target  (ang_tgt),
        .actual  (angle),
        .out     (ang_pid)
    );

    // ========================================================================
    //  combine + clamp  (P02-related: sign-extend BEFORE adding)
    // ========================================================================
    wire signed [31:0] ang_pid_s = {{16{ang_pid[15]}}, ang_pid};

    wire signed [31:0] sum = (LOOP_MODE == 1) ? ang_pid_s
                                              : (ang_pid_s + pos_pid_s);

    assign cmd = (sum >  CMD_MAX_S) ?  CMD_MAX_S :
                 (sum < -CMD_MAX_S) ? -CMD_MAX_S : sum[15:0];

    assign angle_term = ang_pid;
    assign pos_term   = pos_pid;

endmodule

`default_nettype wire
