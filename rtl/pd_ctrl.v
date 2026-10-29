// ============================================================================
//  pd_ctrl.v  --  transparent 4-term integer PD for the Furuta pendulum
// ============================================================================
//  Replaces dual_loop_ctrl.v.  Every term is visible and has a physical unit,
//  so a wrong sign or scale can be found by reading the code instead of
//  guessing at a black box:
//
//      a_err  = wrap12(ang12 - tgt12)          pendulum angle error, counts
//      a_rate = wrap12(ang12 - ang12_d)        pendulum rate, counts/rate_win
//      e_pos  = enc_pos                        arm position, encoder counts
//      e_rate = enc_pos - enc_pos_d            arm rate, counts/rate_win
//
//      acc  = KP_A*a_err + KD_A*a_rate + KP_E*e_pos + KD_E*e_rate   (Q8)
//      cmd  = acc >>> 8
//      cmd  = clamp(cmd, -CMD_MAX, +CMD_MAX)
//      cmd  = soft_limit(cmd, e_pos)           attenuate + actively reverse
//      cmd  = cmd + dead_ff                    dead-band feed-forward
//      cmd  = clamp(cmd, -CMD_MAX, +CMD_MAX)
//
//  ---------------------------------------------------------------------------
//  WHY EACH PIECE EXISTS  (all four were diagnosed from the real logs)
//  ---------------------------------------------------------------------------
//  * wrap12 : the AS5600 angle is a 0..4095 CIRCULAR quantity.  dual_loop_ctrl
//    subtracted it as a plain unsigned number, so whenever the pendulum passed
//    the 0/4095 seam the error jumped by a whole turn and the output slammed to
//    the opposite rail.  That is one of the reasons the old controller's output
//    looked unrelated to the error.
//  * 12-bit  : top.v used to truncate the angle to 10 bits before the
//    controller saw it.  One 10-bit count per 10 ms is 35 deg/s of rate
//    quantisation, which a D term turns into hundreds of counts of duty of
//    pure noise.  Using the full 12 bits divides that by four.
//  * no integral term : pid_ctrl's integrator (AKI_Q8=4, INT_MAX=20000) wound
//    up in ~0.2 s and then contributed a fixed 312 of the 400 available
//    counts, i.e. 78% of the output had nothing to do with the current error.
//    This module has no integrator at all.
//  * dead_ff : measured P100 does not turn, P150 does.  A balancing pendulum
//    needs far less than 150 counts of torque, so WITHOUT a feed-forward the
//    motor simply never receives the small corrections and the arm drifts into
//    the travel limit every single time.  Simulation: with dead_ff = 0, 0 of
//    320 gain sets converge; with dead_ff >= 140, 19 of 320 do.
//  * soft limit with active reverse : the old pos_lock latched the output to
//    zero, which removes exactly the authority needed to bring the arm back.
//    This version attenuates the PD output AND adds an opposite bias that
//    grows to `SOFT_REV` at |e_pos| = SOFT_HI, so the arm is driven home.
//
//  ---------------------------------------------------------------------------
//  BIT WIDTHS
//  ---------------------------------------------------------------------------
//      ang12, tgt12      : 12 bit unsigned, 0..4095
//      wrap12            : 13 bit signed, -2048..+2047
//      enc_pos           : 16 bit signed
//      e_rate            : 17 bit signed
//      gain (Q8)         : 18 bit signed,  range +-512.0 in Q8 terms
//      a_err*a_rate term : 18b * 13b = 31 bit
//      acc               : 32 bit signed, then >>> 8
//      cmd               : 16 bit signed, clamped to +-CMD_MAX
//      atten / bias      : 11 bit unsigned, 1024 = full
//
//  Timing: one 50 MHz clock, all state updates on `tick` (1 kHz single-cycle
//  enable) except the ring-buffer write, which is disabled while !enable so
//  that enabling the motor after a pause never produces a bogus rate.
// ============================================================================
`default_nettype none

module pd_ctrl #(
    parameter integer CMD_MAX   = 400,
    parameter integer SOFT_LO   = 150,
    parameter integer SOFT_REV  = 200   // reverse drive applied at |e|>=SOFT_HI
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,        // 1 kHz single-cycle enable
    input  wire               enable,      // motor enabled ('E1' and not locked)

    // ---- sensors ----------------------------------------------------------
    input  wire [11:0]        ang12,       // AS5600 raw 12-bit angle
    input  wire [11:0]        tgt12,       // balance point, 12-bit
    input  wire signed [15:0] enc_pos,     // arm, quadrature counts

    // ---- runtime-tunable controller words --------------------------------
    input  wire signed [17:0] kp_a,        // Q8, duty per count of a_err
    input  wire signed [17:0] kd_a,        // Q8, duty per count of a_rate
    input  wire signed [17:0] kp_e,        // Q8, duty per count of e_pos
    input  wire signed [17:0] kd_e,        // Q8, duty per count of e_rate
    input  wire signed [10:0] dead_ff,     // dead-band feed-forward, duty counts
    input  wire [1:0]         rate_sel,    // 0=10 ms 1=16 ms 2=32 ms

    // ---- output ----------------------------------------------------------
    output reg  signed [15:0] cmd,
    output wire signed [15:0] cmd_raw,     // before soft limit (diagnostics)

    // ---- diagnostics (for the STAGE=37 telemetry frame) ------------------
    output reg  signed [12:0] dbg_a_err,
    output reg  signed [12:0] dbg_a_rate,
    output reg  signed [16:0] dbg_e_rate,
    output reg  signed [15:0] dbg_acc
);

    // ========================================================================
    //  circular (12 bit) difference -> 13 bit signed, shortest path
    // ========================================================================
    function signed [12:0] wrap12;
        input [11:0] a;
        input [11:0] b;
        reg   [11:0] d;
        begin
            d = a - b;                       // mod 4096
            wrap12 = (d >= 12'd2048) ? ($signed({1'b0, d}) - 13'sd4096)
                                     :  $signed({1'b0, d});
        end
    endfunction

    // ========================================================================
    //  rate delay line: 32-entry ring, tap selected by rate_sel
    //    the tap is read BEFORE the current sample is written, so a delay of
    //    32 ticks reads the slot that is about to be overwritten
    // ========================================================================
    reg [11:0]        a_hist [0:31];
    reg signed [15:0] e_hist [0:31];
    reg [4:0]         wptr;

    // a delay of 32 is the same slot as wptr: the current sample has not been
    // written yet, so a_hist[wptr] still holds the value from 32 ticks ago
    wire [4:0] tap = (rate_sel == 2'd0) ? (wptr - 5'd10) :
                     (rate_sel == 2'd1) ? (wptr - 5'd16) :
                                          wptr;

    wire [11:0]        a_tap = a_hist[tap];
    wire signed [15:0] e_tap = e_hist[tap];

    // ========================================================================
    //  the four terms
    // ========================================================================
    wire signed [12:0] a_err_w  = wrap12(ang12, tgt12);
    wire signed [12:0] a_rate_w = wrap12(ang12, a_tap);

    // e_rate = enc_pos - enc_pos_d, widened to 17 bit so it cannot overflow
    wire signed [16:0] e_rate_w = {enc_pos[15], enc_pos}
                                - {e_tap[15],      e_tap};

    // ========================================================================
    //  Q8 accumulation, 32 bit
    // ========================================================================
    wire signed [31:0] t1 = kp_a * a_err_w;
    wire signed [31:0] t2 = kd_a * a_rate_w;
    wire signed [31:0] t3 = kp_e * enc_pos;
    wire signed [31:0] t4 = kd_e * e_rate_w;

    wire signed [31:0] acc  = t1 + t2 + t3 + t4;
    wire signed [31:0] scmd = acc >>> 8;

    // ========================================================================
    //  clamp
    // ========================================================================
    localparam signed [31:0] CMAX = CMD_MAX;

    wire signed [15:0] cmd_c = (scmd >  CMAX) ?  CMAX[15:0] :
                               (scmd < -CMAX) ? -CMAX[15:0] : scmd[15:0];

    assign cmd_raw = cmd_c;

    // ========================================================================
    //  ★ v2.6（2026-10-21）—— 修致命原理错误：软限位把摆杆控制关掉了
    //
    //   旧实现：cmd = cmd_pd * atten + bias * (1024 - atten)
    //   当 |E| >= 214 时 atten = 0、bias = ∓200 ⇒ 输出恒 = ∓200，
    //   再叠上死区前馈 ±200 ⇒ 恒定 ∓400（满反向），而【摆杆项完全归零】。
    //
    //   实测后果（bal_v24.csv，11.8 s 那次平衡的死亡时刻）：
    //     t=19.76  A=741(上升中)  E=214  M=-400
    //     t=19.80  A=899(上升中)  E=219  M=-400
    //     t=19.84  A=980(上升中)  E=201  M=-400   → 随后 A=1023 倒地
    //   臂一到限位，闭环就被换成开环，而且输出方向正好相反。
    //
    //   修法：软限位**只做反向偏置，绝不衰减摆杆项**。
    //   宁可让臂顶在机械限位上短时堵转，也不能在摆杆需要纠正时撒手。
    // ========================================================================
    wire [15:0] ae = enc_pos[15] ? (~enc_pos + 16'd1) : enc_pos;

    localparam [23:0]        SOFT_LO_U  = SOFT_LO;
    localparam signed [15:0] SOFT_REV_S = SOFT_REV;
    localparam [23:0]        SOFT_REV_U = SOFT_REV;

    // |E| 每超过 SOFT_LO 一个 count，偏置增加 16 duty，饱和在 SOFT_REV
    wire [23:0] bias_raw = ({8'd0, ae} <= SOFT_LO_U)
                         ? 24'd0
                         : (({8'd0, ae} - SOFT_LO_U) << 4);
    wire [23:0] bias_sat = (bias_raw > SOFT_REV_U) ? SOFT_REV_U : bias_raw;

    wire signed [15:0] bias = enc_pos[15] ? $signed({5'd0, bias_sat[10:0]})
                                          : -$signed({5'd0, bias_sat[10:0]});

    // ★ 摆杆项 cmd_c 原样保留，只叠加反向偏置（原来这里是 cmd_c*atten + ...）
    wire signed [31:0] cmd_soft = cmd_c + bias;

    // ========================================================================
    //  dead-band feed-forward + final clamp
    // ========================================================================
    wire signed [31:0] cmd_ff =
        (cmd_soft > 0)  ? (cmd_soft + dead_ff) :
        (cmd_soft < 0)  ? (cmd_soft - dead_ff) : 32'sd0;

    wire signed [15:0] cmd_f = (cmd_ff >  CMAX) ?  CMAX[15:0] :
                               (cmd_ff < -CMAX) ? -CMAX[15:0] : cmd_ff[15:0];

    // ========================================================================
    //  registers
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr       <= 5'd0;
            cmd        <= 16'sd0;
            dbg_a_err  <= 13'sd0;
            dbg_a_rate <= 13'sd0;
            dbg_e_rate <= 17'sd0;
            dbg_acc    <= 16'sd0;
        end else if (tick) begin
            // The ring is written EVERY tick, enabled or not.  If it were
            // written only while enabled, the first tick after 'E1' would read
            // zeros and the D term would see a_rate = the whole angle, slamming
            // the output into the clamp: the classic first-tick burst that
            // kicks the arm before the loop has even closed.
            a_hist[wptr] <= ang12;
            e_hist[wptr] <= enc_pos;
            wptr         <= wptr + 5'd1;

            dbg_a_err  <= a_err_w;
            dbg_a_rate <= a_rate_w;
            dbg_e_rate <= e_rate_w;
            dbg_acc    <= acc[31:16];

            cmd <= enable ? cmd_f : 16'sd0;
        end
    end

endmodule

`default_nettype wire
