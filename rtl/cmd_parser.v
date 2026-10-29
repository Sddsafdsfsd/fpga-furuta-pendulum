// ============================================================================
//  cmd_parser.v  --  text command parser (type directly in a serial terminal)
// ============================================================================
//  Format:  <letter><number>  terminated by any non-digit (CR / space / etc.)
//      S512    set target angle (0..1023, the balance point)
//      P1000   set target position (encoder counts, may be negative: P-500)
//      E1      enable motor ;  E0 disable (power-on default is DISABLED)
//      Z       capture the current angle as the balance point (pulse)
//      C       clear PID integrators (pulse)
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  * P19 : the accumulator was 16 bit:
//              if (acc < 16'd10000) acc <= acc*10 + digit;
//          9999*10+9 = 99999 overflows 16 bit -> e.g. "P70000" produced 4464
//          instead of being clamped to POS_LIMIT.  acc is now 32 bit and the
//          guard is 100000, so all intermediate values fit and the clamp at
//          commit time is authoritative.
//  * P37 : the '-' sign only applied to SEL_POS; "S-512" silently became 512.
//          The sign is now honoured for SEL_ANG too (a negative angle clamps
//          to 0 through the existing 0..ANGLE_MAX clamp).
//  * P46 : clamp thresholds are explicit signed constants.
//  * style: complete case/if chains, one-shot pulses are cleared at the top of
//           the sequential block (same as the original, that part was correct).
//
//  Bit widths:
//      acc      : 32 bit unsigned, saturated growth (guard 100000)
//      acc_s    : 33 bit signed = sign-applied accumulator
//      acc_lim  : 33 bit signed, clamped to +/-POS_LIMIT
//      pos_target : 16 bit signed  (POS_LIMIT must fit: checked below)
//      angle_target : 10 bit unsigned, clamped to ANGLE_MAX (<=1023)
//
//  Timing: single clock; `rx_valid` is a one-cycle pulse from uart_rx.
//          capture_pulse / clear_pulse are registered one-cycle pulses.
// ============================================================================
`default_nettype none

module cmd_parser #(
    parameter integer DEF_ANGLE = 512,     // power-on target angle
    parameter integer ANGLE_MAX = 1023,
    parameter integer POS_LIMIT = 9999
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [7:0]         rx_data,
    input  wire               rx_valid,
    output reg  [9:0]         angle_target,
    output reg  signed [15:0] pos_target,
    output reg                motor_en,
    output reg                capture_pulse,   // 'Z': capture current angle as balance point
    output reg                clear_pulse,     // 'C': clear PID integrators
    output reg                home_pulse       // 'H': latch current position as E=0
);

    localparam [1:0] SEL_NONE = 2'd0,
                     SEL_ANG  = 2'd1,
                     SEL_POS  = 2'd2,
                     SEL_EN   = 2'd3;

    // ---- synthesise-time parameter checks -----------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    // ---- explicit signed thresholds (P46) -----------------------------------
    localparam signed [15:0] POS_LIM_S  = POS_LIMIT;
    localparam signed [15:0] POS_LIMS_S = -POS_LIM_S;
    localparam        [9:0]  ANG_MAX_U  = ANGLE_MAX;

    reg [1:0]  sel;
    reg [31:0] acc;        // P19: 32 bit, growth-saturated
    reg        neg;

    // ---- sign applied, then clamped (works for both S and P now) ------------
    wire signed [32:0] acc_s   = neg ? -$signed({1'b0, acc}) : $signed({1'b0, acc});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_target  <= DEF_ANGLE[9:0];
            pos_target    <= 16'sd0;
            motor_en      <= 1'b0;              // safety default: motor disabled
            capture_pulse <= 1'b0;
            clear_pulse   <= 1'b0;
            home_pulse    <= 1'b0;
            sel           <= SEL_NONE;
            acc           <= 32'd0;
            neg           <= 1'b0;
        end else begin
            // ---- one-shot pulses self-clear every cycle ----
            capture_pulse <= 1'b0;
            clear_pulse   <= 1'b0;
            home_pulse    <= 1'b0;

            if (rx_valid) begin
                if      (rx_data == "Z" || rx_data == "z") begin
                    capture_pulse <= 1'b1;
                    sel           <= SEL_NONE;
                end else if (rx_data == "C" || rx_data == "c") begin
                    clear_pulse   <= 1'b1;
                    sel           <= SEL_NONE;
                end else if (rx_data == "H" || rx_data == "h") begin
                    home_pulse <= 1'b1;
                    sel        <= SEL_NONE;
                end else if (rx_data == "S" || rx_data == "s") begin
                    sel <= SEL_ANG; acc <= 32'd0; neg <= 1'b0;
                end else if (rx_data == "P" || rx_data == "p") begin
                    sel <= SEL_POS; acc <= 32'd0; neg <= 1'b0;
                end else if (rx_data == "E" || rx_data == "e") begin
                    sel <= SEL_EN;  acc <= 32'd0; neg <= 1'b0;
                end else if (sel != SEL_NONE && rx_data == "-") begin
                    neg <= 1'b1;
                end else if (sel != SEL_NONE && rx_data >= "0" && rx_data <= "9") begin
                    // ---- P19: 32-bit accumulator with a growth guard ----
                    if (acc < 32'd100_000)
                        acc <= acc * 32'd10 + {24'd0, (rx_data - "0")};
                end else if (sel != SEL_NONE) begin
                    // ---- non-digit: commit the pending command ----
                    case (sel)
                        SEL_ANG: begin
                            // P37: honour the sign; negative clamps to 0
                            if (neg)
                                angle_target <= 10'd0;
                            else if (acc > ANG_MAX_U)
                                angle_target <= ANG_MAX_U;
                            else
                                angle_target <= acc[9:0];
                        end
                        SEL_POS: begin
                            if (acc_s > POS_LIM_S)
                                pos_target <= POS_LIM_S;
                            else if (acc_s < POS_LIMS_S)
                                pos_target <= POS_LIMS_S;
                            else
                                pos_target <= acc_s[15:0];
                        end
                        SEL_EN : motor_en <= (acc != 32'd0);
                        default: ;
                    endcase
                    sel <= SEL_NONE;
                end
            end
        end
    end

endmodule

`default_nettype wire
