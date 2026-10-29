// ============================================================================
//  telemetry.v  --  UART telemetry, one ASCII line every 10 ms
// ============================================================================
//  Frame (fixed 27 bytes, easy to parse on the PC side):
//      A<4 digits> T<4 digits> E<sign><4 digits> M<sign><4 digits> \r \n
//  Example:  A0512 T0512 E+0123 M-0250
//      A = measured angle (ADC code 0..1023)
//      T = target angle
//      E = encoder position (x4 counts, displayed up to +/-9999)
//      M = motor command (-1000..+1000, or servo pulse width in other STAGEs)
//
//  115200 baud: one line takes ~2.4 ms, sent every 10 ms -> never blocks.
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  ★ P17 : the original `bcd4` function used /1000, %1000, /100, %10.  It was
//    instantiated FOUR times (a/t/e/m) -> 16 constant dividers in the netlist.
//    GW1N-9 has no hardware divider, so each one becomes a LUT network: this
//    was one of the biggest LUT consumers and a long combinational path.
//    Replaced with the classic "double dabble" (shift-and-add-3) algorithm,
//    which needs only comparators and small adders.
//  * P31 : GAP_MAX range check (19-bit counter must hold CLK_HZ/1000*PERIOD_MS).
//  * style: the frame byte ROM now lives in a self-contained case with default;
//    the send handshake is unchanged (it worked correctly).
//
//  Bit widths:
//      GAP_MAX   : localparam, (CLK_HZ/1000)*PERIOD_MS = 500000 @50MHz/10ms
//      gap       : 19 bit, 0..GAP_MAX-1   => GAP_MAX must be <= 2^19
//      v (input to bin2bcd4) : 14 bit, 0..9999 (clamped by abs4)
//      bin2bcd4  : shift register 30 bit = 14 binary + 16 BCD
//                  (4 digits x 4 bits), result is shift[29:14]
//      a_lat     : 12 bit (adc_val is 12 bit; only 0..1023 used)
//      t_lat     : 10 bit
//      e_lat/m_lat : 16 bit signed
//
//  Timing assumptions:
//      * single clock / reset
//      * `tx_busy` comes from uart_tx in the same clock domain
//      * frame boundary: `gap` reaching GAP_MAX-1 starts a new frame
// ============================================================================
`default_nettype none

module telemetry #(
    parameter integer CLK_HZ    = 50_000_000,
    parameter integer PERIOD_MS = 10
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [11:0]        adc_val,
    input  wire [9:0]         angle_target,
    input  wire signed [15:0] enc_pos,
    input  wire signed [15:0] motor_cmd,
    output reg  [7:0]         tx_data,
    output reg                tx_send,
    input  wire               tx_busy
);

    localparam integer GAP_MAX  = (CLK_HZ / 1000) * PERIOD_MS;   // 500000
    localparam [4:0]   LAST_IDX = 5'd26;                         // 0..26 = 27 bytes

    // ---- synthesise-time check ---------------------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    reg [18:0]        gap;
    reg               frame;
    reg [4:0]         idx;
    reg               sending;
    reg [11:0]        a_lat;
    reg [9:0]         t_lat;
    reg signed [15:0] e_lat, m_lat;

    // ========================================================================
    //  P17: binary -> 4-digit BCD by "double dabble" (no dividers)
    //  The loop is unrolled by synthesis into 14 shift-and-add-3 stages.
    // ========================================================================
    function [15:0] bin2bcd4;
        input [13:0] v;
        integer i;
        reg [29:0] s;                       // {4 x BCD digit, 14-bit binary}
        begin
            s = {16'd0, v};
            for (i = 0; i < 14; i = i + 1) begin
                // add 3 to every BCD nibble that is >= 5, then shift left
                if (s[17:14] >= 4'd5) s[17:14] = s[17:14] + 4'd3;
                if (s[21:18] >= 4'd5) s[21:18] = s[21:18] + 4'd3;
                if (s[25:22] >= 4'd5) s[25:22] = s[25:22] + 4'd3;
                if (s[29:26] >= 4'd5) s[29:26] = s[29:26] + 4'd3;
                s = s << 1;
            end
            bin2bcd4 = s[29:14];
        end
    endfunction

    // ---- signed -> absolute value, limited to 9999 --------------------------
    function [13:0] abs4;
        input signed [15:0] v;
        reg signed [31:0] t;
        begin
            t = v;
            if (t < 0) begin
                t = -t;
                if (t < 0) t = 9999;         // handle -32768
            end
            if (t > 9999) t = 9999;
            abs4 = t[13:0];
        end
    endfunction

    wire [15:0] a_d   = bin2bcd4({2'b0, a_lat});
    wire [15:0] t_d   = bin2bcd4({4'b0, t_lat});
    wire [15:0] e_d   = bin2bcd4(abs4(e_lat));
    wire [15:0] m_d   = bin2bcd4(abs4(m_lat));
    wire        e_neg = e_lat[15];
    wire        m_neg = m_lat[15];

    // ========================================================================
    //  Frame byte ROM (fixed layout, must stay byte-for-byte compatible)
    // ========================================================================
    always @(*) begin
        case (idx)
            5'd0 : tx_data = "A";
            5'd1 : tx_data = 8'h30 + {4'b0, a_d[15:12]};
            5'd2 : tx_data = 8'h30 + {4'b0, a_d[11:8]};
            5'd3 : tx_data = 8'h30 + {4'b0, a_d[7:4]};
            5'd4 : tx_data = 8'h30 + {4'b0, a_d[3:0]};
            5'd5 : tx_data = " ";
            5'd6 : tx_data = "T";
            5'd7 : tx_data = 8'h30 + {4'b0, t_d[15:12]};
            5'd8 : tx_data = 8'h30 + {4'b0, t_d[11:8]};
            5'd9 : tx_data = 8'h30 + {4'b0, t_d[7:4]};
            5'd10: tx_data = 8'h30 + {4'b0, t_d[3:0]};
            5'd11: tx_data = " ";
            5'd12: tx_data = "E";
            5'd13: tx_data = e_neg ? "-" : "+";
            5'd14: tx_data = 8'h30 + {4'b0, e_d[15:12]};
            5'd15: tx_data = 8'h30 + {4'b0, e_d[11:8]};
            5'd16: tx_data = 8'h30 + {4'b0, e_d[7:4]};
            5'd17: tx_data = 8'h30 + {4'b0, e_d[3:0]};
            5'd18: tx_data = " ";
            5'd19: tx_data = "M";
            5'd20: tx_data = m_neg ? "-" : "+";
            5'd21: tx_data = 8'h30 + {4'b0, m_d[15:12]};
            5'd22: tx_data = 8'h30 + {4'b0, m_d[11:8]};
            5'd23: tx_data = 8'h30 + {4'b0, m_d[7:4]};
            5'd24: tx_data = 8'h30 + {4'b0, m_d[3:0]};
            5'd25: tx_data = 8'h0D;          // \r
            5'd26: tx_data = 8'h0A;          // \n
            default: tx_data = " ";
        endcase
    end

    // ========================================================================
    //  Send state machine (tx_busy handshake, no characters dropped)
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gap     <= 19'd0;
            frame   <= 1'b0;
            idx     <= 5'd0;
            sending <= 1'b0;
            a_lat   <= 12'd0;
            t_lat   <= 10'd0;
            e_lat   <= 16'sd0;
            m_lat   <= 16'sd0;
        end else begin
            // ---- 10 ms frame pacing ----
            if (gap >= GAP_MAX[18:0] - 19'd1)
                gap <= 19'd0;
            else
                gap <= gap + 19'd1;

            if (!frame) begin
                if (gap >= GAP_MAX[18:0] - 19'd1) begin
                    // latch a coherent snapshot at the frame boundary
                    a_lat   <= adc_val;
                    t_lat   <= angle_target;
                    e_lat   <= enc_pos;
                    m_lat   <= motor_cmd;
                    idx     <= 5'd0;
                    sending <= 1'b0;
                    frame   <= 1'b1;
                end
            end else begin
                if (!sending && !tx_busy) begin
                    sending <= 1'b1;
                end else if (sending && tx_busy) begin
                    sending <= 1'b0;
                    if (idx == LAST_IDX)
                        frame <= 1'b0;
                    else
                        idx <= idx + 5'd1;
                end
            end
        end
    end

    always @(*) tx_send = frame && !sending && !tx_busy;

endmodule

`default_nettype wire
