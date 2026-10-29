// ============================================================================
//  quad_decoder.v  --  quadrature encoder: x4 decode + direction + speed
// ============================================================================
//  Principle (J280 manual 3.3): forward rotation -> A leads B by 90 deg.
//  The 2-bit state {A,B} cycles 00->01->11->10, every transition counts 1 => x4.
//      counts per output rev = 4 * PPR * gear_ratio
//      JGB37-520 (11 PPR, 1:30) -> 4*11*30 = 1320 counts/rev
//
//  If the count decreases on forward rotation, set INVERT=1 (or swap A/B wires).
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  * P35 : `speed <= pos_r - pos_prev` was a 16-bit subtraction.  When the
//    position crosses the +/-32768 wrap boundary, the difference wraps too and
//    the sign/magnitude becomes wrong (dir follows 1 ms later).  Fixed by
//    computing the difference in 17 bits and SATURATING into 16 bits.
//  * P31 : FILTER_N/CNT_BITS range checks (an 8-bit compare breaks for N>=256).
//  * style: keep two-stage synchroniser + 64-tick digital filter (this module
//    was already the only one doing asynchronous inputs correctly).
//
//  Bit widths:
//      a_sync/b_sync : 2-bit shift register (2 flip-flop synchroniser)
//      a_cnt/b_cnt   : 8 bit, 0..FILTER_N-1  => FILTER_N must be <= 256
//      prev/curr     : 2 bit
//      delta/step    : signed 2 bit, -1/0/+1
//      pos_r/pos_prev: signed CNT_BITS (default 16) free-wrapping by design
//      spd_diff      : signed CNT_BITS+1 (17) = sign-extended difference
//      speed         : signed 16, saturated copy of spd_diff
//      dir           : 1 = forward (speed > 0)
//
//  Timing:
//      single clock; pos/speed only update on tick_1k (1 kHz).
//      pos_r updates on every clock where a valid quadrature edge is decoded.
// ============================================================================
`default_nettype none

module quad_decoder #(
    parameter integer CNT_BITS = 16,     // position counter width
    parameter integer FILTER_N = 64,     // stable cycles before accepting a level
    parameter integer INVERT   = 0       // 1 = invert counting direction
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       tick_1k,      // 1 kHz: speed window
    input  wire                       a_in,         // asynchronous!
    input  wire                       b_in,         // asynchronous!
    output reg  signed [CNT_BITS-1:0] pos,          // position (x4 counts)
    output reg  signed [15:0]         speed,        // counts per 1 ms
    output wire                       dir           // 1 = forward
);

    // ---- synthesise-time parameter checks -----------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    // ========================================================================
    //  Stage 1: two-flip-flop synchroniser (P12-class fix for A/B inputs)
    // ========================================================================
    reg [1:0] a_sync, b_sync;
    reg       a_f, b_f;              // filtered levels
    reg [7:0] a_cnt, b_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 2'b11;
            b_sync <= 2'b11;
            a_f    <= 1'b1;
            b_f    <= 1'b1;
            a_cnt  <= 8'd0;
            b_cnt  <= 8'd0;
        end else begin
            a_sync <= {a_sync[0], a_in};
            b_sync <= {b_sync[0], b_in};

            // ---- A channel digital filter: level must hold FILTER_N cycles ----
            if (a_sync[1] == a_f) begin
                a_cnt <= 8'd0;
            end else if (a_cnt >= FILTER_N[7:0] - 8'd1) begin
                a_f   <= a_sync[1];
                a_cnt <= 8'd0;
            end else begin
                a_cnt <= a_cnt + 8'd1;
            end

            // ---- B channel digital filter ----
            if (b_sync[1] == b_f) begin
                b_cnt <= 8'd0;
            end else if (b_cnt >= FILTER_N[7:0] - 8'd1) begin
                b_f   <= b_sync[1];
                b_cnt <= 8'd0;
            end else begin
                b_cnt <= b_cnt + 8'd1;
            end
        end
    end

    // ========================================================================
    //  Stage 2: x4 quadrature decode (combinational, complete default branch)
    // ========================================================================
    reg  [1:0]        prev;
    wire [1:0]        curr  = {a_f, b_f};
    reg  signed [1:0] delta;

    always @(*) begin
        case ({prev, curr})
            // forward sequence  00 -> 01 -> 11 -> 10 -> 00
            4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: delta = 2'sd1;
            // reverse sequence
            4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: delta = -2'sd1;
            // illegal transition (glitch) -> no count
            default:                                 delta = 2'sd0;
        endcase
    end

    wire signed [1:0] step = (INVERT != 0) ? -delta : delta;

    // ========================================================================
    //  Stage 3: position accumulation + 1 kHz speed (P35: saturating)
    // ========================================================================
    reg signed [CNT_BITS-1:0] pos_r;
    reg signed [CNT_BITS-1:0] pos_prev;

    // 17-bit (CNT_BITS+1) sign-extended difference: no wrap inside the window
    wire signed [CNT_BITS:0] spd_diff = {pos_r[CNT_BITS-1], pos_r}
                                      - {pos_prev[CNT_BITS-1], pos_prev};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev     <= 2'b11;
            pos_r    <= {CNT_BITS{1'b0}};
            pos_prev <= {CNT_BITS{1'b0}};
            pos      <= {CNT_BITS{1'b0}};
            speed    <= 16'sd0;
        end else begin
            prev <= curr;

            if (step != 2'sd0)
                pos_r <= pos_r + step;

            if (tick_1k) begin
                // ---- P35: saturate instead of truncating ----
                if (spd_diff > 32'sd32767)
                    speed <= 16'sd32767;
                else if (spd_diff < -32'sd32768)
                    speed <= -16'sd32768;
                else
                    speed <= spd_diff[15:0];

                pos_prev <= pos_r;
                pos      <= pos_r;
            end
        end
    end

    assign dir = (speed > 16'sd0);

endmodule

`default_nettype wire
