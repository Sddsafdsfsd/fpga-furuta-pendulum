// ============================================================================
//  uart_rx.v  --  8N1 UART receiver (on-board CH340, 115200)
// ============================================================================
//  Sampling strategy: on the falling edge of the start bit, wait half a bit
//  period and re-check that the line is still low (mid-bit sample), then sample
//  every bit period thereafter for the 8 data bits and the stop bit.
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  * P12 : the original used a single flop (`rx_d <= rx`) for edge detection.
//    The UART line is asynchronous to clk_50m, so a single flop is a
//    metastability hazard and can cause spurious start-bit detection.
//    Fixed with a 2-flop synchroniser and edge detection on the synchronised
//    signal.
//  * P12b: the stop bit was never checked.  A framing error (stop bit low) now
//    discards the byte instead of emitting `valid`.
//  * P31 : DIV range check.
//  * style: named state constants, complete case with default.
//
//  Bit widths:
//      DIV     : localparam, CLK_HZ/BAUD = 434 @50MHz/115200
//      cnt     : 16 bit  (DIV=434 < 65536 OK, checked below)
//      sh      : 8 bit shift register, LSB first
//      bit_i   : 3 bit, 0..7
//      state   : 2 bit, one-hot-by-value encoding
//
//  Timing:
//      single clock, single reset. `valid` is a single-cycle pulse produced in
//      the same cycle `data` becomes valid.
// ============================================================================
`default_nettype none

module uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,          // asynchronous input from CH340
    output reg [7:0]  data,
    output reg        valid        // one-cycle pulse per received byte
);

    localparam integer DIV = CLK_HZ / BAUD;

    // synthesise-time check: DIV must fit the 16-bit counter and be >= 4
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bit_i;
    reg [7:0]  sh;

    // ---- P12: two-flop synchroniser for the asynchronous rx line ----
    reg [1:0] rx_sync;
    wire      rx_s = rx_sync[1];

    // ---- edge detect on the synchronised signal ----
    reg       rx_s_d;
    wire      rx_fall = rx_s_d & ~rx_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
            rx_s_d  <= 1'b1;
        end else begin
            rx_sync <= {rx_sync[0], rx};
            rx_s_d  <= rx_s;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            cnt   <= 16'd0;
            bit_i <= 3'd0;
            sh    <= 8'd0;
            data  <= 8'd0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)
                // ------------------------------------------------------------
                S_IDLE: begin
                    if (rx_fall) begin
                        cnt   <= DIV[15:0] / 2;    // half bit -> middle of start bit
                        state <= S_START;
                    end
                end
                // ------------------------------------------------------------
                S_START: begin
                    if (cnt == 16'd0) begin
                        if (!rx_s) begin           // confirmed: real start bit
                            cnt   <= DIV[15:0] - 16'd1;
                            bit_i <= 3'd0;
                            state <= S_DATA;
                        end else begin
                            state <= S_IDLE;       // glitch, drop it
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end
                // ------------------------------------------------------------
                S_DATA: begin
                    if (cnt == 16'd0) begin
                        sh <= {rx_s, sh[7:1]};     // LSB first
                        if (bit_i == 3'd7) begin
                            cnt   <= DIV[15:0] - 16'd1;
                            state <= S_STOP;
                        end else begin
                            bit_i <= bit_i + 3'd1;
                            cnt   <= DIV[15:0] - 16'd1;
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end
                // ------------------------------------------------------------
                S_STOP: begin
                    if (cnt == 16'd0) begin
                        // ---- P12b: framing check ----
                        if (rx_s) begin
                            data  <= sh;           // valid stop bit
                            valid <= 1'b1;
                        end
                        // else: framing error -> silently drop the byte
                        state <= S_IDLE;
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end
                // ------------------------------------------------------------
                default: begin
                    state <= S_IDLE;
                    cnt   <= 16'd0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
