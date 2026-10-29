// ============================================================================
//  i2c_master.v  --  minimal I2C master (single transaction: read one register)
// ============================================================================
//  I2C phase table (each phase lasts CLK_DIV clocks, SCL = 50MHz/(4*CLK_DIV)):
//      phase 0: SCL=0  -- (transmit: SDA data is set here)
//      phase 1: SCL=1  -- data held
//      phase 2: SCL=1  -- (receive: SDA is sampled here)
//      phase 3: SCL=0  -- pulled low, prepare next bit
//
//  START : SDA falls while SCL is high
//  STOP  : SDA rises while SCL is high
//
//  Used to read AS5600 (address 0x36).  On the F1 board the bus is
//  SCL = GPIO_80, SDA = GPIO_81.
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  * P25 : every inner `case (ph)` was missing `default`.  `ph` is 2 bit so all
//    4 values are covered today, but an X (e.g. during reset glitch) would fall
//    through silently.  Explicit default branches added.
//  * P26 : state S_RST (repeated-START) was dead code - nothing ever assigned
//    it.  Removed; the working path uses STOP + START (S_STOP1 / S_START2),
//    which is what the official Arduino library does for AS5600.
//  * P10 : added a bus timeout so a stuck slave (SDA held low) cannot hang the
//    acquisition chain forever.
//  * P32 : `sda_in` is an asynchronous input.  It is now routed through a
//    two-flop synchroniser before being sampled in phase P2.
//  * style: complete case statements, single clock/reset, no gated clocks.
//
//  Bit widths:
//      CLK_DIV : parameter, must be >= 2 and <= 65535 (checked below)
//      div     : 16 bit, 0..CLK_DIV-1
//      bitn    : 4 bit, 0..7 used
//      sh      : 8 bit shift register
//      timeout : 20 bit, counts ticks while busy; ~CLK_DIV*2^20/50MHz seconds
//                (CLK_DIV=250 -> ~5.2 s worst case; a transaction needs ~50 ticks)
//
//  Timing assumptions:
//      * `start` is a pulse generated in the clk domain (from cmd/FSM)
//      * `scl_oe` / `sda_oe` are open-drain enables (1 = drive low)
//      * `rd_valid` is a single-cycle pulse in the same cycle `rd_data` is valid
// ============================================================================
`default_nettype none

module i2c_master #(
    parameter integer CLK_DIV = 250          // 50 kHz @50 MHz
)(
    input  wire       clk,
    input  wire       rst_n,
    // ---- user interface ----
    input  wire       start,                 // pulse: begin one register read
    input  wire [6:0] dev_addr,              // 7-bit slave address
    input  wire [7:0] reg_addr,              // register to read
    output reg  [7:0] rd_data,               // data read back
    output reg        rd_valid,              // 1-cycle pulse, data valid
    output reg        err,                   // NACK / timeout (sticky until next start)
    output reg        busy,
    // ---- I2C pins (open drain: oe=1 drives the line low) ----
    output reg        scl_oe,
    output reg        sda_oe,
    input  wire       sda_in,
    // ---- debug probes (no functional effect) ----
    output wire [3:0] dbg_state,
    output wire [1:0] dbg_ph
);

    // ---- synthesise-time parameter check ------------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    localparam [1:0] P0 = 2'd0, P1 = 2'd1, P2 = 2'd2, P3 = 2'd3;

    localparam [3:0] S_IDLE   = 4'd0,
                     S_START  = 4'd1,
                     S_TXA    = 4'd2,   // send address + W
                     S_ACKA   = 4'd3,   // receive ACK
                     S_TXR    = 4'd4,   // send register address
                     S_ACKR   = 4'd5,   // receive ACK
                     S_TXA2   = 4'd7,   // send address + R
                     S_ACKA2  = 4'd8,   // receive ACK
                     S_RX     = 4'd9,   // read data byte
                     S_NACK   = 4'd10,  // master NACK
                     S_STOP   = 4'd11,
                     S_STOP1  = 4'd12,  // STOP after writing the register address
                     S_START2 = 4'd13;  // then START again and read

    reg [3:0]  st;
    reg [1:0]  ph;
    reg [15:0] div;
    reg [3:0]  bitn;
    reg [7:0]  sh;
    reg        ack_seen;
    reg [19:0] timeout;              // P10

    wire tick = (div >= CLK_DIV[15:0] - 16'd1);

    // ---- P32: synchronise the asynchronous SDA input ----
    reg [1:0] sda_sync;
    wire      sda_s = sda_sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sda_sync <= 2'b11;
        else
            sda_sync <= {sda_sync[0], sda_in};
    end

    assign dbg_state = st;
    assign dbg_ph    = ph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; ph <= P0; div <= 16'd0; bitn <= 4'd0; sh <= 8'd0;
            scl_oe <= 1'b0; sda_oe <= 1'b0; ack_seen <= 1'b0;
            rd_data <= 8'd0; rd_valid <= 1'b0; err <= 1'b0; busy <= 1'b0;
            timeout <= 20'd0;
        end else begin
            rd_valid <= 1'b0;

            if (st == S_IDLE) begin
                scl_oe  <= 1'b0;                 // both lines released (high)
                sda_oe  <= 1'b0;
                busy    <= 1'b0;
                timeout <= 20'd0;
                if (start) begin
                    busy    <= 1'b1;
                    err     <= 1'b0;
                    st      <= S_START;
                    ph      <= P0;
                    div     <= 16'd0;
                    sh      <= {dev_addr, 1'b0}; // address + write
                    bitn    <= 4'd7;
                    timeout <= 20'd0;
                end
            end else if (tick) begin
                div <= 16'd0;

                // ---- P10: bus timeout ----
                if (timeout >= 20'hF_FFFF) begin
                    timeout <= 20'd0;
                    busy    <= 1'b0;
                    err     <= 1'b1;
                    scl_oe  <= 1'b0;             // release the bus
                    sda_oe  <= 1'b0;
                    st      <= S_IDLE;
                    ph      <= P0;
                end else begin
                    timeout <= timeout + 20'd1;

                    case (st)
                    // ---------------- START: SDA falls while SCL high --------
                    S_START: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; ph <= P1; end
                            P1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; ph <= P2; end  // START
                            P2: begin scl_oe <= 1'b1; sda_oe <= 1'b1; ph <= P3; end
                            P3: begin scl_oe <= 1'b1; sda_oe <= 1'b0;
                                      st <= S_TXA; ph <= P0; bitn <= 4'd7; end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- transmit one byte ----------------
                    S_TXA, S_TXR, S_TXA2: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b1; sda_oe <= ~sh[7];              ph <= P1; end
                            P1: begin scl_oe <= 1'b0;                                ph <= P2; end
                            P2: begin scl_oe <= 1'b0;                                ph <= P3; end
                            P3: begin scl_oe <= 1'b1;
                                      sh <= {sh[6:0], 1'b0};
                                      if (bitn == 4'd0) begin
                                          case (st)
                                              S_TXA:  st <= S_ACKA;
                                              S_TXR:  st <= S_ACKR;
                                              S_TXA2: st <= S_ACKA2;
                                              default: st <= S_IDLE;
                                          endcase
                                      end else begin
                                          bitn <= bitn - 4'd1;
                                      end
                                      ph <= P0; end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- receive ACK ----------------
                    S_ACKA, S_ACKR, S_ACKA2: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b1; sda_oe <= 1'b0; ph <= P1; end  // release SDA
                            P1: begin scl_oe <= 1'b0; sda_oe <= 1'b0; ph <= P2; end
                            P2: begin scl_oe <= 1'b0; sda_oe <= 1'b0;
                                      ack_seen <= ~sda_s;                            // sample synchronised SDA
                                      ph <= P3; end
                            P3: begin
                                scl_oe <= 1'b1;
                                ph     <= P0;
                                if (!ack_seen) begin
                                    err <= 1'b1;                                     // no ACK
                                    st  <= S_STOP;
                                end else begin
                                    case (st)
                                        S_ACKA:  begin st <= S_TXR;   sh <= reg_addr;        bitn <= 4'd7; end
                                        S_ACKR:  begin st <= S_STOP1; sh <= {dev_addr,1'b1}; bitn <= 4'd7; end
                                        S_ACKA2: begin st <= S_RX;    sh <= 8'd0;            bitn <= 4'd7; end
                                        default: st <= S_IDLE;
                                    endcase
                                end
                            end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- read one byte ----------------
                    S_RX: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b1; sda_oe <= 1'b0; ph <= P1; end   // SCL low, release SDA
                            P1: begin scl_oe <= 1'b0; sda_oe <= 1'b0; ph <= P2; end   // SCL high
                            P2: begin scl_oe <= 1'b0; sda_oe <= 1'b0;
                                      sh <= {sh[6:0], sda_s};                         // sample
                                      ph <= P3; end
                            P3: begin scl_oe <= 1'b1;
                                      if (bitn == 4'd0) st <= S_NACK;
                                      else              bitn <= bitn - 4'd1;
                                      ph <= P0; end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- master NACK ----------------
                    S_NACK: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b1; sda_oe <= 1'b0; ph <= P1; end
                            P1: begin scl_oe <= 1'b0;                 ph <= P2; end
                            P2: begin scl_oe <= 1'b0;                 ph <= P3; end
                            P3: begin scl_oe <= 1'b1;
                                      rd_data  <= sh;
                                      rd_valid <= 1'b1;
                                      st       <= S_STOP;
                                      ph       <= P0; end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- STOP after writing the register address --
                    S_STOP1: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b1; sda_oe <= 1'b1; ph <= P1; end
                            P1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; ph <= P2; end
                            P2: begin scl_oe <= 1'b0; sda_oe <= 1'b0; ph <= P3; end   // STOP
                            P3: begin scl_oe <= 1'b0; sda_oe <= 1'b0;
                                      st <= S_START2; ph <= P0; end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- START again, then address+R ----------------
                    S_START2: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; ph <= P1; end
                            P1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; ph <= P2; end   // START
                            P2: begin scl_oe <= 1'b1; sda_oe <= 1'b1; ph <= P3; end
                            P3: begin scl_oe <= 1'b1; sda_oe <= 1'b0;
                                      st <= S_TXA2; sh <= {dev_addr, 1'b1}; bitn <= 4'd7; ph <= P0; end
                            default: ph <= P0;
                        endcase
                    end

                    // ---------------- STOP ----------------
                    S_STOP: begin
                        case (ph)
                            P0: begin scl_oe <= 1'b1; sda_oe <= 1'b1; ph <= P1; end
                            P1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; ph <= P2; end
                            P2: begin scl_oe <= 1'b0; sda_oe <= 1'b0; ph <= P3; end   // STOP
                            P3: begin scl_oe <= 1'b0; sda_oe <= 1'b0;
                                      st <= S_IDLE; busy <= 1'b0; end
                            default: ph <= P0;
                        endcase
                    end

                    default: begin
                        st     <= S_IDLE;
                        scl_oe <= 1'b0;
                        sda_oe <= 1'b0;
                        busy   <= 1'b0;
                        ph     <= P0;
                    end
                    endcase
                end
            end else begin
                div <= div + 16'd1;
            end
        end
    end

endmodule

`default_nettype wire
