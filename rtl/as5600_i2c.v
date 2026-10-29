// ============================================================================
//  as5600_i2c.v  --  AS5600 I2C read (ported from FpOC, with v2 fixes)
// ============================================================================
//  Origin: https://gitee.com/lzh1762/FpOC_1  RTL/sensors/i2c_register_read.sv
//  The original implementation is field-proven (used in a FOC motor controller),
//  so the bit-level timing below is intentionally left EXACTLY as it was.
//  The v2 refactor only ADDS safety, it does not change the generated waveform.
//
//  Timing (why this differs from a conventional I2C FSM):
//     * SCL is driven directly from counter bit cnt[1]: one bit = 4 epochs
//     * transmit: SDA changes while cnt[1:0]==2'b01 (SCL low)
//     * receive : SDA is sampled while cnt[1:0]==2'b11 (SCL high)
//     * START   : cnt 2..3, SCL held high, SDA pulled low
//     * 2-byte read (AS5600 angle 0x0E/0x0F): ACK before the 2nd byte, NACK last
//
//  SCL frequency = clk / (4 * CLK_DIV).  CLK_DIV=250 @50MHz -> 50 kHz.
//  NOTE: the previous comment claimed 50 kHz while the actual value in use was
//        1000 (-> 12.5 kHz).  Both values are legal; 12.5 kHz gives far more
//        margin on a weak internal pull-up, at the cost of refresh rate (P22).
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  ★ P01 (FATAL) : the original never sampled the ACK slot.  If the sensor is
//    absent / unplugged / the pull-up is broken, `regout` simply keeps its old
//    value (or 0xFFFF after power-up) and the control loop happily uses it.
//    With STAGE=5 the angle becomes 1023, the PID saturates and the 12V geared
//    motor runs at full speed in one direction.
//    FIX: sample SDA in every ACK slot, raise a sticky `err` for the whole
//    transaction, and expose it as an output so the top level can force
//    mot_en=0 (see top.v).
//  ★ P10 : added a bus timeout.  If a slave holds SDA low (or never responds)
//    the counter machine would otherwise sit forever and the whole acquisition
//    chain dies silently.  On timeout the transaction is aborted, `err` is
//    raised and `done` pulses so the caller can retry.
//  ★ P43 : reset port renamed rstn -> rst_n (consistent with the rest).
//  ★ P23 : `en` de-assertion no longer aborts mid-transaction abruptly; it still
//    releases the bus (that part must stay), but `err` is raised so the caller
//    knows the last read is not trustworthy.
//
//  Bit widths:
//      CLK_DIV   : 16 bit parameter, 0 => clamped to 1 (epoch every clock)
//      clkcnt    : 16 bit, 0..CLK_DIV_PARSED
//      cnt       : 8 bit, 0..204  (204 = one full transaction, fits fine)
//      timeout   : 20 bit @50MHz ~ 21 ms  (a transaction at 12.5 kHz takes ~4 ms)
//      send_shift: 8 bit shift register
//      recv_shift: 16 bit shift register
//      regout    : 16 bit  (only updated when the transaction completed cleanly)
//      err       : sticky per transaction, cleared on the next `start`
//
//  Timing assumptions:
//      * single clock `clk`, single reset `rst_n` (active low, async assert)
//      * `en` and `start` are level/pulse signals generated in the clk domain
//      * SDA is an asynchronous input; it is sampled only while SCL is high,
//        which is at least CLK_DIV clocks after the SCL edge (~10 us at 50 kHz)
//        so no extra synchroniser is required for correct operation.  Two-flop
//        synchronisers are still used where SDA feeds a decision (ACK slot).
// ============================================================================
`default_nettype none

module as5600_i2c #(
    parameter [15:0] CLK_DIV       = 16'd250,   // 50MHz/(4*250) = 50 kHz
    parameter [ 6:0] SLAVE_ADDR    = 7'h36,     // AS5600
    parameter [ 7:0] REGISTER_ADDR = 8'h0E      // 0x0E = angle high byte
)(
    input  wire        rst_n,       // P43: was rstn
    input  wire        clk,
    output reg         scl,
    inout  wire        sda,
    input  wire        en,          // 0 = fully release the bus
    input  wire        start,       // pulse: begin one read transaction
    output wire        ready,       // 1 = idle, ready to accept start
    output reg         done,        // 1-cycle pulse: transaction finished
    output reg  [15:0] regout,      // last successfully read value
    output reg         err          // ★ P01: sticky NACK/timeout flag
);

    localparam [15:0] CLK_DIV_PARSED = (CLK_DIV > 16'd0) ? (CLK_DIV - 16'd1) : 16'd0;
    localparam [19:0] TIMEOUT_MAX    = 20'd1_000_000;   // ~20 ms @50MHz

    // ---- parameter check ----------------------------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    // ========================================================================
    //  SDA pad: open drain.  Driven low only when (en && sda_e && !sda_o),
    //  which keeps the original polarity (sda_o=1 means "release").
    // ========================================================================
    reg sda_e, sda_o;
    assign sda = (en && sda_e) ? sda_o : 1'bz;

    // ---- synchronise the SDA input before it is used for an ACK decision ----
    reg [1:0] sda_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sda_sync <= 2'b11;
        else
            sda_sync <= {sda_sync[0], sda};
    end
    wire sda_s = sda_sync[1];

    // ========================================================================
    //  Epoch generator (one I2C quarter-bit)
    // ========================================================================
    reg        epoch;
    reg [15:0] clkcnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            epoch  <= 1'b0;
            clkcnt <= 16'd0;
        end else if (clkcnt == CLK_DIV_PARSED) begin
            epoch  <= 1'b1;
            clkcnt <= 16'd0;
        end else begin
            epoch  <= 1'b0;
            clkcnt <= clkcnt + 16'd1;
        end
    end

    // ========================================================================
    //  Main bit-banged state machine (waveform unchanged from the original)
    //  NOTE: every reg is declared BEFORE it is read (P03 lesson).
    // ========================================================================
    reg [ 7:0] cnt;
    reg [ 7:0] send_shift;
    reg [15:0] recv_shift;
    reg [19:0] timeout;        // P10: bus timeout counter
    reg        nack_seen;      // P01: sticky NACK for this transaction

    // ready: declared after `cnt` so there is no forward reference
    assign ready = (cnt == 8'd0);

    // ACK slot windows (cnt ranges where the slave drives SDA):
    //   cnt 37..39  : ACK after address+W
    //   cnt 73..76  : ACK after register address
    //   cnt 117..120: ACK after address+R
    wire in_ack_slot = ((cnt >= 8'd37) && (cnt < 8'd40)) ||
                       ((cnt >= 8'd73) && (cnt < 8'd77)) ||
                       ((cnt >= 8'd117) && (cnt < 8'd121));

    // P10: one transaction at 12.5 kHz takes ~4 ms; 20 ms timeout is generous
    wire bus_timeout = (cnt != 8'd0) && (timeout >= TIMEOUT_MAX);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl        <= 1'b1;
            sda_e      <= 1'b0;          // release the bus after reset
            sda_o      <= 1'b1;
            cnt        <= 8'd0;
            send_shift <= 8'd0;
            recv_shift <= 16'd0;
            regout     <= 16'd0;
            done       <= 1'b0;
            err        <= 1'b0;
            timeout    <= 20'd0;
            nack_seen  <= 1'b0;
        end else if (!en) begin
            // ---- bus release (unchanged behaviour) + P23: flag the abort ----
            scl   <= 1'b1;
            sda_e <= 1'b0;
            sda_o <= 1'b1;
            done  <= 1'b0;
            timeout <= 20'd0;
            if (cnt != 8'd0) begin       // aborted mid-transaction
                err <= 1'b1;
                cnt <= 8'd0;
            end
        end else if (bus_timeout) begin
            // ---- P10: abort a stuck transaction ----
            timeout <= 20'd0;
            cnt     <= 8'd0;
            scl     <= 1'b1;
            sda_e   <= 1'b0;
            sda_o   <= 1'b1;
            err     <= 1'b1;
            done    <= 1'b1;
            nack_seen <= 1'b0;
        end else begin
            // ---- advance / clear the timeout counter ----
            if (cnt != 8'd0)
                timeout <= timeout + 20'd1;
            else
                timeout <= 20'd0;

            // ---- P01: sample the ACK slot while SCL is high ----
            if (in_ack_slot && (cnt[1:0] == 2'b11) && sda_s)
                nack_seen <= 1'b1;       // SDA high during ACK = NACK

            if (ready) begin
                scl    <= 1'b1;
                sda_e  <= 1'b1;          // driver enabled, SDA released high
                sda_o  <= 1'b1;
                nack_seen <= 1'b0;
                if (start) begin
                    cnt     <= 8'd1;
                    err     <= 1'b0;     // clear the sticky error on a new read
                    done    <= 1'b0;
                    timeout <= 20'd0;
                end
            end else if (done) begin
                done <= 1'b0;
                cnt  <= 8'd0;
            end else if (epoch) begin
                cnt <= cnt + 8'd1;

                if (cnt < 8'd2) begin
                    // idle
                end else if (cnt < 8'd4) begin
                    sda_o      <= 1'b0;                    // START
                    send_shift <= {SLAVE_ADDR, 1'b0};      // address + write
                end else if (cnt < 8'd37) begin
                    scl <= cnt[1];
                    if (cnt[1:0] == 2'b01)
                        {sda_o, send_shift} <= {send_shift, 1'b1};
                end else if (cnt < 8'd40) begin            // ACK slot 1
                    send_shift <= REGISTER_ADDR;
                    scl        <= cnt[1];
                    sda_e      <= 1'b0;
                end else if (cnt < 8'd73) begin
                    scl <= cnt[1];
                    if (cnt[1:0] == 2'b01) begin
                        sda_e <= 1'b1;
                        {sda_o, send_shift} <= {send_shift, 1'b1};
                    end
                end else if (cnt < 8'd77) begin            // ACK slot 2
                    scl   <= cnt[1];
                    sda_e <= 1'b0;
                end else if (cnt == 8'd77) begin
                    scl   <= cnt[1];
                    sda_e <= 1'b1;
                    sda_o <= 1'b1;
                end else if (cnt < 8'd82) begin            // bus idle
                    scl <= 1'b1;
                end else if (cnt < 8'd84) begin            // repeated START
                    scl        <= 1'b1;
                    sda_o      <= 1'b0;
                    send_shift <= {SLAVE_ADDR, 1'b1};      // address + read
                end else if (cnt < 8'd117) begin
                    scl <= cnt[1];
                    if (cnt[1:0] == 2'b01)
                        {sda_o, send_shift} <= {send_shift, 1'b1};
                end else if (cnt < 8'd121) begin           // ACK slot 3
                    scl   <= cnt[1];
                    sda_e <= 1'b0;
                end else if (cnt < 8'd153) begin           // data byte 1
                    scl   <= cnt[1];
                    sda_e <= 1'b0;
                    if (cnt[1:0] == 2'b11)
                        recv_shift <= {recv_shift[14:0], sda_s};
                end else if (cnt < 8'd157) begin           // master ACK
                    scl   <= cnt[1];
                    sda_e <= 1'b1;
                    sda_o <= 1'b0;
                end else if (cnt < 8'd189) begin           // data byte 2
                    scl   <= cnt[1];
                    sda_e <= 1'b0;
                    if (cnt[1:0] == 2'b11)
                        recv_shift <= {recv_shift[14:0], sda_s};
                end else if (cnt < 8'd193) begin           // master NACK
                    scl   <= cnt[1];
                    sda_e <= 1'b1;
                    sda_o <= 1'b1;
                end else if (cnt < 8'd195) begin           // STOP preparation
                    scl   <= 1'b0;
                    sda_o <= 1'b0;
                end else if (cnt < 8'd198) begin
                    scl   <= 1'b1;
                    sda_o <= 1'b0;
                end else if (cnt < 8'd204) begin           // SDA rising = STOP
                    sda_o  <= 1'b1;
                    // ---- P01: only publish data if the whole read was ACKed ----
                    if (!nack_seen)
                        regout <= recv_shift;
                    err <= nack_seen;                      // sticky for the caller
                end else begin
                    done <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
