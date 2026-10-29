// ============================================================================
//  adc_3pa1030.v  --  3PA1030 (10 bit / 50 Msps / 0..2V) parallel ADC front end
// ============================================================================
//  Hardware facts (PocketLab-F1 user manual 3.2.3 / table 3-18-1):
//    * the ADC front end attenuates -5V..+5V to 0V..2V before the 3PA1030
//      => code ~= (Vin + 5.0) / 10.0 * 1023      (0V -> ~512, 3.3V -> ~850)
//    * signals: AD1_OE(15) / AD1_CLK(32) / AD1_OVR(34) / AD1_D0..D9
//    * FPGA generates AD_CLK; the ADC drives parallel data after the clock edge
//
//  ---------------------------------------------------------------------------
//  v2 refactor notes
//  ---------------------------------------------------------------------------
//  * P21 : the original terminated the accumulation with
//              if (n == 1023) value <= acc[19:10];
//          but `acc` at that moment does NOT yet include the 1024th sample,
//          so only 1023 samples were summed while dividing by 1024 -> the mean
//          was systematically low by 0.098% and one sample was dropped.
//          Fixed by adding the current sample when computing the final value.
//  * P31 : CLK_DIV / AVG_SHIFT / SAMPLE_DLY range checks.
//  * P44 : corrected the sample-rate comments (the accumulator actually runs at
//          12.5 Msps because d_sync_v is high every other clock).
//
//  Bit widths:
//      div_cnt   : 8 bit, 0..CLK_DIV-1     => CLK_DIV must be <= 256
//      d_sync    : 10 bit sample register
//      acc       : 32 bit, 1024 * 1023 = 1_047_552 < 2^21  (plenty)
//      n         : 16 bit, 0..2^AVG_SHIFT-1 => AVG_SHIFT must be <= 20
//      value     : 10 bit = acc[AVG_SHIFT+9 : AVG_SHIFT]
//                  requires AVG_SHIFT+9 <= 31 => AVG_SHIFT <= 22
//
//  Timing:
//      single clock; ad_clk is derived and ONLY output to the pin, it is never
//      used as a clock inside the FPGA.  Sample capture happens while ad_clk_r
//      is high (i.e. 20ns after the ADC sees the rising edge at 25MHz).
// ============================================================================
`default_nettype none

module adc_3pa1030 #(
    parameter integer OE_ACTIVE_LOW = 1,      // output-enable polarity (usually active low)
    parameter integer CLK_DIV       = 1,      // 1 => AD_CLK = 50MHz/2 = 25MHz
    parameter integer AVG_SHIFT     = 10,     // average 2^10 = 1024 samples per 1 kHz tick
    parameter integer SAMPLE_DLY    = 1       // extra clocks to wait before sampling
)(
    input  wire        clk,          // 50 MHz system clock (only clock domain)
    input  wire        rst_n,
    input  wire        tick_1k,      // 1 kHz enable pulse
    input  wire [9:0]  ad_d,         // ADC parallel data
    input  wire        ad_ovr,       // ADC overflow flag (diagnostic only for now)
    output wire        ad_clk,       // sampling clock to the ADC
    output wire        ad_oe,        // output enable
    output reg  [9:0]  value,        // averaged sample 0..1023
    output reg         value_valid   // one-cycle pulse per 1 kHz tick
);

    // ---- synthesise-time parameter checks -----------------------------------
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    localparam integer SAMPLE_DLY_C = (SAMPLE_DLY < 1) ? 1 : SAMPLE_DLY;
    localparam [15:0]  AVG_LAST     = (1 << AVG_SHIFT) - 1;

    // ---- output enable: keep the ADC driving the bus ----
    assign ad_oe = OE_ACTIVE_LOW ? 1'b0 : 1'b1;

    // ========================================================================
    //  AD_CLK generation: 50MHz / (2*CLK_DIV).  Output-only, never used as a
    //  clock inside the FPGA (no gated / derived clock anywhere in this design).
    // ========================================================================
    reg [7:0] div_cnt;
    reg       ad_clk_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt  <= 8'd0;
            ad_clk_r <= 1'b0;
        end else if (div_cnt >= CLK_DIV[7:0] - 8'd1) begin
            div_cnt  <= 8'd0;
            ad_clk_r <= ~ad_clk_r;
        end else begin
            div_cnt <= div_cnt + 8'd1;
        end
    end

    assign ad_clk = ad_clk_r;

    // ========================================================================
    //  Sample capture while AD_CLK is high (data is stable 20 ns after the
    //  ADC sees the rising edge at 25 MHz).
    // ========================================================================
    reg [9:0] d_sync;
    reg       d_sync_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_sync   <= 10'd0;
            d_sync_v <= 1'b0;
        end else begin
            d_sync_v <= 1'b0;
            if (ad_clk_r) begin
                d_sync   <= ad_d;      // if data is unstable, raise CLK_DIV
                d_sync_v <= 1'b1;
            end
        end
    end

    // ========================================================================
    //  Accumulate 2^AVG_SHIFT samples per 1 kHz tick, then average.
    // ========================================================================
    reg [31:0] acc;
    reg [15:0] n;
    reg        busy;
    reg [3:0]  dly;

    // ---- P21: final sum including the last sample, then average ----
    wire [31:0] acc_final = acc + {22'd0, d_sync};
    wire [31:0] avg_val   = acc_final >> AVG_SHIFT;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc         <= 32'd0;
            n           <= 16'd0;
            busy        <= 1'b0;
            dly         <= 4'd0;
            value       <= 10'd0;
            value_valid <= 1'b0;
        end else begin
            value_valid <= 1'b0;

            if (tick_1k) begin
                busy <= 1'b1;
                acc  <= 32'd0;
                n    <= 16'd0;
                dly  <= SAMPLE_DLY_C[3:0];
            end else if (busy) begin
                if (dly != 4'd0) begin
                    dly <= dly - 4'd1;
                end else if (d_sync_v) begin
                    if (n == AVG_LAST) begin
                        // ---- P21 fix: acc holds AVG_LAST samples here; ----
                        // acc_final adds the last one, so AVG_LAST+1 samples
                        // (= 2^AVG_SHIFT) are averaged.  The original omitted
                        // the last sample while still dividing by 2^AVG_SHIFT.
                        value       <= avg_val[9:0];
                        busy        <= 1'b0;
                        value_valid <= 1'b1;
                    end else begin
                        acc <= acc + {22'd0, d_sync};
                        n   <= n + 16'd1;
                    end
                end
            end
        end
    end

    // ---- keep the overflow pin referenced (diagnostic only for now) ----
    wire unused_ovr;
    assign unused_ovr = ad_ovr;

endmodule

`default_nettype wire
