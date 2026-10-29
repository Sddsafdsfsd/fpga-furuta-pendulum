// ============================================================================
//  dac_square.v  ——  DAC 慢速方波发生器（给"输出到底有没有"这类排查用）
// ----------------------------------------------------------------------------
//  每 HALF_MS 毫秒在 HIGH 与 LOW 两个码值之间翻转一次（默认 5 秒）。
//  为什么需要它：
//    * 三角波太慢时，LED 只能看到"缓慢滚动"，用万用表也读不准（一直在变）；
//    * 改成 5 秒一档的方波后：
//        - LED 会每 5 秒整片翻转（一眼就能分辨"通/不通"）
//        - 万用表能读到稳定的 ±5V（DAC 满量程），彻底判定 DAC 输出好坏
// ============================================================================
`default_nettype none

module dac_square #(
    parameter integer CLK_HZ  = 50_000_000,
    parameter integer HALF_MS = 5000,     // 每多少毫秒翻转一次
    parameter integer HIGH    = 1023,
    parameter integer LOW     = 0
)(
    input  wire       clk,
    input  wire       rst_n,
    output reg [9:0]  code,
    output reg        update
);
    localparam integer DIV = (CLK_HZ/1000) * HALF_MS;   // 250000000 @5s -> 用 32 位

    reg [31:0] cnt;
    reg        hi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt    <= 32'd0;
            hi     <= 1'b0;
            code   <= LOW[9:0];
            update <= 1'b0;
        end else begin
            update <= 1'b0;
            if (cnt == DIV-1) begin
                cnt    <= 32'd0;
                hi     <= ~hi;
                code   <= hi ? LOW[9:0] : HIGH[9:0];   // 切换后立刻换电平
                update <= 1'b1;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
