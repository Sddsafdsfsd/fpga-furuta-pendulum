// ============================================================================
//  tick_gen.v  ——  全工程统一的节拍（同步使能脉冲）发生器
// ----------------------------------------------------------------------------
//  作用：从 50MHz 单时钟产生各模块需要的"单拍使能脉冲"
//        取代原来散落在 telemetry / tb6612 / servo / ramp 等模块里的自建计数器
//
//  ★ 规范（见设计文档 §4）★
//    · 输出的是【单拍脉冲】，不是时钟 → 绝不用于 always @(posedge tick_1k)
//    · 用法：always @(posedge clk or negedge rst_n) ... else if (tick_1k) ...
//
//  ★ 位宽说明（P31）★
//    cnt_1k 为 16 位，要求 DIV_1K ≤ 65536
//    cnt_100k 为 9 位，要求 DIV_100K ≤ 512
//    下方 generate 块在参数越界时给出综合期错误提示
// ============================================================================
`default_nettype none

module tick_gen #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer TICK_1K_HZ   = 1000,        // 目标 1kHz
    parameter integer TICK_100K_HZ = 100_000      // 目标 100kHz
)(
    input  wire clk,                              // 50MHz
    input  wire rst_n,
    output reg  tick_1k,                          // 1kHz 单拍脉冲
    output reg  tick_100k                         // 100kHz 单拍脉冲
);

    localparam integer DIV_1K   = CLK_HZ / TICK_1K_HZ;     // 50000 @50MHz
    localparam integer DIV_100K = CLK_HZ / TICK_100K_HZ;   // 500   @50MHz

    // ---- 参数约束（文档说明，不再用"例化不存在模块"的写法）----
    //   DIV_1K   must be 2..65536  (16-bit counter)
    //   DIV_100K must be 2..512    (9-bit counter)
    // 【教训】原先这里用 generate + 例化不存在的模块来做综合期报错，
    //   但 Gowin 会把这个分支也展开，导致整个模块的网表被破坏
    //   （表现为引脚丢失、P&R 报 "dedicated pin" 冲突）。

    reg [15:0] cnt_1k;
    reg [8:0]  cnt_100k;

    // ---- 1kHz 节拍 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1k  <= 16'd0;
            tick_1k <= 1'b0;
        end else if (cnt_1k >= DIV_1K[15:0] - 16'd1) begin
            cnt_1k  <= 16'd0;
            tick_1k <= 1'b1;                          // 单拍脉冲
        end else begin
            cnt_1k  <= cnt_1k + 16'd1;
            tick_1k <= 1'b0;
        end
    end

    // ---- 100kHz 节拍 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_100k  <= 9'd0;
            tick_100k <= 1'b0;
        end else if (cnt_100k >= DIV_100K[8:0] - 9'd1) begin
            cnt_100k  <= 9'd0;
            tick_100k <= 1'b1;
        end else begin
            cnt_100k  <= cnt_100k + 9'd1;
            tick_100k <= 1'b0;
        end
    end

endmodule

`default_nettype wire
