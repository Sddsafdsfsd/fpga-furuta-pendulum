// ============================================================================
//  dac_3pd5651.v  ——  3PD5651E 双路 10bit / 125Msps DAC 的并口驱动
// ----------------------------------------------------------------------------
//  硬件事实（PocketLab-F1 用户手册 3.3.4 节 / 表 3-19-1）：
//      DA1_CLK      = GPIO_136
//      DA1_DATA0..9 = GPIO_124,126,128,129,130,131,132,133,134,135   (D0 = LSB)
//      差分电流输出 -> 低通滤波 -> 运放差分转单端，输出 -5V~+5V
//
//  用法：DAC 芯片在 DAC_CLK 的上升沿锁存数据，所以数据要在时钟低电平期间更新
//        （本模块就是这么做的，留出 20ns 建立时间）。
//        DAC 需要连续时钟，即使数据不变也要一直给。
// ============================================================================
`default_nettype none

module dac_3pd5651 #(
    parameter integer CLK_DIV = 1          // 1 => DAC_CLK = 50MHz/2 = 25MHz
)(
    input  wire       clk,                 // 50MHz
    input  wire       rst_n,
    input  wire [9:0] value,               // 想输出的码值
    input  wire       update,              // 更新脉冲（在时钟低电平期间生效）
    output reg  [9:0] d_out,
    output reg        dac_clk
);
    // ---- DAC 采样时钟 ----
    reg [7:0] div_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 8'd0;
            dac_clk <= 1'b0;
        end else if (div_cnt == CLK_DIV-1) begin
            div_cnt <= 8'd0;
            dac_clk <= ~dac_clk;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    // ---- 数据只在 DAC_CLK 低电平期间更新（保证锁存沿的建立时间）----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            d_out <= 10'd0;
        else if (update && (dac_clk == 1'b0))
            d_out <= value;
    end

endmodule

`default_nettype wire
