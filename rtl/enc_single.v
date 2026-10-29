// ============================================================================
//  enc_single.v  ——  单相编码器计数（B 相损坏时的临时方案）
//
//  背景：本项目的 JGB37-520 电机编码器 B 相无输出（实测恒定 0.6V ✗），
//        四倍频解码器因缺少 B 相而"正负交替计数"，净计数恒为 0 ✗。
//        这里改为只用 A 相计数：每个 A 相上升沿计一次数。
//
//  分辨率：11 线 × 减速比 30 = 330 计数/输出轴圈（四倍频时是 1320）
//
//  方向判断：单相无法从信号判断方向 ✗，用"当前电机命令的符号"替代
//            · 命令 > 0 → +1      · 命令 < 0 → -1      · 命令 = 0 → +1（手转时累计）
//            这在开环/闭环驱动小车时是正确的（车往哪走由电机命令决定）；
//            换到 B 相正常的电机后，把 ENC_MODE 改回 0 即可用回四倍频解码。
// ============================================================================
`default_nettype none

module enc_single #(
    parameter integer CNT_MAX = 20000          // 计数限幅（±）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               a_in,            // A 相（已做同步/上拉）
    input  wire signed [15:0] cmd,             // 电机命令，用来判断方向
    output reg  signed [15:0] pos              // 累计位置（计数）
);
    reg a_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_d <= 1'b0;
            pos <= 16'sd0;
        end else begin
            a_d <= a_in;
            if (a_in && !a_d) begin            // A 相上升沿
                if (cmd < 16'sd0) begin
                    if (pos > -CNT_MAX) pos <= pos - 16'sd1;
                end else begin
                    if (pos <  CNT_MAX) pos <= pos + 16'sd1;
                end
            end
        end
    end
endmodule

`default_nettype wire
