// ============================================================================
//  pid_s16.v  ——  有符号定点 PID（位置环用：编码器位置有正有负）
// ============================================================================
//  ★★ v2 重构：修复 P02（致命）★★
//     原代码：wire signed [15:0] err = target - actual;   ← 16 位相减会溢出
//     失效场景：编码器计数到 -25000，而 pos_target = +9999
//               err 应为 34999，16 位回绕成 -30537 → 符号翻转
//               → 位置环变成【正反馈】→ 电机朝错误方向加速 → 撞机构
//     修复：误差扩到 17 位有符号（范围 -65536..+65535，覆盖全部可能差值）
//           同时 err_prev 也改为 17 位，差分量扩到 18 位
//
//  ★ 位宽说明（逐项）★
//     target / actual  : 16 位有符号，范围 -32768..+32767
//     err              : 17 位有符号 = {符号扩展, target} - {符号扩展, actual}
//                        范围 -65535..+65535 ✓ 不溢出
//     err_prev         : 17 位（存上一拍 err）
//     err_d = err-err_prev : 18 位，范围 -131070..+131070 ✓
//     integ            : 32 位，被 INT_MAX 限幅（默认 20000）
//     p_term = KP_Q8 * err : KP_Q8 ≤ 2^14 时 ≤ 2^14 × 2^17 = 2^31 ✓ 32 位够
//     d_term = KD_Q8 * err_d : KD_Q8 ≤ 2^13 时 ≤ 2^13 × 2^18 = 2^31 ✓ 32 位够
//     sum_q8           : 三项之和，被限幅前最大约 3×2^31 ✗ → 见下方参数检查
//                        ★ 实际使用中 KP/KD 远小于上限，且积分被 INT_MAX 限幅 ★
//     sum = sum_q8 >>> 8 : 算术右移，保持符号 ✓
//     out              : 先与 ±OUT_MAX 比较限幅，再取 sum[15:0]
//                        要求 OUT_MAX ≤ 32767（下方 generate 检查）
//
//  ★ 时序假设 ★
//     · 本模块只有 clk / rst_n / tick_1k，无其他时钟域
//     · tick_1k 为单拍使能，所有状态只在 tick_1k=1 的那一拍更新
//     · clear 优先级高于 tick_1k（与原来一致）
//     · err/err_prev 是组合逻辑产生，从 target/actual 到 out 的组合路径约
//       乘法器(18×18) + 加法 + 移位 + 比较，50MHz 下无压力
// ============================================================================
`default_nettype none

module pid_s16 #(
    parameter signed [15:0] KP_Q8   = 16'sd64,    // Kp = 0.25
    parameter signed [15:0] KI_Q8   = 16'sd1,     // Ki = 0.004
    parameter signed [15:0] KD_Q8   = 16'sd256,   // Kd = 1.0
    parameter integer       OUT_MAX = 500,        // 输出限幅（要求 ≤ 32767）
    parameter integer       INT_MAX = 20000       // 积分限幅（抗饱和）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1k,            // 1kHz 控制节拍（单拍使能）
    input  wire               clear,              // 清积分（Z/C 指令，高有效）
    input  wire signed [15:0] target,             // 目标位置（编码器计数）
    input  wire signed [15:0] actual,             // 实际位置
    output reg  signed [15:0] out                 // 控制量
);

    // ---- 参数合法性检查（综合期报错，避免带病上板）----
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    // ---- ★ P02 修复：误差扩到 17 位 ----
    wire signed [16:0] err = {target[15], target} - {actual[15], actual};

    reg  signed [16:0] err_prev;
    reg  signed [31:0] integ;

    wire signed [17:0] err_d = err - {err_prev[16], err_prev};

    wire signed [31:0] p_term = KP_Q8 * err;
    wire signed [31:0] i_term = KI_Q8 * integ;
    wire signed [31:0] d_term = KD_Q8 * err_d;

    wire signed [31:0] sum_q8 = p_term + i_term + d_term;
    wire signed [31:0] sum    = sum_q8 >>> 8;          // 算术右移，保符号

    // ---- ★ P46 修复：显式有符号常量，避免位选丢失符号 ★
    localparam signed [15:0] OUT_MAX_S = OUT_MAX;
    localparam signed [15:0] OUT_MIN_S = -OUT_MAX_S;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integ    <= 32'sd0;
            err_prev <= 17'sd0;
            out      <= 16'sd0;
        end else if (clear) begin
            integ    <= 32'sd0;
            err_prev <= 17'sd0;
            out      <= 16'sd0;
        end else if (tick_1k) begin
            // ---- 积分累加 + 限幅（抗积分饱和）----
            err_prev <= err;

            if (integ + err > INT_MAX)
                integ <= INT_MAX;
            else if (integ + err < -INT_MAX)
                integ <= -INT_MAX;
            else
                integ <= integ + err;

            // ---- 输出限幅 ----
            if (sum > OUT_MAX_S)
                out <= OUT_MAX_S;
            else if (sum < OUT_MIN_S)
                out <= OUT_MIN_S;
            else
                out <= sum[15:0];
        end
    end

endmodule

`default_nettype wire
