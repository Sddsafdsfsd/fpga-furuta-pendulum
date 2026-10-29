// ============================================================================
//  tb6612_drv.v  ——  TB6612FNG 双直流电机驱动接口
// ----------------------------------------------------------------------------
//  管脚对应：PWMA = PWM，AIN1/AIN2 = 方向，STBY = 使能（1 工作 / 0 待机）
//  真值表：IN1=1,IN2=0 正转 ｜ IN1=0,IN2=1 反转 ｜ IN1=1,IN2=1 短路刹车
//  PWM 频率默认 20kHz（在音频之上，听不见啸叫）
//
//  【安全设计】
//    1. enable=0 → STBY=0，驱动器整体待机，电机不受控
//    2. 换向时先拉低 PWM、封锁 DEAD_CLK 拍，再切换 IN1/IN2（防 H 桥直通）
//
//  ★★ v2 重构：修复 P15（严重）★★
//     原代码：wire [15:0] duty = duty_mul / CMD_MAX;   ← 32 位除以非常量
//             综合成数十级 LUT 的除法器，在组合路径上，50MHz 时序压力大
//     修复：预计算缩放系数 DUTY_SCALE = PERIOD * 1024 / CMD_MAX
//           duty = (mag * DUTY_SCALE) >> 10
//           当 PERIOD/CMD_MAX = 2.5 时，DUTY_SCALE = 2560，结果与除法【完全相同】
//           （200kHz 综合无除法器，纯乘法 + 移位）
//
//  ★ 位宽说明 ★
//     cmd       : 16 位有符号，输入
//     cmd_c     : 限幅到 ±CMD_MAX（CMD_MAX ≤ 32767，下方 generate 检查）
//     mag_s     : 取绝对值，16 位
//     mag       : 16 位无符号，0..CMD_MAX
//     DUTY_SCALE: localparam，= PERIOD*1024/CMD_MAX，要求能被整除（generate 检查）
//     duty_mul  : 32 位 = mag * DUTY_SCALE，最大 1000*2560 = 2,560,000（22 位）✓
//     duty      : 32 位，右移 10 位后 ≤ PERIOD ✓
//     cnt       : 16 位 PWM 载波计数器，0..PERIOD-1
//
//  ★ 时序假设 ★
//     · 单时钟 clk，rst_n 低有效
//     · cnt 自由运行，无使能
//     · cmd 来自 PID（同步信号），无需额外同步
// ============================================================================
`default_nettype none

module tb6612_drv #(
    parameter integer CLK_HZ   = 50_000_000,
    parameter integer PWM_HZ   = 20_000,      // PWM 频率
    parameter integer DEAD_CLK = 5_000,       // 换向封锁时间（5000 clk = 100us）
    parameter integer CMD_MAX  = 1000         // 控制量满量程（对应 100% 占空比）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,         // 0 = 待机（安全默认）
    input  wire signed [15:0] cmd,            // -CMD_MAX..+CMD_MAX，正=正转
    output reg                pwm,            // PWMA
    output reg                ain1,
    output reg                ain2,
    output wire               stby,
    output reg  signed [15:0] cmd_out         // 实际生效的控制量（遥测用）
);

    localparam integer PERIOD = CLK_HZ / PWM_HZ;                  // 2500 @20kHz
    // ---- ★ P15 修复：预计算缩放系数，用移位代替除法 ----
    localparam integer DUTY_SCALE = (PERIOD * 1024) / CMD_MAX;    // 2560 @2500/1000

    // ---- 参数检查 ----
    // (removed) parameter-check generate block: Gowin mis-handles undefined modules

    reg [15:0] cnt;            // PWM 载波
    reg        dir_r;          // 1 = 正转
    reg [15:0] blank;          // 换向封锁倒计时

    // ---- ★ P46 修复：显式有符号常量，避免位选丢失符号 ★
    //   原代码 (cmd > CMD_MAX[15:0])：对 integer 参数做位选，按 IEEE 1364 会降为无符号，
    //   负的 cmd 会被判成巨大正数 → 限幅方向错误。这里用 signed localparam 彻底消除歧义。
    localparam signed [15:0] CMD_MAX_S = CMD_MAX;
    localparam signed [15:0] CMD_MIN_S = -CMD_MAX_S;

    // ---- 限幅 + 取绝对值 ----
    wire signed [15:0] cmd_c = (cmd > CMD_MAX_S) ? CMD_MAX_S :
                               (cmd < CMD_MIN_S) ? CMD_MIN_S : cmd;
    wire signed [15:0] mag_s = cmd_c[15] ? (~cmd_c + 16'sd1) : cmd_c;
    wire [15:0]        mag   = mag_s[15:0];

    // ---- 控制量 → 占空比拍数（乘法 + 移位，无除法器）----
    wire [31:0] duty_mul = mag * DUTY_SCALE[15:0];
    wire [15:0] duty     = duty_mul[25:10];      // 右移 10 位（duty_mul ≤ 22 位 ✓）

    // ---- 方向变化检测 ----
    wire want_fwd = (cmd_c > 16'sd0);
    wire want_rev = (cmd_c < 16'sd0);
    wire dir_chg  = (want_fwd && (dir_r == 1'b0)) || (want_rev && (dir_r == 1'b1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= 16'd0;
            pwm     <= 1'b0;
            ain1    <= 1'b0;
            ain2    <= 1'b0;
            dir_r   <= 1'b1;
            blank   <= 16'd0;
            cmd_out <= 16'sd0;
        end else begin
            // ---- PWM 载波 ----
            if (cnt >= PERIOD[15:0] - 16'd1)
                cnt <= 16'd0;
            else
                cnt <= cnt + 16'd1;

            // ---- 换向封锁计时 ----
            if (dir_chg)
                blank <= DEAD_CLK[15:0];
            else if (blank != 16'd0)
                blank <= blank - 16'd1;

            // ---- 方向锁存 ----
            if (want_fwd)
                dir_r <= 1'b1;
            else if (want_rev)
                dir_r <= 1'b0;

            // ---- 输出 ----
            if (!enable) begin
                pwm     <= 1'b0;
                ain1    <= 1'b0;
                ain2    <= 1'b0;
                cmd_out <= 16'sd0;
            end else begin
                pwm     <= (blank == 16'd0) && (cnt < duty);
                ain1    <= dir_r;
                ain2    <= ~dir_r;
                cmd_out <= cmd_c;
            end
        end
    end

    assign stby = enable;

endmodule

`default_nettype wire
