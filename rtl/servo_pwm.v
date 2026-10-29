// ============================================================================
//  servo_pwm.v  ——  舵机 PWM（50Hz，500us~2500us）
// ----------------------------------------------------------------------------
//  舵机接口：F1 扩展板 DUOJI_PWM = GPIO_80（手册表 3-20）
//  标准舵机：周期 20ms，脉宽 0.5ms~2.5ms 对应大致 0~180 度
//  两种工作模式：
//    use_manual = 1 : 按 manual_us 输出（开环测试用）
//    use_manual = 0 : 输出 1500us + offset_us（闭环用，offset 来自 PID）
//  同时把实际脉宽（微秒）回读给遥测，方便上位机画图
// ============================================================================
`default_nettype none

module servo_pwm #(
    parameter integer CLK_HZ    = 50_000_000,
    parameter integer FRAME_US  = 20_000,   // 20ms => 50Hz
    parameter integer MIN_US    = 500,
    parameter integer MAX_US    = 2500,
    parameter integer CENTER_US = 1500
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire signed [15:0] offset_us,   // 闭环偏移 -1000..+1000
    input  wire [11:0]        manual_us,   // 开环脉宽 500..2500
    input  wire               use_manual,
    output reg                pwm,
    output reg  [11:0]        pulse_us_out
);
    localparam integer TICKS_PER_US = CLK_HZ / 1_000_000;         // 50
    localparam integer FRAME_TICKS  = FRAME_US * TICKS_PER_US;    // 1_000_000

    reg [31:0] frame_cnt;
    reg [31:0] width_ticks;
    reg [11:0] width_us;

    wire signed [31:0] want = CENTER_US + offset_us;

    always @(*) begin
        if (use_manual) begin
            if (manual_us < MIN_US)      width_us = MIN_US;
            else if (manual_us > MAX_US) width_us = MAX_US;
            else                         width_us = manual_us;
        end else begin
            if (want < MIN_US)           width_us = MIN_US;
            else if (want > MAX_US)      width_us = MAX_US;
            else                         width_us = want[11:0];
        end
    end

    always @(*) width_ticks = width_us * TICKS_PER_US;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_cnt     <= 32'd0;
            pwm           <= 1'b0;
            pulse_us_out  <= CENTER_US[11:0];
        end else begin
            if (frame_cnt >= FRAME_TICKS-1)
                frame_cnt <= 32'd0;
            else
                frame_cnt <= frame_cnt + 1'b1;

            pwm          <= (frame_cnt < width_ticks);
            pulse_us_out <= width_us;
        end
    end

endmodule

`default_nettype wire
