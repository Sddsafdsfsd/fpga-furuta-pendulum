// ============================================================================
//  ramp_gen.v  ——  慢速三角波码值发生器（给 DAC-ADC 环回自检用）
// ----------------------------------------------------------------------------
//  默认：每 1ms 走 1 个码值，0 -> 1023 -> 0，全程约 2 秒。
//  串口遥测 100 帧/秒，所以一帧大约能看到码值变化 10 —— 画出来是一条漂亮的斜坡。
//  配合 STAGE=6：DAC1 输出这个斜坡，ADC1 通过 BNC 线读回来，
//  遥测里 M = 送出的码值、A = 读回的码值，两条曲线一比就知道模拟通路对不对。
// ============================================================================
`default_nettype none

module ramp_gen #(
    parameter integer CLK_HZ  = 50_000_000,
    parameter integer STEP_US = 1000,      // 每多少微秒走一步
    parameter integer STEP    = 1,         // 每步变化多少码值
    parameter integer MAX     = 1023
)(
    input  wire       clk,
    input  wire       rst_n,
    output reg [9:0]  code,
    output reg        update
);
    localparam integer DIV = (CLK_HZ/1_000_000) * STEP_US;   // 50000 @1ms

    reg [31:0] cnt;
    reg        dir;      // 0 = 上升，1 = 下降

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt    <= 32'd0;
            code   <= 10'd0;
            dir    <= 1'b0;
            update <= 1'b0;
        end else begin
            update <= 1'b0;
            if (cnt == DIV-1) begin
                cnt    <= 32'd0;
                update <= 1'b1;
                if (dir == 1'b0) begin
                    if (code >= MAX - STEP) begin
                        code <= MAX;
                        dir  <= 1'b1;
                    end else begin
                        code <= code + STEP;
                    end
                end else begin
                    if (code <= STEP) begin
                        code <= 10'd0;
                        dir  <= 1'b0;
                    end else begin
                        code <= code - STEP;
                    end
                end
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
