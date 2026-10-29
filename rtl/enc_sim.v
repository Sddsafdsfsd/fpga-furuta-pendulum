// ============================================================================
//  enc_sim.v  ——  内部正交编码器信号源（自检用，不需要任何外部硬件）
// ----------------------------------------------------------------------------
//  按 00 -> 01 -> 11 -> 10 -> 00 的顺序输出 A/B，这正是 quad_decoder 认定的"正转"。
//  每个状态持续 STEP_DIV 个时钟，所以：
//      四倍频计数速率 = 50MHz / STEP_DIV     （计数/秒）
//      默认 STEP_DIV = 25000  =>  2000 计数/秒  =>  遥测里 E 每 10ms 增加约 20
//  用途：还没有电机/编码器时，验证四倍频计数、判向、速度计算、遥测链路。
// ============================================================================
`default_nettype none

module enc_sim #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer STEP_DIV   = 25_000,   // 每个状态持续多少个时钟
    parameter integer REV_MS     = 0         // >0 时：每隔这么多毫秒自动反向一次（测判向用）
)(
    input  wire clk,
    input  wire rst_n,
    output reg  a,
    output reg  b
);
    reg [1:0]  state;
    reg [31:0] cnt;
    reg        rev;          // 1 = 反向序列

    // 自动反向定时（可选）
    reg [31:0] ms_cnt;
    reg [15:0] ms;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= 2'd0;
            cnt    <= 32'd0;
            rev    <= 1'b0;
            ms_cnt <= 32'd0;
            ms     <= 16'd0;
        end else begin
            // 状态推进
            if (cnt == STEP_DIV-1) begin
                cnt   <= 32'd0;
                state <= rev ? (state - 1'b1) : (state + 1'b1);
            end else begin
                cnt <= cnt + 1'b1;
            end

            // 自动反向
            if (REV_MS > 0) begin
                if (ms_cnt == (CLK_HZ/1000)-1) begin
                    ms_cnt <= 32'd0;
                    if (ms == REV_MS-1) begin
                        ms  <= 16'd0;
                        rev <= ~rev;
                    end else begin
                        ms <= ms + 1'b1;
                    end
                end else begin
                    ms_cnt <= ms_cnt + 1'b1;
                end
            end
        end
    end

    // 正转序列：00 -> 01 -> 11 -> 10
    always @(*) begin
        case (state)
            2'd0:    begin a = 1'b0; b = 1'b0; end
            2'd1:    begin a = 1'b0; b = 1'b1; end
            2'd2:    begin a = 1'b1; b = 1'b1; end
            default: begin a = 1'b1; b = 1'b0; end
        endcase
    end

endmodule

`default_nettype wire
