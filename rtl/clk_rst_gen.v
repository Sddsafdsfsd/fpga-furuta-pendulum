// ============================================================================
//  clk_rst_gen.v  ——  全工程唯一的时钟/复位产生模块
// ----------------------------------------------------------------------------
//  作用：
//    1. 上电后产生约 1.31ms 的复位低电平（50MHz 计 65536 拍）
//    2. 复位【异步置位、同步释放】：释放沿相对 clk_50m 同步，避免亚稳态
//    3. 输出的 rst_n_sync 就是全工程唯一的复位源，模块内部不再自建复位
//
//  ★ 时钟/复位规范（见设计文档 §4）★
//    · 唯一时钟 clk_50m（50MHz），全工程单时钟域
//    · 唯一复位 rst_n_sync，其余模块一律 always @(posedge clk or negedge rst_n)
//    · 禁止门控时钟（clk & en 等）
//
//  ⚠️ 关于上电初值：
//    本模块是全工程的复位源，自身无法使用 rst_n，因此 por_cnt 依赖
//    FPGA 位流的上电寄存器初始化（Gowin 支持，Icarus 支持声明初值）。
//    ★ 这是全工程【唯一】允许依赖上电初值的地方，其余模块禁止 ★
// ============================================================================
`default_nettype none

module clk_rst_gen #(
    parameter integer POR_CYCLES = 65536      // 上电复位拍数（≈1.31ms @50MHz）
)(
    input  wire clk_50m,                      // 50MHz 系统时钟
    output wire rst_n_sync                    // 同步释放后的复位（低有效）
);

    // ---- 上电计数：唯一允许用声明初值的地方（本模块是复位源）----
    reg [15:0] por_cnt = 16'd0;

    wire por_done = (por_cnt >= POR_CYCLES[15:0] - 16'd1);

    always @(posedge clk_50m) begin
        if (!por_done)
            por_cnt <= por_cnt + 16'd1;
        // 计满后保持，不再自增（避免回绕重新复位）
    end

    // ---- 复位两级同步：异步置位、同步释放 ----
    //   por_done 上升沿作为"释放请求"，经两级触发器同步到 clk_50m 域
    reg [1:0] rst_sync;

    always @(posedge clk_50m or negedge por_done) begin
        if (!por_done)
            rst_sync <= 2'b00;                // 异步置位：计数器没满 → 复位有效
        else
            rst_sync <= {rst_sync[0], 1'b1};  // 同步释放：两拍后 rst_n_sync=1
    end

    assign rst_n_sync = rst_sync[1];

endmodule

`default_nettype wire
