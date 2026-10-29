// ============================================================================
//  uart_tx.v  ——  8N1 串口发送（板载 CH340，115200）
//  管脚：FPGA_TXD = 48（手册表 3-13）
// ============================================================================
`default_nettype none

module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       send,     // 单拍脉冲
    output reg        tx,
    output reg        busy
);
    localparam integer DIV = CLK_HZ / BAUD;   // 50MHz/115200 = 434

    reg [15:0] cnt;
    reg [3:0]  bit_i;
    reg [9:0]  sh;          // {stop, data[7:0], start}

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx    <= 1'b1;
            busy  <= 1'b0;
            cnt   <= 16'd0;
            bit_i <= 4'd0;
            sh    <= 10'h3FF;
        end else begin
            if (!busy) begin
                if (send) begin
                    sh    <= {1'b1, data, 1'b0};
                    busy  <= 1'b1;
                    cnt   <= 16'd0;
                    bit_i <= 4'd0;
                end
            end else begin
                if (cnt == DIV-1) begin
                    cnt   <= 16'd0;
                    tx    <= sh[0];
                    sh    <= {1'b1, sh[9:1]};
                    if (bit_i == 4'd9) begin
                        busy <= 1'b0;
                        tx   <= 1'b1;
                    end else begin
                        bit_i <= bit_i + 1'b1;
                    end
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
