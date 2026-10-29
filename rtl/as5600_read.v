// ============================================================================
//  as5600_read.v  ——  AS5600 读取（角度 + 磁场诊断 + 无磁铁自检）  v4
//
//  SELFTEST=0  正常工作组：STATUS / AGC / MAGNITUDE / ANGLE
//  SELFTEST=1  自检组（不需要磁铁）：ZPOS / MPOS / MANG / CONF 都有出厂默认值
//         ZPOS   默认 0
//         MPOS   默认 0x0FFF (4095)
//         MANG   默认 0x0FFF (4095)
//         CONF高 默认 0x00（bit11-10=OUTS，00=模拟输出）
//  只要读出的值等于默认值 → I2C 时序/地址/寄存器映射/芯片内部全部正常
//
//  I2C 时钟 CLK_DIV=1250 → 10kHz（PMOD 只有 FPGA 弱上拉，必须慢）
// ============================================================================
`default_nettype none

module as5600_read #(
    parameter integer DEV_ADDR   = 7'h36,
    parameter integer CLK_DIV    = 1250,
    parameter integer PERIOD_1MS = 5,
    parameter integer SELFTEST   = 0,
    parameter integer ENABLE     = 1          // 0 = 停用（避免与新实现争总线）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_1k,
    output reg  [11:0] angle,
    output reg  [11:0] magnitude,
    output reg  [7:0]  status,
    output reg  [7:0]  agc,
    output reg  [11:0] zpos,
    output reg  [11:0] mpos,
    output reg  [11:0] mang,
    output reg  [7:0]  conf_hi,
    output reg         valid,
    output reg         err_flag,
    output wire        scl_oe,
    output wire        sda_oe,
    input  wire        sda_in,
    // ---- 调试探针 ----
    output wire [1:0]  dbg_st,
    output wire [3:0]  dbg_step,
    output wire [3:0]  dbg_i2c_state,
    output wire [1:0]  dbg_i2c_ph
);
    localparam integer NSTEP = (SELFTEST == 1) ? 8 : 6;

    reg         i2c_start;
    reg  [7:0]  i2c_reg;
    wire [7:0]  i2c_data;
    wire        i2c_valid;
    wire        i2c_err;
    wire        i2c_busy;
    // NOTE(P03): the two debug assigns were moved below, after `st`/`step` are
    //            declared.  A forward reference here is exactly the class of bug
    //            that makes a design compile only on tolerant tools.

    i2c_master #(.CLK_DIV(CLK_DIV)) u_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (i2c_start),
        .dev_addr (DEV_ADDR[6:0]),
        .reg_addr (i2c_reg),
        .rd_data  (i2c_data),
        .rd_valid (i2c_valid),
        .err      (i2c_err),
        .busy     (i2c_busy),
        .scl_oe   (scl_oe),
        .sda_oe   (sda_oe),
        .sda_in   (sda_in),
        .dbg_state(dbg_i2c_state),
        .dbg_ph   (dbg_i2c_ph)
    );

    localparam [1:0] S_WAIT = 2'd0, S_REQ = 2'd1, S_GET = 2'd2;

    reg [1:0]  st;
    reg [3:0]  step;
    reg [7:0]  cnt1ms;
    reg [9:0]  gap;          // 事务间隔计数（tBUF）
    reg [7:0]  hi;
    reg        started;

    // ---- P03 fix: these must come AFTER `st` / `step` are declared ----
    assign dbg_st   = st;
    assign dbg_step = step;

    function [7:0] reg_of;
        input [3:0] s;
        begin
            if (SELFTEST == 1) begin
                case (s)
                    4'd0: reg_of = 8'h01;
                    4'd1: reg_of = 8'h02;
                    4'd2: reg_of = 8'h03;
                    4'd3: reg_of = 8'h04;
                    4'd4: reg_of = 8'h05;
                    4'd5: reg_of = 8'h06;
                    4'd6: reg_of = 8'h07;
                    default: reg_of = 8'h08;
                endcase
            end else begin
                case (s)
                    4'd0: reg_of = 8'h0B;
                    4'd1: reg_of = 8'h1A;
                    4'd2: reg_of = 8'h1B;
                    4'd3: reg_of = 8'h1C;
                    4'd4: reg_of = 8'h0E;
                    default: reg_of = 8'h0F;
                endcase
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_WAIT; step <= 4'd0; cnt1ms <= 8'd0; gap <= 10'd0;
            i2c_start <= 1'b0; i2c_reg <= 8'h0B; hi <= 8'd0;
            angle <= 12'd0; magnitude <= 12'd0; status <= 8'd0; agc <= 8'd0;
            zpos <= 12'd0; mpos <= 12'd0; mang <= 12'd0; conf_hi <= 8'd0;
            valid <= 1'b0; err_flag <= 1'b0; started <= 1'b0;
        end else begin
            i2c_start <= 1'b0;
            valid     <= 1'b0;

            case (st)
            S_WAIT: begin
                if (!ENABLE) begin
                    st <= S_WAIT;
                end else if (tick_1k) begin
                    if (cnt1ms + 8'd1 >= PERIOD_1MS) begin
                        cnt1ms <= 8'd0;
                        step   <= 4'd0;
                        st     <= S_REQ;
                    end else begin
                        cnt1ms <= cnt1ms + 8'd1;
                    end
                end
            end

            S_REQ: begin
                if (!started) begin
                    // ★ 关键：等 I2C 主机真正空闲（!busy）再发 start，
                    //   否则上一笔事务还在收尾（NACK/STOP），脉冲会被丢掉 → 死锁
                    if (!i2c_busy) begin
                        // ★ I2C 要求 STOP 后至少 4.7us 总线空闲（tBUF）
                        //   这里给 10us（50MHz 下 500 个时钟）
                        if (gap >= 10'd500) begin
                            i2c_reg   <= reg_of(step);
                            i2c_start <= 1'b1;
                            started   <= 1'b1;
                            gap       <= 10'd0;
                        end else begin
                            gap <= gap + 10'd1;
                        end
                    end else begin
                        gap <= 10'd0;
                    end
                end else if (i2c_valid) begin
                    started <= 1'b0;
                    st      <= S_GET;
                end else if (i2c_err) begin
                    err_flag <= 1'b1;
                    started  <= 1'b0;
                    st       <= S_WAIT;
                end
            end

            S_GET: begin
                if (SELFTEST == 1) begin
                    case (step)
                        4'd0: hi      <= i2c_data;
                        4'd1: zpos    <= {hi[3:0], i2c_data};
                        4'd2: hi      <= i2c_data;
                        4'd3: mpos    <= {hi[3:0], i2c_data};
                        4'd4: hi      <= i2c_data;
                        4'd5: mang    <= {hi[3:0], i2c_data};
                        4'd6: conf_hi <= i2c_data;
                        default: begin
                            valid    <= 1'b1;
                            err_flag <= 1'b0;
                        end
                    endcase
                end else begin
                    case (step)
                        4'd0: status <= i2c_data;
                        4'd1: agc    <= i2c_data;
                        4'd2: hi     <= i2c_data;
                        4'd3: magnitude <= {hi[3:0], i2c_data};
                        4'd4: hi     <= i2c_data;
                        default: begin
                            angle    <= {hi[3:0], i2c_data};
                            valid    <= 1'b1;
                            err_flag <= 1'b0;
                        end
                    endcase
                end

                if (step == NSTEP - 1) begin
                    step <= 4'd0;
                    st   <= S_WAIT;
                end else begin
                    step <= step + 4'd1;
                    st   <= S_REQ;
                end
            end

            default: st <= S_WAIT;
            endcase
        end
    end
endmodule

`default_nettype wire