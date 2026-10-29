// ============================================================================
//  top.v  ——  PocketLab-F1 (GW1N-9 / EQFP144) 姿态控制工程
//  2026 全国大学生嵌入式芯片与系统设计竞赛 · FPGA 创新设计赛道（高云）
//  选题一：基于 FPGA 的实时姿态控制系统
// ----------------------------------------------------------------------------
//  设计要点
//    * 单一 50MHz 时钟域；所有节拍用"使能脉冲"产生，不生成派生时钟
//    * 控制节拍 1kHz：ADC 平均 -> 双环 PID -> PWM
//    * 遥测：每 10ms 一行 ASCII（115200-8-N-1）
//    * STAGE 参数切换验证阶段，不必改代码结构
//
//  分阶段（改 STAGE 重新综合即可）
//    STAGE=1  1Hz 灯闪                     —— 验证工具链/下载/时钟/LED
//    STAGE=2  ADC -> LED                   —— 验证 ADC 与电位器接线
//    STAGE=3  舵机开环扫掠                  —— 验证舵机 PWM 与供电
//    STAGE=4  串口遥测 + 舵机 PID 闭环        —— 单轴闭环练习
//    STAGE=5  编码器 + TB6612 + 双环 PID      —— 倒立摆完整控制链路（默认）
//    STAGE=6  DAC1 -> ADC1 环回自检           —— 用配套 BNC 线验证两套模拟通路
//    STAGE=7  AS5600 I2C 角度                  —— 磁编码器联调（诊断字段 T/E/M）
//    STAGE=8  电机开环占空比测试               —— P<占空比>，2 秒看门狗自动停
//    STAGE=9  引脚电平探测（找线插在哪个脚）    —— LED/A 显示 80~87 电平
//    STAGE=10 AS5600 无磁铁自检                 —— 读出厂默认寄存器对照
//    STAGE=11 I2C 线路诊断                      —— A=sda_in 电平 T=完成轮数 M=错误
//    STAGE=12 I2C 状态机探针                    —— A=i2c状态 T=读状态 E=step M=错误
//    STAGE=13 开源 I2C 实现读角度               —— A=角度(0~4095) T=读取次数
//    STAGE=14 I2C 地址扫描                      —— A=当前地址 T=应答个数 E=首个应答地址
//    STAGE=15 读 STATUS 寄存器验证读通路          —— A=STATUS(应≈23) T=读取次数
//    STAGE=16 磁铁摆放指导                        —— A=AGC T=STATUS E=MAGNITUDE M=角度
//    STAGE=17 串口命令诊断                        —— A=收字节数 T=motor_en E=末字节ASCII M=tgt_pos
//    STAGE=18 自动循环电机测试                    —— A=秒计数 T=- E=- M=实际控制量(±300/0)
//    STAGE=19 磁场质量诊断(快扫)                   —— A=ANGLE T=AGC E=MAGNITUDE M=STATUS
//              判据：3.3V 供电时 AGC 满量程只到 128，AGC=128 表示增益到顶=磁场太弱；
//                    MAGNITUDE 正常 1000~2500，≈1 表示芯片几乎测不到磁场。
//    STAGE=20 读通路/芯片身份自检                   —— 读的是出厂恒定值，
//              正常应恒为 A4095 T0255 E+4095 M+0255（MPOS=0xFFF, MANG=0xFFF）
//              ★ 这是判"读通路是否干净"最硬的一个实验：读的是常量，与磁场/机械无关
//    STAGE=36 设定点自检                            —— A=ang_f[11:2] T=tgt12[11:2] E=enc M=cmd
//    STAGE=37 PD 分项诊断                           —— A=a_err+2048 T=a_rate+512 E=e_rate M=cmd
//    STAGE=38 原始 vs 滤波 + 速率（v2.3）            —— A=ang_f  T=rps  E=ang12  M=nack/秒
//    STAGE=39 接受次数（v2.3，新增）                 —— A=af_upd/秒 T=rps E=ang12  M=nack/秒
//              ★ 38/39 是判决实验：一次捕获分开"读通路丢"和"滤波器吃"。
//                采集时必须手动缓慢、大范围转摆杆（覆盖 0~4095），
//                摆杆不动时 E 本来就不变，这样采出来的变化率没有意义。
//
//  STAGE=5 的接线（全部走 12 脚 PMOD，占用 GPIO_82~87）
//    PMOD IO3 (GPIO_82) -> TB6612 PWMA
//    PMOD IO4 (GPIO_83) -> TB6612 AIN1
//    PMOD IO5 (GPIO_84) -> TB6612 AIN2
//    PMOD IO6 (GPIO_85) -> TB6612 STBY
//    PMOD IO7 (GPIO_86) -> 编码器 A 相
//    PMOD IO8 (GPIO_87) -> 编码器 B 相
//    注意：这 6 个脚本来是 SPI LCD 的，所以这一版**不能插 LCD 屏**；
//          舵机口(GPIO_80) 在 STAGE5 只输出中位脉冲。
// ============================================================================
`default_nettype none

module top #(
    parameter integer STAGE          = 5,          // ★ v2.1 默认改成 5（闭环倒立摆）
                                                   //   原来默认是 8（开环占空比测试）
    parameter integer LED_ACTIVE_LOW = 0,          // ★ 已实证：共阴极、高电平点亮（手册 3.2.6）
                                                   //   证据：STAGE=8 时 enc_pos=0 → led_val=0，
                                                   //   取反后 led=0xFF，8 个灯实测全亮 ⇒ 引脚高=点亮
                                                   //   ⇒ 必须为 0（取反会得到相反的显示语义）
    parameter integer OE_ACTIVE_LOW  = 1,          // 3PA1030 输出使能极性

    // ---- STAGE<=4：单轴（舵机）练习用 PID，Q8 定点 ----
    parameter signed [15:0] KP_Q8      = 16'sd512,   // Kp = 2.0
    parameter signed [15:0] KI_Q8      = 16'sd8,     // Ki = 0.031
    parameter signed [15:0] KD_Q8      = 16'sd1024,  // Kd = 4.0
    parameter integer       OUT_MAX_US = 1000,

    // ---- STAGE=5：双环 PID，Q8 定点 ----
    parameter signed [15:0] AKP_Q8     = 16'sd128,   // 倾角环 Kp = 2.0
    parameter signed [15:0] AKI_Q8     = 16'sd4,     // 倾角环 Ki = 0.016
    parameter signed [15:0] AKD_Q8     = 16'sd0,  // 倾角环 Kd = 4.0
    parameter signed [15:0] PKP_Q8     = 16'sd512,    // 位置环 Kp = 0.25
    parameter signed [15:0] PKI_Q8     = 16'sd1,     // 位置环 Ki = 0.004
    parameter signed [15:0] PKD_Q8     = 16'sd1024,   // 位置环 Kd = 1.0
    parameter integer       LOOP_MODE  = 1,          // 1=串级（推荐） 0=并联叠加（J280 手册图）

    // ---- 编码器 ----
    parameter integer ENC_SIM          = 0,          // 1=用内部自检信号源（无需硬件） 0=用真实 A/B
    parameter integer ENC_INVERT       = 1,          // 四倍频实测：+600(逆时针)时 E 减少 → 翻转为增加 ✓
    parameter integer SELFTEST_MODE    = 0,
    parameter integer I2C_OLD_EN      = 0,          // 0=停用旧的 I2C 实现（改用 FpOC 开源实现）
    parameter integer ENC_MODE         = 0,          // 0=四倍频(A+B)  1=单相计数(只用A，B相损坏时用)

    // ---- STAGE=6 的 DAC 波形选择 ----
    parameter integer DAC_WAVE         = 0           // 0=慢速三角波  1=慢速方波(5秒一档)
)(
    input  wire        clk_50m,     // 50MHz 有源晶振，管脚 11
    // ---- ADC1 (3PA1030) ----
    output wire        ad_clk,      // 管脚 32
    output wire        ad_oe,       // 管脚 15
    input  wire        ad_ovr,      // 管脚 34（本版未用，保留诊断）
    input  wire [9:0]  ad_d,        // D0..D9 = 管脚 47,46,45,44,43,42,41,40,39,38
    // ---- 舵机（管脚 80）----
    output wire        servo_pwm,
    // ---- 板载 CH340 串口 ----
    output wire        uart_tx,     // 管脚 48
    input  wire        uart_rx,     // 管脚 49
    // ---- 8 个蓝色 LED（管脚 23~30）----
    output wire [7:0]  led,
    // ---- 编码器 A/B（PMOD IO7/IO8 = 管脚 86/87）----
    input  wire        enc_a,
    input  wire        enc_b,
    // ---- TB6612 电机驱动（PMOD IO3~IO6 = 管脚 82~85）----
    output wire        mot_pwm,
    output wire        mot_ain1,
    output wire        mot_ain2,
    inout  wire        mot_stby,    // LCD TFT_BL: left high-Z, STBY hardwired to 3V3 on the module
    // ---- DAC1（3PD5651E，BNC 输出，用于模拟环回自检）----
    output wire [9:0]  dac_d,        // DATA0..9 = 管脚 124,126,128,129,130,131,132,133,134,135
    output wire        dac_clk_out,  // 管脚 136
    // ---- AS5600 I2C（SCL = GPIO_90，SDA = GPIO_88）----
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,
    // ---- 引脚探测（STAGE=9：把 80~85 配成输入，用来找出线插在哪个脚）----
    input  wire [5:0]  probe,
    // ---- 板上按键 K1~K7（管脚 58~64，按下为低电平）----
    input  wire [6:0]  key,
    // ---- 4 位数码管（共阳极，段选 a~g,dp + 位选 1~4，低电平有效）----
    output wire [7:0]  smg,
    output wire [3:0]  dig
);

    // ==================================================================
    //  AS5600 读取时序常量
    //  （必须放在这里：ANG_MAX_JUMP 要按实际采样周期折算，而不能写死）
    //
    //  一次事务 = 状态机 cnt 从 0 走到 204 再 done ≈ 205 个 epoch，
    //  每个 epoch = CLK_DIV 个 50 MHz 时钟。
    //       CLK_DIV=1000 -> 205,000 拍 =  4.10 ms -> 244 Hz
    //       CLK_DIV=4000 -> 820,000 拍 = 16.40 ms ->  61 Hz    <- 与实测 T=61 吻合
    //
    //  ★ 陷阱：4*CLK_DIV 是【SCL 的周期】，不是事务长度。
    //     CLK_DIV=4000 -> SCL = 50M/(4*4000) = 3.1 kHz（这就是"3.1 kHz 那档"的由来）。
    //     如果按 4*204*CLK_DIV 算采样周期，采样周期会被放大 4 倍，
    //     ANG_MAX_JUMP 跟着放大 4 倍 -> 跳变剔除形同虚设。
    // ==================================================================
    parameter integer OS_CLKDIV    = 1000;      // 1000=12.5 kHz SCL/244 Hz 读  4000=3.1 kHz/61 Hz 读
    localparam integer SAMPLE_CLK  = 205 * OS_CLKDIV;   // 一次事务的时钟数

    // 允许的最大角速率，单位 counts/ms（1 count = 4096/360 = 0.0879 度）
    //   17 counts/ms = 17,000 counts/s = 1495 度/s（摆杆在平衡附近的物理上限，留 3 倍余量）
    parameter integer ANG_RATE_CPMS = 17;
    //   61 Hz: 17 * 820000 / 50000 = 278 counts/样本
    //  244 Hz: 17 * 205000 / 50000 =  69 counts/样本
    //  （先乘后除，17*820000 = 1.39e7，远不到 32 位溢出）
    //
    // ★ 两个"强制覆盖"，只为了做单变量 A/B 对照（0 = 用自动算出的值）：
    //   想验证"放宽阈值到底修好了还是把坏值放进来了"，必须让前后两版固件
    //   只差这一个参数，其它一个字都不变。把旧值 60/15 塞回去就得到对照组。
    //   平时一律留 0。
    parameter integer ANG_JUMP_FORCE = 0;
    parameter integer ANG_REJ_FORCE  = 0;
    localparam integer JUMP_AUTO = (ANG_RATE_CPMS * SAMPLE_CLK) / 50_000;
    localparam [11:0] ANG_MAX_JUMP = (ANG_JUMP_FORCE > 0) ? ANG_JUMP_FORCE : JUMP_AUTO;
    localparam [ 4:0] ANG_REJ_MAX  = (ANG_REJ_FORCE  > 0) ? ANG_REJ_FORCE  : 5'd3;

    // 三取中值旁路：1 = 直接用原始值，跳过中值滤波。
    //   中值-3 的代价是恒定 1 个采样周期的群延迟（61 Hz 下 16.3 ms，
    //   正好占掉总延迟预算 33 ms 的一半）。它存在的唯一理由是挡 12.5 kHz
    //   下的位错误；3.1 kHz 下如果实测没有野值，这 16.3 ms 是白付的。
    //   由 build_pd.ps1 -MedianBypass 1 注入。
    parameter integer MEDIAN_BYPASS = 0;

    // ------------------------------------------------------------------
    // 上电复位：约 1.3ms 低电平
    // ------------------------------------------------------------------
    reg [15:0] por_cnt = 16'd0;
    reg        rst_n   = 1'b0;

    always @(posedge clk_50m) begin
        if (por_cnt != 16'hFFFF) begin
            por_cnt <= por_cnt + 1'b1;
            rst_n   <= 1'b0;
        end else begin
            rst_n   <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // 1kHz 控制节拍
    // ------------------------------------------------------------------
    reg [15:0] tick_cnt = 16'd0;
    reg        tick_1k  = 1'b0;

    always @(posedge clk_50m) begin
        if (tick_cnt == 16'd49_999) begin
            tick_cnt <= 16'd0;
            tick_1k  <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick_1k  <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // ADC 采集（3PA1030，25MHz 采样，每 1ms 平均 1024 个样本）
    // ------------------------------------------------------------------
    wire [9:0] adc_val;

    // ==================================================================
    //  前向声明区（P03 修复）
    //  原代码里下列信号在【声明之前】就被使用。Gowin 综合器能容错，
    //  但 IEEE 1364 要求先声明后使用，iverilog / Verilator / VCS 会直接
    //  报错；若某工具按隐式 1 位网线处理，连接会静默变成 1 位。
    //  这里统一提前声明，下面原有的重复声明已注释掉。
    // ==================================================================
    wire [15:0]        os_regout;                                       // 原 515 行
    wire               os_scl, os_done, os_ready;                       // 原 514 行
    wire               os2_scl, os2_done;                               // 原 604 行
    wire               scan_busy, scan_err, scan_scl_oe, scan_sda_oe;   // 原 546 行
    wire               fld_scl_st, fld_scl_ag, fld_scl_mg, fld_scl_an;  // 原 644 行
    // STAGE=16/19 磁铁诊断用的四个读取器（声明提到这里：STAGE=19 的排队逻辑
    // 在实例之前就要用到 done/regout，原代码在实例前才声明会被 iverilog 判为前向引用）
    wire               fld_dn_st, fld_dn_ag, fld_dn_mg, fld_dn_an;      // 原 829 行
    wire [15:0]        fld_ro_st, fld_ro_ag, fld_ro_mg, fld_ro_an;      // 原 830 行
    wire               fld_er_st, fld_er_ag, fld_er_mg, fld_er_an;      // NACK 标志
    // ---- AS5600 磁场诊断阶段的公共常量（STAGE=19/20/21/22）----
    //   19：读 ANGLE/AGC/MAGNITUDE/STATUS            12.5 kHz
    //   20：读【出厂恒定值】寄存器 0x03/0x04/0x05/0x06  12.5 kHz
    //   21：同 20，但 I2C 降到 3.1 kHz（CLK_DIV 1000->4000）
    //   22：同 19，但 I2C 降到 3.1 kHz
    //   用途：12.5 kHz 下 25% 的读数是坏的（SDA 上升沿来不及），
    //         只有 3.1 kHz 才 100% 正确 —— 必须用 21/22 才能拿到可信数据。
    localparam FLD_FIELD  = (STAGE == 19) || (STAGE == 22);
    localparam FLD_CONST  = (STAGE == 20) || (STAGE == 21) || (STAGE == 23);
    localparam FLD_DIAG   = FLD_FIELD || FLD_CONST;
    localparam [ 7:0] FLD_RA_ST = FLD_CONST ? 8'h03 : 8'h0B;
    localparam [ 7:0] FLD_RA_AG = FLD_CONST ? 8'h04 : 8'h1A;
    localparam [ 7:0] FLD_RA_MG = FLD_CONST ? 8'h05 : 8'h1B;
    localparam [ 7:0] FLD_RA_AN = FLD_CONST ? 8'h06 : 8'h0E;
    // 23 = 常量寄存器自检 @ 6.25 kHz，用来找“还能正确读的最快时钟”
    localparam [15:0] FLD_CLKDIV = (STAGE == 19) ? 16'd1000 :
                                   (STAGE == 23) ? 16'd2000 : 16'd4000;
    wire               auto_en;                                         // 原 710 行
    wire signed [15:0] auto_cmd;                                        // 原 713 行
    wire signed [15:0] motor_cmd_src;                                   // 原 314 行
    wire               mot_en;      // v2.1: 前向声明（pd_ctrl 实例要用到它）

    adc_3pa1030 #(
        .OE_ACTIVE_LOW (OE_ACTIVE_LOW),
        .CLK_DIV       (1),
        .AVG_SHIFT     (10),
        .SAMPLE_DLY    (1)
    ) u_adc (
        .clk         (clk_50m),
        .rst_n       (rst_n),
        .tick_1k     (tick_1k),
        .ad_d        (ad_d),
        .ad_ovr      (ad_ovr),
        .ad_clk      (ad_clk),
        .ad_oe       (ad_oe),
        .value       (adc_val),
        .value_valid ()
    );

    // ------------------------------------------------------------------
    // 串口接收 + 指令解析（S 角度 / P 位置 / E 使能）
    // ------------------------------------------------------------------
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(
        .CLK_HZ (50_000_000),
        .BAUD   (115200)
    ) u_rx (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .rx    (uart_rx),
        .data  (rx_data),
        .valid (rx_valid)
    );

    wire        [9:0]        tgt_angle;
    wire signed [15:0]       tgt_pos;
    wire                     motor_en;

    wire home_pulse;   // 'H' command: latch current position as the absolute zero
    wire z_capture;    // 'Z' command: capture current angle as the balance point
                       // ★ v2.1: this output existed in cmd_parser from day one
                       //    but was NEVER CONNECTED. 'Z' therefore did nothing
                       //    and tgt_angle stayed at DEF_ANGLE=512 while the rod
                       //    physically balances at A~=750, i.e. a permanent 83
                       //    degree error.  That alone saturates any controller.
    cmd_parser #(
        .DEF_ANGLE (512),
        .ANGLE_MAX (1023),
        .POS_LIMIT (9999)
    ) u_cmd (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        .rx_data      (rx_data),
        .rx_valid     (rx_valid),
        .angle_target (tgt_angle),
        .pos_target   (tgt_pos),
        .motor_en     (motor_en),
        .capture_pulse(z_capture),
        .clear_pulse  (),
        .home_pulse   (home_pulse)
    );

    // ------------------------------------------------------------------
    // 编码器信号源：默认内部自检，接真编码器时把 ENC_SIM 改成 0
    // ------------------------------------------------------------------
    wire sim_a, sim_b;

    enc_sim #(
        .CLK_HZ   (50_000_000),
        .STEP_DIV (25_000),      // 2000 计数/秒
        .REV_MS   (0)            // >0 会周期性反向，用来验证判向
    ) u_enc_sim (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .a     (sim_a),
        .b     (sim_b)
    );

    wire enc_a_use = ENC_SIM ? sim_a : enc_a;
    wire enc_b_use = ENC_SIM ? sim_b : enc_b;

    wire signed [15:0] enc_pos_q, enc_pos_s;
    wire signed [15:0] enc_pos_raw = (ENC_MODE == 1) ? enc_pos_s : enc_pos_q;
    // 'H' command: latch the current physical position as the absolute zero point.
    reg  signed [15:0] enc_off = 16'sd0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)          enc_off <= 16'sd0;
        else if (home_pulse) enc_off <= enc_pos_raw;
    end
    wire signed [15:0] enc_pos = enc_pos_raw - enc_off;
    wire signed [15:0] enc_speed;
    wire               enc_dir;

    quad_decoder #(
        .CNT_BITS (16),
        .FILTER_N (64),
        .INVERT   (ENC_INVERT)
    ) u_quad (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .tick_1k (tick_1k),
        .a_in    (enc_a_use),
        .b_in    (enc_b_use),
        .pos     (enc_pos_q),
        .speed   (enc_speed),
        .dir     (enc_dir)
    );

    // ---- 单相计数（B 相损坏时的备用方案：ENC_MODE=1 时生效）----
    enc_single #(
        .CNT_MAX  (20000)
    ) u_enc_single (
        .clk    (clk_50m),
        .rst_n  (rst_n),
        .a_in   (enc_a_use),
        .cmd     (motor_cmd_src),
        .pos    (enc_pos_s)
    );

    // ------------------------------------------------------------------
    // AS5600 角度跳变剔除（I2C 位错误的兜底）
    //   实测：轴静止时约 16% 的读数出现大幅度错值（0xB17→0xB00 / 0xB17→0x81F
    //   这种位翻转），而本板 SCL/SDA 只有 FPGA 内部 ~50k 上拉，偏弱。
    //   规则：与上次接受值的差 <= MAX_JUMP 才接受；
    //         连续 REJ_MAX 次被拒 ⇒ 认为是真的快速转动，强制接受（防止锁死）。
    //   采样率约 250Hz（os_done 每次读数完成一个脉冲），滞后 1 个样本 ≈ 4ms。
    // ------------------------------------------------------------------
    // ★ v2.3：ANG_MAX_JUMP / ANG_REJ_MAX 已移到文件开头，按 OS_CLKDIV 自动折算。
    //   旧值是写死的 12'd60 —— 那是按 250 Hz 采样算的（允许 1330 度/s），
    //   但在 61 Hz 下 60 counts/样本只有 322 度/s，摆杆一松手就超限。
    //   同一个常数在两种速率下含义差 4 倍，这是必须折算而不能写死的直接原因。

    wire [11:0] ang_raw  = os_regout[11:0];

    // ---- 注：曾经在这里加过"0x000/0xFFF 一律丢弃"，用来对付一颗损坏的
    //   AS5600。换新芯片后必须拆掉 —— 角度经过 0/4095 的绕回点时，真实读数
    //   本来就长这样，把它判成坏值等于在绕回点整段丢数据。
    //   现在改由"三取中值 + 环形跳变距离"来挡坏值：单点坏值被中值滤掉，
    //   连坏两拍才会进来，也会被环形距离挡下。

    // ---- 第一级：三取中值（抓单点坏值，不依赖幅度）----
    //   实测错值既有大幅度位翻转（0xB17→0x81F），也有只差 51 counts 的（0xB17→0x2F8），
    //   后者在跳变阈值之内、只能靠中值滤掉。
    reg [11:0] mr0 = 12'd0, mr1 = 12'd0, mr2 = 12'd0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            mr0 <= 12'd0; mr1 <= 12'd0; mr2 <= 12'd0;
        end else if (os_done) begin
            mr2 <= mr1;
            mr1 <= mr0;
            mr0 <= ang_raw;
        end
    end

    wire [11:0] mxa = (mr0 > mr1) ? mr0 : mr1;
    wire [11:0] mna = (mr0 > mr1) ? mr1 : mr0;
    wire [11:0] ang_med_raw = (mxa > mr2) ? ((mna > mr2) ? mna : mr2) : mxa;

    // ★ v2.3：MEDIAN_BYPASS=1 时旁路中值，省掉那 1 个采样周期的群延迟。
    //   下游全部用 ang_med，所以这里换个来源就够，其它逻辑一行不用改。
    wire [11:0] ang_med = MEDIAN_BYPASS ? ang_raw : ang_med_raw;

    // ---- 第二级：跳变剔除（抓连续坏值 / 上电首次锁定）----
    reg [11:0] ang_f   = 12'd0;
    reg [4:0]  ang_rej = 5'd0;
    reg        ang_init = 1'b0;

    // 环形跳变距离：直接用减法在绕回点会得到 ~4095（例：4090 → 10 差 -4080），
    // 会被误判成跳变、导致角度卡在 0/4095 处不动。必须取环形最短距离。
    // ★ v2.3：原来写的是 (12'd4096 - ang_d12) —— 4096 装不进 12 位常量，会被截断成 0，
    //   靠补码回绕"碰巧算对"。现在写成 (12'd0 - ang_d12)，语义明确、也不再触发截断告警。
    wire [11:0] ang_d12  = ang_med - ang_f;                       // mod 4096
    wire [11:0] ang_diff = (ang_d12 >= 12'd2048) ? (12'd0 - ang_d12) : ang_d12;

    // ---- "本次采样被接受"的脉冲（供 STAGE=39 统计）----
    //   必须写成组合逻辑：os_done 只有 1 拍，用寄存器存 ang_acc 会和 os_done 错开一拍。
    wire ang_first  = os_done && !ang_init;
    wire ang_accept = os_done &&  ang_init &&
                      ((ang_diff <= ANG_MAX_JUMP) || (ang_rej >= ANG_REJ_MAX));
    wire ang_acc    = ang_first || ang_accept;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ang_f    <= 12'd0;
            ang_rej  <= 5'd0;
            ang_init <= 1'b0;
        end else if (!ang_init) begin
            if (os_done) begin
                ang_f    <= ang_med;          // 首样本直接接受
                ang_init <= 1'b1;
                ang_rej  <= 5'd0;
            end
        end else if (os_done) begin
            if ((ang_diff <= ANG_MAX_JUMP) || (ang_rej >= ANG_REJ_MAX)) begin
                ang_f   <= ang_med;
                ang_rej <= 5'd0;
            end else begin
                ang_rej <= ang_rej + 5'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // 控制环的角度来源：
    //   STAGE=5（倒立摆）：用 AS5600 的 12 位角度（经跳变剔除），取高 10 位
    //                      （1024 格，0.35°/格），与 S 命令的 10 位目标值量纲一致
    //   其他阶段：沿用 ADC（电位器/模拟输入）
    // ------------------------------------------------------------------
    wire [9:0] angle_src = ((STAGE == 5) || (STAGE == 34)) ? ang_f[11:2] : adc_val;

    // ------------------------------------------------------------------
    //  ★ v2.1 平衡点（12 位）—— 修 'Z' 命令从来没接上的问题
    //     tgt12 有三个来源，后写者优先：
    //        复位      -> 3000（≈ A=750，摆杆竖直的实测值）
    //        'Z'       -> 捕获当前 ang_f 的 12 位值（真正把"竖直"设为平衡点）
    //        'S<n>'    -> 10 位手输，左移 2 位成 12 位（Z 失效时的手动兜底）
    //     控制器吃 12 位（0.088°/count），遥测 A 仍按 10 位上报，格式不变。
    //
    //  ★ 捕获保护：只接受离竖直 3000 不到 900 counts（79°）的捕获。
    //    实测教训（2026-10-20）：开串口时 RX 上一个杂散字节被判成 'Z'，
    //    而摆杆当时停在绕回缝附近（ang12≈4090），目标就被改成 4090，
    //    与竖直差 1090 counts ≈ 96° —— 这种目标一旦 E1 就是必然撞限位。
    //    加这道判断后，离谱的捕获直接丢弃、tgt12 保持原值；
    //    症状很直观：发 Z 之后 T 不跟着变。
    //    要手动指定任意目标仍然可以发 S<n>。
    // ------------------------------------------------------------------
    // ------------------------------------------------------------------
    //  ★ 2026-10-21：传感器实测输出窗口只有 240°（标度 1.5 倍），
    //    电磁铁已转 90°，倒立位从 0 移到 ≈1400。据此改了三处：
    //      · 默认目标 3000 -> 1400（3000 现在落在饱和区）
    //      · Z 守卫改成"离量程两端太近才拒绝"，原来要求"离 3000 近"
    //        ——那个条件会把新的倒立位 1400 直接拒掉
    //      · 预设 P0 的角度增益按 1/1.5 缩放（707->472, 1963->1309）；
    //        P3 保留原来未缩放的 707/1963，留作 A/B 对照
    // ------------------------------------------------------------------
    reg  [11:0] tgt12    = 12'd3148;   // 2026-10-24 实测：下垂 A=275 + 半圈 512 = 787 => 787 x 4 = 3148
    reg  [9:0]  tgt10_seen = 10'd512;

    // 平衡点必须远离 0/4095 —— 那两端正是传感器的饱和/绕回缝。
    // 只判"离两端够不够远"，不再绑定某个特定角度值。
    // ? v37(2026-10-24):? AS5600 ???????????(?: ??1472???4095 ??;
    //   ?: ??3000???1020 ??)???? ang_f ?? a_err ???? ? ??? ? ??????
    //   ?????? 3000 ??????,??????(??? 3000 / ?? / Z)?????
    wire [12:0] ang_rev  = 13'd6000 - {1'b0, ang_f};
    wire [11:0] ang_ctrl = ang_rev[11:0];
    wire        z_ok   = (ang_f > 12'd2600) && (ang_f < 12'd3400);   // 收窄：3894 已是钳位边沿

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tgt12      <= 12'd3148;
            tgt10_seen <= 10'd512;
        end else if (z_capture) begin
            tgt10_seen <= tgt_angle;
            if (z_ok) tgt12 <= ang_f;         // 离谱的捕获丢弃，保持原目标
        end else if (tgt_angle != tgt10_seen) begin
            tgt10_seen <= tgt_angle;          // 'S<n>'
            tgt12      <= {tgt_angle, 2'b00};
        end
    end

    // STAGE=5：双环 PID -> TB6612 -> 直流电机
    // ------------------------------------------------------------------
    wire signed [15:0] dual_cmd;
    wire signed [15:0] angle_term;
    wire signed [15:0] pos_term;

    dual_loop_ctrl #(
        .AKP_Q8      (AKP_Q8),
        .AKI_Q8      (AKI_Q8),
        .AKD_Q8      (AKD_Q8),
        .ANG_OUT_MAX (400),
        .PKP_Q8      (PKP_Q8),
        .PKI_Q8      (PKI_Q8),
        .PKD_Q8      (PKD_Q8),
        .POS_OUT_MAX (500),
        .LOOP_MODE   (LOOP_MODE),
        .CMD_MAX     (1000)
    ) u_dual (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        .tick_1k      (tick_1k),
        .angle        (angle_src),
        .angle_target (tgt_angle),
        .pos          (enc_pos),
        .pos_target   (tgt_pos),
        .cmd          (dual_cmd),
        .angle_term   (angle_term),
        .pos_term     (pos_term)
    );

    // ------------------------------------------------------------------
    //  ★ v2.1  透明 PD 控制器（替代 dual_loop_ctrl 黑盒）
    //    增益以"预设组"形式给出，用 P0..P7 在运行中切换，**不需要重新综合**。
    //    P 在 STAGE=5 里没有别的用途（pos_target 只被 dual_loop_ctrl 使用），
    //    所以借用它是安全的，也不必改 cmd_parser。
    //    所有数值 = Q8 整数；括号里是物理值 (duty/rad, duty/(rad/s))。
    // ------------------------------------------------------------------
    wire [2:0] pre_sel = (tgt_pos >= 16'sd0 && tgt_pos <= 16'sd7)
                       ? tgt_pos[2:0] : 3'd0;

    reg signed [17:0] sel_kp_a, sel_kd_a, sel_kp_e, sel_kd_e;
    reg signed [10:0] sel_ff;

    always @(*) begin
        case (pre_sel)
            // ★ 2026-10-21：实测 AS5600 输出窗口 240°（标度 1.5 倍），
            //   角度通道的 counts/度 是设计值的 1.5 倍 ⇒ 等效环路增益高 1.5 倍。
            //   所有预设的角度增益（kp_a / kd_a）已按 1/1.5 缩放回标称值；
            //   编码器通道（kp_e / kd_e，1060 counts/rev）不受影响，保持原值。
            //   ★ P3 故意保留未缩放的 707/1963，作为"缩放前后"的 A/B 对照。
            // P0 默认：仿真最终选中（5° 起步、参数 ±30%、1 count 噪声、16 ms 窗口）
            //   标称 Kp=1800 Kd=80 Kpa=150 Kda=38 ff=200
            default: begin sel_kp_a = 18'sd635; sel_kd_a = 18'sd1760;
                       sel_kp_e = 18'sd228;  sel_kd_e = 18'sd3604; sel_ff = 11'sd200; end
            // P1 Kp 提一档（标称 2400）
            3'd1: begin sel_kp_a = 18'sd628;  sel_kd_a = 18'sd1760;
                       sel_kp_e = 18'sd228;  sel_kd_e = 18'sd3604; sel_ff = 11'sd170; end
            // P2 Kp=3200 Kd=45
            3'd2: begin sel_kp_a = 18'sd838;  sel_kd_a = 18'sd736;
                       sel_kp_e = 18'sd91;   sel_kd_e = 18'sd1423; sel_ff = 11'sd170; end
            // P3 ★★ 严格单变量对照：与 P0 只差 kp_a/kd_a（707/1963 vs 472/1309）
            //   注意：原来这里 kp_e/kd_e 是 91/1423，与 P0 的 228/3604 不同
            //   ⇒ 同时改了 4 个增益，任何 P0↔P3 差异都无法归因于缩放。已修正。
            //   P3 只用于验证"未缩放会失稳"，不要用于正式跑。
            3'd3: begin sel_kp_a = 18'sd707;  sel_kd_a = 18'sd1963;
                       sel_kp_e = 18'sd228;  sel_kd_e = 18'sd3604; sel_ff = 11'sd200; end
            // P4 很保守（标称 Kp=1400 Kd=30）
            3'd4: begin sel_kp_a = 18'sd367;  sel_kd_a = 18'sd491;
                       sel_kp_e = 18'sd91;   sel_kd_e = 18'sd1423; sel_ff = 11'sd170; end
            // P5 最快（标称 Kp=4200 Kd=45）
            3'd5: begin sel_kp_a = 18'sd1099; sel_kd_a = 18'sd736;
                       sel_kp_e = 18'sd91;   sel_kd_e = 18'sd1423; sel_ff = 11'sd140; end
            // P6 前馈偏小（ff=120）
            3'd6: begin sel_kp_a = 18'sd635;  sel_kd_a = 18'sd1760;
                       sel_kp_e = 18'sd228;  sel_kd_e = 18'sd3604; sel_ff = 11'sd120; end
            // P7 前馈偏大（ff=220）
            3'd7: begin sel_kp_a = 18'sd635;  sel_kd_a = 18'sd1760;
                       sel_kp_e = 18'sd228;  sel_kd_e = 18'sd3604; sel_ff = 11'sd220; end
        endcase
    end

    // ★ v3.2（2026-10-21）—— 倾斜级联（tilt cascade），复核者提出、我补符号验证
    //
    //   问题：kp_e 是臂位置的【正反馈】（e_pos>0 的正命令把臂继续推向 +E），
    //   所以 E>224 之后外推项 0.891E 永久大于软限位偏置 200，臂只会单向爬。
    //   实测：v30 臂以 +67 counts/s 爬，8.3 秒撞 450 被抛弃（摆杆当时差 0.4°）。
    //
    //   倾斜级联：臂偏了，就悄悄把摆杆的平衡点往那边挪一点，让【摆杆自己】
    //   把臂带回来 —— 一个 duty 都不花在执行器上。
    //   等效回中 = 0.15 × kp_a × 4 / 256 = 1.11 duty/count，
    //   与 kp_e 的 +0.891 相抵后净 −0.209E − 200 ⇒ 任意 E 都回中。
    //   |E|=450 时净回中 −294 duty，是静摩擦 150 的两倍。
    // v3.4：g 从 38/256=0.148 提到 124/256=0.484 —— 解析解 g = kp_e/kp_a = 228/472 = 0.483
    //   净直流力 = (0.891 - 1.844·g)·E - 200，这个 g 让 E 的一次项正好归零
    //   ⇒ 臂在 |E|<186 内没有任何净推力，漂移率 30 -> 0
    //   上限 ±90 counts 不变 ⇒ 摆杆最多被要求偏 5.9°，不超出平衡区
    wire signed [15:0] lean_raw = ($signed(enc_pos) * 16'sd28) >>> 8;
    wire signed [15:0] lean = (lean_raw >  16'sd90) ?  16'sd90 :
                              (lean_raw < -16'sd90) ? -16'sd90 : lean_raw;
    // ★ v3.3 修符号 bug：lean[11:0] 本身已经是 12 位补码（-15 ⇒ 0xFF1），
    //   再取一次负等于把 |lean| 加回去 ⇒ E<0 时倾斜变成"帮着往外推"。
    //   12 位模加天然正确，不需要判符号。
    wire [11:0] tgt_lean = tgt12 + lean[11:0];

    wire signed [15:0] pd_cmd;
    wire signed [15:0] pd_cmd_raw;
    wire signed [12:0] pd_a_err, pd_a_rate;
    wire signed [16:0] pd_e_rate;
    wire signed [15:0] pd_acc;

    // ------------------------------------------------------------------
    // ★ v2.5 平衡区门控（BAL_ZONE）—— 修"疯狂旋转"
    //
    //   实测现象：摆杆一旦远离倒立位，电机会把它越甩越快，最后疯狂自旋。
    //   原因在控制律本身，不是参数：
    //     误差是环形算的，摆杆靠近正下方时误差 ≈ ±180° ⇒ 输出打满；
    //     每掠过最低点一次，误差符号翻转、大臂反向猛推一次 ⇒
    //     相当于每趟往摆杆里泵一次能量 ⇒ 越转越快。
    //
    //   标准修法：只在"够得着"的区间里闭合控制，离得远就彻底不输出。
    //   这样失败时电机安静停住，而不是暴力加速。
    //   门限 ±560 counts @ 15.3 counts/度 ≈ ±36.6°，比可控范围略宽，
    //   留一点余量给"抓接"，又不至于在大偏差区泵能量。
    // ------------------------------------------------------------------
    localparam signed [12:0] BAL_ZONE = 13'sd420;
    wire bal_in_zone = (pd_a_err <= BAL_ZONE) && (pd_a_err >= -BAL_ZONE);


    // ------------------------------------------------------------------
    // ★ v2.1 AS5600 看门狗
    //   实测教训：读取状态机没被使能时（ang_on 漏写一个 STAGE），
    //   ang_f 会安静地停在 0，遥测看起来"稳定"，其实一次读都没有。
    //   而 PD 拿到 ang=0、目标 3000 会算出 +96° 的误差 → 输出直接打满，
    //   把臂轰到限位。所以必须自己判"角度是不是新鲜"：
    //   200 ms 没有新的 os_done 就强制输出 0。
    // ------------------------------------------------------------------
    reg [23:0] ang_wd = 24'd0;
    reg        ang_ok = 1'b0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ang_wd <= 24'd0;
            ang_ok <= 1'b0;
        end else if (os_done) begin
            ang_wd <= 24'd0;
            ang_ok <= 1'b1;
        end else if (ang_wd >= 24'd10_000_000) begin   // 200 ms @50MHz
            ang_ok <= 1'b0;
        end else begin
            ang_wd <= ang_wd + 24'd1;
        end
    end

    pd_ctrl #(
        // ★ v2.9（2026-10-21）—— 400 -> 600，修「原理错误 #1」
        //   可用线性区 = (CMD_MAX - 死区前馈200)/kp_a = (400-200)/31.4 = 6.4 度
        //   而实测扰动幅值约 8.5 度 ⇒ 线性区 < 扰动幅值 ⇒ 环路天生不可能线性工作，
        //   表现就是 mean|M|=86%、饱和 42%、约 4 秒后振荡长到超出权限而倒。
        //   600 把线性区拉到 (600-200)/31.4 = 12.7 度，第一次大于扰动幅值。
        //   ★ 代价：电流和发热上升。每跑 10 秒摸一次 TB6612，烫手就立刻停。
        .CMD_MAX  (600),
        .SOFT_LO  (150),
        .SOFT_REV (400)   // v42: 200->400, reverse authority 2x (saturation |E| 163 -> 175)
    ) u_pd (
        .clk       (clk_50m),
        .rst_n     (rst_n),
        .tick      (tick_1k),
        .enable    (mot_en && ang_ok && bal_in_zone),
        .ang12     (ang_f),      // v38: 回到原始符号（v33 的配置）
        .tgt12     (tgt_lean),
        .enc_pos   (enc_pos),
        .kp_a      (sel_kp_a),
        .kd_a      (sel_kd_a),
        .kp_e      (sel_kp_e),
        .kd_e      (sel_kd_e),
        .dead_ff   (sel_ff),
        .rate_sel  (2'd1),           // 16 ms = 4 个 AS5600 采样周期（仿真最优）
        .cmd       (pd_cmd),
        .cmd_raw   (pd_cmd_raw),
        .dbg_a_err (pd_a_err),
        .dbg_a_rate(pd_a_rate),
        .dbg_e_rate(pd_e_rate),
        .dbg_acc   (pd_acc)
    );

    // ------------------------------------------------------------------
    // STAGE=8：电机开环占空比测试（安全模式）
    //   串口 P<n>  →  占空比 n（限幅 ±600，约 ±60%）
    //   ★ 看门狗：2 秒没收到新命令自动归零（防止电机失控飞转）
    // ------------------------------------------------------------------
    wire signed [15:0] duty_lim = (tgt_pos >  16'sd600) ?  16'sd600 :
                                 (tgt_pos < -16'sd600) ? -16'sd600 : tgt_pos;

    reg signed [15:0] duty_hold;
    reg signed [15:0] p_seen;
    reg [27:0]        wd_cnt;
    localparam [27:0] WD_MAX = 28'd100_000_000;   // 2 秒 @50MHz

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            duty_hold <= 16'sd0; p_seen <= 16'sd0; wd_cnt <= 28'd0;
        end else if (STAGE == 8) begin
            if (tgt_pos != p_seen) begin          // 收到新的 P 命令
                p_seen    <= tgt_pos;
                duty_hold <= duty_lim;
                wd_cnt    <= 28'd0;
            end else if (wd_cnt >= WD_MAX) begin  // 超时 → 自动停
                duty_hold <= 16'sd0;
            end else begin
                wd_cnt <= wd_cnt + 28'd1;
            end
        end else begin
            duty_hold <= 16'sd0; wd_cnt <= 28'd0; p_seen <= tgt_pos;
        end
    end

    // 电机命令来源：STAGE=8 用串口给的占空比；否则用双环 PID 输出
    // (P03) 声明已提到文件上方的前向声明区
    // ------------------------------------------------------------------
    // STAGE=33: bidirectional arm sweep (+/-SW_LIM counts) for travel test
    //   reverses the drive when |enc_pos| exceeds SW_LIM
    //   pos_lock is bypassed in this mode, otherwise the arm would stick at the limit
    //   needs E1; SW_LIM must stay well inside POS_STOP
    // ------------------------------------------------------------------
    localparam signed [15:0] SW_LIM = 16'sd220;
    localparam signed [15:0] SW_CMD = 16'sd600;
    reg signed [15:0] sw_cmd = 16'sd600;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                              sw_cmd <= SW_CMD;
        else if (STAGE == 33) begin
            if      ($signed(enc_pos) >  SW_LIM) sw_cmd <= -SW_CMD;
            else if ($signed(enc_pos) < -SW_LIM) sw_cmd <=  SW_CMD;
        end
    end
    assign motor_cmd_src = (STAGE == 18) ? auto_cmd
                         : (STAGE == 17) ? 16'sd0
                         : (STAGE == 8) ? duty_hold : (STAGE == 34) ? duty_hold : (STAGE == 33) ? sw_cmd : (STAGE == 5) ? cmd_stage5 : dual_cmd;
    // ★ v2.1: STAGE=5 现在直接用 pd_cmd，**不再取反**。
    //   原来这里写的是 -dual_cmd，因为 pid_ctrl 内部算的是 target-actual；
    //   pd_ctrl 内部算的是 angle-target，符号已经正确，再取反就变成正反馈。

    // 电机使能：STAGE=5（倒立摆）或 STAGE=8（开环测试），且串口发过 E1
    assign mot_en = (STAGE == 18) ? auto_en
                : (motor_en && ((STAGE == 5) || (STAGE == 8) || (STAGE == 33) || (STAGE == 34)));

    wire signed [15:0] mot_cmd_act;

    // tb6612 的原始输出（STAGE=30 的直流自走测试要接管这几个引脚，所以先接到内部线）
    wire mot_pwm_drv, mot_ain1_drv, mot_ain2_drv, mot_stby_drv;

    // ------------------------------------------------------------------
    // Arm travel software limit
    //   ★ v2.1: STAGE=5 no longer uses this latching cut-off.
    //     pd_ctrl now has a "fade + active reverse bias" soft limit, so the arm
    //     is driven back instead of being frozen.  The old latch removed
    //     exactly the torque needed to bring the arm home, which is why every
    //     experiment ended after ~6 s and needed an 'E0' to recover.
    //   STAGE=8/33/34 (open-loop ones) keep the latch as a backstop.
    // ------------------------------------------------------------------
    localparam signed [15:0] POS_STOP = 16'sd220;
    localparam signed [15:0] POS_REL  = 16'sd160;

    // ★ v2.7（2026-10-21）—— 出平衡区时"把臂开回中位"，而不是简单地输出 0
    //
    //   独立复核指出 v2.6 的两个副作用：
    //   ① 门控关断只把 pd_cmd 置 0 ⇒ v2.6 新加的"回中偏置"也到不了电机
    //      ⇒ 每次失败后臂停在上次的位置（常在 ±200），下一次起跑时可用行程
    //        只剩 20 counts —— 8 次试验从第 2 次起就是废的。
    //   ② v2.6 把 ±214 的"硬限位"换成了 200 duty 的弹簧 ⇒ 2.6° 的误差就能把臂
    //      推过去 ⇒ 臂可能继续转，而 AS5600 的排线跟着臂走，有绕线风险。
    //
    //   一个改动同时解决两条：出平衡区就开回中位（±350 高于静摩擦 150，
    //   ±25 的死区避免中位附近来回蹭）。
    wire signed [15:0] park_cmd = ($signed(enc_pos) >  16'sd25) ? -16'sd350 :
                                  ($signed(enc_pos) < -16'sd25) ?  16'sd350 : 16'sd0;

    // ★ v2.8（2026-10-21）—— 实机手测确认「机械上没有硬止挡」：
    //   手转大臂可到 E=+1086（=1.02 圈）/-660，唯一的边界是 AS5600 的排线。
    //   v2.7 的软限位只加 200 duty 偏置，摆杆项能轻松压过去 ⇒ 平衡中可能绕线。
    //   这里加一道硬保险：臂超过 ±450（≈0.42 圈）就无条件交给 park 开回中位，
    //   不管摆杆说什么。代价是那一刻摆杆暂时失去控制 —— 但绕线是不可逆的。
    // ★ v3.0（2026-10-21）—— 修 v2.8 的"抢占式"臂限位
    //
    //   v2.8 写的是 (bal_in_zone && arm_safe) ? pd_cmd : park_cmd ——
    //   摆杆还在界内、还在被控制，只要臂漂过 450 就把 PD 整个换成 park，
    //   摆杆那一刻失去控制，随即倒下。
    //   实测（v29_b）：E 漂到 +485 → 立刻掉；|E|>200 段 151/164 帧符号错。
    //   用户诊断正确："几乎每次失衡都是臂角必须偏大才能回正，但超过安全范围就断"。
    //
    //   v3.0 改成【叠加式】：臂超程只叠加一个更强的回中偏置，
    //   摆杆项永远保留（唯一会断掉摆杆项的仍是"摆杆已经出平衡区"）。
    //   ±400 偏置 + 摆杆项，再统一夹紧到 ±600。
    localparam signed [15:0] ARM_HARD = 16'sd280;
    localparam signed [15:0] ARM_PUSH = 16'sd560;

    // v3.1：加滞回（复核者指出 E=449->451 之间是 400 duty 的阶跃，
    //   臂在 450 附近漂时会跨阈值抖，5~10 Hz，正好落在摆杆极限环的带里）。
    //   进 480 / 出 380，滞回 100 counts，且正负对称。
    wire arm_out = ($signed(enc_pos) >  16'sd480) || ($signed(enc_pos) < -16'sd480);
    wire arm_in  = ($signed(enc_pos) <  16'sd380) && ($signed(enc_pos) > -16'sd380);

    reg arm_push = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       arm_push <= 1'b0;
        else if (arm_out) arm_push <= 1'b1;
        else if (arm_in)  arm_push <= 1'b0;
    end

    wire signed [15:0] arm_bias = !arm_push ? 16'sd0 :
                                  ($signed(enc_pos) > 16'sd0) ? -ARM_PUSH : ARM_PUSH;

    wire signed [16:0] cmd_pd_b = pd_cmd + arm_bias;
    wire signed [15:0] cmd_lim  = (cmd_pd_b >  17'sd600) ?  16'sd600 :
                                  (cmd_pd_b < -17'sd600) ? -16'sd600 : cmd_pd_b[15:0];

    // 只有摆杆真的出了平衡区，才交还给 park；臂超程不再夺权
    // ================= STAGE=6 起摆（能量泵能）  2026-10-24 ================
//  原理: u = -K*sign(cos t)*sign(t_dot)   (Astrom Furuta 摆能量法)
//  兼容: 摆杆已在 ±CATCH_ANG 内开 E1 => 直接平衡, 行为与 v42 完全一致
localparam signed [15:0] PUMP_K     = 16'sd420;   // 泵能方波幅值 (先 420, 可调 350~500)
localparam signed [12:0] CATCH_ANG  = 13'sd400;   // 捕获窗口 ±230 counts = ±20.2 deg
localparam [11:0]        CATCH_RATE = 12'd60;    // |单样本差分| 上限 (4.1ms/样本)

// theta: 相对竖直的圆环差 (signed, -2048..+2047)
wire [11:0] d6 = ang_f - tgt12;
wire signed [12:0] a_err6 = (d6 >= 12'd2048) ? ($signed({1'b0, d6}) - 13'sd4096)
                                            :  $signed({1'b0, d6});

// theta_dot 的符号: 1 个采样间隔 (~4.1ms) 差分, 只取符号
// v45: 16ms 窗口差分（4 个采样），避免 4.1ms 差分被 ±1 count 量化噪声主导
reg [1:0] d16_cnt;
always @(posedge clk_50m or negedge rst_n)
    if (!rst_n)       d16_cnt <= 2'd0;
    else if (os_done) d16_cnt <= d16_cnt + 2'd1;
wire d16_tick = os_done && (d16_cnt == 2'd3);

reg [11:0] ang_prev;
always @(posedge clk_50m or negedge rst_n)
    if (!rst_n)        ang_prev <= 12'd0;
    else if (d16_tick) ang_prev <= ang_f;

wire [11:0] dr6    = ang_f - ang_prev;                  // 16ms 窗口差分, 模 4096
wire        r6_up  = (dr6 != 12'd0) && (dr6 < 12'd2048);
wire        r6_big = (dr6 >= CATCH_RATE) && (dr6 <= (12'd4096 - CATCH_RATE));

// sign(cos t): |theta| < 1024 counts (=90deg) 为正
wire cos_pos = !(a_err6 >= 13'sd1024) && !(a_err6 <= -13'sd1024);

// 能量律: 两符号相异 => u 为正
// v45: 只在摆杆真的动了才更新相位；dr6==0 时保持上一次（消除 -420 恒定退化）
reg pump_pos_r;
always @(posedge clk_50m or negedge rst_n)
    if (!rst_n)                      pump_pos_r <= 1'b0;
    else if (d16_tick && dr6 != 12'd0) pump_pos_r <= cos_pos ^ r6_up;

// ======================================================================
// v65 (2026-10-25): 能量反馈泵 —— 替代定幅方波
//
// 【为什么必须改】竖直是摆的最低点、速度最大处, 不可能在此接住一根仍在飞行的杆。
//   69 个捕获段实测入口 |w| = 250~400 deg/s, 保持时间随入口速度单调下降
//   (120~250: 均值 2.65 s / 250~400: 1.23 s / 400+: 0.47 s)。
//
// 【定量要求】由实测摆动周期 T=0.512 s 得 J/(mgl)=(T/2pi)^2=6.64e-3 s^2,
//   逃逸角速度 w_esc = sqrt(4/(T/2pi)^2) = 1406 deg/s。
//   实测到顶 300 deg/s => 释放速度 w0 = sqrt(1406^2+300^2) = 1435 deg/s。
//   要降到 100 deg/s => w0 = 1409 deg/s。  精度要求 = 1435 -> 1409 = 1.8%。
//
// 【本模块的核心, 不是细节】幅值必须与能量差额成正比。
//   方波幅值把最后一拍的能量量子化, 永远打不进 2%;
//   比例幅值让最后一拍可以无限小, 杆是收敛到竖直而不是撞过去。
//   >>> 若把下面 pump_amp 改回常数, 本版本立即退化为 v62, 等于没改。 <<<
//
// 【pump_off_lat 不能再用来停机】它的门限 255 cnt/16.4ms = 1366 deg/s,
//   低于需要的 1435 deg/s, 比例尾巴一出现就会被它截断。
//   停机条件改为 e_deficit <= 0 (能量已达目标)。
//
// 归一化能量 (以 mgl 为单位, 目标 2.0):
//   eps = (J/(2mgl))*w^2 + (1 - cos th)
//   w[rad/s] = dr6 * (2pi/4096) / 0.0164 = dr6 * 0.09354 -> w^2 = 8.750e-3*dr6^2
//   (J/(2mgl))*w^2 = 2.906e-5 * dr6^2
//   Q12 定点: e_kin = (dr6^2 * 122) >> 10
//   标定: dr6=255 (1366 deg/s) 时 e_kin=7742, 与 E_TGT=8192 差 5.5%,
//         正对应实测到顶 250~330 deg/s 的余量。
// ======================================================================
function [12:0] cos_q12;                 // 32 项 Q12 cos LUT, 11.25 deg 一档
    input signed [12:0] th;
    reg [4:0] idx5;
    begin
        idx5 = (th + 13'sd2048) >> 7;    // -2048..2047 -> 0..31
        case (idx5)
            5'd0:  cos_q12 = -13'sd4096;
            5'd1:  cos_q12 = -13'sd4017;
            5'd2:  cos_q12 = -13'sd3785;
            5'd3:  cos_q12 = -13'sd3406;
            5'd4:  cos_q12 = -13'sd2896;
            5'd5:  cos_q12 = -13'sd2273;
            5'd6:  cos_q12 = -13'sd1568;
            5'd7:  cos_q12 = -13'sd794;
            5'd8:  cos_q12 =  13'sd0;
            5'd9:  cos_q12 =  13'sd794;
            5'd10: cos_q12 =  13'sd1568;
            5'd11: cos_q12 =  13'sd2273;
            5'd12: cos_q12 =  13'sd2896;
            5'd13: cos_q12 =  13'sd3406;
            5'd14: cos_q12 =  13'sd3785;
            5'd15: cos_q12 =  13'sd4017;
            5'd16: cos_q12 =  13'sd4096;
            5'd17: cos_q12 =  13'sd4017;
            5'd18: cos_q12 =  13'sd3785;
            5'd19: cos_q12 =  13'sd3406;
            5'd20: cos_q12 =  13'sd2896;
            5'd21: cos_q12 =  13'sd2273;
            5'd22: cos_q12 =  13'sd1568;
            5'd23: cos_q12 =  13'sd794;
            5'd24: cos_q12 =  13'sd0;
            5'd25: cos_q12 = -13'sd794;
            5'd26: cos_q12 = -13'sd1568;
            5'd27: cos_q12 = -13'sd2273;
            5'd28: cos_q12 = -13'sd2896;
            5'd29: cos_q12 = -13'sd3406;
            5'd30: cos_q12 = -13'sd3785;
            default: cos_q12 = -13'sd4017;
        endcase
    end
endfunction

wire signed [12:0] cos_th    = cos_q12(a_err6);
wire [31:0] dr6_sq   = dr6 * dr6;                    // <= 4.19e6
wire [31:0] e_kin    = (dr6_sq * 32'd122) >> 10;     // Q12
// 注意: 必须显式把 cos_th 从 13 位符号扩展到 14 位。
//   写成 14'd4096 + $signed(cos_th) 是错的 —— 14'd4096 无符号会让整个表达式变无符号,
//   cos_th=-4096 会被当成 +4096, 于是下垂时 e_pot=8192=目标, deficit=0, M=0, 杆不动。
wire [13:0] e_pot    = 14'd4096 + {cos_th[12], cos_th};  // (1+cos), Q12, 0(下垂)..8192(竖直)
wire [31:0] e_sum    = e_kin + {18'd0, e_pot};       // eps, Q12
wire signed [31:0] e_deficit = 32'sd8192 - $signed(e_sum);   // E_TGT = 2.0

// v65c: 增益 3/8 -> 1/4。
//   依据: v65b_r1 入口速度中位 431 °/s (v64 是 303), 400+ 段 6/9=67% (v64 是 16%),
//         属于"被推过头"。同一份数据里组内剂量-反应很干净:
//           入口 306 -> 站 5.86 s
//           入口 387 -> 站 1.45 s
//           入口 420~473 -> 站 <=0.97 s
//         => 机制对, 只是泵给的太多。
//   注意: kv 系数 (2.906e-5) 不许动。它是用 "dr6=255 <-> 1400 deg/s 刚好够到顶"
//         反标定出来的, 减半等于宣称 1000 deg/s 就够, 与实测矛盾, 且会让历史数据失效。
//   1/4 时满幅对应 deficit=2400 (目标的 29%), 爬升会慢一些, 到顶更软。
wire signed [31:0] amp_raw  = e_deficit >>> 2;        // 1/4 (原 3/8)
wire signed [15:0] pump_amp = (e_deficit <= 0)      ? 16'sd0
                            : (amp_raw >  32'sd600) ? 16'sd600
                            : (amp_raw < -32'sd600) ? -16'sd600
                            : amp_raw[15:0];

wire signed [15:0] pump_cmd = pump_pos_r ? pump_amp : -pump_amp;

wire sw_catch = (a_err6 <= CATCH_ANG) && (a_err6 >= -CATCH_ANG) && !r6_big
             && ((enc_pos) > -16'sd150) && ((enc_pos) < 16'sd150);  // v52: 交接时大臂必须在中位

// 状态: mot_en 上升沿 + 摆杆不竖直 => 自动泵能; 捕获 => 交接给平衡律
reg mot_en_d, sw_arming;
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        mot_en_d  <= 1'b0;
        sw_arming <= 1'b0;
    end else begin
        mot_en_d <= mot_en;
        if (!mot_en || !ang_ok) sw_arming <= 1'b0;   // v47: 传感器失效立即停泵
        else if (mot_en && !mot_en_d &&
                 !((a_err6 <= CATCH_ANG) && (a_err6 >= -CATCH_ANG)))
            sw_arming <= 1'b1;
        // ------------------------------------------------------------------
    // v63 (2026-10-25): 打破【泵能吸收态】
    //   v53 的 sw_arming 只有三个出口: !mot_en / !ang_ok / sw_catch,
    //   而 sw_catch 要求 |E|<150。
    //   一旦泵能把臂推过 150 且杆同时在飞, 三者永不重合 =>
    //     sw_arming 永远为 1 => cmd 一直走 pump_final,
    //     **绕过 arm_bias = 16*(|E|-150) 这条强回中**
    //     => 臂外漂到 380 被 ARM_GUARD 用 -600 打回, 又漂出去,
    //        在 250~400 之间形成极限环, 到断电才停。
    //   实测(今天 8 次跑次, 6 次卡死): 臂驻留 E∈[300,400) 达 37.8 s/60 s。
    //   修法: 泵能期间臂一旦跑出 ±220, 放弃本轮泵能,
    //         交还 cmd_lim(强回中) 或 park_cmd(bang-bang), 把臂拉回来后
    //         由 L877 的 !sw_arming && !bal_in_zone && !sw_catch 自动重新起摆。
    //   阈值依据(首次捕获前的臂峰值):
    //     成功泵能 v53new_r2=144 / v59b_r1=185  (8.23 s / 11.67 s 捕获)
    //     卡死跑次 v62_r5=243 / r8=253 / v53new_r1=329 / v59b_r2=345 / v62_r1=381
    //     => 220 在两组之间, 两侧各留 35 / 23 的余量。
    // ------------------------------------------------------------------
    else if (sw_arming && (($signed(enc_pos) > 16'sd220) || ($signed(enc_pos) < -16'sd220)))
        sw_arming <= 1'b0;
    else if (sw_arming && sw_catch) sw_arming <= 1'b0;
        // --------------------------------------------------------------
        // v64 (2026-10-25): 给重起摆加迟滞 —— v63 的中止只持续一个时钟周期
        //   v63 的 ①② 在同一个 always 块里, 每拍只走一支:
        //     第 N 拍 |E|>220 -> sw_arming<=0
        //     第 N+1 拍 !sw_arming -> 立刻重新置 1
        //   => 中止 = 1 个 20 ns 周期, 泵能从未在臂回中之后完整跑过一轮。
        //     所以 v63 只修好了臂不再永久锁死, 没修好泵能从中间开始。
        //   实测 v63: |E|>350 从 18~30% 降到 1.3~2.2% (10~15x), park_cmd 占 6~11.5%,
        //             但捕获段只有 0.46/0.90/2.54/1.64 s。
        //   修法: 臂回到 ±80 以内才允许重新起摆, 于是每轮泵能都从臂在中位开始,
        //         正是 sw_catch 要求的入口条件。
        //   依据: 成功泵能(v53new_r2 臂峰值 144 / v59b_r1 185)说明从中位开始可行;
        //         80 远小于中止门 220, 迟滞区足够。
        // --------------------------------------------------------------
        // --------------------------------------------------------------
        // v72: 泵重新武装条件 —— 由"杆出平衡区"改为"杆真的倒了(|th|>90deg)"
        //   v65c 原条件 !bal_in_zone 使泵在杆刚晃过 +-36.9deg 时重新接管臂,
        //   而 36.9deg 本是 PD 能拉回的普通摆幅 => 平衡被泵打断, 上限 4.15s。
        //   改后: |th|<=90deg 一律留在 cmd_lim(与 v42 行为一致),
        //         只有真倒了、平衡律已无救, 才交给泵。
        //   下垂(180deg)时 |a_err6|=2048>1024, 仍允许起摆, 不影响泵能。
        // --------------------------------------------------------------
        else if (!sw_arming && ang_ok && !sw_catch
                 && ((a_err6 > 13'sd1024) || (a_err6 < -13'sd1024))
                 && ($signed(enc_pos) > -16'sd80) && ($signed(enc_pos) < 16'sd80))
            sw_arming <= 1'b1;   // v72: 只有杆倒下且臂回中才重新起摆
    end
end
// ======================================================================

// ---- 大臂位置硬保护：泵能期间禁止大臂越过 ±ARM_GUARD（防绷线）----
localparam signed [15:0] ARM_GUARD = 16'sd380;
// v47: 能量封顶 —— 底速超过 PUMP_RATE_CAP 说明能量已够, 停止泵能让杆自己冲上去
localparam [11:0] PUMP_RATE_CAP = 12'd255;   // 255 cnt/16ms = 15940 cnt/s ; 刚到顶需 252 (实测周期 0.520s)
wire rate_high = (dr6 >= PUMP_RATE_CAP) && (dr6 <= (12'd4096 - PUMP_RATE_CAP));
// v50: 泵能加"大臂回中项" —— 泵能期间大臂不能随机游走, 否则交接时 E 偏离中位导致平衡律 hold 不住
wire signed [15:0] pump_ctr = $signed(enc_pos) >>> 1;              // v52: 退回 E/2 (gain=1 会砍掉正向半周)
wire signed [16:0] pump_sum = $signed(pump_cmd) - $signed(pump_ctr);  // pump - E/2
wire signed [15:0] pump_mix = (pump_sum >  17'sd600) ? 16'sd600
                            : (pump_sum < -17'sd600) ? -16'sd600
                            : pump_sum[15:0];
// v53: 能量判据改成【锁存】—— 逐样本门控只能削峰, 不能限制总能量
//      泵到阈值即锁死停泵, 杆带固定能量上去, 在折返点(P≈0)被接住
reg pump_off_lat;
always @(posedge clk_50m or negedge rst_n)
    if (!rst_n)          pump_off_lat <= 1'b0;
    else if (!mot_en)    pump_off_lat <= 1'b0;                 // E0 清零
    else if (rate_high)  pump_off_lat <= 1'b1;                 // 能量够了 -> 锁死
    else if ((dr6 < 12'd100) && ((a_err6 > 13'sd1500) || (a_err6 < -13'sd1500)))
                         pump_off_lat <= 1'b0;                 // 杆已慢且近下垂 => 能量耗尽, 重新泵
wire signed [15:0] pump_final = ($signed(enc_pos) < -ARM_GUARD) ? 16'sd600
                              : ($signed(enc_pos) >  ARM_GUARD) ? -16'sd600
                              : (e_deficit <= 0) ? 16'sd0          // v65: 能量已达目标 -> 停泵(替代 pump_off_lat)
                              : pump_mix;
wire signed [15:0] cmd_stage5 = sw_arming ? pump_final
                             : (bal_in_zone ? cmd_lim : park_cmd);

    wire use_hard_lock = (STAGE != 5);

    reg pos_lock = 1'b0;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                                              pos_lock <= 1'b0;
        else if (!use_hard_lock)                                 pos_lock <= 1'b0;
        else if (($signed(enc_pos) > POS_STOP) || ($signed(enc_pos) < -POS_STOP)) pos_lock <= 1'b1;
        else if (!motor_en)                                                      pos_lock <= 1'b0;
    end

    wire signed [15:0] cmd_final = (STAGE == 33) ? motor_cmd_src : (pos_lock ? 16'sd0 : motor_cmd_src);
    wire               en_final  = (STAGE == 33) ? mot_en : (mot_en && !pos_lock);

    tb6612_drv #(
        .CLK_HZ   (50_000_000),
        .PWM_HZ   (20_000),
        .DEAD_CLK (5_000),
        .CMD_MAX  (1000)
    ) u_motor (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .enable  (en_final),
        .cmd     (cmd_final),
        .pwm     (mot_pwm_drv),
        .ain1    (mot_ain1_drv),
        .ain2    (mot_ain2_drv),
        .stby    (mot_stby_drv),
        .cmd_out (mot_cmd_act)
    );

    // ------------------------------------------------------------------
    // STAGE=30  电机直流自走测试（上电即转，不需要任何串口命令）
    //   目的：绕开 20kHz PWM，直接给引脚【静态】电平：
    //     · 静态电平能转   → 82~85 上的板载 LCD 电路把 20kHz PWM 滤掉/吃掉了
    //     · 静态电平也不转 → 驱动模块或电机输出侧（AO1/AO2、电机线）的问题
    //   6 秒一循环（50MHz）：
    //     0.0~2.0s  正转  AIN1=1 AIN2=0 PWMA=1（100% 静态直流）
    //     2.0~3.0s  停    PWMA=0
    //     3.0~5.0s  反转  AIN1=0 AIN2=1 PWMA=1
    //     5.0~6.0s  停
    //   LED：led[0]=正转中  led[1]=反转中  led[7]=2Hz 心跳（程序活着的铁证）
    // ------------------------------------------------------------------
    reg [28:0] mt_cnt = 29'd0;

    always @(posedge clk_50m) begin
        if (mt_cnt >= 29'd300_000_000 - 29'd1)     // 6 s
            mt_cnt <= 29'd0;
        else
            mt_cnt <= mt_cnt + 29'd1;
    end

    wire mt_fwd = (mt_cnt <  29'd100_000_000);                                  // 0.0~2.0s
    wire mt_rev = (mt_cnt >= 29'd150_000_000) && (mt_cnt < 29'd250_000_000);    // 3.0~5.0s
    wire mt_on  = mt_fwd | mt_rev;

    // ------------------------------------------------------------------
    // STAGE=31  电机引脚组合自动扫描（编码器当裁判，不需要人看）
    //   82~85 同时挂着板载 LCD，某一路可能根本顶不到高电平。
    //   每 5 秒换一种驱动组合，哪一段 enc_pos 开始变化 ⇒ 那种组合是通的：
    //     idx0  83=1 84=0 85=70%PWM     标准接法（PWM 送 PWMA）
    //     idx1  83=1 84=0 85=1          100% 静态直流
    //     idx2  83=70%PWM 84=0 85=1     PWM 改从 AIN1 送（若 85 被 LCD 拉住就能救）
    //     idx3  83=0 84=70%PWM 85=1     PWM 改从 AIN2 送（反转）
    //     idx4  83=1 84=0 85=1          重复 idx1，确认可复现
    //     idx5  83=0 84=1 85=1          反转 100%
    //   82(STBY) 全程为 1；LED[2:0] 显示当前 idx，LED7 心跳
    // ------------------------------------------------------------------
    reg [15:0] saf_cnt = 16'd0;          // 20kHz PWM 载波
    reg [2:0]  saf_idx = 3'd0;           // 当前组合 0..5
    reg [27:0] saf_tick = 28'd0;         // 5 秒定时

    always @(posedge clk_50m) begin
        if (saf_cnt >= 16'd2499)
            saf_cnt <= 16'd0;
        else
            saf_cnt <= saf_cnt + 16'd1;
    end

    always @(posedge clk_50m) begin
        if (saf_tick >= 28'd250_000_000 - 28'd1) begin
            saf_tick <= 28'd0;
            saf_idx  <= (saf_idx >= 3'd5) ? 3'd0 : saf_idx + 3'd1;
        end else begin
            saf_tick <= saf_tick + 28'd1;
        end
    end

    wire saf_pwm = (saf_cnt < 16'd1750);          // 1750/2500 = 70%

    wire saf_ain1 = (saf_idx == 3'd2) ? saf_pwm :
                    ((saf_idx == 3'd3) || (saf_idx == 3'd5)) ? 1'b0 : 1'b1;
    wire saf_ain2 = (saf_idx == 3'd3) ? saf_pwm :
                    (saf_idx == 3'd5) ? 1'b1 : 1'b0;
    wire saf_pwma = (saf_idx == 3'd0) ? saf_pwm : 1'b1;

    // 电机引脚输出：STAGE=30/31 由上面的测试模块接管，其余阶段仍由 tb6612_drv 驱动
    assign mot_pwm  = (STAGE == 30) ? mt_on   : (STAGE == 31) ? saf_pwma : mot_pwm_drv;
    assign mot_ain1 = (STAGE == 30) ? mt_fwd  : (STAGE == 31) ? saf_ain1 : mot_ain1_drv;
    assign mot_ain2 = (STAGE == 30) ? mt_rev  : (STAGE == 31) ? saf_ain2 : mot_ain2_drv;
    assign mot_stby = 1'bz;   // never drive pin 82 (LCD backlight) - over-current risk

    // ------------------------------------------------------------------
    // STAGE<=4：单轴（舵机）练习用的开环扫掠 + PID 闭环
    // ------------------------------------------------------------------
    reg [15:0] sweep_cnt = 16'd0;
    reg [11:0] sweep_us  = 12'd1500;
    reg        sweep_dir = 1'b0;

    always @(posedge clk_50m) begin
        if (sweep_cnt == 16'd39_999) begin
            sweep_cnt <= 16'd0;
            if (sweep_dir == 1'b0) begin
                if (sweep_us >= 12'd2500) begin
                    sweep_dir <= 1'b1;
                    sweep_us  <= sweep_us - 1'b1;
                end else begin
                    sweep_us  <= sweep_us + 1'b1;
                end
            end else begin
                if (sweep_us <= 12'd500) begin
                    sweep_dir <= 1'b0;
                    sweep_us  <= sweep_us + 1'b1;
                end else begin
                    sweep_us  <= sweep_us - 1'b1;
                end
            end
        end else begin
            sweep_cnt <= sweep_cnt + 1'b1;
        end
    end

    wire signed [15:0] servo_pid_out;

    pid_ctrl #(
        .KP_Q8      (KP_Q8),
        .KI_Q8      (KI_Q8),
        .KD_Q8      (KD_Q8),
        .OUT_MAX_US (OUT_MAX_US),
        .INT_MAX    (20000)
    ) u_servo_pid (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .tick_1k (tick_1k),
        .target  (tgt_angle),
        .actual  (adc_val),
        .out     (servo_pid_out)
    );

    wire [11:0] servo_pulse_us;
    // STAGE=5 时舵机不再参与控制，固定输出中位脉冲 1500us
    wire        servo_manual    = (STAGE == 3) || (STAGE == 5);
    wire [11:0] servo_manual_us = (STAGE == 5) ? 12'd1500 : sweep_us;
    wire signed [15:0] servo_off = (STAGE == 5) ? 16'sd0 : servo_pid_out;

    servo_pwm #(
        .CLK_HZ    (50_000_000),
        .FRAME_US  (20_000),
        .MIN_US    (500),
        .MAX_US    (2500),
        .CENTER_US (1500)
    ) u_servo (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        .offset_us    (servo_off),
        .manual_us    (servo_manual_us),
        .use_manual   (servo_manual),
        .pwm          (servo_pwm),
        .pulse_us_out (servo_pulse_us)
    );

    // ------------------------------------------------------------------
    // STAGE=6：DAC1 -> ADC1 环回自检
    //   用套件自带的 BNC-BNC 跳线把 DAC1 与 ADC1 连起来，
    //   FPGA 自己在 DAC1 上输出慢速三角波，再从 ADC1 读回来。
    //   遥测里 M = 送出的码值、A = 读回的码值；理想情况两条曲线形状一致。
    //   这一项同时验证：DAC 并口时序 + 运放/低通 + 衰减器 + ADC 并口采集。
    // ------------------------------------------------------------------
    wire [9:0] ramp_code;
    wire       ramp_update;
    wire [9:0] sq_code;
    wire       sq_update;

    ramp_gen #(
        .CLK_HZ  (50_000_000),
        .STEP_US (1000),
        .STEP    (1),
        .MAX     (1023)
    ) u_ramp (
        .clk    (clk_50m),
        .rst_n  (rst_n),
        .code   (ramp_code),
        .update (ramp_update)
    );

    dac_square #(
        .CLK_HZ  (50_000_000),
        .HALF_MS (5000),
        .HIGH    (1023),
        .LOW     (0)
    ) u_sq (
        .clk    (clk_50m),
        .rst_n  (rst_n),
        .code   (sq_code),
        .update (sq_update)
    );

    wire [9:0] dac_code   = (DAC_WAVE == 1) ? sq_code   : ramp_code;
    wire       dac_update = (DAC_WAVE == 1) ? sq_update : ramp_update;

    dac_3pd5651 #(
        .CLK_DIV (1)
    ) u_dac (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .value   (dac_code),
        .update  (dac_update),
        .d_out   (dac_d),
        .dac_clk (dac_clk_out)
    );

    // ------------------------------------------------------------------
    // AS5600 磁编码器（I2C 地址 0x36）—— STAGE=7 联调用
    //   SCL = GPIO_90，SDA = GPIO_88（开漏模拟：oe=1 拉低，0 释放）
    // ------------------------------------------------------------------
    wire        scl_oe, sda_oe, sda_in;

    // ==================================================================
    // ★ 诊断用的 AS5600 阶段（36/37/38）必须和 STAGE=5/13 一样被放行，
    //   否则会出一个极难查的假象：
    //     内部状态机照常跑完 204 个 epoch（因为它是靠内部 epoch 推进的，
    //     与引脚无关）→ os_done 照常脉冲 → 遥测 T 显示 ~61，
    //     但 SCL 根本没输出到引脚 → 芯片收不到时钟 → 每个字节都 NACK
    //     → regout 被 !nack_seen 门控住不更新 → A 恒 0。
    //   看起来像"I2C 坏了/线松了"，其实只是这一行 mux 少写了一个分支。
    // ==================================================================
    // ★ v2.3：这份 STAGE 列表原来在两个地方各抄了一遍
    //   （这里的 ang_stage 和下面 os_run 逻辑里的 ang_on），
    //   历史上就因为漏改其中一处出过两次"静默不工作"。
    //   现在只保留一份 ang_en_stage，两处都引用它。
    wire ang_en_stage = ((STAGE == 13) || (STAGE == 5) ||
                         (STAGE == 36) || (STAGE == 37) ||
                         (STAGE == 38) || (STAGE == 39));
    wire ang_stage = ang_en_stage;      // 兼容旧名字

    assign i2c_scl = (STAGE == 16) ? (fld_scl_st & fld_scl_ag & fld_scl_mg & fld_scl_an) :
                     FLD_DIAG ? (fld_scl_st & fld_scl_ag & fld_scl_mg & fld_scl_an) :
                     (STAGE == 15) ? os2_scl :
                     ang_stage ? os_scl :
                     (STAGE == 14) ? (scan_scl_oe ? 1'b0 : 1'bz) : (scl_oe ? 1'b0 : 1'bz);
    // ★ 修复：STAGE=5/13/15/16/19/20/21 的 SDA 必须完全放开（1'bz）
    //   这几个阶段用的是 FpOC 的 as5600_i2c 实例，它的 .sda 直接接在 i2c_sda 上（inout，自带 OE）。
    //   原来这里没有对应分支，SDA 由【已禁用的旧模块 as5600_read 的 sda_oe】拉低 ⇒ 两个驱动源打架
    //   ⇒ 实测 16% 的读数出现位错误（0xB17→0xB00 / →0x81F 等）。这也解释了 STAGE=7 全 0。
    assign i2c_sda = (STAGE == 14) ? (scan_sda_oe ? 1'b0 : 1'bz) :
                     (ang_stage || (STAGE == 15) || (STAGE == 16) || FLD_DIAG) ? 1'bz :
                     (sda_oe ? 1'b0 : 1'bz);
    assign sda_in  = i2c_sda;

    wire [11:0] as5600_angle;
    wire        as5600_valid, as5600_err;
    wire [7:0]  as5600_status, as5600_agc;
    wire [11:0] as5600_mag;
    wire [11:0] st_zpos, st_mpos, st_mang;
    wire [7:0]  st_conf_hi;
    wire [1:0]  dbg_st;
    wire [3:0]  dbg_step, dbg_i2c_state;
    wire [1:0]  dbg_i2c_ph;

    as5600_read #(
        .DEV_ADDR   (7'h36),
        .CLK_DIV    (2500),
        .PERIOD_1MS (5),
        .SELFTEST   (SELFTEST_MODE),
        .ENABLE     (I2C_OLD_EN)
    ) u_as5600 (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .tick_1k    (tick_1k),
        .angle      (as5600_angle),
        .valid      (as5600_valid),
        .err_flag   (as5600_err),
        .status     (as5600_status),
        .agc        (as5600_agc),
        .magnitude  (as5600_mag),
        .zpos       (st_zpos),
        .mpos       (st_mpos),
        .mang       (st_mang),
        .conf_hi    (st_conf_hi),
        .dbg_st        (dbg_st),
        .dbg_step      (dbg_step),
        .dbg_i2c_state (dbg_i2c_state),
        .dbg_i2c_ph    (dbg_i2c_ph),
        .scl_oe     (scl_oe),
        .sda_oe     (sda_oe),
        .sda_in     (sda_in)
    );

    // STAGE=7：遥测 A 字段直接显示 12 位角度（0~4095）
    // ------------------------------------------------------------------
    // STAGE=13：用开源实现（FpOC）读 AS5600 角度
    // ------------------------------------------------------------------
    // (P03) os_scl / os_done / os_ready / os_regout 已在前向声明区声明
    reg  [15:0] os_cnt;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) os_cnt <= 16'd0;
        else if (os_done) os_cnt <= os_cnt + 16'd1;
    end

    // ---- AS5600 角度：单笔 2 字节突发读 0x0E(高) / 0x0F(低) ----
    // 2026-10-19 实测（推翻 10-18 的判断）：
    //   * 2 字节突发读本身是对的：读 0x0E 得 0x0B 0x17，读 0x0F 得 0x17 …，
    //     重叠字节一致 ⇒ 拼接顺序、ACK、采样相位都对。
    //   * 真正的问题是【时钟太快】：本板 SDA 只有很弱的上拉，
    //        CLK_DIV=1000 (12.5 kHz) → 25% 的读丢位（0x0F 读成 0x03 之类）
    //        CLK_DIV=2000 (6.25 kHz) → 100% 稳定，但 100% 是错的（更危险！）
    //        CLK_DIV=4000 (3.1 kHz)  → 598/598 帧完全正确
    //     ⇒ 必须降到 3.1 kHz。
    //   * STOP 之后必须留 >=4.7us 总线空闲（这里 40us），否则从机可能漏掉 START。
    // 分频：SCL = 50MHz / (4*CLK_DIV) —— 注意 4*CLK_DIV 是 SCL 周期，不是事务长度。
    //   一笔事务 ≈ 205 个 epoch ⇒
    //   CLK_DIV=4000 -> 820k 拍 = 16.4 ms（实测 T=61）
    //   CLK_DIV=1000 -> 205k 拍 =  4.1 ms（预期 T≈243）
    // ★ v2.3：OS_CLKDIV 已提到文件开头做 parameter，由 build_pd.ps1 -I2cDiv 注入，
    //   不再在这里手改 —— 手改的综合结果无法从文件名反查（历史上有过 32 分钟改两次、
    //   没有任何记录的固件，谁也不知道手上那个 .fs 到底是哪一档）。
    localparam [15:0] OS_GAP    = 16'd2_000;     // 40 us @50MHz

    // ★ 这个门控是新加 STAGE 时最容易漏的一处：AS5600 的读取状态机
    //   （u_as5600_ang）只在这里被使能，漏写就会出现"T=0 / A=0 恒不动"，
    //   看起来像 I2C 坏了，其实是一次读都没有发起。
    //   v2.3：改成引用同一个 ang_en_stage，不再单独抄一份列表。
    wire        ang_on = ang_en_stage;
    reg         os_run;
    reg  [15:0] os_gap;
    wire        os_en = ang_on && os_run;
    wire        os_scl_i;
    wire [15:0] os_val;
    wire        os_dn, os_er;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            os_run <= 1'b0; os_gap <= 16'd0;
        end else if (!ang_on) begin
            os_run <= 1'b0; os_gap <= 16'd0;
        end else if (!os_run) begin
            if (os_gap >= OS_GAP) begin
                os_gap <= 16'd0;
                os_run <= 1'b1;
            end else begin
                os_gap <= os_gap + 16'd1;
            end
        end else if (os_dn) begin
            os_run <= 1'b0;                  // 放总线，进入 40us 空闲
            os_gap <= 16'd0;
        end
    end

    as5600_i2c #(.CLK_DIV(OS_CLKDIV), .SLAVE_ADDR(7'h36), .REGISTER_ADDR(8'h0E))
    u_as5600_ang (
        .rst_n(rst_n), .clk(clk_50m), .scl(os_scl_i), .sda(i2c_sda),
        .en(os_en), .start(os_en), .ready(),
        .done(os_dn), .regout(os_val), .err(os_er)
    );

    wire [11:0] ang12 = os_val[11:0];            // 一笔事务里就有完整 12 位

    assign os_scl    = os_scl_i;
    assign os_done   = os_dn;                    // 每完成一笔就更新一次（≈61 Hz）
    assign os_regout = {4'b0, ang12};
    // ------------------------------------------------------------------
    // STAGE=14：I2C 地址扫描 —— 逐个探测地址，看哪个有芯片应答
    //   A = 当前探测的地址   T = 有应答的地址个数
    //   E = 第一个应答的地址（AS5600 应该是 54 = 0x36）
    //   M = 已完成的探测次数（证明扫描在跑）
    // ------------------------------------------------------------------
    wire        scan_en = (STAGE == 14);
    reg  [6:0]  scan_addr;
    reg         scan_start;
    // (P03) scan_busy/scan_err/scan_scl_oe/scan_sda_oe 已在前向声明区声明
    reg  [7:0]  scan_ack_cnt, scan_done_cnt;
    reg  [6:0]  scan_first;
    reg         scan_found;
    reg  [1:0]  scan_st;
    reg  [15:0] scan_wait;

    i2c_master #(.CLK_DIV(1250)) u_scan (
        .clk      (clk_50m),
        .rst_n    (rst_n),
        .start    (scan_start),
        .dev_addr (scan_addr),
        .reg_addr (8'h0E),
        .rd_data  (),
        .rd_valid (),
        .err      (scan_err),
        .busy     (scan_busy),
        .scl_oe   (scan_scl_oe),
        .sda_oe   (scan_sda_oe),
        .sda_in   (sda_in)
    );

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            scan_addr <= 7'd8; scan_start <= 1'b0; scan_st <= 2'd0;
            scan_ack_cnt <= 8'd0; scan_done_cnt <= 8'd0;
            scan_first <= 7'd0; scan_found <= 1'b0; scan_wait <= 16'd0;
        end else begin
            scan_start <= 1'b0;
            case (scan_st)
            2'd0: if (!scan_en) begin scan_st <= 2'd0; end
                  else if (!scan_busy) begin scan_start <= 1'b1; scan_st <= 2'd1; end
            2'd1: if (scan_busy) scan_st <= 2'd2;
            2'd2: if (!scan_busy) begin
                      if (!scan_err) begin
                          scan_ack_cnt <= scan_ack_cnt + 8'd1;
                          if (!scan_found) begin scan_found <= 1'b1; scan_first <= scan_addr; end
                      end
                      scan_done_cnt <= scan_done_cnt + 8'd1;
                      if (scan_addr == 7'd120) scan_addr <= 7'd8;
                      else                     scan_addr <= scan_addr + 7'd1;
                      scan_wait <= 16'd20000;      // 每次探测间隔
                      scan_st   <= 2'd3;
                  end
            2'd3: if (scan_wait == 16'd0) begin
                      // 扫完一轮后复位统计，重新开始
                      if (scan_addr == 7'd8) begin
                          scan_ack_cnt <= 8'd0; scan_found <= 1'b0; scan_first <= 7'd0;
                      end
                      scan_st <= 2'd0;
                  end else scan_wait <= scan_wait - 16'd1;
            endcase
        end
    end
    // ------------------------------------------------------------------
    // STAGE=15：读 AS5600 的 STATUS 寄存器（0x0B）—— 不需要磁铁即可验证读通路
    //   A = STATUS 字节（正常应 ≈ 23 = 0x17，含 ML=1 磁场太弱位）
    // ------------------------------------------------------------------
    // (P03) os2_scl / os2_done 已在前向声明区声明
    wire [15:0] os2_regout;
    reg  [15:0] os2_cnt;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) os2_cnt <= 16'd0;
        else if (os2_done) os2_cnt <= os2_cnt + 16'd1;
    end

    as5600_i2c #(
        .CLK_DIV       (16'd1000),
        .SLAVE_ADDR    (7'h36),
        .REGISTER_ADDR (8'h0B)
    ) u_as5600_st (
        .rst_n  (rst_n),
        .clk    (clk_50m),
        .scl    (os2_scl),
        .sda    (i2c_sda),
        .en     (STAGE == 15),
        .start  (STAGE == 15),
        .ready  (),
        .done   (os2_done),
        .regout (os2_regout)
    );
    // ==================================================================
    // STAGE=16 / 19：AS5600 磁场质量诊断（抓 AGC / MAGNITUDE / STATUS / ANGLE）
    //   同一个排队器，两种节拍：
    //     STAGE=16  每读完一个寄存器停 50 ms（原来就是 50 ms 一换，保持兼容）
    //     STAGE=19  每读完一个寄存器只停 40 us（= I2C 的 tBUF 总线空闲），
    //               一圈 4 个寄存器约 16 ms，足够一边移动磁铁一边看数
    //   ★ 关键修复：原来 en 一直拉高 ⇒ 上一笔 STOP 结束的下一拍就发新的 START，
    //     总线空闲只有几十 ns，远小于 I2C 要求的 tBUF(4.7us)。从机可能漏掉
    //     START / 认错字节。现在读完必须等 40us 才发下一笔。
    // ==================================================================
    reg  [1:0]  fld_sel;          // 当前轮到哪个寄存器
    reg         fld_run;          // 1 = 该寄存器正在读（en）
    reg  [21:0] fld_tick;         // 读完之后的空闲计时
    reg  [15:0] fld_err_cnt;      // 累计 NACK/超时笔数

    wire        fld_on   = (STAGE == 16) || FLD_DIAG;
    wire        fld_dn   = fld_dn_st | fld_dn_ag | fld_dn_mg | fld_dn_an;
    wire [21:0] fld_dwell = (STAGE == 16) ? 22'd2_499_999      // 50 ms @50MHz
                                          : 22'd2_000;         // 40 us @50MHz
    // STAGE=20/21 把四个读取器改指向【出厂默认值恒定】的寄存器（见前向声明区的 localparam）：
    //   0x03 MPOS_HI 应为 0x0F    0x04 MPOS_LO 应为 0xFF
    //   0x05 MANG_HI 应为 0x0F    0x06 MANG_LO 应为 0xFF
    // ⇒ 正确时遥测应恒为  A4095 T0255 E+4095 M+0255
    //   任何跳变 = 读通路有错（地址位/数据位错），与磁场无关；
    //   恒定正确 = 读通路干净，且芯片是未被烧写的正品 AS5600。
    // 只看“当前这一笔”的 NACK 标志（别的实例的 err 是上一轮的残留）
    wire        fld_err_sel = (fld_sel == 2'd0) ? fld_er_st :
                              (fld_sel == 2'd1) ? fld_er_ag :
                              (fld_sel == 2'd2) ? fld_er_mg : fld_er_an;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            fld_sel <= 2'd0; fld_run <= 1'b0; fld_tick <= 22'd0; fld_err_cnt <= 16'd0;
        end else if (!fld_on) begin
            fld_sel <= 2'd0; fld_run <= 1'b0; fld_tick <= 22'd0;
        end else if (!fld_run) begin
            if (fld_tick >= fld_dwell) begin
                fld_tick <= 22'd0;
                fld_run  <= 1'b1;
            end else begin
                fld_tick <= fld_tick + 22'd1;
            end
        end else if (fld_dn) begin
            fld_run <= 1'b0;                       // 放掉总线，进入空闲期
            fld_sel <= fld_sel + 2'd1;
            if (fld_err_sel) fld_err_cnt <= fld_err_cnt + 16'd1;
        end
    end

    // (P03) fld_scl_st / _ag / _mg / _an 已在前向声明区声明
    wire fld_en_st = fld_on && fld_run && (fld_sel == 2'd0);
    wire fld_en_ag = fld_on && fld_run && (fld_sel == 2'd1);
    wire fld_en_mg = fld_on && fld_run && (fld_sel == 2'd2);
    wire fld_en_an = fld_on && fld_run && (fld_sel == 2'd3);

    as5600_i2c #(.CLK_DIV(FLD_CLKDIV), .SLAVE_ADDR(7'h36), .REGISTER_ADDR(FLD_RA_ST)) u_fld_st (
        .rst_n(rst_n), .clk(clk_50m), .scl(fld_scl_st), .sda(i2c_sda),
        .en(fld_en_st), .start(fld_en_st),
        .ready(), .done(fld_dn_st), .regout(fld_ro_st), .err(fld_er_st));
    as5600_i2c #(.CLK_DIV(FLD_CLKDIV), .SLAVE_ADDR(7'h36), .REGISTER_ADDR(FLD_RA_AG)) u_fld_ag (
        .rst_n(rst_n), .clk(clk_50m), .scl(fld_scl_ag), .sda(i2c_sda),
        .en(fld_en_ag), .start(fld_en_ag),
        .ready(), .done(fld_dn_ag), .regout(fld_ro_ag), .err(fld_er_ag));
    as5600_i2c #(.CLK_DIV(FLD_CLKDIV), .SLAVE_ADDR(7'h36), .REGISTER_ADDR(FLD_RA_MG)) u_fld_mg (
        .rst_n(rst_n), .clk(clk_50m), .scl(fld_scl_mg), .sda(i2c_sda),
        .en(fld_en_mg), .start(fld_en_mg),
        .ready(), .done(fld_dn_mg), .regout(fld_ro_mg), .err(fld_er_mg));
    as5600_i2c #(.CLK_DIV(FLD_CLKDIV), .SLAVE_ADDR(7'h36), .REGISTER_ADDR(FLD_RA_AN)) u_fld_an (
        .rst_n(rst_n), .clk(clk_50m), .scl(fld_scl_an), .sda(i2c_sda),
        .en(fld_en_an), .start(fld_en_an),
        .ready(), .done(fld_dn_an), .regout(fld_ro_an), .err(fld_er_an));

    // 锁存（在各自 done 时更新，避免读到失效值）
    //   ANGLE / MAGNITUDE 是 2 字节事务：首字节在 regout[15:8]，次字节在 regout[7:0]
    //   ⇒ 12 位值 = {regout[11:8], regout[7:0]}
    reg [7:0]  fld_status, fld_agc;
    reg [11:0] fld_mag, fld_angle;
    reg        fld_st_bad;                        // STATUS 那一笔是否 NACK
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            fld_status<=8'd0; fld_agc<=8'd0; fld_mag<=12'd0; fld_angle<=12'd0;
            fld_st_bad<=1'b0;
        end else begin
            if (fld_dn_st) begin
                fld_status <= fld_ro_st[15:8];
                fld_st_bad <= fld_er_st;
            end
            if (fld_dn_ag) fld_agc   <= fld_ro_ag[15:8];
            if (fld_dn_mg) fld_mag   <= fld_ro_mg[11:0];
            if (fld_dn_an) fld_angle <= fld_ro_an[11:0];
        end
    end
    // ==================================================================
    // STAGE=17：串口命令诊断 —— 看 FPGA 到底收到了什么
    //   A = 累计收到的字节数        T = motor_en（发 E1 后应为 1）
    //   E = 最近一个字节的 ASCII    M = tgt_pos（发 P300 后应为 300）
    // ==================================================================
    reg [7:0]  dbg_rx_last;
    reg [15:0] dbg_rx_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin dbg_rx_last <= 8'd0; dbg_rx_cnt <= 16'd0; end
        else if (rx_valid) begin
            dbg_rx_last <= rx_data;
            dbg_rx_cnt  <= dbg_rx_cnt + 16'd1;
        end
    end
    // ==================================================================
    // STAGE=18：自动循环电机测试（完全不需要串口命令 ✓）
    //   周期 10 秒：
    //     0~1s  待机（STBY=0）
    //     1~4s  E1 + P+300（正转 30%）→ PWMA 应约 1.0V
    //     4~5s  停止（P0）
    //     5~8s  P-300（反转 30%）    → AIN1/AIN2 应翻转
    //     8~10s 停止 + E0（断电）
    //   遥测 M = 当前实际控制量（应看到 +300 / 0 / -300 循环 ✓）
    // ==================================================================
    reg [28:0] auto_cnt;                       // 0 ~ 10s @50MHz（需 29 位：500e6 < 2^29 ✓）
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) auto_cnt <= 29'd0;
        else if (auto_cnt >= 29'd500_000_000) auto_cnt <= 29'd0;
        else auto_cnt <= auto_cnt + 29'd1;
    end

    wire [3:0] auto_sec = auto_cnt / 29'd50_000_000;   // 0..9 秒
    // (P03) auto_en / auto_cmd 已在前向声明区声明，这里改成 assign
    assign auto_en  = (STAGE == 18) && (auto_sec >= 4'd1) && (auto_sec <= 4'd8);
    wire auto_fwd = (STAGE == 18) && (auto_sec >= 4'd1) && (auto_sec <= 4'd3);
    wire auto_rev = (STAGE == 18) && (auto_sec >= 4'd5) && (auto_sec <= 4'd7);
    assign auto_cmd = auto_fwd ? 16'sd600 : (auto_rev ? -16'sd600 : 16'sd0);   // 60% 占空，保证能起转
    // STAGE=9 引脚探测：A 字段 = 80~87 的实时电平（bit0=80 ... bit7=87）
    wire [7:0] pin_map = {enc_b, enc_a, probe};
    // ---- STAGE=11：I2C 线路诊断 ----
    //      A = sda_in 当前电平（空闲应为 1）  T = 已完成的读寄存器轮数
    //      M = 通信错误（-999 = 有 NACK）
    reg [15:0] rd_cycles;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) rd_cycles <= 16'd0;
        else if (as5600_valid) rd_cycles <= rd_cycles + 16'd1;
    end
    wire [11:0] tlm_a11 = {11'b0, sda_in};
    wire [9:0]  tlm_t11 = rd_cycles[9:0];

    // ==================================================================
    // ★ v2.3  STAGE=38 / 39：一次捕获把"读通路丢"和"滤波器吃"分开
    //
    //   只看 ang_f 是分不开的：NACK 会让 regout 保持旧值（as5600_i2c.v 里
    //   `if (!nack_seen) regout <= recv_shift`），跳变剔除也会让 ang_f 停住，
    //   两者对 ang_f 产生的是【完全相同的签名】。
    //
    //   所以把三个计数器同时打出来，并且必须【在同一个 1 秒边界一起锁存】，
    //   否则 nack/rps 的比值没有意义：
    //
    //     T = rps        每秒完成的事务数（os_done 硬件计数，与滤波无关）
    //     M = nack_ps    每秒 NACK 数           判据看比值 nack/rps
    //     E = ang12      原始 12 位（未经中值/剔除）   看它的变化率 = 真实刷新率
    //     A = ang_f   (STAGE=38) 控制器实际拿到的值（经中值+剔除）
    //     A = af_upd_ps(STAGE=39) 每秒"被接受"的次数 —— 比看 A 变没变更准：
    //                    慢速运动时中值会接受一个和旧值相等的样本，
    //                    A 不变但确实接受了，只看 A 会把这种情况算成"丢了"。
    //
    //   判读表（采集时必须手动缓慢、大范围转摆杆，覆盖 0~4095）：
    //     nack≈0、E 变得勤、af_upd≈20  -> 滤波器在吃   -> 改 ANG_MAX_JUMP/ANG_REJ_MAX
    //     nack≈40、E 也变得稀          -> 总线在丢     -> 先修物理层，别碰控制律
    //     nack≈0、E 也不变、rps 低     -> 事务没跑够   -> 查 os_done 实际频率
    // ==================================================================
    reg [31:0] cnt_os   = 32'd0;
    reg [31:0] cnt_nack = 32'd0;
    reg [31:0] cnt_acc  = 32'd0;
    reg [9:0]  rps        = 10'd0;
    reg [9:0]  nack_ps    = 10'd0;
    reg [9:0]  af_upd_ps  = 10'd0;
    reg [25:0] rps_1s     = 26'd0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_os <= 32'd0; cnt_nack <= 32'd0; cnt_acc <= 32'd0;
            rps <= 10'd0; nack_ps <= 10'd0; af_upd_ps <= 10'd0; rps_1s <= 26'd0;
        end else begin
            if (os_done)            cnt_os   <= cnt_os   + 32'd1;
            if (os_done && os_er)   cnt_nack <= cnt_nack + 32'd1;
            if (os_done && ang_acc) cnt_acc  <= cnt_acc  + 32'd1;

            if (rps_1s >= 26'd49_999_999) begin
                rps_1s    <= 26'd0;
                // 三个数在同一个时钟沿一起锁存 -> 永远是同一秒窗口
                rps       <= (cnt_os   > 32'd1023) ? 10'd1023 : cnt_os[9:0];
                nack_ps   <= (cnt_nack > 32'd1023) ? 10'd1023 : cnt_nack[9:0];
                af_upd_ps <= (cnt_acc  > 32'd1023) ? 10'd1023 : cnt_acc[9:0];
                cnt_os <= 32'd0; cnt_nack <= 32'd0; cnt_acc <= 32'd0;
            end else begin
                rps_1s <= rps_1s + 26'd1;
            end
        end
    end

    // STAGE=37 分量诊断的定标（A/T 字段无符号，需加偏置）：
    //   A = a_err  + 2048   （12 位 count，偏置 2048 ⇒ 0..4095 表示 -2048..+2047）
    //   T = a_rate + 512    （10 位字段，偏置 512  ⇒ 0..1023 表示 -512..+511）
    wire [11:0] dbg_ae_u = pd_a_err + 13'sd2048;
    wire [13:0] dbg_ar_s = {pd_a_rate[12], pd_a_rate} + 14'sd512;
    wire [9:0]  dbg_ar_u = (dbg_ar_s[13])          ? 10'd0 :
                           (dbg_ar_s[9:0] > 10'd1023) ? 10'd1023 : dbg_ar_s[9:0];

    // ★ v2.2: STAGE=5 的 A 必须显示【滤波后】的 ang_f，不是原始 os_regout。
    //   原来这里取 os_regout[11:2]，而注释写着"显示控制环实际用的角度"——
    //   两者不一致。后果：操作者看到 A 在 750 和 0/16/242 之间乱跳（那是
    //   I2C 读错的原始值），误以为摆杆不稳，其实控制器吃的是被中值+跳变
    //   剔除保护过的 ang_f。判断闭环是否稳定，必须看 ang_f。
    //   想看原始读数用 STAGE=38 的 E 字段。
    wire [11:0] tlm_a = ((STAGE == 5) || (STAGE == 34)) ? {2'b0, ang_f[11:2]} :   // 倒立摆：显示控制环实际用的角度
                        (STAGE == 36) ? {2'b0, ang_f[11:2]} :
                        (STAGE == 37) ? dbg_ae_u :
                        (STAGE == 38) ? ang_f :                     // 滤波后 12 位（对比 E）
                        (STAGE == 39) ? {2'b0, af_upd_ps} :         // ★ 每秒被接受的次数
                        (STAGE == 7) ? as5600_angle :
                        (STAGE == 5) ? {2'b0, angle_src} :        // 倒立摆：实测摆杆角度（10 位，经滤波）
                        (STAGE == 9) ? {4'b0, pin_map} :
                        (STAGE == 10) ? st_mang :
                        (STAGE == 11) ? tlm_a11 :
                        (STAGE == 12) ? {8'b0, dbg_i2c_state} :
                        (STAGE == 13) ? ang12 :
                        (STAGE == 14) ? {5'b0, scan_addr} :
                        (STAGE == 15) ? {4'b0, os2_regout[15:8]} :
                        (STAGE == 16) ? {4'b0, fld_agc} :
                        (STAGE == 17) ? dbg_rx_cnt[11:0] :
                        (STAGE == 18) ? {8'b0, auto_sec} :
                        FLD_DIAG ? fld_angle : {2'b0, adc_val};
    // STAGE=7 诊断：T 上报磁场幅值(MAGNITUDE 高8位)，E 上报 STATUS（bit5=检测到磁铁），M 上报 AGC；
    //               若 I2C 通信失败则两者都上报 -999，一眼可辨
    wire [9:0] tlm_t = (STAGE == 7)  ? {2'b0, as5600_mag[11:4]} :
                       (STAGE == 10) ? {2'b0, st_conf_hi} :
                       (STAGE == 11) ? tlm_t11 :
                       (STAGE == 12) ? {8'b0, dbg_st} :
                       (STAGE == 13) ? os_cnt[9:0] :          // 读取计数（看刷新率有没有在涨）
                       (STAGE == 14) ? {2'b0, scan_ack_cnt} :
                       (STAGE == 15) ? os2_cnt[9:0] :
                       (STAGE == 16) ? {2'b0, fld_status} :
                       FLD_DIAG ? {2'b0, fld_agc} :
                       (STAGE == 17) ? {9'b0, motor_en} :
                       (STAGE == 36) ? tgt12[11:2] :      // 设定点自检：T 就是 'Z' 捕获到的平衡点
                       (STAGE == 37) ? dbg_ar_u :         // 分量诊断：a_rate + 512
                       ((STAGE == 38) || (STAGE == 39)) ? rps :   // 每秒完成的事务数（硬件计数）
                       (STAGE == 5) ? {7'b0, en_final, pos_lock, motor_en} : tgt_angle;
    wire signed [15:0] tlm_e = (STAGE == 7)
                             ? (as5600_err ? -16'sd999 : ({8'b0, as5600_status}))
                             : (STAGE == 10) ? $signed({4'b0, st_mpos}) :
                               (STAGE == 12) ? $signed({12'b0, dbg_step}) :
                               (STAGE == 14) ? $signed({9'b0, scan_first}) :
                               (STAGE == 16) ? $signed({4'b0, fld_mag}) :
                               FLD_DIAG ? $signed({4'b0, fld_mag}) :
                               (STAGE == 37) ? $signed({2'b0, pd_e_rate}) :
                               // STAGE=38/39：E = 原始 12 位（未经中值、未经跳变剔除）。
                               //   ★ v2.3 去掉了原来"os_er 时报 -999"的写法：err 只有在
                               //   事务结束到下一笔开始之间才有效（约 120us / 16.4ms = 0.73%
                               //   占空比），100 Hz 遥测几乎永远采不到它，采到了也分不清
                               //   "真 NACK" 和"刚刚读完"。现在改用 M = nack_ps（每秒计数），
                               //   那个才是可用的判据，比值看 nack_ps/rps。
                               ((STAGE == 38) || (STAGE == 39)) ? $signed({4'b0, ang12}) :
                               (STAGE == 17) ? $signed({8'b0, dbg_rx_last}) : enc_pos;
    wire signed [15:0] tlm_m7 = (as5600_err ? -16'sd999 : ({8'b0, as5600_agc}));
    wire signed [15:0] tlm_m10 = as5600_err ? -16'sd999 : $signed({4'b0, st_zpos});  // -999 = I2C 通信失败
    // ------------------------------------------------------------------
    // 串口遥测
    // ------------------------------------------------------------------
    wire [7:0]  tx_data;
    wire        tx_send;
    wire        tx_busy;

    // STAGE=6 上报 DAC 送出的码值（与 A 配对看环回）；STAGE=5 上报电机控制量；
    // 其它阶段上报舵机脉宽，方便沿用第四阶段的分析
    wire signed [15:0] m_field = (STAGE == 6) ? $signed({6'b0, dac_code})
                              // ★ v2.3  STAGE=38/39 的 M 改成【每秒 NACK 数】
                              //   （原来是 mot_cmd_act，闭环没使能时恒 0，白白浪费一个字段）
                              : ((STAGE == 38) || (STAGE == 39)) ? $signed({6'b0, nack_ps})
                              : ((STAGE == 5) || (STAGE == 8) || (STAGE == 18) || (STAGE == 34)
                                 || (STAGE == 36) || (STAGE == 37)) ? mot_cmd_act
                              : (STAGE == 7) ? tlm_m7
                              : (STAGE == 10) ? tlm_m10
                              : (STAGE == 12) ? (as5600_err ? -16'sd999 : 16'sd0)
                              : (STAGE == 14) ? $signed({8'b0, scan_done_cnt})
                              : (STAGE == 16) ? $signed({4'b0, fld_angle})
                              : FLD_DIAG ? (fld_st_bad ? -16'sd999
                                                           : $signed({8'b0, fld_status}))
                              : (STAGE == 17) ? tgt_pos
                                             : $signed({4'b0, servo_pulse_us});

    telemetry #(
        .CLK_HZ    (50_000_000),
        .PERIOD_MS (10)
    ) u_tlm (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        .adc_val      (tlm_a),
        .angle_target (tlm_t),
        .enc_pos      (tlm_e),
        .motor_cmd    (m_field),
        .tx_data      (tx_data),
        .tx_send      (tx_send),
        .tx_busy      (tx_busy)
    );

    uart_tx #(
        .CLK_HZ (50_000_000),
        .BAUD   (115200)
    ) u_tx (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .data  (tx_data),
        .send  (tx_send),
        .tx    (uart_tx),
        .busy  (tx_busy)
    );

    // ------------------------------------------------------------------
    // LED
    //   STAGE=1：1Hz 心跳
    //   STAGE=5：显示编码器位置低 8 位（自检时应该能看到二进制计数）
    //   其它    ：显示 ADC 高 8 位
    // ------------------------------------------------------------------
    reg [24:0] blink_cnt = 25'd0;
    reg        blink     = 1'b0;

    always @(posedge clk_50m) begin
        if (blink_cnt == 25'd12_499_999) begin    // 0.25s 翻转 = 2Hz 闪（比 1Hz 更容易看清）
            blink_cnt <= 25'd0;
            blink     <= ~blink;
        end else begin
            blink_cnt <= blink_cnt + 1'b1;
        end
    end

    // ------------------------------------------------------------------
    //  STAGE=0  硬件探针（上板急救用）
    //    绕过 LED 极性取反，直接给引脚原始电平，图案固定为：
    //      led[0] = blink        led[4] = 0
    //      led[1] = ~blink       led[5] = blink
    //      led[2] = 0            led[6] = 0
    //      led[3] = 1            led[7] = 1
    //    → 2 个常高 + 3 个常低 + 3 个闪（led0/led1 互补）
    //    判读：
    //      全灭                 = FPGA 没被配置（比特流没真正下载进去）
    //      有亮有灭但不闪        = 已配置，但 50MHz 时钟没来（时钟故障）
    //      有明显闪烁            = 时钟正常、设计在跑（问题在别处）
    // ------------------------------------------------------------------
    wire [7:0] probe_led = {1'b1, 1'b0, blink, 1'b0, 1'b1, 1'b0, ~blink, blink};

    wire [7:0] adc_hi = adc_val[9:2];
    reg  [7:0] led_val;

    always @(*) begin
        if (STAGE == 1)
            led_val = {8{blink}};
        else if ((STAGE == 5) || (STAGE == 8))
            led_val = enc_pos[7:0];
        else if (STAGE == 7)
            led_val = as5600_angle[11:4];
        else if (STAGE == 9)
            led_val = pin_map;          // LED0..7 = 80..87 号脚电平
        else if (STAGE == 30)
            led_val = {blink, 5'b0, mt_rev, mt_fwd};   // 心跳 + 当前方向指示
        else if (STAGE == 31)
            led_val = {blink, 4'b0, saf_idx};          // 心跳 + 当前组合编号(0..5)
        else if (FLD_DIAG)
            // 磁场强度条：MAGNITUDE 越大灯越亮（磁铁越近/越正）
            led_val = (fld_mag >= 12'd1024) ? 8'hFF : fld_mag[9:2];
        else
            led_val = adc_hi;
    end

    assign led = (STAGE == 0) ? probe_led
               : (STAGE == 17) ? {1'b0, ~key}                  // STAGE=7 诊断：按 K1~K7 -> led[0..6] 亮                     // 探针：原始电平，不取反
                            : (LED_ACTIVE_LOW ? ~led_val : led_val);

    // ---- STAGE=7 诊断输出：四个数码管全亮（本 STAGE 不驱动电机）----
    assign smg = (STAGE == 17) ? 8'h00 : 8'hFF;       // 低有效 => 0 = 段全亮
    assign dig = (STAGE == 17) ? 4'b0000 : 4'b1111;   // 低有效 => 0 = 位全选
endmodule