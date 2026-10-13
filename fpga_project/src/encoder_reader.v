// =====================================================================
// encoder_reader —— 增量式编码器读取（A/B 四倍频 + M/T 法测速）
//
// 【为什么必须用 M/T 法】
// 原来用的是 M 法：固定 10ms 闸门里数脉冲。
//   10 RPM、1320CPR -> 每秒 220 个计数 -> 每 10ms 只有 2.2 个脉冲。
//   整数化后读数在 2/3 之间乱跳，再慢就直接显示 0 —— 低速完全不可用。
//
// M/T 法：在数脉冲的同时，精确测量"这些脉冲之间实际经过了多长时间"：
//   速度 = 脉冲数 / 实测时间(us) * 1e6
// 低速时脉冲少，但时间测得准（微秒级），所以分辨率大幅提升。
//
// 【输出单位】计数/秒（counts per second，带符号）
//   RPM = cps * 60 / CPR
//   CPR=1320 时：RPM = cps * 0.04545      （cps = 1100 -> 50 RPM）
//   本模块分辨率约 1 cps ≈ 0.045 RPM
//
// 【其他】
//   GATE_MS 是刷新周期（默认 10ms -> 100Hz 更新），不再是测量精度瓶颈。
//   每闸门结束时算一次：脉冲数 m、实测时间 tau(us)，speed = m*1e6/tau。
//   除法用 div_seq 顺序除法器，32 拍算完，对时序无压力。
//   DIR_SWAP：电机线或编码器线接反导致方向相反时置 1。
// =====================================================================
module encoder_reader #(
    parameter CLK_FREQ = 50_000_000,
    parameter GATE_MS  = 10,
    parameter DIR_SWAP = 0
) (
    input  wire               Clk,
    input  wire               Rst_n,
    input  wire               enc_a,
    input  wire               enc_b,
    output wire signed [31:0] pos,          // 位置累计（4 倍频计数）
    // syn_keep: 便于 GAO 在线逻辑分析仪观察
    (* syn_keep = 1 *) output reg  signed [15:0] speed,   // 计数/秒（带符号）
    (* syn_keep = 1 *) output reg                speed_valid
);

    // ---------------- A/B 同步 ----------------
    reg [2:0] a_r, b_r;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            a_r <= 3'b000;
            b_r <= 3'b000;
        end else begin
            a_r <= {a_r[1:0], enc_a};
            b_r <= {b_r[1:0], enc_b};
        end
    end
    wire a = a_r[2];
    wire b = b_r[2];

    // ---------------- 4 倍频正交解码 ----------------
    // 正转序列: 00 -> 01 -> 11 -> 10 -> 00
    // 反转序列: 00 -> 10 -> 11 -> 01 -> 00
    // 两位同时跳变视为非法（毛刺），不计数。
    (* syn_keep = 1 *) reg        st_init;
    reg [1:0]  st;
    (* syn_keep = 1 *) reg signed [31:0] cnt;

    wire [3:0] trans = {st, a, b};

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            st_init <= 1'b0;
            st      <= 2'b00;
            cnt     <= 32'd0;
        end else if (!st_init) begin
            // 上电先锁定当前象限，避免把初始状态误判成一次跳变
            st_init <= 1'b1;
            st      <= {a, b};
        end else begin
            st <= {a, b};
            case (trans)
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00:
                    cnt <= cnt + (DIR_SWAP ? -32'sd1 : 32'sd1);
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00:
                    cnt <= cnt - (DIR_SWAP ? -32'sd1 : 32'sd1);
                default:
                    cnt <= cnt;   // 同态保持 / 非法跳变
            endcase
        end
    end

    assign pos = cnt;

    // ---------------- 边沿检测（cnt 变化 = 有编码器边沿）----------------
    reg signed [31:0] cnt_d;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) cnt_d <= 32'sd0;
        else        cnt_d <= cnt;
    end
    wire enc_edge = (cnt != cnt_d);

    // ---------------- 微秒时间戳 ----------------
    localparam US_DIV = CLK_FREQ / 1_000_000;      // 50MHz -> 50
    reg [5:0]  us_cnt;
    reg [31:0] ts;                                 // 单位 us，自由运行
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            us_cnt <= 6'd0;
            ts     <= 32'd0;
        end else if (us_cnt >= US_DIV - 1) begin
            us_cnt <= 6'd0;
            ts     <= ts + 1'b1;
        end else begin
            us_cnt <= us_cnt + 1'b1;
        end
    end

    // ---------------- 闸门（只决定刷新率，不再是精度瓶颈）----------------
    localparam GATE_CNT  = CLK_FREQ / 1000 * GATE_MS;
    localparam GATE_US   = GATE_CNT / US_DIV;      // 闸门长度（us）

    reg [31:0] gate_cnt;
    wire       gate_end = (gate_cnt >= GATE_CNT - 1);
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) gate_cnt <= 32'd0;
        else if (gate_end) gate_cnt <= 32'd0;
        else gate_cnt <= gate_cnt + 1'b1;
    end

    // ---------------- M/T 测量 ----------------
    reg [31:0] t_ref;      // 上一个窗口最后一个边沿的时刻
    reg [31:0] t_last;     // 本窗口最后一个边沿的时刻
    reg [15:0] m;          // 本窗口边沿数
    reg        have_ref;
    reg        neg_lat;    // 本窗口的方向
    reg signed [31:0] cnt_w0;   // 窗口起始位置
    reg [31:0] t_stale;    // 距上一个边沿已经过了多久(us)

    // 连续这么久没有边沿 -> 判定真的停转了，速度清 0
    localparam STALE_US = 150_000;      // 150ms

    wire [31:0] tau   = t_last - t_ref;
    wire [31:0] m_use = (m > 16'd4000) ? 32'd4000 : {16'b0, m};

    // ---- 除法器 ----
    // 注意：div_seq 在运算的 32 拍里会一直读 num/den，
    //       所以必须先把参数锁存到 num_r/den_r，下一拍再发 start。
    //       （如果同一拍既发 start 又把 m/t_ref 清掉，除法器会拿到 0，
    //         结果商恒为 0 —— 这正是"数值不跳动"的原因）
    reg         div_req;      // 参数已锁存，待启动
    reg         div_start;    // 单拍启动
    reg  [31:0] num_r, den_r;
    wire [31:0] div_quo;
    wire        div_done;
    reg         meas_pend;

    div_seq u_div (
        .Clk  (Clk),
        .Rst_n(Rst_n),
        .start(div_start),
        .num  (num_r),
        .den  (den_r),
        .quo  (div_quo),
        .done (div_done)
    );

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            t_ref <= 32'd0; t_last <= 32'd0; m <= 16'd0; have_ref <= 1'b0;
            neg_lat <= 1'b0; cnt_w0 <= 32'sd0; t_stale <= 32'd0;
            div_req <= 1'b0; div_start <= 1'b0;
            num_r <= 32'd0; den_r <= 32'd1; meas_pend <= 1'b0;
            speed <= 16'sd0; speed_valid <= 1'b0;
        end else begin
            speed_valid <= 1'b0;
            div_start   <= 1'b0;

            // ---- 参数已锁存，下一拍真正启动除法器 ----
            if (div_req) begin
                div_req   <= 1'b0;
                div_start <= 1'b1;
            end

            // ---- 编码器边沿 ----
            // 注意：上电后遇到的"第一个边沿"只用来建立参考点 t_ref，
            //       不能计入 m。否则第一个窗口里 τ 只跨了 m-1 个间隔却除以 m，
            //       速度会偏大 m/(m-1) 倍（m=2 时直接偏大 100%）。
            if (enc_edge) begin
                t_last  <= ts;
                // 任何边沿都要重置"静止计时"。
                // 否则从上电起就一直累加，150ms 后误判停转，
                // 而停转分支一旦触发就再也恢复不了 —— 低速会永远读 0。
                t_stale <= 32'd0;
                if (!have_ref) begin
                    t_ref    <= ts;
                    have_ref <= 1'b1;
                end else begin
                    if (m < 16'd60000) m <= m + 1'b1;
                end
            end

            // ---- 闸门结束 ----
            if (gate_end) begin
                if (m > 16'd0 && have_ref && tau != 32'd0) begin
                    // 有新边沿：锁存 M/T 参数（m 个脉冲 / 实测 tau 微秒）
                    num_r     <= m_use * 32'd1_000_000;
                    den_r     <= tau;
                    div_req   <= 1'b1;
                    meas_pend <= 1'b1;
                    neg_lat   <= ((cnt - cnt_w0) < 32'sd0);
                    t_ref     <= t_last;   // 参考点推进到本窗口最后一个边沿
                end else if (t_stale >= STALE_US) begin
                    // 超过 150ms 一个边沿都没有 -> 确实停转
                    speed       <= 16'sd0;
                    speed_valid <= 1'b1;
                    // 清掉参考点：否则停转很久后重新起步，
                    // 第一个 τ 会横跨整个停机时间，读数离谱。
                    // 置 0 后下一个边沿只当参考点，之后就是准的。
                    have_ref    <= 1'b0;
                end else begin
                    // 低速时本闸门可能一个边沿都没有 -> 保持上次速度
                    // （这就是低速读数能稳住的原因，t_ref 不动，τ 会跨窗口累加）
                    t_stale <= t_stale + GATE_US;
                end
                cnt_w0 <= cnt;
                m      <= 16'd0;
            end

            // ---- 除法完成：输出速度（计数/秒）----
            if (div_done && meas_pend) begin
                meas_pend <= 1'b0;
                if (neg_lat)
                    speed <= (div_quo > 32'd32767) ? -16'sd32767 : -div_quo[15:0];
                else
                    speed <= (div_quo > 32'd32767) ?  16'sd32767 :  div_quo[15:0];
                speed_valid <= 1'b1;
            end
        end
    end

endmodule
