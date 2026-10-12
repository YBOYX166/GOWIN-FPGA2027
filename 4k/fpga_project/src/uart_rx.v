// =====================================================================
// uart_rx —— 8N1 串口接收器（和 uart_tx.v 配对使用）
//
//   空闲时 rx 为高。检测到下降沿 -> 起始位，然后在每一位的**中间**采样。
//   收到一个完整字节（且停止位为高）时，valid 拉高一个时钟。
//
//   采样时序：
//     起始位中点确认（等 DIV/2）
//     -> 之后每 DIV 个时钟采 1 位数据（即落在 3DIV/2, 5DIV/2, ... 中点）
//     -> 停止位
//
//   校验：停止位不是高电平则丢弃，不产生 valid（帧错误）。
//
// 【移植】CLK_FREQ 参数化，27MHz(4K) / 50MHz(138K) 都直接用。
//   115200 波特率误差：
//     27MHz -> DIV=234, 实际 115385, 误差 +0.16%
//     50MHz -> DIV=434, 实际 115207, 误差 +0.01%
//   都在 UART 允许的 ±2~3% 以内。
// =====================================================================
`timescale 1ns / 1ps

module uart_rx #(
    parameter CLK_FREQ = 27_000_000,
    parameter BAUD     = 115200
)(
    input  wire       Clk,
    input  wire       Rst_n,
    input  wire       rx,      // 串口输入（空闲高）
    output reg  [7:0] data,    // 收到的字节
    output reg        valid    // 单拍脉冲：data 有效
);

    localparam DIV  = CLK_FREQ / BAUD;
    localparam HALF = DIV / 2;

    // ---------------- 输入同步（防亚稳态）----------------
    reg [1:0] sync;
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) sync <= 2'b11;
        else        sync <= {sync[0], rx};
    end
    wire rx_s = sync[1];

    // ---------------- 接收状态机 ----------------
    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  st;
    reg [15:0] cnt;
    reg [2:0]  bi;
    reg [7:0]  sh;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            st    <= S_IDLE;
            cnt   <= 16'd0;
            bi    <= 3'd0;
            sh    <= 8'd0;
            data  <= 8'd0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (st)
                // 等起始位下降沿
                S_IDLE: begin
                    if (!rx_s) begin
                        st  <= S_START;
                        cnt <= 16'd0;
                    end
                end

                // 半位后确认确实是起始位（滤掉毛刺）
                S_START: begin
                    if (cnt == HALF - 1) begin
                        cnt <= 16'd0;
                        if (!rx_s) begin
                            st <= S_DATA;
                            bi <= 3'd0;
                        end else begin
                            st <= S_IDLE;      // 假起始位，回去
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // 每 DIV 个时钟采 1 位（低位先到）
                S_DATA: begin
                    if (cnt == DIV - 1) begin
                        cnt <= 16'd0;
                        sh  <= {rx_s, sh[7:1]};
                        if (bi == 3'd7) st <= S_STOP;
                        else            bi <= bi + 1'b1;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // 停止位：必须是高，否则算帧错误丢弃
                S_STOP: begin
                    if (cnt == DIV - 1) begin
                        cnt <= 16'd0;
                        st  <= S_IDLE;
                        if (rx_s) begin
                            data  <= sh;
                            valid <= 1'b1;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
