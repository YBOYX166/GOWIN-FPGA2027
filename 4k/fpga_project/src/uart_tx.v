// =====================================================================
// uart_tx —— 最简单的 8N1 串口发送器
//
//   start 给一个单拍脉冲 -> 把 data 发出去，发完 done 拉高一个时钟
//   baud 默认 115200（50MHz / 434 = 115207，误差 0.006%）
// =====================================================================
module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
) (
    input  wire       Clk,
    input  wire       Rst_n,
    input  wire       start,   // 单拍脉冲
    input  wire [7:0] data,
    output reg        tx,
    output reg        done     // 发送完成，单拍
);

    localparam DIV = CLK_FREQ / BAUD;

    reg [15:0] cnt;
    reg [3:0]  bit_i;
    reg [9:0]  sh;      // {stop, data[7:0], start}，先发 sh[0]
    reg        busy;

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            tx    <= 1'b1;
            done  <= 1'b0;
            busy  <= 1'b0;
            cnt   <= 16'd0;
            bit_i <= 4'd0;
            sh    <= 10'h3FF;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                tx <= 1'b1;                 // 空闲保持高
                if (start) begin
                    sh    <= {1'b1, data, 1'b0};
                    busy  <= 1'b1;
                    cnt   <= 16'd0;
                    bit_i <= 4'd0;
                end
            end else if (cnt == DIV - 1) begin
                cnt <= 16'd0;
                tx  <= sh[0];
                sh  <= {1'b1, sh[9:1]};
                if (bit_i == 4'd9) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    tx   <= 1'b1;           // 停止位
                end else begin
                    bit_i <= bit_i + 1'b1;
                end
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule
