// =====================================================================
// div_seq —— 顺序（逐位恢复余数）除法器，32 位无符号
//
// 用在 M/T 测速里：速度 = 脉冲数 * 1e6 / 实测微秒数
// 每 10ms 才用一次，32 拍算完（0.64us），对时序没有任何压力。
//
//   start 给一个单拍脉冲 -> 32 拍后 done 拉高一拍，quo = num / den
//   den = 0 时 quo 会输出全 1，调用方需自行保证 den != 0
//
// 算法：
//   rem = 0
//   for i = 31 downto 0:
//       rem = (rem << 1) | num[i]
//       if (rem >= den) { rem -= den; quo[i] = 1; }
// =====================================================================
module div_seq (
    input  wire        Clk,
    input  wire        Rst_n,
    input  wire        start,
    input  wire [31:0] num,
    input  wire [31:0] den,
    output reg  [31:0] quo,
    output reg         done
);

    reg [31:0] rem;
    reg [5:0]  i;          // 32 -> 1
    reg        run;

    wire [31:0] idx     = i - 6'd1;              // 当前处理的位号 31..0
    wire [31:0] shifted = {rem[30:0], num[idx[4:0]]};
    wire        ge      = (shifted >= den);

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            run  <= 1'b0;
            done <= 1'b0;
            quo  <= 32'd0;
            rem  <= 32'd0;
            i    <= 6'd0;
        end else begin
            done <= 1'b0;

            if (!run) begin
                if (start) begin
                    run <= 1'b1;
                    quo <= 32'd0;
                    rem <= 32'd0;
                    i   <= 6'd32;
                end
            end else begin
                if (ge) begin
                    rem         <= shifted - den;
                    quo[idx[4:0]] <= 1'b1;
                end else begin
                    rem <= shifted;
                end

                if (i == 6'd1) begin
                    run  <= 1'b0;
                    done <= 1'b1;
                end else begin
                    i <= i - 1'b1;
                end
            end
        end
    end

endmodule
