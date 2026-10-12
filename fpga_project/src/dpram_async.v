// 异步双口 RAM：摄像头 PCLK 写，LCD 像素时钟读。
// 必须推断成 BSRAM。800x480x16 会超过 GW5AT-138 的 340 块 BSRAM。
module dpram_async #(
    parameter DATA_W = 12,
    parameter ADDR_W = 19,
    parameter DEPTH  = 800*480
) (
    input  wire              wr_clk,
    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,
    input  wire [DATA_W-1:0] wr_data,
    input  wire              rd_clk,
    input  wire [ADDR_W-1:0] rd_addr,
    output reg  [DATA_W-1:0] rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end
endmodule
