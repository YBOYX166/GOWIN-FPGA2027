// 800x480 RGB 屏驱动（TTL 24bit）。
// 从 RGB565 帧缓冲全屏读出；摄像头未就绪时出彩条。
module lcd_rgb_driver #(
    parameter H_ACTIVE = 800,
    parameter H_FP     = 40,
    parameter H_SYNC   = 128,
    parameter H_BP     = 88,
    parameter H_TOTAL  = 1056,
    parameter V_ACTIVE = 480,
    parameter V_FP     = 10,
    parameter V_SYNC   = 2,
    parameter V_BP     = 33,
    parameter V_TOTAL  = 525,
    parameter H_POL    = 0,
    parameter V_POL    = 0
) (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire [11:0] rd_data,
    input  wire        cam_valid,
    output wire [18:0] rd_addr,
    output reg  [23:0] lcd_rgb,
    output wire        lcd_de,
    output wire        lcd_hs,
    output wire        lcd_vs,
    output wire        lcd_clk,
    output wire        frame_start
);
    assign lcd_clk = ~pclk;
    assign frame_start = (hcnt == H_TOTAL-1) && (vcnt == V_TOTAL-1);

    reg [10:0] hcnt;
    reg [9:0]  vcnt;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 11'd0;
            vcnt <= 10'd0;
        end else if (hcnt == H_TOTAL-1) begin
            hcnt <= 11'd0;
            vcnt <= (vcnt == V_TOTAL-1) ? 10'd0 : vcnt + 1'b1;
        end else begin
            hcnt <= hcnt + 1'b1;
        end
    end

    wire de = (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
    wire hs = (hcnt >= H_ACTIVE + H_FP) && (hcnt < H_ACTIVE + H_FP + H_SYNC);
    wire vs = (vcnt >= V_ACTIVE + V_FP) && (vcnt < V_ACTIVE + V_FP + V_SYNC);
    assign rd_addr = de ? (vcnt * 19'd800 + {9'd0, hcnt[9:0]}) : 19'd0;

    reg de_r, hs_r, vs_r;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            de_r <= 1'b0;
            hs_r <= 1'b1;
            vs_r <= 1'b1;
        end else begin
            de_r <= de;
            hs_r <= (H_POL ? hs : ~hs);
            vs_r <= (V_POL ? vs : ~vs);
        end
    end

    reg [23:0] test_rgb;
    always @(*) begin
        case (hcnt[9:7])
            3'd0: test_rgb = 24'hFFFFFF;
            3'd1: test_rgb = 24'hFFFF00;
            3'd2: test_rgb = 24'h00FFFF;
            3'd3: test_rgb = 24'h00FF00;
            3'd4: test_rgb = 24'hFF00FF;
            3'd5: test_rgb = 24'hFF0000;
            3'd6: test_rgb = 24'h0000FF;
            default: test_rgb = 24'h000000;
        endcase
    end

    // RGB444 {R[3:0],G[3:0],B[3:0]} -> RGB888
    wire [3:0] r4 = rd_data[11:8];
    wire [3:0] g4 = rd_data[7:4];
    wire [3:0] b4 = rd_data[3:0];
    wire [23:0] rgb888 = {r4, r4, g4, g4, b4, b4};

    always @(*) begin
        if (!cam_valid)
            lcd_rgb = de_r ? test_rgb : 24'h000000;
        else
            lcd_rgb = de_r ? rgb888 : 24'h000000;
    end
    assign lcd_de = de_r;
    assign lcd_hs = hs_r;
    assign lcd_vs = vs_r;
endmodule
