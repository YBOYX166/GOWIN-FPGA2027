/////////////////////////////////////////////////////////////////////////////////
// Company       : 姝︽眽鑺�璺�鎭掔�戞妧鏈夐檺鍏�鍙�
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : camera_init
// Description   : 鎽勫儚澶村垵濮嬪寲椹卞姩妯″潡,鏀�鎸乷v5640鍜宱v7725
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module camera_init #(
  parameter SYS_CLOCK       = 50_000_000,//绯荤粺鏃堕挓閲囩敤50MHz
  parameter SCL_CLOCK       = 400_000,   //SCL鎬荤嚎鏃堕挓閲囩敤400kHz
  parameter CAMERA_TYPE     = "ov5640",  //"ov5640" or "ov7725"
  parameter IMAGE_TYPE      = 0,         //0:RGB 1:JPEG
  parameter IMAGE_WIDTH     = 640,       //浠卭v5640鏈夋晥
  parameter IMAGE_HEIGHT    = 480,       //浠卭v5640鏈夋晥
  parameter IMAGE_FLIP_EN   = 0,
  parameter IMAGE_MIRROR_EN = 0
)
(
  input      Clk,
  input      Rst_n,

  output reg Init_Done,
  output     camera_rst_n,
  output     camera_pwdn,

  output     i2c_sclk,
  inout      i2c_sdat
);
  localparam RGB = 0;
  localparam JPEG = 1;
  localparam DELAY_1ms_CNT  = 1_000_000/(1_000_000_000/SYS_CLOCK);
  localparam DELAY_21ms_CNT = 21_000_000/(1_000_000_000/SYS_CLOCK);
  localparam DELAY_5ms_CNT  = 5_000_000/(1_000_000_000/SYS_CLOCK);

  wire [15:0]addr;
  reg wrreg_req;
  reg rdreg_req;
  wire [7:0]wrdata;
  wire [7:0]rddata;
  wire RW_Done;
  wire ack;
  reg [31:0] i2c_dly_cnt_max;

  reg [7:0]cnt;
  wire [23:0]lut;

  wire [7:0]lut_size;
  wire [7:0]device_id;
  wire addr_mode;

  assign camera_pwdn = 0;

  generate
  ////////////////////////////ov5640///////////////////////////////
  if(CAMERA_TYPE == "ov5640")
  begin
    assign device_id = 8'h78;
    assign addr_mode = 1'b1;
    assign addr = lut[23:8];
    assign wrdata = lut[7:0];

    if(IMAGE_TYPE == RGB) //------RGB------
    begin
      assign lut_size = 252;

      ov5640_init_table_rgb #(
        .IMAGE_WIDTH     (IMAGE_WIDTH     ),
        .IMAGE_HEIGHT    (IMAGE_HEIGHT    ),
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN   ),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN )
      )ov5640_init_table_rgb_inst
      (
        .addr (cnt ),
        .clk  (Clk ),
        .q    (lut )
      );
    end
    else  //------JPEG------
    begin
      assign lut_size = 250;

      ov5640_init_table_jpeg #(
        .IMAGE_WIDTH     (IMAGE_WIDTH     ),
        .IMAGE_HEIGHT    (IMAGE_HEIGHT    ),
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN   ),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN )
      )ov5640_init_table_jpeg_inst
      (
        .addr (cnt ),
        .clk  (Clk ),
        .q    (lut )
      );
    end
  end
  ////////////////////////////ov7725////////////////////////////////
  else if(CAMERA_TYPE == "ov7725")
  begin
    assign device_id = 8'h42;
    assign addr_mode = 1'b0;
    assign addr = lut[15:8];
    assign wrdata = lut[7:0];

    if(IMAGE_TYPE == RGB) //------RGB------
    begin
      assign lut_size  = 68;

      ov7725_init_table_rgb #(
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN   ),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN )
      )ov7725_init_table_rgb_inst
      (
        .addr (cnt ),
        .clk  (Clk ),
        .q    (lut )
      );
    end
  end
  endgenerate


  i2c_control #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
  )i2c_control(
    .Clk         (Clk             ),
    .Rst_n       (Rst_n           ),
    .wrreg_req   (wrreg_req       ),
    .rdreg_req   (1'd0            ),
    .addr        (addr            ),
    .addr_mode   (addr_mode       ),
    .wrdata      (wrdata          ),
    .rddata      (rddata          ),
    .device_id   (device_id       ),
    .RW_Done     (RW_Done         ),
    .ack         (ack             ),
    .dly_cnt_max (i2c_dly_cnt_max ),
    .i2c_sclk    (i2c_sclk        ),
    .i2c_sdat    (i2c_sdat        )
  );

  wire Go;   //initial enable
  reg [20:0] delay_cnt;

  //涓婄數骞跺�嶄綅瀹屾垚20ms鍚庡啀閰嶇疆鎽勫儚澶达紝鎵�浠ヤ粠涓婄數鍒板紑濮嬮厤缃�搴旇�ユ槸1 + 20 = 21ms
  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    delay_cnt <= 21'd0;
  else if (delay_cnt >= DELAY_21ms_CNT)
    delay_cnt <= DELAY_21ms_CNT;
  else
    delay_cnt <= delay_cnt + 1'd1;

  //褰撳欢鏃舵椂闂村埌锛屽紑濮嬩娇鑳藉垵濮嬪寲妯″潡瀵筄V5640鐨勫瘎瀛樺櫒杩涜�屽啓鍏�  
  assign Go = (delay_cnt == DELAY_21ms_CNT - 1'b1) ? 1'b1 : 1'b0;

  //5640瑕佹眰涓婄數鍚庡叾澶嶄綅鐘舵�侀渶瑕佷繚鎸�1ms锛屾墍浠ヤ笂鐢靛悗闇�瑕�1ms涔嬪悗鍐嶄娇鑳介噴鏀炬憚鍍忓ご鐨勫�嶄綅淇″彿
  //杩欓噷涓轰簡浼樺寲閫昏緫锛岀畝鍖栨瘮杈冨櫒閫昏緫锛岀洿鎺ヤ娇寤惰繜姣旇緝鍊间负DELAY_1ms_CNT
  assign camera_rst_n = (delay_cnt > DELAY_1ms_CNT);

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    cnt <= 0;
  else if(Go) 
    cnt <= 0;
  else if(cnt < lut_size)begin
    if(RW_Done && (!ack))
      cnt <= cnt + 1'b1;
    else
      cnt <= cnt;
  end
  else
    cnt <= 0;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    Init_Done <= 0;
  else if(Go) 
    Init_Done <= 0;
  else if(cnt == lut_size)
    Init_Done <= 1;

  reg [1:0]state;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)begin
    state <= 0;
    wrreg_req <= 1'b0;
    i2c_dly_cnt_max <= 32'd0;
  end
  else if(cnt < lut_size)begin
    case(state)
      0:
        if(Go)
          state <= 1;
        else
          state <= 0;
      1:
        begin
          wrreg_req <= 1'b1;
          state <= 2;
          if(cnt == 1)
            i2c_dly_cnt_max <= DELAY_5ms_CNT; //寤舵椂5ms
          else
            i2c_dly_cnt_max <= 32'd0;
        end
      2:
        begin
          wrreg_req <= 1'b0;
          if(RW_Done)
            state <= 1;
          else
            state <= 2;
        end
      default:state <= 0;
    endcase
  end
  else
    state <= 0;

endmodule
