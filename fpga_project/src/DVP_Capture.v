/////////////////////////////////////////////////////////////////////////////////
// Company       : 姝︽眽鑺�璺�鎭掔�戞妧鏈夐檺鍏�鍙�
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : DVP_Capture
// Description   : CMOS鎽勫儚澶碊VP鎺ュ彛鎺у埗閫昏緫
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module DVP_Capture(
  input            Rst_n,
  input            PCLK,
  input            Vsync,
  input            Href,
  input     [7:0]  Data,

  output reg       ImageState,
  output           DataValid,
  output    [15:0] DataPixel,
  output           DataHs,
  output           DataVs,
  output    [10:0] Xaddr,
  output    [10:0] Yaddr
);

  reg       r_Vsync;
  reg       r_Href;
  reg [7:0] r_Data;

  reg [15:0]r_DataPixel;
  reg       r_DataValid;
  reg       r_DataHs;
  reg       r_DataVs;
  reg [11:0]Hcount;
  reg [10:0]Vcount;
  reg [3:0] FrameCnt;

  reg       dump_frame;

  //绛夊埌鍒濆�嬪寲鎽勫儚瀹屾垚涓斿ご鍦哄悓姝ヤ俊鍙峰嚭鐜帮紝閲婃斁娓呴浂淇″彿锛屽紑濮嬪啓鍏ユ暟鎹�
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    ImageState <= 1'b1;
  else if(r_Vsync)
    ImageState <= 1'b0;

  //瀵笵VP鎺ュ彛鐨勬暟鎹�浣跨敤瀵勫瓨鍣ㄦ墦涓�鎷嶏紝浠ョ敤淇″彿杈规部妫�娴嬪姛鑳�
  always@(posedge PCLK)
  begin
    r_Vsync <= Vsync;
    r_Href  <= Href;
    r_Data  <= Data;
  end

  //鍦℉REF涓洪珮鐢靛钩鏃讹紝璁℃暟杈撳嚭鏁版嵁涓�鏁�
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    Hcount <= 0;
  else if(r_Href)
    Hcount <= Hcount + 1'd1;
  else
    Hcount <= 0;

  /*鏍规嵁璁℃暟鍣ㄧ殑璁℃暟鍊煎�囨暟鍜屽伓鏁扮殑鍖哄埆锛屽湪璁℃暟鍣ㄤ负鍋舵暟鏃讹紝
  灏咲VP鎺ュ彛鏁版嵁绔�鍙ｄ笂鐨勬暟鎹�瀛樺埌杈撳嚭鍍忕礌鏁版嵁鐨勯珮瀛楄妭锛屽湪璁�
  鏁板櫒涓哄�囨暟鏃讹紝灏咲VP鎺ュ彛鏁版嵁绔�鍙ｄ笂鐨勬暟鎹�瀛樺埌杈撳嚭鍍忕礌鏁版嵁
  鐨勪綆瀛楄妭*/
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    r_DataPixel <= 0;
  else if(!Hcount[0])
    r_DataPixel[15:8] <= r_Data;
  else 
    r_DataPixel[7:0] <= r_Data;

  /*鍦ㄨ�岃�℃暟鍣ㄨ�℃暟鍊间负濂囨暟锛屼笖HREF楂樼數骞虫湡闂达紝浜х敓杈撳嚭
   鏁版嵁鏈夋晥淇″彿*/
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    r_DataValid <= 0;
  else if(Hcount[0] && r_Href)
    r_DataValid <= 1;
  else
    r_DataValid <= 0;

  always@(posedge PCLK)
  begin
    r_DataHs <= r_Href;
    r_DataVs <= ~r_Vsync;
  end

  /*浣跨敤Vcount璁℃暟鍣ㄥ�笻REF淇″彿鐨勯珮鐢靛钩杩涜�岃�℃暟锛岀粺璁�
  涓�甯у浘鍍忎腑鐨勬瘡涓�琛屽浘鍍忕殑琛屽彿*/
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    Vcount <= 0;
  else if(r_Vsync)
    Vcount <= 0;
  else if({r_Href,Href} == 2'b01)
    Vcount <= Vcount + 1'd1;
  else
    Vcount <= Vcount;

  /*杈撳嚭X鍦板潃*/  
  assign Yaddr = Vcount;

  /*鐢变簬涓�琛孨涓�鍍忕礌鐨勫浘鍍忚緭鍑�2N涓�鏁版嵁锛屾墍浠�Hcount璁℃暟
  鍊间负N鐨�2鍊嶏紝灏嗚�ヨ�℃暟鍊奸櫎浠�2鍚庡嵆鍙�浣滀负Xaddr杈撳嚭*/
  assign Xaddr = Hcount[11:1];

  /*甯ц�℃暟鍣�锛屽�规瘡娆＄郴缁熷紑濮嬭繍琛屽悗鐨勫墠10甯у浘鍍忚繘琛岃�℃暟*/
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)  
    FrameCnt <= 0;
  else if({r_Vsync,Vsync}== 2'b01)begin
    if(FrameCnt >= 10)
      FrameCnt <= 4'd10;
    else
      FrameCnt <= FrameCnt + 1'd1;
  end
  else
    FrameCnt <= FrameCnt;

  /*鑸嶅純姣忔�＄郴缁熷紑濮嬭繍琛屽悗鐨勫墠10甯у浘鍍忕殑鏁版嵁锛屼互纭�淇濊緭鍑哄浘鍍忕ǔ瀹�*/
  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    dump_frame <= 0;
  else if(FrameCnt >= 10)
    dump_frame <= 1'd1;
  else
    dump_frame <= 0;

  assign DataPixel = r_DataPixel;
  assign DataValid = r_DataValid & dump_frame;
  assign DataHs = r_DataHs & dump_frame;
  assign DataVs = r_DataVs & dump_frame;

endmodule
