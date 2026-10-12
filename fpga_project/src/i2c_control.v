/////////////////////////////////////////////////////////////////////////////////
// Company       : 姝︽眽鑺�璺�鎭掔�戞妧鏈夐檺鍏�鍙�
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : i2c_control
// Description   : I2C鎺у埗鍣ㄩ《灞傛枃浠�
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module i2c_control #(
  //绯荤粺鏃堕挓閲囩敤50MHz
  parameter SYS_CLOCK = 50_000_000,
  //SCL鎬荤嚎鏃堕挓閲囩敤400kHz
  parameter SCL_CLOCK = 400_000
)
(
  input            Clk,
  input            Rst_n,

  input            wrreg_req,
  input            rdreg_req,
  input     [15:0] addr,
  input            addr_mode,
  input     [7:0]  wrdata,
  output reg[7:0]  rddata,
  input     [7:0]  device_id,
  output reg       RW_Done,

  output reg       ack,

  input     [31:0] dly_cnt_max,
  output           i2c_sclk,
  inout            i2c_sdat
);

  reg [5:0]Cmd;
  reg [7:0]Tx_DATA;
  wire [7:0]Rx_DATA;
  wire Trans_Done;
  wire ack_o;
  reg Go;
  wire [15:0] reg_addr;

  assign reg_addr = addr_mode?addr:{addr[7:0],addr[15:8]};

  localparam 
    WR   = 6'b000001,   //鍐欒�锋眰
    STA  = 6'b000010,   //璧峰�嬩綅璇锋眰
    RD   = 6'b000100,   //璇昏�锋眰
    STO  = 6'b001000,   //鍋滄��浣嶈�锋眰
    ACK  = 6'b010000,   //搴旂瓟浣嶈�锋眰
    NACK = 6'b100000;   //鏃犲簲绛旇�锋眰

  i2c_bit_shift #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
  )i2c_bit_shift(
    .Clk       (Clk        ),
    .Rst_n     (Rst_n      ),
    .Cmd       (Cmd        ),
    .Go        (Go         ),
    .Rx_DATA   (Rx_DATA    ),
    .Tx_DATA   (Tx_DATA    ),
    .Trans_Done(Trans_Done ),
    .ack_o     (ack_o      ),
    .i2c_sclk  (i2c_sclk   ),
    .i2c_sdat  (i2c_sdat   )
  );

  reg [7:0]state;
  reg [7:0]cnt;
  reg [31:0]dly_cnt;

  localparam
    IDLE         = 8'b0000_0001,   //绌洪棽鐘舵��
    WR_REG       = 8'b0000_0010,   //鍐欏瘎瀛樺櫒鐘舵��
    WAIT_WR_DONE = 8'b0000_0100,   //绛夊緟鍐欏瘎瀛樺櫒瀹屾垚鐘舵��
    WR_REG_DONE  = 8'b0000_1000,   //鍐欏瘎瀛樺櫒瀹屾垚鐘舵��
    RD_REG       = 8'b0001_0000,   //璇诲瘎瀛樺櫒鐘舵��
    WAIT_RD_DONE = 8'b0010_0000,   //绛夊緟璇诲瘎瀛樺櫒瀹屾垚鐘舵��
    RD_REG_DONE  = 8'b0100_0000,   //璇诲瘎瀛樺櫒瀹屾垚鐘舵��
    WAIT_DLY     = 8'b1000_0000;   //绛夊甫璇诲啓瀹屾垚鍚庡欢杩熷畬鎴�
  always@(posedge Clk or negedge Rst_n)
  if(!Rst_n)begin
    Cmd <= 6'd0;
    Tx_DATA <= 8'd0;
    Go <= 1'b0;
    rddata <= 0;
    state <= IDLE;
    ack <= 0;
    dly_cnt <= 0;
    cnt <= 0;
    RW_Done <= 1'b0;
  end
  else begin
    case(state)
      IDLE:
        begin
          cnt <= 0;
          dly_cnt <= 0;
          ack <= 0;
          RW_Done <= 1'b0;
          if(wrreg_req)
            state <= WR_REG;
          else if(rdreg_req)
            state <= RD_REG;
          else
            state <= IDLE;
        end

      WR_REG:
        begin
          state <= WAIT_WR_DONE;
          case(cnt)
            0://write_byte(WR | STA, device_id);
            begin Cmd <= WR | STA; Tx_DATA <= device_id; Go <= 1'b1; end

            1://write_byte(WR, reg_addr[15:8]);
            begin Cmd <= WR; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end

            2://write_byte(WR, reg_addr[7:0]);
            begin Cmd <= WR; Tx_DATA <= reg_addr[7:0]; Go <= 1'b1; end

            3://write_byte(WR | STO, wrdata);
            begin Cmd <= WR | STO; Tx_DATA <= wrdata; Go <= 1'b1; end

            default:;
          endcase
        end

      WAIT_WR_DONE:
        begin
          Go <= 1'b0; 
          if(Trans_Done)begin
            ack <= ack | ack_o;
            case(cnt)
              0: begin cnt <= 1; state <= WR_REG;end
              1: 
                begin 
                  state <= WR_REG;
                  if(addr_mode)
                    cnt <= 2; 
                  else
                    cnt <= 3;
                end

              2: begin
                  cnt <= 3;
                  state <= WR_REG;
                end
              3:state <= WR_REG_DONE;
              default:state <= IDLE;
            endcase
          end
        end

      WR_REG_DONE:
        begin
//          RW_Done <= 1'b1;
          state <= WAIT_DLY;
        end

      RD_REG:
        begin
          state <= WAIT_RD_DONE;
          case(cnt)
            0://write_byte(WR | STA, device_id);
            begin Cmd <= WR | STA; Tx_DATA <= device_id; Go <= 1'b1; end

            1: begin
              if(addr_mode)
                //write_byte(WR, reg_addr[15:8]);
                begin Cmd <= WR; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end
              else 
                //write_byte(WR | STO, reg_addr[15:8]);
                begin Cmd <= WR | STO; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end
            end

            2://write_byte(WR | STO, reg_addr[7:0]);
            begin Cmd <= WR | STO; Tx_DATA <= reg_addr[7:0]; Go <= 1'b1; end

            3://write_byte(WR | STA, device_id | 8'd1);
            begin Cmd <= WR | STA; Tx_DATA <= device_id | 8'd1; Go <= 1'b1; end

            4://read_byte(RD | NACK | STO);
            begin Cmd <= RD | NACK | STO; Go <= 1'b1; end

            default:;
          endcase
        end

      WAIT_RD_DONE:
        begin
          Go <= 1'b0; 
          if(Trans_Done)begin
            if(cnt <= 3)
              ack <= ack | ack_o;
            case(cnt)
              0: begin cnt <= 1; state <= RD_REG;end
              1: 
                begin 
                  state <= RD_REG;
                  if(addr_mode)
                    cnt <= 2; 
                  else
                    cnt <= 3;
                end
              2: begin
                  cnt <= 3;
                  state <= RD_REG;
                end
              3:begin
                  cnt <= 4;
                  state <= RD_REG;
                end
              4:state <= RD_REG_DONE;
              default:state <= IDLE;
            endcase
          end
        end

      RD_REG_DONE:
        begin
//          RW_Done <= 1'b1;
          rddata <= Rx_DATA;
          state <= WAIT_DLY;
        end
      default:state <= IDLE;
      WAIT_DLY:
        begin
          if(dly_cnt <= dly_cnt_max) begin
            dly_cnt <= dly_cnt + 1'b1;
            state <= WAIT_DLY;
          end
         else begin
           dly_cnt <= 0;
           RW_Done <= 1'b1;
           state <= IDLE;
         end
       end
    endcase
  end

//鑰冭檻閮ㄥ垎鍥戒骇缂栬瘧鍣ㄦ殏涓嶆敮鎸乼ask锛屽幓鎺夎�ョ�嶅啓娉�
//  task read_byte;
//    input [5:0]Ctrl_Cmd;
//    begin
//      Cmd <= Ctrl_Cmd;
//      Go <= 1'b1; 
//    end
//  endtask
//
//  task write_byte;
//    input [5:0]Ctrl_Cmd;
//    input [7:0]Wr_Byte_Data;
//    begin
//      Cmd <= Ctrl_Cmd;
//      Tx_DATA <= Wr_Byte_Data;
//      Go <= 1'b1; 
//    end
//  endtask

endmodule
