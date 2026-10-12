`timescale 1ns / 1ps
// =====================================================================
// tb_motor_quad —— motor_quad 仿真
// 检查项：
//   1) 速度指令 +500 -> 轮0 正转、占空比 50%；-250 -> 轮1 反转、25%
//   2) 轮0 编码器正转 1kHz、轮1 反转 1kHz（GATE_MS=1）
//      -> 每闸门 speed = ±4，位置累计 ≈ ±48 (12ms)
// 跑法：iverilog -o tb_motor_quad src/motor_*.v src/encoder_reader.v sim/tb_motor_quad.v
// =====================================================================
module tb_motor_quad;

    reg                Clk    = 1'b0;
    reg                Rst_n  = 1'b0;
    reg  signed [63:0] speed_cmd = 64'd0;
    wire        [3:0]  pwm, in1, in2;
    wire        [1:0]  stby;
    reg         [3:0]  enc_a = 4'b0000;
    reg         [3:0]  enc_b = 4'b0000;
    wire signed [127:0] enc_pos;
    wire signed [63:0]  enc_speed;
    wire               speed_valid;

    always #10 Clk = ~Clk;   // 50MHz

    motor_quad #(
        .GATE_MS(1)         // 1ms 测速闸门，加快仿真
    ) dut (
        .Clk        (Clk),
        .Rst_n      (Rst_n),
        .speed_cmd  (speed_cmd),
        .pwm        (pwm),
        .in1        (in1),
        .in2        (in2),
        .stby       (stby),
        .enc_a      (enc_a),
        .enc_b      (enc_b),
        .enc_pos    (enc_pos),
        .enc_speed  (enc_speed),
        .speed_valid(speed_valid)
    );

    // ---------------- 编码器模型 ----------------
    // 轮0 正转：A 1kHz，B 超前 A 90°（序列 00->01->11->10）
    integer ph0 = 0;
    initial forever begin
        #250_000;
        ph0 = (ph0 + 1) % 4;
        case (ph0)
            0: begin enc_a[0] = 1'b0; enc_b[0] = 1'b0; end
            1: begin enc_a[0] = 1'b0; enc_b[0] = 1'b1; end
            2: begin enc_a[0] = 1'b1; enc_b[0] = 1'b1; end
            default: begin enc_a[0] = 1'b1; enc_b[0] = 1'b0; end
        endcase
    end

    // 轮1 反转：A 1kHz，B 滞后 A 90°（序列 00->10->11->01）
    integer ph1 = 0;
    initial forever begin
        #250_000;
        ph1 = (ph1 + 1) % 4;
        case (ph1)
            0: begin enc_a[1] = 1'b0; enc_b[1] = 1'b0; end
            1: begin enc_a[1] = 1'b1; enc_b[1] = 1'b0; end
            2: begin enc_a[1] = 1'b1; enc_b[1] = 1'b1; end
            default: begin enc_a[1] = 1'b0; enc_b[1] = 1'b1; end
        endcase
    end

    // ---------------- 占空比测量 ----------------
    reg        measuring = 1'b0;
    reg [31:0] mc = 0;
    integer    c0 = 0, c1 = 0;

    always @(posedge Clk) begin
        if (measuring) begin
            if (pwm[0]) c0 <= c0 + 1;
            if (pwm[1]) c1 <= c1 + 1;
            if (mc >= 4999)
                measuring <= 1'b0;   // 恰好 5000 个 clk = 2 个 PWM 周期
            else
                mc <= mc + 1;
        end
    end

    // ---------------- 检查任务 ----------------
    integer errors = 0;
    task check;
        input        cond;
        input [255:0] msg;
        begin
            if (cond)
                $display("PASS: %0s", msg);
            else begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    reg signed [15:0] sv0, sv1;

    initial begin
        Rst_n = 1'b0;
        repeat (10) @(posedge Clk);
        Rst_n = 1'b1;

        // ---- 1) 方向与占空比 ----
        speed_cmd[15:0]  = 16'sd500;    // 轮0 正转 50%
        speed_cmd[31:16] = -16'sd250;   // 轮1 反转 25%
        repeat (200) @(posedge Clk);

        mc = 0; c0 = 0; c1 = 0; measuring = 1'b1;
        wait (measuring === 1'b0);

        check(c0 >= 2480 && c0 <= 2520, "wheel0 pwm duty ~50%");
        check(c1 >= 1230 && c1 <= 1270, "wheel1 pwm duty ~25%");
        check(in1[0] === 1'b1 && in2[0] === 1'b0, "wheel0 forward in1=1 in2=0");
        check(in1[1] === 1'b0 && in2[1] === 1'b1, "wheel1 reverse in1=0 in2=1");
        check(stby === 2'b11, "stby both high");

        // ---- 2) 编码器测速（等 12ms ≈ 12 个闸门） ----
        #12_000_000;
        sv0 = enc_speed[15:0];
        sv1 = enc_speed[31:16];

        check(sv0 >= 3 && sv0 <= 5,  "wheel0 speed ~+4 per 1ms gate");
        check(sv1 >= -5 && sv1 <= -3, "wheel1 speed ~-4 per 1ms gate");
        check(enc_pos[31:0]  >= 40 && enc_pos[31:0]  <= 56, "wheel0 pos ~+48");
        check(enc_pos[63:32] <= -40 && enc_pos[63:32] >= -56, "wheel1 pos ~-48");

        if (errors == 0)
            $display("TEST DONE: ALL PASS");
        else
            $display("TEST DONE: %0d FAILURES", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("TIMEOUT: simulation hung");
        $finish;
    end

endmodule
