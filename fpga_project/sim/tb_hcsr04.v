`timescale 1ns / 1ps
// hcsr04_unit 仿真：正常测量（30cm）+ 超时（无回波）两条路径
module tb_hcsr04;
    reg         Clk   = 1'b0;
    reg         Rst_n = 1'b0;
    reg         start = 1'b0;
    reg         Echo  = 1'b0;
    wire        us_tick;
    wire        Trig;
    wire        done;
    wire [14:0] dist_us;
    wire        valid;

    always #10 Clk = ~Clk;   // 50MHz

    us_tick u_tick (
        .Clk     (Clk),
        .Rst_n   (Rst_n),
        .us_tick (us_tick)
    );

    hcsr04_unit uut (
        .Clk     (Clk),
        .Rst_n   (Rst_n),
        .us_tick (us_tick),
        .start   (start),
        .Echo    (Echo),
        .Trig    (Trig),
        .done    (done),
        .dist_us (dist_us),
        .valid   (valid)
    );

    task measure(input [15:0] echo_us);
    begin
        @(negedge Clk); start = 1'b1;
        @(negedge Clk); start = 1'b0;
        wait (Trig === 1'b1);
        wait (Trig === 1'b0);
        if (echo_us != 0) begin
            repeat (echo_us) @(posedge us_tick);   // 模拟声波飞行延迟
            Echo = 1'b1;
            repeat (echo_us) @(posedge us_tick);   // Echo 高电平宽度
            Echo = 1'b0;
        end
        wait (done === 1'b1);
        $display("t=%0t  dist_us=%0d  valid=%b  (expect dist=%0d)", $time, dist_us, valid, echo_us);
    end
    endtask

    initial begin
        repeat (5)  @(posedge Clk);
        Rst_n = 1'b1;
        repeat (20) @(posedge us_tick);

        measure(16'd1740);   // ≈30cm
        measure(16'd580);    // ≈10cm
        measure(16'd0);      // 不回波 -> 应超时，valid=0

        $display("TEST DONE");
        $finish;
    end

    initial begin
        #100_000_000;
        $display("TIMEOUT: simulation hung");
        $finish;
    end
endmodule
