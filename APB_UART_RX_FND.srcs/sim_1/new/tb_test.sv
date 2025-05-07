`timescale 1ns / 1ps
module tb_test();

logic       clk;
logic       reset;
// logic [7:0] GPOA;
// logic [7:0] GPIB;
// logic [7:0] GPIOC;
// logic [7:0] GPIOD;
// logic [3:0] fndCom;
// logic [7:0] fndFont;
logic       RX;


MCU dut(
    .clk(clk),
    .reset(reset),
    .GPOA(),
    .GPIB(),
    .GPIOC(),
    .GPIOD(),
    .fndCom(),
    .fndFont(),
    .RX(RX)
);

always #5 clk = ~clk;
    // UART bit period for 9600 baud at 100 MHz clock
    localparam BAUD_TICKS = 16;
    localparam BIT_PERIOD = 104167; // ns (1/9600)

    task uart_send_byte(input [7:0] data);
        integer i;
        begin
            // Start bit
            RX = 0;
            #(BIT_PERIOD);

            // 8 data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                RX = data[i];
                #(BIT_PERIOD);
            end

            // Stop bit
            RX = 1;
            #(BIT_PERIOD);
        end
    endtask

initial begin
    clk = 0;
    reset = 1;
    #10;
    reset = 0;

            // Send byte 0xA5 = 1010_0101
        uart_send_byte(8'hA5);

        // Wait enough time for receive to complete
        #2000000; // 2 ms

        // Send byte 0x5A = 0101_1010
        uart_send_byte(8'h5A); // 데이터2
        // Wait enough time for receive to complete
        #2000000; // 2 ms
        
        // Send byte 0x3C = 0011_1100
        uart_send_byte(8'h3C); // 데이터3
        // Wait enough time for receive to complete
        #2000000; // 2 ms

        // Send byte 0x3C = 1111_1111
        uart_send_byte(8'hFF); // 추가 데이터 (255 값 확인용)
        // Wait enough time for receive to complete
        #2000000; // 2 ms



        #1000;
        $finish;
end
endmodule
