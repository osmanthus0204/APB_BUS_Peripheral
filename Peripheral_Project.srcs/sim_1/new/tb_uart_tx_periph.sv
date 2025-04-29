`timescale 1ns / 1ps

module tb_uart_tx_periph();

    logic        PCLK;
    logic        PRESET;
    logic [ 4:0] PADDR;
    logic [31:0] PWDATA;
    logic        PWRITE;
    logic        PENABLE;
    logic        PSEL;
    logic [31:0] PRDATA;
    logic        PREADY;
    logic        tx;

    UART_Periph U_UART_Periph(.*);

    always begin
        #5 PCLK= ~PCLK;
    end

    initial begin
        PCLK = 0;
        PRESET = 1;
        #10 PRESET = 0;
    
    end

endmodule
