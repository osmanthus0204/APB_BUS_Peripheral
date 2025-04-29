`timescale 1ns / 1ps

module MCU (
    input  logic       clk,
    input  logic       reset,
    output logic [7:0] GPOA,
    input  logic [7:0] GPIB,
    inout  logic [7:0] GPIOC,
    inout  logic [7:0] GPIOD,
    output logic [3:0] FndComm,
    output logic [7:0] FndFont,
    output             tx
);
    // global signals
    logic        PCLK;
    logic        PRESET;
    // APB Interface Signals
    logic [31:0] PADDR;
    logic [31:0] PWDATA;
    logic        PWRITE;
    logic        PENABLE;
    logic        PSEL_RAM;
    logic        PSEL_GPO;
    logic        PSEL_GPI;
    logic        PSEL_GPIOC;
    logic        PSEL_GPIOD;
    logic        PSEL_TIMER;
    logic        PSEL_FND;
    logic        PSEL_UART;
    logic [31:0] PRDATA_RAM;
    logic [31:0] PRDATA_GPO;
    logic [31:0] PRDATA_GPI;
    logic [31:0] PRDATA_GPIOC;
    logic [31:0] PRDATA_GPIOD;
    logic [31:0] PRDATA_TIMER;
    logic [31:0] PRDATA_FND;
    logic [31:0] PRDATA_UART;
    logic        PREADY_RAM;
    logic        PREADY_GPO;
    logic        PREADY_GPI;
    logic        PREADY_GPIOC;
    logic        PREADY_GPIOD;
    logic        PREADY_TIMER;
    logic        PREADY_FND;
    logic        PREADY_UART;

    // CPU - APB_Master Signals
    // Internal Interface Signals
    logic        transfer;  // trigger signal
    logic        ready;
    logic [31:0] addr;
    logic [31:0] wdata;
    logic [31:0] rdata;
    logic        write;  // 1:write, 0:read
    logic        dataWe;
    logic [31:0] dataAddr;
    logic [31:0] dataWData;
    logic [31:0] dataRData;

    // ROM Signals
    logic [31:0] instrCode;
    logic [31:0] instrMemAddr;

    assign PCLK = clk;
    assign PRESET = reset;
    assign addr = dataAddr;
    assign wdata = dataWData;
    assign dataRData = rdata;
    assign write = dataWe;

    rom U_ROM (
        .addr(instrMemAddr),
        .data(instrCode)
    );

    RV32I_Core U_Core (.*);

    APB_Master U_APB_Master (
        .*,
        .PSEL0  (PSEL_RAM),
        .PSEL1  (PSEL_GPO),
        .PSEL2  (PSEL_GPI),
        .PSEL3  (PSEL_GPIOC),
        .PSEL4  (PSEL_GPIOD),
        .PSEL5  (PSEL_FND),
        .PSEL6  (PSEL_TIMER),
        .PSEL7  (PSEL_UART),
        .PRDATA0(PRDATA_RAM),
        .PRDATA1(PRDATA_GPO),
        .PRDATA2(PRDATA_GPI),
        .PRDATA3(PRDATA_GPIOC),
        .PRDATA4(PRDATA_GPIOD),
        .PRDATA5(PRDATA_FND),
        .PRDATA6(PRDATA_TIMER),
        .PRDATA7(PRDATA_UART),
        .PREADY0(PREADY_RAM),
        .PREADY1(PREADY_GPO),
        .PREADY2(PREADY_GPI),
        .PREADY3(PREADY_GPIOC),
        .PREADY4(PREADY_GPIOD),
        .PREADY5(PREADY_FND),
        .PREADY6(PREADY_TIMER),
        .PREADY7(PREADY_UART)
    );

    ram U_RAM (
        .*,
        .PSEL  (PSEL_RAM),
        .PRDATA(PRDATA_RAM),
        .PREADY(PREADY_RAM)
    );

    GPO_Periph U_GPOA (
        .*,
        .PSEL   (PSEL_GPO),
        .PRDATA (PRDATA_GPO),
        .PREADY (PREADY_GPO),
        // export signals
        .outPort(GPOA)
    );

    GPI_Periph U_GPIB (
        .*,
        .PSEL  (PSEL_GPI),
        .PRDATA(PRDATA_GPI),
        .PREADY(PREADY_GPI),
        // inport signals
        .inPort(GPIB)
    );

    GPIO_Periph U_GPIOC (
        .*,
        .PSEL   (PSEL_GPIOC),
        .PRDATA(PRDATA_GPIOC),
        .PREADY(PREADY_GPIOC),
        .inoutPort(GPIOC)
    );

    GPIO_Periph U_GPIOD (
        .*,
        .PSEL     (PSEL_GPIOD),
        .PRDATA   (PRDATA_GPIOD),
        .PREADY   (PREADY_GPIOD),
        .inoutPort(GPIOD)
    );

    FND_Periph U_FND (
        .*,
        .PSEL   (PSEL_FND),
        .PRDATA (PRDATA_FND),
        .PREADY (PREADY_FND),
        .FndComm(FndComm),
        .FndFont(FndFont)
    );

    Timer_Periph U_TIMER (
        .*,
        .PSEL  (PSEL_TIMER),
        .PRDATA(PRDATA_TIMER),
        .PREADY(PREADY_TIMER)
    );

    UART_Periph U_UART (
        .*,
        .PSEL(PSEL_UART),
        .PRDATA(PSEL_UART),
        .PREADY(PREADY_UART),
        .tx(tx)
    );
endmodule
