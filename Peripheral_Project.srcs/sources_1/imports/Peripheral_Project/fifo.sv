`timescale 1ns / 1ps

module UART_Periph (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 4:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    // outport signals
    output logic        tx
);

    logic tick;
    logic wr_en, rd_en;
    logic [ 1:0] FSR;
    logic [ 7:0] FWD;
    logic [ 7:0] FRD;
    logic [15:0] BRR;
    logic [ 3:0] UCR;

    APB_SlaveIntf_FIFO U_APB_Intf_FIFO (
        .*,
        .wr_en(wr_en),
        .rd_en(rd_en)
    );

    baud_tick_gen U_BAUD_GEN (
        .clk(PCLK),
        .rst(PRESET),
        .DIV_Mantissa(BRR[15:4]),
        .DIV_Fraction(BRR[3:0]),
        .tick(tick)
    );

    uart_tx U_UART_TX (
        .clk(PCLK),
        .rst(PRESET),
        .tick(tick),
        .start_trigger(UCR[0] & UCR[1] & UCR[3]),
        .data(FRD),
        .o_tx_done(FSR[2]),
        .o_tx(tx)
    );


    fifo U_FIFO_TX (
        .clk  (PCLK),
        .rst  (PRESET),
        .wdata(FWD),
        .wr_en(wr_en),
        .full (FSR[0]),
        .rdata(FRD),
        .rd_en(rd_en),
        .empty(FSR[1])
    );

endmodule

module APB_SlaveIntf_FIFO (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 4:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    // internal signals
    output logic        wr_en,
    output logic        rd_en,
    input  logic [ 2:0] FSR,
    output logic [ 7:0] FWD,
    input  logic [ 7:0] FRD,
    output logic [15:0] BRR,
    output logic [ 3:0] UCR
);
    logic [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3, slv_reg4;

    assign slv_reg0[2:0] = FSR;  // FIFO State Register -> 0: empty, 1 : full, 2: tx complete
    assign FWD = slv_reg1[7:0];  // write data 8bit
    assign slv_reg2[7:0] = FRD;  // read data 8bit
    assign BRR = slv_reg3[15:0]; // Baud Rate Register -> [15:4] DIV_Mantissa(정수부), [3:0] DIV_Fraction 
    assign UCR = slv_reg4[3:0];  // UART Control Register -> 0: UART enable, 1: UART_tx enable, 2: UART_rx enalbe, 3: UART_tx Trigger

    // wr_en, rd_en control
    always_comb begin
        wr_en = 1'b0;
        rd_en = 1'b0;
        if (PSEL && PENABLE) begin
            case (PADDR[3:2])
                3'd1: if (PWRITE) wr_en = 1'b1; // TXDATA (slv_reg1)
                3'd2: if (!PWRITE) rd_en = 1'b1; // RXDATA (slv_reg2)
                default: ;
            endcase
        end
    end


    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            // slv_reg0 <= 0;
            slv_reg1 <= 0;
            // slv_reg2 <= 0;
            slv_reg3 <= 0;
            slv_reg4 <= 0;
        end else begin
            if (PSEL && PENABLE) begin
                PREADY <= 1'b1;
                if (PWRITE) begin
                    case (PADDR[3:2])
                        3'd0: ;
                        3'd1: slv_reg1 <= PWDATA;
                        3'd2: ;
                        3'd3: slv_reg3 <= PWDATA;
                        3'd4: slv_reg4 <= PWDATA;
                    endcase
                end else begin
                    PRDATA <= 32'bx;
                    case (PADDR[3:2])
                        3'd0: PRDATA <= slv_reg0;
                        3'd1: PRDATA <= slv_reg1;
                        3'd2: PRDATA <= slv_reg2;
                        3'd3: PRDATA <= slv_reg3;
                        3'd4: PRDATA <= slv_reg4;
                    endcase
                end
            end else begin
                PREADY <= 1'b0;
            end
        end
    end
endmodule

module uart_tx (
    input clk,
    input rst,
    input tick,
    input start_trigger,
    input [7:0] data,
    output o_tx_done,
    output o_tx
);
    // fsm state
    parameter IDLE = 0, SEND = 1, START = 2, DATA = 3, STOP = 4;

    reg [3:0] state, next;
    reg tx_reg, tx_next;
    reg tx_done_reg, tx_done_next;
    reg [2:0] bit_count_reg, bit_count_next;
    reg [3:0] tick_count_reg, tick_count_next;
    reg [7:0] temp_data_reg, temp_data_next; //

    assign o_tx_done = tx_done_reg;
    assign o_tx = tx_reg;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            state <= 0;
            tx_reg <= 1'b1;    //  Uart tx line을 초기에 항상 1로 만들기 위함.
            tx_done_reg <= 0;
            bit_count_reg <= 0;
            tick_count_reg <= 0;
            temp_data_reg <= 0; //
        end else begin
            state          <= next;
            tx_reg         <= tx_next;
            tx_done_reg    <= tx_done_next;
            bit_count_reg  <= bit_count_next;
            tick_count_reg <= tick_count_next;
            temp_data_next <= temp_data_reg; //
        end
    end

    // next
    always @(*) begin
        next            = state;
        tx_next         = tx_reg;
        tx_done_next    = tx_done_reg;
        bit_count_next  = bit_count_reg;
        tick_count_next = tick_count_reg;
        // tx_done_next = data;
        case (state)
            IDLE: begin
                tx_next = 1'b1;
                tx_done_next = 1'b0;
                tick_count_next = 4'h0;
                if (start_trigger) begin
                    next = SEND;
                    temp_data_next = data; //
                end
            end
            SEND: begin
                if (tick == 1'b1) begin
                    next = START;
                end
            end
            START: begin
                tx_done_next = 1'b1;
                tx_next      = 1'b0;  // 출력을 0으로 유지.
                if (tick == 1'b1) begin
                    if (tick_count_reg == 15) begin
                        next = DATA;
                        bit_count_next = 1'b0;
                        tick_count_next = 1'b0; // next state로 갈때 tick_count 초기화
                    end else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
            DATA: begin
                // tx_next = data[bit_count_reg];  // UART LSB first
                tx_next = temp_data_reg[bit_count_reg]; //
                if (tick) begin
                    if (tick_count_reg == 15) begin
                        tick_count_next = 0; // 다음 상태로 가기전에 초기화
                        if (bit_count_reg == 7) begin
                            next = STOP;
                        end else begin
                            next = DATA;
                            bit_count_next = bit_count_reg + 1;  // bit count 증가
                        end
                    end else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
            STOP: begin
                tx_next = 1'b1;
                if (tick == 1'b1) begin
                    if (tick_count_reg == 15) begin
                        next = IDLE;
                    end else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
        endcase
    end
endmodule

module baud_tick_gen (
    input logic clk,
    input logic rst,
    input logic [11:0] DIV_Mantissa,
    input logic [3:0] DIV_Fraction,
    output logic tick
);

    logic [15:0] baud_divider;
    logic [15:0] baud_cnt;

    assign baud_divider = (DIV_Mantissa << 4) | DIV_Fraction;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            baud_cnt <= 0;
            tick <= 0;
        end else begin
            if (baud_cnt == baud_divider) begin
                baud_cnt <= 0;
                tick <= 1;
            end else begin
                baud_cnt <= baud_cnt + 1;
                tick <= 0;
            end
        end
    end

endmodule

module fifo (
    input  bit         clk,
    input  logic       rst,
    // write side
    input  logic [7:0] wdata,
    input  logic       wr_en,
    output logic       full,
    // read side
    output logic [7:0] rdata,
    input  logic       rd_en,
    output logic       empty
);

    logic [1:0] wr_ptr, rd_ptr;

    ram_fifo U_ram_fifo (
        .clk  (clk),
        .wAddr(wr_ptr),
        .wdata(wdata),
        .wr_en(~full & wr_en),
        .rAddr(rd_ptr),
        .rdata(rdata)
    );

    fifo_control_unit U_fifo_control_unit (.*);

endmodule

module ram_fifo (
    input  bit         clk,
    input  logic [1:0] wAddr,
    input  logic [7:0] wdata,
    input  logic       wr_en,
    input  logic [1:0] rAddr,
    output logic [7:0] rdata
);

    logic [7:0] mem[0:2**2-1];

    always_ff @(posedge clk) begin
        if (wr_en) mem[wAddr] <= wdata;
    end

    assign rdata = mem[rAddr];
endmodule

module fifo_control_unit (
    input  bit         clk,
    input  logic       rst,
    // write side
    output logic [1:0] wr_ptr,
    input  logic       wr_en,
    output logic       full,
    // read side
    output logic [1:0] rd_ptr,
    input  logic       rd_en,
    output logic       empty
);

    localparam READ = 2'b01, WRITE = 2'b10, READ_WRITE = 2'b11;
    logic [1:0] fifo_state;
    logic [1:0] wr_ptr_reg, wr_ptr_next, rd_ptr_reg, rd_ptr_next;
    logic empty_reg, empty_next, full_reg, full_next;

    assign fifo_state = {wr_en, rd_en};
    assign full = full_reg;
    assign empty = empty_reg;
    assign wr_ptr = wr_ptr_reg;
    assign rd_ptr = rd_ptr_reg;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr_reg <= 0;
            rd_ptr_reg <= 0;
            full_reg   <= 1'b0;
            empty_reg  <= 1'b1;
        end else begin
            wr_ptr_reg <= wr_ptr_next;
            rd_ptr_reg <= rd_ptr_next;
            full_reg   <= full_next;
            empty_reg  <= empty_next;
        end
    end

    always_comb begin : fifo_comb
        empty_next  = empty_reg;
        full_next   = full_reg;
        wr_ptr_next = wr_ptr_reg;
        rd_ptr_next = rd_ptr_reg;
        case (fifo_state)
            READ: begin
                if (!empty_reg) begin
                    full_next   = 1'b0;
                    rd_ptr_next = rd_ptr_reg + 1;
                    if (rd_ptr_next == wr_ptr_reg) begin
                        empty_next = 1'b1;
                    end
                end
            end
            WRITE: begin
                if (!full_reg) begin
                    empty_next  = 1'b0;
                    wr_ptr_next = wr_ptr_reg + 1;
                    if (wr_ptr_next == rd_ptr_reg) begin
                        full_next = 1'b1;
                    end
                end
            end
            READ_WRITE: begin
                if (empty_reg) begin
                    wr_ptr_next = wr_ptr_reg + 1;
                    empty_next  = 1'b0;
                end else if (full_reg) begin
                    rd_ptr_next = rd_ptr_reg + 1;
                    full_next   = 1'b0;
                end else begin
                    wr_ptr_next = wr_ptr_reg + 1;
                    rd_ptr_next = rd_ptr_reg + 1;
                end
            end
        endcase
    end
endmodule
