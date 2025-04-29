`timescale 1ns / 1ps

module Timer_Periph (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY
    // export signals
);
    logic [1:0] tcr;
    logic [31:0] tcnt;
    logic [31:0] psc;
    logic [31:0] arr;

    APB_SlaveIntf_TIMER U_APB_Intf_TIMER (.*);

    timer U_TIMER(
        .clk(PCLK),
        .rst(PRESET),
        .en(tcr[0]),
        .clear(tcr[1]),
        .*
    );
endmodule

module APB_SlaveIntf_TIMER (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    // internal signals
    output logic [ 1:0] tcr,
    input  logic [31:0] tcnt,
    output logic [31:0] psc,
    output logic [31:0] arr
);
    logic [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3;

    assign tcr = slv_reg0[1:0];
    assign slv_reg1 = tcnt;
    assign psc = slv_reg2;
    assign arr = slv_reg3;

    always_ff @(posedge PCLK or posedge PRESET) begin
        if (PRESET) begin
            slv_reg0 <= 0;
            // slv_reg1 <= 0;
            slv_reg2 <= 0;
            slv_reg3 <= 0;
        end else begin
            if (PSEL && PENABLE) begin
                PREADY <= 1'b1;
                if (PWRITE) begin
                    case (PADDR[3:2])
                        2'd0: slv_reg0 <= PWDATA;
                        2'd1: ;  // write 불가
                        2'd2: slv_reg2 <= PWDATA;
                        2'd3: slv_reg3 <= PWDATA;
                    endcase
                end else begin
                    PRDATA <= 32'bx;
                    case (PADDR[3:2])
                        2'd0: PRDATA <= slv_reg0;
                        2'd1: PRDATA <= slv_reg1;
                        2'd2: PRDATA <= slv_reg2;
                        2'd3: PRDATA <= slv_reg3;
                    endcase
                end
            end else begin
                PREADY <= 1'b0;
            end
        end
    end
endmodule

module timer (
    input  logic        clk,
    input  logic        rst,
    input  logic        en,
    input  logic        clear,
    input  logic [31:0] arr,
    input  logic [31:0] psc,
    output logic [31:0] tcnt
    );
    logic o_clk;

    counter U_Counter(.*);

    clk_divider_timer U_Clk_Div_Timer(.*);
endmodule

module counter (
    input  logic        clk,
    input  logic        rst,
    input  logic        clear,
    input  logic        o_clk,
    input  logic [31:0] arr,
    output logic [31:0] tcnt
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            tcnt <= 0;
        end else begin
            if (o_clk) begin
                if (tcnt == arr) begin
                    tcnt <= 0;
                end else begin
                    tcnt <= tcnt + 1;
                end
            end
            else if (clear) begin
                tcnt <= 0;
            end
        end
    end
endmodule

module clk_divider_timer (
    input  logic        clk,
    input  logic        rst,
    input  logic        en,
    input  logic        clear,
    input  logic [31:0] psc,
    output logic        o_clk
);
    logic [31:0] counter;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            o_clk   <= 1'b0;
            counter <= 0;
        end else begin
            if (clear) begin
                o_clk   <= 1'b0;
                counter <= 0;
            end
            if (en) begin
                if (counter == psc) begin
                    o_clk   <= 1'b1;
                    counter <= 0;
                end else begin
                    counter <= counter + 1;
                    o_clk   <= 1'b0;
                end
            end
        end
    end
endmodule
