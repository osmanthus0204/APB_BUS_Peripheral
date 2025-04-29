`timescale 1ns / 1ps

module FND_Periph (
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
    // outport signals
    output logic [ 3:0] FndComm,
    output logic [ 7:0] FndFont
);

    logic FCR;
    logic [3:0] FPR;
    logic [13:0] FDR;
    logic [3:0] w_FDR;
    logic o_clk;
    logic [1:0] sel;
    logic [3:0] digit_1, digit_10, digit_100, digit_1000;
    logic [3:0] fndDot;
    logic dotDp;
    logic [7:0] w_FndFont;

    assign FndFont = {dotDp, w_FndFont[6:0]};

    APB_SlaveIntf_FND U_APB_Intf_FND (.*);

    FND U_FND_IP (
        .FDR(w_FDR),
        .FndFont(w_FndFont)
    );

    clk_divider U_clk_divider (
        .clk  (PCLK),
        .rst  (PRESET),
        .o_clk(o_clk)
    );

    counter_4 U_counter_4 (
        .clk  (o_clk),
        .rst  (PRESET),
        .count(sel)
    );

    decoder_2x4 U_decoder_2x4 (
        .FCR(FCR),
        .x  (sel),
        .y  (FndComm)
    );

    digit_splitter U_digit_splitter (
        .a(FDR),
        .digit_1(digit_1),
        .digit_10(digit_10),
        .digit_100(digit_100),
        .digit_1000(digit_1000)
    );

    mux_4x1 U_mux_4x1 (
        .sel(sel),
        .digit_1(digit_1),
        .digit_10(digit_10),
        .digit_100(digit_100),
        .digit_1000(digit_1000),
        .bcd(w_FDR)
    );

    mux_4x1_1bit U_mux_4x1_dot (
        .FCR(FCR),
        .sel(sel),
        .x  (fndDot),
        .y  (dotDp)
    );

    comp_dot U_comp_dot (
        .FCR(FCR),
        .FPR(FPR),
        .dot_data(fndDot)
    );

endmodule

module APB_SlaveIntf_FND (
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
    output logic        FCR,
    output logic [13:0] FDR,
    output logic [ 3:0] FPR
);
    logic [31:0] slv_reg0, slv_reg1, slv_reg2;  //slv_reg3;

    assign FCR = slv_reg0[0];
    assign FDR = slv_reg1[13:0];
    assign FPR = slv_reg2[3:0];

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            slv_reg0 <= 0;
            slv_reg1 <= 0;
            slv_reg2 <= 0;
            // slv_reg3 <= 0;
        end else begin
            if (PSEL && PENABLE) begin
                PREADY <= 1'b1;
                if (PWRITE) begin
                    case (PADDR[3:2])
                        2'd0: slv_reg0 <= PWDATA;
                        2'd1: slv_reg1 <= PWDATA;
                        2'd2: slv_reg2 <= PWDATA;
                        // 2'd3: slv_reg3 <= PWDATA;
                    endcase
                end else begin
                    PRDATA <= 32'bx;
                    case (PADDR[3:2])
                        2'd0: PRDATA <= slv_reg0;
                        2'd1: PRDATA <= slv_reg1;
                        2'd2: PRDATA <= slv_reg2;
                        // 2'd3: PRDATA <= slv_reg3;
                    endcase
                end
            end else begin
                PREADY <= 1'b0;
            end
        end
    end

endmodule

module FND (
    input  logic [3:0] FDR,
    output logic [7:0] FndFont
);
    always_comb begin
        case (FDR)
            4'h0: FndFont = 8'hC0;
            4'h1: FndFont = 8'hF9;
            4'h2: FndFont = 8'hA4;
            4'h3: FndFont = 8'hB0;
            4'h4: FndFont = 8'h99;
            4'h5: FndFont = 8'h92;
            4'h6: FndFont = 8'h82;
            4'h7: FndFont = 8'hF8;
            4'h8: FndFont = 8'h80;
            4'h9: FndFont = 8'h90;
            4'hA: FndFont = 8'h88;
            4'hB: FndFont = 8'h83;
            4'hC: FndFont = 8'hC6;
            4'hD: FndFont = 8'hA1;
            4'hE: FndFont = 8'h86;
            4'hF: FndFont = 8'h8E;
            default: FndFont = 8'hFF;
        endcase
    end

endmodule

module clk_divider (
    input  logic clk,
    input  logic rst,
    output logic o_clk
);
    parameter COUNT = 500_000;
    logic [$clog2(500_000)-1:0] counter;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            o_clk   <= 0;
            counter <= 0;
        end else if (counter == COUNT - 1) begin
            o_clk   <= 1;
            counter <= 0;
        end else begin
            o_clk   <= 0;
            counter <= counter + 1;
        end
    end
endmodule

module counter_4 (
    input logic clk,
    input logic rst,
    output logic [1:0] count
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0;
        end else begin
            count <= count + 1;
        end
    end
endmodule

module decoder_2x4 (
    input  logic       FCR,
    input  logic [1:0] x,
    output logic [3:0] y
);
    always_comb begin
        if (FCR) begin
            case (x)
                2'b00:   y = 4'b1110;
                2'b01:   y = 4'b1101;
                2'b10:   y = 4'b1011;
                2'b11:   y = 4'b0111;
                default: y = 4'b1111;
            endcase
        end else begin
            y=4'b1111;
        end
    end
endmodule

module digit_splitter (
    input  logic [13:0] a,
    output logic [ 3:0] digit_1,
    output logic [ 3:0] digit_10,
    output logic [ 3:0] digit_100,
    output logic [ 3:0] digit_1000
);

    assign digit_1 = a % 10;
    assign digit_10 = a / 10 % 10;
    assign digit_100 = a / 100 % 10;
    assign digit_1000 = a / 1000;
endmodule

module mux_4x1 (
    input [1:0] sel,
    input [3:0] digit_1,
    digit_10,
    digit_100,
    digit_1000,
    output reg [3:0] bcd
);
    always @(sel, digit_1, digit_10, digit_100, digit_1000) begin
        case (sel)
            2'b00:   bcd = digit_1;
            2'b01:   bcd = digit_10;
            2'b10:   bcd = digit_100;
            2'b11:   bcd = digit_1000;
            default: bcd = 4'bx;
        endcase
    end
endmodule

module comp_dot (
    input logic FCR,
    input [3:0] FPR,
    output [3:0] dot_data
);

    assign dot_data = (FCR) ? ~FPR : 4'b1111;

endmodule

module mux_4x1_1bit (
    input logic FCR,
    input [1:0] sel,
    input [3:0] x,
    output reg y
);

    always @(*) begin
        y = 1'b1;
        if (FCR) begin
            case (sel)
                2'b00: y = x[0];
                2'b01: y = x[1];
                2'b10: y = x[2];
                2'b11: y = x[3];
            endcase
        end else begin
            y = x[0];
        end
    end

endmodule
