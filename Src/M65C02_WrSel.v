`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
// 
// Create Date:     19:43:14 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_WrSel.v 
// Project Name:    C:\XProjects\ISE10.1i\MAM6502 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//
// Dependencies: 
//
// Revision: 
// 
//  0.01    13I15   MAM     File Created
//
//  1.00    13J07   MAM     Updated headers and modified the original implemen-
//                          tation to use two ROMs instead of one 64x5 ROM. One
//                          ROM provides a direct decode of the Reg_WE field,
//                          the other provides a decode of the WSel fiedl when
//                          (Reg_WE == 3'b100). This change provides a signifi-
//                          cant improvement to the M65C02_ALUv2 module when
//                          synthesize as a standalone module.
//
//  1.10    14F28   MAM     Adjusted comments to reflect changes made during
//                          integration: (1) corrected the uP driven write
//                          enable logic to select PSW when writing A, X, or Y.
//  
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_WrSel(
    input   Rst,
    input   Clk,
    
    input   [2:0] Reg_WE,
    input   [2:0] WSel,
    
    output  reg SelA,
    output  reg SelX,
    output  reg SelY,
    output  reg SelP,
    
    output  reg SelS
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     uP_A, uP_X, uP_Y, uP_P, uP_S;
reg     iD_A, iD_X, iD_Y, iD_P, iD_S;
wire    WE;

//  Decode Register Write Enables

always @(*)
begin
    case(Reg_WE)
        3'b000 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b000_0_0;
        3'b001 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b100_1_0;
        3'b010 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b010_1_0;
        3'b011 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b001_1_0;
        3'b100 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b000_0_0;
        3'b101 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b000_0_1;
        3'b110 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b000_1_0;
        3'b111 : {uP_A, uP_X, uP_Y, uP_P, uP_S} <= #1 5'b000_0_0;
    endcase
end

assign WE = (Reg_WE == 3'b100);

always @(*)
begin
    case({WE, WSel})
        4'b0000 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0001 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0010 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0011 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0100 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0101 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0110 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b0111 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b1000 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b1001 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b100_1_0;
        4'b1010 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b010_1_0;
        4'b1011 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b001_1_0;
        4'b1100 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_0;
        4'b1101 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_0_1;
        4'b1110 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_1_0;
        4'b1111 : {iD_A, iD_X, iD_Y, iD_P, iD_S} <= #1 5'b000_1_0;
    endcase
end

always @(negedge Clk or posedge Rst)
begin
    if(Rst)
        {SelA, SelX, SelY, SelP, SelS} <= #1 0;
    else begin
        SelA <= #1 (uP_A | iD_A);
        SelX <= #1 (uP_X | iD_X);
        SelY <= #1 (uP_Y | iD_Y);
        //
        SelP <= #1 (uP_P | iD_P);
        //
        SelS <= #1 (uP_S | iD_S);
    end
end

endmodule
