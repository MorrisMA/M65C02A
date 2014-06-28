`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates
// Engineer:        Michael A. Morris 
// 
// Create Date:     22:14:57 10/02/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_PSWv2.v
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  Module implements the PSW of the M65C02_ALUv2 module.  
//
// Dependencies:    None
//
// Revision:
//
//  0.01    13J02   MAM     File Created
//
//  1.00    13J02   MAM     Reverted back to using a case statement for genera-
//                          ing multiplexer for a unitary 6-bit PSW register. 
//                          However, optimized the encoding of the least signi-
//                          ficant 3 bits of the 4-bit CCSel field to maximize
//                          performance and logic reduction. Left implementation
//                          discussed below as comments after the released code.
//
//  1.01    14A19   MAM     Note: two case statement members, pBRK & pPHP, are
//                          included in the WE_P case statement. These CC select
//                          conditions do not assert WE_P, but are included in
//                          case statement below to allow the synthesizer to 
//                          further optimize the control logic/multiplexer for
//                          each bit in the PSW register. Added comments to case
//                          statement members pBRK and pPHP.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_PSWv2(
    input   Clk,
    
    input   SO,
    output  Clr_SO,
    
    input   SelP,
    input   Valid,
    input   Rdy,
    
    input   ISR,
    
    input   [2:0] CCSel,
    
    input   [7:6] M,
    input   OV,
    input   LU_Z,
    input   COut,
    input   [7:0] DO,

    output  [7:0] P,
    
    output  N,
    output  V,
    output  D,
    output  Z,
    output  C
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameters
//

localparam pPSW  = 3'b000;  // Set P from ALU
localparam pBRK  = 3'b001;  // Set P.4 when pushing P during interrupt handling
localparam pZ    = 3'b010;  // Set Z = ~|(A & M)
localparam pNVZ  = 3'b011;  // Set N and V flags from M[7:6], and Z = ~|(A & M)
localparam pPHP  = 3'b100;  // Set P.4 when executing PHP instruction
localparam pNZ   = 3'b101;  // Set N and Z flags from ALU
localparam pNZC  = 3'b110;  // Set N, Z, and C flags from ALU
localparam pNVZC = 3'b111;  // Set N, V, Z, and C from ALU

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     [5:0] PSW;
wire    I;

////////////////////////////////////////////////////////////////////////////////
//
//  P - Processor Status Word: {N, V, 1, B, D, I, Z, C}
//

assign WE_P = (SelP | SO) & Valid & Rdy;

always @(posedge Clk)
begin
    if(ISR & Rdy)
        PSW <= #1 {1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0};
    else if(WE_P) 
        case(CCSel)
            pPSW  : PSW <= #1 {DO[7],(DO[6] | SO), DO[3:0]};
            pBRK  : PSW <= #1 {DO[7],(DO[6] | SO), DO[3:0]};
            pZ    : PSW <= #1 {    N,(V     | SO),   D,   I, LU_Z,   C};
            pNVZ  : PSW <= #1 { M[7],(M[6]  | SO),   D,   I, LU_Z,   C};
            pPHP  : PSW <= #1 {DO[7],(V     | SO),   D,   I,~|DO ,   C};
            pNZ   : PSW <= #1 {DO[7],(V     | SO),   D,   I,~|DO ,   C};
            pNZC  : PSW <= #1 {DO[7],(V     | SO),   D,   I,~|DO ,COut};
            pNVZC : PSW <= #1 {DO[7],(OV    | SO),   D,   I,~|DO ,COut};
        endcase
end

//  Decode PSW bits

assign N = PSW[5];  // Negative, nominally Out[7], but M[7] if BIT/TRB/TSB
assign V = PSW[4];  // oVerflow, nominally OV,     but M[6] if BIT/TRB/TSB
assign D = PSW[3];  // Decimal, set/cleared by SED/CLD, cleared on ISR entry
assign I = PSW[2];  // Interrupt Mask, set/cleared by SEI/CLI, set on ISR entry
assign Z = PSW[1];  // Zero, nominally ~|Out, but ~|(A&M) if BIT/TRB/TSB
assign C = PSW[0];  // Carry, set by ADC/SBC, and ASL/ROL/LSR/ROR instructions

//  Assign PSW bits to P (PSW output port)

assign BRK = (CCSel == pBRK);
assign PHP = (CCSel == pPHP);

assign P = {N, V, 1'b1, (BRK | PHP), D, I, Z, C};

//  Generate for Acknowledge SO Command

assign Clr_SO = SO & Valid & Rdy;

endmodule
