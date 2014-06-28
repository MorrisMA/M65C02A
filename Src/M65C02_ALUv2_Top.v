`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
// 
// Create Date:     21:30:54 03/17/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_ALUv2_Top.v 
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
// Dependencies:    M65C02_ALUv2.v      // M65C02A Arithmetic/Logic Unit (ALU)
//                      M65C02_LU.v     // M65C02A ALU Logic Unit
//                      M65C02_SU.v     // M65C02A ALU Shift Unit
//                      M65C02_Add.v    // M65C02A ALU Adder Unit
//                      M65C02_LST.v    // M65C02A ALU Load/Store/Transfer Unit
//                      M65C02_WrSel.v  // M65C02A ALU Reg Write Enable ROM
//
// Revision:
//
//  0.01    13C17   MAM     Puts the M65C02_ALUv2 module into a top level module
//                          with all external inputs and inputs connected to
//                          IOB FFs. This construction measures the maximum com-
//                          binatorial path delays in the M65C02_ALUv2 module.
//                          As a consequence, this module provides a measure of
//                          the performance that can be expected when the
//                          synthesis/MAP/PAR tools integrate the module with
//                          the other elements of the M65C02 CPU.
// 
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_ALUv2_Top(
    input   iRst,           // System Reset - synchronous reset 
    input   Clk,            // System Clock
    
    input   iRdy,
    input   iEn,

    input   [2:0] iReg_WE,
    input   iISR,
    
    input   iSO,
    output  reg oClr_SO,
    
    output  reg oSelS,
    input   [7:0] iS,
    
    input   [4:0] iFU_Sel,
    input   [1:0] iOp,
    input   [1:0] iQSel,
    input   [1:0] iRSel,
    input   [1:0] iCSel,
    input   [2:0] iWSel,
    input   [2:0] iOSel,
    input   [3:0] iCCSel,
    
    input   [7:0] iK,
    input   [7:0] iTmp,
    input   [7:0] iM,

    output  reg [7:0] oDO,
    output  reg oVal,
    output  reg oCC,
    
    output  reg [7:0] oX,
    output  reg [7:0] oY,
    
    output  reg [7:0] oPSW
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     Rst;                // System Reset - synchronous reset 

reg     Rdy;
reg     En;

reg     [2:0] Reg_WE;
reg     ISR;

reg     SO;
wire    Clr_SO;

wire    SelS;
reg     [7:0] S;

reg     [4:0] FU_Sel;
reg     [1:0] Op;
reg     [1:0] QSel;
reg     [1:0] RSel;
reg     [1:0] CSel;
reg     [2:0] WSel;
reg     [2:0] OSel;
reg     [3:0] CCSel;

reg     [7:0] K;
reg     [7:0] Tmp;
reg     [7:0] M;

wire    [7:0] ALU_DO;
wire    Val;
wire    CC_Out;

wire    [7:0] X;
wire    [7:0] Y;

wire    [7:0] P;            // Processor Status Word

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(posedge Clk) Rst     <= #1 iRst;

always @(posedge Clk) Rdy     <= #1 iRdy;
always @(posedge Clk) En      <= #1 iEn;

always @(posedge Clk) SO      <= #1 iSO;

always @(posedge Clk) Reg_WE  <= #1 iReg_WE;
always @(posedge Clk) ISR     <= #1 iISR;

always @(posedge Clk) S       <= #1 iS;

always @(posedge Clk) FU_Sel  <= #1 iFU_Sel;
always @(posedge Clk) Op      <= #1 iOp;
always @(posedge Clk) QSel    <= #1 iQSel;
always @(posedge Clk) RSel    <= #1 iRSel;
always @(posedge Clk) CSel    <= #1 iCSel;
always @(posedge Clk) WSel    <= #1 iWSel;
always @(posedge Clk) OSel    <= #1 iOSel;
always @(posedge Clk) CCSel   <= #1 iCCSel;

always @(posedge Clk) K       <= #1 iK;
always @(posedge Clk) Tmp     <= #1 iTmp;
always @(posedge Clk) M       <= #1 iM;

// Instantiate the module

M65C02_ALUv2    ALU (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .Rdy(Rdy), 
                    .En(En), 

                    .Reg_WE(Reg_WE), 
                    .ISR(ISR), 

                    .SO(SO),
                    .Clr_SO(Clr_SO),

                    .SelS(SelS), 
                    .S(S), 

                    .FU_Sel(FU_Sel), 
                    .Op(Op), 
                    .QSel(QSel), 
                    .RSel(RSel), 
                    .CSel(CSel), 
                    .WSel(WSel), 
                    .OSel(OSel), 
                    .CCSel(CCSel),

                    .K(K),
                    .Tmp(Tmp),
                    .M(M), 

                    .DO(ALU_DO), 
                    .Val(Val), 
                    .CC_Out(CC_Out), 

                    .X(X), 
                    .Y(Y),
                    
                    .P(P)
                );

always @(posedge Clk) oClr_SO <= #1 Clr_SO;

always @(posedge Clk) oSelS   <= #1 SelS;

always @(negedge Clk) oDO     <= #1 ALU_DO;
always @(negedge Clk) oVal    <= #1 Val;
always @(posedge Clk) oCC     <= #1 CC_Out;

always @(posedge Clk) oX      <= #1 X;
always @(posedge Clk) oY      <= #1 Y;

always @(posedge Clk) oPSW    <= #1 P;

endmodule
