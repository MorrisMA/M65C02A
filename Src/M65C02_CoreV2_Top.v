`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
// 
// Create Date:     21:30:54 03/17/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_CoreV2_Top.v 
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
// Dependencies: 
//
// Dependencies:    M65C02_MPCv5.v      // M65C02A Microprogram Controller (MPC)
//                  M65C02_AddrGen.v    // M65C02A Address Generator
//                      M65C02_StkPtr.v // M65C02A Stack Pointer 
//                  M65C02_ALUv2.v      // M65C02A Arithmetic/Logic Unit (ALU)
//                      M65C02_LST.v    // M65C02A ALU Load/Store/Transfer Unit
//                      M65C02_LU.v     // M65C02A ALU Logic Unit
//                      M65C02_SU.v     // M65C02A ALU Shift Unit
//                      M65C02_Add.v    // M65C02A ALU Adder Unit
//                      M65C02_WrSel.v  // M65C02A ALU Reg Write Enable ROM
//                      M65C02_PSWv2.v  // M65C02A ALU Processor Status Word
//
// Revision:
//
//  0.01    13C17   MAM     Puts the M65C02_CoreV2 module into a top level
//                          module with all external inputs and inputs connected
//                          to IOB FFs. This construction measures the maximum
//                          combinatorial path delays in the M65C02_CoreV2
//                          module. Thus, this module provides a measure of the
//                          performance that can be expected when the synthe-
//                          sis/MAP/PAR tools integrate the module with the
//                          other elements of the M65C02A soft-core.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_CoreV2_Top(
    input   iRst,           // System Reset - synchronous reset 
    input   Clk,            // System Clock
    
    output  reg oIRQ_Msk, 
    input   ixIRQ, 
    input   iInt, 
    input   [15:0] iVector,
    
    input   iSO, 
    output  reg oClr_SO, 

    output  reg oDone, 
    output  reg oSC, 
    output  reg [2:0] oMode,
    output  reg oRMW, 

    output  reg oIntSvc, 
    output  reg oISR, 

    input   iWait, 
    output  reg oRdy, 

    output  reg [ 1:0] oIO_Op, 
    output  reg [15:0] oAO,
    input       [ 7:0] iDI,
    output  reg [ 7:0] oDO
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     Rst;                // System Reset - synchronous reset 

wire    IRQ_Msk; 
reg     xIRQ; 
reg     Int; 
reg     [15:0] Vector;

reg     SO; 
wire    Clr_SO; 

wire    Done; 
wire    SC; 
wire    [2:0] Mode; 
wire    RMW; 

wire    IntSvc; 
wire    ISR; 

reg     Wait; 
wire    Rdy; 

wire    [ 1:0] IO_Op; 
wire    [15:0] AO;
reg     [ 7:0] DI; 
wire    [ 7:0] DO;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(posedge Clk) Rst    <= #1 iRst;
always @(posedge Clk) xIRQ   <= #1 ixIRQ; 
always @(posedge Clk) Int    <= #1 iInt; 
always @(posedge Clk) Vector <= #1 iVector;

always @(posedge Clk) SO     <= #1 iSO; 
always @(posedge Clk) Wait   <= #1 iWait; 
always @(posedge Clk) DI     <= #1 iDI; 

// Instantiate the module

M65C02_CoreV2   Core (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .IRQ_Msk(IRQ_Msk), 
                    .xIRQ(xIRQ), 
                    .Int(Int), 
                    .Vector(Vector),
                    
                    .SO(SO), 
                    .Clr_SO(Clr_SO), 

                    .Done(Done), 
                    .SC(SC), 
                    .Mode(Mode),
                    .RMW(RMW), 

                    .IntSvc(IntSvc), 
                    .ISR(ISR), 

                    .Wait(Wait), 
                    .Rdy(Rdy), 

                    .IO_Op(IO_Op), 
                    .AO(AO), 
                    .DI(DI),
                    .DO(DO)
                );

always @(negedge Clk) oIRQ_Msk <= #1 IRQ_Msk; 
always @(negedge Clk) oClr_SO  <= #1 Clr_SO; 

always @(negedge Clk) oDone    <= #1 Done; 
always @(negedge Clk) oSC      <= #1 SC; 
always @(negedge Clk) oMode    <= #1 Mode; 
always @(negedge Clk) oRMW     <= #1 RMW; 

always @(negedge Clk) oIntSvc  <= #1 IntSvc; 
always @(negedge Clk) oISR     <= #1 ISR; 

always @(negedge Clk) oRdy     <= #1 Rdy; 

always @(negedge Clk) oIO_Op   <= #1 IO_Op; 
always @(negedge Clk) oAO      <= #1 AO;
always @(negedge Clk) oDO      <= #1 DO;

endmodule
