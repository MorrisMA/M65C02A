`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
//
// Create Date:     16:25:47 06/22/2014
// Design Name:     M65C02_CoreV2
// Module Name:     C:/XProjects/ISE10.1i/M65C02A/tb_M65C02_CoreV2.v
// Project Name:    M65C02A
// Target Device:   Xilinx SRAM-based FPGAs  
// Tool versions:   Xilinx ISE 10.1i SP3
//  
// Description: 
//
// Verilog Test Fixture created by ISE for module: M65C02_CoreV2
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_M65C02_CoreV2;

parameter pM65C02_ROM    = "Src/M65C02_Tst3.txt";
parameter pM65C02_RAM    = "Src/M65C02_RAM.txt";
parameter pM65C02_uPgm   = "Src/M65C02_uPgm_V4.coe";
parameter pM65C02_IDec   = "Src/M65C02_IDecode_ROM.coe";
//
parameter pRST_Vector    = 16'hFFFC;
parameter pIRQ_Vector    = 16'hFFFE;
parameter pBrk_Vector    = 16'hFFFE;
//
parameter pInt_Hndlr     = 9'h021;
//
parameter pIRQ_On        = 16'hFFF8;
parameter pIRQ_Off       = 16'hFFF9;
//
parameter pIO_WR         = 2'b01;
//
parameter pRAM_AddrWidth = 11;

// UUT Signals

reg     Rst;
reg     Clk;

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
wire    [ 7:0] DO;
wire    [ 7:0] DI;

// Simulation Variables

reg Sim_Int = 0;

// Instantiate the Unit Under Test (UUT)

M65C02_CoreV2   #(
                    .pInt_Hndlr(pInt_Hndlr)
                )uut (
                    .Rst(Rst), 
                    .Clk(Clk), 

                    .IRQ_Msk(IRQ_Msk), 
                    .xIRQ(xIRQ), 

                    .Int(Int), 
                    .Vector(Vector), 

                    .IntSvc(IntSvc), 
                    .ISR(ISR), 

                    .SO(SO), 
                    .Clr_SO(Clr_SO), 

                    .Done(Done), 
                    .SC(SC), 
                    .Mode(Mode), 
                    .RMW(RMW), 

                    .Wait(Wait), 
                    .Rdy(Rdy), 

                    .IO_Op(IO_Op), 
                    .AO(AO), 
                    .DI(DI), 
                    .DO(DO)
                );

//  Instantiate RAM Module

wire    [7:0] ROM_DO;
reg     ROM_WE;

M65C02_RAM  #(
                .pAddrSize(pRAM_AddrWidth),
                .pDataSize(8),
                .pFileName(pM65C02_ROM)
            ) ROM (
                .Clk(~Clk),

                .Ext(1'b1),     // 1 cycle memory
                .ZP(1'b1),

                .WE(ROM_WE),
                .AI(AO[(pRAM_AddrWidth - 1):0]),
                .DI(DO),
                .DO(ROM_DO)
            );

//  Instantiate RAM Module

wire    [7:0] RAM_DO;
reg     RAM_WE;

M65C02_RAM  #(
                .pAddrSize(pRAM_AddrWidth),
                .pDataSize(8),
                .pFileName(pM65C02_RAM)
            ) RAM (
                .Clk(~Clk),
                
                .Ext(1'b1),     // 1 cycle Block RAM
                .ZP(1'b1),
                
                .WE(RAM_WE),
                .AI(AO[(pRAM_AddrWidth - 1):0]),
                .DI(DO),
                .DO(RAM_DO)
            );

initial begin
    // Initialize Inputs
    Rst    = 1;
    Clk    = 1;
    xIRQ   = 0;
    Int    = 1;
    Vector = pRST_Vector;
    SO     = 0;
    Wait   = 0;
    
    force IRQ_Msk = 1;

    // Wait 100 ns for global reset to finish

    #101 Rst = 0;
    
    // Add stimulus here
    
    @(negedge ISR) release IRQ_Msk;

end
  
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Clocks
//

always #5 Clk = ~Clk;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Generate Write Enables for "ROM" and "RAM" modules and multiplex DO onto DI
//

always @(*) ROM_WE = (IO_Op == 1) & ( &AO[15:pRAM_AddrWidth]);
always @(*) RAM_WE = (IO_Op == 1) & (~|AO[15:pRAM_AddrWidth]);

assign DI = ((&AO[15:pRAM_AddrWidth]) ? ROM_DO : RAM_DO);

///////////////////////////////////////////////////////////////////////////////
//
//  Reset/NMI/Brk/IRQ Vector Generator
//

always @(*)
begin
    Vector = ((Mode == 3'b101) ? pBrk_Vector
                               : ((Int) ? pIRQ_Vector
                                        : pRST_Vector));
end

// Simulate Interrupts

always @(*)
begin
    if((AO == pIRQ_On) && (IO_Op == pIO_WR))
        Sim_Int = 1;
    else if((AO == pIRQ_Off) && (IO_Op == pIO_WR))
        Sim_Int = 0;
end

always @(*)
begin
    Int  = ((IRQ_Msk) ? 0 : (Sim_Int | (Mode == 3'b111)));
    xIRQ = (Mode == 3'b111);
end

////////////////////////////////////////////////////////////////////////////////

`include "Src/M65C02_Mnemonics.txt"

endmodule

