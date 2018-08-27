////////////////////////////////////////////////////////////////////////////////
//
//  M65C02A soft-core microcomputer project.
// 
//  Copyright 2015 by Michael A. Morris, dba M. A. Morris & Associates
//
//  All rights reserved. The source code contained herein is publicly released
//  under the terms and conditions of the GNU General Public License as conveyed
//  in the license provided below.
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along with
//  this program.  If not, see <http://www.gnu.org/licenses/>, or write to
//
//  Free Software Foundation, Inc.
//  51 Franklin Street, Fifth Floor
//  Boston, MA  02110-1301 USA
//
//  Further, no use of this source code is permitted in any form or means
//  without inclusion of this banner prominently in any derived works.
//
//  Michael A. Morris <morrisma_at_mchsi_dot_com>
//  164 Raleigh Way
//  Huntsville, AL 35811
//  USA
//
////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris 
// 
// Create Date:     01/03/2015 
// Design Name:     Enhanced Microprogrammed 6502-Compatible Soft-Core Processor
// Module Name:     M65C02A_Core.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:     (See additional comments section below)
//
// Dependencies:    M65C02A_MPC.v
//                      M65C02A_uPgm_ROM.coe    (M65C02A_uPgm_ROM.txt)
//                      M65C02A_IDecode_ROM.coe (M65C02A_IDecode_ROM.txt)
//                  M65C02A_AddrGen.v
//                      M65C02A_StkPtrV2.v
//                  M65C02A_ForthVM.v
//                  M65C02A_ALUv2.v
//                      M65C02A_ALU.v
//                          M65C02A_LST.v
//                          M65C02A_LU.v
//                          M65C02A_SU.v
//                          M65C02A_Add.v
//                          M65C02A_WrSel.v
//                          M65C02A_RegStk.v
//                          M65C02A_RegStkV2.v
//                              M65C02A_StkPtr.v
//                          M65C02A_PSW.v
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release.
// 
//  1.11    15F28   MAM     Added (but left commented out) support for the Co-
//                          Processor interface.
//
//  1.12    15F29   MAM     Modified interface to the FORTH VM (and renamed
//                          control field) to allow the control field to be used
//                          to implement the MOV instruction.
//
//  1.20    15G18   MAM     Added a delay line for Rst used during startup to
//                          force operation in Kernel mode before the PSW is 
//                          properly initialized. (Note: PSW is not reset so
//                          that the correct PC and PSW can be written to the
//                          lowest three location in page 1 during reset.) Added
//                          the delayed reset and BRK signals into the Kernel
//                          mode signal. BRK is required in order to place the
//                          processor context on the kernel stack. Corrected the
//                          operation of the internal wait state generator. Also
//                          modified the equation for Done, which affects the
//                          external Sync signal. Sync is not asserted during
//                          the processing of an interrupt. It will be asserted
//                          on the instruction fetch of the first instruction in
//                          the interrupt service routine. Corrected/reversed
//                          changes to the instruction mode field to support
//                          MOV instruction. Reverted to a mode for BRK, and put
//                          MOV as a bit in the SPC instruction opcode field.
//
//  1.30    15H01   MAM     Removed the need for delayed Rst. Modified the
//                          interface so the core provides the Kernel mode sig-
//                          nal and the other decode instruction mode signals to
//                          itself and the external microcomputer. The Rst delay
//                          line was replaced by a circuit that used BRV2 and
//                          BRV1 to set a register/latch for the duration of an
//                          exception/trap or interrupt. BRK is a trap, so this
//                          new circuit works for BRK as well as all interrupts
//                          and traps such as ABRT, INV, etc. To fully support
//                          the Kernel/User processor mode, a signal was created
//                          to indicate that an RTI instruction was being exe-
//                          cuted. The RTI signal, coupled with the latched BRV2
//                          signal provides the means by which the kernel mode
//                          stack pointer is used throughout exception/traps and
//                          RTI. This functionality fully implements the mode
//                          switching logic of the M65C02A core.
//
//  1.31    15J27   MAM     Added output port to provide indication that the STP
//                          instruction is being executed and the core has stop-
//                          ped executing. The STP output, along with the WAI
//                          output, can be used to trigger external watchdog
//                          timers that will prevent the processor from halting
//                          for extended periods of time. Adding the STP port
//                          required adding a decoder on the IR specifically for
//                          the STP instruction opcode.
//
//  1.32    15K12   MAM     Modified the OSX inputs to the address generator and
//                          ALU modules to allow the instruction decoder to
//                          select the default stack for FORTH VM ENT/PHI/PLI
//                          instructions. OSX would normally be used to select
//                          X as the stack pointer. The modification allows the
//                          FORTH VM instructions to use X as the RSP without
//                          specifying OSX. Non-FORTH instructions would still
//                          need to specify OSX to use X as the stack pointer.
//
//  1.40    15L10   MAM     Added ADJ instruction mode, and passed it down into
//                          the ALU modules to support direct addition of an
//                          imm16 value to the system and auxiliary stack
//                          pointers.
//
//  1.41    15L28   MAM     Added NA_Op[5] (IP) address select into Forth_VM
//                          select in order to enable the LDA/ADD/STA ip,I++
//                          instructions without modifying the mode and opcode
//                          fields of the instruction decode ROM.
//
//  1.50    16D10   MAM     Modified the instruction mode decode to support a
//                          second special instruction group. Within this group
//                          are located the ADJ instruction, and the selects
//                          that allow IND prefix instruction to enable the
//                          implementation of a true arithmetic left shift 
//                          instruction using ASL (for 8/16-bit) and ROL for 
//                          >16-bit, and an arithmetic right shift instruction 
//                          using LSR.In addition, this new special instruction
//                          group allows enabling INC/DEC A to set the carry
//                          flag in order to support simple extended precision
//                          counters.
//
//  1.51    16L25   MAM     Modified the bits in OP2 used to select the dst mode
//                          from bits OP2[5:4] to OP2[3:2]. OP2[7] remains the
//                          bit to determine block (0) or single cycle (1) mode.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_Core #(
    parameter pSIZ          = 1'b0, // Default value of SIZ prefix flag
    
    parameter pAddrWidth    = 8'd9, // MPC Address Width
    parameter pMPC_Stk      = 8'd0, // MPC Stack Select: 0 - 1 Lvl; 1 - 4 Lvl
    parameter pRst_Addrs    = 8'd0, // MPC Reset Address
        
    parameter pInt_Hndlr    = 9'd9, // _Int microroutine address, Reset default
    parameter pM65C02A_uPgm = "Pgms/M65C02A_uPgm_ROM.coe",
    parameter pM65C02A_IDec = "Pgms/M65C02A_IDecode_ROM.coe",

    parameter pDef_Page     = 8'h01,// Default System Stack Pointer Page
    parameter pSk_Rst       = 8'h02,// Kernel Stack Pointer Default Value
    parameter pSu_Rst       = 8'hFF // User Stack Pointer Default Value
)(
    input   Rst,            // System Reset Input
    input   Clk,            // System Clock Input
    
    //  Processor Core Interrupt Interface
    
    output  IRQ_Msk,        // Interrupt mask from P to Interrupt Handler
    output  LE_Int,         // Interrupt Latch Enable - Hold Int until serviced

    input   xIRQ,           // External Maskable Interrupt Request Input
    input   Int,            // Interrupt input from Interrupt Handler
    input   [15:0] Vector,  // ISR Vector from Interrupt Handler

    output  reg VP,         // Interrupt Vector Pull Indicator
    
    //  Processor Core Set oVerflow Flag Interface
    
    input   SO,             // Set oVerflow Flag in PSW
    output  Clr_SO,         // Clr SO Command - Acknowledge
    
    //  Processor Core Status Interface
    
    output  Kernel,         // Kernel/User Mode
    
    output  ADJ,            // ADJ Instruction
    output  BRK,            // BRK Instruction
    output  COP,            // COP Instruction
    output  FTH,            // ForthVM Instructions
    output  PFX,            // Prefix Instructions
    output  SPC,            // Special Instructions

    output  Done,           // Instruction Complete/Fetch Strobe
    output  SC,             // Single Cycle Instruction
    output  RMW,            // Read-Modify-Write Operation
    
    //  Processor Core Memory Cycle Length Control Interface
    
    input   Wait,           // Wait Input
    output  reg Rdy,        // Internal Ready
    
    //  Processor Core Memory Interface    
    
    output  [ 1:0] IO_Op,   // Instruction Fetch Strobe
    output  [15:0] AO,      // External Address
    input   [ 7:0] DI,      // External Data In (Registered Data Path)
    output  [ 7:0] DO,      // External Data Out
    
    //  Processor Core Prefix Instruction Flag Interface
    
    output  reg IND,        // Indirect Addressing Mode Override
    output  reg SIZ,        // Size Override 
    output  reg OAX,        // Override Op(A) with Op(X)
    output  reg OAY,        // Override Op(A) with Op(Y)
    output  reg OSX,        // Override Op(S) with Op(X) 

    //  Processor Core Co-Processor Interface
    //    Co-Processor Interface Defines Dir as: 0 - Write; 1 - Read
    //    Co-Processor Register Addresses:
    //        0 - Flags/Status - On reads, sets V using SO
    //        1 - AO/AI
    //        2 - BO/BI
    //        3 - CO/CI     
    
    output  COP_En,         // Co-Processor Enable
    input   COP_SIZ,        // Co-Processor Interface Width

    output  [ 2:0] COP_Sel, // Co-Processor Select: 0 - Mul/Div; 7 - Sys
    output  [ 1:0] COP_RA,  // Co-Processor Register Address
    output  COP_Dir,        // Co-Processor Data Direction Flag
    output  [15:0] COP_DI,  // Co-Processor Data Input  (ALU Output Bus)
    input   [15:0] COP_DO,  // Co-Processor Data Output (ALU Input/M Bus)

    input   COP_Busy,       // Co-Processor Busy Flag
    input   COP_Done,       // Co-Processor Done Flag

    output  COP_Start,      // Co-Processor Start Strobe (Clear Busy and Done)
    output  COP_Clear,      // Co-Processor Clear Done Flag Strobe
    output  COP_Pulse,      // Co-Processor Pulse Strobe
    
    //  Processor Core Internal State Interface
    
    output  [15:0] X,       // Internal Processor Registers - Index Register X
    output  [15:0] Y,       // Internal Processor Registers - Index Register Y
    output  [15:0] A,       // Internal Processor Registers - Accumulator
    output  [15:0] IP,      // Internal Processor FORTH VM IP Register
    output  [15:0] W,       // Internal Processor FORTH VM W  Register
    output  [ 7:0] P,       // Internal Processor Status Word
    output  [15:0] S,       // Internal Processor System Stack Pointer
    output  [15:0] M,       // Internal Processor Memory Operand Registers
    
    output  [ 7:0] IR       // Internal Processor Instruction Register
);

////////////////////////////////////////////////////////////////////////////////
//
// Local Parameter Declarations
//

localparam  pROM_AddrWidth = 8'd9;
localparam  pROM_Width     = 8'd36;
localparam  pROM_Depth     = (2**pROM_AddrWidth);

localparam  pDEC_AddrWidth = 8'd8;
localparam  pDEC_Width     = 8'd36;
localparam  pDEC_Depth     = (2**pDEC_AddrWidth);

localparam  pBA_Fill = (pROM_AddrWidth - pDEC_AddrWidth);

localparam  pBRV1    = 2'b01;           // MPC Via[1:0] code for BRV1 inst.
localparam  pBRV2    = 2'b10;           // MPC Via[1:0] code for BRV2 inst.
localparam  pBRV3    = 2'b11;           // MPC Via[1:0] code for BRV3 inst.
localparam  pFTCH    = 4'b0010;         // MPC I[3:0] core for FTCH instruction
localparam  pBMW     = 4'b0011;         // MPC I[3:0] code for BMW instruction
localparam  pBMW0    = 4'b1100;         // MPC I[3:0] code for BMW0 instruction
localparam  pBMW1    = 4'b1101;         // MPC I[3:0] code for BMW1 instruction
localparam  pBMW2    = 4'b1110;         // MPC I[3:0] code for BMW2 instruction
localparam  pBMW3    = 4'b1111;         // MPC I[3:0] code for BMW3 instruction

localparam  pIO_WR   = 2'b01;           // Memory Write
localparam  pIO_RD   = 2'b10;           // Memory Read
localparam  pIO_IF   = 2'b11;           // Instruction Fetch

localparam  pDO_ALU  = 2'b00;           // DO <= ALU_Out
localparam  pDO_MDH  = 2'b01;           // DO <= (PHW?OP2:(PHR?Hi(MAR):Hi(PC)))
localparam  pDO_MDL  = 2'b10;           // DO <= (PHW?OP1:(PHR?Lo(MAR):Lo(PC)))
localparam  pDO_PSW  = 2'b11;           // DO <= P (also available on ALU_Out)
//
localparam  pDI_Mem  = 2'b00;           // ALU_M <= DI
localparam  pDI_OP2  = 2'b01;           // OP2   <= DI
localparam  pDI_OP1  = 2'b10;           // OP1   <= DI
localparam  pDI_IR   = 2'b11;           // IR    <= DI

localparam  pNOP     = 8'hEA;           // NOP opcode
localparam  pSTP     = 8'hDB;           // STP opcode

localparam  pMode    = 5;               // Bit number of M(ode) bit in P
localparam  pIntMsk  = 2;               // Bit number of Interrupt mask bit in P

//  Fixed Microcode Mode Field Decode

localparam  pMd_WAI  = 7;               // WAI Instruction
localparam  pMd_PFX  = 6;               // Prefix Operations
localparam  pMd_SPC1 = 5;               // Special 1 Operations
localparam  pMd_FTH  = 4;               // FORTH VM Operations
localparam  pMd_BRK  = 3;               // BRK Instruction
localparam  pMd_COP  = 2;               // COP Instruction
localparam  pMd_SPC2 = 1;               // Special 2 Operations
localparam  pMd_VAL  = 0;               // Valid Instructions

////////////////////////////////////////////////////////////////////////////////
//
// Local Signal Declarations
//

wire    [2:0] Mode;                     // Instruction Mode

wire    SPC1;                           // Mode Decode for Mode 5: SPeCial 1
wire    SPC2;                           // Mode Decode for Mode 1: SPeCial 2

wire    PSH;                            // Mode Decode for PHW/PHA/PHX/PHY
wire    PHR;                            // Mode Decode for PHR
wire    MOV;                            // Mode Decode for MOV
wire    RTI;                            // Mode Decode for RTI
wire    DUP;                            // Mode decode for DUP
wire    SWP;                            // Mode decode for SWP
wire    ROT;                            // Mode decode for ROT
wire    WRD;                            // Mode Decode for 16-bit Ops

wire    VEN;                            // Mode Decode for Arithmetic Left Shift
wire    CEN;                            // Mode Decode for Enable C for INC/DEC 

wire    PHI;                            // Mode Decode for PHI
wire    PLI;                            // Mode Decode for PLI
wire    RSPX;                           // Mode Decode for Default RSP

wire    iOSX;                           // Mode Decode for OSX
wire    iOAY;                           // Mode Decode for OAY
wire    iOAX;                           // Mode Decode for OAX
wire    iSIZ;                           // Mode Decode for SIZ | ISZ
wire    iIND;                           // Mode Decode for IND | ISZ

wire    BRV1;                           // MPC BRV1 Instruction Decode
wire    BRV2;                           // MPC BRV2 Instruction Decode
wire    BRV3;                           // MPC BRV3 Instruction Decode

reg     [(pROM_Width - 1):0] uP_ROM [(pROM_Depth - 1):0]; // Microprogram ROM

wire    [3:0] I;                        // MPC Instruction Input
wire    [3:0] T;                        // MPC Test Inputs
reg     [2:0] MW;                       // MPC Multi-way Branch Select
reg     [(pROM_AddrWidth - 1):0] BA;    // MPC Branch Address Input
wire    [(pROM_AddrWidth - 1):0] MA;    // MPC uP ROM Address Output
wire    [1:0] Via;                      // MPC Via Mux Control Output

reg     [(pROM_Width - 1)    :0] uPL;   // MPC uP ROM Pipeline Register
wire    [(pROM_AddrWidth - 2):0] uP_BA; // uP Branch Address Field

wire    [3:0] uPCntl;                   // Microprogram Control Field
wire    [8:0] NA_Op;                    // Memory Address Register Control Fld
wire    [2:0] DI_Op;                    // Memory Data Input Control Field
wire    [3:0] DO_Op;                    // Memory Data Output Control Field
wire    Reg_WE;                         // Register Write Enable Control Field
wire    [2:0] uMCntl;                   // General Micro-Machine Control Field

wire    Prv;                            // Force complement of AR in AddrsGen
wire    TSiz;                           // Test SIZ signal
wire    Mod;                            // Force % 256 for NA, page in AL[15:8]

wire    En;                             // ALU Enable Control Field

wire    CE_IR, CE_OP2, CE_OP1;          // Clock Enables: IR, OP2, OP1
reg     [7:0] rIR, OP2, OP1;            // Internal Registers

wire    [1:0] Stk_Op;                   // Stack Pointer Op: NOP, POP, PSH

//  Instruction Decoder ROM

reg     [(pDEC_Width - 1):0] ID_ROM [(pDEC_Depth - 1):0]; // Inst. Decode ROM

//  Instruction Decoder Pipeline Register (Asynchronous Distributed ROM)

reg     [(pDEC_Width - 1):0] IDEC;      // Decode ROM Pipeline Reg.
wire    [7:0] IDEC_A;                   // Decode Address

//  Instruction Decoder (Fixed) Output

wire    [5:0] FU_Sel;                   // M65C02A ALU Functional Unit Select
wire    [1:0] Op;                       // M65C02A ALU Operation Select
wire    [1:0] QSel;                     // M65C02A ALU Q Operand Select
wire    [1:0] RSel;                     // M65C02A ALU R Operand Select
wire    [1:0] CSel;                     // M65C02A ALU Adder Carry In Select
wire    [2:0] WSel;                     // M65C02A ALU Register Write Select
wire    [2:0] OSel;                     // M65C02A ALU Output Select
wire    [3:0] CCSel;                    // M65C02A ALU Condition Code Control
wire    [7:0] Opcode;                   // M65C02A ALU Mask/Constants or Opcode

wire    [15:0] MAR;                     // M65C02A Memory Address Register (MAR)
wire    [15:0] PC;                      // M65C02A Program Counter (PC)
wire    SelS;                           // M65C02A Stack Pointer Select

wire    [15:0] ALU_DO;                  // M65C02A ALU Data Output Bus
wire    Valid;                          // M65C02A ALU Output Valid Signal
wire    CC;                             // ALU Condition Code Output

wire    ALU_Z;                          // M65C02A ALU Flags

reg     dTSZ;                           // ALU DO Multiplexer Control 16-bit Ops

wire    [15:0] VM;                      // FORTH VM IP/W Multiplexer Output

wor     [15:0] MuxDat;                  // Multiplexer Data: ALU_DO, MAR, PC
wor     [ 7:0] OutMux;                  // Data Output Multiplexer

reg     rBRV2;                          // M65C02A Latched BRV2 --> Trap
  
////////////////////////////////////////////////////////////////////////////////
//
//  Start Implementation
//
////////////////////////////////////////////////////////////////////////////////

//  Define Instruction Cycle Status Signals

assign Done = Via[0];           // Instruction Complete (1)     - BRV1 | BRV3
assign SC   = (&Via);           // Single Cycle Instruction (1) - BRV3             

//  Generate Rdy Signal: used as a clock enable for internal components

always @(*)
begin
    case({Done, (FU_Sel[5] & Reg_WE), Valid, ~Wait})
        4'b0000 : Rdy <= 0;
        4'b0001 : Rdy <= 1;     // Non-ALU external cycle ready
        4'b0010 : Rdy <= 0;
        4'b0011 : Rdy <= 1;     // Non-ALU external cycle ready
        4'b0100 : Rdy <= 0;
        4'b0101 : Rdy <= 1;     // Operands not ready, external cycle ready
        4'b0110 : Rdy <= 0;
        4'b0111 : Rdy <= 1;     // Operands not ready, external cycle ready
        4'b1000 : Rdy <= 0;
        4'b1001 : Rdy <= 1;     // Non-ALU op and external fetch ready
        4'b1010 : Rdy <= 0;
        4'b1011 : Rdy <= 1;     // Non-ALU op and external fetch ready
        4'b1100 : Rdy <= 0;     // ALU op and external fetch not ready
        4'b1101 : Rdy <= 0;     // ALU op not ready and external fetch ready
        4'b1110 : Rdy <= 0;     // ALU op ready and external fetch not ready
        4'b1111 : Rdy <= 1;     // ALU op and external fetch cycle ready
    endcase
end

////////////////////////////////////////////////////////////////////////////////
//
//  Microprogram Controller Interface
//

//  Decode MPC Instructions being used for strobes

assign BRV1 = (Via == pBRV1);
assign BRV2 = (Via == pBRV2);
assign BRV3 = (Via == pBRV3);

//  Define the MW[2:0] Multi-Way Input Signals

always @(*)
begin
    if(&I[3:2]) // Used by the BMWx MPC instructions
        case(I[1:0])
            2'b00 : MW <= {1'b0,  SIZ, 1'b0};   // #Imm Operand Instructions
            2'b01 : MW <= {1'b0, ~IND, ~SIZ};   // RMW Instructions
            2'b10 : MW <= {1'b0, ~IND, 1'b0};   // Other Instructions
            2'b11 : MW <= {1'b0, 1'b0, ~IND};   // Other Instructions
        endcase
    else        // Used by the BMW MPC instruction
        MW <= {2'b00, Int};
end

//  Implement the Branch Address Field Multiplexer for Instruction Decode

always @(*) BA <= ((Done) ? ((Int & SC) ? pInt_Hndlr
                                        : {{pBA_Fill{1'b1}}, DI[3:0], DI[7:4]})
                          : {{pBA_Fill{1'b0}}, uP_BA});

//  Assign Test Input Signals

assign T = {xIRQ, Int, ALU_Z, CC};

//  Generate TSZ Microprogram Control Signal
//      Conditionally modifies the MPC instruction

assign TSZ = SIZ & TSiz;    // if(1), substitute FTCH for MPC Instruction

//  Instantiate Microprogram Controller/Sequencer - modified F9408A MPC

M65C02A_MPC     #(
                    .pAddrWidth(pAddrWidth),
                    .pMPC_Stk(pMPC_Stk),
                    .pRst_Addrs(pRst_Addrs)
                ) MPC (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .Rdy(Rdy),              // MPC Clock Enable
                    
                    .I(((TSZ & ~PSH) ? pFTCH : I)), // Instruction 
                    .T(T),                  // Test signal inputs
                    .MW(MW),                // Multi-way branch inputs
                    .BA(BA),                // Branch address input
                    .Via(Via),              // BRVx multiplexer control output

                    .MA(MA)                 // Microprogram ROM address output
                );

//  Infer Microprogram ROM and initialize with file created by SMRTool

initial
    $readmemb(pM65C02A_uPgm, uP_ROM, 0, (pROM_Depth - 1));
    
always @(posedge Clk)
begin
    if(Rdy | Rst)
        uPL <= #1 uP_ROM[MA];
end

//  Assign uPL fields

assign I      = uPL[35:32];     // MPC Instruction Field (4)
assign uP_BA  = uPL[31:24];     // MPC Branch Address Field (8)
assign uPCntl = uPL[23:20];     // Microprogram Control Field (4)
assign NA_Op  = uPL[19:11];     // Next Address Operation (9)
assign IO_Op  = uPL[10: 9];     // IO Operation Control (2)
assign DI_Op  = uPL[ 7: 5];     // DI Demultiplexer Control (3)
assign DO_Op  = uPL[ 8: 5];     // DO Multiplexer Control (4) (same as DI_Op)
assign Reg_WE = uPL[    4];     // Register Write Enable Field (1)
assign ISR    = uPL[    3];     // Set to clr D and set I & M on interrupts (1)
assign uMCntl = uPL[ 2: 0];     // General Micro-Machine Control Field (3)

//  Decode uPCntl Field

assign Mod    = uPCntl[0];      // NA <= NA % 256, page set by AL[15:8]
assign TSiz   = uPCntl[1];      // Test SIZ prefix flag
assign Prv    = uPCntl[2];      // NA + ~AR + Ci
assign SPR    = uPCntl[3];      // SP-Relative Addressing Mode Enable

//  Stack Operation Select

assign Stk_Op = {NA_Op[6] & ~NA_Op[1], NA_Op[0]};   // 0x=>NOP; 10=>PSH; 11=>POP

//  Decode DI_Op Control Field

assign Ld_OP1 = IO_Op[1] & DI_Op[2];
assign Ld_OP2 = IO_Op[1] & DI_Op[1];
assign SignDI = DI_Op[0];

//  Operand Register 1 (Low Byte)

assign CE_OP1 = Ld_OP1 & Rdy & ~CE_IR;

always @(posedge Clk)
begin
    if(Rst | BRV2)
        OP1 <= #1 Vector[7:0];
    else if(CE_OP1)
        OP1 <= #1 DI;      
end

//  Operand Register 2 (High Byte)

assign CE_OP2 = Ld_OP2 & Rdy & ~CE_IR;

always @(posedge Clk)
begin
    if(Rst | BRV2)
        OP2 <= #1 Vector[15:8];
    else if(CE_OP2)
        OP2 <= #1 ((Ld_OP1) ? ((SignDI) ? {8{DI[7]}} : {8{1'b0}}) : DI);
end

//  Instruction Register
//       Load IR from DI | uP_BA 

assign CE_IR  = (BRV1 | (BRV3 & ~Int)) & Rdy;
assign IDEC_A = {DI[3:0], DI[7:4]}; 

always @(posedge Clk)
begin
    if(Rst | BRV2)
        rIR <= #1 pNOP;
    else if(CE_IR)   // IR: DI
        rIR <= #1 DI;
end

//  Assign Internal Registers

assign IR = rIR;
assign M  = ((COP_En) ? COP_DO : {OP2, OP1});

//  Co-Processor Interface

assign COP_En    = COP & Reg_WE;

assign COP_Sel   = COP & OP2[7:5];
assign COP_RA    = COP & OP2[2:1];
assign COP_Dir   = COP & OP2[0];
assign COP_DI    = COP & ALU_DO;

assign COP_Start = COP & (OP2[4:3] == 2'b01) & (OP2[2:1] == 2'b00);
assign COP_Clear = COP & (OP2[4:3] == 2'b10) & (OP2[2:1] == 2'b00);
assign COP_Pulse = COP & (OP2[4:3] == 2'b11) & (OP2[2:1] == 2'b00);

//
//  Co-Processor Status Interface
//      As implemented sets the V flag in PSW according to the desired test.
//
//      Alternative implementation, which may be better, would be to map the
//      Co-Processor status signals, Done and Busy, into N and V of PSW. A
//      Co-Processor status poll instruction would set the N and V flags in the
//      PSW so a normal processor branch instructions can be used to test the
//      condition. In other words, treat a co-processor status check as a BIT
//      instruction, setting N, V, and Z flags in the manner already defined in
//      the ALU module. MAM, 18H12.
//

//reg   COP_SO;
//
//always @(*)
//begin
//    case({COP_En & (OP2[2:0] == 3'b001), OP2[4:3]})
//        3'b100  : COP_SO <= ~COP_Busy;  // Set V if Co-Processor not Busy
//        3'b101  : COP_SO <=  COP_Busy;  // Set V if Co-Processor Busy
//        3'b110  : COP_SO <= ~COP_Done;  // Set V if Co-Processor not Done
//        3'b111  : COP_SO <=  COP_Done;  // Set V if Co-Processor Done
//        default : COP_SO <= 0;          // Otherwise do not set V 
//    endcase
//end
 
//  Infer Instruction Decode ROM and initialize with file created by SMRTool

initial
    $readmemb(pM65C02A_IDec, ID_ROM, 0, (pDEC_Depth - 1));

always @(posedge Clk)
begin
    if(Rst)
        IDEC <= #1 0;
    else if(CE_IR)
        IDEC <= #1 ID_ROM[IDEC_A];  
end

//  Decode Fixed Microcode Word

assign  Mode   = IDEC[35:33];       // M65C02A Instruction Type/Mode
assign  RMW    = IDEC[32];          // M65C02A Read-Modify-Write Instruction
assign  FU_Sel = IDEC[31:26];       // M65C02A ALU Functional Unit Select Field
assign  Op     = IDEC[25:24];       // M65C02A ALU Operation Select Field
assign  QSel   = IDEC[23:22];       // M65C02A ALU AU Q Bus Mux Select Field
assign  RSel   = IDEC[21:20];       // M65C02A ALU AU/SU R Bus Mux Select Field
assign  CSel   = IDEC[19:18];       // M65C02A ALU AU/SU Carry Mux Select Field
assign  WSel   = IDEC[17:15];       // M65C02A ALU Register Write Select Field
assign  OSel   = IDEC[14:12];       // M65C02A ALU Register Output Select Field
assign  CCSel  = IDEC[11: 8];       // M65C02A ALU Condition Code Control Field
assign  Opcode = IDEC[ 7: 0];       // M65C02A Valid Opcode Control Field

// Decode Mode Field

//  Prefix Instruction Mode Decodes

assign PFX  = (Mode == pMd_PFX);    // Prefix Instructions (Mode = 6)

assign iOSX = (PFX) & Opcode[4];    // Instruction is OSX | OSZ | OIS
assign iOAY = (PFX) & Opcode[3];    // Instruction is OAY
assign iOAX = (PFX) & Opcode[2];    // Instruction is OAX
assign iSIZ = (PFX) & Opcode[1];    // Instruction is SIZ | ISZ | OSZ
assign iIND = (PFX) & Opcode[0];    // Instruction is IND | ISZ | OIS

//  Special 1 Instruction Mode Decodes

assign SPC1 = (Mode == pMd_SPC1);   // Special 1 Instructions (Mode = 5)

assign PSH  = (SPC1) & Opcode[7];   // PHW zp/abs/#imm; PHA/PHX/PHY
assign PHR  = (SPC1) & Opcode[6];   // PHR rel16
assign MOV  = (SPC1) & Opcode[5];   // MOV Instruction
assign RTI  = (SPC1) & Opcode[4];   // RTI Instruction
assign DUP  = (SPC1) & Opcode[3];   // Accumulator Stack DUPlicate (16-bit only)
assign SWP  = (SPC1) & Opcode[2];   // Accumulator Stack SWaP      (16-bit only) 
assign ROT  = (SPC1) & Opcode[1];   // Accumulator Stack ROTate    (16-bit only)
assign WRD  = (SPC1) & Opcode[0] | (COP_En & COP_SIZ);  // Set 16-bit op size

//  Special 2 Instruction Mode Decodes

assign SPC2 = (Mode == pMd_SPC2);   // Special 2 Instructions (Mode = 1)

assign ADJ  = (SPC2) & Opcode[7];               // Adjust S: S <= S + M
//
assign LSCX = (SPC2) & Opcode[6];               // LDX/STX/CPX instruction
//
assign VEN  = (SPC2) & Opcode[1] & BRV3 & IND;  // Enable V for ASL/ROL A
assign CEN  = (SPC2) & Opcode[0] & BRV3 & IND;  // Enable C for INC/DEC A

//  Output indicator that special instruction being processed

assign SPC  = SPC1 | SPC2;

//  FORTH VM Instruction Mode Decodes 

assign FTH  = (Mode == pMd_FTH);    // FORTH VM Instructions (Mode = 4)

assign PHI  = (FTH) & Opcode[5];    // FORTH VM Push IP/W => RS
assign PLI  = (FTH) & Opcode[4];    // FORTH VM Pull IP/W <= RS
assign RSPX = (FTH) & Opcode[3];    // FORTH VM Use X for ENT/PHI/PLI if ~OSX

//  Remaining Mode Decodes: Mode = {3, 2}

assign BRK  = (Mode == pMd_BRK);    // Current Instruction is BRK     (Mode = 3)
assign COP  = (Mode == pMd_COP);    // Current Instruction is COP     (Mode = 2)

//  Define Core Operating Mode
//      During execution of an RTI instruction, the PSW value is read from the
//      kernel stack and written into P on the following cycle. The transition
//      from kernel to user mode must be delayed until the instruction fetch
//      cycle, indicated by the Done, so that the subsequent two pops of the
//      return address will come from the kernel stack.

always @(posedge Clk or posedge BRV1)
begin
    if(BRV1)
        rBRV2 <= #1 0;
    else if(BRV2)
        rBRV2 <= #1 1;
end

assign Kernel = (P[pMode] | ((BRV2 | rBRV2 | RTI) & ~Done));

//  Implement Prefix Instruction Registers:
//
//      IND - Address mode override maps zp => (zp) and abs => (abs)
//      SIZ - Operation Size override remaps operation from 8 to 16 bits
//      OAY - Override: A <=> Y;
//      OAX - Override: A <=> X;
//      OSX - Override: X <=> S; X => SP
//
//  IND, SIZ, OAX, OAY, and OSX are sticky flags which retain their programmed
//  state until the completion of the following non-prefix instruction.
//
//  OAX and OSX are mutually exclusive, and OAX and OAY are also mutually exclu-
//  sive. Applying OSX while OAX is set, will reset OAX; applying OAX while OSX
//  is set will reset OSX. Similarly, applying OAX while OAY is set will reset 
//  OAY; applying OAY while OAX is set will reset OAX. applying OAY while OSX is
//  set is allowed.
//
//  The PFX mode indicator will be set after the fetch of the first non-prefix
//  instruction, and will therefore preserve the state of the prefix flags. On
//  the CE_IR following a non-prefix instruction the PFX mode indicator will not
//  be set, so all prefix flags will be reset.

always @(posedge Clk)
begin
    if(Rst) begin
        OSX <= #1 0;                  // Initialize FFs
        IND <= #1 0;
        SIZ <= #1 pSIZ;
        OAX <= #1 0;
        OAY <= #1 0;
    end else if(CE_IR) begin
        // OSX: 8B      | CB | DB
        OSX <= #1 ((PFX) ? iOSX | OSX & ~iOAX         : 0);     
        // IND: 9B | BB      | DB
        IND <= #1 ((PFX) ? iIND | IND                 : 0);     
        // SIZ: AB | BB | CB | DB
        SIZ <= #1 ((PFX) ? ((pSIZ ^ iSIZ) | SIZ)      : pSIZ);  
        // OAX: EB
        OAX <= #1 ((PFX) ? iOAX | OAX & ~iOSX & ~iOAY : 0);
        // OAY: FB
        OAY <= #1 ((PFX) ? iOAY | OAY &         ~iOAX : 0);
    end
end

//  Next Address Generator

M65C02A_AddrGen     #(
                        .pDef_Page(pDef_Page),
                        .pSk_Rst(pSk_Rst),
                        .pSu_Rst(pSu_Rst)
                    ) AddrGen (
                        .Rst(Rst),          // System Reset 
                        .Clk(Clk),          // System Clock
                        
                        .Kernel(Kernel),    // Core Operating Mode
                        
                        .Vector(Vector),    // Interrupt/Trap Vector

                        .NA_Op(NA_Op),      // Next Address Control Code

                        .IND(IND),          // Prefix Instruction Flags 
                        .SIZ(SIZ),         
                        .OAX(OAX),
                        .OAY(OAY),
                        .OSX(OSX ^ RSPX),   // Use X as RS if RSPX & ~OSX
                        
                        .SPR(SPR & ~LSCX),  // SP-Relative Addressing Enable

                        .Mod(Mod),          // Mod 256 Addressing Flag
                        .Prv(Prv),          // Force complement of AR to dec NA

                        .CC(CC),            // Conditional Branch Condition Code 
                        .BRV3(BRV3),        // Single Cycle Instruction Complete
                        .Int(Int),          // Internal Interrupt/Trap Request

                        .Rdy(Rdy),          // Memory Cycle Complete Flag
                        .Valid(Valid),      // ALU Result Valid Flag

                        .M(M),              // Memory Operand Registers
                        .IP(VM),            // FORTH VM Instruction Pointer
                        
                        .X(X),              // ALU Index Register X
                        .Y(Y),              // ALU Index Register Y
                        .A(A),              // ALU Accumulator Register

                        .AO(AO),            // Address Out (Next Address)

                        .SelS(SelS),        // System Stack Pointer Select
                        .Stk_Op(Stk_Op),    // System Stack Pointer Operation
                        .SDI(ALU_DO),       // System Stack Pointer Data In
                        .SDO(S),            // System Stack Pointer Data Out
                        
                        .MAR(MAR),          // System Memory Address Register 
                        .PC(PC)             // System Program Counter Register
                    );

//  Interrupt Service Flag

always @(posedge Clk)
begin
    if(Rst)
        VP <= #1 0;
    else if(ISR)
        VP <= #1 Mod;
end

// Instantiate the FORTH VM module

M65C02A_ForthVM FVM (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .En(FTH | NA_Op[5]),
                    
                    .Rdy(Rdy), 

                    .IND(IND), 
                    .PHI(PHI), 
                    .PLI(PLI),
                    .DUP(DUP),

                    .VMCntl(uMCntl), 

                    .A(A),
                    .M(M), 
                    .T(VM), 

                    .IP(IP), 
                    .W(W)
                );

//  Instantiate the M65C02 ALU Module

assign En = Reg_WE;

M65C02A_ALUv2   ALUv2 (
                    .Rst(Rst),          // System Reset
                    .Clk(Clk),          // System Clock
                    
                    .Rdy(Rdy),          // Ready
                    
                    .En(En),            // M65C02A ALU Enable Strobe Input
                    .Reg_WE(Reg_WE),    // M65C02A ALU Register Write Enable
                    .ISR(ISR),          // M65C02A ALU Interrupt Svc Rtn Strobe
                    
                    .SO(SO),            // M65C02A ALU Set oVerflow Flag in PSW
                    .Clr_SO(Clr_SO),    // M65C02A ALU Clr SO - Acknowledge
                    
//                    .COP_SO(COP_SO),    // M65C02A ALU COP Set oVerflow Strobe
                    .COP_SO(1'b0),      // M65C02A ALU COP Set oVerflow Strobe
                    
                    .MOV(MOV),          // M65C02A ALU MOV Instruction Flag
                    .uMCntl(uMCntl),    // M65C02A ALU MOV Microprogram Controls
                    .SrcMode(OP2[1:0]), // M65C02A ALU MOV Source Pointer Mode
                    .DstMode(OP2[3:2]), // M65C02A ALU MOV Destination Ptr Mode
                    
                    .IND(IND),          // M65C02A ALU MOV CC Overrride Flag
                    .SIZ(SIZ | WRD | FTH | ADJ),    // M65C02A ALU Size Override
                    .OAX(OAX),          // M65C02A ALU Register Override
                    .OAY(OAY),          // M65C02A ALU Register Override
                    .OSX(OSX ^ RSPX),   // M65C02A ALU Stack Override
                    
                    .DUP(DUP),          // M65C02A ALU Register Stack DUPlicate
                    .SWP(SWP),          // M65C02A ALU Register Stack SWaP
                    .ROT(ROT),          // M65C02A ALU Register Stack ROTate
                    
                    .SelS(SelS),        // M65C02A ALU Stack Pointer Select
                    .S(S),              // M65C02A ALU Stack Pointer Input

                    .Mod(Mod),          // M65C02A ALU Aux Stack Ptr % 256 Input
                    .Stk_Op(Stk_Op),    // M65C02A ALU Aux Stack Ptr Ops
                    
                    .ADJ(ADJ),          // M65C02A Stack Adjust Instruction
                    .VEN(VEN),          // M65C02A Arithmetic Left Shift Enable
                    .CEN(CEN),          // M65C02A Enable C for INC/DEC A

                    .FU_Sel(FU_Sel[4:0]),   // M65C02A ALU Functional Unit Sel
                    .Op(Op),            // M65C02A ALU Operation Select
                    .QSel(QSel),        // M65C02A ALU Q Data Mux Select
                    .RSel(RSel),        // M65C02A ALU R Data Mux Select
                    .CSel(CSel),        // M65C02A ALU Adder Carry Select
                    .WSel(WSel),        // M65C02A ALU Register Write Select
                    .OSel(OSel),        // M65C02A ALU Output Register Select
                    .CCSel(CCSel),      // M65C02A ALU Condition Code Select
                    
                    .K(((SPC2) ? 8'b0 : Opcode)),   // M65C02A K Bus Input
                    .T(VM),             // M65C02A ALU FORTH VM Register Input
                    .M(M),              // M65C02A ALU Memory Operand Input

                    .DO(ALU_DO),        // M65C02A ALU Data Output Multiplexer
                    .Val(Valid),        // M65C02A ALU Output Valid Strobe
                    .CC_Out(CC),        // M65C02A ALU Condition Code Mux
                    
                    .ALU_C(),           // M65C02A ALU Carry Out
                    .ALU_Z(ALU_Z),      // M65C02A ALU Zero Out
                    .ALU_V(),           // M65C02A ALU OVerflow
                    .ALU_N(),           // M65C02A ALU Negative

                    .X(X),              // M65C02A ALU Index Register
                    .Y(Y),              // M65C02A ALU Index Register
                    .A(A),              // M65C02A ALU Accumulator Register

                    .P(P)               // M65C02A Processor Status Word Reg
                );

//  Decode P

assign IRQ_Msk = P[pIntMsk];            // Interrupt Mask Bit

//  Generate ALU DO Multiplexer Control for 16-bit Write Operations

always @(posedge Clk)
begin
    if(Rst)
        dTSZ <= #1 0;
    else
        dTSZ <= #1 TSZ;
end

//  External Bus Data Output

reg     Sel_P, Sel_DL, Sel_DH;

assign HB = ((SIZ) ?  (^{dTSZ, PSH}) : 1'b0);
assign LB = ((SIZ) ? ~(^{dTSZ, PSH}) : 1'b1);

always @(*)
begin
    case(DO_Op)
        4'b0000 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b0001 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, LB,   HB  };    // ALU
        4'b0010 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b1};    // DMH
        4'b0011 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b1};    // DOH
        4'b0100 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b1, 1'b0};    // DML
        4'b0101 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b1, 1'b0};    // DOL
        4'b0110 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b0111 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1000 : {Sel_P, Sel_DL, Sel_DH} <= {1'b1, 1'b0, 1'b0};    // P
        4'b1001 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1010 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1011 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1100 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1101 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1110 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
        4'b1111 : {Sel_P, Sel_DL, Sel_DH} <= {1'b0, 1'b0, 1'b0};    // NO OUTPUT
    endcase
end

assign MuxDat = ((DO_Op[0]) ? ALU_DO : ((PHR) ? MAR : PC));
 
assign OutMux = ((Sel_DH) ? MuxDat[15:8] : 0);
assign OutMux = ((Sel_DL) ? MuxDat[ 7:0] : 0);
assign OutMux = ((Sel_P ) ? P            : 0);

assign DO = OutMux;

//  Assign External Interrupt Handler Control Signals

assign LE_Int = BRV2;

////////////////////////////////////////////////////////////////////////////////
//
//  End Implementation
//
////////////////////////////////////////////////////////////////////////////////

endmodule
