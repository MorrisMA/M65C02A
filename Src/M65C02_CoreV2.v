////////////////////////////////////////////////////////////////////////////////
//
//  M65C02A soft-core module for M65C02A soft-core microcomputer project.
// 
//  Copyright 2013-2014 by Michael A. Morris, dba M. A. Morris & Associates
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

`timescale 1ns / 100ps

////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris 
// 
// Create Date:     07:12:56 09/17/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_CoreV2.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:     (See additional comments section below)
//
// Dependencies:    M65C02_MPCv5.v
//                      M65C02_uPgm_V4.coe (M65C02_uPgm_V4.txt)
//                      M65C02_IDecode_ROM.coe (M65C02_IDecode_ROM.txt)
//                  M65C02_AddrGenV2.v
//                      M65C02_StkPtr.v
//                  M65C02_ALU_v2.v
//                      M65C02_LST.v
//                      M65C02_LU.v
//                      M65C02_SU.v
//                      M65C02_Add.v
//                      M65C02_WrSel.v
//                      M65C02_PSWv2.v
//
// Revision: 
//
//  1.00    13I17   MAM     Forked from M65C02_Core.v. Changed the MPC from
//                          M65C02_MPCv4.v to M65C02_MPCv5.v. The v5 MPC does
//                          not contain the microcycle length controller. That
//                          function will either be implemented as a separate
//                          module in the core, or it will not be used. If not
//                          used, then the core will resemble the original core,
//                          M65C02_Base.v, and will require the memory interface
//                          to insert any wait states needed to access external
//                          memory.
//
//  1.00    14F28   MAM     Updated comments to reflect changes made during the
//                          integration and testing of the module: (1) updated
//                          the component list; (2) removed unused unregistered
//                          input ID and adjusted the instruction decode mecha-
//                          to use DI instead; (3) adjusted the NA_Op field to
//                          reflect optimizations made to the Address Generator
//                          module; (4) used freed NA_Op microprogram bits to
//                          provide an additional control field used to prepare
//                          the module to support full microprogram control of
//                          the core to support virtual machines, new instruc-
//                          tions, etc.; (5) separated OP1 from DI and made a
//                          separate register (as in previous versions of the
//                          core) and adjusted connections to ALU; (6) added 
//                          explicit control of address mod 256 operations using
//                          the new (overloaded) microprogram control field;
//                          (7) Changed the IntSvc FF to perform the Vector Pull
//                          function using the microprogram ISR control and one
//                          of the bits in the new microprogram control field;
//                          (8) adjusted default reset value of the stack poin-
//                          ter to 2 to support modification to reset behavior 
//                          of the core which now pushes the PC and PSW to the
//                          stack on reset; and (9) adjusted the connections of
//                          the Tmp and M operand ports on the ALU module.
//
//  2.00    14H09   MAM     Added multiplexer, controlled by Sel_BA (uP_Cntl==1)
//                          to support PHW (PEI/PEA/PHW) and PHR (PER) instruc-
//                          tions. Modifications allow {OP2, OP1} to be pushed
//                          onto the stack using PCH and PCL as the selects.
//
//  2.10    14H16   MAM     Added an additional qualifier to the CE_IR signal so
//                          that the IR is not loaded unless the Sel_BA is also
//                          asserted.
//
//  2.11    14K14   MAM     Adjusted default uPgm and IDec files to correspond
//                          to last tested configuration.
//
//  2.20    14K18   MAM     Modified the definitions of the prefix instructions.
//                          IND and SIZ may now be specified independently or
//                          simultaneously using three opcodes. OAX and OAY are
//                          now specified using two other opcodes. A total of
//                          five opcodes are needed to specify the prefixes.
//
//  2.30    14L06   MAM     Completed the incorporation of the register override
//                          prefix instructions. OAX, OAY, and OSY are now fully
//                          supported by the M65C02A. Also, prepared the module
//                          for the incoporation of the register stack support
//                          instructions and the the Forth VM instructions.
// 
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_CoreV2 #(
    parameter pAddrWidth   = 9,     // MPC Address Width
    parameter pMPC_Stk     = 0,     // MPC Stack Select: 0 - 1 Lvl; 1 - 4 Lvl
    parameter pRst_Addrs   = 0,     // MPC Reset Address
        
    parameter pInt_Hndlr   = 0,     // _Int microroutine address, Reset default
    parameter pM65C02_uPgm = "Pgms/M65C02_uPgm_V4a.coe",
    parameter pM65C02_IDec = "Pgms/M65C02_IDecode_ROMa.coe",

    parameter pStkPtr_Rst  = 2      // Stk Ptr Value after Reset
)(
    input   Rst,            // System Reset Input
    input   Clk,            // System Clock Input
    
    //  Processor Core Interrupt Interface
    
    output  IRQ_Msk,        // Interrupt mask from P to Interrupt Handler
    output  LE_Int,         // Interrupt Latch Enable - Hold Int until serviced

    input   Int,            // Interrupt input from Interrupt Handler
    input   xIRQ,           // External Maskable Interrupt Request Input
    input   [15:0] Vector,  // ISR Vector from Interrupt Handler

    output  reg VP,         // Interrupt Vector Pull Indicator
    
    input   SO,             // Set oVerflow Flag in PSW
    output  Clr_SO,         // Clr SO Command - Acknowledge
    
    //  Processor Core Status Interface
    
    output  Done,           // Instruction Complete/Fetch Strobe
    output  SC,             // Single Cycle Instruction
    output  [2:0] Mode,     // Mode - Instruction Type/Mode
    output  RMW,            // Read-Modify-Write Operation
    
    //  Processor Core Memory Controller Interface
    
    input   Wait,           // Wait Input
    output  reg Rdy,        // Internal Ready
    
    //  Processor Core Memory Interface    
    
    output  [ 1:0] IO_Op,   // Instruction Fetch Strobe
    output  [15:0] AO,      // External Address
    input   [ 7:0] DI,      // External Data In (Registered Data Path)
    output  [ 7:0] DO,      // External Data Out
    
    output  [ 7:0] X,       // Internal Processor Registers - Index Register X
    output  [ 7:0] Y,       // Internal Processor Registers - Index Register Y
    output  [ 7:0] A,       // Internal Processor Registers - Accumulator

    output  [ 7:0] P,       // Internal Processor Status Word
    
    output  reg [ 7:0] OP1, // Internal Temporary/Operand Registers
    output  reg [ 7:0] OP2,
    output  reg [ 7:0] IR,  // Instruction Register
    
    output  reg IND,        // Indirect Addressing Mode Override
    output  reg SIZ,        // Size Override 
    output  reg OAX,        // Override Op(A) with Op(X)
    output  reg OAY,        // Override Op(A) with Op(Y)
    output  reg OSY         // Override Op(S) with Op(Y) 
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

localparam  pBRV1    = 2'b01;   // MPC Via[1:0] code for BRV1 instruction
localparam  pBRV2    = 2'b10;   // MPC Via[1:0] code for BRV2 instruction
localparam  pBRV3    = 2'b11;   // MPC Via[1:0] code for BRV3 instruction
localparam  pBMW     = 4'b0011; // MPC I[3:0] code for BMW instruction

localparam  pIO_WR   = 2'b01;   // Memory Write
localparam  pIO_RD   = 2'b10;   // Memory Read
localparam  pIO_IF   = 2'b11;   // Instruction Fetch

localparam  pDO_ALU  = 2'b00;   // DO <= ALU_Out
localparam  pDO_MDH  = 2'b01;   // DO <= ((PHW) ? OP2:((PHR) ? Hi(MAR):Hi(PC)))
localparam  pDO_MDL  = 2'b10;   // DO <= ((PHW) ? OP1:((PHR) ? Lo(MAR):Lo(PC)))
localparam  pDO_PSW  = 2'b11;   // DO <= P (also available on ALU_Out)
//
localparam  pDI_Mem  = 2'b00;   // ALU_M <= DI
localparam  pDI_OP2  = 2'b01;   // OP2   <= DI
localparam  pDI_OP1  = 2'b10;   // OP1   <= DI
localparam  pDI_IR   = 2'b11;   // IR    <= DI

localparam  pNOP     = 8'hEA;   // NOP opcode

localparam  pIntMsk  = 2;       // Bit number of Interrupt mask bit in P

//  Fixed Microcode Mode Field Decode

localparam  pMd_WAI  = 7;       // WAI Instruction
localparam  pMd_PFX  = 6;       // Prefix Operations
localparam  pMd_SPC  = 5;       // Special Operations: PHW xxx, and PHR rel16
localparam  pMd_FTH  = 4;       // FORTH VM Operations
localparam  pMd_BRK  = 3;       // BRK Instruction
localparam  pMd_COP  = 2;       // COP Instruction
localparam  pMd_INV  = 1;       // Invalid Instructions
localparam  pMd_VAL  = 0;       // Valid Instructions

////////////////////////////////////////////////////////////////////////////////
//
// Local Signal Declarations
//

wire    WAI;                    // Instruction Mode Decode for WAIt
wire    PFX;                    // Instruction Mode Decode for PReFix
wire    SPC;                    // Instruction Mode Decode for SPeCial
//wire    FTH;                    // Instruction Mode Decode for ForTH
wire    BRK;                    // Instruction Mode Decode for BReaK
wire    COP;                    // Instruction Mode Decode for CO-Processor
wire    INV;                    // Instruction Mode Decode for INValid
wire    VAL;                    // Instruction Mode Decode for VALid

wire    WOP;                    // Instruction Mode Decode for PHW/PLW
wire    AOP;                    // Instruction Mode Decode for PHR

//wire    NXT;                    // Instruction Mode Decode for NXT
//wire    ENT;                    // Instruction Mode Decode for ENT
//wire    PHI;                    // Instruction Mode Decode for PHI
//wire    INI;                    // Instruction Mode Decode for INI

//wire    DUP;                    // Instruction Mode decode for DUP
//wire    SWP;                    // Instruction Mode decode for SWP
//wire    ROT;                    // Instruction Mode decode for ROT
wire    iOSY;                   // Instruction Mode Decode for OSY/OAS
wire    iOAY;                   // Instruction Mode Decode for OAY
wire    iOAX;                   // Instruction Mode Decode for OAX
wire    iSIZ;                   // Instruction Mode Decode for SIZ | ISZ
wire    iIND;                   // Instruction Mode Decode for IND | ISZ

wire    BRV1;                           // MPC BRV1 Instruction Decode
wire    BRV3;                           // MPC BRV3 Instruction Decode

reg     [(pROM_Width - 1):0] uP_ROM [(pROM_Depth - 1):0]; // Microprogram ROM

wire    [3:0] I;                        // MPC Instruction Input
wire    [3:0] T;                        // MPC Test Inputs
wire    [2:0] MW;                       // MPC Multi-way Branch Select
reg     [(pROM_AddrWidth - 1):0] BA;    // MPC Branch Address Input
wire    [(pROM_AddrWidth - 1):0] MA;    // MPC uP ROM Address Output
wire    [1:0] Via;                      // MPC Via Mux Control Output

reg     [(pROM_Width - 1)    :0] uPL;   // MPC uP ROM Pipeline Register
wire    [(pROM_AddrWidth - 2):0] uP_BA; // uP Branch Address Field

wire    [ 2:0] uPCntl;                  // Microprogram Control Field
wire    [10:0] NA_Op;                   // Memory Address Register Control Fld
wire    [ 3:0] DI_Op;                   // Memory Data Input Control Field
wire    [ 3:0] DO_Op;                   // Memory Data Output Control Field
wire    [ 2:0] Reg_WE;                  // Register Write Enable Control Field

wire    En;                             // ALU Enable Control Field

wire    CE_IR, CE_OP1, CE_OP2;          // Clock Enables: IR, OP1

wire    [1:0] Stk_Op;                   // Stack Pointer Op: NOP, POP, PSH

//  Instruction Decoder ROM

reg     [(pDEC_Width - 1):0] ID_ROM [(pDEC_Depth - 1):0]; // Inst. Decode ROM

//  Instruction Decoder Pipeline Register (Asynchronous Distributed ROM)

reg     [(pDEC_Width - 1):0] IDEC;  // Instruction Decode ROM Pipeline Reg.
wire    [7:0] IDEC_A;               // Instruction Decode Address

//  Instruction Decoder (Fixed) Output

wire    [5:0] FU_Sel;               // M65C02 ALU Functional Unit Select Field
wire    [1:0] Op;                   // M65C02 ALU Operation Select Field
wire    [1:0] QSel;                 // M65C02 ALU Q Operand Select Field
wire    [1:0] RSel;                 // M65C02 ALU R Operand Select Field
wire    [1:0] CSel;                 // M65C02 ALU Adder Carry In Select Field
wire    [2:0] WSel;                 // M65C02 ALU Register Write Select Field
wire    [2:0] OSel;                 // M65C02 ALU Output Select Field
wire    [3:0] CCSel;                // M65C02 ALU Condition Code Control Field
wire    [7:0] Opcode;               // M65C02 Rockwell Instruction Mask Field
  
wire    [15:0] MAR;                 // M65C02 Memory Address Register (MAR)
wire    [15:0] PC;                  // M65C02 Program Counter (PC)
wire    [ 7:0] S;                   // M65C02 Stack Pointer (S)

wire    [7:0] DO_ALU;               // M65C02 ALU Data Output Bus
wire    Valid;                      // M65C02 ALU Output Valid Signal
wire    CC;                         // ALU Condition Code Output

wire    SelS;                       // Stack Pointer Select

wire    [15:0] MuxDat;              // Multiplexer Data: {OP2,OP1}, MAR, PC
wor     [ 7:0] OutMux;              // Data Output Multiplexer

////////////////////////////////////////////////////////////////////////////////
//
//  Start Implementation
//
////////////////////////////////////////////////////////////////////////////////

//  Define Instruction Cycle Status Signals

assign Done = (|Via);               // Instruction Complete (1)     - ~BRV0
assign SC   = (&Via);               // Single Cycle Instruction (1) -  BRV3             

//  Generate Rdy Signal: used as a clock enable for internal components

always @(*)
begin
    case({Done, (FU_Sel[5] & |Reg_WE), Valid, ~Wait})
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

//  Define the Multi-Way Input Signals
//      Implement a 4-way branch when executing WAI, and a 2-way otherwise

assign MW = ((WAI) ? {uP_BA[2], xIRQ, Int} : {uP_BA[2:1], Int});

//  Implement the Branch Address Field Multiplexer for Instruction Decode

always @(*)
begin
    case(Via)
        pBRV1   : BA <= {{pBA_Fill{BRV1}}, DI[3:0], DI[7:4]};
        pBRV3   : BA <= ((Int) ? pInt_Hndlr
                               : {{pBA_Fill{BRV3}}, DI[3:0], DI[7:4]});
        default : BA <= {{pBA_Fill{1'b0}}, uP_BA};
    endcase
end

//  Assign Test Input Signals

assign T = {IND, SIZ, OAY, OAX};

//  Instantiate Microprogram Controller/Sequencer - modified F9408A MPC

M65C02_MPCv5    #(
                    .pAddrWidth(pAddrWidth),
                    .pMPC_Stk(pMPC_Stk),
                    .pRst_Addrs(pRst_Addrs)
                ) MPCv5 (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .Rdy(Rdy),              // MPC Clock Enable
                    
                    .I(I),                  // Instruction 
                    .T(T),                  // Test signal input
                    .MW(MW),                // Multi-way branch inputs
                    .BA(BA),                // Branch address input
                    .Via(Via),              // BRVx multiplexer control output

                    .MA(MA)                 // Microprogram ROM address output
                );

//  Infer Microprogram ROM and initialize with file created by SMRTool

initial
    $readmemb(pM65C02_uPgm, uP_ROM, 0, (pROM_Depth - 1));
    
always @(posedge Clk)
begin
    if(Rdy | Rst)
        uPL <= #1 uP_ROM[MA];
end

//  Assign uPL fields

assign I      = uPL[35:32];     // MPC Instruction Field (4)
assign uP_BA  = uPL[31:24];     // MPC Branch Address Field (8)
assign uPCntl = uPL[23:22];     // Microprogram Control Field (2)
assign NA_Op  = uPL[21:10];     // Next Address Operation (12) (uPL[21]: Rsvd)
assign IO_Op  = uPL[9:8];       // IO Operation Control (2)
assign DI_Op  = uPL[7:4];       // DI Demultiplexer Control (4)
assign DO_Op  = uPL[7:4];       // DO Multiplexer Control (4) (same as DI_Op)
assign Reg_WE = uPL[3:1];       // Register Write Enable Field (3)
assign ISR    = uPL[0];         // Set to clear D and set I on interrupts (1)

//  Decode uPCntl Field

//  Stack Operation Select

assign Stk_Op = {NA_Op[7] & ~NA_Op[9], NA_Op[0]};   // 0x: nop; 10: Psh; 11: Pop

// Mod256: Page 0 == 3'b010; Page 1 = 3'b011

assign Mod256 = (uPCntl == 2);  // Explicit control address wrap in page
assign Sel_BA = (uPCntl == 1);  // Explicit control of IR source

//  Decode DI_Op Control Field

assign Ld_OP1 = IO_Op[1] &  DI_Op[2];
assign Ld_OP2 = IO_Op[1] &  DI_Op[1];

//  Operand Register Data Input Bus

wire [7:0] OP_DI = ((Sel_BA) ? uP_BA : DI);

//  Operand Register 1

assign CE_OP1 = Ld_OP1 & Rdy & ~CE_IR;

always @(posedge Clk)
begin
    if(Rst | BRV2)
        OP1 <= #1 Vector[7:0];
    else if(CE_OP1)
        OP1 <= #1 OP_DI;      
end

//  Operand Register 2

assign CE_OP2 = Ld_OP2 & Rdy & ~CE_IR;
assign SignDI = DI_Op[0];

always @(posedge Clk)
begin
    if(Rst | BRV2)
        OP2 <= #1 Vector[15:8];
    else if(CE_OP2)   // OP2: {sign(OP_DI),   OP_DI}
        OP2 <= #1 ((SignDI) ? {8{OP_DI[7]}} : OP_DI);
end

//  Instruction Register
//       Load IR from DI | uP_BA 

assign CE_IR  = (BRV1 | (BRV3 & ~Int) | (Sel_BA & DI_Op[3])) & Rdy;
assign IDEC_A = ((Sel_BA) ? {uP_BA[3:0], uP_BA[7:4]} : {DI[3:0], DI[7:4]}); 

always @(posedge Clk)
begin
    if(Rst | BRV2)
        IR <= #1 pNOP;
    else if(CE_IR)   // IR: {uP_BA,  DI}
        IR <= #1 ((Sel_BA) ? uP_BA : DI);
end

//  Infer Instruction Decode ROM and initialize with file created by SMRTool

initial
    $readmemb(pM65C02_IDec, ID_ROM, 0, (pDEC_Depth - 1));

always @(posedge Clk)
begin
    if(Rst)
        IDEC <= #1 0;
    else if(CE_IR)
        IDEC <= #1 ID_ROM[IDEC_A];
end

//  Decode Fixed Microcode Word

assign  Mode   = IDEC[35:33];       // M65C02 Instruction Type/Mode
assign  RMW    = IDEC[32];          // M65C02 Read-Modify-Write Instruction
assign  FU_Sel = IDEC[31:26];       // M65C02 ALU Functional Unit Select Field
assign  Op     = IDEC[25:24];       // M65C02 ALU Operation Select Field
assign  QSel   = IDEC[23:22];       // M65C02 ALU AU Q Bus Mux Select Field
assign  RSel   = IDEC[21:20];       // M65C02 ALU AU/SU R Bus Mux Select Field
assign  CSel   = IDEC[19:18];       // M65C02 ALU AU/SU Carry Mux Select Field
assign  WSel   = IDEC[17:15];       // M65C02 ALU Register Write Select Field
assign  OSel   = IDEC[14:12];       // M65C02 ALU Register Output Select Field
assign  CCSel  = IDEC[11: 8];       // M65C02 ALU Condition Code Control Field
assign  Opcode = IDEC[ 7: 0];       // M65C02 Valid Opcode Control Field

//// Decode Mode for internal control signals

assign WAI  = (Mode == pMd_WAI);    // Instruction is WAI
//
assign PFX  = (Mode == pMd_PFX);    // Prefix Instructions
//assign DUP  = (PFX) & Opcode[7];    // Accumulator Stack DUPlicate
//assign SWP  = (PFX) & Opcode[6];    // Accumulator Stack SWaP
//assign ROT  = (PFX) & Opcode[5];    // Accumulator Stack ROTate
assign iOSY = (PFX) & Opcode[4];    // Instruction is OAY
assign iOAY = (PFX) & Opcode[3];    // Instruction is OAY
assign iOAX = (PFX) & Opcode[2];    // Instruction is OAX
assign iSIZ = (PFX) & Opcode[1];    // Instruction is SIZ | ISZ
assign iIND = (PFX) & Opcode[0];    // Instruction is IND | ISZ
//
assign SPC  = (Mode == pMd_SPC);
assign WOP  = (SPC) & Opcode[7];    // PHW zp/abs/#imm; PLW zp/abs;
assign AOP  = (SPC) & Opcode[6];    // PHR rel16
//
//  (Mode == pMd_FTH) -- Reserved for FORTH VM instructions
//
assign BRK  = (Mode == pMd_BRK);        // Current Instruction is BRK
assign COP  = (Mode == pMd_COP);        // Current Instruction is COP
assign INV  = (Mode == pMd_INV);        // Current Instruction is Invalid
assign VAL  = (Mode == pMd_VAL);        // Current Instruction is Valid

//  Implement Prefix Registers
//      IND     - Address mode override maps zp => (zp) and abs => (abs)
//      SIZ     - Operation Size override remaps operation from 8 to 16 bits
//      OAX/OAY - Overrides: OAX - A<=>X; OAY - A<=>Y
//      OSY     - Override:  OSY - Y<=>S; Y => SP

always @(posedge Clk)
begin
    if(Rst)
        {OSY, OAY, OAX, SIZ, IND} <= #1 0;
    else if(CE_IR) begin
        IND <= #1 ((PFX) ? iIND | IND                 : 0); // IR == 0x9B | 0xBB
        SIZ <= #1 ((PFX) ? iSIZ | SIZ                 : 0); // IR == 0xAB | 0xBB
        OAX <= #1 ((PFX) ? iOAX | OAX &         ~iOAY : 0); // IR == 0xEB
        OAY <= #1 ((PFX) ? iOAY | OAY & ~iOSY & ~iOAX : 0); // IR == 0xFB
        OSY <= #1 ((PFX) ? iOSY | OSY & ~iOAY         : 0); // IR == 0x8B
    end
end

//  Next Address Generator

M65C02_AddrGenV2    #(
                        .pStkPtr_Rst(pStkPtr_Rst)
                    ) AddrGen (
                        .Rst(Rst), 
                        .Clk(Clk),
                        
                        .Vector(Vector),

                        .NA_Op(NA_Op),

                        .OAX(OAX),
                        .OAY(OAY),
                        .OSY(OSY),

                        .Mod256(Mod256),

                        .CC(CC), 
                        .BRV3(BRV3), 
                        .Int(Int), 

                        .Rdy(Rdy),
                        .Valid(Valid),

                        .OP1(OP1),
                        .OP2(OP2),
                        
                        .X(X),
                        .Y(Y),
                        .A(A),

                        .AO(AO), 

                        .SelS(SelS),
                        .Stk_Op(Stk_Op),
                        .SDI(DO_ALU),
                        .SDO(S),
                        
                        .MAR(MAR),
                        .PC(PC)
                    );

//  Interrupt Service Flag

always @(posedge Clk)
begin
    if(Rst)
        VP <= #1 0;
    else if(ISR)
        VP <= #1 Mod256; // Multi-purpose microprogram fld, use care
end

//  Instantiate the M65C02 ALU Module

assign En = (|Reg_WE);

M65C02_ALUv2    ALU (
                    .Rst(Rst),          // System Reset
                    .Clk(Clk),          // System Clock
                    
                    .Rdy(Rdy),          // Ready
                    
                    .En(En),            // M65C02 ALU Enable Strobe Input
                    .Reg_WE(Reg_WE),    // M65C02 ALU Register Write Enable
                    .ISR(ISR),          // M65C02 ALU Interrupt Service Rtn Strb
                    
                    .SO(SO),            // M65C02 ALU Set oVerflow Flag in PSW
                    .Clr_SO(Clr_SO),    // M65C02 ALU Clr SO - Acknowledge
                    
                    .SIZ(SIZ),          // M65C02 ALU Operation Size Override
                    .OAX(OAX),          // M65C02 ALU Register Override
                    .OAY(OAY),          // M65C02 ALU Register Override
                    .OSY(OSY),          // M65C02 ALU Stack Override
                    
                    .SelS(SelS),        // M65C02 ALU Stack Pointer Select
                    .S(S),              // M65C02 ALU Stack Pointer

                    .Stk_Op(Stk_Op),    // M65C02 ALU Y Stck Ops

                    .FU_Sel(FU_Sel[4:0]),   // M65C02 ALU Functional Unit Select
                    .Op(Op),            // M65C02 ALU Operation Select
                    .QSel(QSel),        // M65C02 ALU Q Data Mux Select
                    .RSel(RSel),        // M65C02 ALU R Data Mux Select
                    .CSel(CSel),        // M65C02 ALU Adder Carry Select
                    .WSel(WSel),        // M65C02 ALU Register Write Select
                    .OSel(OSel),        // M65C02 ALU Output Register Select
                    .CCSel(CCSel),      // M65C02 ALU Condition Code Select
                    
                    .K(Opcode),         // M65C02 ALU Rockwell Instruction Mask
                    .Tmp(OP2),          // M65C02 ALU Temporary Holding Register
                    .M(OP1),            // M65C02 ALU Memory Operand

                    .DO(DO_ALU),        // M65C02 ALU Data Output Multiplexer
                    .Val(Valid),        // M65C02 ALU Output Valid Strobe
                    .CC_Out(CC),        // M65C02 ALU Condition Code Mux

                    .X(X),              // M65C02 ALU Index Register
                    .Y(Y),              // M65C02 ALU Index Register
                    .A(A),              // M65C02 ALU Accumulator Register

                    .P(P)               // M65C02 Processor Status Word Register
                );

//  Decode P

assign IRQ_Msk = P[pIntMsk];            // Interrupt Mask Bit

//  External Bus Data Output

assign MuxDat = ((WOP) ? {OP2, OP1} : ((AOP) ? MAR : PC));

assign OutMux = ((DO_Op[0]) ? DO_ALU       : 0);
assign OutMux = ((DO_Op[1]) ? MuxDat[15:8] : 0);
assign OutMux = ((DO_Op[2]) ? MuxDat[ 7:0] : 0);
assign OutMux = ((DO_Op[3]) ? P            : 0);

assign DO = OutMux;

//  Assign External Interrupt Handler Control Signals

assign LE_Int = BRV2;

////////////////////////////////////////////////////////////////////////////////
//
//  End Implementation
//
////////////////////////////////////////////////////////////////////////////////

endmodule
