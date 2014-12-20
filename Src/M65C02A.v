////////////////////////////////////////////////////////////////////////////////
//
//  M65C02A soft-core microcomputer project
//
//  Copyright (C) 2012-2014  Michael A. Morris
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
// Company:         M. A. Morris & Assoc.
// Engineer:        Michael A. Morris
// 
// Create Date:     12:49:16 11/18/2012 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02A.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This module provides a synthesizable implementation of a 65C02 micropro-
//  cessor similar to the WDC W65C02S. The original W65C02 implemented a set of
//  enhancements to the MOS6502 microprocessor. Two new addressing modes were
//  added, several existing instructions were rounded out using the new address-
//  ing modes, and some additional instructions were added to fill in holes pre-
//  sent in the instruction set of the MOS6502. Rockwell second sourced the
//  W65C02, and in the process added 4 bit-oriented instructions using 32 op-
//  codes. WDC released the W65816/W65802 16-bit enhancements to the W65C02. Two
//  of the new instructions in these processors, WAI and STP, were combined with
//  the four Rockwell instructions, RMBx/SMBx and BBRx/BBSx, along with the ori-
//  ginal W65C02's instruction set to realize the W65C02S.
//
//  The M65C02A core is a realization of the W65C02S instruction set. It is not
//  a cycle accurate implementation, and it does not attempt to match the idio-
//  syncratic behavior of the W65C02 or the W65C02S with respect to unused op-
//  codes. In the M65C02A core, all unused opcodes are realized as single byte,
//  single cycle NOPs.
//
//  This module demonstrates how to incorporate the M65C02_CoreV2.v processor
//  core module into an application-specific implementation. The core logic in-
//  corporates most of the logic required for a microprocessor implementation:
//  ALU, registers, address generator, and instruction decode and sequencing.
//  Not included in the core logic are the memory interface, the interrupt hand-
//  ler, the clock generator, and any peripherals.
//
//  This module integrates the M65C02_CoreV2.v module, a simple vectored inter-
//  rupt controller, a simple MMU, and 28kB of internal Block RAM memory. One
//  objective is a module that allows Klaus Dormann's 6502_functional_test code
//  to be executed at speed from internal BRAM to verify that the new core
//  architecture and microprogram implement the 65C02 ISA. 
//
// Dependencies:
//      M65C02A.v                           (M65C02 Microcomputer: RAM/ROM/IO)
//          M65C02_CoreV2.v                 (M65C02 Processor Core)
//              M65C02_MPCv5.v              (M65C02 Microprogram Controller)
//                  M65C02_uPgm_V4a.coe     (M65C02_uPgm_V4a.txt)
//                  M65C02_IDecode_ROMa.coe (M65C02_IDecode_ROMa.txt)
//              M65C02_AddrGenV2.v          (M65C02 Address Generator)
//                  M65C02_StkPtr.v         (M65C02 Stack Pointer)
//              M65C02_ALUv2.v              (M65C02 ALU)
//                  M65C02_LST.v            (M65C02 ALU Load/Store/Transfer Mux)
//                  M65C02_LU.v             (M65C02 ALU Logic Unit) 
//                  M65C02_SU.v             (M65C02 ALU Shift/Rotate Unit)
//                  M65C02_Add.v            (M65C02 ALU Dual Mode Adder Unit) 
//                  M65C02_WrSel.v          (M65C02 ALU Register Write Decoder)
//                  M65C02_StkPtr.v         (M65C02 ALU Alternate Stack Pointer)
//                  M65C02_PSWv2.v          (M65C02 ALU Processor Status Word)
//          M65C02_IntHndlrV2.v             (Interrupt Handler)
//          M65C02_MMU.v                    (Memory Management Unit)
//          M65C02_SPIxIF.v                 (SPI Master I/F)
//              DPSFmnCE.v                  (Transmit & Receive FIFOs)
//              SPIxIF.v                    (Configurable SPI Master)
//                  fedet.v                 (falling edge detector)
//                  redet.v                 (rising edge detector)
//          UART.v                          (COM0/COM1 Asynch. Serial Ports)
//              DPSFmnCS.v                  (Transmit & Receive FIFOs)
//              UART_BRG.v                  (UART Baud Rate Generator)
//              UART_TXSM.v                 (UART Transmit SM/Shifter)
//              UART_RXSM.v                 (UART Receive SM/Shifter)
//              UART_INT.v                  (UART Interrupt Generator)
//                  fedet.v                 (falling edge detector)
//                  redet.v                 (rising edge detector)
//
// Revision: 
//
//  0.00    14F22   MAM     Initial File Creation
//
//  0.10    14G04   MAM     First integrated module.
//
//  1.00    14G19   MAM     Last Version before Kernel/User Mode added.
//
//  1.01    14K14   MAM     Adjusted default uPgm and IDec filenames to synch
//                          up with the final tested configuration.
//
//  1.02    14K15   MAM     Updated comments
//
//  1.10    14K27   MAM     Incorporated changes to support the defined prefix
//                          instructions. Modified the IO Page memory map, and
//                          changed the ALU, Address Generator, and MMU. The ALU
//                          and Address Generator were changed to support the
//                          prefix instructions. The MMU was changed to map the
//                          registers such the VA can be used instead of special
//                          instructions. Also changed the mapping in the IO
//                          page decoding for the peripherals.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A #(
    parameter pKernel_SP_Rst  = 8'h02,      // SP Value after Rst
    parameter pUser_SP_Rst    = 8'hFF,      // SP Value after Rst

    parameter pVec_IRQ        = 16'hFFFE,   // Ext. Maskable Interrupt
    parameter pVec_BRK        = 16'hFFFE,   // BReaK Instruction Trap
    parameter pVec_RST        = 16'hFFFC,   // ReSeT Trap (Highest Priority)
    parameter pVec_NMI        = 16'hFFFA,   // Ex. Non-Maskable Interrupt
    parameter pVec_ABRT       = 16'hFFF8,   // MMU ABoRT Trap (N/S)
    parameter pVec_RSVD1      = 16'hFFF6,   // Reserved for Future Use (BRK)
    parameter pVec_COP        = 16'hFFF4,   // COP Instruction Trap
    parameter pVec_RSVD0      = 16'hFFF2,   // Reserved for Future Use
    parameter pVec_INV        = 16'hFFF0,   // Invalid Instruction Trap
    parameter pVec_RQST7      = 16'hFFEE,   // Int. Maskable Interrupt 7 (COM0)
    parameter pVec_RQST6      = 16'hFFEC,   // Int. Maskable Interrupt 6 (COM1)
    parameter pVec_RQST5      = 16'hFFEA,   // Int. Maskable Interrupt 5 (SPI0)
    parameter pVec_RQST4      = 16'hFFE8,   // Int. Maskable Interrupt 4 (N/U)
    parameter pVec_RQST3      = 16'hFFE6,   // Int. Maskable Interrupt 3 (N/U)
    parameter pVec_RQST2      = 16'hFFE4,   // Int. Maskable Interrupt 2 (N/U)
    parameter pVec_RQST1      = 16'hFFE2,   // Int. Maskable Interrupt 1 (N/U)
    parameter pVec_RQST0      = 16'hFFE0,   // Int. Maskable Interrupt 0 (N/U)
    
    parameter pInt_Hndlr      = 9'h021,     // Microprogram Interrupt Handler

    parameter pVAL            = 3'b000,     // VALid instructions
    parameter pINV            = 3'b001,     // INValid instructions
    parameter pCOP            = 3'b010,     // CO-Processor instruction
    parameter pBRK            = 3'b011,     // BRK instruction
    parameter pFTH            = 3'b100,     // FORTH VM instructions
    parameter pPFX            = 3'b101,     // PreFiX instructions
    parameter pSPC            = 3'b110,     // Special instructions
    parameter pWAI            = 3'b111,     // WAI instruction

    parameter pNOP            = 8'hEA,      // M65C02 Core NOP instruction

    parameter pWS_Delay       = 4'b0011,    // Default Internal Wait State Delay

    parameter pFrequency      = 14745600,   // M65C02 Development Board 
//    parameter pFrequency       = 29491200,  // Chameleon Board

    parameter pUART_LCR       = 8'h40,  // ~BRRE, RTSo, 232 w/o HS, 8N1 
    parameter pUART_IER       = 8'h00,  // No interrupts enabled
    parameter pUART_BRG       = 115200,
    parameter pUART_RTO       = 8'd3,   // RTO asserted on Rx idle 3 char times
    parameter pUART_TF_Depth  = 8'd0,   // Tx FIFO Depth = (2**(0+4))=16
    parameter pUART_RF_Depth  = 8'd0,   // Rx FIFO Depth = (2**(0+4))=16
    parameter pUART_TF_Init   = "Pgms/UART_TF_16.coe",
    parameter pUART_RF_Init   = "Pgms/UART_RF_16.coe",

    parameter pSPI_CR         = 8'h30,  // Rate=1/16, Mode=0, Dir=MSB, Sel=0 
    parameter pSPI_FIFO_Depth = 8'd4,   // Depth = (1 << 4) = 16
    parameter pSPI_FIFO_Init  = "Pgms/SPI_FIFO_Init_16.coe",

    parameter pMON_AddrWidth  = 8'd12,      // Internal ROM Address Width:  4 kB
//    parameter pMON_File       = "Pgms/M65C02_Tst3.txt",
    parameter pMON_File       = "Pgms/Mon65_SBC25.txt",
//    parameter pROM_AddrWidth  = 8'd13,      // Internal RAM Address Width:  8 kB
////    parameter pROM_File       = "Pgms/EhBASIC0.txt",
//    parameter pROM_File       = "Pgms/figFORTH.txt",
    parameter pROM1_AddrWidth = 8'd12,      // Internal RAM Address Width:  4 kB
    parameter pROM1_File      = "Pgms/figFRTHb.txt",
    parameter pROM0_AddrWidth = 8'd12,      // Internal RAM Address Width:  4 kB
    parameter pROM0_File      = "Pgms/figFRTHa.txt",
    parameter pRAM_AddrWidth  = 8'd14,      // Internal RAM Address Width: 16 kB
    parameter pRAM_File       = "Pgms/65C02_ft.txt",
//    parameter pRAM_File       = "Pgms/figFORTH.txt",
                                                                        
    parameter pM65C02_uPgm    = "Pgms/M65C02_uPgm_V4a.coe",     // SEQ   :  2 kB
    parameter pM65C02_IDec    = "Pgms/M65C02_IDecode_ROMa.coe"  // DEC   :  2 kB
)(                                                              // Total : 32 kB
    input   nRst,               // System Reset Input
    input   Clk,                // System Clk Input (Phi2 Input)
//    
//    //  65C02-compatible Interface
//    
//    input   BE,                 // Bus Enable Input
//    
//    output  Phi1O,              // Output Clock Phase 1
//    output  Phi2O,              // Output Clock Phase 2
//
    input   nNMI,               // Non-Maskable Interrupt Input     (Active Low)
    input   nIRQ,               // Maskable Interrupt Request Input (Active Low)
    output  nVP,                // Vector Pull Output Strobe        (Active Low)
//    
    input   nSO,                // Set oVerflow Input               (Active Low)
//    
    output  Sync,               // Instruction Fetch Status Strobe Output
    output  nML,                // Memory Lock Status Output        (Active Low)
//    
    input   RdyIn,              // Bus Ready Input
//    
//    output  RnW,                // Read/nWrite Data Strobe Output
    output  [15:0] AB,          // Physical Address Outputs
//    inout   [ 7:0] DB,          // Data Inputs/Outputs
    output   [ 7:0] DB,          // Data Inputs/Outputs
//    
//    //  M65C02A I/O Extensions
//    
    output  [3:0] nCE,          // Decoded Chip Select Outputs
    output  nRD,                // Read Stobe Output                (Active Low)
    output  nWR,                // Write Strobe Output              (Active Low)
    output  [3:0] XA,           // Extended Physical Address Outputs
    
    //  SPI0 - Serial Peripheral Interface Master
    
    output  [1:0] nSSel,
    output  SCK,
    output  MOSI,
    input   MISO,
    
    //  COM0 - Universal Asynchronous Receiver/Transmitter (UART)

    output  COM0_TxD,
    input   COM0_RxD,
    output  COM0_nRTS,
    input   COM0_nCTS,
    
    output  COM0_DE,
    
    //  COM1 - Universal Asynchronous Receiver/Transmitter (UART)

    output  COM1_TxD,
    input   COM1_RxD,
    output  COM1_nRTS,
    input   COM1_nCTS,
    
    output  COM1_DE
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     Rst;                    // Internal reset (Clk)

reg     NMI;                    // NMI latch/register to hold NMI until serviced
reg     IRQ;                    // External maskable interrupt request input

reg     SO;                     // External Set Overflow (falling edge active)
wire    Set_SO, Clr_SO;

reg     Ext_Wait;               // External Wait Request Input

wire    Int;                    // Interrupt handler interrupt signal to M65C02
wire    [15:0] Vector;          // Interrupt handler interrupt vector to M65C02

wire    IRQ_Msk;                // M65C02 Core interrupt mask
wire    LE_Int;                 // M65C02 Core Int latch enable - hold Int/Vec 
wire    VP;                     // M65C02 Core interrupt vector pull 
wire    Done;                   // M65C02 Core instruction complete/fetch
wire    [2:0] Mode;             // M65C02 Core instruction mode
wire    RMW;                    // M65C02 Core Read-Modify-Write indicator
wire    Wait;                   // M65C02 Core Microcycle Wait Request Input
wire    Rdy;                    // M65C02 Core Microcycle Complete Output
wire    [ 1:0] IO_Op;           // M65C02 Core I/O cycle type
wire    [15:0] VA;              // M65C02 Core Address Output
wor     [ 7:0] CPU_DI;          // M65C02 Core Data Input
wire    [ 7:0] CPU_DO;          // M65C02 Core Data Output

wire    [ 7:0] P;               // M65C02 Core Processor Status Word Register 
wire    Kernel;                 // M65C02 Core Operating Mode

wire    WE, RE;                 // M65C02 Core Decoded IO Operations

wire    INV;                    // M65C02 Core Decoded INValid instructions
wire    COP;                    // M65C02 Core Decoded COP instruction
wire    BRK;                    // M65C02 Core Decoded BRK instruction
wire    PFX;                    // M65C02 Core Decoded PreFiX instructions
wire    PHR;                    // M65C02 Core Decoded PHR instruction
wire    PHW;                    // M65C02 Core Decoded PHW instructions
wire    WAI;                    // M65C02 Core Decoded WAI instruction

wire    IO_Page;                // IO Page Decode
reg     [5:0] IO_Sel;           // IO Selects: Vec, MAP, MMU, SPI0, COM0, COM1
wire    Sel_VEC;                // Vector Table Select; maps to Monitor ROM/RAM
wire    Sel_MAP, Sel_MMU;       // Mapping RAM and MMU Control/Status Registers
wire    Sel_SPI0;               // SPI Master Select
wire    Sel_COM0, Sel_COM1;     // COM0, COM1 UART Selects

wire    [ 7:0] MMU_DO;          // MMU Data Output - Used to read MMU registers
wire    [19:0] PA;              // MMU Physical Address Output              
wire    [15:1] CE;              // MMU Chip Enable: Ext=CE[7:1]; Int=CE[15:8]
wire    Int_WS;                 // MMU Internal Wait State Request

reg     [3:0] WSGen;            // Internal Wait State Generator Counter
wire    Int_Wait;               // Internal Wait State Request Signal

wire    [7:0] MON_DO;           // Output Data Bus of MON
wire    [7:0] ROM1_DO;          // Output Data Bus of ROM 1
wire    [7:0] ROM0_DO;          // Output Data Bus of ROM 0
wire    [7:0] RAM_DO;           // Output Data Bus of RAM

wire    [7:0] SPI0_DO;          // SPI0 Output Data Bus
wire    SPI0_IRQ;               // SPI0 Interrupt Request
wire    [1:0] SSel;             // SPI0 Slave Selects

wire    [7:0] COM0_DO;          // COM0 Output Data Bus
wire    COM0_RTS, COM0_CTS;     // COM0 Modem Control Signals
wire    COM0_IRQ;               // COM0 Interrupt Request

wire    [7:0] COM1_DO;          // COM1 Output Data Bus
wire    COM1_RTS, COM1_CTS;     // COM1 Modem Control Signals
wire    COM1_IRQ;               // COM1 Interrupt Request

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(posedge Clk)
begin
    Rst <= #1 ~nRst;
end

always @(negedge Clk)
begin
    if(Rst)
        Ext_Wait <= #1 1;
    else
        Ext_Wait <= #1 ~RdyIn;        
end

//  Process nSO

fedet   FE1 (                   //  Detect falling edge of external nSO pin
            .rst(Rst), 
            .clk(Clk),
            .din(nSO),
            .pls(Set_SO)        // Generate pulse on falling edge of nSO
        );
        
always @(posedge Clk)           // Set and hold SO signal until core ACKs.
begin
    if(Rst | Clr_SO)            // Clear SO on reset or when core ACKs
        SO <= #1 Set_SO;
    else if(Set_SO)             // Set SO on falling edge of nSO pin
        SO <= #1 1;
end

//
//  Process External NMI and maskable IRQ Interrupts
//

always @(posedge Clk)           // Synchronize external inputs to core's clock
begin
    NMI <= #1 ~nNMI;
    IRQ <= #1 ~nIRQ;
end

//  Instantiate M65C02 Vectored Interrupt Handler

M65C02_IntHndlrV2   IntHndlr (
                        .Rst(Rst),              // System Reset 
                        .Clk(Clk),              // System Clock
                        
                        .Rdy(Rdy),
                        
                        .ABRT(ABRT),            // MMU Trap (Not Implemented)
                        .INV(INV),              // INValid instruction Trap
                        .NMI(NMI),              // NMI input 
                        .IRQ(IRQ),              // IRQ input
                        .RQST({COM0_IRQ, COM1_IRQ, SPI0_IRQ, 5'b0}),
                        .BRK(BRK),              // BRK Instruction
                        .COP(COP),              // COP Instruction

                        .IRQ_Msk(IRQ_Msk),      // Interrupt Mask (PSW.I)
                        .LE_Int(LE_Int),        // Latch Int/Vector until VP
                        .VP(VP),                // Interrupt Service Vector Pull
                        
                        .Int(Int),              // M65C02 Core Interrupt Signal 
                        .Vector(Vector)         // M65C02 Core Interrupt Vector 
                    );

//  Instantiate M65C02 Core

assign Wait = Int_Wait | Ext_Wait;          // Combine int and ext Wait signals

M65C02_CoreV2   #(
                    .pStkPtr_Rst(pKernel_SP_Rst),   // M65C02A Reset Value for S
                    .pInt_Hndlr(pInt_Hndlr),        // M65C02A Int. Microroutine
                    .pM65C02_uPgm(pM65C02_uPgm),    // M65C02A Microprogram COE
                    .pM65C02_IDec(pM65C02_IDec)     // M65C02A Inst. Decoder COE
                ) uP (
                    .Rst(Rst),              // System Reset
                    .Clk(Clk),              // System Clock
                    
                    .IRQ_Msk(IRQ_Msk),      // M65C02A Core Interrupt Mask
                    .xIRQ(IRQ),             // M65C02A Core Extrn Interrupt Flag
                    .Int(Int),              // M65C02A Core Interrupt Request
                    .Vector(Vector),        // M65C02A Core Interrupt Vector
                    .LE_Int(LE_Int),        // M65C02A Core Latch Enable Int/Vec
                    .VP(VP),                // M65C02A Core Interrupt Vec. Pull

                    .SO(SO),                // M65C02A Core Set oVerflow Input
                    .Clr_SO(Clr_SO),        // M65C02A Core Clr SO input

                    .Done(Done),            // M65C02A Core Instruction Complete
                    .SC(),                  // M65C02A Core Single Cycle Flag
                    .Mode(Mode),            // M65C02A Core Instruction Mode
                    .RMW(RMW),              // M65C02A Core Read/Modify/Write
                    
                    .Wait(Wait),            // M65C02A Core Microcycle Wait Rqst

                    .Rdy(Rdy),              // M65C02A Core Microcycle Ready Out
                    
                    .IO_Op(IO_Op),          // M65C02A Core Bus Cycle Type
                    .AO(VA),                // M65C02A Core Address Output Bus
                    .DI(CPU_DI),            // M65C02A Core Data Input Bus
                    .DO(CPU_DO),            // M65C02A Core Data Output Bus
                    
                    .X(),                   // M65C02A Core X Index Register
                    .Y(),                   // M65C02A Core Y Index Register
                    .A(),                   // M65C02A Core Accumulator Register
                    
                    .P(P),                  // M65C02A Core P Register
                    
                    .OP1(),                 // M65C02A Core Operand Register 1
                    .OP2(),                 // M65C02A Core Operand Register 2
                    .IR(),                  // M65C02A Core Instruction Register
                    
                    .IND(),                 // M65C02A Core Address Override
                    .SIZ(),                 // M65C02A Core Size Override
                    .OAX(),                 // M65C02A Core Op(A) <=> Op(X)
                    .OAY(),                 // M65C02A Core Op(A) <=> Op(Y)
                    .OSY()                  // M65C02A Core Op(S) <=> Op(Y)              
                );

//  Decode Core Control Signals

assign WE  = IO_Op[0];              // For speed, converted IO_OP to one-hot
assign RE  = IO_Op[1];

assign INV = (Mode == pINV);        // Invalid instruction decoded
assign COP = (Mode == pCOP);        // Co-Processor instruction decoded
assign BRK = (Mode == pBRK);        // BRK instruction decoded
assign FTH = (Mode == pFTH);        // FORTH VM instruction decoded
assign PFX = (Mode == pPFX);        // Prefix instructions decoded
assign SPC = (Mode == pSPC);        // Special instructions decoded
assign WAI = (Mode == pWAI);        // WAit for Interrupt instruction decoded

//  MMU - Maps Virtual Addresses to Physical Addresses using 16 4kB pages
//
//  CE[15]  - MON   : Boot ROM - 0xF000:FFFF; ( 4 kB -  2 BRAMs)
//  CE[14]  - ROM1  : User ROM - 0xE000:EFFF; ( 4 kB -  2 BRAMs)
//  CE[13]  - ROM0  : User ROM - 0xD000:DFFF; ( 4 kB -  2 BRAMs)
//  CE[ 8]  - RAM   : User RAM - 0x0000:3FFF; (16 kB -  8 BRAMs)
//                             Total RAM/ROM: (28 kB - 14 BRAMs
//
//  Internal ROM space includes the I/O page. The I/O page is 0xFF00:FFFF;
//  The nWP input disable writes in the range 0xF000:FEFF. Writes in the I/O
//  page are not inhibited by the nWP input. The result is that the top 32
//  location of the ROM BRAMs contains the interrupt/trap vector table, and it
//  may be modified by the programmer.
//
//  The following is the I/O page decode map:
//
//  Sel_VEC     - VecTbl    : BRAM(MON) - 0xFFE0:FFFF   (32 Bytes)
//  Sel_MAP     - MAP       : LUTs/FFs  - 0xFFA0:FFDF   (64 Bytes)
//  Sel_MMU     - MMU  (Hi) : LUTs/FFs  - 0xFF98:FF9F   ( 8 Bytes)
//  Sel_SPI0    - SPI0 (Lo) : LUTs/FFs  - 0xFF90:FF97   ( 8 Bytes)
//  Sel_COM1    - COM1 (Hi) : LUTs/FFs  - 0xFF88:FF8F   ( 8 Bytes)
//  Sel_COM0    - COM0 (Lo) : LUTs/FFs  - 0xFF80:FF87   ( 8 Bytes)
//

//  Implement I/O Decode for Internal Functions

assign Kernel   = P[5];      // IO Page only mapped to Kernel mode 0xFF00:FFFF
assign IO_Page  = Kernel & (&VA[15:8]);                         // 0xFF00:FFFF

always @(*)
begin
    case({IO_Page, VA[7:3]})
        6'b110000 : IO_Sel <= 6'b0_0001_0;  // COM0 : 0xFF80:FF87
        6'b110001 : IO_Sel <= 6'b0_0010_0;  // COM1 : 0xFF88:FF8F
        6'b110010 : IO_Sel <= 6'b0_0100_0;  // SPI0 : 0xFF90:FF97
        6'b110011 : IO_Sel <= 6'b0_1000_0;  // MMU  : 0xFF98:FF9F
        6'b110100 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFA0:FFA7 Usr[ 3: 0]
        6'b110101 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFA8:FFAF Usr[ 7: 4]
        6'b110110 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFB0:FFB7 Usr[11: 8]
        6'b110111 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFB8:FFBF Usr[15:12]
        6'b111000 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFC0:FFC7 Krn[ 3: 0]
        6'b111001 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFC8:FFCF Krn[ 7: 4]
        6'b111010 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFD0:FFD7 Krn[11: 8]
        6'b111011 : IO_Sel <= 6'b1_0000_0;  // MAP  : 0xFFD8:FFDF Krn[15:12]
        6'b111100 : IO_Sel <= 6'b0_0000_1;  // VEC  : 0xFFE0:FFE7
        6'b111101 : IO_Sel <= 6'b0_0000_1;  // VEC  : 0xFFE8:FFEF
        6'b111110 : IO_Sel <= 6'b0_0000_1;  // VEC  : 0xFFF0:FFF7
        6'b111111 : IO_Sel <= 6'b0_0000_1;  // VEC  : 0xFFF8:FFFF
        default   : IO_Sel <= 6'b0_0000_0;
    endcase
end

assign {Sel_MAP, Sel_MMU, Sel_SPI0, Sel_COM1, Sel_COM0, Sel_VEC} = IO_Sel;

//  Instantiate Memory Management Module

M65C02_MMU  MAP (
                .Rst(Rst),
                .Clk(Clk),
                
                .Rdy(Rdy),

                .Mode(Kernel),
                .Sync(Done),
                .IO_Op(IO_Op),

                .VA(VA),                // Virtual Address and Register Select
                
                .Sel_MAP(Sel_MAP),
                .Sel_MMU(Sel_MMU),
                .WE(WE),
                .RE(RE),
                .MMU_DI(CPU_DO),
                .MMU_DO(MMU_DO),

                .PA(PA),
                .CE(CE),
                .Int_WS(Int_WS),

                .ABRT(ABRT)
            );

assign CPU_DI = ((Sel_MAP | Sel_MMU) ? MMU_DO : 0);

//  Implement Internal Wait State Generator
//      Applies to off-chip memory cycles. MMU WS field provides a minimum delay
//      to allow external logic to decode the address and determine if addi-
//      tional wait states are required.

always @(negedge Clk)
begin
    if(Rst)
        WSGen <= #1 0;
    else if(~|WSGen)
        WSGen <= #1 ((Int_WS) ? pWS_Delay : (WSGen - 1));
end

assign Int_Wait = Int_WS & |WSGen;

//  Generate Selects for Internal BRAMs

assign Sel_MON  = CE[15] & ~IO_Page | Sel_VEC;   // Create IO_Page hole
assign Sel_ROM1 = CE[14];
assign Sel_ROM0 = CE[13];
assign Sel_RAM  = CE[ 8];

//  MON - 0xF000-FEFF (Monitor), 0xFFE0-FFFF (Vector Table) ( 4 kB)

BRAM_SP_mn  #(
                .pBRAM_AddrWidth(pMON_AddrWidth),
                .pBRAM_SP_mn_Init(pMON_File)
            ) MON (
                .Clk(~Clk), 
                .WP(1'b0),
                
                .CE(Sel_MON), 

                .WE(WE), 
                .PA(PA[(pMON_AddrWidth - 1):0]), 
                .DI(CPU_DO), 
                .DO(MON_DO)
            );
            
assign CPU_DI = ((Sel_MON) ? MON_DO : 0);

//  ROM 1 - 0xE000-EFFF ( 4 kB)

BRAM_SP_mn  #(
                .pBRAM_AddrWidth(pROM1_AddrWidth),
                .pBRAM_SP_mn_Init(pROM1_File)
            ) ROM1 (
                .Clk(~Clk), 
                .WP(1'b0),
                
                .CE(Sel_ROM1), 

                .WE(WE), 
                .PA(PA[(pROM1_AddrWidth - 1):0]), 
                .DI(CPU_DO), 
                .DO(ROM1_DO)
            );
            
assign CPU_DI = ((Sel_ROM1) ? ROM1_DO : 0);

//  ROM 0 - 0xD000-DFFF ( 4 kB)

BRAM_SP_mn  #(
                .pBRAM_AddrWidth(pROM0_AddrWidth),
                .pBRAM_SP_mn_Init(pROM0_File)
            ) ROM0 (
                .Clk(~Clk), 
                .WP(1'b0),
                
                .CE(Sel_ROM0), 

                .WE(WE), 
                .PA(PA[(pROM0_AddrWidth - 1):0]), 
                .DI(CPU_DO), 
                .DO(ROM0_DO)
            );
            
assign CPU_DI = ((Sel_ROM0) ? ROM0_DO : 0);

//  RAM  - 0x0000-3FFF (16 kB)

BRAM_SP_mn  #(
                .pBRAM_AddrWidth(pRAM_AddrWidth),
                .pBRAM_SP_mn_Init(pRAM_File)
            ) RAM (
                .Clk(~Clk), 
                .WP(1'b0),
                
                .CE(Sel_RAM), 

                .WE(WE), 
                .PA(PA[(pRAM_AddrWidth - 1):0]), 
                .DI(CPU_DO), 
                .DO(RAM_DO)
            );
            
assign CPU_DI = ((Sel_RAM) ? RAM_DO : 0);

////////////////////////////////////////////////////////////////////////////////
//
//  Internal I/O devices: 1 SPI Master (Buffered), 2 UARTs (Buffered)
//

//  SPI0 - SPI Master Peripheral (0xFF90-FF97)
        
M65C02_SPIxIF   #(
                    .pDefault_CR(pSPI_CR),
                    .pSPI_FIFO_Depth(pSPI_FIFO_Depth),
                    .pSPI_FIFO_Init(pSPI_FIFO_Init)
                ) SPI0 (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .IRQ(SPI0_IRQ), 

                    .Sel(Sel_SPI0), 
                    .RE(RE), 
                    .WE(WE), 
                    .Reg(PA[1:0]), 
                    .DI(CPU_DO), 
                    .DO(SPI0_DO), 
                    
                    .SSel(SSel), 
                    .SCK(SCK), 
                    .MOSI(MOSI), 
                    .MISO(MISO)
                );

assign CPU_DI = ((Sel_SPI0) ? SPI0_DO : 0);

assign nSSel = ~SSel;      

//  COM1 - UART Peripheral (0xFF88-FF8F)

UART    #(
            .pFrequency(pFrequency),
            .pDefault_LCR(pUART_LCR),
            .pDefault_IER(pUART_IER),
            .pBaudrate(pUART_BRG),
            .pRTOChrDlyCnt(pUART_RTO),
            .pTF_Depth(pUART_TF_Depth),
            .pRF_Depth(pUART_RF_Depth),
            .pTF_Init(pUART_TF_Init),
            .pRF_Init(pUART_RF_Init)
        ) COM1 (
            .Rst(Rst), 
            .Clk(Clk),
            
            .IRQ(COM1_IRQ),
            
            .Sel(Sel_COM1), 
            .Reg(PA[1:0]), 
            .RE(RE), 
            .WE(WE), 
            .DI(CPU_DO), 
            .DO(COM1_DO),
            
            .TxD(COM1_TxD), 
            .RxD(COM1_RxD), 
            .xRTS(COM1_RTS), 
            .xCTS(COM1_CTS),
            .xDE(COM1_DE), 

            .Mode(),

            .TxIdle(), 
            .RxIdle()
        );
        
assign CPU_DI = ((Sel_COM1) ? COM1_DO  : 0);       

assign COM1_nRTS = ~COM1_RTS;
assign COM1_CTS  = ~COM1_nCTS;

//  COM0 - UART Peripheral (0xFF80-FF87)

UART    #(
            .pFrequency(pFrequency),
            .pDefault_LCR(pUART_LCR),
            .pDefault_IER(pUART_IER),
            .pBaudrate(pUART_BRG),
            .pRTOChrDlyCnt(pUART_RTO),
            .pTF_Depth(pUART_TF_Depth),
            .pRF_Depth(pUART_RF_Depth),
            .pTF_Init(pUART_TF_Init),
            .pRF_Init(pUART_RF_Init)
        ) COM0 (
            .Rst(Rst), 
            .Clk(Clk),
            
            .IRQ(COM0_IRQ),
            
            .Sel(Sel_COM0), 
            .Reg(PA[1:0]), 
            .RE(RE), 
            .WE(WE), 
            .DI(CPU_DO), 
            .DO(COM0_DO),
            
            .TxD(COM0_TxD), 
            .RxD(COM0_RxD), 
            .xRTS(COM0_RTS), 
            .xCTS(COM0_CTS),
            .xDE(COM0_DE),
            
            .Mode(),

            .TxIdle(), 
            .RxIdle()
        );
        
assign CPU_DI   = ((Sel_COM0) ? COM0_DO  : 0);       

assign COM0_nRTS = ~COM0_RTS;
assign COM0_CTS  = ~COM0_nCTS;

////////////////////////////////////////////////////////////////////////////////
//
//  Define External Connections
//

assign Sync =  Done;
assign nML  = ~RMW | Done;
assign nVP  = ~VP;

//assign nCE  = ~{CE[11], CE[15], CE[13], CE[8]};
assign nCE  = ~{CE[4], CE[3], CE[2], CE[1]};
assign nWR  = ~WE;
assign nRD  = ~RE;
assign XA   =  PA[19:16];
assign AB   =  PA[15: 0];
assign DB   = ((WE) ? CPU_DO : CPU_DI);

endmodule
