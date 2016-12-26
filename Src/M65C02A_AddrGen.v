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
// Module Name:     M65C02A_AddrGen.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This file provides the M65C02A_Core module's address generator function. 
//
// Dependencies:    none 
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release.
//
//  1.10    15F21   MAM     Incorporated support for two system stack pointers:
//                          (1) Kernel Mode Stack Pointer (Sk), and (2) User
//                          Mode Stack Pointer (Su).
//
//  1.20    15G06   MAM     Modified to better support the OSX prefix instruc-
//                          tion. Previous implementation did not allow the OSX
//                          prefix to be used for reverting to stack-relative
//                          addressing of column 3 instruction from the default
//                          BP-relative (X-relative) addressing mode.
//
//  1.30    15G18   MAM     Changed the Abs address signal multiplexer to incor-
//                          porate the Ld_PC signal. Corrects a problem with the
//                          conditionally branch Rockwell instructions.
//
//  1.31    15G20   MAM     Reversed change to Sel_SP made in r1.20.
//
//  1.40    15K04   MAM     Modified the % 256 address logic. The logic checks
//                          if the upper 7 bits of the AL field is 0, and if so,
//                          performs % 256 address calculations by using the
//                          AL[15:8] as the upper byte of the virtual address.
//                          If AL[15:9] <> 0, then the requested % 256 address
//                          calculation is not performed, and the virtual ad-
//                          dress is the address sum provided.
//
//  1.5     16C31   MAM     Modified the Ci input to the address generator to
//                          be either NA_Op[0] or NA_Op[3] based on the OSX flag
//                          setting. If OSX is true, the swapping of X & S makes
//                          S the index register for any addressing mode indexed
//                          by X. By changing the input to Ci based on OSX, S is
//                          incremented by 1, with the result that {OP2, OP1} is
//                          a zero-based offset into the stack area. This is
//                          consistent with the base pointer and stack pointer
//                          relative addressing modes supported by the core.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_AddrGen #(
    parameter pDef_Page = 1,        // Default System Stack in Page 1
    parameter pSk_Rst   = 2,        // Initialize Sk to support saving PC and P
    parameter pSu_Rst   = 255       // Initialize Su to top of user stack
)(
    input   Rst,                    // System Reset
    input   Clk,                    // System Clock
    
    input   Kernel,                 // Core Operating Mode: 1 - Kernel; 0 - User

    input   [15:0] Vector,          // Interrupt/Trap Vector
    
    input   [ 8:0] NA_Op,           // Address Generator Operation Select
    
    input   IND,                    // Override: Address Indirection
    input   SIZ,                    // Override: Operation Size
    input   OAX,                    // Override: Swap A and X
    input   OAY,                    // Override: Swap A and Y
    input   OSX,                    // Override: Swap S and X, and X is stk ptr
    
    input   SPR,                    // Enable SP-Relative Addressing Mode
    
    input   Mod,                    // % 256 Control - wrap address to page
    input   Prv,                    // Previous Address Control
    
    input   CC,                     // Conditional Branch Input Flag
    input   BRV3,                   // Interrupt or Next Instruction Select
    input   Int,                    // Unmasked Interrupt Request Input

    input   Rdy,                    // Ready Input
    input   Valid,                  // Data Valid
    
    input   [15:0] M,               // Memory Operand Register Input
    
    input   [15:0] IP,              // FORTH VM Instruction Pointer
    
    input   [15:0] X,               // X Index Register Input
    input   [15:0] Y,               // Y Index Register Input
    input   [15:0] A,               // A Index Register Input

    output  reg [15:0] AO,          // Address Output

    input   SelS,                   // Stack Pointer Select
    input   [ 1:0] Stk_Op,          // Stack Pointer Operation Select
    input   [15:0] SDI,             // Stack Pointer Data Input
    output  [15:0] SDO,             // Stack Pointer Data Output

    output  reg [15:0] MAR,         // Program Counter
    output  reg [15:0] PC           // Program Counter
);

////////////////////////////////////////////////////////////////////////////////
//
//  Module Declarations
//

wire    [15:0] S;                   // Stack Pointer Register

wire    [15:0] Abs;                 // 16-bit Operand Input

wor     [15:0] AL, AR;              // Wired-OR busses for address operands
wire    [15:0] NA;                  // Next Address (w/o application of % 256)

wire    CE_MAR;                     // Memory Address Register Clock Enable
wire    CE_PC;                      // Program Counter Clock Enable

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Next Address Generator

//  Decode Next Address Operation Field

//          L PSIM XYA C
//          d CtPA   b i
//          P  k R   s  
// Vec:  9'b1_0000_001_0;   // NA <=       {OP2,OP1} + 0; PC  <= NA
// Jmp:  9'b1_0000_001_0;   // NA <=       {OP2,OP1} + 0; PC  <= NA
// JmpX: 9'b1_0000_101_0;   // NA <= X   + {OP2,OP1} + 0; PC  <= NA
// JmpY: 9'b1_0000_011_0;   // NA <= Y   + {OP2,OP1} + 0; PC  <= NA
// Rtn:  9'b1_0000_001_1;   // NA <=       {OP2,OP1} + 1; PC  <= NA
// PC:   9'b1_1000_000_0;   // NA <= PC              + 0; PC  <= NA
// Inc:  9'b1_1000_000_1;   // NA <= PC              + 1; PC  <= NA
// Bra:  9'b1_1000_001_0;   // NA <= PC  + {OP2,OP1} + 1; PC  <= NA
// Rel:  9'b0_1000_001_0;   // NA <= PC  + {OP2,OP1} + 1; MAR <= PC + Rel16
// Psh:  9'b0_0100_000_0;   // NA <= S               + 0;
// Pop:  9'b0_0100_000_1;   // NA <= S               + 1;
// SPM:  9'b0_0100_001_1;   // NA <= S   + {OP2,OP1} + 1;
// BPM:  9'b0_0000_101_1;   // NA <= BP  + {OP2,OP1} + 1;
// IPN:  9'b0_0010_000_0;   // NA <= IP              + 0;
// ZPM:  9'b0_0000_001_0;   // NA <=       {OP2,OP1} + 0;
// ZPX:  9'b0_0000_101_0;   // NA <= X   + {OP2,OP1} + 0;
// ZPY:  9'b0_0000_011_0;   // NA <= Y   + {OP2,OP1} + 0;
// LDA:  9'b0_0000_001_0;   // NA <=       {OP2,OP1} + 0;
// LDAX: 9'b0_0000_101_0;   // NA <= X   + {OP2,OP1} + 0;
// LDAY: 9'b0_0000_011_0;   // NA <= Y   + {OP2,OP1} + 0;
// SRC:  9'b0_0000_101_0;   // NA <= X               + 0;
// DST:  9'b0_0000_011_0;   // NA <= Y               + 0;
// MAR:  9'b0_0001_000_0;   // NA <= MAR             + 0;                         
// Nxt:  9'b0_0001_000_1;   // NA <= MAR             + 1;
//
//  Note:   {OP2, OP1} is equivalent to {0, OP1} after a normal load of OP1. In
//          this case, OP2 is automatically loaded with a 0 when OP1 is loaded
//          with the operand from memory. If OP1 is loaded with a SGN operation
//          code, then OP2 is loaded with the sign extension of the operand be-
//          ing loaded into OP1. These two load operations are what allow  
//          {OP2, OP1} to be used as a zero page pointer, or an absolute memory
//          pointer, or a signed 8-bit/16-bit relative displacement.

assign Ld_PC   = NA_Op[8];
//
assign Sel_PC  = NA_Op[7];          // JMP/JSR/Bcc/BRL/RTS/RTI
assign Sel_SP  = ((OSX) ? NA_Op[3] | SPR : NA_Op[6]); // if OSX, X <=> S
assign Sel_IP  = NA_Op[5];          // IP-relative w/ autoincrement: ip,I++
assign Sel_MAR = NA_Op[4];          // Sequential memory access/PHR
//
assign Sel_X   = ((OSX) ? NA_Op[6] : NA_Op[3]); // if OSX, X <=> S
assign Sel_Y   = NA_Op[2];
assign Sel_Abs = NA_Op[1];
//
assign Ci      = NA_Op[0];
//
assign Ld_MAR  = ~(NA_Op[6] & ~Sel_Abs);

//  Generate 16-bit Right Address Operand
//      For stk relative: OP2 needs to be 0
//      For Bcc relative: OP2 needs to be sign extension of OP1

assign Abs = ((CC | ~Ld_PC) ? M : 0);

//  Generate Left Address Operand

assign AL = ((Sel_PC ) ? PC  : 0);
assign AL = ((Sel_SP ) ? S   : 0);
assign AL = ((Sel_IP ) ? IP  : 0);
assign AL = ((Sel_X  ) ? ((OAX) ? A : X) : 0);  // if OAX then swap X & A
assign AL = ((Sel_Y  ) ? ((OAY) ? A : Y) : 0);  // if OAY then swap Y & A
assign AL = ((Sel_MAR) ? MAR : 0);

//  Generate Right Address Operand

assign AR = ((Sel_Abs) ? Abs : 0);

//  Compute Next Address

assign NA = (AL + ((Prv) ? ~AR : AR) + Ci);

//  Generate Address Output - wrap NA on page boundary when Mod is asserted
//      When Mod is asserted, the page is determined by AL[15:8]
//
//      Mod is ignored if AL[15:9] <> 0. This allows linear address calculations
//      when index registers not in page 0 and page 1, otherwise address
//      performed modulo 256.

always @(*)
begin
    if(Rst)
        AO <= Vector;
    else
        AO <= ((Mod & ~|AL[15:9]) ? {AL[15:8], NA[7:0]} : NA);
end

//  Memory Address Register

assign CE_MAR = Rdy & Ld_MAR;

always @(posedge Clk)
begin
    if(CE_MAR)
        MAR <= #1 AO;
end

//  Program Counter

assign CE_PC = Rdy & ((BRV3) ? (Ld_PC & ~Int) : Ld_PC);

always @(posedge Clk)
begin
    if(CE_PC)
        PC <= #1 AO;
end

//  Instantiate Stack Pointer
//
//      Mod is ignored if S[15:9] <> 0. This allows stacks outside of page 0/1
//      to be greater than 256 bytes, otherwise stacks are limited to 256 bytes

M65C02A_StkPtrV2    #(
                        .pDef_Page(pDef_Page),
                        .pSk_Rst(pSk_Rst),
                        .pSu_Rst(pSu_Rst)
                    ) SysStk (
                        .Rst(Rst), 
                        .Clk(Clk),
                        
                        .Mode(Kernel),
                        
                        .Ind(IND),
                        .Size(SIZ),
                        
                        .Mod(Mod & ~|S[15:9]),
                        
                        .Rdy(Rdy), 
                        .Valid(Valid),
                        
                        .Sel(SelS), 
                        .Stk_Op(((OSX) ? 2'b0 : Stk_Op)), 
                        .SDI(SDI),
                        .SDO(SDO),
                        
                        .S(S)
                    );
                
endmodule
