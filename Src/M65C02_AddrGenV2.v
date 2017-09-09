////////////////////////////////////////////////////////////////////////////////
//
//  Address Generator module for M65C02A soft-core microcomputer project.
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
// Create Date:     19:59:21 09/18/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_AddrGenV2.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This file provides the M65C02_Core module's address generator function. This
//  module is taken from the address generator originally included in the
//  M65C02_Core module. The only difference is the addition of an explicit sig-
//  nal which generates relative offset for conditional branches, Rel.
//
// Dependencies:    none 
//
// Revision: 
//
//  1.00    13I18   MAM     Initial creation of the file from M65C02_AddrGen.
//
//  1.10    13J08   MAM     Modifying the next address generation logic to fold
//                          NA_Op, PC_Op, Stk_Op, and ZP fields into a single
//                          one-hot control field that can replace these fields
//                          in the uPgm microcode.
//
//  2.00    14F28   MAM     Added comments to reflect changes made during inte-
//                          gration and verification of the module: (1) changed
//                          the one-hot NA_Op control field from 13 to 11 bits;
//                          (2) added explicit external inputs for address gene-
//                          mod 256 and page control; (3) added explicit control
//                          NA_Op[10] to control the loading of the PC; (4) add-
//                          ed explicit control NA_Op[9] to control the future
//                          stack relative instructions; (4) reduced the size of
//                          the left operand multiplexer; (5) made NA calcula-
//                          tion as a single 16-bit operation rather than split
//                          into two separate 8-bit operations with a control on
//                          the carry propagation between the halves; (6) imple-
//                          mented the % 256 function using a multiplexer, and
//                          applied to both page 0 (data) and page 1 (stack);
//                          (7) changed MAR to only register NA on Rdy - removed
//                          need to load Vector, etc. on Rst; (8) removed Rst 
//                          from PC which allows the modification to the Reset
//                          behavior of the core to push the PC and PSW to the
//                          stack in locations Stk[2:0] <= {PCH, PCL, PSW}; and
//                          (9) adjusted the stack pointer module inputs to re-
//                          flect changes to NA_Op to support stack relative
//                          addressing.
//
//  2.01    14H09   MAM     Added comment line describing how (sp,S),Y address-
//                          ing is implemented, added the {0, OP1} to AR to im-
//                          plement stack relative addressing, and changed name
//                          of StkRel signal to Stk_Rel.
//
//  2.02    14K15   MAM     Removed commented out code.
//
//  2.10    14K29   MAM     Modified the logic for % 256 operations. Now the
//                          operation is performed using the upper 8 bits of the
//                          left operand input. This sets up the module to sup-
//                          port extending all of the registers to 16 bits, and
//                          placing the data/page and stack pages anywhere in 
//                          the address space.
//
//  2.20    14K30   MAM     Modified the stack pointer input/output to use dedi-
//                          cated input/output busses. This allows the implemen-
//                          tation of two stack pointers: S and Y. The direct
//                          input reduces the number of multiplexers needed to
//                          support the register override prefix instructions.
//
//  2.20    14L06   MAM     Completed modifications to incorporate OAX, OAY, 
//                          and OSY prefix instructions: OAX switches A and X;
//                          OAY switches A and Y; and OSY switches Y with S for
//                          stack operations, and Y and S in the Y-specific
//                          instructions. The Accumulator replaces X as an index
//                          register when OAX asserted, ans replaces Y as an
//                          index register when OAY asserted. Y replaces S when
//                          OSY is asserted. With OSY asserted, stack pointer
//                          operations are performed by the stack pointer logic
//                          of the ALU's Y register, otherwise, the stack poin-
//                          ter logic of S in this module performs the opera-
//                          tions required.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_AddrGenV2 #(
    parameter pStkPtr_Rst = 2       // Init SP to push PCH, PCL, P on reset
)(
    input   Rst,                    // System Reset
    input   Clk,                    // System Clock

    input   [15:0] Vector,          // Interrupt/Trap Vector
    
    input   [10:0] NA_Op,           // Address Generator Operation Select

    input   OAX,                    // Override: Swap A and X
    input   OAY,                    // Override: Swap A and Y
    input   OSY,                    // Override: Swap S and Y, and S and A
    
    input   Mod256,                 // Mod 256 Control - wrap address to page
    
    input   CC,                     // Conditional Branch Input Flag
    input   BRV3,                   // Interrupt or Next Instruction Select
    input   Int,                    // Unmasked Interrupt Request Input

    input   Rdy,                    // Ready Input
    input   Valid,                  // Data Valid
    
    input   [7:0] OP1,              // Operand Register 1 Input
    input   [7:0] OP2,              // Operand Register 2 Input
    
    input   [7:0] X,                // X Index Register Input
    input   [7:0] Y,                // Y Index Register Input
    input   [7:0] A,                // A Index Register Input

    output  reg [15:0] AO,          // Address Output

    input   SelS,                   // Stack Pointer Select
    input   [1:0] Stk_Op,           // Stack Pointer Operation Select
    input   [7:0] SDI,              // Stack Pointer Data Input
    output  [7:0] SDO,              // Stack Pointer Data Output

    output  reg [15:0] MAR,         // Program Counter
    output  reg [15:0] PC           // Program Counter
);

////////////////////////////////////////////////////////////////////////////////
//
//  Module Declarations
//

wire    [ 7:0] S;               // Stack Pointer Register

wor     [15:0] AL, AR;          // Wired-OR busses for address operands
wire    [15:0] NA;              // Next Address (w/o application of % 256)

wire    CE_MAR;                 // Memory Address Register Clock Enable

wire    [15:0] Rel;             // Branch Address Sign-Extended Offset
wire    CE_PC;                  // Program Counter Clock Enable

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Next Address Generator

//  Decode Next Address Operation Field

//           LO PSZAM XYR C
//           df CtPbA   e i
//           Pf  k sR   l  
// Vec:  11'b10_00010_000_0; // NA <= {OP2,OP1}           + 0; PC <= NA
// Jmp:  11'b10_00010_000_0; // NA <= {OP2,OP1}           + 0; PC <= NA
// JmpX: 11'b10_00010_100_0; // NA <= {OP2,OP1} + {0,  X} + 0; PC <= NA
// JmpY: 11'b10_00010_010_0; // NA <= {OP2,OP1} + {0,  Y} + 0; PC <= NA
// Rtn:  11'b10_00010_000_1; // NA <= {OP2,OP1}           + 1; PC <= NA
// PC:   11'b10_10000_000_0; // NA <= PC                  + 0; PC <= NA
// Inc:  11'b10_10000_000_1; // NA <= PC                  + 1; PC <= NA
// Bra:  11'b10_10000_001_0; // NA <= PC        + Rel     + 1; PC <= NA
// Rel:  11'b00_10000_001_0; // NA <= PC        + Rel     + 1;
// Psh:  11'b00_01000_000_0; // NA <= {  1, SP}           + 0;
// Pop:  11'b00_01000_000_1; // NA <= {  1, SP}           + 1;
// SPN:  11'b01_01000_000_1; // NA <= {  1, SP} + {0,OP1} + 1;
// DPN:  11'b00_00100_000_0; // NA <= {  0,OP1}           + 0;
// DPX:  11'b00_00100_100_0; // NA <= {  0,OP1} + {0,  X} + 0;
// DPY:  11'b00_00100_010_0; // NA <= {  0,OP1} + {0,  Y} + 0;
// LDA:  11'b00_00010_000_0; // NA <= {OP2,OP1}           + 0;
// LDAX: 11'b00_00010_100_0; // NA <= {OP2,OP1} + {0,  X} + 0;
// LDAY: 11'b00_00010_010_0; // NA <= {OP2,OP1} + {0,  Y} + 0;
// MAR:  11'b00_00001_000_0; // NA <= MAR                 + 0;                         
// Nxt:  11'b00_00001_000_1; // NA <= MAR                 + 1;

assign Ld_PC   =  NA_Op[10];
assign Stk_Rel =  NA_Op[ 9];
//
assign Sel_PC  =  NA_Op[8];
assign Sel_SP  =  NA_Op[7] & ~OSY;  // When ~OSY use S for stack operations
assign Sel_SPY =  NA_Op[7] &  OSY;  // When  OSY use Y for stack operations
assign Sel_ZP  =  NA_Op[6];
assign Sel_Abs =  NA_Op[5];
assign Sel_MAR =  NA_Op[4];
//
assign Sel_X   =  NA_Op[3];
assign Sel_Y   =  NA_Op[2];
assign Sel_Rel =  NA_Op[1];
//
assign Ci      = NA_Op[0];
//
assign Ld_MAR  = ~(NA_Op[7] & ~Stk_Rel);    // NA_Op[11]: not needed any longer

//  Generate Relative Address

assign Rel = ((CC) ? {OP2, OP1} : 0);

//  Generate Left Address Operand

assign AL = ((Sel_PC ) ? PC           : 0);
assign AL = ((Sel_SP ) ? {8'h01, S  } : 0);
//
assign AL = ((Sel_SPY) ? {8'h00, Y  } : 0);
//
assign AL = ((Sel_ZP ) ? {8'h00, OP1} : 0);
assign AL = ((Sel_Abs) ? {OP2  , OP1} : 0);
assign AL = ((Sel_MAR) ? MAR          : 0);

//  Generate Right Address Operand

assign AR = ((Sel_X  ) ? {8'h00, ((OAX) ? A : X)} : 0);
assign AR = ((Sel_Y  ) ? {8'h00, ((OAY) ? A : Y)} : 0);
assign AR = ((Sel_Rel) ? Rel                      : 0);
//
assign AR = ((Stk_Rel) ? {8'h00, OP1}             : 0);

//  Compute Next Address

assign NA = (AL + AR + Ci);

//  Generate Address Output - Truncate Next Address when Mod256 asserted
//      When Mod256 is asserted, the memory page is determined AL[15:8]

always @(*)
begin
    if(Rst)
        AO <= Vector;
    else
        AO <= ((Mod256) ? {AL[15:8], NA[7:0]} : NA);
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

M65C02_StkPtr   #(
                    .pStkPtr_Rst(pStkPtr_Rst)
                ) Stk (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .Rdy(Rdy), 
                    .Valid(Valid),
                    
                    .Sel(SelS), 
                    .Stk_Op(((OSY) ? 2'b0 : Stk_Op)), 
                    .D(SDI),
                    
                    .Q(SDO)
                );
                
assign S = SDO;

endmodule
