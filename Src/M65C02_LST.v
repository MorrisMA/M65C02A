////////////////////////////////////////////////////////////////////////////////
//
//  ALU Load/Store/Transfer module for M65C02A soft-core microcomputer project.
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

`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
// 
// Create Date:     17:06:48 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_LST.v 
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
// Dependencies:    None.
//
// Revision:
// 
//  0.01    13I15   MAM     Initial coding. Pulled implementation details from
//                          the parent module, M65C02_ALU.v, and generated a
//                          standalone module instantiated in the parent.
//
//  1.00    14I02   MAM     Modified the OSel field encoding to allow the modi-
//                          fication of the destination register. Encoding makes
//                          A = 3, Y = 2, and X = 1. This allows the OAX and OAY
//                          prefix instructions, which are mutually exclusive,
//                          to be applied using XOR gates to the OSel field and
//                          change A into X or A into Y.
//
//  1.30    14L06   MAM     Completed modifications to incorporate OAX, OAY, 
//                          and OSY prefix instructions: OAX switches A and X;
//                          OAY switches A and Y; and OSY switches Y with S for
//                          stack operations, and Y and S in the Y-specific
//                          instructions.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_LST(
    input   En,
    
    input   OAX,            // Override: swap A and X
    input   OAY,            // Override: swap A and Y
    input   OSY,            // Override: swap Y and S, and use Y as SP
    
    input   [2:0] OSel,

    input   [7:0] A,
    input   [7:0] X,
    input   [7:0] Y,
    input   [7:0] Tmp,
    input   [7:0] S,
    input   [7:0] P,
    input   [7:0] M,
    
    output  [8:0] Out,
    output  Val
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wor     [8:0] Mux;
wire    [7:1] Sel;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Define the X output select
//      if(OAX) X => A
//      else X

assign Sel[1] = En & (  ((OSel == 3) &  OAX)            // X
                      | ((OSel == 1) & ~OAX));

//  Define the Y output select
//      if(OAY) Y => A
//      else if(OSY) Y => S
//      else Y

assign Sel[2] = En & (  ((OSel == 3) &  OAY)            // Y
                      | ((OSel == 5) &  OSY)
                      | ((OSel == 2) & ~(OAY | OSY)));
                      
//  Define the A output select
//      if(OAX) A => X
//      else if(OAY) A => Y
//      else A;
                                  
assign Sel[3] = En & (  ((OSel == 1) &  OAX)            // A
                      | ((OSel == 2) &  OAY)
                      | ((OSel == 3) & ~(OAX | OAY)));
                      
//  Define the Tmp (OP1) output select

assign Sel[4] = En & (OSel == 4);                       // T (OP2)

//  Define the S output select
//      if(OSY) S => Y
//      else S;

assign Sel[5] = En & (  ((OSel == 2) &  OSY)            // S
                      | ((OSel == 5) & ~OSY));

//  Define the P (PSW) output select

assign Sel[6] = En & (OSel == 6);                       // P

//  Define the M (OP1) output select

assign Sel[7] = En & (OSel == 7);                       // M (OP1)

//  Load/Store/Transfer Multiplexer

//  Generate wired-OR multiplexer

assign Mux = ((Sel[1]) ? {1'b0, X}   : 0);      // STX/TXA/TXS/PHX
assign Mux = ((Sel[2]) ? {1'b0, Y}   : 0);      // STY/TYA/PHY
assign Mux = ((Sel[3]) ? {1'b0, A}   : 0);      // STA/TAX/TAY/PHA
assign Mux = ((Sel[4]) ? {1'b0, Tmp} : 0);      // 
assign Mux = ((Sel[5]) ? {1'b0, S}   : 0);      // TSX
assign Mux = ((Sel[6]) ? {1'b0, P}   : 0);      // PHP
assign Mux = ((Sel[7]) ? {1'b0, M}   : 0);      // LDA/PLA/LDX/PLX/LDY/PLY/PLP

//  Assign Module Outputs

assign Out = Mux;
assign Val = En;

endmodule
