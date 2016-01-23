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
// Module Name:     M65C02A_LST.v 
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module implement the Load/Store/Transfer multiplexer used by the core
//  to output data from the registers and the ALU onto the output bus. In addi-
//  tion, paths through the multiplexer are included to allow the memory operand
//  register(s), i.e. {OP2, OP1}, to be passed through the ALU to the output.
//  The module also passes the FORTH VM registers through the ALU to the output
//  to support the PHI and IND PHI instructions.
//
//  The module also supports the OAX, OAY, and OSX prefix instructions. Multi-
//  plexers are included to implement the required register swaps to support
//  these prefix instructions.
//
// Dependencies:    None.
//
// Revision:
// 
//  1.00    15A03   MAM     Initial release.
//
//  1.01    15K21   MAM     Updated comments to match the implementation.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_LST(
    input   En,
    
    input   OAX,            // Override: swap A and X
    input   OAY,            // Override: swap A and Y
    input   OSX,            // Override: swap X and S, and use X as SP
    
    input   [2:0] OSel,

    input   [15:0] X,
    input   [15:0] Y,
    input   [15:0] A,
    input   [15:0] T,
    input   [15:0] S,
    input   [15:0] P,
    input   [15:0] M,
    
    output  [16:0] Out,
    output  Val
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wor     [16:0] Mux;
wire    [ 7:1] Sel;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Define the X output select
//      if(OAX) X => A
//      else if(OSX) X => S
//      else X

assign Sel[1] = En & (  ((OSel == 3) &  OAX)            // X
                      | ((OSel == 5) &  OSX)
                      | ((OSel == 1) & ~(OAX | OSX)));

//  Define the Y output select
//      if(OAY) Y => A
//      else Y

assign Sel[2] = En & (  ((OSel == 3) &  OAY)            // Y
                      | ((OSel == 2) & ~OAY));
                      
//  Define the A output select
//      if(OAX) A => X
//      else if(OAY) A => Y
//      else A;
                                  
assign Sel[3] = En & (  ((OSel == 1) &  OAX)            // A
                      | ((OSel == 2) &  OAY)
                      | ((OSel == 3) & ~(OAX | OAY)));
                      
//  Define the Tmp ({IP | W}) output select

assign Sel[4] = En & (OSel == 4);                       // T ({IP | W})

//  Define the S output select
//      if(OSX) S => X
//      else S;

assign Sel[5] = En & (  ((OSel == 1) &  OSX)            // S
                      | ((OSel == 5) & ~OSX));

//  Define the P (PSW) output select

assign Sel[6] = En & (OSel == 6);                       // P

//  Define the M ({OP2, OP1}) output select

assign Sel[7] = En & (OSel == 7);                       // M ({OP2, OP1})

//  Load/Store/Transfer Multiplexer

//  Generate wired-OR multiplexer

assign Mux = ((Sel[1]) ? {1'b0, X}   : 0);      // STX/TXA/TXS/PHX
assign Mux = ((Sel[2]) ? {1'b0, Y}   : 0);      // STY/TYA/PHY
assign Mux = ((Sel[3]) ? {1'b0, A}   : 0);      // STA/TAX/TAY/PHA
assign Mux = ((Sel[4]) ? {1'b0, T}   : 0);      // PHI (Forth VM)
assign Mux = ((Sel[5]) ? {1'b0, S}   : 0);      // TSX
assign Mux = ((Sel[6]) ? {1'b0, P}   : 0);      // PHP
assign Mux = ((Sel[7]) ? {1'b0, M}   : 0);      // LDA/PLA/LDX/PLX/LDY/PLY/PLP

//  Assign Module Outputs

assign Out = Mux;
assign Val = En;

endmodule
