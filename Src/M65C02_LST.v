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
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_LST(
    input   En,
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
reg     [7:1] Sel;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(*)
begin
    casex({En, OSel})
        4'b0000 : Sel <= 7'h00;
        4'b0001 : Sel <= 7'h00;
        4'b0010 : Sel <= 7'h00;
        4'b0011 : Sel <= 7'h00;
        4'b0100 : Sel <= 7'h00;
        4'b0101 : Sel <= 7'h00;
        4'b0110 : Sel <= 7'h00;
        4'b0111 : Sel <= 7'h00;
        4'b1000 : Sel <= 7'h00;
        4'b1001 : Sel <= 7'h01;
        4'b1010 : Sel <= 7'h02;
        4'b1011 : Sel <= 7'h04;
        4'b1100 : Sel <= 7'h08;
        4'b1101 : Sel <= 7'h10;
        4'b1110 : Sel <= 7'h20;
        4'b1111 : Sel <= 7'h40;
    endcase
end

//  Load/Store/Transfer Multiplexer

//  Generate wired-OR multiplexer

assign Mux = ((Sel[1]) ? {1'b0, A}   : 0);      // STA/TAX/TAY/TAS/PHA
assign Mux = ((Sel[2]) ? {1'b0, X}   : 0);      // STX/TXA/TXS/PHX
assign Mux = ((Sel[3]) ? {1'b0, Y}   : 0);      // STY/TYA/PHY
assign Mux = ((Sel[4]) ? {1'b0, Tmp} : 0);      // PHW/PHR
assign Mux = ((Sel[5]) ? {1'b0, S}   : 0);      // TSX/TSA
assign Mux = ((Sel[6]) ? {1'b0, P}   : 0);      // PHP
assign Mux = ((Sel[7]) ? {1'b0, M}   : 0);      // LDA/PLA/LDX/PLX/LDY/PLY/PLP

//  Assign Module Outputs

assign Out = Mux;
assign Val = En;

endmodule
