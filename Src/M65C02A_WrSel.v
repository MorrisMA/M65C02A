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
// Module Name:     M65C02A_WrSel.v 
// Project Name:    C:\XProjects\ISE10.1i\MAM6502 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module generates the register write enable signals for the various
//  registers of the M65C02A processor core: X, Y, A, P, and S. It also imple-
//  ments support for the OAX, OAY, and OSX prefix instructions.
//
// Dependencies: 
//
// Revision: 
// 
//  1.00    15A03   MAM     Initial release.
//  
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_WrSel(
    input   Rst,
    input   Clk,
    
    input   OAX,            // Register Override: swap A and X
    input   OAY,            // Register Override: swap A and Y
    input   OSX,            // Register Override: swap X and S, X is SP
    
    input   WE,
    input   [2:0] WSel,
    
    output  reg SelA,
    output  reg SelX,
    output  reg SelY,
    output  reg SelP,
    
    output  reg SelS
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     iD_A, iD_X, iD_Y, iD_P, iD_S;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(*)
begin
    casex({WE, WSel})
        4'b0xxx : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b1000 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b1001 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b100_1_0;
        4'b1010 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b010_1_0;
        4'b1011 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b001_1_0;
        4'b1100 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b1101 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_1;
        4'b1110 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_1_0;
        4'b1111 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_1_0;
    endcase
end

always @(negedge Clk or posedge Rst)
begin
    if(Rst)
        {SelX, SelY, SelA, SelP, SelS} <= #1 0;
    else begin
        SelX <= #1 (  (iD_A &  OAX)
                    | (iD_S &  OSX)
                    | (iD_X & ~(OAX | OSX)));
        //
        SelY <= #1 (  (iD_A &  OAY)
                    | (iD_Y & ~OAY));
        //
        SelA <= #1 (  (iD_X &   OAX)
                    | (iD_Y &   OAY)
                    | (iD_A & ~(OAX | OAY)));
        //
        SelP <= #1 (iD_P);
        //
        SelS <= #1 (  (iD_X &  OSX)
                    | (iD_S & ~OSX));
    end
end

endmodule
