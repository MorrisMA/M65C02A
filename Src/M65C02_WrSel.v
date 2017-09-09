////////////////////////////////////////////////////////////////////////////////
//
//  ALU Register Write Cntl module for M65C02A soft-core microcomputer project.
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
// Create Date:     19:43:14 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_WrSel.v 
// Project Name:    C:\XProjects\ISE10.1i\MAM6502 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//
// Dependencies: 
//
// Revision: 
// 
//  0.01    13I15   MAM     File Created
//
//  1.00    13J07   MAM     Updated headers and modified the original implemen-
//                          tation to use two ROMs instead of one 64x5 ROM. One
//                          ROM provides a direct decode of the Reg_WE field,
//                          the other provides a decode of the WSel fiedl when
//                          (Reg_WE == 3'b100). This change provides a signifi-
//                          cant improvement to the M65C02_ALUv2 module when
//                          synthesize as a standalone module.
//
//  1.10    14F28   MAM     Adjusted comments to reflect changes made during
//                          integration: (1) corrected the uP driven write
//                          enable logic to select PSW when writing A, X, or Y.
//
//  1.20    14I02   MAM     Modified to support OAX and OAY prefix instructions.
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

module M65C02_WrSel(
    input   Rst,
    input   Clk,
    
    input   OAX,            // Register Override: swap A and X
    input   OAY,            // Register Override: swap A and Y
    input   OSY,            // Register Override: swap Y and S, Y is SP
    
    input   [2:0] Reg_WE,
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

reg     uP_A, uP_X, uP_Y, uP_P, uP_S;
reg     iD_A, iD_X, iD_Y, iD_P, iD_S;
wire    WE;

//  Decode Register Write Enables

always @(*)
begin
    case(Reg_WE)
        3'b000 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b000_0_0;
        3'b001 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b100_1_0;
        3'b010 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b010_1_0;
        3'b011 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b001_1_0;
        3'b100 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b000_0_0;
        3'b101 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b000_0_1;
        3'b110 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b000_1_0;
        3'b111 : {uP_X, uP_Y, uP_A, uP_P, uP_S} <= 5'b000_0_0;
    endcase
end

assign WE = (Reg_WE == 3'b100);

always @(*)
begin
    case({WE, WSel})
        4'b0000 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0001 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0010 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0011 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0100 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0101 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0110 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
        4'b0111 : {iD_X, iD_Y, iD_A, iD_P, iD_S} <= 5'b000_0_0;
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
        {SelA, SelX, SelY, SelP, SelS} <= #1 0;
    else begin
        SelA <= #1 (  ((uP_X | iD_X) &   OAX)
                    | ((uP_Y | iD_Y) &   OAY)
                    | ((uP_A | iD_A) & ~(OAX | OAY)));
        //
        SelX <= #1 (  ((uP_A | iD_A) &  OAX)
                    | ((uP_X | iD_X) & ~OAX));
        //
        SelY <= #1 (  ((uP_A | iD_A) &   OAY)
                    | ((uP_S | iD_S) &   OSY)
                    | ((uP_Y | iD_Y) & ~(OAY | OSY)));
        //
        SelP <= #1 (uP_P | iD_P);
        // 
        SelS <= #1 (  ((uP_Y | iD_Y) &  OSY)
                    | ((uP_S | iD_S) & ~OSY));
    end
end

endmodule
