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
// Module Name:     M65C02A_StkPtr.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A
// Target Devices:  Generic SRAM-based FPGA
// Tool versions:   Xilinx ISE 10.1i SP3
// 
// Description:
//
//  This module implements the functions of the M65C02A stack pointer. 
//
// Dependencies:    none.
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_StkPtr #(
    parameter pStkPtr_Rst = 0
)(
    input   Rst,
    input   Clk,
    
    input   Page,               // Stack Pointer Page
    input   Size,               // Stack Pointer Operation Size
    input   Mod,                // % 256 Pointer Limit
    
    input   Rdy,
    input   Valid,
    
    input   Sel,
    input   [ 1:0] Stk_Op,
    input   [15:0] D,
    
    output  reg [15:0] Q
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wire    [15:0] Sum;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Stack Pointer

assign Ld = Rdy & (Sel & Valid);
assign CE = Rdy & Stk_Op[1];

assign Sum = Q + ((Stk_Op[0]) ? ( 1) : (~0));  // ? Pop  (+1) : Push (-1)

always @(posedge Clk)
begin
    if(Rst)
        Q <= #1 {{7{1'b0}}, Page, pStkPtr_Rst};
    else if(Ld)
        Q <= #1 ((Size) ? D : {{7{1'b0}}, Page, D[7:0]});
    else if(CE)
        Q <= #1 ((Mod) ? {Q[15:8], Sum[7:0]} : Sum);    
end

endmodule
