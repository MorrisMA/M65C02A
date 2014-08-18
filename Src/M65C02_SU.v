////////////////////////////////////////////////////////////////////////////////
//
//  ALU Shift/rotate Unit module for M65C02A soft-core microcomputer project.
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
// Create Date:     08:15:35 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_SU.v 
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
//  1.00    13J23   MAM     Corrected error in the operation multiplexer. Op = 1
//                          is for right shift/rotate operations, and Op = 0 is
//                          for left shift/rotate operations.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_SU(
    input   En,
    
    input   Op,
    input   [7:0] SU,
    input   Ci,
    
    output  reg [8:0] Out,
    output  reg OV,
    output  Val
);

always @(*)
begin
    if(En)
        {OV, Out} <= ((Op) ? {^SU[7:6], {SU[0], {Ci, SU[7:1]}}}     // LSR/ROR
                           : {^SU[7:6], {SU[7], {SU[6:0], Ci}}});   // ASL/ROL
    else
        {OV, Out} <= 0;
end

assign Val = En;

endmodule
