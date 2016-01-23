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
// Module Name:     M65C02A_SU.v 
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module implements the shift/rotate instructions supported by the 
//  6502/65C02 instruction set architecture. Support is provided by the module
//  for shift/rotate instructions: ASL/ROL/LSR/ROR. The module supports both
//  8-bit and 16-bit versions of all (if applicable) of these instructions.
//
// Dependencies:    None.
//
// Revision:
// 
//  1.00    15A03   MAM     Initial release.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_SU (
    input   En,
    
    input   SIZ,
    
    input   Op,
    input   [15:0] SU,
    input   Ci,
    
    output  reg [16:0] Out,
    output  reg OV,
    output  Val
);

always @(*)
begin
    if(En)
        if(SIZ)
            {OV, Out} <= ((Op) ? {^SU[15:14],                   // OV    LSR/ROR
                                 { SU[ 0],                      // Out[16] -- Co 
                                 { Ci, SU[15:1]}}}              // Out[15:0]
                               : {^SU[15:14],                   // OV    ASL/ROL
                                 { SU[15],                      // Out[16] -- Co 
                                 { SU[14:0], Ci}}});            // Out[15:0]
        else
            {OV, Out} <= ((Op) ? {^SU[ 7: 6],                   // OV    LSR/ROR
                                 { SU[ 0],                      // Out[16] -- Co 
                                 { 8'h00, {Ci, SU[7:1]}}}}      // Out[15:0]
                               : {^SU[ 7: 6],                   // OV    ASL/ROL
                                 { SU[ 7],                      // Out[16] -- Co 
                                 { 8'h00, {SU[6:0], Ci}}}});    // Out[15:0]    
    else
        {OV, Out} <= 0;
end

assign Val = En;

endmodule
