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
// Module Name:     M65C02A_LU.v 
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module implements the logic instructions supported by the 6502/65C02
//  instruction set architecture. Support is provided by the module for all
//  logic instructions: ORA/ANL/EOR/BIT/TRB/TSB/RMBx/SMBx/BBRx/BBSx, and
//  SEC/CLC/SEI/CLI/SED/CLD/CLV. The module supports both 8-bit and 16-bit ver-
//  sions of all (if applicable) of these instructions.
//
// Dependencies:    None.
//
// Revision:
// 
//  1.00    15A03   MAM     Initial release.
//
//  1.01    15K21   MAM     Updated comments to correct errors.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_LU(
    input   En,
    
    input   SIZ,
    
    input   [ 1:0] Op,
    input   [15:0] L,
    input   [15:0] M,
    
    output  [16:0] Out,
    output  Z,
    
    output  Val
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     [15:0] DO;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(*)
begin
    if(En)
        case(Op)
            2'b00 : DO <= (~L & M);    //     TRB/RMBx/CLC/CLI/CLD/CLV
            2'b01 : DO <= ( L & M);    // AND/BIT/BBRx/BBSx
            2'b10 : DO <= ( L | M);    // ORA/TSB/SMBx/SEC/SEI/SED
            2'b11 : DO <= ( L ^ M);    // EOR
        endcase
    else
        DO <= 0;
end

assign Out = ((En) ? ((SIZ) ? {1'b0, DO} : {9'h00, DO[7:0]}   ) : 0);
assign Z   = ((En) ? ((SIZ) ? ~|(L & M)  : ~|((L & M) & 8'hFF)) : 0);
assign Val = En;

endmodule
