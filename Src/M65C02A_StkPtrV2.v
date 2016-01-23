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
// Create Date:     06/21/2015 
// Design Name:     Enhanced Microprogrammed 6502-Compatible Soft-Core Processor
// Module Name:     M65C02A_StkPtrV2.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A
// Target Devices:  Generic SRAM-based FPGA
// Tool versions:   Xilinx ISE 10.1i SP3
// 
// Description:
//
//  This module implements the functions of the M65C02A stack pointer. It is an
//  enhancement of the standard stack pointer function, which continues to be
//  used as the alternate stack pointer implemented using the X register. This
//  module supports two stack pointers: (1) kernel mode stack pointer (Sk) and
//  (2) user mode stack pointer (Su).
//
//  The user mode stack pointer is used for normal stack operations when the M
//  (mode) bit in the P register is cleared. Su can be written and read using
//  the TXS and TSX instructions while in user mode. In addition, it can be
//  written and read while in the kernel mode using the TXS and TSX instructions
//  by applying the IND and/or ISZ prefix instructions. (Note: while in the user
//  mode, the IND prefix is ignored. Thus, writing or reading Sk while in the
//  user mode is prevented.)
//
// Dependencies:    none.
//
// Revision: 
//
//  1.00    15F21   MAM     Initial release.
//
//  1.10    15G11   MAM     Modified declared parameters to include widths. As
//                          originally defined without widths, they were being
//                          treated as 32-bit. Thus, kernel and user stack ptrs
//                          were being initialized in page 0 rather than page 1
//                          as required.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_StkPtrV2 #(
    parameter pDef_Page = 8'd1,
    parameter pSk_Rst   = 8'd2,
    parameter pSu_Rst   = 8'd255
)(
    input   Rst,
    input   Clk,
    
    input   Mode,               // Core Operating Mode: 0 - User; 1 - Kernel

    input   Ind,                // Stack Pointer indirection flag (IND)
    input   Size,               // Stack Pointer Operation Size   (SIZ)
    
    input   Mod,                // % 256 Pointer Limit
    
    input   Rdy,
    input   Valid,
    
    input   Sel,                // Stack Pointer Select
    input   [ 1:0] Stk_Op,      // Stack Pointer Operation 
    input   [15:0] SDI,         // Stack Pointer Input Data from ALU/Registers
    output  [15:0] SDO,         // Stack Pointer Output Data to ALU/Registers
    
    output  [15:0] S            // Stack Pointer Output to Address Generator
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wire    [15:0] Sum, StkIn;
reg     [15:0] Sk, Su;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

assign Sum   = S + ((Stk_Op[0]) ? ( 1) : (~0));     // ? Pop (+1) : Push (-1)
assign StkIn = ((Mod) ? {S[15:8], Sum[7:0]} : Sum); // ? mod 256  : mod 65536

//  Kernel Stack Pointer - Sk

assign Ld_Sk = (Mode & ~Ind) & Rdy & (Sel & Valid);
assign CE_Sk =  Mode         & Rdy & Stk_Op[1];

always @(posedge Clk)
begin
    if(Rst)
        Sk <= #1 {pDef_Page, pSk_Rst};
    else if(Ld_Sk)
        Sk <= #1 ((Size) ? SDI : {pDef_Page, SDI[7:0]});
    else if(CE_Sk)
        Sk <= #1 StkIn;    
end

//  User Stack Pointer - Su

assign Ld_Su = (~Mode | Mode & Ind) & Rdy & (Sel & Valid);
assign CE_Su =  ~Mode               & Rdy & Stk_Op[1];

always @(posedge Clk)
begin
    if(Rst)
        Su <= #1 {pDef_Page, pSu_Rst};
    else if(Ld_Su)
        Su <= #1 ((Size) ? SDI : {pDef_Page, SDI[7:0]});
    else if(CE_Su)
        Su <= #1 StkIn;    
end

//  Output Data Multiplexers

assign S   = (( Mode)              ? Sk : Su); // To Address Generator
assign SDO = ((~Mode | Mode & Ind) ? Su : Sk); // To ALU/Registers

endmodule
