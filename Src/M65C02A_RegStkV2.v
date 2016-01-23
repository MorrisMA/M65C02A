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
// Module Name:     M65C02A_RegStkV2.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This module implements a push-down register stack that provides a minimum of
//  three registers: {TOS, NOS, BOS}. The Sel, DUP, SWP, and ROT inputs provide
//  the controls for the register stack. When Sel is asserted, the value pro-
//  vided on D is written into the TOS (Top-Of-Stack) register.
//
//  The DUP, SWP, and ROT inputs provide the controls for implementing the stack
//  operations.
//  
//  When DUP is asserted, the TOS of stack register is pushed down into the 
//  push-down register stack. Each of the other registers in the stack is pushed
//  down one level, and the value in the BOS register is discarded:
//
//      TOS <= TOS; NOS <= TOS; BOS <= NOS;
//  
//  When SWP is asserted, the values of the TOS and NOS registers are exchanged:
//
//      TOS <= NOS; NOS <= TOS; BOS <= BOS;
//
//  When ROT is asserted, the value in TOS is written into the BOS register, and
//  the other registers are popped one level:
//
//      TOS <= NOS; NOS <= BOS; BOS <= TOS;
//
// Dependencies: 
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release.
//
//  1.10    15H08   MAM     Corrected the operation of the register stack. When
//                          stack manipulation instructions were executed, only
//                          the NOS and BOS registers were affected. The TOS 
//                          element unaffected by the SWP and ROT stack instruc-
//                          tions. Select for TOS register changed from only Sel
//                          to |{Sel, DUP, SWP, ROT}.
//
// Additional Comments:
//
//  The functionality of the TOS register in this implementation includes the
//  functionality of a 6502/65C02 style stack pointer. Additional functionality
//  is possible, as described in the "Additional Comments" of the M65C02A_RegStk
//  module, for each of the registers of the register stack, but the
//  target implementation of this version of the module only requires the addi-
//  tional capabilities to implement a 6502/65C02-compatible stack.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_RegStkV2 #(
    parameter pWidth = 16,
    parameter pTOS   = 16'h0000,
    parameter pNOS   = 16'h0000,
    parameter pBOS   = 16'h0000
)(
    input   Rst,
    input   Clk,
    
    input   Rdy,
    input   Valid,
    
    input   Sel,
    
    input   DUP,
    input   SWP,
    input   ROT,
    
    input   [(pWidth - 1):0] D,
    output  [(pWidth - 1):0] Q,
    
    input   Page,
    input   Size,
    input   Mod,
    
    input   [1:0] Stk_Op,
    
    output      [(pWidth - 1):0] TOS,
    output  reg [(pWidth - 1):0] NOS,
    output  reg [(pWidth - 1):0] BOS
);

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

reg    [15:0] iTOS, iNOS, iBOS;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(*)
begin
    if(Rst)
        {iTOS, iNOS, iBOS} <= #1 {pTOS, pNOS, pBOS};
    else if(Rdy & Valid)
        case({Sel, DUP, SWP, ROT})  // Note: controls are one-hot
            4'b1000 : {iTOS, iNOS, iBOS} <= #1 {D,   NOS, BOS};    // Load TOS
            4'b0100 : {iTOS, iNOS, iBOS} <= #1 {TOS, TOS, NOS};    // DUP
            4'b0010 : {iTOS, iNOS, iBOS} <= #1 {NOS, TOS, BOS};    // SWP/XCH
            4'b0001 : {iTOS, iNOS, iBOS} <= #1 {NOS, BOS, TOS};    // ROT
            default : {iTOS, iNOS, iBOS} <= #1 {TOS, NOS, BOS};    // NOP
        endcase
    else
        {iTOS, iNOS, iBOS} <= #1 {TOS, NOS, BOS};
end

always @(posedge Clk)
begin
    if(Rst)
        {NOS, BOS} <= #1 {pNOS, pBOS};
    else
        {NOS, BOS} <= #1 {iNOS, iBOS};
end

//  Implement TOS as a 6502/65C02-compatible stack pointer

M65C02A_StkPtr  #(
                    .pStkPtr_Rst(0)
                ) AuxStk (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .Rdy(Rdy), 
                    .Valid(Valid),
                    
                    .Sel(Sel | DUP | SWP | ROT), 

                    .Page(Page),
                    .Size(Size),
                    .Mod(Mod),
                    
                    .Stk_Op(Stk_Op), 
                    
                    .D(iTOS),
                    .Q(TOS)
                );

assign Q = TOS;

endmodule
