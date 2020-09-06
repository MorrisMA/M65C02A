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
/// Module Name:    M65C02A_RegStk.v
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
//  2.00    15J25   MAM     Modified the register stack to support byte swapping
//                          and rotation of nibbles for the TOS register when
//                          the IND prefix is applied to the SWP and the ROT
//                          instructions. PSW flags are not affected by these
//                          operations.
//
//  2.10    15K25   MAM     Added SIZ and T to the interface, and overloaded the
//                          DUP operation to implement TAI, TIA, and XAI opera-
//                          tions between A and IP of the FORTH VM. TAI is de-
//                          fined as IND DUP, and does not update A. TIA is de-
//                          fined as SIZ DUP, and writes T into A. XAI is de-
//                          fined as ISZ DUP, and writes T into A, and in the
//                          FORTH VM, simultaneously writes A into IP.
//
//  2.20    16C31   MAM     Improved readability of the operations assigned to
//                          TOS by adding an intermediate variable to implement
//                          byte swap logic.
//
// Additional Comments:
//
//  This particular implementation is based on discrete registers, and limited
//  to 3 registers. However, this is strictly an implementation decision, and
//  there is no reason why the register stack cannot be expanded with additional
//  registers or LUT-based LIFOs to greater depths. This particular implementa-
//  tion decision is simply a matter of expediency.
//
//  Further, the functionality of the TOS register in this implementation is
//  that of a simple register. That is also an implementation decision, and
//  additional functionality can be included for the TOS. For example, auto-
//  increment/decrement or stack pointer operations can be included as addi-
//  tional functions to the TOS register.
//
//  In addition, this implementation only provides direct access to the TOS
//  register. This is also an implementation decision and not a universal res-
//  triction. In other applications, simultaneous access to the TOS and other
//  registers in the stack may be advantageous, and additional access ports
//  and/or control signals can be easily added to the implementation.
//
//  The Rdy and Valid inputs are provided to easily tie this particular imple-
//  mentation of a register stack into the M65C02A core's internal bus. This
//  is also an implementation decision, and can be modified as necessary to
//  interface the register stack with other busses.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_RegStk #(
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
    
    input   IND,
    input   SIZ,
    
    input   DUP,
    input   SWP,
    input   ROT,
    
    input   [(pWidth - 1):0] D,
    input   [(pWidth - 1):0] T,
    
    output  [(pWidth - 1):0] Q,
    
    output  reg [(pWidth - 1):0] TOS,
    output  reg [(pWidth - 1):0] NOS,
    output  reg [(pWidth - 1):0] BOS
);

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

wire    [15:0] BSW;
assign BSW = {TOS[7:0], TOS[15:8]};

wire    [15:0] Rev;
assign Rev = {TOS[ 0], TOS[ 1], TOS[ 2], TOS[ 3],
              TOS[ 4], TOS[ 5], TOS[ 6], TOS[ 7],
              TOS[ 8], TOS[ 9], TOS[10], TOS[11],
              TOS[12], TOS[13], TOS[14], TOS[15]};

always @(posedge Clk)
begin
    if(Rst)
        {TOS, NOS, BOS} <= #1 {pTOS, pNOS, pBOS};
    else
        if(Rdy & Valid)
            case({Sel, DUP, SWP, ROT})  // Note: controls are one-hot
                4'b1000 : {TOS, NOS, BOS} <= #1 {  D, NOS, BOS};    // Load TOS
                4'b0100 : {TOS, NOS, BOS} <= #1 
                            ((SIZ) ? {  T, NOS, BOS}                // TIA
                                   : {TOS, TOS, NOS});              // DUP Stk
                4'b0010 : {TOS, NOS, BOS} <= #1
                            ((IND) ? {BSW, NOS, BOS}                // SWP TOS
                                   : {NOS, TOS, BOS});              // SWP Stk
                4'b0001 : {TOS, NOS, BOS} <= #1
                            ((IND) ? {Rev, NOS, BOS}                // Rev TOS
                                   : {NOS, BOS, TOS});              // ROT Stk
                default : {TOS, NOS, BOS} <= #1 {TOS, NOS, BOS};    // NOP
            endcase
end

assign Q = TOS;

endmodule
