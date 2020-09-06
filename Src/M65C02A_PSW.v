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
// Module Name:     M65C02A_PSW.v
// Project Name:    C:\XProjects\ISE10.1i\M6502A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  Module implements the PSW of the M65C02_ALUv2 module.  
//
// Dependencies:    None
//
// Revision:
//
//  1.00    15A03   MAM     Initial release.
//
//  1.10    15B08   MAM     Modified to incorporate the mode bit. Labeled as K
//                          instead of M as shown in the User's Guide, because
//                          the M signal name already used for the memory op.
//                          The CCSel code for pBRK is reused during RTI. When
//                          the RTI instruction writes the PSW, the K (mode) bit
//                          is updated. Required expansion of PSW from 6 to 7
//                          bits. Also forced K to 1 when an interrupt is taken.
//
//  1.11    15F28   MAM     Added (but left commented out) logic to support COP
//                          status testing using the PSW.V flag.
//
//  1.20    15G18   MAM     Corrected issue with the Kernel mode and the BRK
//                          handling added when Kernel/User modes and Sk and Su
//                          stack pointers added to the module. The wrong stack
//                          pointer was being used part of the time that the
//                          BRK microroutine was executing, and switching to
//                          Kernel mode with BRK was clearing the PSW.
//
//  1.30    15G21   MAM     Correction that allows the K (Mode) to be loaded
//                          from data on the stack when PLP/RTI executed. K is 
//                          only loaded from the stack or when the current mode
//                          is set to Kernel, otherwise the mode is preserved.
//
//  1.31    15G31   MAM     Changed the logic that allows modification of K bit
//                          so that K is allowed to change only on an RTI; PLP
//                          cannot be allowed to change the K bit. CCSel field
//                          passed in as pBRK during RTI instruction to imple-
//                          ment this feature.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_PSW (
    input   Clk,
    
    input   SO,
    output  Clr_SO,
    
    input   COP_SO,
    
    input   SelP,
    input   Valid,
    input   Rdy,
    
    input   ISR,
    
    input   [3:0] CCSel,
    
    input   ALU_C,
    input   ALU_Z,
    input   ALU_V,
    input   ALU_N,

    input   LU_Z,
    input   [ 7:6] M,

    input   [15:0] DO,

    output  [ 7:0] P,
    
    output  N,
    output  V,
    output  K,
    output  B,
    output  D,
    output  I,
    output  Z,
    output  C
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameters
//

localparam pPSW  = 3'b000;  // Set P from ALU
localparam pBRK  = 3'b001;  // Set P.4 when pushing P during interrupt handling
localparam pZ    = 3'b010;  // Set Z = ~|(A & M)
localparam pNVZ  = 3'b011;  // Set N and V flags from M[7:6], and Z = ~|(A & M)
localparam pPHP  = 3'b100;  // Set P.4 when executing PHP instruction
localparam pNZ   = 3'b101;  // Set N and Z flags from ALU
localparam pNZC  = 3'b110;  // Set N, Z, and C flags from ALU
localparam pNVZC = 3'b111;  // Set N, V, Z, and C from ALU

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     [6:0] PSW;
wire    SV;

////////////////////////////////////////////////////////////////////////////////
//
//  PSW - Processor Status Word: {N, V, K, D, I, Z, C}
//

assign WE_P = ((SelP | SO | COP_SO) & Valid & Rdy); // Write Enable for PSW
assign SV   = (        SO | COP_SO);                // Set V - SO | COP_SO

always @(posedge Clk)
begin
    if(ISR & Rdy)
        PSW <= #1 {N, V, 1'b1, 1'b0, 1'b1, Z, C};
    else if(WE_P) 
        case(CCSel[2:0])
            pPSW  : PSW <= #1 {DO[7], (DO[6] | SV), K    , DO[3:0]           };
            pBRK  : PSW <= #1 {DO[7], (DO[6] | SV), DO[5], DO[3:0]           };
            pZ    : PSW <= #1 {    N, (V     | SV), K    , D, I,  LU_Z,     C};
            pNVZ  : PSW <= #1 { M[7], (M[6]  | SV), K    , D, I,  LU_Z,     C};
            pPHP  : PSW <= #1 {DO[7], (V     | SV), K    , D, I, ALU_Z,     C};
            pNZ   : PSW <= #1 {ALU_N, (V     | SV), K    , D, I, ALU_Z,     C};
            pNZC  : PSW <= #1 {ALU_N, (V     | SV), K    , D, I, ALU_Z, ALU_C};
            pNVZC : PSW <= #1 {ALU_N, (ALU_V | SV), K    , D, I, ALU_Z, ALU_C};
        endcase
end

//  Decode CCSel

assign BRK = ((CCSel[3]) ? 1'b0 : (CCSel[2:0] == pBRK));
assign PHP = ((CCSel[3]) ? 1'b0 : (CCSel[2:0] == pPHP));

//  Decode PSW bits

assign N = PSW[6];  // Negative, nominally Out[7], but M[7] if BIT/TRB/TSB
assign V = PSW[5];  // oVerflow, nominally OV,     but M[6] if BIT/TRB/TSB
assign K = PSW[4];  // Mode, nominally 1, modified by DO[5] only during RTI
assign B = (BRK | PHP); // Break
assign D = PSW[3];  // Decimal, set/cleared by SED/CLD, cleared on ISR entry
assign I = PSW[2];  // Interrupt Mask, set/cleared by SEI/CLI, set on ISR entry
assign Z = PSW[1];  // Zero, nominally ~|Out, but ~|(A&M) if BIT/TRB/TSB
assign C = PSW[0];  // Carry, set/cleared  by SEC/CLC, ADC/SBC, ASL/ROL/LSR/ROR

//  Assign PSW bits to P (PSW output port)

assign P = {N, V, K, B, D, I, Z, C};

//  Generate for Acknowledge SO Command

assign Clr_SO = SO & Valid & Rdy;

endmodule
