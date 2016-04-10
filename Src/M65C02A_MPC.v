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
// Module Name:     M65C02A_MPC.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02A
// Target Devices:  Generic SRAM-based FPGA
// Tool versions:   Xilinx ISE 10.1i SP3
// 
// Description:
//
// This module implements a simple microprogram sequencer based on the Fair-
// child F9408. The sequencer provides:
//
//          (1) 4-bit instruction input
//          (2) four-level LIFO stack;
//          (3) program counter and incrementer;
//          (4) 4-bit registered test input;
//          (5) 8-way multi-way branch control input;
//          (6) branch address input;
//          (7) 4-way branch address select output;
//          (8) next address output.
//
// These elements provide a relatively flexible general purpose microprogram
// controller without a complex instruction set. The sixteen instructions can
// be categorized into three classes: (1) fetch, (2) unconditional branches,
// and (3) conditional branches. The fetch instruction class, a single instruc-
// tion class, simply increments the program counter and outputs the current
// value of the program counter on the next address bus. The unconditional 
// branch instruction class provides instructions to select the next instruc-
// tion using the Via[1:0] outputs and output that value on the next address
// bus and simultaneously load the program counter. The unconditional branch
// instruction class also provides for 8-way multiway branching using an exter-
// nal (priority) encoder/branch selector, and microprogram subroutine call and 
// return instructions.
//
// As provided in "Principles of Firmware Engineering in Microprogram Control"
// by Michael Andrews, the instruction set and operation map for the F9408
// implementation is given below:
//
//  I[3:0] MNEM Definition       T[3:0]      MA[m:0]      Via Inh  Operation
//   0000  RTS  Return            xxxx      TOS[m:0]       00  0  PC<=MA;Pop
//   0001  BSR  Call Subroutine   xxxx       BA[m:0]       00  1  PC<=MA;Push
//   0010  FTCH Next Instruction  xxxx        PC+1         00  0  PC<=MA[m:0]
//   0011  BMW  Multi-way Branch  xxxx  {BA[m:3],MW[2:0]}  00  1  PC<=MA[m:0]
//   0100  BRV0 Branch Via 0      xxxx       BA[m:0]       00  1  PC<=MA[m:0]
//   0101  BRV1 Branch Via 1      xxxx       BA[m:0]       01  1  PC<=MA[m:0]
//   0110  BRV2 Branch Via 2      xxxx       BA[m:0]       10  1  PC<=MA[m:0]
//   0111  BRV3 Branch Via 3      xxxx       BA[m:0]       11  1  PC<=MA[m:0]
//   1000  BTH0 Branch T0 High    xxx1  {T0?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1001  BTH1 Branch T1 High    xx1x  {T1?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1010  BTH2 Branch T2 High    x1xx  {T2?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1011  BTH3 Branch T3 High    1xxx  {T2?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1100  BTL0 Branch T0 Low     xxx0  {T0?PC+1:BA[m:0]}  00  1  PC<=MA[m:0]
//   1101  BTL1 Branch T1 Low     xx0x  {T1?PC+1:BA[m:0]}  00  1  PC<=MA[m:0]
//   1110  BTL2 Branch T2 Low     x0xx  {T2?PC+1:BA[m:0]}  00  1  PC<=MA[m:0]
//   1111  BTL3 Branch T3 Low     0xxx  {T3?PC+1:BA[m:0]}  00  1  PC<=MA[m:0]
//
// Dependencies:    none.
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release.
//
//  2.00    15xxx   MAM     Modified the instruction set such that multi-way
//                          branches are implemented using a dedicated adder
//                          rather than bit stuffing as used in the original.
//                          Adding the multi-way field to the branch address
//                          field allows the multi-way tables to be placed on
//                          any boundary rather than on alignment boundaries of
//                          2 (1 select bit), 4 (2 select bits), or 8 (3 select
//                          bits) microprogram words. This modification greatly
//                          improves the readability of the microprogram with
//                          only a minor performance and resource utilization
//                          penalty. Four additional BMW instructions were added
//                          to support the various enhanced instructions. These
//                          four instructions, BMWx, replaced the BTLx instruc-
//                          tions. The BTHx instructions were retained for con-
//                          ditional testing of the T inputs to the module.
//
// Additional Comments: 
//
//  Modified instruction set:
//
//  I[3:0] MNEM Definition       T[3:0]      MA[m:0]      Via Inh  Operation
//   0000  RTS  Return            xxxx      TOS[m:0]       00  0  PC<=MA;Pop
//   0001  BSR  Call Subroutine   xxxx       BA[m:0]       00  1  PC<=MA;Push
//   0010  FTCH Next Instruction  xxxx        PC+1         00  0  PC<=MA[m:0]
//   0011  BMW  Multi-way Branch  xxxx       {BA+MW}       00  1  PC<=MA[m:0]
//   0100  BRV0 Branch Via 0      xxxx       BA[m:0]       00  1  PC<=MA[m:0]
//   0101  BRV1 Branch Via 1      xxxx       BA[m:0]       01  1  PC<=MA[m:0]
//   0110  BRV2 Branch Via 2      xxxx       BA[m:0]       10  1  PC<=MA[m:0]
//   0111  BRV3 Branch Via 3      xxxx       BA[m:0]       11  1  PC<=MA[m:0]
//   1000  BTH0 Branch T0 High    xxx1  {T0?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1001  BTH1 Branch T1 High    xx1x  {T1?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1010  BTH2 Branch T2 High    x1xx  {T2?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1011  BTH3 Branch T3 High    1xxx  {T2?BA[m:0]:PC+1}  00  1  PC<=MA[m:0]
//   1100  BMW0 BMW with Select 0 xxxx       {BA+MW}       00  1  PC<=MA[m:0]
//   1101  BMW1 BMW with Select 1 xxxx       {BA+MW}       00  1  PC<=MA[m:0]
//   1110  BMW2 BMW with Select 2 xxxx       {BA+MW}       00  1  PC<=MA[m:0]
//   1111  BMW3 BMW with Select 3 xxxx       {BA+MW}       00  1  PC<=MA[m:0]
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_MPC #(
    parameter pAddrWidth = 10,          // Original F9408 => 10-bit Address
    parameter pMPC_Stk   = 1'b0,        // MPC Stack Depth
    parameter pRst_Addrs = 0            // Reset Address
)(
    input   Rst,                        // Module Reset (Synchronous)
    input   Clk,                        // Module Clock
    
    input   Rdy,                        // Rdy - MPC Clock Enable

    input   [3:0] I,                    // Instruction (see description)
    input   [3:0] T,                    // Conditional Test Inputs
    input   [2:0] MW,                   // Multi-way Branch Address Select
    input   [(pAddrWidth-1):0] BA,      // Microprogram Branch Address Field
    
    output  [1:0] Via,                  // Unconditional Branch Address Select
    output  [(pAddrWidth-1):0] MA       // Microprogram Address
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameters
//

localparam pRTS  =  0;  // Return from Subroutine
localparam pBSR  =  1;  // Branch to Subroutine
localparam pFTCH =  2;  // Fetch Next Instruction
localparam pBMW  =  3;  // Multi-way Branch
localparam pBRV0 =  4;  // Branch Via External Branch Address Source #0
localparam pBRV1 =  5;  // Branch Via External Branch Address Source #1
localparam pBRV2 =  6;  // Branch Via External Branch Address Source #2
localparam pBRV3 =  7;  // Branch Via External Branch Address Source #3
localparam pBTH0 =  8;  // Branch if T[0] is Logic 1, else fetch next instr.
localparam pBTH1 =  9;  // Branch if T[1] is Logic 1, else fetch next instr.
localparam pBTH2 = 10;  // Branch if T[2] is Logic 1, else fetch next instr.
localparam pBTH3 = 11;  // Branch if T[3] is Logic 1, else fetch next instr.
localparam pBMW0 = 12;  // Multi-way Branch on MW using selector 0
localparam pBMW1 = 13;  // Multi-way Branch on MW using selector 1
localparam pBMW2 = 14;  // Multi-way Branch on MW using selector 2
localparam pBMW3 = 15;  // Multi-way Branch on MW using selector 3

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     dRst;                             // Reset stretcher
wire    MPC_Rst;                          // Internal MPC Reset signal

wire    [(pAddrWidth - 1):0] Next;        // Output Program Counter Incrementer
wire    [(pAddrWidth - 1):0] MWA;         // Output Multi-way Address Generator
reg     [(pAddrWidth - 1):0] PC_In;       // Input to Program Counter
reg     [(pAddrWidth - 1):0] PC;          // Program Counter

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Implement module reset generator

always @(posedge Clk)
begin
    if(Rst)
        dRst <= #1 1;
    else
        dRst <= #1 0;
end

assign MPC_Rst = (Rst | dRst);

//  Implement Return Stack

reg [(pAddrWidth - 1):0] A;     // 1 Deep LIFO Stack Register

generate
    if(pMPC_Stk) begin                          // Implement 4-Level LIFO Stack
        reg [(pAddrWidth - 1):0] B, C, D;       // 4 Deep LIFO Stack Registers
        
        always @(posedge Clk)
        begin
            if(MPC_Rst)
                {A, B, C, D} <= #1 0;
            else if(Rdy)
                if(I == pBSR)
                    {A, B, C, D} <= #1 {Next, A, B, C};
                else if(I == pRTS)
                    {A, B, C, D} <= #1 {B, C, D, {pAddrWidth{1'b0}}};
        end
    end else begin                              // Implement 1-Level LIFO Stack
        always @(posedge Clk)
        begin
            if(MPC_Rst)
                A <= #1 0;
            else if(Rdy)
                if(I == pBSR)
                    A <= #1 Next;
                else if(I == pRTS)
                    A <= #1 {pAddrWidth{1'b0}};
        end
    end
endgenerate

//  Program Counter Incrementer

assign Next = PC + 1;

//  Generate Multi-way Address

assign MWA = BA + MW;

//  Generate Unconditional Branch Address Select

assign Via = ((I[3:2] == 2'b01) ? I[1:0] : 0);

//  Generate Program Counter Input Signal

always @(*)
begin
    if(MPC_Rst)
        PC_In <= pRst_Addrs;
    else
        case(I)
            4'b0000 : PC_In <= A;                           // pRTS
            4'b0001 : PC_In <= BA;                          // pBSR    
            4'b0010 : PC_In <= Next;                        // pFTCH   
            4'b0011 : PC_In <= MWA;                         // pBMW    
            4'b0100 : PC_In <= BA;                          // pBRV0   
            4'b0101 : PC_In <= BA;                          // pBRV1   
            4'b0110 : PC_In <= BA;                          // pBRV2   
            4'b0111 : PC_In <= BA;                          // pBRV3   
            4'b1000 : PC_In <= (T[0] ? BA : Next);          // pBTH0
            4'b1001 : PC_In <= (T[1] ? BA : Next);          // pBTH1
            4'b1010 : PC_In <= (T[2] ? BA : Next);          // pBTH2
            4'b1011 : PC_In <= (T[3] ? BA : Next);          // pBTH3
            4'b1100 : PC_In <= MWA;                         // pBMW0   
            4'b1101 : PC_In <= MWA;                         // pBMW1   
            4'b1110 : PC_In <= MWA;                         // pBMW2   
            4'b1111 : PC_In <= MWA;                         // pBMW3   
        endcase
end

//  Generate Microprogram Address (Program Counter)

always @(posedge Clk)
begin
    if(MPC_Rst)
        PC <= #1 pRst_Addrs;
    else if(Rdy)
        PC <= #1 PC_In;
end

//  Assign Memory Address Bus

assign MA = PC_In;

endmodule
