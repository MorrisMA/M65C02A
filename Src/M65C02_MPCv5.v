////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2013-2014 by Michael A. Morris, dba M. A. Morris & Associates
//
//  All rights reserved. The source code contained herein is publicly released
//  under the terms and conditions of the GNU Lesser Public License. No part of
//  this source code may be reproduced or transmitted in any form or by any
//  means, electronic or mechanical, including photocopying, recording, or any
//  information storage and retrieval system in violation of the license under
//  which the source code is released.
//
//  The source code contained herein is free; it may be redistributed and/or
//  modified in accordance with the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either version 2.1 of
//  the GNU Lesser General Public License, or any later version.
//
//  The source code contained herein is freely released WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
//  PARTICULAR PURPOSE. (Refer to the GNU Lesser General Public License for
//  more details.)
//
//  A copy of the GNU Lesser General Public License should have been received
//  along with the source code contained herein; if not, a copy can be obtained
//  by writing to:
//
//  Free Software Foundation, Inc.
//  51 Franklin Street, Fifth Floor
//  Boston, MA  02110-1301 USA
//
//  Further, no use of this source code is permitted in any form or means
//  without inclusion of this banner prominently in any derived works.
//
//  Michael A. Morris
//  Huntsville, AL
//
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates
// Engineer:        Michael A. Morris
// 
// Create Date:     12:02:40 10/28/2012 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_MPCv5.v
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
// The instruction encodings of the F9408, as provided in "Principles of Firm-
// ware Engineering in Microprogram Control" by Michael Andrews. The instruc-
// tion set and operation map for the implementation is given below:
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
//  1.00    13I14   MAM     Removed embedded microcycle controller from M65C02_
//                          MPCv3.v. Added parameter to control the implementa-
//                          tion of a one level or a four level return stack.
//
// Additional Comments: 
//
//  The Version 5 Microprogram Controller (M65C02_MPCv5) is based on the Fair-
//  child F9408 MPC. It extends that microprogram controller by incorporating 
//  an microcycle enable input which allows each microcycle to be controlled by
//  external logic.
//
//  The purpose of these extensions is to allow easy implementation of a varia-
//  ble length microprogram cycle, i.e. microcycle. In turn, this simplifies the
//  implementation of microprogrammed state machines which interface to synchro-
//  nous memories found in most FPGAs, or to external synchronous/asynchronous
//  memories.
//
//  When a microprogrammed state machine interfaces to a synchronous memory,
//  there is a one cycle delay (or more) between the presentation of the address
//  and the output of the data at that address. In many instances, the micro-
//  program is unable to perform any useful work during the first cycle. Thus,
//  the microprogram must perform an explicit delay operation, which generally
//  requires a state to be added to every read of these memories. If there are
//  a significant number of these read operations in the microprogram, then
//  there is an opportunity for the microprogram to be incorrectly programmed
//  when one or more of the delay cycles are not included in the microprogram.
//  Isolating the resulting fault in the state machine may be difficult.
//
//  To avoid errors of this type, microcycles which read from or write to 
//  devices, such as memories, can be automatically extended explicitly by a
//  microprogram field or external logic. Using this type of facility reduces
//  the number of states required to interface a microprogrammed state machine
//  to these types of devices. It also makes the microprogram less tedious to
//  develop and improves overall productivity, which is a prime reason for
//  choosing a microprogrammed approach for developing complex state machines.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_MPCv5 #(
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
localparam pBTL0 = 12;  // Branch if T[0] is Logic 0, else fetch next instr.
localparam pBTL1 = 13;  // Branch if T[1] is Logic 0, else fetch next instr.
localparam pBTL2 = 14;  // Branch if T[2] is Logic 0, else fetch next instr.
localparam pBTL3 = 15;  // Branch if T[3] is Logic 0, else fetch next instr.

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     dRst;                             // Reset stretcher
wire    MPC_Rst;                          // Internal MPC Reset signal

wire    [(pAddrWidth - 1):0] Next;        // Output Program Counter Incrementer
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

reg [(pAddrWidth - 1):0] A;                 // LIFO Stack Top-Of-Stack Reg

generate
    if(pMPC_Stk) begin  //  Implement 4-Level LIFO Stack
        reg [(pAddrWidth - 1):0] B, C, D;   // LIFO Stack Extra Registers
        
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
    end else begin      //  Implement 1-Level LIFO Stack
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

//  Generate Unconditional Branch Address Select

assign Via = ((I[3:2] == 1) ? I[1:0] : 0);       

//  Generate Program Counter Input Signal

always @(*)
begin
    if(MPC_Rst)
        PC_In <= pRst_Addrs;
    else
        case(I)
            4'b0000 : PC_In <= A;                               // pRTS
            4'b0001 : PC_In <= BA;                              // pBSR    
            4'b0010 : PC_In <= Next;                            // pFTCH   
            4'b0011 : PC_In <= {BA[(pAddrWidth - 1):3], MW};    // pBMW    
            4'b0100 : PC_In <= BA;                              // pBRV0   
            4'b0101 : PC_In <= BA;                              // pBRV1   
            4'b0110 : PC_In <= BA;                              // pBRV2   
            4'b0111 : PC_In <= BA;                              // pBRV3   
            4'b1000 : PC_In <= (T[0] ? BA   : Next);            // pBTH0   
            4'b1001 : PC_In <= (T[1] ? BA   : Next);            // pBTH1   
            4'b1010 : PC_In <= (T[2] ? BA   : Next);            // pBTH2   
            4'b1011 : PC_In <= (T[3] ? BA   : Next);            // pBTH3   
            4'b1100 : PC_In <= (T[0] ? Next : BA  );            // pBTL0   
            4'b1101 : PC_In <= (T[1] ? Next : BA  );            // pBTL1   
            4'b1110 : PC_In <= (T[2] ? Next : BA  );            // pBTL2   
            4'b1111 : PC_In <= (T[3] ? Next : BA  );            // pBTL3   
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
