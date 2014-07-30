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
// Create Date:     14:50:14 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_Add 
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
//  0.01    13I15   MAM     Created module based on the M65C02A_Add.v module.
//
//  1.00    13K09   MAM     Reverted BCD addition/subtraction to single-cycle
//                          operation -- Removed pipeline registers for BCD
//                          results. Thus, binary addition/subtraction performed
//                          during first half of the cycle, and BCD adjustment
//                          performed during the second half of the cycle.
//
//  1.01    14F21   MAM     Put the module back to the state indicated for 1.00.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_Add(
    input   Rst,                // Module Reset
    input   Clk,                // System Clock
    
    input   En_AU,              // Enable Binary Adder Functional Unit
    input   En_DU,              // Enable Decimal Adder Functional Unit
    input   Op,                 // Adder Operation: 0 - Addition; 1 - Subtract
    
    input   [7:0] Q,            // Adder Input Q
    input   [7:0] R,            // Adder Input R
    input   Ci,                 // Adder Carry In
    
    output  [8:0] Out,
    output  OV,
    output  Val
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wire    [7:0] M;                // Right Operand: if(Op) M = ~R, else M = R

reg     rEn_DU;                 // Pipelined/Registered Control Signals
reg     rOp;

wire    [7:0] S;                // Intermediate Binary Sum: S <= A + B + Ci
wire    C7, C6, C3;             // Sum Carry Out from Bitn 

reg     [7:0] rS;               // BCD mode first stage binary sum register
reg     rC7, rC3;               // BCD mode first stage carry registers

reg     MSN_GT9, MSN_GT8, LSN_GT9;  // Digit value comparator signals

reg     [1:0] DA;               // Decimal Adjust Controls
reg     [7:0] Adj;              // Second stage BCD adjusted registers

reg     [7:0] BCD;              // Second stage adjusted Sum
reg     BCD_Co, BCD_OV, BCD_Val;

wor     [8:0] Sum;
wor     V;
wor     Valid;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Capture Input Control Signals

assign Rst_Add = (Rst | (~(En_AU ^ En_DU))); 

always @(negedge Clk or posedge Rst_Add)
begin
    if(Rst_Add)
        {rEn_DU, rOp} <= #1 0;
    else
        {rEn_DU, rOp} <= #1 {En_DU, Op};
end        

//  Adder First Stage - Combinatorial; Binary Sums and Carries

assign M = ((Op) ? ~R : R);

assign {C3, S[3:0]} = Q[3:0] + M[3:0] + Ci;
assign {C6, S[6:4]} = Q[6:4] + M[6:4] + C3;
assign {C7, S[7]  } = Q[7]   + M[7]   + C6;

//  Adder Binary Stage - Output (Binary Sum and Carry)

assign {Valid, V, Sum} = ((En_AU) ? {En_AU, (C7 ^ C6), {C7, S}} : 0);

//  Propagate binary sum and carry to Adder Second Stage - Decimal Adder

always @(negedge Clk or posedge Rst_Add)
begin
    if(Rst_Add)
        {rC7, rC3, rS} <= #1 0;
    else if(En_DU)
        {rC7, rC3, rS} <= #1 {C7, C3, S};
end

//  Generate Digit/Nibble Value Comparators

always @(*)
begin
    case(rS[7:4])
        4'b0000 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0001 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0010 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0011 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0100 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0101 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0110 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b0111 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b1000 : {MSN_GT9, MSN_GT8} <= 2'b00;
        4'b1001 : {MSN_GT9, MSN_GT8} <= 2'b01;
        4'b1010 : {MSN_GT9, MSN_GT8} <= 2'b11;
        4'b1011 : {MSN_GT9, MSN_GT8} <= 2'b11;
        4'b1100 : {MSN_GT9, MSN_GT8} <= 2'b11;
        4'b1101 : {MSN_GT9, MSN_GT8} <= 2'b11;
        4'b1110 : {MSN_GT9, MSN_GT8} <= 2'b11;
        4'b1111 : {MSN_GT9, MSN_GT8} <= 2'b11;
    endcase
end

always @(*)
begin
    case(rS[3:0])
        4'b0000 : LSN_GT9 <= 0;
        4'b0001 : LSN_GT9 <= 0;
        4'b0010 : LSN_GT9 <= 0;
        4'b0011 : LSN_GT9 <= 0;
        4'b0100 : LSN_GT9 <= 0;
        4'b0101 : LSN_GT9 <= 0;
        4'b0110 : LSN_GT9 <= 0;
        4'b0111 : LSN_GT9 <= 0;
        4'b1000 : LSN_GT9 <= 0;
        4'b1001 : LSN_GT9 <= 0;
        4'b1010 : LSN_GT9 <= 1;
        4'b1011 : LSN_GT9 <= 1;
        4'b1100 : LSN_GT9 <= 1;
        4'b1101 : LSN_GT9 <= 1;
        4'b1110 : LSN_GT9 <= 1;
        4'b1111 : LSN_GT9 <= 1;
    endcase
end

//  Adder Decimal Stage - Decimal Adjustment Factor

always @(*)
begin
    if(rEn_DU)  // Generate Decimal Mode Digit Adjust Signals
        if(rOp) begin   // SBC Operations
            DA[1] <= ~rC7 | (~rC3 & MSN_GT9);
            DA[0] <= ~rC3;
        end else begin  // ADC Operations
            DA[1] <= (rC7 | MSN_GT9 | (MSN_GT8 & ~rC3 & LSN_GT9));
            DA[0] <=  rC3 | LSN_GT9;
        end
    else
        DA <= 0;
end

//  Adder Decimal Stage - Decimal Adjustment Factor Selection

always @(*)
begin
     case({rOp, DA})
        3'b000 : Adj <= 8'h00;  // 0
        3'b001 : Adj <= 8'h06;  // ±06 BCD
        3'b010 : Adj <= 8'h60;  // ±60 BCD
        3'b011 : Adj <= 8'h66;  // ±66 BCD
        3'b100 : Adj <= 8'h00;  // 0
        3'b101 : Adj <= 8'hFA;  // ±06 BCD
        3'b110 : Adj <= 8'hA0;  // ±60 BCD
        3'b111 : Adj <= 8'h9A;  // ±66 BCD
    endcase
end

//  Adder Second Stage Result Register

always @(*)
begin
    {BCD, BCD_Val, BCD_OV, BCD_Co} <= {(rS[7:0] + Adj),
                                       rEn_DU,
                                       DA[1],
                                       (rOp ^ DA[1])    };
end

//  Adder Decimal Stage - Output

assign {Valid, V, Sum} = ((BCD_Val) ? {BCD_Val, BCD_OV, {BCD_Co, BCD}} : 0);

//  Adder Output Valid and Overflow Outputs

assign Out = Sum;
assign OV  = V;
assign Val = Valid;

endmodule
