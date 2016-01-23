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
// Module Name:     M65C02A_Add.v
// Project Name:    C:\XProjects\ISE10.1i\M6502A
// Target Devices:  Generic SRAM-based FPGA
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module provides single cycle operation for 8-bit binary and BCD arith-
//  metic, and single cycle operation for 16-bit binary arithmetic. The module
//  supports the following instructions: ADC/SBC/INC/DEC/CMP.
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

module M65C02A_Add (
    input   Rst,                // Module Reset
    input   Clk,                // System Clock
    
    input   SIZ,                // Operation Size
    
    input   En_AU,              // Enable Binary Adder Functional Unit
    input   En_DU,              // Enable Decimal Adder Functional Unit
    input   Op,                 // Adder Operation: 0 - Addition; 1 - Subtract
    
    input   [15:0] Q,           // Adder Input Q
    input   [15:0] R,           // Adder Input R
    input   Ci,                 // Adder Carry In
    
    output  [16:0] Out,
    output  OV,
    output  Val
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wire    [15:0] M;               // Right Operand: if(Op) M = ~R, else M = R

reg     rEn_DU;                 // Pipelined/Registered Control Signals
reg     rOp;

wire    [15:0] S;               // Intermediate Binary Sum: S <= A + B + Ci
wire    C15, C14, C7, C6, C3;   // Sum Carry Out from Bit n

wire    [15:0] AU_Sum;          // Binary Adder Sum
wire    AU_OV, AU_Co;           // Binary Adder Overflow and Carry Out

reg     [ 7:0] rS;              // BCD mode first stage binary sum register
reg     rC7, rC3;               // BCD mode first stage carry registers

reg     MSN_GT9, MSN_GT8, LSN_GT9;  // Digit value comparator signals

reg     [1:0] DA;               // Decimal Adjust Controls
reg     [7:0] Adj;              // Second stage BCD adjusted registers

reg     [7:0] BCD;              // Second stage adjusted Sum
reg     BCD_Co, BCD_OV, BCD_Val;

wor     [16:0] Sum;
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

assign {C3,  S[ 3:0]} = Q[ 3:0] + M[ 3:0] + Ci;
assign {C6,  S[ 6:4]} = Q[ 6:4] + M[ 6:4] + C3;
assign {C7,  S[ 7]  } = Q[ 7]   + M[ 7]   + C6;
assign {C14, S[14:8]} = Q[14:8] + M[14:8] + C7;
assign {C15, S[15]  } = Q[15]   + M[15]   + C14;

//  Adder Binary Stage - Output (Binary Sum and Carry)

assign AU_OV  = ((SIZ) ? C15 ^ C14 : C7 ^ C6        );
assign AU_Co  = ((SIZ) ? C15       : C7             );
assign AU_Sum = ((SIZ) ? S         : {8'h00, S[7:0]}); 

assign {Valid, V, Sum} = ((En_AU) ? {En_AU, AU_OV, {AU_Co, AU_Sum}} : 0);

//  Propagate binary sum and carry to Adder Second Stage - Decimal Adder

always @(negedge Clk or posedge Rst_Add)
begin
    if(Rst_Add)
        {rC7, rC3, rS} <= #1 0;
    else if(En_DU)
        {rC7, rC3, rS} <= #1 {C7, C3, S[7:0]};
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

assign {Valid, V, Sum} = ((BCD_Val) ? {BCD_Val, BCD_OV, {BCD_Co, {8'b0, BCD}}}
                                    : 0);

//  Adder Output Valid and Overflow Outputs

assign Out = Sum;
assign OV  = V;
assign Val = Valid;

endmodule
