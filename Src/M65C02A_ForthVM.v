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
// Module Name:     M65C02A_ForthVM.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This module provides the functions needed to implement a FORTH VM supporting
//  the Direct Threaded Code (DTC) or the Indirect Threaded Code (ITC) models.
//  The module provides 16-bit registers for the Interpretive Pointer (IP) and
//  Working (W) register.
//
//  The IP and W registers may be loaded from memory (M) or incremented by 1
/// or 2. IP may also be loaded with the value in the W register.
//
//  The operation of this module are controlled by the En signal, the 3-bit
//  microprogram control field (VMCntl), the IND prefix flag, PHI signal and the
//  PLI signal. The IND prefix flag determines if the IP or W is the destination
//  or source register for the PLI and PHI instructions. The Enable signal, En,
//  determines when the VMCntl field is valid.
//
// Dependencies:
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release
//
//  1.10    14F29   MAM     Added an enable signal in order to make use of the
//                          3-bit VMCntl microprogram field for implementing 
//                          the M65C02A MOV instruction. The enable is combined
//                          with the Rdy signal to qualify updates to the IP and
//                          W registers. Otherwise no other changes made to the
//                          module.
//
//  1.20    15G23   MAM     Corrected the multiplexer and select controls. Added
//                          another term which uses IND to select W rather than 
//                          IP, and included complementary terms in the SelI_S
//                          and the SelW_S signals. Now INI and IND INI work as
//                          expected.
//
//  1.30    15K25   MAM     Added DUP and A to interface, and added additional
//                          internal multiplexer in order to enable IP <= A. 
//                          Rename and renumbered some of the local parameters
//                          to add IP <= A as an operation replacing the unused
//                          IP += 2 operation. Renamed internal busses A and B
//                          to X and Y in order to bring in A.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_ForthVM (
    input   Rst,
    input   Clk,
    
    input   En,
    
    input   Rdy,

    input   IND,                // Enables IP <= A or W <= M 
    input   PHI,                // if(IND) T  <= W else T  <= IP
    input   PLI,                // if(IND) W  <= M else IP <= M
    input   DUP,                // Enables IP <= A
    
    input   [ 2:0] VMCntl, 

    input   [15:0] A,
    input   [15:0] M,
    output  [15:0] T,

    output  reg [15:0] IP,
    output  reg [15:0] W
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameters
//

localparam pSTW = 3'b001;   // T  <= W;  
localparam pIP1 = 3'b010;   // IP <= IP + 1;
localparam pWP2 = 3'b011;   // W  <= W  + 2; 
localparam pM2W = 3'b100;   // W  <= M;
localparam pM2I = 3'b101;   // IP <= M;
localparam pA2I = 3'b110;   // IP <= A;
localparam pW2I = 3'b111;   // IP <= W;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

wire    [15:0] X, Y, S;
wor     [15:0] iIP, iW;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Incrementer

assign X =(((VMCntl == pWP2) | ((VMCntl == pIP1) &  IND)) ? W : IP);
assign Y = ((VMCntl == pWP2) ? 16'h2 : 16'h1);
assign S = X + Y;

assign SelI_S = ((VMCntl == pIP1) & ~IND        );
assign SelI_M = ((VMCntl == pM2I) & ~(PLI & IND));
assign SelI_A = ((VMCntl == pA2I) |  (IND & DUP));
assign SelI_W = ((VMCntl == pW2I)               );

assign SelW_S = ((VMCntl == pWP2) | (VMCntl == pIP1) &  IND       );
assign SelW_M = ((VMCntl == pM2W) | (VMCntl == pM2I) & (IND & PLI));

assign iIP = ((SelI_S) ? S : 0);
assign iIP = ((SelI_M) ? M : 0);
assign iIP = ((SelI_A) ? A : 0);
assign iIP = ((SelI_W) ? W : 0);

assign iW  = ((SelW_S) ? S : 0);
assign iW  = ((SelW_M) ? M : 0);

always @(posedge Clk)
begin
    if(Rst)
        {IP, W} <= #1 0;
    else if(En & Rdy) begin
        IP <= #1 ((SelI_S | SelI_M | SelI_W) ? iIP : IP);
        W  <= #1 ((SelW_S | SelW_M         ) ? iW  : W );
    end
end

assign T = (((VMCntl == pSTW) | (PHI & IND)) ? W : IP);

endmodule
