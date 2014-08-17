////////////////////////////////////////////////////////////////////////////////
//
//  Receive Timeout module for UART.
// 
//  Copyright 2008-2014 by Michael A. Morris, dba M. A. Morris & Associates
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

///////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates
// Engineer:        Michael A. Morris
//
// Create Date:     06:50:40 06/14/2008 
// Design Name:     Synchronous Serial Peripheral (SSP) Interface UART 
// Module Name:     ../VerilogCoponentsLib/SSP_UART/UART_RTO.v
// Project Name:    Verilog Components Library
// Target Devices:  XC3S50A-4VQG100I, XC3S20A-4VQG100I, XC3S700AN-4FFG484I 
// Tool versions:   ISE 10.1i SP3 
//
// Description: This module implements a receive timeout timer which sets a
//              flag to indicate that the timeout occured. The flag is cleared
//              by reset or by the reading of the Receive Holding Register.
//
//              The Receive Timeout Val should be set by the client module to a
//              value which corresponds to the desired timeout value in terms
//              of the number of bit times. This module scales the 16x bit rate
//              clock enable by 16 to set the internal bit rate clock enable.
//              The client module provide the character frame length and the 
//              number of characters to delay. The character frame length
//              varies as a function of the frame format, e.g. 8N1, 8E2, or
//              7O1. The number of characters to delay for the timeout is a
//              fixed number of character periods, e.g. 3, in most cases. The
//              module allows these values to be independently specified.
//              
//
// Dependencies:    none
//
// Revision History:
//
//  0.01    08E14   MAM     File Created
//
//  1.00    08E14   MAM     Initial Release
//
//  1.10    08E14   MAM     Changed the input parameters to have two delay 
//                          parameters: CCntVal - character length; and
//                                      RTOVal  - # characters to delay.
//                          The result is a faster implementation, and one
//                          the RTOVal can usually be set as a constant.
//
//  1.11    08E15   MAM     Changed Module Name
//
//  2.00    11B06   MAM     Converted to Verilog 2001.
//
// Additional Comments:
//
///////////////////////////////////////////////////////////////////////////////

module UART_RTO(
    input   Rst,
    input   Clk,
    
    input   CE_16x,

    input   WE_RHR,
    input   RE_RHR,

    input   [3:0] CCntVal, 
    input   [3:0] RTOVal,

    output  reg RcvTimeout
);

///////////////////////////////////////////////////////////////////////////////
//
//  Local Signal Declarations
//

    wire    Clr_RTOArm;
    reg     RTOArm;
    
    wire    Clr_BCnt;
    reg     [3:0] BCnt;
    reg     TC_BCnt;
    
    wire    Clr_CCnt;
    reg     [3:0] CCnt;
    reg     TC_CCnt;
    
    wire    Clr_RTOCnt, CE_RTOCnt;
    reg     [3:0] RTOCnt;
    reg     TC_RTOCnt;
    
    wire    Clr_RTO, CE_RTO;

///////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//
//  RTO Arm FF
//      Armed with each received character, cleared by Rst, RE_RHR, or timeout
//

assign Clr_RTOArm = RE_RHR | CE_RTO;

always @(posedge Clk)
begin
    if(Rst)
        RTOArm <= #1 0;
    else if(Clr_RTOArm)
        RTOArm <= #1 0;
    else if(WE_RHR)
        RTOArm <= #1 1;
end

///////////////////////////////////////////////////////////////////////////////
//
//  Bit Rate Divider
//      Held in reset until RTO Armed or if Rst asserted
//

assign Clr_BCnt = Rst | WE_RHR | Clr_RTOArm | ~RTOArm;

always @(posedge Clk)
begin
    if(Clr_BCnt)
        BCnt <= #1 0;
    else if(CE_16x)
        BCnt <= #1 BCnt + 1;
end

always @(posedge Clk)
begin
    if(Clr_BCnt)
        TC_BCnt <= #1 0;
    else if(CE_16x)
        TC_BCnt <= #1 (BCnt == 4'b1110);
end

///////////////////////////////////////////////////////////////////////////////
//
//  Character Frame Divider
//      Held in reset until RTO Armed or if Rst asserted
//

assign Clr_CCnt = Clr_BCnt;
assign CE_CCnt = CE_16x & TC_BCnt;

always @(posedge Clk)
begin
    if(Clr_CCnt)
        CCnt <= #1 CCntVal;
    else if(CE_CCnt)
        CCnt <= #1 ((TC_CCnt) ? CCntVal : CCnt - 1);
end

always @(posedge Clk)
begin
    if(Clr_CCnt)
        TC_CCnt <= #1 0;
    else if(CE_16x)
        TC_CCnt <= #1 (CCnt == 0);
end

///////////////////////////////////////////////////////////////////////////////   
//
//  Receive Timeout Counter
//      Held in reset until RTO Armed or if Rst asserted
//      Counts bit periods when RTO Armed

assign Clr_RTOCnt = Clr_BCnt;
assign CE_RTOCnt  = CE_16x & TC_BCnt & TC_CCnt;

always @(posedge Clk)
begin
    if(Clr_RTOCnt)
        RTOCnt <= #1 RTOVal;
    else if(CE_RTOCnt)
        RTOCnt <= #1 ((TC_RTOCnt) ? RTOVal : RTOCnt - 1);
end

always @(posedge Clk)
begin
    if(Clr_RTOCnt)
        TC_RTOCnt <= #1 0;
    else if(CE_16x)
        TC_RTOCnt <= #1 (RTOCnt == 0);
end

///////////////////////////////////////////////////////////////////////////////
//
//  Receive Timeout Latch
//      Cleared by Rst or any read of the RHR
//      Set by RTOCnt terminal count

assign Clr_RTO = RE_RHR;
assign CE_RTO  = CE_16x & TC_BCnt & TC_CCnt & TC_RTOCnt;

always @(posedge Clk)
begin
    if(Rst)
        RcvTimeout <= #1 0;
    else if(Clr_RTO)
        RcvTimeout <= #1 0;
    else if(CE_RTO)
        RcvTimeout <= #1 1;
end

endmodule
