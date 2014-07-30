////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2013 by Michael A. Morris, dba M. A. Morris & Associates
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
// Create Date:     09:37:34 06/15/2008 
// Design Name:     Parallel Interface UART 
// Module Name:     ../VerilogCoponentsLib/UART/UART_INT.v
// Project Name:    Verilog Components Library
// Target Devices:  XC3S50A-4VQG100I, XC3S20A-4VQG100I, XC3S700AN-4FFG484I 
// Tool versions:   ISE 10.1i SP3 
//
// Description:
//
//  This module implements the interrupt request logic for the parallel inter-
//  face UART. Interrupt requests are generated for four conditions:
//
//  (1) Transmit FIFO Empty;
//  (2) Transmit FIFO Half Empty;
//  (3) Receive FIFO Half Full;
//  (4) and Receive Timeouts.
//
//  The interrupt flags will be reset/cleared as indicated below except when the
//  rst/clr pulse is coincident with the pulse which would set the flag, the 
//  flag remains set so that the new assertion pulse is not lost. 
//
//  There remains a small probability that the second pulse may be lost. This
//  can be remedied by stretching the setting pulse to a width equal to the un-
//  certainty between the setting and resetting pulses: approximately 4 clock
//  cycles when re1ce modules are used to generate the Clr_Int pulse.
//
// Dependencies: redet.v, fedet.v
//
// Revision History:
//
//  0.01    13L28   MAM     File Created. Implementation copied from that used
//                          for the SSP UART
//
// Additional Comments: 
//
//      The interrupt flags are set and reset under a variety of conditions.
// 
//      iTHE -  Set on the falling edge of Transmit FIFO Half Full (TF_HF) 
//              Rst on Clr_Int.
//
//      iTFE -  Set on the rising edge of the Transmit FIFO Empty Flag (TF_EF)
//              Rst on Clr_Int.
//
//      iRFE -  Set on the rising edge of Receive Error at output of Rx FIFO
//              Rst of Clr_Int.
//
//      iRHF -  Set on the rising edge of Receive FIFO Half Full (RF_HF)
//              Rst on Clr_Int or on the falling edge of RF_HF.
//
//      iRTO -  Set on the rising edge of RTO
//              Rst on Clr_Int.
//
////////////////////////////////////////////////////////////////////////////////

module UART_INT(
    input   Rst,                    // Reset
    input   Clk,                    // Clock
    
    input   TF_HF,                  // Transmit FIFO Half-Full Flag
    input   TF_EF,                  // Transmit FIFO Empty Flag
    input   TxIdle,                 // Transmit Idle Flag
    
    input   RF_FE,                  // Receive FIFO Framing Error Flag
    input   RF_PE,                  // Receive FIFO Parity Error Flag
    input   RF_HF,                  // Receive FIFO Half-Full Flag
    input   RF_EF,                  // Receive FIFO Empty Flag
    input   RTO,                    // Receive Timeout Flag
    
    input   Clr_Int,                // Clear interrupt pulse
    
    output  reg iTHE,               // Interrupt - Tx FIFO Half Empty
    output  reg iTEF,               // Interrupt - Tx FIFO Empty
    output  reg iTEM,               // Interrupt - Tx SR Empty

    output  reg iRFE,               // Interrupt - Rx FIFO Framing Error
    output  reg iRPE,               // Interrupt - Rx FIFO Parity Error
    output  reg iRHF,               // Interrupt - Rx FIFO Half Full
    output  reg iRTO                // Interrupt - Receive Time Out Detected
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Signal Declarations
//

    wire    reTF_EF, feTF_HF, reTxIdle;
    wire    reRF_FE, reRF_PE;
    wire    reRF_HF, feRF_HF, reRF_EF;
    wire    reRTO;
    
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

redet   RE1 (.rst(Rst), .clk(Clk), .din(TF_EF),  .pls(reTF_EF));
fedet   FE1 (.rst(Rst), .clk(Clk), .din(TF_HF),  .pls(feTF_HF));
redet   RE2 (.rst(Rst), .clk(Clk), .din(TxIdle), .pls(reTxIdle));

redet   RE3 (.rst(Rst), .clk(Clk), .din(RF_FE),  .pls(reRF_FE));
redet   RE4 (.rst(Rst), .clk(Clk), .din(RF_PE),  .pls(reRF_PE));
redet   RE5 (.rst(Rst), .clk(Clk), .din(RF_HF),  .pls(reRF_HF));
fedet   FE2 (.rst(Rst), .clk(Clk), .din(RF_HF),  .pls(feRF_HF));
redet   RE6 (.rst(Rst), .clk(Clk), .din(RF_EF),  .pls(reRF_EF));

redet   RE7 (.rst(Rst), .clk(Clk), .din(RTO),    .pls(reRTO));

////////////////////////////////////////////////////////////////////////////////
//
//  Transmit FIFO Half Empty Interrupt Flag
//

assign Rst_iTHE = Rst | (iTHE & Clr_Int);

always @(posedge Clk or posedge feTF_HF)
begin
    if(feTF_HF)
        iTHE <= #1 1;
    else if(Rst_iTHE)
        iTHE <= #1 feTF_HF;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Transmit FIFO Empty Interrupt Flag
//

assign Rst_iTEF = Rst | (iTEF & Clr_Int);

always @(posedge Clk or posedge reTF_EF)
begin
    if(reTF_EF)
        iTEF <= #1 1;
    else if(Rst_iTEF)
        iTEF <= #1 reTF_EF;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Transmit SR Empty Interrupt Flag
//

assign Rst_iTEM = Rst | (iTEM & Clr_Int);

always @(posedge Clk or posedge reTxIdle)
begin
    if(reTxIdle)
        iTEM <= #1 1;
    else if(Rst_iTEM)
        iTEM <= #1 reTxIdle;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Receive FIFO Framing Error Interrupt Flag
//

assign Rst_iRFE = Rst | (iRFE & Clr_Int);

always @(posedge Clk or posedge reRF_FE)
begin
    if(reRF_FE)
        iRFE <= #1 1;
    else if(Rst_iRFE)
        iRFE <= #1 (RF_FE | reRF_FE);
end

////////////////////////////////////////////////////////////////////////////////
//
//  Receive FIFO Parity Error Interrupt Flag
//

assign Rst_iRPE = Rst | (iRPE & Clr_Int);

always @(posedge Clk or posedge reRF_PE)
begin
    if(reRF_PE)
        iRPE <= #1 1;
    else if(Rst_iRPE)
        iRPE <= #1 (RF_PE | reRF_PE);
end

////////////////////////////////////////////////////////////////////////////////
//
//  Receive Data Available Interrupt Flag
//

assign Rst_iRHF = Rst | (iRHF & (Clr_Int | feRF_HF | reRF_EF));

always @(posedge Clk or posedge reRF_HF)
begin
    if(reRF_HF)
        iRHF <= #1 1;
    else if(Rst_iRHF)
        iRHF <= #1 reRF_HF;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Receive Timeout Interrupt Flag
//

assign Rst_iRTO = Rst | (iRTO & Clr_Int);

always @(posedge Clk or posedge reRTO)
begin
    if(reRTO)
        iRTO <= #1 1;
    else if(Rst_iRTO)
        iRTO <= #1 reRTO;
end

endmodule
