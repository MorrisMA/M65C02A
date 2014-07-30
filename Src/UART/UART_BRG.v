////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2008-2013 by Michael A. Morris, dba M. A. Morris & Associates
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
// Create Date:     13:28:23 05/10/2008 
// Design Name:     Parallel Interface UART 
// Module Name:     ../VerilogCoponentsLib/UART/UART_BRG.v
// Project Name:    Verilog Components Library
// Target Devices:  XC3S50A-4VQG100I, XC3S20A-4VQG100I, XC3S700AN-4FFG484I 
// Tool versions:   ISE 10.1i SP3 
//
// Description:
//
//  This module implements a simple 16-bit Baud Rate Generator for the simple
//  parallel interface UART.
//
// Dependencies: 
//
// Revision History:
//
//  0.01    08E10   MAM     File Created
//
//  1.00    08E10   MAM     Initial Release
//
//  1.10    08E13   MAM     Changed interface so Prescaler and Divider values
//                          passed directly in by removing Baud Rate ROM.
//
//  1.11    08E14   MAM     Reduced width of divider from 10 to 8 bits.
//
//  1.20    08E15   MAM     Changed the structure of the PSCntr and Divider
//                          to use a multiplxer on the input to load or count
//                          which results in a more efficient implementation.
//                          Added a registered TC on the PSCntr which functions
//                          to break the combinatorial logic chains and speed
//                          counter implementations.
//
//  1.30    08G26   MAM     Corrected initial condition of the PSCntr, which
//                          caused the prescaler to always divide by two. 
//                          Removed FF in PSCntr TC path to remove the divide
//                          by two issue. CE_16x output remains as registered.
//
//  2.00    11B06   MAM     Converted to Verilog 2001.
//
//  2.10    13L28   MAM     Module now part of the Parallel UART project. Modi-
//                          fied to support a single 16-bit divider than the
//                          4-bit pre-scaler and 8-bit divider used in the SSP
//                          UART version of the module. Added parameter to set
//                          default divider value so baud rate set on reset. A
//                          load input, Ld, added to load baud rate divider.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module UART_BRG #(
    parameter pDiv_Default = 16'h01DF   // Divider Default: 9600@73.7280MHz
)(
    input   Rst,
    input   Clk,

    input   Ld,             // Baud Rate Generator Load
    input   [15:0] Div,     // Baud Rate Generator Divider Load Value
    
    output  reg CE_16x      // Clock Enable Output - 16x Baud Rate Output 
);

////////////////////////////////////////////////////////////////////////////////    
//
//  Local Signal Declarations
//

reg     [15:0] Divider;                 // BRG Divider
wire    TC_Divider;                     // BRG Divider TC
    
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

// BRG Divider

always @(posedge Clk)
begin
    if(Rst)
        Divider <= #1 pDiv_Default;
    else if(Ld | TC_Divider)
        Divider <= #1 Div;
    else
        Divider <= #1 (Divider - 1);
end

assign TC_Divider = ~|Divider;

// Output 16x Bit Clock/CE

always @(posedge Clk)
begin    
    if(Rst)
        CE_16x <= #1 1'b0;
    else
        CE_16x <= #1 TC_Divider;
end

endmodule
