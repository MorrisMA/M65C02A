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
// Create Date:     12:06:18 08/18/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_IntHndlr.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A
// Target Devices:  SRAM-based FPGAs: XC3S50A-xVQ100I, XC3S200A-xVQ100I 
// Tool versions:   Xilinx ISE 10.1i SP3
// 
// Description: 
//
//  This module implements a simple interrupt handler for the M65C02 soft-core
//  microprocessor. It accepts external active low inputs for Non-Maskable
//  Interrupt request (nNMI) and maskable Interrupt ReQuest (nIRQ). It synchro-
//  nizes both inputs to the internal system clock (Clk), and generates internal
//  signals NMI and IRQ. NMI is falling edge sensitive, and IRQ is active low
//  level sensitive. The module also accepts the core's mode output (Mode) and
//  generates an internal BReaK software trap request (BRK).
//
//  The non-maskable interrupt request, nNMI, has priority, followed by BRK, and
//  finally nIRQ. The core, from the I bit in the processor register, provides a
//  mask that prevents the generation of the internal IRQ signal.
//
//  Vectors for each of the four interrupt/trap sources are set using para-
//  meters. The current implementation aims to maintain compatibility with the
//  WDC W65C02S processor, so IRQ and BRK share the same vector. A quick edit
//  of the parameters allows an independent vector location to be added for BRK.
//  Similarly, the vectors for any of the interrupt/trap sources can be moved
//  to any location in the memory space, if W65C02S compatibility is not desired
//  or required.
//
// Dependencies:    redet.v 
//
// Revision:
//
//  0.01    13H18   MAM     File Created
//
//  1.00    13L22   MAM     Modified for the M65C02Duo CPU.
//
//  2.00    14G06   MAM     Modified to support the M65C02A Mon/IO page concept.
//                          Added support for additional maskable interrupt re-
//                          quests. RQST[7] is the highest priority interrupt of
//                          for the new maskable interrupts. Also added support
//                          for the COP, INV, and SYS (XCE) traps, and an MMU
//                          cycle ABoRT trap (highest priority interrupt/trap
//                          after RST).
// 
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_IntHndlrV2 #(
    parameter pVec_RST   = 16'hFFFC,    // Highest Priority
    parameter pVec_ABRT  = 16'hFFE0,    // (Not Currently Supported - set to 0)
    parameter pVec_NMI   = 16'hFFFA,    //
    parameter pVec_INV   = 16'hFFE2,    //
    parameter pVec_SYS   = 16'hFFE4,    //
    parameter pVec_IRQ   = 16'hFFFE,    //
    parameter pVec_RQST7 = 16'hFFF6,    //
    parameter pVec_RQST6 = 16'hFFF4,    //
    parameter pVec_RQST5 = 16'hFFF2,    //
    parameter pVec_RQST4 = 16'hFFF0,    //
    parameter pVec_RQST3 = 16'hFFEE,    //
    parameter pVec_RQST2 = 16'hFFEC,    //
    parameter pVec_RQST1 = 16'hFFEA,    //
    parameter pVec_RQST0 = 16'hFFE8,    //
    parameter pVec_BRK   = 16'hFFFE,    //
    parameter pVec_COP   = 16'hFFE6     // Lowest Priority
)(
    input   Rst,                // Highest Priority Interrupt Source            
    input   Clk,
    
    input   Rdy,                // Microcycle Ready Input
    
    input   ABRT,               // ABoRT MMU trap
    input   NMI,                // Non-Maskable Interrupt Request (6502 NMI)
    input   INV,                // Invalid Instruction Trap
    input   SYS,                // System Call Trap
    input   IRQ,                // Maskable Interrupt Request (6502 IRQ)
    input   [7:0] RQST,         // Maskable Interrupt Requests (RQST[7] highest)
    input   BRK,                // BReaK instruction trap
    input   COP,                // COProcessor instruction trap
    
    input   IRQ_Msk,            // Interrupt Request Mask
    input   LE_Int,             // Latch/hold Int/Vector until RE of VP 
    input   VP,                 // Vector Pull - asserted during Vector read 
    
    output  reg Int,            // Interrupt request
    output  reg [15:0] Vector   // Vector address for interrupt/trap source
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Declarations
//

wire    RE_NMI;                 // Rising Edge NMI (multi-stage synchronizer)
reg     rNMI;                   // Registered/latched NMI (held until serviced)

wire    iIRQ;                   // Masked internal IRQ
wire    [7:0] iRQST;            // Masked internal RQST

reg     rVP;                    // registered/delayed VP
wire    RE_VP;                  // Rising Edge of Vector Pull, deassert Hold
reg     Hold;                   // When asserted, Hold Int/Vector

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Perform rising edge detection on the external non-maskable interrupt input

redet   RE1 (
            .rst(Rst), 
            .clk(Clk), 
            .din(NMI), 
            .pls(RE_NMI)
        );
        
//  Detect the falling edge of VP to re-enable Int/Vector

always @(posedge Clk or posedge Rst)
begin
    if(Rst)
        rVP <= 0;
    else if(Rdy)
        rVP <= #1 VP;
end

assign RE_VP = ~rVP & VP;

//  Capture and hold the rising edge pulse for NMI in NMI FF until serviced by
//      the processor.

assign CE_rNMI = Rdy & RE_VP;

always @(posedge Clk or posedge RE_NMI)
begin
    if(RE_NMI)
        rNMI <= 1'b1;
    else if(Rst)
        rNMI <= #1 0;
    else if(CE_rNMI)
        rNMI <= #1 RE_NMI;
end

//  Generate Int/Vector hold (lock) signal

assign CE_Hold  = Rdy & (LE_Int | RE_VP);

always @(posedge Clk or posedge Rst)
begin
    if(Rst)
        Hold <= 1;
    else if(CE_Hold)
        Hold <= #1 LE_Int;
end

//  Resolve Highest Priority Interrupt/Trap and select vector

assign iIRQ  = ~IRQ_Msk & IRQ;
assign iRQST = ~IRQ_Msk & RQST;

always @(negedge Clk)
begin
    if(Rst)
        {Int, Vector} <= #1 {1'b0, pVec_RST};
    else if(~Hold)
        casex({ABRT, rNMI, INV, SYS, iIRQ, iRQST, BRK, COP})
            15'b1xxx_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_ABRT };
            15'b01xx_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_NMI  };
            15'b001x_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_INV  };   
            15'b0001_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_SYS  };   
            15'b0000_1_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_IRQ  };   
            15'b0000_0_1xxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST7};   
            15'b0000_0_01xxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST6};
            15'b0000_0_001xxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST5};
            15'b0000_0_0001xxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST4};
            15'b0000_0_00001xxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST3};
            15'b0000_0_000001xx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST2};
            15'b0000_0_0000001x_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST1};
            15'b0000_0_00000001_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST0};
            15'b0000_0_00000000_1x : {Int, Vector} <= #1 {1'b1, pVec_BRK  };
            15'b0000_0_00000000_01 : {Int, Vector} <= #1 {1'b1, pVec_COP  };
            default : {Int, Vector} <= #1 {1'b0, pVec_RST  };
        endcase
end

endmodule
