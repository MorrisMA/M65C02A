////////////////////////////////////////////////////////////////////////////////
//
//  Interrupt Handler module for M65C02A soft-core microcomputer project.
//
//  Copyright (C) 2013-2014  Michael A. Morris
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

module M65C02_IntHndlrV2 #(             // Pri
    parameter pVec_IRQ   = 16'hFFFE,    //  11
    parameter pVec_BRK   = 16'hFFFE,    //   2
    parameter pVec_RST   = 16'hFFFC,    //  15 - Highest Priority
    parameter pVec_NMI   = 16'hFFFA,    //  13
    parameter pVec_ABRT  = 16'hFFF8,    //  14 - Second Highest Priority
    parameter pVec_RSVD1 = 16'hFFF6,    //   0 - Reserved for BRK
    parameter pVec_COP   = 16'hFFF4,    //   1 - Lowest Practical Interrupt/Trap
    parameter pVec_RSVD0 = 16'hFFF2,    //   0
    parameter pVec_INV   = 16'hFFF0,    //  12
    parameter pVec_RQST7 = 16'hFFEE,    //  10
    parameter pVec_RQST6 = 16'hFFEC,    //   9
    parameter pVec_RQST5 = 16'hFFEA,    //   8
    parameter pVec_RQST4 = 16'hFFE8,    //   7
    parameter pVec_RQST3 = 16'hFFE6,    //   6
    parameter pVec_RQST2 = 16'hFFE4,    //   5
    parameter pVec_RQST1 = 16'hFFE2,    //   4
    parameter pVec_RQST0 = 16'hFFE0     //   3
)(
    input   Rst,                // Highest Priority Interrupt Source            
    input   Clk,
    
    input   Rdy,                // Microcycle Ready Input
    
    input   ABRT,               // ABoRT MMU trap
    input   NMI,                // Non-Maskable Interrupt Request (6502 NMI)
    input   INV,                // Invalid Instruction Trap
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
        casex({ABRT, rNMI, INV, iIRQ, iRQST, BRK, COP})
            14'b1xx_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_ABRT };
            14'b01x_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_NMI  };
            14'b001_x_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_INV  };   
            14'b000_1_xxxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_IRQ  };   
            14'b000_0_1xxxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST7};   
            14'b000_0_01xxxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST6};
            14'b000_0_001xxxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST5};
            14'b000_0_0001xxxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST4};
            14'b000_0_00001xxx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST3};
            14'b000_0_000001xx_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST2};
            14'b000_0_0000001x_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST1};
            14'b000_0_00000001_xx : {Int, Vector} <= #1 {1'b1, pVec_RQST0};
            14'b000_0_00000000_1x : {Int, Vector} <= #1 {1'b1, pVec_BRK  };
            14'b000_0_00000000_01 : {Int, Vector} <= #1 {1'b1, pVec_COP  };
            default : {Int, Vector} <= #1 {1'b0, pVec_RST  };
        endcase
end

endmodule
