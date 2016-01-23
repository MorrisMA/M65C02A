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
// Module Name:     M65C02A_IntHndlr.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A
// Target Devices:  SRAM-based FPGAs: XC3S50A-xVQ100I, XC3S200A-xVQ100I 
// Tool versions:   Xilinx ISE 10.1i SP3
// 
// Description: 
//
//  This module implements a simple interrupt handler for the M65C02A soft-core
//  microprocessor. It accepts external active high inputs for Non-Maskable
//  Interrupt request (NMI) and maskable Interrupt ReQuest (IRQ). NMI is rising
//  edge sensitive, and IRQ is active high level sensitive. The module also
//  processed various other signals from the core and generates vectors for the
//  BRK, INV, and ABRT traps.
//
//  The module defines a fixed priority for each trap/interrupt processed. RST
//  is the highest priority, followed by ABRT, BRK, INV, NMI, IRQ, etc. A com-
//  plete definition of the priority of the traps and interrupt requests is
//  given below.
//
//  Vectors for each of the interrupt/trap sources are set using parameters. The
//  current implementation aims to maintain compatibility with the WDC W65C02S
//  processor, so IRQ and BRK share the same vector. A quick edit of the para-
//  meters allows an independent vector location to be added for BRK. Similarly,
//  the vectors for any of the interrupt/trap sources can be moved to any loca-
//  tion in the memory space, if W65C02S compatibility is not required.
//
// Dependencies:    redet.v 
//
// Revision:
//
//  1.00    15A03   MAM     Intial release.
//
//  1.01    15F25   MAM     Cleaned up comments and added #1 delay to reset
//                          clause of several FFs for consistency with personal
//                          coding standard. Added additional comments regarding
//                          the vectors.
//
//  1.10    15H03   MAM     Removed unused COP interrupt request input. Changed
//                          priority order of interrupt sources such that BRK
//                          is the highest priority, followed by INV, and then
//                          NMI, IRQ, and the 8 other maskable interrupts. Pri-
//                          ority changed to ensure that the BRK instruction is
//                          taken first before any external interrupt requests.
// 
// Additional Comments:
//
//  The address vectors used by the M65C02A processor core are not defined by
//  the core. The address vectors used for RST, nNMI, nIRQ/BRK, etc. are deter-
//  mined by this module. The core's interrupt/trap handler does not impose any
//  requirement on the location of these vectors within the address space. Thus,
//  it is possible for the application to locate the associated vectors to any
//  memory location, including page 0 or directly to the service routine in 
//  RAM/ROM.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_IntHndlr #(              // Pri
    parameter pVec_IRQ   = 16'hFFFE,    //  10 - Maskable Interrupt Svc Rqst
    parameter pVec_BRK   = 16'hFFFE,    //  13 - BRK Instruction Trap
    parameter pVec_RST   = 16'hFFFC,    //  15 - Highest Priority Trap
    parameter pVec_NMI   = 16'hFFFA,    //  11 - Non-Maskable Interrupt Svc Rqst
    parameter pVec_ABRT  = 16'hFFF8,    //  14 - Second Highest Priority Trap
    parameter pVec_RSVD2 = 16'hFFF6,    //  13 - Reserved for BRK
    parameter pVec_RSVD1 = 16'hFFF4,    //   1 - Lowest Practical Interrupt/Trap
    parameter pVec_RSVD0 = 16'hFFF2,    //   0
    parameter pVec_INV   = 16'hFFF0,    //  12 - Reserved: SYS Call Trap
    parameter pVec_RQST7 = 16'hFFEE,    //   9 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST6 = 16'hFFEC,    //   8 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST5 = 16'hFFEA,    //   7 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST4 = 16'hFFE8,    //   6 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST3 = 16'hFFE6,    //   5 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST2 = 16'hFFE4,    //   4 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST1 = 16'hFFE2,    //   3 - Maskable Interrupt Svc Rqst
    parameter pVec_RQST0 = 16'hFFE0     //   2 - Maskable Interrupt Svc Rqst
)(
    input   Rst,                // Highest Priority Interrupt Source            
    input   Clk,
    
    input   Rdy,                // Microcycle Ready Input
    
    input   ABRT,               // ABoRT MMU trap
    input   BRK,                // BReaK instruction trap
    input   INV,                // Invalid Instruction Trap
    input   NMI,                // Non-Maskable Interrupt Request (6502 NMI)
    input   IRQ,                // Maskable Interrupt Request (6502 IRQ)
    input   [7:0] RQST,         // Maskable Interrupt Requests (RQST[7] highest)
    
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
        
//  Detect the rising edge of VP to capture/hold Int/Vector

always @(posedge Clk or posedge Rst)
begin
    if(Rst)
        rVP <= #1 0;
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
        rNMI <= #1 1'b1;
    else if(Rst)
        rNMI <= #1 0;
    else if(CE_rNMI)
        rNMI <= #1 RE_NMI;
end

//  Generate Int/Vector hold (lock) signal

assign CE_Hold = Rdy & (LE_Int | RE_VP);

always @(posedge Clk or posedge Rst)
begin
    if(Rst)
        Hold <= #1 1;
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
        casex({ABRT, BRK, INV, rNMI, iIRQ, iRQST})
            13'b1xx_xx_xxxxxxxx : {Int, Vector} <= #1 {1'b1, pVec_ABRT };
            13'b01x_xx_xxxxxxxx : {Int, Vector} <= #1 {1'b1, pVec_BRK  };
            13'b001_xx_xxxxxxxx : {Int, Vector} <= #1 {1'b1, pVec_INV  };   
            13'b000_1x_xxxxxxxx : {Int, Vector} <= #1 {1'b1, pVec_NMI  };
            13'b000_01_xxxxxxxx : {Int, Vector} <= #1 {1'b1, pVec_IRQ  };   
            13'b000_00_1xxxxxxx : {Int, Vector} <= #1 {1'b1, pVec_RQST7};   
            13'b000_00_01xxxxxx : {Int, Vector} <= #1 {1'b1, pVec_RQST6};
            13'b000_00_001xxxxx : {Int, Vector} <= #1 {1'b1, pVec_RQST5};
            13'b000_00_0001xxxx : {Int, Vector} <= #1 {1'b1, pVec_RQST4};
            13'b000_00_00001xxx : {Int, Vector} <= #1 {1'b1, pVec_RQST3};
            13'b000_00_000001xx : {Int, Vector} <= #1 {1'b1, pVec_RQST2};
            13'b000_00_0000001x : {Int, Vector} <= #1 {1'b1, pVec_RQST1};
            13'b000_00_00000001 : {Int, Vector} <= #1 {1'b1, pVec_RQST0};
            default : {Int, Vector} <= #1 {1'b0, pVec_RST};
        endcase
end

endmodule
