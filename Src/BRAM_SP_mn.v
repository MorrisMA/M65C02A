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
// Create Date:     12:44:04 12/31/2013 
// Design Name:     M65C02 Dual Core
// Module Name:     BRAM_SP_mn.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02Duo 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This module provides a single clock, single-port Block RAM. A write protect
//  input, WP, will disable writes of the Block RAM if asserted.
//
// Dependencies: 
//
// Revision: 
//
//  0.00    13L31   MAM     Initial File Creation.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module BRAM_SP_mn #(
    parameter pBRAM_AddrWidth  = 8'd11,
    parameter pBRAM_Width      = 8'd8,
    parameter pBRAM_SP_mn_Init = "Pgms/BRAM_SP.txt"
)(
    input   Clk,
    
    input   WP,
    input   CE,
    input   WE,
    input   [(pBRAM_AddrWidth - 1):0] PA,
    input   [(pBRAM_Width - 1):0] DI,
    output  [(pBRAM_Width - 1):0] DO
);

localparam pBRAM_Depth = (2**pBRAM_AddrWidth);

//  Infer shared Block RAM with Write Protect

reg [(pBRAM_Width - 1):0] BRAM [(pBRAM_Depth - 1):0];

//  Initialize Block RAM

wire    BRAM_CE, BRAM_WE;
wire    [(pBRAM_AddrWidth - 1):0] BRAM_Addr;
wire    [(pBRAM_Width - 1):0] BRAM_DI;
reg     [(pBRAM_Width - 1):0] BRAM_DO;

assign BRAM_CE   = CE;
assign BRAM_WE   = WE & ~WP;
assign BRAM_Addr = PA;
assign BRAM_DI   = DI;

initial
    $readmemh(pBRAM_SP_mn_Init, BRAM, 0, (pBRAM_Depth - 1));

always @(posedge Clk)
begin
    if(BRAM_CE) begin
        if(BRAM_WE)
            BRAM[BRAM_Addr] <= #1 BRAM_DI;
        
        BRAM_DO <= #1 BRAM[BRAM_Addr];
    end
end

assign DO = ((BRAM_CE) ? BRAM_DO : 0);

endmodule
