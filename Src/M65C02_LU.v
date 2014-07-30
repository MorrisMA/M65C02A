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
// Create Date:     08:48:13 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_LU.v 
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
//  0.01    13I15   MAM     Initial coding. Pulled implementation details from
//                          the parent module, M65C02_ALU.v, and generated a
//                          standalone module instantiated in the parent.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_LU(
    input   En,
    
    input   [1:0] Op,
    input   [7:0] L,
    input   [7:0] M,
    
    output  reg [8:0] Out,
    output  Z,
    
    output  Val
);

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(*)
begin
    if(En)
        case(Op)
            2'b00 : Out <= ~L & M;  // TRB/RMBx/SEC/SEI/SED/REP
            2'b01 : Out <=  L & M;  // AND/BIT/BBRx/BBSx
            2'b10 : Out <=  L | M;  // ORA/TSB/SMBx/CLC/CLI/CLD/CLV/SEP
            2'b11 : Out <=  L ^ M;  // EOR
        endcase
    else
        Out <= 0;
end

assign Z   = ((En) ? ~|(L & M) : 0);
assign Val = En;

endmodule
