////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2006-2014 by Michael A. Morris, dba M. A. Morris & Associates
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

//////////////////////////////////////////////////////////////////////////////////
// Company:         Alpha Beta Technologies, Inc.
// Engineer:        Michael A. Morris 
// 
// Create Date:     03/01/2008
// Design Name:     USB MBP HDL 
// Module Name:     fedet.v
// Project Name:    4020 HAWK ZAOM Upgrade
// Target Devices:  XC2S150-5PQ208I 
// Tool versions:   ISE 8.2i
//
// Description:     Multi-stage synchronizer with falling edge detection
//
// Dependencies:    None
//
// Revision History:
//
//  0.01    08C01   MAM     File Created
//
// Additional Comments: 
//
///////////////////////////////////////////////////////////////////////////////

module fedet(rst, clk, din, pls);

///////////////////////////////////////////////////////////////////////////////
//
//  Module Port Declarations
//
    
    input   rst;
    input   clk;
    input   din;
    output  pls;

///////////////////////////////////////////////////////////////////////////////
//
//  Module Level Declarations
//

    reg [2:0] QSync;
    
///////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

always @(posedge clk or posedge rst) begin
    if(rst)
        #1 QSync <= 3'b011;
    else 
        #1 QSync <= {~QSync[0] & QSync[1], QSync[0], din};
end

assign pls = QSync[2];

endmodule
