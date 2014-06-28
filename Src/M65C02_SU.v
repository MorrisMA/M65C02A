`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
// 
// Create Date:     08:15:35 09/15/2013 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_SU.v 
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
//  1.00    13J23   MAM     Corrected error in the operation multiplexer. Op = 1
//                          is for right shift/rotate operations, and Op = 0 is
//                          for left shift/rotate operations.
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_SU(
    input   En,
    
    input   Op,
    input   [7:0] SU,
    input   Ci,
    
    output  reg [8:0] Out,
    output  reg OV,
    output  Val
);

always @(*)
begin
    if(En)
        {OV, Out} <= ((Op) ? {^SU[7:6], {SU[0], {Ci, SU[7:1]}}}     // LSR/ROR
                           : {^SU[7:6], {SU[7], {SU[6:0], Ci}}});   // ASL/ROL
    else
        {OV, Out} <= 0;
end

assign Val = En;

endmodule
