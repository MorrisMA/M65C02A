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
