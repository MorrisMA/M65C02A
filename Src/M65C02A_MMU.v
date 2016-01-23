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
// Module Name:     M65C02A_MMU 
// Project Name:    C:\XProjects\ISE10.1i\M65C02Duo 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This module implements a simple memory management unit for the M65C02A soft-
//  core microprocessor. It uses dual-ported distributed memory to map a virtual
//  address to a physical address and a chip enable. The uppermost 4 bits of the
//  16-bit M65C02A virtual address plus a Kernel/User mode input is used to
//  address a set of 32 memory mapping registers.
//
//  This MMU only provides simple address substitution and chip select genera-
//  tion. Address arithmetic and range checking could be added, but the addi-
//  tional logic in the critical address output data path will result in lower
//  performance. Provisions are included in the module to allow the module to be
//  expanded to support Kernel/User mode and access controls for virtual memory.
//  The present implementation only supports a simple mapping from virtual to
//  physical address, but is does allow the total address space to be expanded
//  to more than 4MB.
//
//  The 16-bit mapping register is defined by the following structure:
//
//      MMU <= {(Rsvd[2:0], WS, CS[3:0], PA[19:12]}
//
//  PA[19:12]   : most significant 8 bits of the physical address of a segment
//  CS[3:0]     : selects the chip select to use for each segment
//  WS          : add constant number of wait states as determined by pWS_Out
//  Rsvd[2:0]   : reserved for future use as access controls for each segment
//
//  The uppermost three bits are currently not implemented. There are plans to
//  modify the M65C02 core to support Kernel/User mode and potentially virtual
//  memory. These three bits are reserved for this purpose at this time in order
//  that a future MMU module can be dropped into the design without any signifi-
//  cant changes required to the implementation of the M65C02A microprocessor.
//  
// Dependencies:    None 
//
// Revision: 
//
//  1.00    15A03   MAM     Initial release.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_MMU #(
    parameter pMAP_Init = "Pgms/M65C02A_MMU32.coe"  // MAP RAM Initialization
)(
    input   Rst,                        // System Reset
    input   Clk,                        // System Clock
    
    input   Rdy,                        // M65C02A Microcycle Ready Signal                   

    // MMU General Inputs

    input   Mode,                       // M65C02A Operating Mode
    input   Sync,                       // M65C02A Instruction Fetch Strobe
    input   [ 1:0] IO_Op,               // M65C02A IO Operation

    input   [15:0] VA,                  // M65C02A Virtual Address/Reg Select

    // MAP/MMU/Vec Management Interface

    input   Sel_MAP,                    // MMU Mapping Registers Select
    input   Sel_MMU,                    // MMU Status Register Select
    input   WE,                         // MMU Write Enable
    input   RE,                         // MMU Read Enable
    input   [7:0] MMU_DI,               // MMU Register Data Input
    output  [7:0] MMU_DO,               // MMU Register Data Output

    // MAP Virtual-Physical Mapping Interface

    output  [19:0] PA,                  // MMU Physical (Mapped) Address Output
    output  [15:1] CE,                  // MMU Chip Enable Output
    output  Int_WS,                     // MMU Interanal Wait State Request

    // MAP Access Violation and Mapping Error Interface
    
    output  ABRT                        // Memory Cycle ABoRT trap output
);

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

reg     [15:0] MAP [31:0];              // MMU Map RAM - LUT-based Dual-Port RAM
wire    [15:0] MAP_DI, MAP_SPO;         // MMU Map RAM Data I/O Busses
wire    MAP_WE, MAP_RE;                 // MMU Map RAM Write and Read Enables

wire    [ 4:0] Page;                    // MMU Map RAM Page Select
wire    [15:0] MAP_DPO;                 // MMU Map RAM Dual-Port RAM Output
reg     [15:1] CS_Out;                  // One-Hot Decode Chip Select (CS)

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Implement Dual-Ported Distributed RAM for MMU MAP

initial
    $readmemh(pMAP_Init, MAP, 0, 31);
    
assign MAP_WE = Sel_MAP & WE & Rdy;
assign MAP_DI = ((VA[0]) ? {MMU_DI, MAP_SPO[7:0]} : {MAP_SPO[15:8], MMU_DI});

always @(posedge Clk)
begin
    if(MAP_WE)
        MAP[VA[5:1]] <= #1 MAP_DI;
end

assign MAP_SPO = MAP[VA[5:1]];

//  Provide multiplexer to read out the contents of the selected MMU register

assign MAP_RE = Sel_MAP & RE;
assign MMU_DO = ((MAP_RE) ? ((VA[0]) ? MAP_SPO[15:8] : MAP_SPO[7:0]) : 0);

////////////////////////////////////////////////////////////////////////////////
//
//  MMU Mapping Logic
//

//  Map Virtual Address into Wait State, Chip Enable, and Physical Address

assign Page    = {Mode, VA[15:12]};
assign MAP_DPO = MAP[Page];

//  Map the 4-bit encoded CS field of the MAP RAM into One-Hot CS output

always @(*)
begin
    case(MAP_DPO[11:8])
        4'b0000  : CS_Out[15:1] <= 15'b00000000_0000000;
        4'b0001  : CS_Out[15:1] <= 15'b00000000_0000001;
        4'b0010  : CS_Out[15:1] <= 15'b00000000_0000010;
        4'b0011  : CS_Out[15:1] <= 15'b00000000_0000100;
        4'b0100  : CS_Out[15:1] <= 15'b00000000_0001000;
        4'b0101  : CS_Out[15:1] <= 15'b00000000_0000000;
        4'b0110  : CS_Out[15:1] <= 15'b00000000_0000000;
        4'b0111  : CS_Out[15:1] <= 15'b00000000_0000000;
        4'b1000  : CS_Out[15:1] <= 15'b00000001_0000000;
        4'b1001  : CS_Out[15:1] <= 15'b00000010_0000000;
        4'b1010  : CS_Out[15:1] <= 15'b00000100_0000000;
        4'b1011  : CS_Out[15:1] <= 15'b00001000_0000000;
        4'b1100  : CS_Out[15:1] <= 15'b00010000_0000000;
        4'b1101  : CS_Out[15:1] <= 15'b00100000_0000000;
        4'b1110  : CS_Out[15:1] <= 15'b01000000_0000000;
        4'b1111  : CS_Out[15:1] <= 15'b10000000_0000000;
    endcase
end

//  Output the mapped physical address (PA), CE, and WS signals

assign PA     = {MAP_DPO[7:0], VA[11:0]};
assign CE     = CS_Out;
assign Int_WS = MAP_DPO[12];

////////////////////////////////////////////////////////////////////////////////
//
//  MMU Access Validation and Page Modification Logic
//

assign ABRT = 1'b0;

endmodule
