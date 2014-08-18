////////////////////////////////////////////////////////////////////////////////
//
//  Testbench for M65C02A soft-core microcomputer project.
//
//  Copyright (C) 2014  Michael A. Morris
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
// Create Date:     19:44:19 07/04/2014
// Design Name:     M65C02A
// Module Name:     C:/XProjects/ISE10.1i/M65C02A/Src/tb_M65C02A.v
// Project Name:    M65C02A
// Target Device:   Xilinx SRAM FPGAs  
// Tool versions:   ISE 10.1i SP3
// Description: 
//
// Verilog Test Fixture created by ISE for module: M65C02A
//
// Dependencies:
// 
// Revision:
// 
//  0.00    14F04   MAM     Initial creation
//
//  1.00    14G08   MAM     Modified to override the default microprogram files.
//
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_M65C02A;

// Inputs

reg     nRst = 0;
reg     Clk  = 1;

reg     nNMI = 1;
reg     nIRQ = 1;
wire    nVP;

reg     nSO;

wire    Sync;
wire    nML;

reg     Rdy;

wire    [ 3:0] nCE;
wire    nRD;
wire    nWR;
wire    [19:16] XA;
wire    [15: 0] AB;
wire    [ 7: 0] DB;

wire    [1:0] nSSel;
wire    SCK;
wire    MOSI;
reg     MISO;

wire    COM0_TxD;
reg     COM0_RxD;
wire    COM0_nRTS;
reg     COM0_nCTS;
wire    COM0_DE;

wire    COM1_TxD;
reg     COM1_RxD;
wire    COM1_nRTS;
reg     COM1_nCTS;
wire    COM1_DE;

// Instantiate the Unit Under Test (UUT)

M65C02A #(
            .pM65C02_uPgm("Pgms/M65C02_uPgm_V4a.coe"),          // SEQ   :  2 kB
            .pM65C02_IDec("Pgms/M65C02_IDecode_ROMa.coe")       // DEC   :  2 kB
        ) uut (
            .nRst(nRst), 
            .Clk(Clk), 
            
            .nNMI(nNMI), 
            .nIRQ(nIRQ), 
            .nVP(nVP),

            .nSO(nSO),
            
            .Sync(Sync), 
            .nML(nML), 

            .RdyIn(Rdy), 

            .nCE(nCE), 
            .nRD(nRD), 
            .nWR(nWR), 
            .XA(XA),
            .AB(AB),
            .DB(DB),
            
            .nSSel(nSSel),
            .SCK(SCK),
            .MOSI(MOSI),
            .MISO(MISO),
            
            .COM0_TxD(COM0_TxD),
            .COM0_RxD(COM0_RxD),
            .COM0_nRTS(COM0_nRTS),
            .COM0_nCTS(COM0_nCTS),
            .COM0_DE(COM0_DE),

            .COM1_TxD(COM1_TxD),
            .COM1_RxD(COM1_RxD),
            .COM1_nRTS(COM1_nRTS),
            .COM1_nCTS(COM1_nCTS),
            .COM1_DE(COM1_DE)
        );

initial begin
    // Initialize Inputs
    nRst      = 0;
    Clk       = 1;
    nNMI      = 1;
    nIRQ      = 1;
    nSO       = 1;
    Rdy       = 1;
    MISO      = 1;
    COM0_RxD  = 1;
    COM0_nCTS = 0;
    COM1_RxD  = 1;
    COM1_nCTS = 0;

    // Wait 10 clock cycles for global reset to finish (100 ns minimum)
    
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk) #1 nRst = 1;
    
    // Add stimulus here
    
    @(posedge uut.WAI) nIRQ = 0;
    @(negedge uut.WAI) nIRQ = 1;

end

//always #5 Clk = ~Clk;
always #16.954 Clk = ~Clk;

always @(posedge Clk)
begin
    if(uut.IO_Op[0])
        if(uut.VA == 16'hFFF8)
            nIRQ = #1 0;
        else if(uut.VA == 16'hFFF9)
            nIRQ = #1 1;
end


reg TFF;
reg [7:0] Tst;

always @(posedge Clk)
begin
    if(uut.Rst) begin
        TFF = #1 1;
        Tst = #1 0;
    end else if(uut.IO_Op[0])
        if(uut.VA == 16'h0200) begin
            TFF = #1 ((uut.CPU_DO != Tst) ? ~TFF : TFF);
            Tst = #1 uut.CPU_DO;
        end
end

reg [7:0] FD_Bx [0:15];

always @(posedge Clk)
begin
    if(uut.IO_Op[0]) begin
        if(uut.VA[15:4] == 12'h00B)
            FD_Bx[uut.VA[3:0]] = #1 uut.CPU_DO;
    end
end

reg [7:0] FD_Fx [0:15];

always @(posedge Clk)
begin
    if(uut.IO_Op[0]) begin
        if(uut.VA[15:4] == 12'h00F)
            FD_Fx[uut.VA[3:0]] = #1 uut.CPU_DO;
    end
end

reg [7:0] FD_20x [0:15];

always @(posedge Clk)
begin
    if(uut.IO_Op[0]) begin
        if(uut.VA[15:4] == 12'h020)
            FD_20x[uut.VA[3:0]] = #1 uut.CPU_DO;
    end
end

endmodule

