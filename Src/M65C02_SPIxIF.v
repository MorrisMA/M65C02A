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
// Company:         Michael A. Morris 
// Engineer:        M. A. Morris & Associates
// 
// Create Date:     17:27:03 12/27/2013 
// Design Name:     M65C02 Dual Core 
// Module Name:     M65C02_SPIxIF.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02Duo 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
//
// Description:
//
//  This module provides a wrapper for an SPI Master Interface that interfaces
//  that module to the memory/IO bus of the M65C02/M65CO2Duo microprocessor.
//  The wrapper provides a control register, a status register, a transmit FIFO
//  interface, and a receive FIFO interface.
//
//  The control register is a R/W register. It controls the features of the
//  SPIxIF module, and provides a way to read the current value of the register.
//  The status register is a RO register. It allows the status of the SPI Master
//  interface to be monitored. Specifically, the Empty/Full status of the Tx/Rx
//  FIFOs can be monitored.
//
//  The SPIxIF module provides a means by which the data received simultaneously
//  with the transmitted data can be ignored or written into the receive data
//  FIFO. With SPI memory devices, the data received from the devices before the
//  command and the address have been written are undefined. Using this capabi-
//  lity of the SPIxIF, it is possible to ignore all data received from these
//  devices until after the command and address have been written.
//
//  To provide this feature, the SPIxIF module utilizes a bit written into the
//  TD FIFO. This asdditional TD bit is used to enable or disable the capturing
//  of the receive data. If the 9th TD bit is set, then the serial data captured
//  during the transmission of the TD data is saved in the RD FIFO. If the 9th
//  bit is clear, i.e. not set, then the serial data captured during the trans-
//  mission of the TD is NOT saved in the RD FIFO.
//
//  The M65C02_SPIxIF allocates two registers for this purpose. The first regis-
//  ter is set on an even address, and writes to this register clear the 9th TD
//  bit. This results in the data received being ignored. The second register is
//  set on an odd address, and writes to this register set the 9th TD bit. This
//  results in the data received being captured in the RD FIFO. Writes to these
//  logical registers result in writing of data to a single TD FIFO with the 9th
//  bit clear or set.
//
//  The single RD FIFO is mapped as a RO component to the same addresses as the
//  single, write-only (WO) TD FIFO. Reading from the WO TD FIFO addresses reads
//  from RD FIFO. Either TD FIFO address can be used for reading the RD FIFO.
//
// Dependencies:     
//
// Revision: 
//
//  0.00    13L24   MAM     Initial File Creation. 
//
// Additional Comments:
//
//  The address/register map of this module is provided in the following table:
//
//  Reg[1:0]    Mode    Name    Description
//     00        RW      CR     SPIxIF Control Register
//     01        RO      SR     SPIxIF Status Register
//     10        WO      TD0    Transmit Data FIFO - Bit 9 Clr
//     11        WO      TD1    Transmit Data FIFO - Bit 9 Set
//     1x        RO      RD     Receive Data FIFO
//
//  The SPIxIF Control Register is defined in the following table:
//
//  Bit     Name    Description
//   0      Port    SPI Port/Slave Select: 1 - SSel[1]; 0 - SSel[0]
//   1      Man     SSel Operation: 0 - Auto; 1 - Manual
//  3:2     Mode    SPI Operating Mode: 0, 1, 2, 3
//  6:4     Baud    SPI Shift Clock Rate: Clk/(2**(Baud + 1))
//   7      IE      SPI TD FIFO Interrupt Enable: 1 - Enable; 0 - Disable
//
//  The SPIxIF Status Register is defined in the following table:
//
//  Bit     Name    Description
//   0      TxRdy   SPI Tx Ready: 1 - Ready (Tx FIFO not Full)
//   1      TF_HF   SPI Tx FIFO Half Full Flag
//   2      TF_EF   SPI Tx FIFO Empty Flag
//   3      RxErr   SPI Rx FIFO Overflow: 1 - Rx FIFO Full
//  6:4     Rsvd    Reseved: read as 0s
//   7      Active  SPI shift Active: 1 - SPI Active; 0 - Shift Completed
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_SPIxIF #(
    parameter pDefault_CR     = 8'h30,  // Rate=1/16, Mode=0, SSel=Auto, Sel=0 
    parameter pShift_Dir      = 1'b0,   // Shift Direction: 0 - MSB, 1 - LSB
    parameter pSPI_FIFO_Depth = 8'd4,   // Depth = (1 << 4) = 16
    parameter pSPI_FIFO_Init  = "Src/SPI_FIFO_Init_16.coe"
)(
    input   Rst,
    input   Clk,
    
    //  M65C02 Module Interface
    
    input   Sel,
    input   RE,
    input   WE,
    
    input   [1:0] Reg,
    input   [7:0] DI,
    output  wor [7:0] DO,
    
    // M65C02 Interrupt Interface

    output  IRQ,
    
    //  SPI Interface
    
    output  reg [1:0] SSel,
    output  SCK,
    output  MOSI,
    input   MISO
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameters
//

localparam pFIFO_Depth = (2**(pSPI_FIFO_Depth));

localparam pSel_CR  = 2'b00;
localparam pSel_SR  = 2'b01;
localparam pSel_TD0 = 2'b10;
localparam pSel_TD1 = 2'b11;
localparam pSel_RD0 = 2'b10;
localparam pSel_RD1 = 2'b11;

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wire    WE_CR, OE_CR;
reg     [7:0] CR;                       // Control Register

wire    Port, Man, IE;                  // Control Register Fields
wire    [1:0] Mode;
wire    [2:0] Rate;

wire    RE_SR, OE_SR;
wire    [7:0] SR;                       // Status Register

wire    WE_TF, RE_TF;                   // SPI Transmit FIFO Signals 
wire    [8:0] SPI_TD;
wire    TF_FF, TF_HF, TF_EF;

wire    WE_RF, RE_RF, OE_RF;            // SPI Receive FIFO Signals
wire    [7:0] SPI_RD, RD;
wire    RF_FF;

wire    SS;                             // SPI Slave Select Strobe

wire    feRE_SR, feTF_EF, reTF_EF;      // Interrupt Generator
wire    Rst_Int;
reg     Int;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Control Register

assign WE_CR = (Sel & (Reg == pSel_CR)) & WE;
assign OE_CR = (Sel & (Reg == pSel_CR));

always @(posedge Clk)
begin
    if(Rst)
        CR <= #1 pDefault_CR;
    else if(WE_CR)
        CR <= #1 DI;
end

//  Decode Control Register

assign Port = CR[0];                    // SPI Port/Slave Select
assign Man  = CR[1];                    // SPI SSel Operation: 0-Auto; 1-Man
assign Mode = CR[3:2];                  // SPI Mode Select
assign Rate = CR[6:4];                  // SPI Shift Clock Rate Select
assign IE   = CR[7];                    // SPI Interrupt Enable

//  Status Register

assign RE_SR = (Sel & (Reg == pSel_SR) & RE);
assign OE_SR = (Sel & (Reg == pSel_SR));

assign SR = {SS, 3'b000, RF_FF, TF_EF, TF_HF, ~TF_FF};

//  Instantiate Transmit and Receive Data FIFOs

assign WE_TF = Sel & Reg[1] & WE;

DPSFnmCE    #(
                .addr(pSPI_FIFO_Depth),
                .width(9),
                .init(pSPI_FIFO_Init)
            ) TF (
                .Rst(Rst), 
                .Clk(Clk), 

                .WE(WE_TF), 
                .DI({Reg[0], DI}), 

                .RE(RE_TF), 
                .DO(SPI_TD),
                
                .FF(TF_FF), 
                .HF(TF_HF), 
                .EF(TF_EF), 

                .Cnt()
            );
            
assign RE_RF = Sel & Reg[1] & RE;
assign OE_RF = Sel & Reg[1];

DPSFnmCE    #(
                .addr(pSPI_FIFO_Depth),
                .width(8),
                .init(pSPI_FIFO_Init)
            ) RF (
                .Rst(Rst), 
                .Clk(Clk), 

                .WE(WE_RF), 
                .DI(SPI_RD), 

                .RE(RE_RF), 
                .DO(RD), 

                .FF(RF_FF), 
                .EF(), 
                .HF(), 

                .Cnt()
            );

//  Instantiate SPIxIF Module

SPIxIF  SPI (
            .Rst(Rst), 
            .Clk(Clk),
            
            .LSB(pShift_Dir), 
            .Mode(Mode), 
            .Rate(Rate),
            
            .DAV(~TF_EF),
            
            .FRE(RE_TF), 
            .TD(SPI_TD),
            
            .FWE(WE_RF), 
            .RD(SPI_RD), 

            .SS(SS), 
            .SCK(SCK), 
            .MOSI(MOSI), 
            .MISO(MISO)
        );

//  Assign SS to selected port

always @(*)
begin
    case({Man, Port})
        2'b00 : SSel <= {1'b0, SS  };
        2'b01 : SSel <= {SS  , 1'b0};
        2'b10 : SSel <= {1'b0, 1'b1};
        2'b11 : SSel <= {1'b1, 1'b0};        
    endcase
end

//  Assign Interrupt Request Output

fedet   FE1 (.rst(Rst), .clk(Clk), .din(RE_SR),  .pls(feRE_SR));
fedet   FE2 (.rst(Rst), .clk(Clk), .din(TF_EF),  .pls(feTF_EF));
redet   RE1 (.rst(Rst), .clk(Clk), .din(TF_EF),  .pls(reTF_EF));

assign Rst_Int = (Rst | feRE_SR | feTF_EF);

always @(posedge Clk or posedge Rst_Int)
begin
    if(Rst_Int)
        Int <= #1 0;
    else if(reTF_EF)
        Int <= #1 TF_EF;
end

assign IRQ = ((IE) ? Int : 0);

//  Define Data Output (DO)

assign DO = ((OE_CR) ? CR : 0);
assign DO = ((OE_SR) ? SR : 0);
assign DO = ((OE_RF) ? RD : 0);

endmodule
