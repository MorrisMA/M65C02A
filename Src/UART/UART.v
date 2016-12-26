////////////////////////////////////////////////////////////////////////////////
//
//  Parallel Interface Universal Asynchronous Receiver/Transmiter
// 
//  Copyright 2013-2014 by Michael A. Morris, dba M. A. Morris & Associates
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
// Create Date:     16:12:34 12/28/2013 
// Design Name:     M65C02 Dual Core 
// Module Name:     M65C02_UART.v
// Project Name:    C:\XProjects\ISE10.1i\M65C02Duo 
// Target Devices:  Generic SRAM-based FPGA, XC3S50A-4VQG100I, XC3S200A-4VQG100I 
// Tool versions:   ISE 10.1i SP3 
//
// Description:
//
//  This module integrates the various elements of a simplified UART to create
//  an efficient, programmable UART. The M65C02_UART provides an M65C02-compati-
//  ble interface to a simplified 16C550-style UART module. The simplified UART
//  module implements those features most often used and deletes those features
//  which are seldom used.
//
//  The UART module supports a reduced number of line formats. For example, it
//  does not support the 5-bit or 6-bit frame formats supported by 16C550-compa-
//  tible UARTs. It also does not support 1.5 stop bits, or space or mark parity
//  settings. Instead, the UART module supports four operating modes that allows
//  easier support of RS-232/RS-485 industrial automation protocols.
//
//  These four operating modes allow both 2-wire and 4-wire RS-232 protocols to
//  be easily implemented. The UART incorporates HW flow control for the 4-wire
//  RS-232 protocols. In contrast, a 16C550-compatible UART requires RTS/CTS or
//  DTR/DSR flow control to be implement in the firmware/software driver.
//
//  In addition, the two RS-485 operating modes provide automatic support for 
//  half-duplex protocols w/ or w/o collision detection. Automatic support for
//  half-duplex protocols is based on the feature of the UART to automatically
//  assert and deassert the RS-485 Drive Enable (DE) signal of standard 75176-
//  compatible RS-485 transceivers. In addition, these RS-485 modes support a
//  programmable line idle time such as that used to frame Modbus-RTU and Profi-
//  bus. 
//
// Dependencies:
//
//      UART.v
//          re1ce.v
//          BRSFmnCE.v
//          UART_BRG.v
//          UART_TXSM.v
//          UART_RXSM.v
//          UART_RTO.v
//          UART_INT.v
//              redet.v
//
// Revision History:
//
//  0.01    13L28   MAM     File Created
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module UART #( 
    // Default UART Settings
    
    parameter pDefault_LCR = 8'h80,     // RTSo, Tx Enable, 232 w/o HS, 8N1
    parameter pDefault_IER = 8'h00,     // All interrupts disabled
    
    //  Default UART Baud Rate Settings
    
    parameter pFrequency = 29491200,    // 29.4912 MHz
    parameter pBaudrate  = 115200,      // 115200 bps
    
    // Default Receive Time Out Character Delay Count

    parameter pRTOChrDlyCnt = 3,        // Default Number of Characters for RTO

    // FIFO Configuration Parameters

    parameter pTF_Depth = 0,            // Tx FIFO Depth: 2**(TF_Depth + 4)
    parameter pRF_Depth = 0,            // Rx FIFO Depth: 2**(RF_Depth + 4)
    parameter pTF_Init  = "Pgms/UART_TF_16.coe",// Tx FIFO Memory Initialization
    parameter pRF_Init  = "Pgms/UART_RF_16.coe" // Rx FIFO Memory Initialization
)(
    input   Rst,                        // System Reset
    input   Clk,                        // System Clock
    
    //  External Interrupt Request
    
    output  reg IRQ,                    // Interrupt Request
    
    //  Parallel Interface
    
    input   Sel,                        // UART Select
    input   [1:0] Reg,                  // UART Register Select
    input   RE,                         // UART Read Strobe
    input   WE,                         // UART Write Strobe
    input   [7:0] DI,                   // UART Data Input Bus
    output  wor [7:0] DO,               // UART Data Output Bus
    
    //  External UART Interface
    
    output  TxD,                        // TxD
    input   RxD,                        // RxD
    output  reg xRTS,                   // RS-232 Mode RTS (Ready-To-Receive)
    input   xCTS,                       // RS-232 Mode CTS (Okay-To-Send)
    
    output  xDE,                        // RS-485 Mode Transceiver Drive Enable
    
    output  Mode,                       // Mode: 0 - RS232; 1 - RS485

    //  TxSM/RxSM Status
    
    output  TxIdle,                     // UART TxSM Idle Status Output
    output  RxIdle                      // UART RxSM Idle Status Output
); 

////////////////////////////////////////////////////////////////////////////////
//
//  Module Parameters
// 

localparam pDiv_Default = ((pFrequency / (16 * pBaudrate)) - 1);

//  Register Addresses

localparam pTHR = 0;            // Tx Data Holding Register (FIFO) (BRL)
localparam pRHR = 0;            // Rx Data Holding Register (FIFO)
localparam pLSR = 1;            // UART Line Status Register       (BRH)
localparam pLCR = 2;            // UART Line Control Register
localparam pIER = 3;            // UART Interrupt Enable Register

//  FIFO Configuration Parameters

localparam pWidth = 8;          // Maximum Character width
localparam pxFThr = 8;          // TF/RF Half-Full Flag Theshold (%, 4 bits)

////////////////////////////////////////////////////////////////////////////////   
//
//  Declarations
//

reg     [7:0] LSR;              // UART Line Status Register (LCR)
wire    Sel_LSR;                // LSR: Register Select
wire    RE_LSR;                 // LSR: Read Enable Line Status Register

wire    CTSi;                   // LSR: CTS Input
wire    RTSi;                   // LSR: RTS Input
wire    iTHE;                   // LSR: Transmit Half Empty Interrupt
wire    iTEF;                   // LSR: Transmit FIFO Empty Interrupt
wire    iTEM;                   // LSR: Transmit SR Empty Interrupt
wire    TxRdy;                  // LSR: Transmitter Ready
wire    iRFE;                   // LSR: Receive Framing Error Interrupt
wire    iRPE;                   // LSR: Receive Parity Error Interrupt
wire    iRHF;                   // LSR: Receive Half Full Interrupt
wire    iRTO;                   // LSR: Receive Timeout Interrupt
wire    RxRdy;                  // LSR: Receiver Ready

reg     Clr_Int;                // Clear Interrupt Flags - read of LSR

reg     [7:0] LCR;              // UART Line Control Register (LCR)
wire    Sel_LCR;                // LCR: Register Select
wire    WE_LCR;                 // LCR: Write Enable Line Control Register

wire    BRRE, RTSo;             // LCR: BRR Access Enable, RTS Outputs
wire    [1:0] MD;               // LCR: Serial Interface Operating Mode
wire    [3:0] FMT;              // LCR: Serial Frame Format

reg     Len, NumStop, ParEn;    // Char Length, # Stop Bits, Parity Enable
reg     [1:0] Par;              // Parity Selector

wire    Hold;                   // Tx Hold - RS485 Send held off if asserted

reg     [7:0] IER;              // UART Interrupt Enable Register (IER)
wire    Sel_IER;                // IER: Register Select
wire    WE_IER;                 // IER: Write Enable Interrupt Control Register

wire    IE;                     // IER: Master Interrupt Enable
wire    IE_THE;                 // IER: Enable Tx FIFO Half Empty Interrupt
wire    IE_TEF;                 // IER: Enable Tx FIFO Empty Interrupt
wire    IE_TEM;                 // IER: Enable Tx SR Empty Interrupt
wire    IE_RFE;                 // IER: Enable Receive Framing Error Interrupt
wire    IE_RPE;                 // IER: Enable Receive Parity Error Interrupt
wire    IE_RHF;                 // IER: Enable Rx FIFO Half Full Interrupt
wire    IE_RTO;                 // IER: Enable Receive Time Out Interrupt

reg     [15:0] Div;             // UART Baud Rate Divider Register (DIV)
wire    WE_BRR;                 // DIV: Write Enable Divider Register

wire    [7:0] THR;              // Transmit Holding Register (Tx FIFO Output)
wire    WE_THR;                 // Write Enable - Transmit Holding Register(THR)
wire    TF_FF, TF_EF, TF_HF;    // Transmit FIFO Flags - Full, Empty, Half

wire    [9:0] RD, RHR;          // Receive Data (In), Receive Holding Register
wire    WE_RHR;                 // Write Enable - Receive Holding Register (RHR)
wire    RE_RHR;                 // Read Enable - RHR
wire    RF_EF, RF_HF;           // Receive FIFO Flags - Empty, Half

reg     RxDi;                   // Mode Dependent RxD input

reg     [ 3:0] CCntVal;         // RTO Character Length: {10 | 11 | 12} - 1
wire    [ 3:0] RTOVal;          // RTO Character Delay Value: (N - 1)
wire    RTO;                    // Receive Timeout Flag

////////////////////////////////////////////////////////////////////////////////    
//
//  Implementation
//

//  Break out Register Select Address

assign Sel_THR = Sel & (Reg == pTHR);   // Offset 0
assign Sel_RHR = Sel & (Reg == pRHR);   // Offset 0
assign Sel_LSR = Sel & (Reg == pLSR);   // Offset 1
assign Sel_LCR = Sel & (Reg == pLCR);   // Offset 2
assign Sel_IER = Sel & (Reg == pIER);   // Offset 3

//  Drive UART Data Output Bus

assign DO = ((Sel_RHR) ? ((BRRE) ? Div[ 7:0] : RHR) : 0);   // Offset 0
assign DO = ((Sel_LSR) ? ((BRRE) ? Div[15:8] : LSR) : 0);   // Offset 1
assign DO = ((Sel_LCR) ? LCR                        : 0);   // Offset 2
assign DO = ((Sel_IER) ? IER                        : 0);   // Offset 3

////////////////////////////////////////////////////////////////////////////////
//
//  UART Line Stature Register (LSR) - Read-only
//

assign RE_LSR = RE & Sel_LSR;

assign RxRdy = ((IE_RTO) ? iRTO : ~RF_EF);
assign TxRdy = ~TF_FF;

always @(posedge Clk)
begin
    if(Rst)
        LSR <= #1 0;
    else
        LSR <= #1 {CTSi,                    // CTS Level
                   RTSi,                    // RTS level
                   (iTHE | iTEF | iTEM),    // Tx Interrupt
                   TxRdy,                   // Tx Ready - Tx FIFO not Full
                   iRFE,                    // Rx Framing Error
                   iRPE,                    // Rx Parity Error
                   iRHF,                    // Rx FIFO Half Full or more
                   RxRdy};                  // Rx Ready - Rx FIFO not Empty
end

//  Generate Clear Interrupt LSR Read

always @(posedge Clk)
begin
    if(Rst)
        Clr_Int <= #1 0;
    else 
        Clr_Int <= #1 RE_LSR;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Write UART Line Control Register (LCR) - Read/Write
//

assign WE_LCR = WE & Sel_LCR;

always @(posedge Clk)
begin
    if(Rst)
        LCR <= #1 0;
    else if(WE_LCR)
        LCR <= #1 DI;
end

//  Assign LCR Fields

assign BRRE = LCR[7];               // Select Baud Rate Register
assign RTSo = LCR[6];               // RTS Output in 2-Wire RS-232 Mode
assign MD   = LCR[5:4];             // Operating Mode Select
assign FMT  = LCR[3:0];             // Frame Format Select

//  Format Decode

always @(FMT)
case(FMT)
    4'b0000 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b0, 1'b0, 2'b00};   // 8N1
    4'b0001 : {Len, NumStop, ParEn, Par} <= {1'b1, 1'b0, 1'b1, 2'b00};   // 7O1
    4'b0010 : {Len, NumStop, ParEn, Par} <= {1'b1, 1'b0, 1'b1, 2'b01};   // 7E1
    4'b0011 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b0, 1'b1, 2'b00};   // 8O1
    4'b0100 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b0, 1'b1, 2'b01};   // 8E1
    4'b0101 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b0, 1'b1, 2'b10};   // 8S1
    4'b0110 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b0, 1'b1, 2'b11};   // 8M1
    4'b0111 : {Len, NumStop, ParEn, Par} <= {1'b1, 1'b1, 1'b1, 2'b00};   // 7O2
    4'b1000 : {Len, NumStop, ParEn, Par} <= {1'b1, 1'b1, 1'b1, 2'b01};   // 7E2
    4'b1001 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b1, 1'b0, 2'b00};   // 8N2
    4'b1010 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b1, 1'b1, 2'b00};   // 8O2
    4'b1011 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b1, 1'b1, 2'b01};   // 8E2
    4'b1100 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b1, 1'b1, 2'b10};   // 8S2
    4'b1101 : {Len, NumStop, ParEn, Par} <= {1'b0, 1'b1, 1'b1, 2'b11};   // 8M2
    4'b1110 : {Len, NumStop, ParEn, Par} <= 0;  // 8N1; Unused
    4'b1111 : {Len, NumStop, ParEn, Par} <= 0;  // 8N1; Unused
endcase

//  Receive Timeout Character Frame Length

always @(FMT)
case(FMT)
    4'b0000 : CCntVal <= 4'h9;   // 8N1,  9 <= 10 - 1
    4'b0001 : CCntVal <= 4'h9;   // 7O1,  9 <= 10 - 1
    4'b0010 : CCntVal <= 4'h9;   // 7E1,  9 <= 10 - 1
    4'b0011 : CCntVal <= 4'hA;   // 8O1, 10 <= 11 - 1
    4'b0100 : CCntVal <= 4'hA;   // 8E1, 10 <= 11 - 1
    4'b0101 : CCntVal <= 4'hA;   // 8S1, 10 <= 11 - 1
    4'b0110 : CCntVal <= 4'hA;   // 8M1, 10 <= 11 - 1
    4'b0111 : CCntVal <= 4'h9;   // 7O2,  9 <= 10 - 1
    4'b1000 : CCntVal <= 4'h9;   // 7E2,  9 <= 10 - 1
    4'b1001 : CCntVal <= 4'hA;   // 8N2, 10 <= 11 - 1
    4'b1010 : CCntVal <= 4'hB;   // 8O2, 11 <= 12 - 1
    4'b1011 : CCntVal <= 4'hB;   // 8E2, 11 <= 12 - 1
    4'b1100 : CCntVal <= 4'hB;   // 8S2, 11 <= 12 - 1
    4'b1101 : CCntVal <= 4'hB;   // 8M2, 11 <= 12 - 1
    4'b1110 : CCntVal <= 4'h9;   // 8N1,  9 <= 10 - 1
    4'b1111 : CCntVal <= 4'h9;   // 8N1,  9 <= 10 - 1
endcase

assign RTOVal = (pRTOChrDlyCnt - 1);    // Set RTO Character Delay Count

////////////////////////////////////////////////////////////////////////////////
//
//  Write UART Interrupt Enable Register
//

assign WE_IER = WE & Sel_IER;

always @(posedge Clk)
begin
    if(Rst)
        IER <= #1 0;
    else if(WE_IER)
        IER <= #1 DI;
end

//  Assign LCR Fields

assign IE     = IER[7];                 // Master Interrupt Enable
assign IE_THE = IER[6];                 // Interrupt Enable Tx FIFO Half Empty
assign IE_TEF = IER[5];                 // Interrupt Enable Tx FIFO Empty
assign IE_TEM = IER[4];                 // Interrupt Enable Tx SR Empty
assign IE_RFE = IER[3];                 // Interrupt Enable Rx Framing Error
assign IE_RPE = IER[2];                 // Interrupt Enable Rx Parity Error
assign IE_RHF = IER[1];                 // Interrupt Enable Rx FIFO Half Full
assign IE_RTO = IER[0];                 // Interrupt Enable Rx Time Out

//  Assert IRQ when IE is set

assign Rst_IRQ = Rst | Clr_Int | ~IE;

always @(posedge Clk)
begin
    if(Rst_IRQ)
        IRQ <= #1 0;
    else if(~IRQ)
        IRQ <= #1 IE & (  iTHE & IE_THE
                        | iTEF & IE_TEF
                        | iTEM & IE_TEM
                        | iRFE & IE_RFE
                        | iRPE & IE_RPE
                        | iRHF & IE_RHF
                        | iRTO & IE_RTO );
end

////////////////////////////////////////////////////////////////////////////////
//
//  BRR - Baud Rate Register
//

assign WE_BRR = WE & (Sel_THR | Sel_LSR) & BRRE;

always @(posedge Clk)
begin
    if(Rst)
        Div <= #1 pDiv_Default;        // Default: 9600 bps
    else if(WE_BRR) begin
        Div[ 7:0] <= #1 ((Sel_THR) ? DI : Div[ 7:0]);
        Div[15:8] <= #1 ((Sel_LSR) ? DI : Div[15:8]);
    end
end

////////////////////////////////////////////////////////////////////////////////
//
//  Xmt/Rcv Holding Register Instantiations - Dual-Port Synchronous FIFOs
//
//  THR FIFO - 2**(pTFLen + 4) x pWidth FIFO

assign WE_THR = WE & Sel_THR & (~BRRE);

DPSFnmCE    #(
                .addr((pTF_Depth + 4)),
                .width(pWidth),
                .init(pTF_Init)
            ) TF1 (
                .Rst(Rst), 
                .Clk(Clk), 

                .WE(WE_THR), 
                .DI(DI), 

                .RE(RE_THR), 
                .DO(THR), 

                .FF(TF_FF),
                .HF(TF_HF), 
                .EF(TF_EF), 
                .Cnt()
            );

//  RHR FIFO - 2**(pRFLen + 4) x (pWidth + 1) FIFO

assign RE_RHR = Sel_RHR & RE;

DPSFnmCE    #(
                .addr((pRF_Depth + 4)),
                .width((pWidth + 2)),
                .init(pRF_Init)
            ) RF1 (
                .Rst(Rst), 
                .Clk(Clk), 

                .WE(WE_RHR), 
                .DI(RD), 

                .RE(RE_RHR), 
                .DO(RHR), 

                .FF(),
                .HF(RF_HF), 
                .EF(RF_EF), 
                .Cnt()
            );

////////////////////////////////////////////////////////////////////////////////
//
//  Configure external/internal serial port signals according to MD[1:0]
//      MD[1:0] = 0,1 - RS-232; 2,3 - RS-485

assign RS485 =  MD[1];
assign RS232 = ~RS485;
assign Mode  =  RS485;

//  Assert DE in the RS-485 modes whenever the TxSM is not idle, and deassert
//      whenever the RS-485 modes are not selected

assign xDE = (RS485 ? ~TxIdle : 0);

//  Connect the UART's RxD serial input to the appropriate external RxD input
//      Hold RxD to logic 1 when in the RS-485 w/o Loopback mode and the TxSM
//      is transmitting data. In this manner, the external xOE signal to the 
//      RS-485 transceiver can always be asserted.

always @(posedge Clk or posedge Rst)
begin
    if(Rst)
        RxDi <= #1 1;
    else
        case(MD)
            2'b00 : RxDi <= #1 RxD;
            2'b01 : RxDi <= #1 RxD;
            2'b10 : RxDi <= #1 (TxIdle ? RxD : 1);
            2'b11 : RxDi <= #1 RxD;
        endcase
end

// RS-232 auto-Handshaking is implemented as Ready-To-Receive (RTR) based on
//      the Rcv FIFO flag settings. xRTS, which should connect to the receiving
//      side's xCTS, is asserted whenever the local receive FIFO is less than 
//      half full. If a similar UART with hardware handshaking is connected,
//      then that transmitter should stop sending until the local FIFO is read
//      so that it is below the HF mark. Since local reads of the receive FIFO
//      are expected to be much faster than the RS-232 baud rate, it is not 
//      expected that hysteresis will be required to prevent rapid assertion
//      and deassertion of RTS.
//
//      This handshaking mechanism was selected for the automatic handshaking
//      mode because it prevents (or attempts to prevent) receive FIFO over-
//      flow in the receiver. Furthermore, it reduces the software workload in
//      the transmitter's send routines.
//
//      For all other modes, the CTSi control signal to the UART_TXSM is held
//      at logic one. This effectively disables the TxSM's handshaking logic,
//      and allows the transmitter to send data as soon as data is written to
//      Xmt FIFO.

always @(*)
begin
    case(MD)
        2'b00 : xRTS <= RTSo;
        2'b01 : xRTS <= ~RF_HF;
        2'b10 : xRTS <= 0;
        2'b11 : xRTS <= 0;
    endcase
end

assign RTSi = ((RS232  ) ? xRTS : xDE);
assign CTSi = ((MD == 1) ? xCTS : 1);

////////////////////////////////////////////////////////////////////////////////
//
//  UART Baud Rate Generator Instantiation
//

assign Ld_BRG = BRRE;

UART_BRG    #(
                .pDiv_Default(pDiv_Default)
            )BRG (
                .Rst(Rst), 
                .Clk(Clk),
                
                .Ld(Ld_BRG),
                .Div(Div), 
                
                .CE_16x(CE_16x)
            );

////////////////////////////////////////////////////////////////////////////////
//
//  UART Transmitter State Machine & Shift Register Instantiation
//

assign Hold = MD[1] & ~RTSo;    // Hold Tx when RS485 mode and RTSo not asserted

UART_TXSM   XMT (
                .Rst(Rst), 
                .Clk(Clk), 
                
                .CE_16x(CE_16x), 
                
                .Len(Len), 
                .ParEn(ParEn), 
                .Par(Par),
                .StartDly(5'b0),
                
                .TF_RE(RE_THR), 
                .THR(THR), 
                .TF_EF(TF_EF | Hold), 
                
                .TxD(TxD), 
                .CTSi(CTSi), 
            
                .TxIdle(TxIdle), 
                .TxStart(), 
                .TxShift(), 
                .TxStop()
            );

////////////////////////////////////////////////////////////////////////////////
//
//  UART Receiver State Machine & Shift Register Instantiation
//

UART_RXSM   RCV (
                .Rst(Rst), 
                .Clk(Clk), 
                
                .CE_16x(CE_16x),
                
                .Len(Len), 
                .NumStop(NumStop), 
                .ParEn(ParEn), 
                .Par(Par),
                
                .RxD(RxDi), 
                
                .RD(RD), 
                .WE_RHR(WE_RHR), 
                
                .RxWait(), 
                .RxIdle(RxIdle), 
                .RxStart(), 
                .RxShift(), 
                .RxParity(), 
                .RxStop(), 
                .RxError()
            );

////////////////////////////////////////////////////////////////////////////////
//
//  UART Receive Timeout Module Instantiation
//

UART_RTO    TMR (
                .Rst(Rst), 
                .Clk(Clk),
                
                .CE_16x(CE_16x),
                
                .WE_RHR(WE_RHR), 
                .RE_RHR(RE_RHR),
                
                .CCntVal(CCntVal), 
                .RTOVal(RTOVal),
                
                .RcvTimeout(RTO)
            );

////////////////////////////////////////////////////////////////////////////////
//
//  UART Interrupt Generator Instantiation
//

UART_INT    INT (
                .Rst(Rst), 
                .Clk(Clk), 
                
                .TF_HF(TF_HF), 
                .TF_EF(TF_EF),
                .TxIdle(TxIdle),
                
                .RF_FE(RHR[9]),
                .RF_PE(RHR[8]),
                .RF_HF(RF_HF), 
                .RF_EF(RF_EF),
                
                .RTO(RTO), 
                
                .Clr_Int(Clr_Int),
                
                .iTHE(iTHE),
                .iTEF(iTEF),
                .iTEM(iTEM),
                
                .iRPE(iRPE),
                .iRFE(iRFE),
                .iRHF(iRHF), 
                .iRTO(iRTO)
            );

endmodule
