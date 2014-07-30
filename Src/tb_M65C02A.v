`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:44:19 07/04/2014
// Design Name:   M65C02A
// Module Name:   C:/XProjects/ISE10.1i/M65C02A/Src/tb_M65C02A.v
// Project Name:  M65C02A
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: M65C02A
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
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

M65C02A uut (
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

