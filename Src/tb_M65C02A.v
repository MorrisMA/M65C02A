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

reg     nWait;

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

//  Internal Register Probes

reg    [15:0] X;
reg    [15:0] Y;
reg    [15:0] A;
//reg    [15:0] T;
reg    [ 7:0] P;
reg    [15:0] S;
reg    [15:0] M;

// Instantiate the Unit Under Test (UUT)

M65C02A #(
            .pMON_File("Pgms/M65C02_Tst3.txt"),
//            .pMON_File("Pgms/Mon65_SBC25.txt"),

//            .pRAM_File("Pgms/65C02_ft2.txt"),
            .pRAM_File("Pgms/65C02_ft.txt"),
//            .pRAM_File("Pgms/fig6502a.txt"),

            .pFrequency(29491200),
            .pUART_BRG(921600),
            .pM65C02A_uPgm("Pgms/M65C02A_uPgm_ROM.coe"),        // SEQ   :  2 kB
            .pM65C02A_IDec("Pgms/M65C02A_IDecode_ROM.coe")      // DEC   :  2 kB
        ) uut (
            .nRst(nRst), 
            .Clk(Clk), 
            
            .nNMI(nNMI), 
            .nIRQ(nIRQ), 
            .nVP(nVP),

            .nSO(nSO),
            
            .Sync(Sync), 
            .nML(nML), 

            .nWait(nWait), 

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
    nWait     = 1;
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

always #5 Clk = ~Clk;
//always #16.954 Clk = ~Clk;      // Clk = 29.4912 MHz

always @(posedge Clk)
begin
    if(uut.IO_Op[0])
        if(uut.VA == 16'hFFF8)
            nIRQ = #1 0;
        else if(uut.VA == 16'hFFF9)
            nIRQ = #1 1;
end

//  Probe Internal Processor Registers

always @(uut.X or uut.Y or uut.A or uut.P or uut.S or uut.M)
begin
    X = uut.X;
    Y = uut.Y;
    A = uut.A;
    P = uut.P;
    S = uut.S;
    M = uut.M;
end

//  Probe 6502_Functional_Test Test Number

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

//  fig-FORTH Trace Support Registers

reg [15:0] tir_adr;
reg [ 7:0] IR;

reg [ 7:0] tdata_r, tdata_w;
reg [ 7:0] txsdat_r, txsdat_w;

reg [15:0] tipdat_r;
reg [15:0] tipdat_w;

reg [15:0] tw_dat_r;
reg [15:0] tw_dat_w;

parameter pXSADR = 16'h00CD;
parameter pIPADR = 16'h00C6;
parameter pW_ADR = 16'h00C9;

always @(posedge Clk)
begin
    if(uut.IO_Op[1]) begin
        if(uut.Sync) begin
            tir_adr <= #1 uut.VA;
            IR      <= #1 uut.CPU_DI;
            if(IR == 8'h00) begin
                if(4'hF == tir_adr[15:12]) begin
                    $display("\tExpected BRK: PC=%x\n", tir_adr);
                end else begin
                    $display("\tUnexpected BRK: PC=%x\n", tir_adr);
                    $stop;
                end
            end
        end
        
        if(uut.VA[15:9] == 7'h00) begin
            tdata_r <= #1 uut.CPU_DI;
        end
                
        if(uut.VA[15:0] == pXSADR) begin
            txsdat_r <= #1 uut.CPU_DI;
        end

        if(uut.VA[15:0] == pIPADR) begin
            tipdat_r[ 7:0] <= #1 uut.CPU_DI;
        end

        if(uut.VA[15:0] == (pIPADR + 1)) begin
            tipdat_r[15:8] <= #1 uut.CPU_DI;
        end
        
        if(uut.VA[15:0] == pW_ADR) begin
            tw_dat_r[ 7:0] <= #1 uut.CPU_DI;
        end

        if(uut.VA[15:0] == (pW_ADR + 1)) begin
            tw_dat_r[15:8] <= #1 uut.CPU_DI;
        end
    end

    if(uut.IO_Op[0]) begin
        if(uut.VA[15:9] == 7'h00) begin
            tdata_w <= #1 uut.CPU_DO;
        end
                
        if(uut.VA[15:0] == pXSADR) begin
            txsdat_w <= #1 uut.CPU_DO;
        end

        if(uut.VA[15:0] == pIPADR) begin
            tipdat_w[ 7:0] <= #1 uut.CPU_DO;
        end

        if(uut.VA[15:0] == (pIPADR + 1)) begin
            tipdat_w[15:8] <= #1 uut.CPU_DO;
        end
        
        if(uut.VA[15:0] == pW_ADR) begin
            tw_dat_w[ 7:0] <= #1 uut.CPU_DO;
        end

        if(uut.VA[15:0] == (pW_ADR + 1)) begin
            tw_dat_w[15:8] <= #1 uut.CPU_DO;
        end
    end
end

//  Display Data Written to COM0 (0xFF80)

reg [7:0] TD;

always @(posedge Clk)
begin
    if(uut.IO_Op[0]) begin
        if(uut.VA == 16'hFF80) begin
            TD <= #1 uut.CPU_DO;
        end
    end
end

//  Convert IR to Instruction Mnemonics

always @(IR)
//  Convert IR to Addressing Mode Mnemonics

always @(IR)

endmodule