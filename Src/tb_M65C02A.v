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
reg     [((4*8) - 1):0] sOpcode;    // Opcode Mnemonics String
always @(IR)begin    case(IR)        8'h00 : sOpcode <= "BRK ";        8'h01 : sOpcode <= "ORA ";        8'h02 : sOpcode <= "COP ";        8'h03 : sOpcode <= "ORA ";        8'h04 : sOpcode <= "TSB ";        8'h05 : sOpcode <= "ORA ";        8'h06 : sOpcode <= "ASL ";        8'h07 : sOpcode <= "ORA ";        8'h08 : sOpcode <= "PHP ";        8'h09 : sOpcode <= "ORA ";        8'h0A : sOpcode <= "ASL ";        8'h0B : sOpcode <= "DUP ";        8'h0C : sOpcode <= "TSB ";        8'h0D : sOpcode <= "ORA ";        8'h0E : sOpcode <= "ASL ";        8'h0F : sOpcode <= "ORA ";        8'h10 : sOpcode <= "BPL ";        8'h11 : sOpcode <= "ORA ";        8'h12 : sOpcode <= "ORA ";        8'h13 : sOpcode <= "ORA ";        8'h14 : sOpcode <= "TRB ";        8'h15 : sOpcode <= "ORA ";        8'h16 : sOpcode <= "ASL ";        8'h17 : sOpcode <= "ASL ";        8'h18 : sOpcode <= "CLC ";        8'h19 : sOpcode <= "ORA ";        8'h1A : sOpcode <= "INC ";        8'h1B : sOpcode <= "SWP ";        8'h1C : sOpcode <= "TRB ";        8'h1D : sOpcode <= "ORA ";        8'h1E : sOpcode <= "ASL ";        8'h1F : sOpcode <= "ASL ";        8'h20 : sOpcode <= "JSR ";        8'h21 : sOpcode <= "ANL ";        8'h22 : sOpcode <= "JMP ";        8'h23 : sOpcode <= "ANL ";        8'h24 : sOpcode <= "BIT ";        8'h25 : sOpcode <= "ANL ";        8'h26 : sOpcode <= "ROL ";        8'h27 : sOpcode <= "ANL ";        8'h28 : sOpcode <= "PLP ";        8'h29 : sOpcode <= "ANL ";        8'h2A : sOpcode <= "ROL ";        8'h2B : sOpcode <= "ROT ";        8'h2C : sOpcode <= "BIT ";        8'h2D : sOpcode <= "ANL ";        8'h2E : sOpcode <= "ROL ";        8'h2F : sOpcode <= "ANL ";        8'h30 : sOpcode <= "BMI ";        8'h31 : sOpcode <= "ANL ";        8'h32 : sOpcode <= "ANL ";        8'h33 : sOpcode <= "ANL ";        8'h34 : sOpcode <= "BIT ";        8'h35 : sOpcode <= "ANL ";        8'h36 : sOpcode <= "ROL ";        8'h37 : sOpcode <= "ROL";        8'h38 : sOpcode <= "SEC ";        8'h39 : sOpcode <= "ANL ";        8'h3A : sOpcode <= "DEC ";        8'h3B : sOpcode <= "NXT ";        8'h3C : sOpcode <= "BIT ";        8'h3D : sOpcode <= "ANL ";        8'h3E : sOpcode <= "ROL ";        8'h3F : sOpcode <= "ROL";        8'h40 : sOpcode <= "RTI ";        8'h41 : sOpcode <= "EOR ";        8'h42 : sOpcode <= "MOV ";        8'h43 : sOpcode <= "EOR ";        8'h44 : sOpcode <= "STA ";        8'h45 : sOpcode <= "EOR ";        8'h46 : sOpcode <= "LSR ";        8'h47 : sOpcode <= "EOR";        8'h48 : sOpcode <= "PHA ";        8'h49 : sOpcode <= "EOR ";        8'h4A : sOpcode <= "LSR ";        8'h4B : sOpcode <= "PHI ";        8'h4C : sOpcode <= "JMP ";        8'h4D : sOpcode <= "EOR ";        8'h4E : sOpcode <= "LSR ";        8'h4F : sOpcode <= "EOR ";        8'h50 : sOpcode <= "BVC ";        8'h51 : sOpcode <= "EOR ";        8'h52 : sOpcode <= "EOR ";        8'h53 : sOpcode <= "EOR ";        8'h54 : sOpcode <= "LDA ";        8'h55 : sOpcode <= "EOR ";        8'h56 : sOpcode <= "LSR ";        8'h57 : sOpcode <= "LSR";        8'h58 : sOpcode <= "CLI ";        8'h59 : sOpcode <= "EOR ";        8'h5A : sOpcode <= "PHY ";        8'h5B : sOpcode <= "INI ";        8'h5C : sOpcode <= "XMA ";        8'h5D : sOpcode <= "EOR ";        8'h5E : sOpcode <= "LSR ";        8'h5F : sOpcode <= "LSR";        8'h60 : sOpcode <= "RTS ";        8'h61 : sOpcode <= "ADC ";        8'h62 : sOpcode <= "PHR ";        8'h63 : sOpcode <= "ADC ";        8'h64 : sOpcode <= "STZ ";        8'h65 : sOpcode <= "ADC ";        8'h66 : sOpcode <= "ROR ";        8'h67 : sOpcode <= "ADC ";        8'h68 : sOpcode <= "PLA ";        8'h69 : sOpcode <= "ADC ";        8'h6A : sOpcode <= "ROR ";        8'h6B : sOpcode <= "PLI ";        8'h6C : sOpcode <= "JMP ";        8'h6D : sOpcode <= "ADC ";        8'h6E : sOpcode <= "ROR ";        8'h6F : sOpcode <= "ADD ";        8'h70 : sOpcode <= "BVS ";        8'h71 : sOpcode <= "ADC ";        8'h72 : sOpcode <= "ADC ";        8'h73 : sOpcode <= "ADC ";        8'h74 : sOpcode <= "STZ ";        8'h75 : sOpcode <= "ADC ";        8'h76 : sOpcode <= "ROR ";        8'h77 : sOpcode <= "ROR ";        8'h78 : sOpcode <= "SEI ";        8'h79 : sOpcode <= "ADC ";        8'h7A : sOpcode <= "PLY ";        8'h7B : sOpcode <= "ENT ";        8'h7C : sOpcode <= "JMP ";        8'h7D : sOpcode <= "ADC ";        8'h7E : sOpcode <= "ROR ";        8'h7F : sOpcode <= "ROR ";        8'h80 : sOpcode <= "BRA ";        8'h81 : sOpcode <= "STA ";        8'h82 : sOpcode <= "JMP ";        8'h83 : sOpcode <= "STA ";        8'h84 : sOpcode <= "STY ";        8'h85 : sOpcode <= "STA ";        8'h86 : sOpcode <= "STX ";        8'h87 : sOpcode <= "STA ";        8'h88 : sOpcode <= "DEY ";        8'h89 : sOpcode <= "BIT ";        8'h8A : sOpcode <= "TXA ";        8'h8B : sOpcode <= "OSX ";        8'h8C : sOpcode <= "STY ";        8'h8D : sOpcode <= "STA ";        8'h8E : sOpcode <= "STX ";        8'h8F : sOpcode <= "STA ";        8'h90 : sOpcode <= "BCC ";        8'h91 : sOpcode <= "STA ";        8'h92 : sOpcode <= "STA ";        8'h93 : sOpcode <= "STA ";        8'h94 : sOpcode <= "STY ";        8'h95 : sOpcode <= "STA ";        8'h96 : sOpcode <= "STX ";        8'h97 : sOpcode <= "TSB ";        8'h98 : sOpcode <= "TYA ";        8'h99 : sOpcode <= "STA ";        8'h9A : sOpcode <= "TXS ";        8'h9B : sOpcode <= "IND ";        8'h9C : sOpcode <= "STZ ";        8'h9D : sOpcode <= "STA ";        8'h9E : sOpcode <= "STZ ";        8'h9F : sOpcode <= "TSB ";        8'hA0 : sOpcode <= "LDY ";        8'hA1 : sOpcode <= "LDA ";        8'hA2 : sOpcode <= "LDX ";        8'hA3 : sOpcode <= "LDA ";        8'hA4 : sOpcode <= "LDY ";        8'hA5 : sOpcode <= "LDA ";        8'hA6 : sOpcode <= "LDX ";        8'hA7 : sOpcode <= "LDA ";        8'hA8 : sOpcode <= "TAY ";        8'hA9 : sOpcode <= "LDA ";        8'hAA : sOpcode <= "TAX ";        8'hAB : sOpcode <= "SIZ ";        8'hAC : sOpcode <= "LDY ";        8'hAD : sOpcode <= "LDA ";        8'hAE : sOpcode <= "LDX ";        8'hAF : sOpcode <= "LDA ";        8'hB0 : sOpcode <= "BCS ";        8'hB1 : sOpcode <= "LDA ";        8'hB2 : sOpcode <= "LDA ";        8'hB3 : sOpcode <= "LDA ";        8'hB4 : sOpcode <= "LDY ";        8'hB5 : sOpcode <= "LDA ";        8'hB6 : sOpcode <= "LDX ";        8'hB7 : sOpcode <= "TRB ";        8'hB8 : sOpcode <= "CLV ";        8'hB9 : sOpcode <= "LDA ";        8'hBA : sOpcode <= "TSX ";        8'hBB : sOpcode <= "ISZ ";        8'hBC : sOpcode <= "LDY ";        8'hBD : sOpcode <= "LDA ";        8'hBE : sOpcode <= "LDX ";        8'hBF : sOpcode <= "TRB ";        8'hC0 : sOpcode <= "CPY ";        8'hC1 : sOpcode <= "CMP ";        8'hC2 : sOpcode <= "PUL ";        8'hC3 : sOpcode <= "CMP ";        8'hC4 : sOpcode <= "CPY ";        8'hC5 : sOpcode <= "CMP ";        8'hC6 : sOpcode <= "DEC ";        8'hC7 : sOpcode <= "CMP ";        8'hC8 : sOpcode <= "INY ";        8'hC9 : sOpcode <= "CMP ";        8'hCA : sOpcode <= "DEX ";        8'hCB : sOpcode <= "WAI ";        8'hCC : sOpcode <= "CPY ";        8'hCD : sOpcode <= "CMP ";        8'hCE : sOpcode <= "DEC ";        8'hCF : sOpcode <= "CMP ";        8'hD0 : sOpcode <= "BNE ";        8'hD1 : sOpcode <= "CMP ";        8'hD2 : sOpcode <= "CMP ";        8'hD3 : sOpcode <= "CMP ";        8'hD4 : sOpcode <= "PSH ";        8'hD5 : sOpcode <= "CMP ";        8'hD6 : sOpcode <= "DEC ";        8'hD7 : sOpcode <= "DEC ";        8'hD8 : sOpcode <= "CLD ";        8'hD9 : sOpcode <= "CMP ";        8'hDA : sOpcode <= "PHX ";        8'hDB : sOpcode <= "STP ";        8'hDC : sOpcode <= "ADJ ";        8'hDD : sOpcode <= "CMP ";        8'hDE : sOpcode <= "DEC ";        8'hDF : sOpcode <= "DEC ";        8'hE0 : sOpcode <= "CPX ";        8'hE1 : sOpcode <= "SBC ";        8'hE2 : sOpcode <= "PUL ";        8'hE3 : sOpcode <= "SBC ";        8'hE4 : sOpcode <= "CPX ";        8'hE5 : sOpcode <= "SBC ";        8'hE6 : sOpcode <= "INC ";        8'hE7 : sOpcode <= "SBC ";        8'hE8 : sOpcode <= "INX ";        8'hE9 : sOpcode <= "SBC ";        8'hEA : sOpcode <= "NOP ";        8'hEB : sOpcode <= "OAX ";        8'hEC : sOpcode <= "CPX ";        8'hED : sOpcode <= "SBC ";        8'hEE : sOpcode <= "INC ";        8'hEF : sOpcode <= "SUB ";        8'hF0 : sOpcode <= "BEQ ";        8'hF1 : sOpcode <= "SBC ";        8'hF2 : sOpcode <= "SBC ";        8'hF3 : sOpcode <= "SBC ";        8'hF4 : sOpcode <= "PSH ";        8'hF5 : sOpcode <= "SBC ";        8'hF6 : sOpcode <= "INC ";        8'hF7 : sOpcode <= "INC ";        8'hF8 : sOpcode <= "SED ";        8'hF9 : sOpcode <= "SBC ";        8'hFA : sOpcode <= "PLX ";        8'hFB : sOpcode <= "OAY ";        8'hFC : sOpcode <= "PSH ";        8'hFD : sOpcode <= "SBC ";        8'hFE : sOpcode <= "INC ";        8'hFF : sOpcode <= "INC ";    endcaseend
//  Convert IR to Addressing Mode Mnemonics
reg     [((8*8) - 1):0] sAddrMd;   // Addressing Mode Mnemonics String
always @(IR)begin    case(IR)        8'h00 : sAddrMd <= "  #imm  ";        8'h01 : sAddrMd <= " (zp,X) ";        8'h02 : sAddrMd <= "  #imm  ";        8'h03 : sAddrMd <= "  bp,B  ";        8'h04 : sAddrMd <= "   zp   ";        8'h05 : sAddrMd <= "   zp   ";        8'h06 : sAddrMd <= "   zp   ";        8'h07 : sAddrMd <= "  sp,S  ";        8'h08 : sAddrMd <= "  imp   ";        8'h09 : sAddrMd <= "  #imm  ";        8'h0A : sAddrMd <= "  acc   ";        8'h0B : sAddrMd <= "   -    ";        8'h0C : sAddrMd <= "  abs   ";        8'h0D : sAddrMd <= "  abs   ";        8'h0E : sAddrMd <= "  abs   ";        8'h0F : sAddrMd <= " ip,I++ ";        8'h10 : sAddrMd <= "  rel   ";        8'h11 : sAddrMd <= " (zp),Y ";        8'h12 : sAddrMd <= "  (zp)  ";        8'h13 : sAddrMd <= "(bp,B),Y";        8'h14 : sAddrMd <= "   zp   ";        8'h15 : sAddrMd <= "  zp,X  ";        8'h16 : sAddrMd <= "  zp,X  ";        8'h17 : sAddrMd <= "  bp,B  ";        8'h18 : sAddrMd <= "  imp   ";        8'h19 : sAddrMd <= "  abs,Y ";        8'h1A : sAddrMd <= "  acc   ";        8'h1B : sAddrMd <= "   -    ";        8'h1C : sAddrMd <= "  abs   ";        8'h1D : sAddrMd <= "  abs,X ";        8'h1E : sAddrMd <= "  abs,X ";        8'h1F : sAddrMd <= " ip,I++ ";        8'h20 : sAddrMd <= "  abs   ";        8'h21 : sAddrMd <= " (zp,X) ";        8'h22 : sAddrMd <= "  (zp)  ";        8'h23 : sAddrMd <= "  bp,B  ";        8'h24 : sAddrMd <= "   zp   ";        8'h25 : sAddrMd <= "   zp   ";        8'h26 : sAddrMd <= "   zp   ";        8'h27 : sAddrMd <= "  sp,S  ";        8'h28 : sAddrMd <= "  imp   ";        8'h29 : sAddrMd <= "  #imm  ";        8'h2A : sAddrMd <= "  acc   ";        8'h2B : sAddrMd <= "   -    ";        8'h2C : sAddrMd <= "  abs   ";        8'h2D : sAddrMd <= "  abs   ";        8'h2E : sAddrMd <= "  abs   ";        8'h2F : sAddrMd <= " ip,I++ ";        8'h30 : sAddrMd <= "  rel   ";        8'h31 : sAddrMd <= " (zp),Y ";        8'h32 : sAddrMd <= "  (zp)  ";        8'h33 : sAddrMd <= "(bp,B),Y";        8'h34 : sAddrMd <= "  zp,X  ";        8'h35 : sAddrMd <= "  zp,X  ";        8'h36 : sAddrMd <= "  zp,X  ";        8'h37 : sAddrMd <= "  bp,B  ";        8'h38 : sAddrMd <= "  imp   ";        8'h39 : sAddrMd <= "  abs,Y ";        8'h3A : sAddrMd <= "  acc   ";        8'h3B : sAddrMd <= "   -    ";        8'h3C : sAddrMd <= "  abs,X ";        8'h3D : sAddrMd <= "  abs,X ";        8'h3E : sAddrMd <= "  abs,X ";        8'h3F : sAddrMd <= " ip,I++ ";        8'h40 : sAddrMd <= "  imp   ";        8'h41 : sAddrMd <= " (zp,X) ";        8'h42 : sAddrMd <= "  sm,dm ";        8'h43 : sAddrMd <= "  bp,B  ";        8'h44 : sAddrMd <= "   ---  ";        8'h45 : sAddrMd <= "   zp   ";        8'h46 : sAddrMd <= "   zp   ";        8'h47 : sAddrMd <= "  sp,S  ";        8'h48 : sAddrMd <= "  imp   ";        8'h49 : sAddrMd <= "  #imm  ";        8'h4A : sAddrMd <= "  acc   ";        8'h4B : sAddrMd <= "   -    ";        8'h4C : sAddrMd <= "  abs   ";        8'h4D : sAddrMd <= "  abs   ";        8'h4E : sAddrMd <= "  abs   ";        8'h4F : sAddrMd <= " ip,I++ ";        8'h50 : sAddrMd <= "  rel   ";        8'h51 : sAddrMd <= " (zp),Y ";        8'h52 : sAddrMd <= "  (zp)  ";        8'h53 : sAddrMd <= "(bp,B),Y";        8'h54 : sAddrMd <= "   ---  ";        8'h55 : sAddrMd <= "  zp,X  ";        8'h56 : sAddrMd <= "  zp,X  ";        8'h57 : sAddrMd <= "  bp,B  ";        8'h58 : sAddrMd <= "  imp   ";        8'h59 : sAddrMd <= "  abs,Y ";        8'h5A : sAddrMd <= "  imp   ";        8'h5B : sAddrMd <= "   -    ";        8'h5C : sAddrMd <= "  sp,S  ";        8'h5D : sAddrMd <= "  abs,X ";        8'h5E : sAddrMd <= "  abs,X ";        8'h5F : sAddrMd <= " ip,I++ ";        8'h60 : sAddrMd <= "  imp   ";        8'h61 : sAddrMd <= " (zp,X) ";        8'h62 : sAddrMd <= "  rel16 ";        8'h63 : sAddrMd <= "  bp,B  ";        8'h64 : sAddrMd <= "   zp   ";        8'h65 : sAddrMd <= "   zp   ";        8'h66 : sAddrMd <= "   zp   ";        8'h67 : sAddrMd <= "  sp,S  ";        8'h68 : sAddrMd <= "  imp   ";        8'h69 : sAddrMd <= "  #imm  ";        8'h6A : sAddrMd <= "  acc   ";        8'h6B : sAddrMd <= "   -    ";        8'h6C : sAddrMd <= " (abs)  ";        8'h6D : sAddrMd <= "  abs   ";        8'h6E : sAddrMd <= "  abs   ";        8'h6F : sAddrMd <= " ip,I++ ";        8'h70 : sAddrMd <= "  rel   ";        8'h71 : sAddrMd <= " (zp),Y ";        8'h72 : sAddrMd <= "  (zp)  ";        8'h73 : sAddrMd <= "(bp,B),Y";        8'h74 : sAddrMd <= "  zp,X  ";        8'h75 : sAddrMd <= "  zp,X  ";        8'h76 : sAddrMd <= "  zp,X  ";        8'h77 : sAddrMd <= "  bp,B  ";        8'h78 : sAddrMd <= "  imp   ";        8'h79 : sAddrMd <= "  abs,Y ";        8'h7A : sAddrMd <= "  imp   ";        8'h7B : sAddrMd <= "   -    ";        8'h7C : sAddrMd <= " (abs,X)";        8'h7D : sAddrMd <= "  abs,X ";        8'h7E : sAddrMd <= "  abs,X ";        8'h7F : sAddrMd <= " ip,I++ ";        8'h80 : sAddrMd <= "  rel   ";        8'h81 : sAddrMd <= " (zp,X) ";        8'h82 : sAddrMd <= "(bp,B),Y";        8'h83 : sAddrMd <= "  bp,B  ";        8'h84 : sAddrMd <= "   zp   ";        8'h85 : sAddrMd <= "   zp   ";        8'h86 : sAddrMd <= "   zp   ";        8'h87 : sAddrMd <= "  sp,S  ";        8'h88 : sAddrMd <= "  imp   ";        8'h89 : sAddrMd <= "  #imm  ";        8'h8A : sAddrMd <= "  imp   ";        8'h8B : sAddrMd <= "   -    ";        8'h8C : sAddrMd <= "  abs   ";        8'h8D : sAddrMd <= "  abs   ";        8'h8E : sAddrMd <= "  abs   ";        8'h8F : sAddrMd <= " ip,I++ ";        8'h90 : sAddrMd <= "  rel   ";        8'h91 : sAddrMd <= " (zp),Y ";        8'h92 : sAddrMd <= "  (zp)  ";        8'h93 : sAddrMd <= "(bp,B),Y";        8'h94 : sAddrMd <= "  zp,X  ";        8'h95 : sAddrMd <= "  zp,X  ";        8'h96 : sAddrMd <= "  zp,Y  ";        8'h97 : sAddrMd <= "  bp,B  ";        8'h98 : sAddrMd <= "  imp   ";        8'h99 : sAddrMd <= "  abs,Y ";        8'h9A : sAddrMd <= "  imp   ";        8'h9B : sAddrMd <= "   -    ";        8'h9C : sAddrMd <= "  abs   ";        8'h9D : sAddrMd <= "  abs,X ";        8'h9E : sAddrMd <= "  abs,X ";        8'h9F : sAddrMd <= " ip,I++ ";        8'hA0 : sAddrMd <= "  #imm  ";        8'hA1 : sAddrMd <= " (zp,X) ";        8'hA2 : sAddrMd <= "  #imm  ";        8'hA3 : sAddrMd <= "  bp,B  ";        8'hA4 : sAddrMd <= "   zp   ";        8'hA5 : sAddrMd <= "   zp   ";        8'hA6 : sAddrMd <= "   zp   ";        8'hA7 : sAddrMd <= "  sp,S  ";        8'hA8 : sAddrMd <= "  imp   ";        8'hA9 : sAddrMd <= "  #imm  ";        8'hAA : sAddrMd <= "  imp   ";        8'hAB : sAddrMd <= "   -    ";        8'hAC : sAddrMd <= "  abs   ";        8'hAD : sAddrMd <= "  abs   ";        8'hAE : sAddrMd <= "  abs   ";        8'hAF : sAddrMd <= " ip,I++ ";        8'hB0 : sAddrMd <= "  rel   ";        8'hB1 : sAddrMd <= " (zp),Y ";        8'hB2 : sAddrMd <= "  (zp)  ";        8'hB3 : sAddrMd <= "(bp,B),Y";        8'hB4 : sAddrMd <= "  zp,X  ";        8'hB5 : sAddrMd <= "  zp,X  ";        8'hB6 : sAddrMd <= "  zp,Y  ";        8'hB7 : sAddrMd <= "  bp,B  ";        8'hB8 : sAddrMd <= "  imp   ";        8'hB9 : sAddrMd <= "  abs,Y ";        8'hBA : sAddrMd <= "  imp   ";        8'hBB : sAddrMd <= "   -    ";        8'hBC : sAddrMd <= "  abs,X ";        8'hBD : sAddrMd <= "  abs,X ";        8'hBE : sAddrMd <= "  abs,Y ";        8'hBF : sAddrMd <= " ip,I++ ";        8'hC0 : sAddrMd <= "  #imm  ";        8'hC1 : sAddrMd <= " (zp,X) ";        8'hC2 : sAddrMd <= "   zp   ";        8'hC3 : sAddrMd <= "  bp,B  ";        8'hC4 : sAddrMd <= "   zp   ";        8'hC5 : sAddrMd <= "   zp   ";        8'hC6 : sAddrMd <= "   zp   ";        8'hC7 : sAddrMd <= "  sp,S  ";        8'hC8 : sAddrMd <= "  imp   ";        8'hC9 : sAddrMd <= "  #imm  ";        8'hCA : sAddrMd <= "  imp   ";        8'hCB : sAddrMd <= "   -    ";        8'hCC : sAddrMd <= "  abs   ";        8'hCD : sAddrMd <= "  abs   ";        8'hCE : sAddrMd <= "  abs   ";        8'hCF : sAddrMd <= " ip,I++ ";        8'hD0 : sAddrMd <= "  rel   ";        8'hD1 : sAddrMd <= " (zp),Y ";        8'hD2 : sAddrMd <= "  (zp)  ";        8'hD3 : sAddrMd <= "(bp,B),Y";        8'hD4 : sAddrMd <= "   zp   ";        8'hD5 : sAddrMd <= "  zp,X  ";        8'hD6 : sAddrMd <= "  zp,X  ";        8'hD7 : sAddrMd <= "  bp,B  ";        8'hD8 : sAddrMd <= "  imp   ";        8'hD9 : sAddrMd <= "  abs,Y ";        8'hDA : sAddrMd <= "  imp   ";        8'hDB : sAddrMd <= "   -    ";        8'hDC : sAddrMd <= "   -    ";        8'hDD : sAddrMd <= "  abs,X ";        8'hDE : sAddrMd <= "  abs,X ";        8'hDF : sAddrMd <= " ip,I++ ";        8'hE0 : sAddrMd <= "  #imm  ";        8'hE1 : sAddrMd <= " (zp,X) ";        8'hE2 : sAddrMd <= "  abs   ";        8'hE3 : sAddrMd <= "  bp,B  ";        8'hE4 : sAddrMd <= "   zp   ";        8'hE5 : sAddrMd <= "   zp   ";        8'hE6 : sAddrMd <= "   zp   ";        8'hE7 : sAddrMd <= "  sp,S  ";        8'hE8 : sAddrMd <= "  imp   ";        8'hE9 : sAddrMd <= "  #imm  ";        8'hEA : sAddrMd <= "   -    ";        8'hEB : sAddrMd <= "   -    ";        8'hEC : sAddrMd <= "  abs   ";        8'hED : sAddrMd <= "  abs   ";        8'hEE : sAddrMd <= "  abs   ";        8'hEF : sAddrMd <= " ip,I++ ";        8'hF0 : sAddrMd <= "  rel   ";        8'hF1 : sAddrMd <= " (zp),Y ";        8'hF2 : sAddrMd <= "  (zp)  ";        8'hF3 : sAddrMd <= "(bp,B),Y";        8'hF4 : sAddrMd <= " #imm16 ";        8'hF5 : sAddrMd <= "  zp,X  ";        8'hF6 : sAddrMd <= "  zp,X  ";        8'hF7 : sAddrMd <= "  bp,B  ";        8'hF8 : sAddrMd <= "  imp   ";        8'hF9 : sAddrMd <= "  abs,Y ";        8'hFA : sAddrMd <= "  imp   ";        8'hFB : sAddrMd <= "   -    ";        8'hFC : sAddrMd <= "  abs   ";        8'hFD : sAddrMd <= "  abs,X ";        8'hFE : sAddrMd <= "  abs,X ";        8'hFF : sAddrMd <= " ip,I++ ";        endcaseend

endmodule
