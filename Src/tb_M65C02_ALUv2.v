////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2012-2013 by Michael A. Morris, dba M. A. Morris & Associates
//
//  All rights reserved. The source code contained herein is publicly released
//  under the terms an conditions of the GNU Lesser Public License. No part of
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
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
//
// Create Date:     09:02:48 11/28/2009
// Design Name:     M65C02_ALUv2
// Module Name:     C:/XProjects/ISE10.1i/M6502A/tb_M65C02_ALUv2.v
// Project Name:    M6502A
// Target Device:   SRAM-based FPGA  
// Tool versions:   Xilinx ISE 10.1i SP3
//  
// Description: 
//
// Verilog Test Fixture created by ISE for module: M65C02_ALUv2
//
// Dependencies:
// 
// Revision:
// 
//  0.01    09K28   MAM     Initial coding
//
// Additional Comments:
//
//  The simulation performs checks against expected results. A test vector file
//  could have been used. However, the vector would have been more difficult to
//  interpret and construct. Although this implementation is a brute force
//  approach, the test vectors as defined in this file can be used to construct
//  the "fixed" microprogram ROM. Only the En input field does not belong in
//  the "fixed" microcode ROM, i.e. instruction decoder ROM.
//
////////////////////////////////////////////////////////////////////////////////

module tb_M65C02_ALUv2;

`include "Src/M65C02_ALUv2_Def.txt"

////////////////////////////////////////////////////////////////////////////////

reg     Rst;
reg     Clk;

// Internal Ready

reg     Rdy;            // Operand Ready

// Execution Control

wire    En;             // ALU Enable
reg     [2:0] Reg_WE;   // ALU Register Write Enable
reg     ISR;            // Interrup Service Routine Flag

//  Set oVerflow Interface

reg     SO;             // Set oVerflow Bit, i.e. PSW.V
wire    Clr_SO;         // Clear SO command - acknowledge

// Stack Interface

wire    SelS;           // ALU Select Stack Pointer 
reg     [7:0] S;        // AddrGen Stack Pointer

// ALU Interface

reg     [5:0] FU_Sel;   // ALU Functional Unit Select
reg     [1:0] Op;       // ALU Functional Unit Operation Select
reg     [1:0] QSel;     // ALU Q/L Bus Select
reg     [1:0] RSel;     // ALU R Bus Select
reg     [1:0] CSel;     // ALU Carry In Select
reg     [2:0] WSel;     // ALU Register Write Select
reg     [2:0] OSel;     // ALU Register Output Select
reg     [3:0] CCSel;    // ALU Condition Code Output Select

//  ALU Input Data Interface

reg     [7:0] K;        // ALU/Core Bit Mask Input
reg     [7:0] Tmp;      // ALU/Core Temporary Register - OP1
reg     [7:0] M;        // ALU/Core Data Input (DI)  - Memory Operand

//  ALU Output Data Interface

wire    [7:0] DO;       // ALU/Core Data Output (DO)
wire    Valid;          // ALU/Core ALU Output Valid Flag

//  ALU Condition Code Interface

wire    CC_Out;            // ALU/Core Condition Code Multiplexer Output

//  ALU Processor Index Registers

wire    [7:0] X;           // ALU/Core Index Register X
wire    [7:0] Y;           // ALU/Core Index Register Y

//  ALU Processor Status Register 

wire    [7:0] P;           // ALU/Core Processor Status Word

//  Simulation Variables

integer i         = 0;
reg     [7:0] Val = 0;
reg     Done      = 1;
reg     Wait      = 0;

// Instantiate the Unit Under Test (UUT)

M65C02_ALUv2    uut (
                    .Rst(Rst), 
                    .Clk(Clk),
                    
                    .Rdy(Rdy),
                    
                    .En(En),
                    .Reg_WE(Reg_WE),
                    .ISR(ISR),
                    
                    .SO(SO),
                    .Clr_SO(Clr_SO),
                    
                    .SelS(SelS),
                    .S(S),
                    
                    .FU_Sel(FU_Sel[4:0]),
                    .Op(Op),
                    .QSel(QSel),
                    .RSel(RSel),
                    .CSel(CSel),
                    .WSel(WSel), 
                    .OSel(OSel), 
                    .CCSel(CCSel),
                    
                    .K(K),
                    .Tmp(Tmp),
                    .M(M),                

                    .DO(DO),
                    .Val(Valid),
                    
                    .CC_Out(CC_Out),
                    
                    .X(X), 
                    .Y(Y), 
                    
                    .P(P) 
                );

////////////////////////////////////////////////////////////////////////////////

initial begin
    // Initialize Inputs
    Rst    = 1;
    Clk    = 1;

    Reg_WE = 0;
    ISR    = 1;
    
    SO     = 0;
    
    FU_Sel = 0;
    Op     = 0;
    QSel   = 0;
    RSel   = 0;
    CSel   = 0;
    WSel   = 0;
    OSel   = 0;
    CCSel  = 0;
    
    K      = 0;
    Tmp    = 0;
    
    M      = 0;

    // Wait 100 ns for global reset to finish
    #101 Rst = 0; ISR = 0;
    @(posedge Clk) #1;
    
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
    
    if(   (uut.A==0)
       && (X==0)
       && (Y==0)
       && (S==2)
       && ((P & (K_M | K_B | K_I | K_D))==(K_M |K_I)))
        $display("Reset Initialization Complete: Registers Initialized");
    else begin
        $display("Reset Initialization Complete: Registers Not Initialized");
        $stop;
    end
    
    Tst_Reg_Load;
    Tst_Stack_Pull;
    Tst_SP_Load;
    Tst_LST;
    Tst_LU;
    Tst_SU;
    
    Tst_AU;

    $display("End of Test\n\tAll Tests Passed");
    Reg_WE = 0;
    repeat(4) @(posedge Clk) #1;
    $stop;
end

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Clock and External Logic                                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//  Clock Generator
//
    
always #5 Clk = ~Clk;

////////////////////////////////////////////////////////////////////////////////
//
//  Stack Pointer Register from M65C02 Address Generator
//

always @(posedge Clk)
begin
    if(Rst)
        S <= #1 2;
    else if(SelS)
        S <= #1 DO;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Decode/Demultiplex of M65C02 Processor Status Word
//

assign C = P[0];
assign Z = P[1];
assign I = P[2];
assign D = P[3];
assign V = P[6];
assign N = P[7];

////////////////////////////////////////////////////////////////////////////////
//
//  M65C02 Cycle Length Control - Ready Generator
//

always @(*)
begin
    case({Done, (FU_Sel[5] & |Reg_WE), Valid, ~Wait})
        4'b0000 : Rdy <= 0;
        4'b0001 : Rdy <= 1;     // Non-ALU external cycle ready
        4'b0010 : Rdy <= 0;
        4'b0011 : Rdy <= 1;     // Non-ALU external cycle ready
        4'b0100 : Rdy <= 0;
        4'b0101 : Rdy <= 1;     // Operands not ready, external cycle ready
        4'b0110 : Rdy <= 0;
        4'b0111 : Rdy <= 1;     // Operands not ready, external cycle ready
        4'b1000 : Rdy <= 0;
        4'b1001 : Rdy <= 1;     // Non-ALU op and external fetch ready
        4'b1010 : Rdy <= 0;
        4'b1011 : Rdy <= 1;     // Non-ALU op and external fetch ready
        4'b1100 : Rdy <= 0;     // ALU op and external fetch not ready
        4'b1101 : Rdy <= 0;     // ALU op not ready and external fetch ready
        4'b1110 : Rdy <= 0;     // ALU op ready and external fetch not ready
        4'b1111 : Rdy <= 1;     // ALU op and external fetch cycle ready
    endcase
end

////////////////////////////////////////////////////////////////////////////////
//
//  Generate ALU Enable - En
//

assign En = |Reg_WE;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Tasks and Functions                                                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//  General Helper Tasks
//

task Tst_AB;
    input   [7:0] A, B, Msk;
    input   [((20 * 8) - 1):0] Msg;

begin
    if((A & ~Msk) != (B & ~Msk)) begin
        $display("\t%s Found 0x%h, Expected 0x%h", Msg, (A & ~Msk), (B & ~Msk));
        Reg_WE = 0;
        repeat(4) @(posedge Clk) #1;
        $stop;
    end
end

endtask;

task Tst_NZ;
    input   [7:0] A;

begin
    if((A[7] != N) || (~|A != Z)) begin
        $display("\tError: NZ Flags Incorrect;");
        Reg_WE = 0;
        repeat(4) @(posedge Clk) #1;
        $stop;
    end
end

endtask;

task IDecode;
    input   [5:0] ID_FU_Sel;
    input   [1:0] ID_Op;
    input   [1:0] ID_QSel;
    input   [1:0] ID_RSel;
    input   [1:0] ID_CSel;
    input   [2:0] ID_WSel;
    input   [2:0] ID_OSel;
    input   [3:0] ID_CCSel;

begin
    FU_Sel = ID_FU_Sel;
    Op     = ID_Op;
    QSel   = ID_QSel;
    RSel   = ID_RSel;
    CSel   = ID_CSel;
    WSel   = ID_WSel;
    OSel   = ID_OSel; 
    CCSel  = ID_CCSel;
end

endtask;

////////////////////////////////////////////////////////////////////////////////
//
//  ALU Register Load Operations
//

task Load;
    input   [2:0] Dst;      // Index for Data Destination
    input   [2:0] Src;      // Index for Data Source
    input   [7:0] Val;      // Value to Load into Register if Src: {Tmp, M}
    
    reg     [7:0] Old_PSW;  // Value of P before Load
    
begin
    K   = 0;
    Tmp = ((Src == OS_T) ? Val : 0);
    M   = ((Src == OS_M) ? Val : 0);

    IDecode(LST, NOP, NOP, NOP, NOP, Dst, Src, PSW);
    Reg_WE = RWE_R;

    @(negedge Clk) Old_PSW = uut.P;

    case(Dst)
        WS_A, WS_X, WS_Y :
            begin
                CCSel = P_NZ;
                
                @(posedge Clk) #1.1;

                Tst_AB(uut.DO, Val, 0, "Error: DO != Val; --");

                case(Dst)
                    WS_A :
                        begin
                            Tst_AB(uut.A, uut.DO, 0, "Error: A != DO; ----");
                            Tst_NZ(uut.A);
                        end

                    WS_X :
                        begin
                            Tst_AB(uut.X, uut.DO, 0, "Error: X != DO; ----");
                            Tst_NZ(uut.X);
                        end

                    WS_Y :
                        begin                        
                            Tst_AB(uut.Y, uut.DO, 0, "Error: Y != DO; ----");
                            Tst_NZ(uut.Y);
                        end
                endcase

                Tst_AB(uut.P, Old_PSW, K_NZ, "Error: Unexpected P ");
            end
            
        WS_S :
            begin
                CCSel = NOP;
                
                @(posedge Clk) #1.1;

                case(OSel)
                    OS_A :                       
                        Tst_AB(uut.DO, uut.A, 0, "Error: DO != A; ----");
                        
                    OS_X :
                        Tst_AB(uut.DO, uut.X, 0, "Error: DO != X; ----");
                    
                    default :
                        begin
                            $display("\tInvalid Source for Test: %d", Src);
                            Reg_WE = NOP;
                            repeat(4) @(posedge Clk) #1;
                            $stop;
                        end
                endcase
                    
                Tst_AB(uut.S, uut.DO,  0,    "Error: S != DO; ----");
                Tst_AB(uut.P, Old_PSW, K_MB, "Error: Unexpected P ");
            end
            
        default :
            begin
                $display("\tInvalid Destination for Test: %d", Dst);
                Reg_WE = NOP;
                repeat(4) @(posedge Clk) #1;
                $stop;
            end
    endcase
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  ALU Register Stack Pull Operations
//

task Pull;
    input   [2:0] Dst;      // Index for Data Destination
    input   [7:0] Val;      // Value to Load into Register from Src: M
    
    reg     [7:0] Old_PSW;  // Value of P before Load
    
begin
    case(Dst)
        WS_A, WS_X, WS_Y :
            begin
                Old_PSW = uut.P;
                
                IDecode(LST, NOP, NOP, NOP, NOP, Dst, OS_M, P_NZ);
                
                K   = 0;
                Tmp = 0;
                M   = Val;
                
                Reg_WE = RWE_R;

                @(posedge Clk) #1.1;
                                        
                Tst_AB(uut.DO, Val,  0, "Error: DO != Val ---");

                case(Dst)
                    WS_A :
                        begin
                            Tst_AB(uut.A, uut.DO,  0, "Error: A != DO -----");
                            Tst_NZ(uut.A);
                        end

                    WS_X :
                        begin
                            Tst_AB(uut.X, uut.DO,  0, "Error: X != DO -----");
                            Tst_NZ(uut.X);
                        end

                    WS_Y :
                        begin
                            Tst_AB(uut.Y, uut.DO,  0, "Error: Y != DO -----");
                            Tst_NZ(uut.Y);
                        end
                endcase

                Tst_AB(uut.P, Old_PSW,  K_NZ, "Error: Unexpected P ");
            end
        
        WS_P :
            begin
                IDecode(LST, NOP, NOP, NOP, NOP, Dst, OS_M, PSW);
                
                K   = 0;
                Tmp = 0;
                M   = Val;
                
                Reg_WE = RWE_R;
                @(posedge Clk) #1.1;

                Tst_AB(uut.DO, Val,  0, "Error: DO != Val ---");
                Tst_AB(uut.P, uut.DO,  K_MB, "Error: P != DO -----");
            end
        
        default :
            begin
                $display("\tInvalid Destination for Test: %d", Dst);
                Reg_WE = NOP;
                repeat(4) @(posedge Clk) #1;
                $stop;
            end
    endcase
end

endtask

task Tst_Reg_Load;

    reg     [3:0] i;
    reg     [7:0] Val;

begin
    $display("**** ALU Load Register Tests");
    
    $display("******* Load A");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Load(WS_A, OS_M, Val);
        $display("\t**** LDA #%h", Val);
    end
    
    $display("******* Load X");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Load(WS_X, OS_M, Val);
        $display("\t**** LDX #%h", Val);
    end
    
    $display("******* Load Y");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Load(WS_Y, OS_M, Val);
        $display("\t**** LDY #%h", Val);
    end
    
    $display("**** ALU Load Register Tests - Complete");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Test ALU Register Stack Pull Operations
//

task Tst_Stack_Pull;

begin
    $display("**** ALU Register Stack Pull Tests");
    
    $display("******* PLA");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Pull(WS_A, Val);
        $display("\t**** PLA %h", Val);
    end
    
    $display("******* PLP");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Pull(WS_P, Val);
        $display("\t**** PLP %h", Val);
    end

    $display("******* PLX");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Pull(WS_X, Val);
        $display("\t**** PLX %h", Val);
    end
    
    $display("******* PLY");
    for(i = 0; i < 9; i = i + 1) begin
        Val = (1 << i);
        Pull(WS_Y, Val);
        $display("\t**** PLY %h", Val);
    end

    $display("**** ALU Register Stack Pull Tests - Complete");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Test Stack Pointer Load Operations
//

task Tst_SP_Load;

begin
    $display("**** ALU Stack Pointer Load Tests");
    
    Load(WS_X, OS_M, 8'hAA); $display("\t**** LDX %h", uut.X);
    Load(WS_S, OS_X, 8'h00); $display("\t**** TXS %h", S);
    Load(WS_X, OS_M, 8'h55); $display("\t**** LDX %h", uut.X);
    Load(WS_S, OS_X, 8'h00); $display("\t**** TXS %h", S);
    
    Load(WS_A, OS_M, 8'h99); $display("\t**** LDA %h", uut.X);
    Load(WS_S, OS_A, 8'h00); $display("\t**** TXA %h", S);
    Load(WS_A, OS_M, 8'h66); $display("\t**** LDA %h", uut.X);
    Load(WS_S, OS_A, 8'h00); $display("\t**** TXA %h", S);

    Load(WS_X, OS_M, 8'h55); $display("\t**** LDX %h", uut.X);
    Load(WS_S, OS_X, 8'h00); $display("\t**** TXS %h", S);
    
    $display("**** ALU Stack Pointer Load Tests - Complete");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Test ALU Load/Store/Transfer Multiplexer Operations
//

task Tst_LST;
    
    reg     [7:0] Old_PSW;
    reg     [3:0] i;
    reg     [7:0] Val;
    
begin
    $display("**** ALU LST Multiplexer Tests");

    Load(WS_A, OS_M, 8'h11); $display("\t**** LDA %h", uut.A);
    Load(WS_X, OS_M, 8'h22); $display("\t**** LDX %h", uut.X);
    Load(WS_Y, OS_M, 8'h33); $display("\t**** LDY %h", uut.Y);
    Pull(WS_P, 8'h66);       $display("\t**** PLP %h", uut.P);

    K = 8'hFF; Tmp = 8'h44; M = 8'h77;
    
    Reg_WE = NOP;
    
    for(i = 0; i < 8; i = i + 1) begin
        Old_PSW = uut.P;
        
        IDecode(LST, NOP, NOP, NOP, NOP, NOP, i[2:0], NOP);
        Val  = 0;
        
        @(posedge Clk) #1.1;
        
        Tst_AB(uut.DO, Val,     0, "Error: DO != Val ---");
        Tst_AB(uut.P,  Old_PSW, 0, "Error: Unexpected P ");
    end
    
    Reg_WE = RWE_M;

    for(i = 0; i < 8; i = i + 1) begin
        Old_PSW = uut.P;
        
        IDecode(LST, NOP, NOP, NOP, NOP, NOP, i[2:0], NOP);
        Val  = {i, i};
        
        @(posedge Clk) #1.1;
        
        Tst_AB(uut.DO, Val,     0, "Error: DO != Val ---");
        Tst_AB(uut.P,  Old_PSW, 0, "Error: Unexpected P ");
    end

    $display("**** ALU LST Multiplexer Tests - Complete");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Helper Tasks for the Test Logic Unit (LU) task
//

task Clr_Carry_Flag;

begin
    IDecode(LU, CLC, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_C;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask

task Set_Carry_Flag;

begin
    IDecode(LU, SEC, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_C;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask

task Clr_IRQ_Mask;

begin
    IDecode(LU, CLI, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_I;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask    

task Set_IRQ_Mask;

begin
    IDecode(LU, SEI, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_I;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask 

task Clr_Decimal_Mode;

begin
    IDecode(LU, CLD, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_D;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask   

task Set_Decimal_Mode;

begin
    IDecode(LU, SED, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_D;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask

task Clr_oVerflow_Flag;

begin
    IDecode(LU, CLV, L_K, R_P, Ci_C, WS_P, NOP, PSW);
    K = K_V;
    Reg_WE = RWE_R;
    @(posedge Clk) #1.1;
    Reg_WE = 0;
end

endtask    

////////////////////////////////////////////////////////////////////////////////

task Clr_Tst_Carry_Flag;

begin
    $write("\tClear and Test Carry Flag - ");
    Clr_Carry_Flag;
    Tst_AB(uut.P & K_C, 0, ~K_C, "Error: C != 0 ------");
    $display("Pass");
end

endtask

task Set_Tst_Carry_Flag;

begin
    $write("\tSet and Test Carry Flag - ");
    Set_Carry_Flag;
    Tst_AB(uut.P & K_C, K_C, ~K_C, "Error: C != 1 ------");
    $display("Pass");
end

endtask

task Clr_Tst_IRQ_Mask;

begin
    $write("\tClear and Test Interrupt Mask Flag - ");
    Clr_IRQ_Mask;
    Tst_AB(uut.P & K_I, 0, ~K_I, "Error: I != 0 ------");
    $display("Pass");
end

endtask

task Set_Tst_IRQ_Mask;

begin
    $write("\tSet and Test Interrupt Mask Flag - ");
    Set_IRQ_Mask;
    Tst_AB(uut.P & K_I, K_I, ~K_I, "Error: I != 1 ------");
    $display("Pass");
end

endtask

task Clr_Tst_Decimal_Mode;

begin
    $write("\tClear and Test Decimal Mode Flag - ");
    Clr_Decimal_Mode;
    Tst_AB(uut.P & K_D, 0, ~K_D, "Error: D != 0 ------");
    $display("Pass");
end

endtask

task Set_Tst_Decimal_Mode;

begin
    $write("\tSet and Test Decimal Mode Flag - ");
    Set_Decimal_Mode;
    Tst_AB(uut.P & K_D, K_D, ~K_D, "Error: D != 1 ------");
    $display("Pass");
end

endtask

task Clr_Tst_oVerflow_Flag;

begin
    $write("\tClear and Test oVerflow Flag - ");
    Clr_oVerflow_Flag;
    Tst_AB(uut.P & K_V, 0, ~K_V, "Error: V != 0 ------");
    $display("Pass");
end

endtask

task Set_Tst_oVerflow_Flag;

begin
    $write("\tSet and Test oVerflow Flag - ");

    fork
        begin
            Clr_oVerflow_Flag;
        end
        
        begin
            SO = 1;
        end
    join

    Tst_AB(uut.P & K_V, K_V, ~K_V, "Error: V != 1 ------");
    
    if(!Clr_SO) begin
        $display("\tError: Expected Clr_SO == 1; Found Clr_SO=%b", Clr_SO);
        Reg_WE = 0;
        repeat(4) @(posedge Clk) #1;
        $stop;
    end else begin
        SO = 0;
    end
    
    $display("Pass");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Test Logic Unit (LU)
//

task Tst_LU;

    reg     [7:0] i, Val, Reg, Old_PSW;
    
begin
    $display("**** ALU LU Functional Tests");
    
    $display("******* Test LU AND Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, AND, L_A, R_M, Ci_C, WS_A, NOP, P_NZ);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tAND L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (Reg & M); $display(" ALU=%h", Val);

        Tst_AB(uut.A, Val, 0, "Error: A != A & M -");

        Val =   (Old_PSW & ~K_NZ)
              | (((Reg & M) & K_N) | ((~|(Reg & M)) ? K_Z : 0));

        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU AND Function - Complete");
    
    $display("******* Test LU ORA Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, ORA, L_A, R_M, Ci_C, WS_A, NOP, P_NZ);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tORA L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (Reg | M); $display(" ALU=%h", Val);

        Tst_AB(uut.A, Val, 0, "Error: A != A | M -");

        Val =   (Old_PSW & ~K_NZ)
              | (((Reg | M) & K_N) | ((~|(Reg | M)) ? K_Z : 0));
              
        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU ORA Function - Complete");

    $display("******* Test LU EOR Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, EOR, L_A, R_M, Ci_C, WS_A, NOP, P_NZ);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tEOR L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (Reg ^ M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.A, Val, 0, "Error: A != A ^ M -");

        Val =   (Old_PSW & ~K_NZ)
              | (((Reg ^ M) & K_N) | ((~|(Reg ^ M)) ? K_Z : 0));
              
        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU EOR Function - Complete");

    $display("******* Test LU BIT Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, BIT, L_A, R_M, Ci_C, WS_M, NOP, P_NVZ);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tBIT L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (Reg & M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 0, "Error: DO != A & M ");
        Tst_AB(uut.A,  Reg, 0, "Error: A  != A ----");

        Val = (Old_PSW & ~K_NVZ)
              | (   (M & K_N)
                 |  (M & K_V)
                 | ((~|(Reg & M)) ? K_Z : 0));
              
        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU BIT Function - Complete");

    $display("******* Test LU BIT #imm Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, BIT, L_A, R_M, Ci_C, WS_M, NOP, P_Z);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tBIT #imm L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (Reg & M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 0, "Error: DO != A & M ");
        Tst_AB(uut.A,  Reg, 0, "Error: A  != A ----");

        Val = (Old_PSW & ~K_Z) | ((~|(Reg & M)) ? K_Z : 0);
              
        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU BIT #imm Function - Complete");

    $display("******* Test LU TRB Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, TRB, L_A, R_M, Ci_C, WS_M, NOP, P_Z);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tTRB L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (~Reg & M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 0, "Error: DO != ~A & M");
        Tst_AB(uut.A,  Reg, 0, "Error: A  != A ----");

        Val = (Old_PSW & ~K_Z) | ((~|(Reg & M)) ? K_Z : 0);
              
        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU TRB Function - Complete");

    $display("******* Test LU TSB Function");
    
    K = 8'hAA; Tmp = 8'h55;
    for(i = 0; i < 16; i = i + 1) begin
        Val = $random;
        Load(WS_A, OS_M, Val);
        Reg     = uut.A;
        Old_PSW = uut.P;
        
        IDecode(LU, TSB, L_A, R_M, Ci_C, WS_M, NOP, P_Z);
        
        M = $random; 
        
        Reg_WE = RWE_R;

        $write("\tTSB L=%h, R=%h", uut.A, M);
        
        @(posedge Clk) #1.1;
        
        Val = (Reg | M); $display(" ALU=%h", Val);

        Tst_AB(uut.DO, Val, 8'h00, "Error: M != A | P; ");

        Tst_AB(uut.A, Reg, 8'h00, "Error: A != Old(A) ");

        Val = (Old_PSW & ~K_Z) | ((~|(Reg & M)) ? K_Z : 0);
              
        Tst_AB(uut.P, Val, 8'h00, "Error: Invalid PSW ");
    end

    $display("******* Test LU TSB Function - Complete");

    $display("******* Test LU RMB Function");
    
    K   = 8'h00;
    Tmp = 8'h00;
    M   = 8'hFF; 
        
    for(i = 0; i < 8; i = i + 1) begin
        Old_PSW = uut.P;

        IDecode(LU, RMB, L_K, R_M, Ci_C, WS_M, NOP, TRUE);
        
        K = (1 << i);

        Reg_WE = RWE_M;

        $write("\tRMB K=%h, R=%h", K, M);
        
        @(posedge Clk) #1.1;
        
        Val = (~K & M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 8'h00, "Error: M != ~K & M ");

        Tst_AB(uut.P, Old_PSW, 8'h00, "Error: Invalid PSW ");
        
        M = uut.DO;
    end

    $display("******* Test LU RMB Function - Complete");

    $display("******* Test LU SMB Function");
    
    K   = 8'h00;
    Tmp = 8'h00;
    M   = 8'h00;
    
    for(i = 0; i < 8; i = i + 1) begin
        Old_PSW = uut.P;

        IDecode(LU, SMB, L_K, R_M, Ci_C, WS_M, NOP, TRUE);
        
        K = (1 << i);
        
        Reg_WE = RWE_M;

        $write("\tSMB K=%h, R=%h", K, M);
        
        @(posedge Clk) #1.1;
        
        Val = (K | M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 8'h00, "Error: M != K | M ");

        Tst_AB(uut.P, Old_PSW, 8'h00, "Error: Invalid PSW ");
        
        M = uut.DO;
    end

    $display("******* Test LU SMB Function - Complete");

    $display("******* Test LU BBR Function");
    
    K   = 8'h00;
    Tmp = 8'h00;
    M   = 8'h55;
    
    for(i = 0; i < 16; i = i + 1) begin
        Old_PSW = uut.P;

        IDecode(LU, BBR, L_K, R_M, Ci_C, WS_M, NOP, BEQ);
        
        K = (1 << i[2:0]);
        
        Reg_WE = RWE_M;

        $write("\tBBR K=%h, R=%h", K, M);
        
        @(posedge Clk) #1.1;
        
        Val = (K & M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 8'h00, "Error: M != K & M ");
        
        if(CC_Out != ~|Val) begin
            $display("\tError: Unexpected CC Out ; Found %b, Expected %b", CC_Out, ~|Val);
            Reg_WE = 0;
            @(posedge Clk) #1;
            @(posedge Clk) #1;
            $stop;
        end

        Tst_AB(uut.P, Old_PSW, K_MB, "Error: Invalid PSW ");
       
        if(i == 7) M = ~M;
    end

    $display("******* Test LU BBR Function - Complete");

    $display("******* Test LU BBS Function");
    
    K   = 8'h00;
    Tmp = 8'h00;
    M   = 8'hAA;
    
    for(i = 0; i < 16; i = i + 1) begin
        Old_PSW = uut.P;

        IDecode(LU, BBS, L_K, R_M, Ci_C, WS_M, NOP, BNE);
        
        K = (1 << i[2:0]);
        
        Reg_WE = RWE_M;

        $write("\tBBS K=%h, R=%h", K, M);
        
        @(posedge Clk) #1.1;
        
        Val = (K & M); $display(" ALU=%h", Val);
        
        Tst_AB(uut.DO, Val, 8'h00, "Error: M != K & M ");

        if(CC_Out != |Val) begin
            $display("\tError: Unexpected CC Out ; Found %b, Expected %b", CC_Out, |Val);
            Reg_WE = 0;
            repeat(4) @(posedge Clk) #1;
            $stop;
        end

        Tst_AB(uut.P, Old_PSW, K_MB, "Error: Invalid PSW ");
        
        if(i == 7) M = ~M;
    end

    $display("******* Test LU BBS Function - Complete");

    $display("******* Test LU PSW Bit Set/Reset Functions");
    
    Pull(WS_P, 8'hFF);
    
    Clr_Tst_Carry_Flag;
    Set_Tst_Carry_Flag;
    Clr_Tst_IRQ_Mask;
    Set_Tst_IRQ_Mask;
    Clr_Tst_Decimal_Mode;
    Set_Tst_Decimal_Mode;
    Clr_Tst_IRQ_Mask;
    Set_Tst_IRQ_Mask;
    Clr_Tst_oVerflow_Flag;
    Set_Tst_oVerflow_Flag;
    
    $display("******* Test LU PSW Bit Set/Reset Functions - Complete");

    $display("******* Test LU PSW REP/SEP Functions");
    
    Pull(WS_P, 8'hFF);
    
    K   = 8'h00;
    Tmp = 8'h00;
    M   = 8'hFF;
    
    Old_PSW = uut.P;

    IDecode(LU, REP, L_M, R_P, Ci_C, WS_P, NOP, TRUE);

    Reg_WE = RWE_R;

    $write("\tREP P=%h, M=%h", Old_PSW, M);
    
    @(posedge Clk) #1.1;
    
    Val = (~M & Old_PSW); $display(" ALU=%h, P=%h", Val, uut.P);
    
    Tst_AB(uut.DO, Val, 8'h00, "Error: P != ~M & P; ");
    Tst_AB(uut.P,  K_M, 8'h00, "Incorrect P; -------");

    Old_PSW = uut.P;

    IDecode(LU, SEP, L_M, R_P, Ci_C, WS_P, NOP, TRUE);
    
    Reg_WE = RWE_R;

    $write("\tSEP P=%h, M=%h", Old_PSW, M);
    
    @(posedge Clk) #1.1;
    
    Val = (M | Old_PSW); $display(" ALU=%h, P=%h", Val, uut.P);

    Tst_AB(uut.DO,   Val, 8'h00, "Error: P != M | P; -");
    Tst_AB(uut.P,  8'hFF,   K_B, "Incorrect P; -------");

    $display("******* Test LU PSW REP/SEP Functions - Complete");

    $display("**** ALU LU Functional Tests - Complete");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Test Shift Unit (SU)
//

task Tst_SU;

    reg     [7:0] i, Val, Reg, Old_PSW;
    reg     Ci, Co;
    
begin
    $display("**** ALU SU Functional Tests");
    
    $display("******* Test SU ASL Function");
    
    K = 8'h00; Tmp = 8'h00; M = 8'hFF;
    
    for(i = 0; i < 9; i = i + 1) begin
        Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;
    
        IDecode(SU, ASL, Q_M, R_M, Ci_0, WS_M, OS_M, P_NZC);
        Reg_WE = RWE_R;

        @(negedge Clk);
        
        Old_PSW   = uut.P;
        Ci        = ((uut.P & 1) != 0);
        {Co, Val} = (M << 1);
        
        $write("\tASL M=%h, Ci=%b", uut.M, Ci);
        $display(" DO=%h, Co=%b", Val, Co);

        Tst_AB(uut.DO, Val, 0, "Error: DO != M << 1 ");
        
        if(Co != ((M & K_N) != 0)) begin
            $display("\tError: Co != M[7]; Found %b, Expected %b", Co, ((M & K_N) != 0));
            Reg_WE = 0;
            repeat(4) @(posedge Clk) #1;
            $stop;
        end

        @(posedge Clk) #1 M = DO; #0.1;
        
        Val =   (Old_PSW & ~K_NZC)
              | ((Val & K_N) | ((~|Val) ? K_Z : 0) | Co);

        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU ASL Function - Complete");
    
    $display("******* Test SU ROL Function");
    
    K = 8'hFF; Tmp = 8'hFF; M = 8'h00;
    
    for(i = 0; i < 9; i = i + 1) begin
        Set_Carry_Flag; K = 8'hFF; Tmp = 8'hFF;

        IDecode(SU, ROL, Q_M, R_M, Ci_C, WS_M, OS_M, P_NZC);
        Reg_WE = RWE_R;

        @(negedge Clk);
        
        Old_PSW   = uut.P;
        Ci        = ((uut.P & 1) != 0);
        {Co, Val} = ((M << 1) | Ci);
        
        $write("\tROL M=%h, Ci=%b", M, Ci);
        $display(" DO=%h, Co=%b", Val, Co);
        
        Tst_AB(DO, Val, 0, "Error: DO != M << 1 ");
        
        if(Co != ((M & K_N) != 0)) begin
            $display("\tError: Co != M[7]; Found %b, Expected %b", Co, ((M & K_N) != 0));
            Reg_WE = 0;
            repeat(4) @(posedge Clk) #1;
            $stop;
        end

        @(posedge Clk) #1 M = DO; #0.1;

        Val =   (Old_PSW & ~K_NZC)
              | ((Val & K_N) | ((~|Val) ? K_Z : 0) | Co);

        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
        
    end

    $display("******* Test LU ROL Function - Complete");

    $display("******* Test SU LSR Function");
    
    K = 8'h00; Tmp = 8'h00; M = 8'hFF;
    
    for(i = 0; i < 9; i = i + 1) begin
        Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;
    
        IDecode(SU, LSR, Q_M, R_M, Ci_0, WS_M, OS_M, P_NZC);
        Reg_WE = RWE_R;

        @(negedge Clk);
        
        Old_PSW   = uut.P;
        Ci        = ((uut.P & 1) != 0);
        {Co, Val} = {M[0], (M >> 1)};
        
        $write("\tLSR M=%h, Ci=%b", uut.M, Ci);
        $display(" DO=%h, Co=%b", Val, Co);

        Tst_AB(uut.DO, Val, 0, "Error: DO != M >> 1 ");
        
        @(posedge Clk) #1 M = DO; #0.1;
        
        Val =   (Old_PSW & ~K_NZC)
              | ((Val & K_N) | ((~|Val) ? K_Z : 0) | Co);

        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
    end

    $display("******* Test LU LSR Function - Complete");
    
    $display("******* Test SU ROR Function");
    
    K = 8'hFF; Tmp = 8'hFF; M = 8'h00;
    
    for(i = 0; i < 9; i = i + 1) begin
        Set_Carry_Flag; K = 8'hFF; Tmp = 8'hFF;

        IDecode(SU, ROR, Q_M, R_M, Ci_C, WS_M, OS_M, P_NZC);
        Reg_WE = RWE_R;

        @(negedge Clk);
        
        Old_PSW   = uut.P;
        Ci        = (uut.P[0] != 0);
        {Co, Val} = {M[0], ((M >> 1) | (Ci << 7))};
        
        $write("\tROR M=%h, Ci=%b", M, Ci);
        $display(" DO=%h, Co=%b", Val, Co);
        
        Tst_AB(DO, Val, 0, "Error: DO != M >> 1 ");
        
        @(posedge Clk) #1 M = DO; #0.1;

        Val =   (Old_PSW & ~K_NZC)
              | ((Val & K_N) | ((~|Val) ? K_Z : 0) | Co);

        Tst_AB(uut.P, Val, 0, "Error: P Invalid ---");
        
    end

    $display("******* Test LU ROR Function - Complete");

    $display("**** ALU SU Functional Tests - Complete");
end

endtask

////////////////////////////////////////////////////////////////////////////////
//
//  Test Adder Unit (AU)
//

//  Task AU_Model provides a logical model of the binary/decimal mode adder that
//      is implemented in the UUT. Its purpose is not to functionally represent
//      the implementation in the UUT, but to logically compute the expected
//      outputs (sum and status) for the various operations that the UUT adder
//      is expected to perform.

task AU_Model;
    input   iOp;
    input   [7:0] iA, iB;
    input   [7:0] iPSW;
    input   [7:0] iMsk;
    output  [7:0] oSum;
    output  [7:0] oPSW;
    
    reg     D, Ci;
    reg     [4:0] LSN, MSN;
    reg     [7:0] A, B, ALU;
    reg     C7, C3;
    reg     N, V, Z, C;     

begin
    D  = iPSW[3];
    Ci = iPSW[0];
    A  = iA;
    B  = ((iOp) ? ~iB : iB);

    if(D) begin         // Decimal Mode Operations
        if(iOp) begin   // Decimal Mode Subtraction
            LSN[4:0] = A[3:0] + B[3:0] + Ci;
            C3       = LSN[4] & ~(LSN[3] & (LSN[2] | LSN[1]));
            ALU[3:0] = ((C3) ? (LSN[3:0] + 0) : (LSN[3:0] + 10));

            MSN[4:0] = A[7:4] + B[7:4] + C3;
            C7       = MSN[4] & ~(MSN[3] & (MSN[2] | MSN[1]));
            ALU[7:4] = ((C7) ? (MSN[3:0] + 0) : (MSN[3:0] + 10));        
        end else begin  // Decimal Mode Addition
            LSN[4:0] = A[3:0] + B[3:0] + Ci;
            C3       = LSN[4] | (LSN[3] & (LSN[2] | LSN[1]));
            ALU[3:0] = ((C3) ? (LSN[3:0] + 6) : (LSN[3:0] + 0));

            MSN[4:0] = A[7:4] + B[7:4] + C3;
            C7       = MSN[4] | (MSN[3] & (MSN[2] | MSN[1]));
            ALU[7:4] = ((C7) ? (MSN[3:0] + 6) : (MSN[3:0] + 0));
        end

        N = ALU[7]; V = ((Op) ? ~C7 : C7); Z = ~|ALU; C = C7;
    end else begin      // Binary Mode Operations
            LSN[4:0] = A[3:0] + B[3:0] + Ci;
            C3       = LSN[4];
            ALU[3:0] = LSN[3:0];

            MSN[3:0] = A[6:4] + B[6:4] + C3;
            MSN[4]   = (MSN[3] & (A[7] ^ B[7]) | (A[7] & B[7]));
            C7       = MSN[4];
            ALU[7:4] = {A[7] ^ B[7] ^ MSN[3], MSN[2:0]};        

            N = ALU[7]; V = (MSN[4] ^ MSN[3]); Z = ~|ALU; C = C7;
     end
    
    oSum = ALU;
    oPSW = ((iPSW & ~iMsk) | ({N, V, 4'b0, Z, C} & iMsk));
end

endtask;

//  Test Adder Unit
    
task Tst_AU;

    integer i, j, k;
    reg     [7:0] Val, AU_Sum, AU_PSW;
    
begin
    $display("**** ALU AU Functional Tests");
    
    K = 8'h00; Tmp = 8'h00;
    
    $display("******* Test AU Decimal Mode ADC Function");

    Set_Decimal_Mode;
    for(i = 0; i < 100; i = i + 1) begin
        for(j = 0; j < 100; j = j + 1) begin
            //  Perform Test with Carry Not Set
            
            Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 10);
            Val = (k * 16) + (i - (k * 10));
            Load(WS_A, OS_M, Val);

            k = (j / 10);
            M = (k * 16) + (j - (k * 10));

            IDecode(ADD, ADC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(ADC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
            
            //  Perform Test with Carry Set
            
            Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 10);
            Val = (k * 16) + (i - (k * 10));
            Load(WS_A, OS_M, Val);

            k = (j / 10);
            M = (k * 16) + (j - (k * 10));

            IDecode(ADD, ADC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(ADC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
        end
    end

    $display("******* Test AU Decimal Mode ADC Function - Complete");
    
    $display("******* Test AU Decimal Mode SBC Function");

    Set_Decimal_Mode;
    for(i = 0; i < 100; i = i + 1) begin
        for(j = 0; j < 100; j = j + 1) begin
            //  Perform Test with Carry Set - Borrow not asserted
            
            Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 10);
            Val = (k * 16) + (i - (k * 10));
            Load(WS_A, OS_M, Val);

            k = (j / 10);
            M = (k * 16) + (j - (k * 10));

            IDecode(ADD, SBC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(SBC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
            
            //  Perform Test with Carry Clear - Borrow asserted
            
            Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 10);
            Val = (k * 16) + (i - (k * 10));
            Load(WS_A, OS_M, Val);

            k = (j / 10);
            M = (k * 16) + (j - (k * 10));

            IDecode(ADD, SBC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(SBC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
        end
    end

    $display("******* Test AU Decimal Mode SBC Function - Complete");

    $display("******* Test AU INC Function");

    Clr_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        //  Perform Test with Carry Not Set
        
        Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, INC, Q_A, R_Z, Ci_1, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(INC, Val, 0, ((uut.P & K_NZ) | K_C), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
        
        //  Perform Test with Carry Set
        
        Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, INC, Q_A, R_Z, Ci_1, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(INC, Val, 0, ((uut.P & K_NZ) | K_C), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
    end

    Set_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        //  Perform Test with Carry Not Set
        
        Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, INC, Q_A, R_Z, Ci_1, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(INC, Val, 0, ((uut.P & K_NZ) | K_C), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
        
        //  Perform Test with Carry Set
        
        Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, INC, Q_A, R_Z, Ci_1, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(INC, Val, 0, ((uut.P & K_NZ) | K_C), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
    end

    $display("******* Test AU INC Function - Complete");
    
    $display("******* Test AU DEC Function");

    Clr_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        //  Perform Test with Carry Set
        
        Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, DEC, Q_A, R_Z, Ci_0, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(DEC, Val, 0, (uut.P & K_NZ), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
        
        //  Perform Test with Carry Not Set
        
        Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, DEC, Q_A, R_Z, Ci_0, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(DEC, Val, 0, (uut.P & K_NZ), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
    end

    Set_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        //  Perform Test with Carry Set
        
        Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, DEC, Q_A, R_Z, Ci_0, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(DEC, Val, 0, (uut.P & K_NZ), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
        
        //  Perform Test with Carry Not Set
        
        Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

        k   = (i / 16);
        Val = (k * 16) + (i - (k * 16));
        Load(WS_A, OS_M, Val);

        M = Val;

        IDecode(IDC, DEC, Q_A, R_Z, Ci_0, WS_A, OS_A, P_NZ);
        Reg_WE = RWE_R;

        AU_Model(DEC, Val, 0, (uut.P & K_NZ), K_NZ, AU_Sum, AU_PSW);
        
        @(posedge Clk) #1.1;

        Tst_AB(uut.A, AU_Sum,   0, "Error: A != AU_Sum -");
        Tst_AB(uut.P & K_NZ, AU_PSW & K_NZ, 0, "Error: P != AU_PSW -");
    end

    $display("******* Test AU DEC Function - Complete");
    
    
    $display("******* Test AU Binary Mode ADC Function");

    Clr_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        for(j = 0; j < 256; j = j + 1) begin
            //  Perform Test with Carry Not Set
            
            Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(ADD, ADC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(ADC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
            
            //  Perform Test with Carry Set
            
            Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(ADD, ADC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(ADC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
        end
    end

    $display("******* Test AU Binary Mode ADC Function - Complete");
    
    $display("******* Test AU Binary Mode SBC Function");

    Clr_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        for(j = 0; j < 256; j = j + 1) begin
            //  Perform Test with Carry Set - Borrow not asserted
            
            Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(ADD, SBC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(SBC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
            
            //  Perform Test with Carry Clear - Borrow asserted
            
            Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(ADD, SBC, Q_A, R_M, Ci_C, WS_A, OS_A, P_NVZC);
            Reg_WE = RWE_R;

            AU_Model(SBC, Val, M, uut.P, K_NVZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum,      0, "Error: A != AU_Sum -");
            Tst_AB(uut.P, AU_PSW, K_NVZC, "Error: P != AU_PSW -");
        end
    end

    $display("******* Test AU Binary Mode SBC Function - Complete");
    
    $display("******* Test AU CMP Function");

    Clr_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        for(j = 0; j < 256; j = j + 1) begin
            //  Perform Test with Carry Not Set
            
            Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(IDC, CMP, Q_A, R_M, Ci_1, WS_A, OS_M, P_NZC);
            Reg_WE = RWE_R;

            AU_Model(CMP, Val, M, (uut.P & K_NZC) | K_C, K_NZC, AU_Sum, AU_PSW);
                        
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum, 0, "Error: A != AU_Sum -");
            Tst_AB(uut.P & K_NZC, AU_PSW & K_NZC, 0, "Error: P != AU_PSW -");
            
            //  Perform Test with Carry Set
            
            Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(IDC, CMP, Q_A, R_M, Ci_1, WS_A, OS_M, P_NZC);
            Reg_WE = RWE_R;

            AU_Model(CMP, Val, M, (uut.P & K_NZC) | K_C, K_NZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum, 0, "Error: A != AU_Sum -");
            Tst_AB(uut.P & K_NZC, AU_PSW & K_NZC, 0, "Error: P != AU_PSW -");
        end
    end

    Set_Decimal_Mode;
    for(i = 0; i < 256; i = i + 1) begin
        for(j = 0; j < 256; j = j + 1) begin
            //  Perform Test with Carry Not Set
            
            Clr_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(IDC, CMP, Q_A, R_M, Ci_1, WS_A, OS_M, P_NZC);
            Reg_WE = RWE_R;

            AU_Model(CMP, Val, M, (uut.P & K_NZC) | K_C, K_NZC, AU_Sum, AU_PSW);
                        
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum, 0, "Error: A != AU_Sum -");
            Tst_AB(uut.P & K_NZC, AU_PSW & K_NZC, 0, "Error: P != AU_PSW -");
            
            //  Perform Test with Carry Set
            
            Set_Carry_Flag; K = 8'h00; Tmp = 8'h00;

            k   = (i / 16);
            Val = (k * 16) + (i - (k * 16));
            Load(WS_A, OS_M, Val);

            k = (j / 16);
            M = (k * 16) + (j - (k * 16));

            IDecode(IDC, CMP, Q_A, R_M, Ci_1, WS_A, OS_M, P_NZC);
            Reg_WE = RWE_R;

            AU_Model(CMP, Val, M, (uut.P & K_NZC) | K_C, K_NZC, AU_Sum, AU_PSW);
            
            @(posedge Clk) #1.1;

            Tst_AB(uut.A, AU_Sum, 0, "Error: A != AU_Sum -");
            Tst_AB(uut.P & K_NZC, AU_PSW & K_NZC, 0, "Error: P != AU_PSW -");
        end
    end

    $display("******* Test AU CMP Function - Complete");

    IDecode(NOP, NOP, NOP, NOP, NOP, NOP, NOP, NOP);
    
    $display("**** ALU AU Functional Tests - Complete");
end

endtask

endmodule

