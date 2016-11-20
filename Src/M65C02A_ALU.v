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
// Module Name:     M65C02A_ALU.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module implements the ALU for the M65C02A microprocessor. Included in
//  the ALU are the accumulator (A), pre-/post-index registers (X and Y), and 
//  processor status word (P). The microcode, composed of a fixed part from the
//  instruction decoder (IDEC) and a variable portion from the execution engine
//  (uPgm), control a Load/Store/Transfer (LST) multiplexer, a Logic Unit (LU),
//  a Shift/rotate Unit (SU), and a dual-mode (binary/BCD) Adder Unit (AU).
//
//  The Load/Store/Transfer (LST) multiplexer allows the various registers in
//  the ALU, plus the external stack pointer (S) in the address generator module
//  (from the M65C02A Address Generator), to be multiplexed onto the ALU output
//  data bus. This allows registers to be loaded from memory (data memory or the
//  external data/return stack), stored to memory, or transfered from one regis-
//  ter to another.
//
//  The LU performs bit-wise operations (OR, AND, and XOR) on two operands. The
//  LU also supports the following instructions which only use variations of the
//  LU's basic functions: BIT/TRB/TSB, RMB/SMB/BBR/BBS, and CLC/SEC/CLI/SEI/CLD
//  /SED/CLV.
//
//  Using non-circular or circular shifts on the accumulator (A) or memory (M),
//  the SU performs arithmetic and logical operations. The four basic instruc-
//  tions, ASL/ROL/LSR/ROR, can be implemented using just two operations: left
//  shift and right shift. Various combinations of the operand and the carry
//  logic signal paths allow the non-circular shifts and circular shifts
//  (rotates) needed to implement these four instructions. The ALU can easily
//  support arithmetic overflow detection on the ASL instruction, and support an
//  arithmetic right shift (ASR) through modifications to the microprogram ROMs.
//
//  The AU performs binary two's complement addition, or unsigned BCD addition/
//  subtraction on two operands. Subtraction is performed using 1's complement
//  logic. A 2's complement result or extended precision addition/subtraction is
//  performed if the Carry flag, C, is cleared before addition or set before
//  subtraction. The AU is also used for increments, decrements, and compares.
//  For these operations, the arithmetic mode is fully 2's complement, i.e. the
//  carry input to the AU is automatically cleared/set as necessary.
//
//  The ALU requires the En input to be asserted to initiate the operation that
//  the fixed IDEC microword determines. Asserting the En input indicates that
//  the required operands for an operation are available. The addressing mode of
//  the instruction will determine when En is asserted. In the case of non-
//  program flow control instructions, En alone controls the updates to these
//  registers: A, X, Y, P. For implicit or accumulator mode instructions, En is
//  generally asserted when the Instruction Register (IR) is loaded with the op-
//  code. Otherwise, it is asserted when the required memory operand is availa-
//  ble from memory (M). The operation defined by IDEC will be completed with
//  the available operands in the ALU registers and the memory operand (M).
//
//  With the En input asserted, then the Rdy input controls the actual writing
//  of registers. To write a register, the En and Rdy inputs must be asserted, 
//  and the register must be selected. A register is written if its code is set
//  in the WSel field and Reg_WE is asserted.
//
//  The table below presents all of the valid combinations of the FU_Sel and Op
//  fields. The FU_Sel field selects the functional unit that implements the 
//  desired operation. The Op input selects the operation to be performed by LU,
//  SU, and AU functional units.
//
//      LST: LST = {M, S, P, T, A, Y, X, 0}
//      LU :   L = {A, P, K, M}, R = {A, P, K, M}; Ci = {Q[7], 1, 0, C}
//      SU :   Q = {A, Y, X, M},                   Ci = {Q[7], 1, 0, C} 
//      AU :   Q = {A, Y, X, M}, R = {A, P, K, M}; Ci = {Q[7], 1, 0, C}
//  
//   FU_Sel  Op  Mnemonic     Operation           Condition Code
//
//  1_100_00 00    XFR    ALU <= {OSel=(7..0): M, S, P, T, Y, X, A, 0}
//                                L : R
//  1_010_00 01    AND    ALU <=  A & M;     N <= ALU[7]; Z <= ~|ALU;
//  1_010_00 10    ORA    ALU <=  A | M;     N <= ALU[7]; Z <= ~|ALU;
//  1_010_00 11    EOR    ALU <=  A ^ M;     N <= ALU[7]; Z <= ~|ALU;
//
//  1_010_00 01    BIT    ALU <=  A & M;     N <= M[7];   Z <= ~|(A & M);
//                                           V <= M[6];
//  1_010_00 00    TRB    ALU <= ~A & M;                  Z <= ~|(A & M);
//  1_010_00 10    TSB    ALU <=  A & M;                  Z <= ~|(A & M);
//
//  1_010_00 00    RMBx   ALU <= ~K & M;     K <= (1 << IR[6:4]);
//  1_010_00 10    SMBx   ALU <=  K | M;     K <= (1 << IR[6:4]);
//  1_010_00 01    BBRx   ALU <=  K & M;     K <= (1 << IR[6:4]); CC_Out <= EQ;
//  1_010_00 01    BBSx   ALU <=  K & M;     K <= (1 << IR[6:4]); CC_Out <= NE;
//
//  1_010_00 00    CLC    ALU <= ~K & P;     K <= 1;    P <= ALU
//  1_010_00 10    SEC    ALU <=  K | P;     K <= 1;    P <= ALU
//  1_010_00 00    CLI    ALU <= ~K & P;     K <= 4;    P <= ALU
//  1_010_00 10    SEI    ALU <=  K | P;     K <= 4;    P <= ALU
//  1_010_00 00    CLD    ALU <= ~K & P;     K <= 8;    P <= ALU
//  1_010_00 10    SED    ALU <=  K | P;     K <= 8;    P <= ALU
//  1_010_00 00    CLV    ALU <= ~K & P;     K <= 64;   P <= ALU
//
//  1_001_00 00    ASL    ALU <= {Q[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= Q[7];
//                                           Ci <= 0; (V <= R[7] ^ R[6];) 
//  1_001_00 00    ROL    ALU <= {Q[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= Q[7];
//                                           Ci <= C; (V <= R[7] ^ R[6];)
//  1_001_00 01    LSR    ALU <= {Ci,Q[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= Q[0];
//                                           Ci <= 0; (Ci <= R[7])
//  1_001_00 01    ROR    ALU <= {Ci,Q[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= Q[0];
//                                           Ci <= C; 
//
//  1_000_10 00    ADC    ALU <= Q +  M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  1_000_10 01    SBC    ALU <= Q + ~M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  1_000_01 00    INC    ALU <= Q +  0 + 1; N <= ALU[7]; Z <= ~|ALU;
//  1_000_01 01    DEC    ALU <= Q + ~0 + 0; N <= ALU[7]; Z <= ~|ALU;
//  1_000_01 01    CMP    ALU <= Q + ~M + 1; N <= ALU[7]; Z <= ~|ALU;
//                                                        C <= COut;
//
//  Although the condition codes modified by the ALU are shown above, the CCSel
//  input field will actually control loading condition codes into P register.
//  This is especially true for the N and V condition codes with respect to the
//  BIT instruction. N and V are modified by BIT as indicated above, except for
//  for the BIT #imm instruction which only modifies Z, and leaves N and V un-
//  changed like the TRB and TSB instructions.
//
//  The ALU is controlled using a microprogrammed architecture. Explicit control
//  over the functional units, functional unit operation, left and right operand
//  select, and carry select provide the means by which the M65C02 execution
//  unit implements the execution phase of each instruction. A functional unit
//  is selected by the FU_Sel field. A single bit in the FU_Sel is used to
//  select the LST data multiplexer, the logic unit (LU), and the shift unit
//  (SU). The AU has two associated FU_Sel bits. One is used to select the AU
//  only for INC, DEC, and CMP functions, i.e. IDC select. The other is used to
//  select the AU when the ADC and SBC functions are needed. The decimal mode of
//  the 6502/65C02 only applies to ADC/SBC instructions, so the FU_Sel bit for
//  ADC/SBC, ADD select, is qualified by the Decimal flag in the PSW. If D is
//  set, then a decimal mode ADC/SBC is performed. If D is not set, then the
//  binary mode of the adder is used for ADC/SBC. The binary made of the adder
//  is always used for the INC/DEC/CMP instructions.
//
//  The LST multiplexer's output is selected by the OSel field. The operands of
//  the LU, SU, and AU are selected by the QSel, RSel, and CSel fields. These
//  fields are two bit fields, which allow them to select 1 of 4 operands. The
//  QSel is used for the left operand of the LU and AU, and for the single
//  operand of the SU. There are different combinations of the left operands for
//  the LU and the AU, so in essence there are two left operand multiplexers
//  controlled by a common set of control signals. The CSel selects the carry
//  input into both the SU and the AU.
//
//  The following table provides the mapping of the QSel, RSel, and CSel fields
//  to the various operands to the functional units.
//
//      QSel    RSel    QSel    CSel    QSel    RSel    CSel   
//      LU-L    LU-R    SU-L    SU-Ci   AU-L    AU-R    AU-Ci
//  00    M       M       M       C       M       M       C
//  01    K       K       X       0       X       K       0
//  10    P       P       Y       1       Y       P       1
//  11    A       A       A      Q[7]     A       A      Q[7]
//
//  The coding for the WSel and OSel fields follows:
//        3     3
//  Sel  WSel  OSel
//  000   -     0
//  001   X     X
//  010   Y     Y
//  011   A     A
//  100   -     T
//  101   S     S
//  110   P     P
//  111   M     M
//
//  The WSel field provides the register enable for writing into a register,
//  and the OSel field determines the value of the ALU result bus. Typically
//  the ALU result bus is the ALU. But on the occasion of a load, store, xfer,
//  push, and pull instructions, the ALU bus is driven by A, X, Y, 0, S, P, and
//  M. In this manner, the ALU result can be either loaded into the register
//  designated by the WSel field, or it can be written to memory.
//
//  The coding of these control fields and the ALU field is designed to yield a
//  0 at the output of the module if all the input control fields are logical
//  0. This means that a NOP is simply a zero for all ALU control fields.
//
//  The CC_Sel field controls the CC_Out condition code test signal and the 
//  updating of the individual bits in P in response to specific instructions:
//
//  0_xxx: NOP/TRUE - CC_Out =  1
//
//  1_000: CC       - CC_Out = ~C;
//  1_001: CS       - CC_Out =  C;
//  1_010: NE       - CC_Out = ~Z;
//  1_011: EQ       - CC_Out =  Z;
//  1_100: VC       - CC_Out = ~V;
//  1_101: VS       - CC_Out =  V;
//  1_110: PL       - CC_OUT = ~N;
//  1_111: MI       - CC_Out =  N;
//
//  x_000: pPSW     - P <= ALU;
//  x_001: pBRK     - P.4 <= 1 on push P during BRK ISR
//  x_010: Z        -                         Z <= ~|(A & M);
//  x_011: NVZ      - N <= M[7];   V <= M[6]; Z <= ~|(A & M); 
//  x_100: pPHP     - P.4 <= 1 on PHP
//  x_101: NZ       - N <= ALU[7];            Z <= ~|ALU;
//  x_110: NZC      - N <= ALU[7];            Z <= ~|ALU;     C <= COut
//  x_111: NVZC     - N <= ALU[7]; V <= OVF;  Z <= ~|ALU;     C <= COut;
//
// Dependencies:    M65C02A_ALU.v
//                      M65C02A_LST.v,      // Load/Store/Transfer Multiplexer
//                      M65C02A_LU.v,       // Logic Unit
//                      M65C02A_SU.v,       // Shift Unit
//                      M65C02A_Add.v,      // Adder Unit
//                      M65C02A_WrSel.v     // Write Select ROM
//                      M65C02A_RegStk.v    // Register Stack for A
//                      M65C02A_RegStkV2.v  // Register Stack for X/Y w/ StkPtr 
//                      M65C02A_PSW.v       // Processor Status Word (Register)
//
// Revision:
//
//  1.00    15A03   MAM     Initial release.
//
//  1.10    15B08   MAM     Modified the port list of the PSW module to account
//                          for the added Kernel/User mode register. Added ports
//                          for B and I; not necessary, but completes the expan-
//                          sion of P.
//
//  1.11    15F28   MAM     Added (but left commented out) logic to support COP
//                          status testing using the PSW.V flag.
//
//  1.20    15J25   MAM     Added the IND prefix flag to the A register stack
//                          to support byte swapping and nibble rotation in TOS.
//
//  1.30    15K25   MAM     Added the SIZ prefix flag to the A register stack
//                          to support direct transfers and exchanges between IP
//                          and TOS of A. Implements TAI, TIA, and XAI instruc-
//                          tions.
//
//  1.40    15L10   MAM     Added ADJ input port to support ADJ #imm16 instruc-
//                          tion, which allows direct adjustment of the system
//                          or auxiliary stack pointers.
//
//  1.50    15L19   MAM     Modified the K input to include a multiplexer that
//                          selects Y if ADJ asserted, or K if ADJ not asserted.
//
//  1.51    15L28   MAM     Corrected the J multiplexer previously modified to
//                          provide support for the ADJ instruction. When ADJ
//                          is asserted, the S bus is substituted for the X bus
//                          unless OSX is asserted in which case X is passed 
//                          through. If ADJ is not asserted, S is sent through
//                          when OSX is asserted else X is passed through. Modi-
//                          fied the SU to support an ASR function when LSR A is
//                          prefixed by IND. 
//
//  1.52    15L29   MAM     Added SelA, SelX, and SelY as qualifiers to the SU
//                          unit select so the ASR operation only applies when
//                          LSR is selected for a register: A, X, or Y. Other-
//                          wise, IND is an address modifier for the RMW LSR 
//                          instruction.
//
//  1.60    16I19   MAM     Reverted the ADJ instruction to use an immediate 
//                          operand. Removed O multiplexer, and reset E multi-
//                          plexer to K and M instead of O and M.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_ALU (
    input   Rst,            // System Reset - synchronous reset 
    input   Clk,            // System Clock
    
    input   Rdy,            // Ready
    
    input   En,             // Enable - ALU functions
    input   Reg_WE,         // Register Write Enable 
    input   ISR,            // Asserted on entry to Interrupt Service Routine
    
    input   SO,             // Set Overflow status bit Command
    output  Clr_SO,         // Acknowledge for SO Command
    
    input   COP_SO,         // Co-Processor Set oVerflow Strobe Input
                    
    //  Instruction Prefix Inputs and Supporting Controls
    
    input   IND,            // Alternate Register Stack Operation Select
    input   SIZ,            // Extend 8-bit operation into 16-bit operation
    input   OAX,            // Override: A <=> X 
    input   OAY,            // Override: A <=> Y 
    input   OSX,            // Override: S <=> X; X is stack pointer
    
    input   DUP,            // Duplicate Register Stack TOS
    input   SWP,            // Swap TOS and NOS
    input   ROT,            // Rotate Register Stack
    
    //  Address Generator (Stack Pointer) Interface
    
    output  SelS,           // Stack Pointer Select
    input   [15:0] S,       // Stack Pointer input from Address Generator
    
    input   Mod,            // % 256 Stack Pointer Limit Control Input
    input   [1:0] Stk_OpX,  // Auxiliary Stack Pointer Operation: NOP, POP, PSH
    input   [1:0] Stk_OpY,  // MOV Destination Pointer Operation: NOP, POP, PSH
    
    input   ADJ,            // Stack Pointer Adjust Instruction
    
    //  ALU Ports
    
    input   [4:0] FU_Sel,   // ALU Functional Unit Select
    input   [1:0] Op,       // ALU Operation Select
    input   [1:0] QSel,     // ALU Q Bus Multiplexer Select
    input   [1:0] RSel,     // ALU R Bus Multiplexer Select
    input   [1:0] CSel,     // ALU Carry In Multiplexer Select
    input   [2:0] WSel,     // ALU Register WE Select
    input   [2:0] OSel,     // ALU Output Multiplexer Select
    input   [3:0] CCSel,    // ALU Condition Code Operation Select
    
    input   [ 7:0] K,       // ALU Bit Mask
    input   [15:0] T,       // ALU FORTH VM Register Mux Input
    input   [15:0] M,       // ALU Memory Operand Input

    output  [15:0] DO,      // ALU Data Output(asynchronous)
    output  Val,            // ALU Output Valid

    output  reg CC_Out,     // Condition Code Test Output
    
    //  ALU Test Results
    
    output  ALU_C,          // ALU Carry Out
    output  ALU_Z,          // ALU Zero Out
    output  ALU_V,          // ALU oVerflow Out
    output  ALU_N,          // ALU Negative Out

    //  Internal Processor Registers
    
    output  [15:0] X,       // X Index Register
    output  [15:0] Y,       // Y Index Register
    output  [15:0] A,       // Accumulator Register
    
    output  [ 7:0] P        // Processor Status Word
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameter Decalarations
//

//  FU_Sel  Op   Mnemonic     Operation            Condition Code
//  1_00_00 00     XFR    ALU <= {OSel: M, P, S, Tmp, Y, X, A, 0}
//
//  0_10_00 01     AND    ALU <=  A & M;     N <= ALU[7]; Z <= ~|ALU;
//  0_10_00 10     ORA    ALU <=  A | M;     N <= ALU[7]; Z <= ~|ALU;
//  0_10_00 11     EOR    ALU <=  A ^ M;     N <= ALU[7]; Z <= ~|ALU;
//
//  0_10_00 01     BIT    ALU <=  A & M;     N <= M[7];   Z <= ~|(A & M);
//                                           V <= M[6];
//  0_10_00 00     TRB    ALU <= ~A & M;                  Z <= ~|(A & M);
//  0_10_00 10     TSB    ALU <=  A | M;                  Z <= ~|(A & M);
//
//  0_10_00 00     RMBx   ALU <= ~K & M;
//  0_10_00 10     SMBx   ALU <=  K | M;
//  0_10_00 01     BBRx   ALU <=  K & M;
//  0_10_00 01     BBSx   ALU <=  K & M;
//
//  0_10_00 00     CLC    ALU <= ~K & P;     C <= 0;
//  0_10_00 10     SEC    ALU <=  K | P;     C <= 1;
//  0_10_00 00     CLI    ALU <= ~K & P;     I <= 0;
//  0_10_00 10     SEI    ALU <=  K | P;     I <= 1;
//  0_10_00 00     CLD    ALU <= ~K & P;     D <= 0;
//  0_10_00 10     SED    ALU <=  K | P;     D <= 1;
//  0_10_00 00     CLV    ALU <= ~K & P;     V <= 0;
//
//  0_01_00 00     ASL    ALU <= {Q[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= Q[7];
//  0_01_00 00     ROL    ALU <= {Q[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= Q[7];
//  0_01_00 01     LSR    ALU <= {Ci,Q[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= Q[0];
//  0_01_00 01     ROR    ALU <= {Ci,Q[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= Q[0];
//
//  0_00_01 00     INC    ALU <= Q +  K + 1; N <= ALU[7]; Z <= ~|ALU;
//  0_00_01 00     DEC    ALU <= Q + ~K + 0; N <= ALU[7]; Z <= ~|ALU;
//  0_00_01 00     CMP    ALU <= Q + ~M + 1; N <= ALU[7]; Z <= ~|ALU;
//                                                        C <= COut;
//  0_00_10 00     ADC    ALU <= Q +  M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  0_00_10 00     SBC    ALU <= Q + ~M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  0_00_10 00     ADJ    ALU <= Q +  M + 0; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;

//  CCSel - Condition Code Select

localparam pTRUE = 4'd0;    // Set CC_Out TRUE
//
localparam pCC   = 4'd8;    // Set CC_Out if Carry Clear        (C Clear)
localparam pCS   = 4'd9;    // Set CC_Out if Carry Set          (C Set)
localparam pNE   = 4'd10;   // Set CC_Out if Not Equal to Zero  (Z Clear)
localparam pEQ   = 4'd11;   // Set CC_Out if Equal to Zero      (Z Set)
localparam pVC   = 4'd12;   // Set CC_Out if Not Overflow       (V Clear)
localparam pVS   = 4'd13;   // Set CC_Out if Overflow           (V Set)
localparam pPL   = 4'd14;   // Set CC_Out if Plus               (N Clear)
localparam pMI   = 4'd15;   // Set CC_Out if Negative           (N Set)
//
localparam pPSW  = 4'd0;    // Set P from ALU Output (PLP | Set/Clr instruction)
localparam pBRK  = 4'd1;    // Set P[4] when P pushed using BRK instruction
localparam pZ    = 4'd2;    // Set Z = ~|(A & M)
localparam pNVZ  = 4'd3;    // Set N and V flags from M[7:6], and Z = ~|(A & M)
localparam pPHP  = 4'd4;    // Set P[4] when P pushed using PHP instruction
localparam pNZ   = 4'd5;    // Set N and Z flags from ALU Output
localparam pNZC  = 4'd6;    // Set N, Z, and C flags from ALU Output
localparam pNVZC = 4'd7;    // Set N, V, Z, and C from ALU Output

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

//  Functional Unit Select/Enables

wire    En_LST;                 // Load/Store/Transfer Enable
wire    En_LU;                  // Logic Unit Enable
wire    En_SU;                  // Shift/Rotate Unit Enable
wire    En_AU;                  // Adder Unit Enable
wire    En_DU;                  // Decimal (BCD) Unit Enable

//  ALU Busses

wire    [15:0] B;               // A bus multiplexer for src reg overrides
wire    [15:0] J;               // X bus multiplexer for src reg overrides

wire    [15:0] G, H, L;         // LU Input Busses
wire    [15:0] W, U, Q;         // Adder Left Operand Input Busses (SU <= W)
wire    [15:0] E, F, R, O;      // Adder Right Operand Input Busses

reg     Ci;                     // Adder Carry In signal

//  ALU Component Output Busses

wor     [16:0] Out;             // ALU Output
wor     OV;                     // ALU Adder Overflow Flag
wor     Valid;                  // ALU Output Valid

wire    LU_Z;                   // Zero Detector for BIT/TRB/TSB/BBR/BBS

//  ALU Registers

wire    SelA, SelX, SelY, SelP; // ALU Register Selects

wire    N, V, D, Z, C;          // ALU PSW flags

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  ALU Functional Unit Selection

assign En_LST = En & FU_Sel[4];     // Load/Store/Transfer
assign En_LU  = En & FU_Sel[3];     // AND/OR/XOR/BIT/TRB/TSB/RMB/SMB/BBR/BBS
assign En_SU  = En & FU_Sel[2];                       // Shift/Rotate
assign En_AU  = En & ((FU_Sel[1] & ~D) | FU_Sel[0]);  // Binary Adder
assign En_DU  = En &  (FU_Sel[1] &  D);               // Decimal Adder

//  If no functional unit selected, then assert Valid

assign Valid = ~|FU_Sel;

//  B Multiplexer - Used to implement ALU Source Register Overrides

assign B = ((OAX) ? X : ((OAY) ? Y : A));

//  J Multiplexer - Used to implement ALU Source Register Overrides

assign J = ((ADJ) ? ((OSX) ? X : S) : ((OSX) ? S : X));

//  LU (Left Operand) Multiplexer
//      00: L = M; 01: L = K;
//      10: L = P; 11: L = A;

assign G = ((QSel[0]) ? K : M);     // ? K : M;
assign H = ((QSel[0]) ? B : P);     // ? A : P; 
assign L = ((QSel[1]) ? H : G);

//  AU/SU (Left Operand) Multiplexer
//      00: Q = M; 01: Q = X;
//      10: Q = Y; 11: Q = A;

assign W = ((QSel[0]) ? J : M);     // ? X : M; INX/DEX/CPX, INC/DEC/CMP/ADC/SBC
assign U = ((QSel[0]) ? B : Y);     // ? A : Y; INC/DEC/CMP/ADC/SBC, INY/DEY/CPY 
assign Q = ((QSel[1]) ? U : W);

//  LU/AU (Right Operand) Multiplexer
//      00: R = M; 01: R = K;
//      10: R = P; 11: R = A;

assign E = ((RSel[0]) ? K : M);     // ? K : M; INC/DEC, ADC/SBC/CMP
assign F = ((RSel[0]) ? B : P);     // ? A : P; INC/DEC, CLC/.../CLV
assign R = ((RSel[1]) ? F : E);

// Carry In Multiplexer

always @(*)
begin
    case(CSel)
        2'b00 : Ci <= C;            // ADC/SBC/ROL/ROR
        2'b01 : Ci <= 0;            // DEC/ASL/LSR
        2'b10 : Ci <= 1;            // INC/CMP         
        2'b11 : Ci <= N;            // ASR (Reserved for future use)
    endcase
end

//  Load/Store/Transfer Multiplexer Instantiation
//  0   -   : STZ
//  1   X   : STX/TXA/TXS/PHX
//  2   Y   : STY/TYA/PHY
//  3   A   : STA/TAX/TAY/PHA
//  4   T   : IP/W
//  5   S   : TSX
//  6   P   : PHP
//  7   M   : LDA/PLA/LDX/PLX/LDY/PLY/PLP/PHW/PLR/PHI/PLI

M65C02A_LST LST (
                .En(En_LST), 

                .OAX(OAX),          // Override: Swap A and X
                .OAY(OAY),          // Override: Swap A and Y
                .OSX(OSX),          // Override: Swap X and S, and use X as SP
                
                .OSel(OSel),
                
                .X(X),
                .Y(Y),
                .A(A),
                .T(T),
                .S(S), 
                .P({8'b0, P}), 
                .M(M), 

                .Out(Out), 
                .Val(Valid)
            );

// Logic Unit Instantiation
//      AND/ORA/EOR/BIT/TRB/TSB
//      RMBx/SMBx/BBRx/BBSx
//      SEC/CLC/SEI/CLI/SED/CLD/CLV

M65C02A_LU  LU (
                .En(En_LU),
                
                .SIZ(SIZ),
                
                .Op(Op),
                .L(L),
                .M(R),
                
                .Out(Out),
                .Z(LU_Z),
                .Val(Valid)
            );

//  Shift Unit Instantiation
//      SU, Ci - ASL/ROL/LSR/ROR
//          support for adding ASR instruction is included
//          support for adding ASL overflow detection also included

assign En_ASR = Op[0] & IND & (SelX | SelY | SelA);

M65C02A_SU  SU (
                .En(En_SU),
                
                .SIZ(SIZ),
                
                .Op(Op[0]),
                .SU(Q),
                .Ci((En_ASR) ? (V ^ ((SIZ) ? Q[15] : Q[7])) : Ci),
                
                .Out(Out),
                .OV(OV),
                .Val(Valid)
            );

//  Adder (Binary and Decimal Adders) Instantiation
//      Binary Adder Unit Implementation (INC/INX/INY/DEC/DEX/DEY/CMP/ADC/SBC)
//      Decimal (BCD) Adder Unit Implementation (ADC/SBC (Decimal-Only))

M65C02A_Add AU (
                .Rst(Rst), 
                .Clk(Clk),
                
                .SIZ(SIZ),
                
                .En_AU(En_AU), 
                .En_DU(En_DU), 
                .Op(Op[0]), 
                
                .Q(Q), 
                .R(R), 
                .Ci(Ci), 
                
                .Out(Out),
                .OV(OV),
                .Val(Valid)
            );
            
//  Define ALU Output

assign DO    = ((SIZ) ? Out : {8'h00, Out[7:0]});
assign ALU_C = Out[16];
assign ALU_Z = ~|DO;
assign ALU_V = OV;
assign ALU_N = ((SIZ) ? Out[15] : Out[7]);
assign Val   = Valid;

//  Condition Code Output
//      Additional multiplexer to support BBRx/BBSx instructions included.
//      In 16-bit arithmetic, i.e. ADC/SBC/CMP/CPX/CPY, the NVZC flags are set,
//      and the tests have been modified to support both signed and unsigned
//      conditionals. Branch instructions preceded by the SIZ prefix perform
//      multi-flag tests to provide a greater range of conditional branches:
//
//      signed:     BMI => BLE;  BPL => BGT; BVS => BLT; BVC => BGE;
//      unsigned:   BEQ => BHIS; BNE => BLO; BCS => BHI; BCC => BLOS;
//
//      In this case, the SIZ prefix instruction is not used to increase the
//      width of the relative branch offset, but instead it is used to access
//      another set of conditional tests to better support extended arithmetic
//      comparisons of 16-bit operands. 

always @(posedge Clk)
begin
    if(Rst)
        CC_Out <= #1 1;
    else
        case(CCSel)
            pCC     : CC_Out <= #1 ~C;
            pCS     : CC_Out <= #1  C;
            pNE     : CC_Out <= #1 ((SIZ) ? ~(C | Z) : ((En_LU) ? ~LU_Z : ~Z));
            pEQ     : CC_Out <= #1 ((SIZ) ?  (C | Z) : ((En_LU) ?  LU_Z :  Z));
            pVC     : CC_Out <= #1 ((SIZ) ? ~((N ^ V)    ) : ~V);
            pVS     : CC_Out <= #1 ((SIZ) ?  ((N ^ V)    ) :  V);
            pPL     : CC_Out <= #1 ((SIZ) ? ~((N ^ V) | Z) : ~N);
            pMI     : CC_Out <= #1 ((SIZ) ?  ((N ^ V) | Z) :  N);
            default : CC_Out <= #1  1;
        endcase 
end

////////////////////////////////////////////////////////////////////////////////
//
//  Internal ALU Registers (Distributed Dual-Port SRAM)
//

M65C02A_WrSel   WrSel (
                    .Rst(Rst),
                    .Clk(Clk),
                    
                    .OAX(OAX),
                    .OAY(OAY),
                    .OSX(OSX),
                    
                    .WE(Reg_WE), 
                    .WSel(WSel),
                    
                    .SelA(SelA), 
                    .SelX(SelX), 
                    .SelY(SelY), 
                    .SelP(SelP),

                    .SelS(SelS) 
                );

////////////////////////////////////////////////////////////////////////////////
//
//  X - Pre-Index Register
//

M65C02A_RegStkV2    RegX (
                        .Rst(Rst),
                        .Clk(Clk),
                        
                        .Rdy(Rdy),
                        .Valid(Valid),
                        
                        .Sel(SelX),
                        
                        .DUP((OAX ? DUP : 1'b0)),
                        .SWP((OAX ? SWP : 1'b0)),
                        .ROT((OAX ? ROT : 1'b0)),
                        
                        .D(DO),
                        .Q(X),
                        
                        .Page(1'b0),
                        .Size(SIZ),
                        .Mod(Mod),
                        
                        .Stk_Op(Stk_OpX),
                        
                        .TOS(),
                        .NOS(),
                        .BOS()
                    );

////////////////////////////////////////////////////////////////////////////////
//
//  Y - Post-Index Register
//

M65C02A_RegStkV2    RegY (
                        .Rst(Rst),
                        .Clk(Clk),
                        
                        .Rdy(Rdy),
                        .Valid(Valid),
                        
                        .Sel(SelY),
                        
                        .DUP((OAY ? DUP : 1'b0)),
                        .SWP((OAY ? SWP : 1'b0)),
                        .ROT((OAY ? ROT : 1'b0)),
                        
                        .D(DO),
                        .Q(Y),
                        
                        .Page(1'b0),
                        .Size(1'b1),
                        .Mod(1'b0),
                        
                        .Stk_Op(Stk_OpY),
                        
                        .TOS(),
                        .NOS(),
                        .BOS()
                    );

////////////////////////////////////////////////////////////////////////////////
//
//  A - Accumulator
//

M65C02A_RegStk  RegA (
                    .Rst(Rst),
                    .Clk(Clk),
                    
                    .Rdy(Rdy),
                    .Valid(Valid),
                    
                    .Sel(SelA),
                    
                    .IND(IND),
                    .SIZ(SIZ),
                    
                    .DUP((~(OAX | OAY) ? DUP : 1'b0)),
                    .SWP((~(OAX | OAY) ? SWP : 1'b0)),
                    .ROT((~(OAX | OAY) ? ROT : 1'b0)),
                    
                    .D(DO),
                    .T(T),
                    
                    .Q(A),
                    
                    .TOS(),
                    .NOS(),
                    .BOS()
                );

////////////////////////////////////////////////////////////////////////////////
//
//  P - Processor Status Word: {N, V, M, B, D, I, Z, C}
//
//  A circuit has been added for 16-bit arithmetic that forces the ALU into
//  binary mode. The decimal mode support in the adder functional unit is res-
//  tricted to the 8-bit mode. Performing 16-bit BCD addition/subtraction must
//  be done using cascaded 8-bit operations using the normal 6502/65C02 instruc-
//  tion set; the 16-bit added does not support decimal mode arithmetic.

wire    iD;     // Temporary Decimal Status

M65C02A_PSW     PSW (
                    .Clk(Clk),
                    
                    .SO(SO), 
                    .Clr_SO(Clr_SO),

                    .COP_SO(COP_SO),

                    .SelP(SelP), 
                    .Valid(Valid), 
                    .Rdy(Rdy),
                    
                    .ISR(ISR), 

                    .CCSel(CCSel), 

                    .ALU_C(ALU_C), 
                    .ALU_Z(ALU_Z),
                    .ALU_V(ALU_V),
                    .ALU_N(ALU_N),

                    .LU_Z(LU_Z),
                    .M(((SIZ) ? M[15:14] : M[7:6])),

                    .DO(DO),

                    .P(P),
                    
                    .N(N),
                    .V(V),
                    .K(),
                    .B(),
                    .D(iD),
                    .I(),
                    .Z(Z),
                    .C(C)
                );
                
assign D = iD & ~SIZ;   // Decimal mode only applies to 8-bit arithmetic

endmodule
