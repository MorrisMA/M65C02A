////////////////////////////////////////////////////////////////////////////////
//
//  ALU module for M65C02A soft-core microcomputer project.
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
// Create Date:     07:00:31 11/25/2009 
// Design Name:     WDC W65C02 Microprocessor Re-Implementation
// Module Name:     M65C02_ALUv2.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module implements the ALU for the M65C02 microprocessor. Included in
//  the ALU are the accumulator (A), pre/post-index registers (X and Y), and 
//  processor status word (P). The microcode, composed of a fixed part from the
//  instruction decoder (IDEC) and a variable portion from the execution engine,
//  control a Load/Store/Transfer (LST) multiplexer, a Logic Unit (LU), a Shift/
//  Rotate Unit (SU), and a dual-mode (binary/BCD) Adder Unit (AU).
//
//  The Load/Store/Transfer (LST) multiplexer allows the various registers in
//  the ALU, plus the external stack pointer (S) in the address generator module
//  (from the M65C02 Address Generator), to be multiplexed onto the ALU output
//  data bus. This allows registers to be loaded from memory (data memory or the
//  external data/return stack), stored to memory, or transfered from one regis-
//  ter to another.
//
//  The LU performs bit-wise operations (OR, AND, and XOR) on two operands. The
//  LU also supports the following instructions which only use variations of the
//  LU's basic functions: BIT/TRB/TSB, RMB/SMB/BBR/BBS, CLC/SEC/CLI/SEI/CLD/SED/
//  CLV, and REP/SEP
//
//  The SU performs arithmetic and logical operations using non-circular or cir-
//  cular shifts on a register (A) or a memory (M) operand. The four basic in-
//  structions, ASL/ROL/LSR/ROR, can be implemented using just two operations:
//  left shift and right shift. Various combinations of the operand and the
//  carry logic signal paths allow the non-circular shifts and circular shifts
//  (rotates) needed to implement these four instructions. The ALU can easily
//  support arithmetic overflow detection on the ASL instruction, and support an
//  arithmetic right shift (ASR) through modifications to the microprogram ROMs.
//
//  The AU performs binary two's complement addition, or unsigned BCD addition/
//  subtraction on two operands. Subtraction is performed using 1's complement
//  logic. A 2's complement result, extended precision addition or subtraction
//  is performed if the Carry flag, C, is cleared before addition and set before
//  subtraction. The AU is also used for increments, decrements, and compares.
//  For these operations, the arithmetic mode is fully 2's complement.
//
//  The ALU requires the En input to be asserted to initiate the operation that
//  the fixed IDEC microword determines. Asserting the En input indicates that
//  the required operands for an operation are available. The addressing mode of
//  the instruction will determine when En is asserted. In the case of non-
//  program flow control instructions, En alone controls the updates to these
//  registers: A, X, Y, P. For implicit or accumulator mode instructions, En is
//  generally asserted when the Instruction Register (IR) is loaded with the op-
//  code. Otherwise, it is asserted when the required memory operand is availa-
//  ble from memory (M). The operation defined by IDEC should be completed with
//  the available operands in the ALU registers and the memory operand (M).
//
//  With the En input asserted, then the Rdy input controls the actual writing
//  of registers. To write a register, the En and Rdy inputs must be asserted, 
//  and the register must be selected. A register is written if its code is set
//  in the WSel field and the Reg_WE field is set to the pWr_Reg control value,
//  or if the register's select code is set in the Reg_WE field. A second level
//  ROM in the ALU provides the decoding of this logic and generates write 
//  selects for each of the ALU registers (A, X, Y, P) and for S in the address
//  generator module.
//
//  The table below presents all of the valid combinations of the FU_Sel and Op
//  fields. The FU_Sel field selects the functional unit that implements the 
//  desired operation. The Op input selects the operation to be performed by LU,
//  SU, and AU functional units.
//
//      LST: LST = {M, S, P, T, Y, X, A, 0}
//      LU : LU  = {P, K, A, M}, R = {P, K, A, M}; Ci = {1, 0, Q[7], C}
//      SU : SU  = {Y, X, A, M},                   Ci = {1, 0, Q[7], C} 
//      AU : Q   = {Y, X, A, M}, R = {P, K, A, M}; Ci = {1, 0, Q[7], C}
//  
//   FU_Sel  Op  Mnemonic     Operation           Condition Code
//
//  11_00_00 00    XFR    ALU <= {OSel=(7..0): M, S, P, T, Y, X, A, 0}
//
//  10_10_00 01    AND    ALU <=  A & M;     N <= ALU[7]; Z <= ~|ALU;
//  10_10_00 10    ORA    ALU <=  A | M;     N <= ALU[7]; Z <= ~|ALU;
//  10_10_00 11    EOR    ALU <=  A ^ M;     N <= ALU[7]; Z <= ~|ALU;
//
//  10_10_00 01    BIT    ALU <=  A & M;     N <= M[7];   Z <= ~|(A & M);
//                                           V <= M[6];
//  10_10_00 00    TRB    ALU <= ~A & M;                  Z <= ~|(A & M);
//  10_10_00 10    TSB    ALU <=  A | M;                  Z <= ~|(A & M);
//
//  10_10_00 00    RMBx   ALU <= ~K & M;     K <= (1 << IR[6:4]);
//  10_10_00 10    SMBx   ALU <=  K | M;     K <= (1 << IR[6:4]);
//  10_10_00 01    BBRx   ALU <=  K & M;     K <= (1 << IR[6:4]); CC_Out <= EQ;
//  10_10_00 01    BBSx   ALU <=  K & M;     K <= (1 << IR[6:4]); CC_Out <= NE;
//
//  10_10_00 00    CLC    ALU <= ~K & P;     K <= 1;    P <= ALU
//  10_10_00 10    SEC    ALU <=  K | P;     K <= 1;    P <= ALU
//  10_10_00 00    CLI    ALU <= ~K & P;     K <= 4;    P <= ALU
//  10_10_00 10    SEI    ALU <=  K | P;     K <= 4;    P <= ALU
//  10_10_00 00    CLD    ALU <= ~K & P;     K <= 8;    P <= ALU
//  10_10_00 10    SED    ALU <=  K | P;     K <= 8;    P <= ALU
//  10_10_00 00    CLV    ALU <= ~K & P;     K <= 64;   P <= ALU
//
//  10_10_00 00    REP    ALU <= ~M & P;                P <= ALU
//  10_10_00 00    SEP    ALU <=  M | P;                P <= ALU
//  
//  10_01_00 00    ASL    ALU <= {R[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= R[7];
//                                           Ci <= 0; (V <= R[7] ^ R[6];) 
//  10_01_00 00    ROL    ALU <= {R[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= R[7];
//                                           Ci <= C; (V <= R[7] ^ R[6];)
//  10_01_00 01    LSR    ALU <= {Ci,R[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= R[0];
//                                           Ci <= 0; (Ci <= R[7])
//  10_01_00 01    ROR    ALU <= {Ci,R[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= R[0];
//                                           Ci <= C; 
//
//  10_00_10 00    ADC    ALU <= Q +  M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  10_00_10 01    SBC    ALU <= Q + ~M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  10_00_01 00    INC    ALU <= Q +  0 + 1; N <= ALU[7]; Z <= ~|ALU;
//  10_00_01 01    DEC    ALU <= Q + ~0 + 0; N <= ALU[7]; Z <= ~|ALU;
//  10_00_01 01    CMP    ALU <= Q + ~M + 1; N <= ALU[7]; Z <= ~|ALU;
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
//  over the functional units, functional unit operations, left operand select,
//  right operand select, and carry select provide the means by which the M65C02
//  execution unit implements the execution phase of each instruction. A func-
//  tional unit is selected by the FU_Sel field. A single bit in the FU_Sel is
//  used to select the LST data multiplexer, the logic unit (LU), and the shift
//  unit (SU). The AU has two associated FU_Sel bits. One is used to select the
//  AU only when INC, DEC, and CMP functions, i.e. IDC select. The other is used
//  to select the AU when the ADC and SBC functions are needed. The decimal mode
//  of the 6502/65C02 only applies to ADC/SBC instructions, so the FU_Sel bit
//  for ADC/SBC, ADD select, is qualified by the Decimal flag in the PSW. If D
//  is set, then a decimal mode ADC/SBC is performed. If D is not set, then the
//  binary mode of the adder is used to INC/DEC/CMP and ADC/SBC.
//
//  The LST multiplexer's operands are selected by the OSel field. The operands
//  of the LU, SU, and AU are selected by the QSel, RSel, and CSel fields. These
//  fields are two bit fields, which allow them to select 1 of 4 operands. The
//  QSel is used for the left operand of the LU and AU, and for the single
//  operand of the SU. There are different combinations of the left operands for
//  the LU and the AU, so in essence there are two left operand multiplexers
//  controlled a common set of control signals. The CSel selects the carry input
//  into both the SU and the AU.
//
//  The following table provides the mapping of the QSel, RSel, and CSel fields
//  to the various operands to the functional units.
//
//      QSel    RSel    QSel    CSel    QSel    RSel    CSel   
//      LU-L    LU-R    SU-L    SU-Ci   AU-L    AU-R    AU-Ci
//  00    M       M       M       C       M       M       C
//  01    A       A       A      Q[7]     A       A      Q[7]
//  10    K       K       X       0       X       K       0
//  11    P       P       Y       1       Y       P       1
//
//  The coding for the WSel and OSel fields follows:
//        3     3
//  Sel  WSel  OSel
//  000   -     0
//  001   A     A
//  010   X     X
//  011   Y     Y
//  100   -     Tmp
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
// Dependencies:    M65C02_ALUv2.v
//                      M65C02_LST.v,       // Load/Store/Transfer Multiplexer
//                      M65C02_LU.v,        // Logic Unit
//                      M65C02_SU.v,        // Shift Unit
//                      M65C02_Add.v,       // Adder Unit
//                      M65C02_WrSel.v      // Write Select ROM
//                      M65C02_PSW.v        // Processor Status Word (Register)
//
// Revision:
// 
//  1.00    13I16   MAM     Ported the M65C02_ALU.v module. Removed the Rockwell
//                          instructions, modified the operation code to support
//                          a one-hot 5-bit functional unit select and a 2-bit
//                          operation select; combined the Logic Unit (LU) and
//                          the BIT Unit (BU) into a single module; combined the
//                          binary mode adder module, M65C02_BIN.v, and the
//                          decimal mode adder module, M65C02_BCD.v, into a
//                          single, dual mode adder module, M65C02_Adder.v; re-
//                          moved the stack pointer module and added ports to 
//                          import the stack pointer (for TSX support) and ex-
//                          port a stack pointer write enable, SelS; and created
//                          independent modules for the LU, SU, LST, and regis-
//                          ter write enable ROM for a more modular implementa-
//                          tion. Finally, implement wor busses for ALU_DO, Val,
//                          Co, and OV. All these changes provide an increase
//                          in speed from ~73 MHz to ~101 MHz in a -4 Spartan 3A
//                          FPGA.
//
//  1.10    13J01   MAM     Modified the integrated version to support an ALU
//                          that incorporates support for more instructions, and
//                          maintains compatibility with the capabilities of the
//                          M65C02 ALU, core, and microprocessor. The objective
//                          being an ALU and core that can be configured to pro-
//                          vide the instruction sets of the MOS6502, GSC65SC02,
//                          W65C02S, and the M65C02A simply by supplying new
//                          contents for the microprogram and instruction de-
//                          coder ROMs.
//
//  1.20    13J07   MAM     Significantly improved the description section. Made
//                          minor changes to the definition of the QSel, RSel, 
//                          and CSel fields. When integrated into the
//                          M65C02_CoreV2 module, the result is a reported speed
//                          improvement of 5-6%, but also a slight reduction in
///                         the number of slices and LUTs required such that the
//                          ALU only requires ~220 LUTs, 59 Registers, and 155
//                          slices.
//
//  1.30    14F21   MAM     Modified the R multiplexer to select {P, K, A, M}
//                          instead of {P, 0, A, M}. This modification allows
//                          the microprogram to support AU INC/DEC by any value
//                          including 1 or -1, and to support LU operations with
//                          its own mask values.
//
//  1.31    14G04   MAM     Exposed the Accumulator as a module port.
//
// Additional Comments:
//
//  Revision 1.10 of the module incorporates all those features required to pro-
//  vide the instruction set capabilities of the MOS6502, GSC65SC02, W65C02S,
//  and the M65C02A simply by updating the contents of the microprogram ROMs and
//  no logic changes. The ALU control signals provided by the microprogram and
//  instruction decoder ROMs have been modified to provide additional control of
//  new operand multiplexers. The new multiplexers will allow the implementation
//  of all M65C02/W65C02S instructions using only four functional units: Load/
//  Store/Transfer (LST) unit, Logic Unit (LU), Shift/Rotate Unit (SU), and an
//  Arithmetic Unit (AU). Additional multiplexers have been added, and the Op, 
//  QSel, RSel, Sub, and CSel control fields have been modified and overloaded.
//
//  The result is an ALU substantially smaller and faster than the ALU in the
//  M65C02. First, the separate functional unit for implementing the Rockwell
//  instructions (RMBx/SMBx/BBRx/BBSx) is no longer required. Those instructions
//  are now provided using only the Logic Unit. Second, capabilities have been
//  added to the LU operand source select multiplexer so that it is possible to
//  perform logic operations on the PSW (P) so that the REP/SEP instructions
//  from the WDC '816/802 can be included simply by microprogram and instruction
//  decode ROM changes. Third, it will be possible to include the '816/802 PEA
//  and PEI instructions as well.
//
//  One of the drivers in the improved performance of this ALU is the use of
//  wired-OR busses for the outputs of the functional units. The output bus of
//  the ALU_DO is driven by all the functional units. If the unit is not 
//  selected, ALU_DO is driven by the functional unit to 0. A similar implemen-
//  tation is used for the Co, OV, and data Valid signals.
//
//  Another driver in the improved performance of the ALU is the use of, pre-
//  decoded one-hot functional unit select signals as part of the instruction
//  decode ROM. These signals significantly reduce the number of logic levels
//  needed to implement the functional unit selects and functional unit output
//  multiplexers required in the ALU. The wired-OR busses are more compatible
//  with the FPGA internal architecture, and the synthesizer is able to better
//  optimize the number of logic levels needed to implement the muxes.
//
//  If there wasn't a limitation that a single 18kb Block RAM can be configured
//  to support, the instruction decode ROM width could be widened even more than
//  the 36 bits required. This would result in additional performance because
//  additional one-hot control bits could be included. The additional decrease
//  in the logic levels required to implement the ALU would result in another
//  significant increase in speed.
//  
//  Given the target FPGA, the XC3S50A, there are no additional Block RAMs that
//  can be used to provide the additional width in both the instruction decode 
//  and the microprogram ROMs. The second target FPGA, the XC3S200A in the same
//  VQ100 package, can provide additional Block RAMs for use in the instruction
//  and microprogram ROMs.
//
//  A final addition to improve the compatibility of the M65C02A to the W65C02S
//  has been the inclusion of an external Set_Overflow port. The nSO pin on the
//  W65C02 and MOS6502 sets the V flag in the PSW whenever a falling edge is
//  asserted on that external pin of those processors. The ALU now incorporates
//  the capability to include a pulsed signal, SO, that will force the V bit to
//  be set whenever the PSW is updated, or at the completion of any instruction
//  that doesn't set the PSW. In other words, the V is forced to a logic 1 on an
//  instruction boundary if SO is asserted.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02_ALUv2 (
    input   Rst,            // System Reset - synchronous reset 
    input   Clk,            // System Clock
    
    input   Rdy,            // Ready
    
    input   En,             // Enable - ALU functions
    input   [2:0] Reg_WE,   // Register Write Enable 
    input   ISR,            // Asserted on entry to Interrupt Service Routine
    
    input   SO,             // Set Overflow status bit Command
    output  Clr_SO,         // Acknowledge for SO Command

    //  Address Generator (Stack Pointer) Interface
    
    output  SelS,           // Stack Pointer Select
    input   [7:0] S,        // Stack Pointer input from Address Generator
    
    //  ALU Ports
    
    input   [4:0] FU_Sel,   // ALU Functional Unit Select
    input   [1:0] Op,       // ALU Operation Select
    input   [1:0] QSel,     // ALU Q Bus Multiplexer Select
    input   [1:0] RSel,     // ALU R Bus Multiplexer Select
    input   [1:0] CSel,     // ALU Carry In Multiplexer Select
    input   [2:0] WSel,     // ALU Register WE Select
    input   [2:0] OSel,     // ALU Output Multiplexer Select
    input   [3:0] CCSel,    // ALU Condition Code Operation Select
    
    input   [7:0] K,        // ALU Bit Mask for Rockwell Instructions
    input   [7:0] Tmp,      // ALU Temporary Operand Holding Register
    input   [7:0] M,        // ALU Memory Operand Input
    output  [7:0] DO,       // ALU Data Output(asynchronous)
    output  Val,            // ALU Output Valid

    output  reg CC_Out,     // Condition Code Test Output

    //  Internal Processor Registers
    
    output  reg [7:0] A,    // Accumulator Register
    output  reg [7:0] X,    // X Index Register
    output  reg [7:0] Y,    // Y Index Register
    
    output  [7:0] P         // Processor Status Word
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameter Decalarations
//

//  {FU_Sel, Op} Encodings

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
//  0_10_00 00     REP    ALU <= ~M & P;     P <= ALU;
//  0_10_00 10     SEP    ALU <=  M | P;     P <= ALU;
//
//  0_01_00 00     ASL    ALU <= {R[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= R[7];
//  0_01_00 00     ROL    ALU <= {R[6:0],Ci} N <= ALU[7]; Z <= ~|ALU; C <= R[7];
//  0_01_00 01     LSR    ALU <= {Ci,R[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= R[0];
//  0_01_00 01     ROR    ALU <= {Ci,R[7:1]} N <= ALU[7]; Z <= ~|ALU; C <= R[0];
//
//  0_00_01 00     INC    ALU <= Q +  K + 1; N <= ALU[7]; Z <= ~|ALU;
//  0_00_01 00     DEC    ALU <= Q + ~K + 0; N <= ALU[7]; Z <= ~|ALU;
//  0_00_01 00     CMP    ALU <= Q + ~M + 1; N <= ALU[7]; Z <= ~|ALU;
//                                                        C <= COut;
//  0_00_10 00     ADC    ALU <= Q +  M + C; N <= ALU[7]; Z <= ~|ALU;
//                                           V <= OV;     C <= COut;
//  0_00_10 00     SBC    ALU <= Q + ~M + C; N <= ALU[7]; Z <= ~|ALU;
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

wire    [7:0] G, H, L;          // LU Input Busses
wire    [7:0] W, U, Q;          // Adder Left Operand Input Busses (SU <= W)
wire    [7:0] E, F, R;          // Adder Right Operand Input Busses

reg     Ci;                     // Adder Carry In signal

//  ALU Component Output Busses

wor     [8:0] Out;              // ALU Output
wor     OV;                     // ALU Adder Overflow Flag
wor     Valid;                  // ALU Output Valid

wire    COut;                   // Carry Out -> input to C
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

//  LU (Left Operand) Multiplexer
//      00: L = M; 01: L = A;
//      10: L = K; 11: L = P;

assign G = ((QSel[0]) ? A : M);     // ? A : M;
assign H = ((QSel[0]) ? P : K);     // ? P : K; 
assign L = ((QSel[1]) ? H : G);

//  AU/SU (Left Operand) Multiplexer
//      00: Q = M; 01: Q = A;
//      10: Q = X; 11: Q = Y;

assign W = ((QSel[0]) ? A : M);     // ? A : M; INC/DEC/CMP/ADC/SBC
assign U = ((QSel[0]) ? Y : X);     // ? Y : X; INY/DEY/CPY, INX/DEX/CPX 
assign Q = ((QSel[1]) ? U : W);

//  LU/AU (Right Operand) Multiplexer
//      00: R = M; 01: R = A;
//      10: R = 0; 11: R = P;

assign E = ((RSel[0]) ? A : M);     // ? A : M; ADC/SBC/CMP
assign F = ((RSel[0]) ? P : K);     // ? P : K; INC/DEC, CLC/.../CLV, REP/SEP
assign R = ((RSel[1]) ? F : E);

// Carry In Multiplexer

always @(*)
begin
    case(CSel)
        2'b00 : Ci <= C;            // ADC/SBC/ROL/ROR
        2'b01 : Ci <= Q[7];         // ASR (Reserved for future use)
        2'b10 : Ci <= 0;            // DEC/ASL/LSR
        2'b11 : Ci <= 1;            // INC/CMP         
    endcase
end

//  Load/Store/Transfer Multiplexer Instantiation
//  0   -   : STZ
//  1   A   : STA/TAX/TAY/PHA/TAS
//  2   X   : STX/TXA/TXS/PHX/SAX/TXY
//  3   Y   : STY/TYA/PHY/SAY/TYX
//  4   Tmp : PEA/PEI (lo byte held in Tmp, Tmp pushed to stack after hi byte)
//  5   S   : TSX/TSA
//  6   P   : PHP
//  7   M   : LDA/PLA/LDX/PLX/LDY/PLY/PLP

M65C02_LST  LST (
                .En(En_LST), 
                .OSel(OSel),
                
                .A(A),
                .X(X),
                .Y(Y),
                .Tmp(Tmp),
                .S(S), 
                .P(P), 
                .M(M), 

                .Out(Out), 
                .Val(Valid)
            );

// Logic Unit Instantiation
//      AND/ORA/EOR/BIT/TRB/TSB
//      RMBx/SMBx/BBRx/BBSx
//      SEC/CLC/SEI/CLI/SED/CLD/CLV
//      REP/SEP

M65C02_LU   LU (
                .En(En_LU),
                
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

M65C02_SU   SU (
                .En(En_SU),
                
                .Op(Op[0]),
                .SU(Q),
                .Ci(Ci),
                
                .Out(Out),
                .OV(OV),
                .Val(Valid)
            );

//  Adder (Binary and Decimal Adders) Instantiation
//      Binary Adder Unit Implementation (INC/INX/INY/DEC/DEX/DEY/CMP/ADC/SBC)
//      Decimal (BCD) Adder Unit Implementation (ADC/SBC (Decimal-Only))

M65C02_Add  AU (
                .Rst(Rst), 
                .Clk(Clk),
                
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

assign DO   = Out[7:0];
assign COut = Out[8];
assign Val  = Valid;

//  Condition Code Output
//      Additional multiplexer to support BBRx/BBSx instructions included.

always @(posedge Clk)
begin
    if(Rst)
        CC_Out <= #1 1;
    else
        case(CCSel)
            pCC     : CC_Out <= #1 ~C;
            pCS     : CC_Out <= #1  C;
            pNE     : CC_Out <= #1 ((En_LU) ? ~LU_Z : ~Z);
            pEQ     : CC_Out <= #1 ((En_LU) ?  LU_Z :  Z);
            pVC     : CC_Out <= #1 ~V;
            pVS     : CC_Out <= #1  V;
            pPL     : CC_Out <= #1 ~N;
            pMI     : CC_Out <= #1  N;
            default : CC_Out <= #1  1;
        endcase 
end

////////////////////////////////////////////////////////////////////////////////
//
//  Internal ALU Registers (Distributed Dual-Port SRAM)
//

M65C02_WrSel    WrSel (
                    .Rst(Rst),
                    .Clk(Clk),
                    
                    .Reg_WE(Reg_WE), 
                    .WSel(WSel),
                    
                    .SelA(SelA), 
                    .SelX(SelX), 
                    .SelY(SelY), 
                    .SelP(SelP),

                    .SelS(SelS) 
                );

////////////////////////////////////////////////////////////////////////////////
//
//  A - Accumulator
//

assign WE_A = SelA & Valid & Rdy;

always @(posedge Clk)
begin
    if(Rst)
        A <= #1 0;
    else if(WE_A)
        A <= #1 Out;
end

////////////////////////////////////////////////////////////////////////////////
//
//  X - Pre-Index Register
//

assign WE_X = SelX & Valid & Rdy;

always @(posedge Clk)
begin
    if(Rst)
        X <= #1 0;
    else if(WE_X)
        X <= #1 Out;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Y - Post-Index Register
//

assign WE_Y = SelY & Valid & Rdy;

always @(posedge Clk)
begin
    if(Rst)
        Y <= #1 0;
    else if(WE_Y)
        Y <= #1 Out;
end

////////////////////////////////////////////////////////////////////////////////
//
//  P - Processor Status Word: {N, V, 1, B, D, I, Z, C}
//

M65C02_PSWv2    PSW (
                    .Clk(Clk),
                    
                    .SO(SO), 
                    .Clr_SO(Clr_SO), 

                    .SelP(SelP), 
                    .Valid(Valid), 
                    .Rdy(Rdy),
                    
                    .ISR(ISR), 

                    .CCSel(CCSel[2:0]), 

                    .M(M[7:6]), 
                    .OV(OV),
                    .LU_Z(LU_Z), 
                    .COut(COut), 
                    .DO(DO),

                    .P(P),
                    
                    .N(N),
                    .V(V),
                    .D(D),
                    .Z(Z),
                    .C(C)
                );

endmodule
