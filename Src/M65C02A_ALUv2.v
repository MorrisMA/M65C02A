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
// Create Date:     07/04/2015 
// Design Name:     Enhanced Microprogrammed 6502-Compatible Soft-Core Processor
// Module Name:     M65C02A_ALUv2.v 
// Project Name:    C:\XProjects\ISE10.1i\M65C02A 
// Target Devices:  Generic SRAM-based FPGA 
// Tool versions:   Xilinx ISE10.1i SP3
// 
// Description:
//
//  This module provides a wrapper for the normal M65C02A ALU module. The func-
//  tion of the wrapper is to modify the ALU control fields to support the MOV
//  instruction.
//
// Dependencies:    M65C02A_ALU.v
//                      M65C02A_LST.v,      // Load/Store/Transfer Multiplexer
//                      M65C02A_LU.v,       // Logic Unit
//                      M65C02A_SU.v,       // Shift Unit
//                      M65C02A_Add.v,      // Adder Unit
//                      M65C02A_WrSel.v     // Write Select ROM
//                      M65C02A_RegStk.v    // Register Stack for A and Y
//                      M65C02A_RegStkV2.v  // Register Stack for X w/ StkPtr 
//                      M65C02A_PSW.v       // Processor Status Word (Register)
//
// Revision:
//
//  1.00    15G04   MAM     Initial release.
//
//  1.10    15Gxx   MAM     Modified the condition code output so the IND is not
//                          required to terminate the MOV instruction after each
//                          transfer. Instead, if the MSB of the mode byte is
//                          set, the CC output of the ALU is forced to a state
//                          that causes the microprogram to complete the MOV
//                          instruction and fetch the following instruction. 
//                          This allows the following instruction to be an
//                          interruptable instruction so that the instruction
//                          sequence loop formed with this MOV instruction will
//                          move a byte with the same timing as the MVN/MVP 
//                          instructions of the 65C816 microprocessor: 7 cycles.
//
//  1.20    15J25   MAM     Added the IND prefix flag to the ALU module to pro-
//                          vide support for byte swapping and nibble rotation
//                          in Accumulator register stack TOS.
//
//  1.30    15L10   MAM     Added ADJ input port to support ADJ #imm16 instruc-
//                          tion, which allows direct adjustment of the system
//                          or auxiliary stack pointers.
//
//  1.40    16D10   MAM     Added support for arithmetic right shift, arithmetic
//                          left shift, and enabled carry for INC/DEC A (when
//                          preceded by IND prefix).
//
// Additional Comments:
//
//  This module provides a means for controlling the M65C02A's ALU module to
//  provide the functionality required by the MOV sm,dm instruction. The MOV
//  instruction is defined to provide independent operations of the source and
//  destination address registers which are provided by the TOS elements of the
//  X and Y register stacks, respectively. The transfer count is provided by the
//  TOS of register of the A register stack. These three registers are expected
//  to be initialized to 16-bit values. The operating mode of the instruction is
//  provided by an 8-bit operand which follows the MOV op-code. The MOV sm,dm
//  instruction is represented by an opcode, 0x54, followed by a single imme-
//  diate operand byte that encodes the source and destination modes. The source
//  and destination pointers, X and Y, respectively, may be held, incremented,
//  or decremented. The MSB of the mode byte determines if the MOV instruction
//  is a single byte move (1) or a block move of N bytes (0).
//
//  The count register is decremented and sets the NZ flags in the ALU's PSW. If
//  the MSB of the mode byte is set, then the MOV instruction performs only one
//  transfer cycle. This behavior is defined in order to allow a conditional
//  branch instruction to be used to continue the transfers until the count
//  register is zero. This implementation choice allows a block move operation
//  to be interruptable without requiring the MOV microsequence to be interrup-
//  table, which allows considerable savings in the complexity and logic 
//  required to achieve interruptable block move operations at the programmer's
//  discretion.
//
//  The mode operand of the MOV instruction is held in the OP2 register. The
//  source mode bits, OP2[1:0], map directly to the definitions of the Stk_Op
//  control field used to adjust the auxiliary stack of the X register stack.
//  That is, source pointer hold maps to Stk_Op == NOP; source pointer increment
//  maps to Stk_Op == POP; and source pointer decrement maps to Stk_Op == PSH.
//  The destination mode bits, OP2[3:2], map directly to the definitions of the
//  Stk_Op control field used to adjust the Y Top-Of-Stack register. That is,
//  destination pointer hold maps to Stk_Op == NOP; destination pointer incre-
//  ment maps to Stk_Op == POP; and destination pointer decrement maps to
//  Stk_Op == PSH. OP2[7] holds the single cycle control bit.
//
//  The count register, the A top-of-stack register, is decremented by injecting
//  into the ALU control fields the control patterns associated with a DEC A
//  instruction. During the decrement of the count register, the source memory
//  location is read and loaded into OP1. Following that read cycle, a write
//  cycle stores the data held in OP1 to the memory location indicated by the 
//  destination pointer (Y).
//
//  During the read operation, the count register decrement operation also sets
//  the N and Z ALU flags. During the write operation, the microsequence tests
//  the state of the Z flag, and if not set, then the cycle is repeated until
//  the count register is zero. During the write cycle, the source and destina-
//  tion pointers, X and Y, respectively, are modified as indicated by the
//  appropriate mode bits held in OP2. If OP2[7] is set, the CC status is forced
//  to 0 so that the instruction terminates after a single transfer cycle.
//
//  The MOV instruction requires a minimum of four cycles. One cycle is required
//  to fetch and decode the instruction opcode. Another cycle is required to
//  load the instruction mode operand into OP2. One cycle is required to read
//  the source into OP1, and another is needed to store OP1 to the destination.
//  If the block version of the MOV instruction is used, the last two cycles are
//  repeated until the count register is decremented to zero (0).
//
//  The uMCntl field is used to control the MOV instruction operation. uMCntl[2]
//  enables the destination pointer adjustment; uMCntl[1] enables the source
//  pointer adjustment; and uMCntl[0] enables the ALU to decrement the count
//  register, A. With uMCntl[2] or uMCntl[1] a logic 0, the Stk_OpY and Stk_OpX
//  controls are set to 2'b00, or no operation. With uMCntl[0] == 1'b1, the
//  ALU controls are driven in such a manner as to generate a DEC A operation.
//  With uMCntl[0] == 1'b0, the ALU controls are driven to output OP1 on the ALU
//  output data bus.
//
////////////////////////////////////////////////////////////////////////////////

module M65C02A_ALUv2 (
    input   Rst,            // System Reset - synchronous reset 
    input   Clk,            // System Clock
    
    input   Rdy,            // Ready
    
    input   En,             // Enable - ALU functions
    input   Reg_WE,         // Register Write Enable 
    input   ISR,            // Asserted on entry to Interrupt Service Routine
    
    input   SO,             // Set Overflow status bit Command
    output  Clr_SO,         // Acknowledge for SO Command
    
    input   COP_SO,         // Co-Processor Set oVerflow Strobe Input
                    
    //  MOV Instruction Control Interface

    input   MOV,            // MOV instruction
    input   [2:0] uMCntl,   // MOV instruction microprogram control signals
    input   [1:0] SrcMode,  // MOV instruction source pointer mode control
    input   [1:0] DstMode,  // MOV instruction destination pointer mode control

    //  Instruction Prefix Inputs and Supporting Controls
    
    input   IND,            // Override CC during MOV instruction
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
    input   [1:0] Stk_Op,   // Stack Pointer Operation: NOP, POP, PUSH
    
    input   ADJ,            // Stack Pointer Adjust Instruction Input
    input   VEN,            // Enable true Arithmetic Left Shift Operation
    input   CEN,            // Enable C for INC/DEC A Instructions
    
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

    output  CC_Out,         // Condition Code Test Output
    
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

//  FU_Sel - Functional Unit Select

localparam pLST  = 5'd16;   // Load/Store/Transfer Functional Unit
localparam pIDC  = 5'd01;   // Increment/Decrement/Compare Functional Unit

//  Op    - Functional Unit Operation Select

localparam pINC  = 2'd0;    // Increment - Add
localparam pDEC  = 2'd1;    // Decrement - Subtract

//  QSel  - Left Operand Select

localparam pQ_M  = 2'd0;    // Q <= M
localparam pQ_A  = 2'd3;    // Q <= A

//  RSel  - Right Operand Select

localparam pR_M  = 2'd0;    // R <= M
localparam pR_K  = 2'd1;    // R <= K - from Opcode field of IDEC

//  CSel  - Carry In Operand Select

localparam pCi_C = 2'd0;    // Ci <= C
localparam pCi_0 = 2'd1;    // Ci <= 0

//  WSel  - (Register) Write Select (Controls PSAYX Register Write Enables)

localparam pWS_0 = 3'd0;    // M <= ALU_DO (no internal registers written)
localparam pWS_A = 3'd3;    // A <= ALU_DO

//  OSel  - (Register) Output Select (Controls LST Multiplexer)

localparam pOS_A = 3'd3;    // ALU_DO <= A
localparam pOS_M = 3'd7;    // ALU_DO <= M ({OP2, OP1})

//  CCSel - Condition Code Select

localparam pNE   = 4'd10;   // Set CC_Out if Not Equal to Zero  (Z Clear)
localparam pNZ   = 4'd5;    // Set N and Z flags from ALU Output
localparam pNZC  = 4'd6;    // Set N, Z, and C flags during INC/DEC A Opds
localparam pNVZC = 4'd7;    // Set N, V, Z, and C flags during 16-bit IDC Ops

////////////////////////////////////////////////////////////////////////////////
//
//  Declarations
//

wire    CMP;

wire    iSIZ;

wire    [7:0] iK;

wire    [4:0] iFU_Sel;
wire    [1:0] iOp;
wire    [1:0] iQSel, iRSel, iCSel;
wire    [2:0] iWSel, iOSel;
wire    [3:0] iCCSel;

wire    [1:0] Stk_OpX, Stk_OpY;

wire    iCC_Out;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

assign CMP = Op[1] & FU_Sel[0];

assign Stk_OpY = ((MOV) ? ((uMCntl[2]) ? DstMode : 0)
                        :                          0 );
assign Stk_OpX = ((MOV) ? ((uMCntl[1]) ? SrcMode : 0) 
                        : ((OSX)       ? Stk_Op  : 0));

assign iSIZ    = ((MOV) ? 1 : SIZ);
assign iK      = ((MOV) ? 0 : K  );

assign iFU_Sel = ((MOV) ? ((uMCntl[0])       ? pIDC  : pLST)  : FU_Sel        );
assign iOp     = ((MOV) ? ((uMCntl[0])       ? pDEC  : pINC)  : Op            );
assign iQSel   = ((MOV) ? ((uMCntl[0])       ? pQ_A  : pQ_M)  : QSel          );
assign iRSel   = ((MOV) ? ((uMCntl[0])       ? pR_K  : pR_M)  : RSel          );
assign iCSel   = ((MOV) ? ((uMCntl[0])       ? pCi_0 : pCi_C) : CSel          );
assign iWSel   = ((MOV) ? ((uMCntl[0])       ? pWS_A : pWS_0) : WSel          );
assign iOSel   = ((MOV) ? ((uMCntl[0])       ? pOS_A : pOS_M) : OSel          );
assign iCCSel  = ((MOV) ? ((uMCntl[0])       ? pNZ   : pNE                   )
                        : ((SIZ & CMP | VEN) ? pNVZC : ((CEN) ? pNZC : CCSel)));         

M65C02A_ALU    ALU (
                    .Rst(Rst),          // System Reset
                    .Clk(Clk),          // System Clock
                    
                    .Rdy(Rdy),          // Ready
                    
                    .En(En),            // M65C02A ALU Enable Strobe Input
                    .Reg_WE(Reg_WE),    // M65C02A ALU Register Write Enable
                    .ISR(ISR),          // M65C02A ALU Interrupt Svc Rtn Strobe
                    
                    .SO(SO),            // M65C02A ALU Set oVerflow Flag in PSW
                    .Clr_SO(Clr_SO),    // M65C02A ALU Clr SO - Acknowledge
                    
                    .COP_SO(COP_SO),    // M65C02A ALU COP Set oVerflow Strobe
                    
                    .IND(IND),          // M65C02A ALU Alt. Reg. Stack Op. Sel.
                    .SIZ(iSIZ),         // M65C02A ALU Operation Size Override
                    .OAX(OAX),          // M65C02A ALU Register Override
                    .OAY(OAY),          // M65C02A ALU Register Override
                    .OSX(OSX),          // M65C02A ALU Stack Override
                    
                    .DUP(DUP),          // M65C02A ALU Register Stack DUPlicate
                    .SWP(SWP),          // M65C02A ALU Register Stack SWaP
                    .ROT(ROT),          // M65C02A ALU Register Stack ROTate
                    
                    .SelS(SelS),        // M65C02A ALU Stack Pointer Select
                    .S(S),              // M65C02A ALU Stack Pointer Input

                    .Mod(Mod),          // M65C02A ALU Aux Stack Ptr % 256 Input
                    .Stk_OpX(Stk_OpX),  // M65C02A ALU Aux Stack Ptr Ops
                    .Stk_OpY(Stk_OpY),  // M65C02A ALU MOV Dst Ptr Ops
                    
                    .ADJ(ADJ),          // M65C02A Stack Pointer Adjust Instr.

                    .FU_Sel(iFU_Sel),   // M65C02A ALU Functional Unit Sel
                    .Op(iOp),           // M65C02A ALU Operation Select
                    .QSel(iQSel),       // M65C02A ALU Q Data Mux Select
                    .RSel(iRSel),       // M65C02A ALU R Data Mux Select
                    .CSel(iCSel),       // M65C02A ALU Adder Carry Select
                    .WSel(iWSel),       // M65C02A ALU Register Write Select
                    .OSel(iOSel),       // M65C02A ALU Output Register Select
                    .CCSel(iCCSel),     // M65C02A ALU Condition Code Select
                    
                    .K(iK),             // M65C02A ALU Auxiliary Constant Input
                    .T(T),              // M65C02A ALU FORTH VM Register Input
                    .M(M),              // M65C02A ALU Memory Operand Input

                    .DO(DO),            // M65C02A ALU Data Output Multiplexer
                    .Val(Val),          // M65C02A ALU Output Valid Strobe
                    .CC_Out(iCC_Out),   // M65C02A ALU Condition Code Mux
                    
                    .ALU_C(ALU_C),      // M65C02A ALU Carry Out
                    .ALU_Z(ALU_Z),      // M65C02A ALU Zero Out
                    .ALU_V(ALU_V),      // M65C02A ALU OVerflow
                    .ALU_N(ALU_N),      // M65C02A ALU Negative

                    .X(X),              // M65C02A ALU Index Register
                    .Y(Y),              // M65C02A ALU Index Register
                    .A(A),              // M65C02A ALU Accumulator Register

                    .P(P)               // M65C02A Processor Status Word Reg
                );
                
assign CC_Out = ((MOV) ? ((M[15]) ? 0 : ~P[1]) : iCC_Out);

endmodule
