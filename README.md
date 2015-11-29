M65C02A Microprocessor Core
=======================

Copyright (C) 2014-2015, Michael A. Morris <morrisma@mchsi.com>.
All Rights Reserved.

Released under GPL v3.

General Description
-------------------

This project provides a synthesizable enhanced version of the 6502/65C02 
processor core. The M65C02A soft-core processor that implements the 
instruction set architecture (ISA) of the 6502/65C02 microprocessors as 
exemplified by the instruction set of the WDC W65C02S. The M65C02A soft-core 
processor features a completely reworked microprogrammed control structure 
compared to that used in the preceding M65C02 project. The basic logic 
structure of the core has been significantly altered to allow the 
implementation of a significantly enhanced 8/16-bit version of the 6502/65C02 
processors. 

This release supercedes any prior releases and provides the completed version 
of the planned M65C02A soft-core processor. As provided, the M65C02A soft-core 
processor provides the following enhancements to 6502/65C02 processors:

    (1)     M65C02A core allows the 6502/65C02 index registers, X and Y, to be 
used as accumulators. Although the one address, accumulator-based 
architecture of the 6502/65C02 microprocessors is preserved, three on-chip 
accumulators should make it easier for the programmer to keep extended width 
results in on-chip registers rather than loading and storing partial results 
from/to memory;

    (2)     M65C02A core allows the basic registers (A, X, Y, S) to be extended to 
16 bits in width. To maintain compatibility with 6502/65C02 microprocessors, 
the default operation width of the registers and ALU operations is 8 bits. 
Internally, the upper byte of any register (A, X, Y, S) or the memory operand 
register (M) is forced to logic 0 (except for S which is forced to 0x01) 
unless the programmer explicitly extends the width of the operation with a 
prefix instruction;

    (3)     M65C02A core’s ALU registers (A, X, and Y) are implemented using a 
modified, three level push-down register stack. This provides the programmer 
the ability to preserve intermediate results on-chip. The modification to the 
register stack is that load and store instructions only affect the TOS 
locations of the A, X, and Y register stacks. In other words, the TOS location 
of the register stacks is not automatically pushed on loads from memory, nor 
is it automatically popped on stores to memory. Explicit actions are required 
by the programmer to manage the contents of the register stacks associated 
with A, X, and Y;

    (4)     M65C02A core’s X Top-Of-Stack register, XTOS, serves as a base pointer 
for the base-relative addressing modes: bp,B and (bp,B),Y. This addressing mode 
provides the stack frame capability needed by programming languages like C and 
Pascal, and which must be emulated by 6502/65C02 microprocessors. (Note: base-
relative addressing using XTOS is generally associated with the system stack, 
but can be used in a more general way with any data structures in memory.)

    (5)     M65C02A core’s XTOS can function as a third (auxiliary) stack pointer, 
SX, when instructions are prefixed with the osx instruction. (Note: when used 
as the auxiliary stack pointer, S becomes the source/target for all of the 
6502/65C02 instructions specific to the X register: ldx, stx, cpx, txa, tax, 
plx, phx. This feature provides seven more ways to affect the system stack 
pointer: lds, sts, cps, tsa, tas, pls, phs.)

    (6)     The M65C02A core provides support for kernel and user modes. The 
previously unused and unimplemented bit of the processor status word (P), bit 
5, is used to indicate the processor mode, M. The M65C02A core provides kernel 
mode and a user mode stack pointers, SK and SU, respectively, for this 
purpose. SU may be manipulated from kernel mode routines, but SK is 
inaccessible to user mode routines. (Note: a 6502/65C02 program will stay in 
the kernel mode unless bit 5 (kernel mode) of the PSW on the system stack is 
cleared when an rti instruction is performed. On reset, the M65C02A defaults 
to kernel mode for compatibility with 6502/65C02 microprocessors.)

    (7)     M65C02A core provides automatic support for stacks greater than 256 
bytes. This feature is automatically activated whenever stacks are allocated 
in memory outside of memory page 0 (0x0000-0x00FF) or memory page 1 (0x0100-
0x01FF). (Note: a limitation of this feature is that if the stack grows into 
page 1, then the mod 256 behavior of normal 6502/65C02 stacks will be 
automatically restored.)

    (8)     M65C02A core provides a prefix instruction, ind or isz, to add 
indirection to an addressing mode. When an indirection prefix instruction is 
applied, indirection is performed before indexing. (Note: a consequence of 
this rule is that the indexed zero page direct addressing modes, zp,X and 
zp,Y, are converted to post-indexed indirect addressing modes: (zp),X and 
(zp),Y. Similar behavior applies to the indexed absolute addressing modes. 
When ind or isz is applied to an indirect addressing mode, the result is a 
double indirection as expected. However, the rule that indirection is applied 
before indexing still applies. The result is that pre-indexed indirect 
addressing modes, (zp,X) and (abs,X), translate into post-indexed double 
indirect addressing modes, ((zp)),X and ((abs)),X, instead of into pre-indexed 
double indirect addressing modes, ((zp,X)) and ((abs,X)).)

    (9)     M65C02A core provides a prefix instruction, siz or isz, which promotes 
the width of the ALU operation from 8 to 16 bits. The only restriction is that 
BCD operations cannot be promoted from 8-bit to 16-bit; BCD arithmetic is only 
available for 8-bit operands.

    (10)    M65C02A core allows the CMP/CPX/CPY instructions to set the V flag 
when a 16-bit operation is being performed. The M65C02A core also implement 
multi-flag conditional branches. The multi-flag conditional branches support 
four signed conditional branches: Than (LT), Less Than or Equal (LE), Greater 
Than (GT), and Greater Than or Equal (GE). In addition, four unsigned 
conditional branches are supported: lower (LO), lower or same (LOS), higher 
(HI), and Higher or Same (HOS).

    (11)    M65C02A core provides support for the implementation of virtual 
machines (VMs) for threaded interpreter’s such FORTH. The M65C02A core’s IP 
and W are 16 bit registers which support the implementation of DTC/ITC FORTH 
VMs using several dedicated M65C02A instructions. The core's microprogram 
implements the DTC FORTH inner interpreter using a with an instruction, and it 
also implements the ENTER/DOCOLON operation with an instruction. The ITC FORTH 
version of these operations are supported using the ind prefix instruction. 
Instructions for pushing, popping, and incrementing IP and W are also 
included. Further, a new addressing mode, IP-relative with auto-increment, is 
applied to the LDA, ADC, and STA instructions. The LDA ip,I++ and STA ip,I++ 
instructions allow faster access to literals, constants, and variables. The 
ADC ip,I++ instruction allows the synthesis of IP-relative branches and jumps. 
Finally, the IP can be transferred to/from, or exchanged with, the A.

(12)    M65C02 core provides a multi-purpose move byte instruction. The 
instruction has two operating modes: single cycle, and multi-cycle. In the 
single cycle mode, the instruction terminates after each move is completed. 
The count (A), source pointer (X), and destination pointer (Y) registers are 
updated before the instruction terminates. Decrementing the count register (A) 
sets the ALU ZC flags like a DEC A (DEA) instruction. This allows the 
programmer the option to loop back and continue the execution of the single 
cycle move instruction. The multi-cycle transfers the entire block before 
terminating. Because of this behavior, the multi-cycle move instruction is not 
interruptable. The source and destination pointers may be independently 
configured to increment, decrement, or hold. This allows the M65C02A move 
instruction to support a wide range of data transfer tasks.

(13)    M65C02A core provides support for implementing application-specific 
coprocessors. Direct support for application-specific coprocessors allows an 
implementation based on the M65C02A core to be easily extended in a domain-
specific manner.

To demonstrate the use of the M65C02A core, an example of its use to implement 
a microcomputer is provided as part of the release. The example microcomputer 
constructed using the M65C02A core provides the following features:

    •	M65C02A core (synthesizable, enhanced 6502/65C02-compatible core)
    •	a Multi-Source (16) Interrupt Handler                            
    •	a Memory Management Unit (with support for Kernel and User modes)
    •	28kB of memory (built from synchronous Block RAM)                
    •	2 Universal Asynchronous Receiver/Transmitter (UARTs)            
    •	1 Synchronous Peripheral Interface (SPI)                         

Implementation
--------------

The implementation of the current core provided consists of the following 
Verilog source files and several memory initialization files:

    M65C02A.v                           (M65C02A Microcomputer: RAM/ROM/IO)
        M65C02A_Core.v                  (M65C02A Processor Core)
            M65C02A_MPC.v               (M65C02A Microprogram Controller)
                M65C02A_uPgm_ROM.coe    (M65C02A_uPgm_ROM.txt)
                M65C02A_IDecode_ROM.coe (M65C02A_IDecode_ROM.txt)
            M65C02A_AddrGen.v           (M65C02A Address Generator)
                M65C02A_StkPtrV2.v      (M65C02A Dual Stack Pointer)
            M65C02A_ALUv2.v             (M65C02A ALU Wrapper)
                M65C02A_ALU.v           (M65C02A ALU)
                    M65C02A_LST.v       (M65C02A ALU Load/Store/Xfr Mux)
                    M65C02A_LU.v        (M65C02A ALU Logic Unit) 
                    M65C02A_SU.v        (M65C02A ALU Shift/Rotate Unit)
                    M65C02A_Add.v       (M65C02A ALU Dual Mode Adder Unit) 
                    M65C02A_WrSel.v     (M65C02A ALU Register Write Decoder)
                    M65C02A_RegStk.v    (M65C02A ALU Register Stack)
                    M65C02A_RegStkV2.v  (M65C02A ALU Reg. Stack w/ Stk Ptr)
                    M65C02A_PSWv2.v     (M65C02A ALU Processor Status Word)
        M65C02A_IntHndlr.v              (Interrupt Handler)
        M65C02A_MMU.v                   (Memory Management Unit)
        M65C02A_SPIxIF.v                (SPI Master I/F)
            DPSFmnCE.v                  (Transmit & Receive FIFOs)
            SPIxIF.v                    (Configurable SPI Master)
                fedet.v                 (falling edge detector)
                redet.v                 (rising edge detector)
        UART.v                          (COM0/COM1 Asynch. Serial Ports)
            DPSFmnCS.v                  (Transmit & Receive FIFOs)
            UART_BRG.v                  (UART Baud Rate Generator)
            UART_TXSM.v                 (UART Transmit SM/Shifter)
            UART_RXSM.v                 (UART Receive SM/Shifter)
            UART_INT.v                  (UART Interrupt Generator)
                fedet.v                 (falling edge detector)
                redet.v                 (rising edge detector)

    M65C02A.ucf             - User Constraints File: period and pin LOCs
    M65C02A.bmm             - Block RAM Memory Definition File
    M65C02A.tcl             - Project settings file
    
    tb_M65C02A.v            - Completed M65C02A soft-microcomputer testbench

M65C02A-Specific Instructions
--------------

The M65C02A soft-core processor is fully compatible with all 6502/65C02 
instructions and addressing modes. Regression testing using Klaus Dormann's 
functional test suite executes without error. Unlike the mode switching used 
by the 65816, the M65C02A uses prefix instructions. This decision can be good 
or bad. It is good in that 6502/65C02 functions can be interspersed without 
any concerns. It is bad in that it requires the programmer must explicitly 
specify the address indirection, register overrides, and operation size. In 
most cases, the additional instruction length required to specify the various 
prefix instructions does not result in poorer performance, and it generally 
does not increase program size.

This section will describe, in cursory detail, the various instructions that 
are specific to the M65C02 soft-core processor. Only enough detail will be 
provided to convey the character and behavior of the M65C02A-specific 
instructions. Additional detail can be gleaned from the Verilog source code, 
the microprogram listings, and the User Guide.

#Prefix instructions

The M65C02A gains much of its power from six (6) prefix instructions: IND, 
SIZ, ISZ, OAX, OAY, and OSX. The IND and SIZ instructions add indirection and 
promote ALU operations to 16 bits, respectively. The ISZ prefix instruction 
applies IND and SIZ simultaneously. These three prefix instructions can also 
be applied to implicit addressing mode instructions in order to increase the 
opcode space. In general, however, IND, SIZ, and ISZ are intended to be 
applied to logic, shift/rotate, and arithmetic instructions. IND converts a 
direct addressing mode to an indirect addressing mode. When applied to an 
indirect addressing mode, IND adds a second level of indirection.

Indirection added by IND is always performed before indexing. This rule has 
some consequences. For example, Pre-indexed Zero Page Direct [zp,X] is 
converted to Post-Indexed Zero Page Indirect [(zp),X] when IND is prefixed to 
an instruction using the Pre-Indexed Zero Page Direct addressing mode. When 
IND is applied to an instruction using the Pre-Indexed Zero Page Indirect 
[(zp,X)] addressing mode, the result is Post-Indexed Zero Page Double Indirect 
[((zp)),X] not Pre-Indexed Zero Page Double Indirect [((zp,X))].

When SIZ is applied to an instruction, the operation is doubled in size. While 
operating on 8-bit quantities, the upper half of registers and/or ALU results 
is forced to zero. This has the consequence that when mixing 8-bit and 16-bit 
operations, the 8-bit registers/values will appear to be unsigned to the 16-
bit registers/values. There are only two limitations regarding promotion of 
ALU operations to 16-bit widths: (1) BCD arithmetic is only valid for 8-bit 
quantities, and (2) The Rockwell instructions cannot be promoted to 16 bits.

ISZ can be used whenever indirection and a 16-bit ALU operation are desired. 
IND can be used to improve the utility of instructions such as BIT/TRB/TSB by 
adding indirection and increasing the width of the operation from 8 to 16 
bits. Only IND and ISZ were combined into a single prefix instruction. The 
other three prefix instructions may be applied in combination with IND, SIZ, 
and ISZ.

The OAX and OAY prefix instructions override the sourc/target registers. 
OAX allows X to function as an accumulator using any instruction which has A 
as a source/destination register. For its part, A takes on the pre-index 
register role of A in the addressing modes of the instructions prefixed by 
OAX. The OAY prefix instruction produces the same results with the Y and A 
registers.

Ths OSX prefix instruction allows the programmer to override the default stack 
pointer of an instruction with X. The X register has the capability of 
functioning as a third, auxiliary stack pointer. The system stack pointer is 
not converted to function as the pre-index register. Instead, S is substituted 
for X in any instruction specific to X, i.e. LDX/STX/CPX, TAX/TXA, and 
PHX/PLX. This allows S to be more easily manipulated. Note that the PHX/PLX 
will use the auxiliary stack whose stack pointer is X.

When multiple prefix instructions are necessary, OSX and OAX are mutually 
exclusive, and so are OAY and OAX. Internal flags record the execution of the 
six prefix instructions. Excution of a mutually exclusive prefix instruction 
will result in the flag register of the previously set mutually exclusive 
instruction being reset. Furthermore, the flag registers for prefix 
instructions are sticky, and remain set until the completion of a non-prefix 
instruction, i.e. the completiong of the instruction which is the target of 
one or more prefix instructions. Finally, no protection is provided against an 
infinite long series of prefix instructions, all of which are uninterruptable, 
or application of prefix instructions to create an existing addressing mode.

##Access to User Stack Pointer (SU) from Kernel Mode

When in Kernel mode, access to SU is provided by applying the IND or ISZ to 
the instruction sequences that access the system stack pointer:

[SIZ] TSX           =>  TSX : X  <= SK;     IND/ISZ TSX =>  TSX : X  <= SU
[SIZ] TXS           =>  TXS : SK <= X;      IND/ISZ TXS =>  TXS : SU <= X
    
[SIZ] OSX/OAX TXA   =>  TSA : A  <= SK;     IND/ISZ TSA =>  TSA : A  <= SU
[SIZ] OSX/OAX TAX   =>  TAS : SK <= A;      IND/ISZ TAS =>  TAS : SU <= A

[SIZ] OSX OAY TXA   =>  TSY : Y  <= SK;     IND/ISZ TSY =>  TSY : Y  <= SU
[SIZ] OSX OAY TAX   =>  TYS : SK <= Y;      IND/ISZ TYS =>  TYS : SU <= Y
    
The fastest way to access either SK or SU is to use the standard TXS/TSX 
instructions. Use only SIZ to access the 16-bit SK or use only ISZ to access 
the 16-bit SU. Use either OAX or OSX in order to transfer SK or SU to/from A. 
Use OSX and OAY to transfer SK or SU to/from Y.

The M65C02A implements stacks using mod 256 behavior when they are located in 
page 0 or page 1, and mod 65536 behavior when not located in these two pages. 
Therefore, it is recommended that SIZ or ISZ is used when reading or writing 
the system stack pointers in order to transfer the upper byte from/to the 
stack pointers. This will preserve or set the behavior of the stacks.

Without using IND or ISZ, these instruction sequences will access the system 
stack pointer, SK or SU, based on the state of the M bit in P. Applying only 
IND will transfer only the lower 8-bits of SU. Adding SIZ by using ISZ will 
transfer the complete 16-bit value of SU; if only SIZ is used, then only SK 
will be accessed.

#Register Stack Manipulation instructions

A unique feature of the M65C02A soft-core processor are the three level push-
down stacks used to implement each of the three primary registers. Load and 
store operations do not automatically perform push and pop operations. This 
behavior allows the M65C02A soft-core to seamlessly emulate the 6502/65C02 
processors.

All three register stacks are implemented with 3 16-bit registers. Deeper 
stacks are possible, but a three deep stack strikes a good balance in utility 
and complexity. The three register stacks are supported by three single cycle 
instructions: DUP, SWP, and ROT. The operations of these three instructions 
are described by the following equations:

    DUP :   {TOS, NOS, BOS} <= {TOS, TOS, NOS};
    SWP :   {TOS, NOS, BOS} <= {NOS, TOS, BOS};
    ROT :   {TOS, NOS, BOS} <= {NOS, BOS, TOS};
    
To implement a stack push, the programmer must push the current TOS value 
prior to loading a new value by duplicating the TOS:

    DUP
    LDA abs
    
To implement a stack pop, the programmer must pop the TOS value after a store 
to memory by rotating the stack:

    STA (zp)
    ROT 
    
Although this preserves the TOS in BOS, it provides the needed popping of the 
register stack.

##Special Behavior of the A Register Stack

In addition to the operations discussed above, the A register stack provides 
several special behaviors. Using the IND prefix instruction, the bytes of the 
A TOS register can be swapped:

    IND SWP :   {{TOS[7:0], TOS[15:8]}, NOS, BOS}
    
Using the IND prefix instruction, the nibbles of the A TOS register can be 
rotated right:

    IND ROT :   {{TOS[3:0], TOS[15:4]}, NOS, BOS}
    
Using the IND prefix instruction, the A TOS register can be written into the 
FORTH VM IP register:

    IND DUP :   IP <= ATOS;             => TAI
    
Using the SIZ prefix instruction the FORTH VM IP register can be written into 
the A TOS register:

    SIZ DUP :   ATOS <= IP;             => TIA
    
Finally, using the ISZ prefix instruction the A TOS register and the FORTH VM 
IP register can be exchanged:

    ISZ DUP :   ATOS <= IP; IP <= ATOS; => XAI
    
##Special Behavior of the X Register Stack

In addition to the behaviors discussed above, the X TOS register implements a 
counter function that allows it to operate as a third stack pointer. Further, 
the same logic used to implement 6502/65C02 stacks is used to support the 
M65C02A move byte instruction. When a push is performed, the X TOS register is 
decremented if X is the default stack pointer or when the OSX flag is set and 
X is not the default stack pointer. When a pop is performed, the X TOS 
register is incremented if X is the default stack pointer or when the OSX flag 
is set and X is not the default stack pointer. The M65C02A move byte 
instruction uses this functionality to implement the source pointer increment, 
decrement, or hold operations.

##Special Behavior of the Y Register Stack

In addition to the behaviors discussed above, the Y TOS register implements a 
counter function that allows it to operate as the destination pointer for the 
M65C02A move byte instruction. The Y TOS register uses the same counter 
implementation as the X TOS register. The M65C02A move byte instruction uses 
this functionality to implement the destination pointer increment, decrement, 
or hold operations.

#Base-Pointer Relative Addressing Mode Instructions

The M65C02A introduces the base-pointer relative [bp,B] and post-indexed base-
pointer relative indirect [(bp,B),Y] addressing modes. Unlike the stack-
pointer relative addressing modes of the W65C816 microprocessor, the M65C02A 
base-pointer relative addressing modes is designed to support the stack frames 
such as those used by programming languages such as C and Pascal. 

The base-pointer (B) is X, and the offset included in the instruction, bp, is 
zero based; this makes the value on the top of the stack offset 0. Further, 
offset bp is signed so that positive addresses point to parameters of a 
function, and negative offsets point to local variables. On entry into the 
function, the current base pointer, X, is pushed, and then the stack pointer 
to X to mark the stack frame; offset 0 is the previous base pointer, offset 2 
is the return address, and offset 4 and above are the parameters passed into 
function.

There are 17 M65C02A instructions with this addressing mode. There are eight 
standard base-pointer relative instructions:

    ORA/ANL/EOR/ADC/STA/LDA/CMP/SBC bp,B

There are eight standard post-indexed base-pointer relative indirect 
instructions:

    ORA/ANL/EOR/ADC/STA/LDA/CMP/SBC (bp,B),Y
    
These instructions support the IND, SIZ, ISZ, OAX and OAY prefix instructions 
described above. IND, SIZ, and ISZ have the expected effects. When using OAX, 
A becomes the base pointer, and X is the accumulator. When using OAY, A 
becomes the post-index register, and Y is the accumulator. Since this 
addressing mode is not dependent on a stack pointer, the OSX prefix 
instruction has no effect. The registers of the X register stack, especially 
TOS and NOS, can be used to set up and maintain base pointers into the system 
stack and into any other memory structure such as the auxiliary stack, also in 
X.

The last M65C02A instruction supporting this addressing mode is a jump 
instruction:

    JMP (bp,B),Y

This instruction supports the IND, OAX, and OAY prefix instructions. IND adds 
a second level of indirection, OAX allows A to function as the base-pointer, 
and OAY allows A to be used as the post-index register:

    IND JMP (bp,B),Y        =>  JMP ((bp,B)),Y

    OAX JMP (bp,B),Y        =>  JMP (bp,A),Y 
    OAX IND JMP (bp,B),Y    =>  JMP ((bp,A)),Y

    OAY JMP (bp,B),Y        =>  JMP (bp,B),A 
    OAY IND JMP (bp,B),Y    =>  JMP ((bp,B)),A
    
Recall that OAX and OAY are mutually exclusive. This constraint avoids the 
non-sensical situation where OAX and OAY are both applied which would result 
in A being simultaneously used as the base-pointer and the post-index 
register.

#Extended (16-bit) Push/Pop Instructions

The M65C02A provides four push instructions and two pop/pull instructions not 
found on the 6502/65C02 processors. These instructions push/pull 16-bit values 
to/from the system stack:

    PHR rel16           ; PusH word Relative

    PSH #imm16          ; PuSH word Immediate

    PSH zp              ; PuSH word Zero Page Direct
    PSH abs             ; PuSH word Absolute

    PUL zp              ; PULl word Zero Page Direct             
    PUL abs             ; PULl word Absolute
    
The PHR instruction resolves the absolute address of the 16-bit relative 
displacement, and pushes that 16-bit value onto the system stack. The rel16 
parameter is the distance from the address of the next instruction to the 
target, i.e. relative to the PC. The stack used (SK/SU or SX) by this 
instruction can be changed by adding the OSX prefix instruction. Other prefix 
instructions have no affect on this instruction.

The PHR rel16 instruction can be used to synthesize a BSR rel16 instruction 
using an instruction sequence like the following:

        PSH #$1
        PHR rel16
    $1: RTS

Another use of the PHR rel16 is to resolve, in a PC-relative manner, the 
address of constant or variable needed at run time, when the program can be 
relocated when it is loaded in memory. Using a base-relative LDA/STA 
instruction prefixed with IND, the constant/variable can be accessed using the 
pointer pushed onto the stack. Using the post-indexed base relative indirect 
version of the same instructions, the constant/variable can be accessed 
through the resolved pointer with indexing. Either technique can be used to 
access objects in a position-independent manner using PHR to resolve the 
address of the object.

The PSH #imm16 instruction simply pushes the 16-bit immediate constant 
following the instruction onto the system stack. The stack used (SK/SU or SX) 
by this instruction can be changed by adding the OSX prefix instruction. Other 
prefix instructions have no affect on this instruction.

The last four extended stack operations support the zero page direct and 
absolute addressing modes to write/read 16-bit values to/from memory. They 
support the IND and the OSX prefix instructions with the expected results to 
the addressing mode (IND) and default stack pointer (OSX). Other prefix 
instructions have no affect on these instructions.

#FORTH VM Support

The 6502/65C02 processors have long supported FORTH. The FORTH inner 
interpreter can be implemented using the native instruction set. The 65C02-
specific instructions and addressing modes can be used to implement a FORTH 
interpreter slightly faster than an FORTH interpreter which only uses 6502 
instructions and addressing modes.

When developing the instruction set for the M65C02A soft-core processor, it 
was decided that it would include instructions to support a FORTH VM. An 
analysis driven by a review of a 6502-compatible fig-FORTH implementation, 
"Threaded Interpretive Languages" by R. G. Loeliger, research by Dr. Phillip 
Koopman, and the "Moving FORTH" articles written by Dr. Brad Rodriguez 
(developer and maintainer of Camel FORTH), it was decided that the M65C02A 
would directly implement (as recommended by Jeff Laughton) the FORTH VM using 
internal 16-bit registers for the Interpretive Pointer (IP) and the Working 
register (W). Further, it was decided to dedicate several instructions to 
directly support the implementation of the FORTH VM:

    NXT         ; NEXT
    ENT         ; ENTER/CALL/DOCOLON using Return Stack (RS)
    PHI         ; Push IP on RS
    PLI         ; Pull IP from RS
    INI         ; Increment IP
    LDA ip,I++  ; Load byte into A using IP-relative with autoincrement address
    ADC ip,I++  ; Add byte into A using IP-relative with autoincrement address
    STA ip,I++  ; Store byte in A at IP-relative with autoincrement address
    
NXT and ENT implement the NEXT and ENTER/CALL/DOCOLON functionality needed to 
implement a Direct Threaded Code (DTC) FORTH VM. If these instructions are 
prefixed by IND, then an Indirect Threaded Code (ITC) FORTH VM is the result.

The FORTH VM supported by the M65C02A will provide the following mapping of 
the various FORTH VM registers (described by Brad Rodriguez):

    IP  - Internal dedicated 16-bit register
    W   - Internal dedicated 16-bit register
    PSP - System Stack Pointer (S), allocated in memory (page 1 an option)
    RSP - Auxiliary Stack Pointer (X), allocated in memory (page 0 an option)
    UP  - Memory (page 0 an option)
    X   - Not needed, {OP2, OP1} or MAR can provide temporary storage required

The following pseudo code defines the operations for these three operations in 
terms of the ITC and the DTC models:

                   ITC                                   DTC
    ================================================================================
    NEXT:   W      <= (IP++) -- Ld *Code_Fld    ; W      <= (IP++) -- Ld *Code_Fld
            PC     <= (W)    -- Jump Indirect   ; PC     <= W      -- Jump Direct
    ================================================================================
    ENTER: (RSP--) <= IP     -- Push IP on RS   ;(RSP--) <= IP     -- Push IP on RS
            IP     <= W + 2  -- => Param_Fld    ; IP     <= W + 2  -- => Param_Fld
    ;NEXT
            W      <= (IP++) -- Ld *Code_Fld    ; W      <= (IP++) -- Ld *Code_Fld
            PC     <= (W)    -- Jump Indirect   ; PC     <= W      -- Jump Direct
    ================================================================================
    EXIT:
            IP     <= (++RSP) -- Pop IP frm RS  ; IP     <= (++RSP)-- Pop IP frm RS
    ;NEXT
            W      <= (IP++) -- Ld *Code_Fld    ; W      <= (IP++) -- Ld *Code_Fld
            PC     <= (W)    -- Jump Indirect   ; PC     <= W      -- Jump Direct
    ================================================================================
    
ENT, PHI, and PLI all default to the RS, which is set to be implemented by the 
auxiliary stack feature of the X TOS register. If OSX is prefixed to these 
instructions, the PS is used instead. The PS is implemented with the system 
stack pointer (SK or SU) of the M65C02A.

PHI, PLI, and INI operate on the FORTH VM IP register. If prefixed by IND, 
these instructions operate on the FORTH VM W register: PHW, PLW, INW.

The ip,I++ addressing mode is an M65C02A-specific addressing mode. The 
instructions using this addressing mode can be used in a general manner 
independently of a FORTH VM. However, the addressing mode was included in 
order to access constants, literals, and pointers stored in the FORTH 
instruction stream. Like most M65C02A instructions, they default to operating 
on 8-bit values, but they support the IND, SIZ, and ISZ prefix instructions 
with the expected results to the addressing mode and the operation width. 

The LDA ip,I++ instruction will load the byte which follows the current IP 
into the accumulator and advances the IP by 1. If this instruction is prefixed 
by SIZ, then the word following the current IP is loaded into the accumulator 
and the IP is advanced by 2. If prefixed by IND, the instruction becomes LDA 
(ip,I++), which uses the 16-bit word following the current IP as a byte 
pointer. The IP is advanced by 2, and the byte pointed to by the pointer is 
loaded into the accumulator. If prefixed by ISZ, the word following the 
current IP is used as a word pointer, while the IP is advanced by 2, to load a 
word into the accumulator.

The LDA ip,I++ instruction is matched by the STA ip,I++. Without indirection, 
the STA ip,I++ instruction will write directly into the FORTH VM instruction 
stream. With indirection, the STA (ip,I++) instruction can be used for 
directly updating byte/word variables whose pointers are stored in the FORTH 
VM instruction stream. (Although, the ability to create self-modifying FORTH 
programs may be useful when compiling FORTH programs, the STA ip,I++ 
instruction is expected to be prefixed with IND or ISZ under normal usage.) 

Finally, the ADC ip,I++ instruction allows constants (or relative offsets) 
located at the current IP to be added to the accumulator. Like LDA ip,I++ and 
STA ip,I++, the ADC ip,I++ supports the IND, SIZ, and ISZ prefix instructions.

For example, IP-relative conditional FORTH branches can be implemented using 
the following instruction sequence:
            
            [SIZ] Bxx $1    ; [2[3]] test xx condition and branch if not true
            ISZ DUP [A]     ; [2] exchange A and IP (XAI)
            CLC             ; [1] prepare C flag for addition
            SIZ ADC ip,I++  ; [5] add IP-relative offset to A
            ISZ DUP [A]     ; [2] exchange A and IP (XAI)
    $1:

The IP-relative conditional branch instruction sequence only requires 12[13] 
clock cycles, and IP-relative jumps require only 10 clock cycles.

Conditional branches and unconditional jumps to absolute addresses rather than 
relative addresses can also be easily implemented. A conditional branch to an 
absolute address can be implemented as follows:
            
            [SIZ] Bxx $1    ; [2[3]] test xx condition and branch if not true
            SIZ LDA ip,I++  ; [5] load absolute address and autoincrement IP
            IND DUP [A]     ; [2] transfer A to IP (TAI)
    $1:

Thus, a conditional branch to an absolute address requires 9[10] cycles, and 
the unconditional absolute jump only requires 7 clock cycles. Clearly, if the 
position independence of IP-relative branches and jumps is not required, then 
the absolute address branches and jumps provide greater performance.

(Note: the M65C02A supports the eight 6502/65C02 branch instructions which 
perform true/false tests of the four ALU flags. When prefixed by SIZ, the 
eight branch instructions support additional tests of the ALU flags which 
support both signed and unsiged comparisons. The four signed conditional 
branches supported are: less than, less than or equal, greater than, and 
greater than or equal. The four unsigned conditional branches supported are: 
lower than, lower than or same, higher than, and higher than or same. These 
conditional branches are enabled by letting the 16-bit comparison instructions 
set the V flag.)

#PC-relative 16-bit Unconditional Branch 

The M65C02A provides an unconditional branch to a 16-bit PC-relative address:

        BRL rel16
    
This instruction can be used to implement position-indepent code modules. In 
addition, it can be used to synthesize calls to position-independent 
subroutines:

        PSH #($1+2)
    $1: BRL rel16
    
This instruction's behavior is not modified by any of the M65C02A prefix 
instructions.

#Move Byte Instruction

The M65C02A provides a move byte instruction. The instruction includes an 
immediate parameter which specifies whether uninterruptable block moves or 
interruptable single byte moves are performed. In addition, the parameter 
controls whether the source pointer and destination pointers are incremented, 
decremented, or unchanged. Further, each pointer can be independently 
controlled. Two mnemonics are reserved for this instruction, although only one 
opcode is used:

        MVB sm,dm           ; Block Move, MSB of parameter byte is 0
        MVS sm,dm           ; Single Move, MSB of parameter byte is 1

The accumulator is used as the count register. The NZC flags in P are set by 
this instruction so that the MVS mode can be used with a conditional branch to 
test if the transfer is complete or not.

The X and Y registers function as the source and destination pointers, 
respectively. The sm and dm fields are the source and destination pointer 
modes. These fields are defined as: I (increment), D (decrement), or H (hold). 
The source mode is encoded in the bits [1:0] and the destination mode is 
encoded in bits [5:4] of the parameter byte. The encodings are 3 - I, 2 - D, 
and 0 - Hold (unchanged).

