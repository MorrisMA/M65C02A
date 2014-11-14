M65C02A Microprocessor Core
=======================

Copyright (C) 2014, Michael A. Morris <morrisma@mchsi.com>.
All Rights Reserved.

Released under GPL v3.

General Description
-------------------

This project provides a synthesizable microprogrammed IP core that implements 
the instruction set architecture (ISA) of the 6502/65C02 microprocessors. The 
initial release provides the instruction set of the 65C02 plus the WAI and STP
instructions added by WDC in its W65C02S processor. 

The M65C02A core features a completely reworked microprogrammed control 
structure compared to that used in the M65C02 project. In addition, the basic 
logic structure of the core has been significantly altered to reduce the logic 
required to implement the processor. In the process, logic has been added to 
the basic structure that will allow the core to support adding instructions 
and addressing modes. Theses changes to the core's logic will allow 
enhacements such as stack relative addressing and custom FORTH support 
instructions to be added without resynthesizing the core, i.e. all 
enhancements can be added by only changing the microprogram.

Another benefit of the new microprogram and logic structure is that the 
resulting M65C02A core is significantly smaller than the M65C02 core. It is 
also better equipped to work as a single cycle core with both internal and 
external memory.

Implementation
--------------

The implementation of the core provided consists of five Verilog source files 
and several memory initialization files:

    M65C02_CoreV2.v         - M65C02 Top level module
        M65C02_MPCv5.v      - M65C02 MPC with microcycle length controller
        M65C02_AddrGenV2.v  - M65C02 Address Generator module
            M65C02_StkPtr.v - M65C02 Stack Pointer module
        M65C02_ALUv2.v      - M65C02 ALU module
            M65C02_LST.v    - M65C02 ALU Load/Store/Transfer module
            M65C02_LU.v     - M65C02 ALU Logic Unit module
            M65C02_SU.v     - M65C02 ALU Shift Unit module
            M65C02_Add.v    - M65C02 ALU Dual Mode Adder Unit module
            M65C02_WrSel.v  - M65C02 ALU Register Write Enable module
            M65C02_PSWv2.v  - M65C02 ALU Processor Status Word module
    
    M65C02_IDecode_ROMa.coe - M65C02A core microprogram ALU control fields
    M65C02_uPgm_V4a.coe     - M65C02A core microprogram (sequence control)

    M65C02_IDecode_ROMa.txt - M65C02A core microprogram ALU control fields
    M65C02_uPgm_V4a.txt     - M65C02A core microprogram (sequence control)

    M65C02_CoreV2.ucf       - User Constraints File: period and pin LOCs
    M65C02A.tcl             - Project settings file
    
    tb_M65C02A.v            - Completed M65C02A soft-microcomputer testbench
    tb_M65C02_CoreV2.v      - Completed core testbench with test RAM
    
    M65C02_Tst3.txt         - Memory configuration file of M65C02 "ROM" program
    M65C02_RAM.txt          - Memory configuration file for "RAM"
    
    tb_M65C02_ALUv2.v       - self-checking testbench for the M65C02A ALU module

Synthesis
---------

The two primary objectives of the M65C02A core are (1) to minimize the area, 
i.e. logic requirements, and (2) to support single cycle operation from 
internal block RAMs. The M65C02A core meets these two objectives. The settings 
for the synthesis and PAR tools are the same for the M65C02A core as they are 
for the M65C02 core.

The following table shows the results for the M65C02A core compared to the 
M65C02 core:

The ISE 10.1i SP3 implementation results are as follows:

    Core:                           M65C02A       M65C02
    Number of Slice FFs:              125           191
    Number of 4-input LUTs:           482           747
    Number of Occupied Slices:        346           459
    
    Number of BUFGMUXs:                1             1
    Number of RAMB16BWEs               2             2
    
    Best Case Achievable:           22.312 (1)     13.180 (2)
    
    Notes:
        (1) Single cycle memory operation, and single cycle BCD math operations.
        (2) Requires 1 cycle to generate the address and another to read memory.
            Also requires at least two cycles for BCD math operations.
            
The result is that the M65C02A can be connected to internal block RAM for 
single cycle operation, and the M65C02 requires at least two cycles for 
operation with internal block RAM. Further, the M65C02A adder can provide BCD 
math operations in a single cycle, while the M65C02 BCD add unit will require 
two cycles. If the M65C02 core is connected to Block RAM and configured for 
single cycle operation by runnning the core logic on the rising edge of the 
clock and the Block RAM on the falling edge of the same clock, the resulting 
minimum period is at least twice the best period of the core: 13.180. This 
suggests that the best single cycle period of the M65C02 is 26.36 ns. This is 
4.05 ns slower than the M65C02A core, which suggests that the M65C02A, in 
addition to being smaller, is faster than the M65C02 core by significant 
margin.

Status
------

Design and verification is complete. See the release notes below for specific 
features that have been included in the instruction set of the M65C02A core.

Release Notes
-------------

###Release 2.1.0

Release 2.1.0 provides the basic 65C02 instruction set plus the W65C02S WAI 
(WAit for Interrupt) instruction and STP (SToP) instructions. The microprogram 
structure and logic support the implementation of the Rockwell SMBx/RMBx and 
BBSx/BBRx instructions. The microprogram and logic structure also support the 
inclusion of a stack relative addressing mode and instructions. It is also 
possible to support virtual machines such as FORTH interpreter.

###Release 2.2.0

Release 2.2.0 provides the same basic core as Release 2.1.0, but the core has 
been augmented by a rudimentary interrupt handler and MMU, 28kB internal 
memory, and 1 SPI Master and 2 UART peripherals. In addition, the soft-processor 
is being prepared to support Kernel and User modes.

The mode of the processor will be kept in an unimplemented PSW bit, bit 5. The 
normal setting of this is a logic 1, and that will signify Kernel mode. Access 
to IO will be restricted to the Kernel mode. The top-most 256 bytes are 
reserved for IO. The MMU included in this release includes support for the 
Kernel/User mode. The peripherals included in the release are mapped to the 
uppermost 128 bytes of the IO space. The vector table, which is considered 
part of the IO space, is mapped back to the upper most 32 bytes of the 4kB 
Monitor ROM.

The multiplexers for the internal memories and the IO peripherals decrease the 
raw performance that can be achieved. In a Spartan-3A XC3S200A-4VQG100I FPGA, 
the best performance that can be achieved is approximately 30 MHz. In a 
Spartan 6 XC6SLX9-3FTG256I FPGA, the best performance that can be achieved is 
better than 40MHz. The improved performance is a result of two factors: (1) 
the 6-bit LUTs of the Spartan-6 versus the 4-bit LUTs of the Spartan-3A.

A TCL file is included with the repository that can be used to rebuild the 
project. The source files for all of the HDL have been included. Further, all 
of the memory initialization files have been included. Klaus Dormann's 
6502_Functional_Test test program has been completed using ISIM through all of 
the functional tests except binary and BCD addition/subtraction tests. A self-
checking test bench for these tests is included.

###Release 2.2.1

Release 2.2.1 provides an update that adds the capability to support both 8-bit
and 16-bit relative addressing. Changes where made to the operand register data
path and to the relative address mode multiplexer in the address generator. In
the operand register data path, a sign extension multiplexer, explicitly con-
trolled by the microprogram, allows the most significant operand register (OP2)
to be loaded with either an 8-bit value (16-bit relative addressing) or the sign
extension of the least significant operand register (OP1) (8-bit relative
addressing). In addition, the previously unused DI_Op[0] field bit is used to
controlled the sign extension multiplexer in the OP2 data path. To maintain the
current 8-bit relative mode instructions, the microprogram for all of the branch
instructions was updated. (Klaus Dormann's test suite was used for regression
testing, and it passed.)

###Release 2.2.2

With the IO peripheral complement and full BRAM complement, the M65C02A 
microcomputer is able to achieve operation at 30+ MHz in its target FPGA: 
XC3S200A-4VQG100I. Speeds in excess of 40+ MHz are reported in an
XC6SLX9-3FGG256I FPGA.

Improved the support for 16-bit relative addressing. Incorporated and tested the 
following instructions:

    COP zp                                      : CO-Processor Trap - 0xFFF4, X <= operand
    JSR (sp,S),Y                                : Stk Relative Indexed Indirect
    COP #imm                                    : CO-Processor Trap - 0xFFF4, X <= operand
    PHR rel16                                   : Push 16-bit PC-relative addrs
    JMP (sp,S),Y                                : Stk Relative Indexed Indirect
    PLW zp                                      : Pull 16-bit operand zp direct
    PLW abs                                     : Pull 16-bit operand abd direct
    ORA/AND/EOR/ADC/STA/LDA/CMP/SBC sp,S        : Stk Relative
    ORA/AND/EOR/ADC/STA/LDA/CMP/SBC (sp,S),Y    : Stk Relative Indexed Indirect
    PHW zp                                      : Push 16-bit operand zp direct
    PHW #imm16                                  : Push 16-bit constant
    RMBx/SMBx zp                                : Rockwell bit-oriented set/clr
    BSR rel16                                   : Branch to Subroutine PC-relative
    BRA rel16                                   : Branch PC-relative
    PHW abs                                     : Push 16-bit operand abs direct    
    BBRx/BBSx zp,rel                            : Rockwell bit-oriented branches

The following two images define the complete opcode matrix of the M65C02A:

![M65C02A Enhanced Instruction Set: 0x00-0x7F](https://github.com/MorrisMA/M65C02A/blob/master/Images/M65C02A-InstructionSetMatrix%2800-7F%29.JPG)

![M65C02A Enhanced Instruction Set: 0x80-0xFF](https://github.com/MorrisMA/M65C02A/blob/master/Images/M65C02A-InstructionSetMatrix%2880-FF%29.JPG)

###Release 2.2.3

Added four prefix instructions, and tested that they set the appropriate 
internal flags. Began applying the addressing mode modification prefix 
instruction: IND. Remaining three prefix instructions implemented, but their 
intended effects on the instructions have not been implemented for this 
release.

The microprogram and the microprogram has been modified to support the IND 
prefix instruction for the following instructions:

    JSR abs                 => JSR (abs)                : Added absolute indirect
    PLW zp                  => PLW (zp)                 : Added zero page indirect
    PLW abs                 => PLW (abs)                : Added absolute indirect
    ORA/AND/EOR/ADC sp,S    => ORA/AND/EOR/ADC (sp,S)   : Added stack relative indirect
    STA/LDA/CMP/SBC sp,S    => STA/LDA/CMP/SBC (sp,S)   : Added stack relative indirect
    PHW zp                  => PHW (zp)                 : Added zero page indirect
    PHW abs                 => PHW (abs)                : Added zero page indirect
    STZ/STX/STY zp          => STZ/STX/STY (zp)         : Added zero page indirect
    LDX/LDY zp              => LDX/LDY (zp)             : Added zero page indirect
    CPX/CPY zp              => CPX/CPY (zp)             : Added zero page indirect
    STZ/STY zp,X            => STZ/STY (zp,X)           : Added pre-indexed zero page indirect
    STX zp,Y                => STX (zp),Y               : Added post-indexed zero page indirect

Applied the addressing mode modification prefix instruction, IND, to the 
RMW instructions:

    TSB/TRB/BIT zp          => TSB/TRB/BIT (zp)         : Added zero page indirect
    BIT zp,X                => BIT (zp,X)               : Added pre-indexed zero page indirect
    ASL/ROL/LSR/ROR zp      => ASL/ROL/LSR/ROR (zp)     : Added zero page indirect
    ASL/ROL/LSR/ROR zp,X    => STA/LDA/CMP/SBC (zp,X)   : Added pre-indexed zero page indirect
    DEC/INC zp              => DEC/INC (zp)             : Added zero page indirect
    DEC/INC zp,X            => DEC/INC (zp,X)           : Added pre-indexed zero page indirect
    RMBx/SMBx zp            => RMBx/SMBx (zp)           : Added zero page indirect
    BBRx/BBSx zp,rel        => BBRx/BBSx (zp),rel       : Added zero page indirect
    TSB/TRB/BIT abs         => TSB/TRB/BIT (abs)        : Added absolute indirect
    BIT abs,X               => BIT (abs,X)              : Added pre-indexed absolute indirect
    ASL/ROL/LSR/ROR abs     => ASL/ROL/LSR/ROR (abs)    : Added absolute indirect
    ASL/ROL/LSR/ROR abs,X   => STA/LDA/CMP/SBC (abs,X)  : Added pre-indexed absolute indirect
    DEC/INC abs             => DEC/INC (abs)            : Added absolute indirect
    DEC/INC abs,X           => DEC/INC (abs,X)          : Added pre-indexed absolute indirect

Applied the addressing mode modification prefix instruction, IND, to the 
Read-Only and Write-Only Absolute Address Mode instructions:

    ORA/AND/EOR/ADC abs,Y   => ORA/AND/EOR/ADC (abs),Y  : Added post-indexed absolute indirect
    STA/LDA/CMP/SBC abs,Y   => STA/LDA/CMP/SBC (abs),Y  : Added post-indexed absolute indirect
    ORA/AND/EOR/ADC abs     => ORA/AND/EOR/ADC (abs)    : Added absolute indirect
    STA/LDA/CMP/SBC abs     => STA/LDA/CMP/SBC (abs)    : Added absolute indirect
    STY/LDY/CPY abs         => STY/LDY/CPY (abs)        : Added absolute indirect
    STX/LDX/CPX abs         => STX/LDX/CPX (abs)        : Added absolute indirect
    LDY abs,X               => LDY (abs,X)              : Added pre-indexed absolute indirect
    LDX abs,Y               => LDX (abs),Y              : Added post-indexed absolute indirect
    STZ abs                 => STZ (abs)                : Added absolute indirect
    STZ abs,X               => STZ (abs,X)              : Added pre-indexed absolute indirect

###Release 2.2.3a

Changed the default microprogram filenames to reflect last tested 
configuration. No other changes made.

