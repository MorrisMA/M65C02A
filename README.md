M65C02A Microprocessor Core
=======================

Copyright (C) 2014, Michael A. Morris <morrisma@mchsi.com>.
All Rights Reserved.

Released under LGPL.

General Description
-------------------

This project provides a synthesizable microprogrammed IP core that implements 
the instruction set architecture (ISA) of the 6502/65C02 microprocessors. The 
initial release provides the instruction set of the 65C02 plus the WAI 
instruction added by WDC in its W65C02S processor. The next release will add the 
Rockwell instructions and the the W65C02S' STP instruction.

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
    
    M65C02_IDecod_ROM.coe   - M65C02A core microprogram ALU control fields
    M65C02_uPgm_V4.coe      - M65C02A core microprogram (sequence control)

    M65C02_CoreV2.ucf       - User Constraints File: period and pin LOCs
    M65C02A.tcl             - Project settings file
    
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
    Number of Slice FFs:              181           191
    Number of 4-input LUTs:           458           747
    Number of Occupied Slices:        236           459
    Total Number of 4-input LUTs:     467           760
    
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