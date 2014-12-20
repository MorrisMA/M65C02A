..\..\as65\as65 -cgntvw132h58z -l -ofig6502a.bin fig6502a.a65
..\bin2txt fig6502a.bin fig6502a.txt
copy fig6502a.txt c:\XProjects\ISE10.1i\M65C02A\Pgms\figFORTH.txt /y
rem
..\..\as65\as65 -cgntvw132h58z -s2 -l -ofig6502a.hex fig6502a.a65
type fig6502a.hex | ..\..\ih2mem2 > figforth.mem
move /y figforth.mem c:\XProjects\ISE10.1i\M65C02A\Pgms

