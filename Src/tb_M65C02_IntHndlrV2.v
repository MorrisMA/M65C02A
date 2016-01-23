`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   08:18:33 07/06/2014
// Design Name:   M65C02_IntHndlr
// Module Name:   C:/XProjects/ISE10.1i/M65C02A/tb_M65C02_IntHndlr.v
// Project Name:  M65C02A
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: M65C02_IntHndlr
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_M65C02_IntHndlrV2;

// UUT Signals 

reg     Rst;
reg     Clk;

reg     Rdy;

reg     ABRT;
reg     NMI;
reg     INV;
reg     SYS;
reg     IRQ;
reg     [7:0] RQST;
reg     BRK;
reg     COP;

reg     IRQ_Msk;
reg     LE_Int;
reg     VP;

wire    Int;
wire    [15:0] Vector;

// Instantiate the Unit Under Test (UUT)

M65C02_IntHndlrV2   uut (
                        .Rst(Rst), 
                        .Clk(Clk),

                        .Rdy(Rdy),
                        
                        .ABRT(ABRT),
                        .NMI(NMI),
                        .INV(INV),
                        .SYS(SYS),
                        .IRQ(IRQ), 
                        .RQST(RQST),
                        .BRK(BRK),
                        .COP(COP),
                        
                        .IRQ_Msk(IRQ_Msk), 
                        .LE_Int(LE_Int),
                        .VP(VP),
                        
                        .Int(Int), 
                        .Vector(Vector)
                    );

initial begin
    // Initialize Inputs
    Rst     = 1;
    Clk     = 1;
    
    Rdy     = 1;
    
    ABRT    = 0;
    NMI     = 0;
    INV     = 0;
    SYS     = 0;
    IRQ     = 0;
    RQST    = 0;
    BRK     = 0;
    COP     = 0;
    
    IRQ_Msk = 1;
    LE_Int  = 0;
    VP      = 0;

    // Wait 100 ns for global reset to finish
    #101 Rst = 0;
    
    // Add stimulus here
    
    @(posedge Clk) #1 LE_Int = 1;
    @(posedge Clk) #1 LE_Int = 0;
    @(posedge Clk) IRQ_Msk = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 0;
    
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);

    @(posedge Clk) #1 NMI = 1;
    @(posedge Clk) #1 NMI = 0;
    
    @(posedge Int);
    
    @(posedge Clk);
    @(posedge Clk) #1 LE_Int = 1;
    @(posedge Clk) #1 LE_Int = 0;
    @(posedge Clk) IRQ_Msk = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 0;

    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk) #1 IRQ = 1;
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk) #1 IRQ_Msk = 0;
    
    @(posedge Int);
    
    @(posedge Clk) #1 NMI = 1;
    @(posedge Clk) #1 NMI = 0;
    @(posedge Clk) #1 LE_Int = 1;
    @(posedge Clk) #1 LE_Int = 0;
    @(posedge Clk) #1 IRQ_Msk = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 0;

    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk) #1 IRQ_Msk = 0;
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk) #1 LE_Int = 1;
    @(posedge Clk) #1 LE_Int = 0;
    @(posedge Clk) #1 IRQ_Msk = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 1;
    @(posedge Clk) #1 VP = 0;

    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk);
    @(posedge Clk) #1 IRQ = 0; 
    @(posedge Clk);
    @(posedge Clk) #1 IRQ_Msk = 0;

    @(posedge Clk);
    @(posedge Clk);
    
    @(posedge Clk) COP     = 1;
    @(posedge Clk) BRK     = 1;
    @(posedge Clk) RQST[0] = 1;
    @(posedge Clk) RQST[1] = 1;
    @(posedge Clk) RQST[2] = 1;
    @(posedge Clk) RQST[3] = 1;
    @(posedge Clk) RQST[4] = 1;
    @(posedge Clk) RQST[5] = 1;
    @(posedge Clk) RQST[6] = 1;
    @(posedge Clk) RQST[7] = 1;
    @(posedge Clk) IRQ     = 1;
    @(posedge Clk) SYS     = 1;
    @(posedge Clk) INV     = 1;
    @(posedge Clk) NMI     = 1;
    @(posedge Clk) ABRT    = 1;

end
 
always #5 Clk = ~Clk;
 
endmodule

