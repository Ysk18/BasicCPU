`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Kadir Ozlem
// Project Name: BLG222E Project 1 Simulation
//////////////////////////////////////////////////////////////////////////////////

module ArithmeticLogicUnitSimulation();
    reg[15:0] A, B;
    reg[4:0] FunSel;
    reg WF;
    wire[15:0] ALUOut;
    wire[3:0] FlagsOut;
    integer test_no;
    wire Z, C, N, O;
    CrystalOscillator clk();
    ArithmeticLogicUnit ALU( .A(A), .B(B), .FunSel(FunSel), .WF(WF), 
                            .Clock(clk.clock), .ALUOut(ALUOut), .FlagsOut(FlagsOut));
        
    FileOperation F();
    
    assign {Z,C,N,O} = FlagsOut;
    
    initial begin
        F.SimulationName ="ArithmeticLogicUnit";
        F.InitializeSimulation(0);
        clk.clock = 0;
        
        //Test 1
        test_no = 1;
        A = 16'h1234;
        B = 16'h4321;
        ALU.FlagsOut = 4'b1111;
        FunSel =5'b10100;
        WF =1;
        #5
        F.CheckValues(ALUOut,16'h5555, test_no, "ALUOut");
        F.CheckValues(Z,1, test_no, "Z");
        F.CheckValues(C,1, test_no, "C");
        F.CheckValues(N,1, test_no, "N");
        F.CheckValues(O,1, test_no, "O");
        //Test 2
        test_no = 2;
        clk.Clock();
        
        F.CheckValues(ALUOut,16'h5555, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,0, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");

        //Test 3
        test_no = 3;
        A = 16'h7777; // 16'b0111011101110111
        B = 16'h8889; // 16'b1000100010001001
        ALU.FlagsOut = 4'b0000;
        FunSel =5'b10101;
        WF =1;
        #5
        clk.Clock();
        
        F.CheckValues(ALUOut,16'h0001, test_no, "ALUOut");
        F.CheckValues(Z,1, test_no, "Z");
        F.CheckValues(C,1, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");
        
        //Test 4   // CSR
        test_no = 4; 
        A = 16'h1F1F; 
        B = 16'h8889; 
        ALU.FlagsOut = 4'b1111;
        FunSel =5'b11111; // CSR // only in unsigned integers So there is no negativity
        WF =1;
        #5
        clk.Clock();
        
        F.CheckValues(ALUOut,16'h8F8F, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,1, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");

        //Test 5    // CSL
        test_no = 5; 
        A = 16'h1EF9; 
        B = 16'h8889; 
        ALU.FlagsOut = 4'b1111;
        FunSel =5'b11110; // CSL // only in unsigned integers So there is no negativity
        WF =1;
        #5
        clk.Clock();
        
        F.CheckValues(ALUOut,16'h3DF3, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,0, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");

        //Test 6     // ASR
        test_no = 6; 
        A = 16'h8765; 
        B = 16'h8889; 
        ALU.FlagsOut = 4'b0000;
        FunSel =5'b11101; // ASR  // in signed integers
        WF =1;
        #5
        clk.Clock();
        
        F.CheckValues(ALUOut,16'hC3B2, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,1, test_no, "C");
        F.CheckValues(N,1, test_no, "N");
        F.CheckValues(O,0, test_no, "O");

        //Test 7     // LSR
        test_no = 7; 
        A = 16'h1234; 
        B = 16'h8889; 
        ALU.FlagsOut = 4'b0100;
        FunSel =5'b11100; // LSR  // in unsigned integers
        WF =1;
        #5
        clk.Clock();
        
        F.CheckValues(ALUOut,16'h091A, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,0, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");

    
        //Test 8     // LSL
        test_no = 8; 
        A = 16'h1234; 
        B = 16'h8889; 
        ALU.FlagsOut = 4'b0000;
        FunSel =5'b11011; // LSL  // in unsigned integers
        WF =1;
        #5
        clk.Clock();
        
        F.CheckValues(ALUOut,16'h2468, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,0, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");
         
         
        F.FinishSimulation();
    end
endmodule