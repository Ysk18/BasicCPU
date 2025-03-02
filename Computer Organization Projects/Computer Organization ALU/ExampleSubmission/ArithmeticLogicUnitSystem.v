`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Muhammed Ali Acikgoz and Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////
module ArithmeticLogicUnitSystem (
    input [2:0] RF_OutASel, RF_OutBSel, RF_FunSel,
    input [3:0] RF_RegSel, RF_ScrSel,
    input [4:0] ALU_FunSel,
    input ALU_WF,
    input [1:0] ARF_OutCSel, ARF_OutDSel,
    input [2:0] ARF_FunSel, ARF_RegSel,
    input IR_LH, IR_Write, Mem_WR, Mem_CS,
    input [1:0] MuxASel, MuxBSel,
    input MuxCSel,
    input Clock,

    output [15:0] MuxAOut, MuxBOut,
    output [7:0] MuxCOut
);
    wire [15:0] OutA;
    wire [15:0] OutB;
    wire [15:0] ALUOut;
    wire [3:0] FlagsOut;
    
    wire [15:0] OutC;
    wire [15:0] Address;
    wire [7:0] MemOut;
    
    wire [15:0] IROut;
    
    assign MuxAOut = (MuxASel == 0) ? ALUOut :
                     (MuxASel == 1) ? OutC :
                     (MuxASel == 2) ? {8'b0, MemOut} :
                                      {8'b0, IROut[7:0]}; // or IROut, it is not explained

    assign MuxBOut = (MuxBSel == 0) ? ALUOut :
                     (MuxBSel == 1) ? OutC :
                     (MuxBSel == 2) ? {8'b0, MemOut} :
                                      {8'b0, IROut[7:0]};

    assign MuxCOut = (MuxCSel) ? ALUOut[15:8] : ALUOut[7:0];

    RegisterFile RF(
        .I(MuxAOut), 
        .OutASel(RF_OutASel),
        .OutBSel(RF_OutBSel), 
        .FunSel(RF_FunSel),
        .RegSel(RF_RegSel), 
        .ScrSel(RF_ScrSel),
        .Clock(Clock), 
        .OutA(OutA), 
        .OutB(OutB)
    );

    ArithmeticLogicUnit ALU( 
        .A(OutA), 
        .B(OutB), 
        .FunSel(ALU_FunSel), 
        .WF(ALU_WF), 
        .Clock(Clock), 
        .ALUOut(ALUOut), 
        .FlagsOut(FlagsOut)
    );

    Memory MEM (
        .Address(Address), // ?
        .Data(MuxCOut),
        .WR(Mem_WR),
        .CS(Mem_CS),
        .Clock(Clock),
        .MemOut(MemOut)
    );

    InstructionRegister IR(
        .I(MemOut),
        .Write(IR_Write),
        .LH(IR_LH),
        .Clock(Clock),
        .IROut(IROut)
    );
    
    AddressRegisterFile ARF(
        .I(MuxBOut),
        .OutCSel(ARF_OutCSel),
        .OutDSel(ARF_OutDSel),
        .FunSel(ARF_FunSel),
        .RegSel(ARF_RegSel),
        .Clock(Clock),
        .OutC(OutC),
        .OutD(Address) // ?
    );
    
endmodule