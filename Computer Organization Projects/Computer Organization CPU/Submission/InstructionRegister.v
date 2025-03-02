`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Muhammed Ali Acikgoz and Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////
module InstructionRegister(
    input wire LH,
    input wire Write,
    input wire [7:0] I,
    input wire Clock,
    output reg [15:0] IROut
);

    always @(posedge Clock) begin
        if (Write) begin
            if (LH) // Load MSB
                IROut[15:8] <= I;
            else // Load LSB
                IROut[7:0] <= I;
        end
    end

endmodule