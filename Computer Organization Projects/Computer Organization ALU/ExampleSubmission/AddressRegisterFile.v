`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Muhammed Ali Acikgoz and Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////
module AddressRegisterFile(
    input wire [15:0] I,
    input wire [1:0] OutCSel, OutDSel,
    input wire [2:0] FunSel, RegSel,
    input wire Clock,
    output reg [15:0] OutC, OutD
);

    wire [15:0] QPC, QAR, QSP;
    reg EPC, EAR, ESP;
    
    // Instantiate custom module for general purpose registers
    Register PC (.I(I), .E(EPC), .FunSel(FunSel), .Clock(Clock), .Q(QPC));
    Register AR (.I(I), .E(EAR), .FunSel(FunSel), .Clock(Clock), .Q(QAR));
    Register SP (.I(I), .E(ESP), .FunSel(FunSel), .Clock(Clock), .Q(QSP));

    // Apply register enable signals based on RegSel and ScrSel inputs
    always @(*) begin
        case(RegSel)
            4'b000: begin
                EPC = 1'b1;
                EAR = 1'b1;
                ESP = 1'b1;
            end
            4'b001: begin
                EPC = 1'b1;
                EAR = 1'b1;
                ESP = 1'b0;
            end
	       4'b010: begin
                EPC = 1'b1;
                EAR = 1'b0;
                ESP = 1'b1;
            end
            4'b011: begin
                EPC = 1'b1;
                EAR = 1'b0;
                ESP = 1'b0;
            end
	       4'b100: begin
                EPC = 1'b0;
                EAR = 1'b1;
                ESP = 1'b1;
            end
            4'b101: begin
                EPC = 1'b0;
                EAR = 1'b1;
                ESP = 1'b0;
            end
	       4'b110: begin
                EPC = 1'b0;
                EAR = 1'b0;
                ESP = 1'b1;
            end
            4'b111: begin
                EPC = 1'b0;
                EAR = 1'b0;
                ESP = 1'b0;
            end

            default: begin
                EPC = 1'b0;
                EAR = 1'b0;
                ESP = 1'b0;
            end
        endcase
    end

    // Output selection
    always @(*) begin
        case(OutCSel)
            3'b00: OutC = QPC;
            3'b01: OutC = QPC;
            3'b10: OutC = QAR;
            3'b11: OutC = QSP;
        endcase

        case(OutDSel)
            3'b00: OutD = QPC;
            3'b01: OutD = QPC;
            3'b10: OutD = QAR;
            3'b11: OutD = QSP;
        endcase
    end
endmodule