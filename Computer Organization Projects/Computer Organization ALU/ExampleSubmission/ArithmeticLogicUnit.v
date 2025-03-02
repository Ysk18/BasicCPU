`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Muhammed Ali Acikgoz and Yavuz Selim Kara
// Project Name: BLG222E Project 1
//////////////////////////////////////////////////////////////////////////////////
module ArithmeticLogicUnit(
    input [15:0] A, B,
    input [4:0] FunSel,
    input WF,
    input Clock,
    output reg [15:0] ALUOut,
    output reg [3:0] FlagsOut
);

    reg [16:0] temp_result; // An extra bit for overflow detection
    reg [15:0] A_ext, B_ext;
    
    
    always @(*) begin
        // Sign extension for 8-bit inputs
        A_ext = {{8{A[7]}}, A[7:0]};
        B_ext = {{8{B[7]}}, B[7:0]};
        
        
        case(FunSel)
            // 8-bit inputs
            5'b00000: begin
                ALUOut = A_ext;
                temp_result[15:0] = A_ext;
                
            end
            5'b00001: begin
                ALUOut = B_ext;
                temp_result[15:0] = B_ext;

            end
            5'b00010: begin
                temp_result[7:0] = ~A[7:0];
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0];

            end
            5'b00011: begin
                temp_result[7:0] = ~B[7:0];
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0];

            end
            5'b00100: begin
                temp_result = A_ext + B_ext;
                ALUOut = temp_result[15:0];

                
            end
            5'b00101: begin
                temp_result = A_ext + B_ext + FlagsOut[2]; 
                ALUOut = temp_result[15:0];

                
            end
            5'b00110: begin
                temp_result = A_ext + ~B_ext + 1'b1;
                ALUOut = temp_result[15:0];

                
            end
            5'b00111: begin
                temp_result[7:0] = A[7:0] & B[7:0];
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0];
                

            end
            5'b01000: begin
                temp_result[7:0] = A[7:0] | B[7:0];
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0];

            end
            5'b01001: begin
                temp_result[7:0] = A[7:0] ^ B[7:0];
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0];

            end
            5'b01010: begin
                temp_result[7:0] = ~(A[7:0] & B[7:0]);
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0];

            end
            5'b01011: begin // padded with 0 for 8-bit shift operations
                temp_result[7:0] = {A[6:0], 1'b0};
                temp_result = {9'b0, temp_result[7:0]}; // Logic shift left
                ALUOut = temp_result[15:0];

            end
            5'b01100: begin
                temp_result[7:0] = {1'b0, A[7:1]};
                temp_result = {9'b0, temp_result[7:0]}; // Logic shift right
                ALUOut = temp_result[15:0];
            end
            5'b01101: begin
                temp_result[7:0] = {A[7], A[7:1]}; // Arithmetic shift right
                temp_result = {{9{A[7]}}, temp_result[7:0]};
                ALUOut = temp_result[15:0];
             
            end
            5'b01110: begin
                temp_result[7:0] = {A[6:0], FlagsOut[2]};
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0]; // Circular shift left

            end
            5'b01111: begin
                temp_result[7:0] = {FlagsOut[2], A[7:1]};
                temp_result = {9'b0, temp_result[7:0]};
                ALUOut = temp_result[15:0]; // Circular shift right
            end
            // 16-bit inputs
            5'b10000: begin
                temp_result[15:0] = A;
                ALUOut = A;

            end
            5'b10001: begin
                temp_result[15:0] = B;
                ALUOut = B;

            end
            5'b10010: begin
                temp_result[15:0] = ~A;
                ALUOut = ~A;

            end
            5'b10011: begin
                temp_result[15:0] = ~B;
                ALUOut = ~B;

            end
            5'b10100: begin
                temp_result = A + B;
                ALUOut = temp_result[15:0];

                
            end
            5'b10101: begin
                temp_result = A + B;
                temp_result = temp_result + FlagsOut[2];
                ALUOut = temp_result[15:0];

                
            end
            5'b10110: begin
                temp_result = A + ~B + 1'b1;
                ALUOut = temp_result[15:0];

                
            end
            5'b10111: begin
                temp_result[15:0] = A & B;
                ALUOut = A & B;
                 
            end
            5'b11000: begin
                temp_result[15:0] = A | B;
                ALUOut = A | B;

            end
            5'b11001: begin
                temp_result[15:0] = A ^ B;
                ALUOut = A ^ B;

            end
            5'b11010: begin
                temp_result[15:0] = ~(A & B);
                ALUOut = ~(A & B);

            end
            5'b11011: begin
                temp_result[15:0] = {A[14:0], 1'b0};
                ALUOut = {A[14:0], 1'b0}; // Logic shift left

      
            end
            5'b11100: begin
                temp_result[15:0] = {1'b0, A[15:1]};
                ALUOut = {1'b0, A[15:1]}; // Logic shift right

            end
            5'b11101: begin
                temp_result[15:0] = {A[15], A[15:1]};
                ALUOut = {A[15], A[15:1]}; // Arithmetic shift right
               
            end
            5'b11110: begin
                temp_result[15:0] = {A[14:0], FlagsOut[2]};
                ALUOut = {A[14:0], FlagsOut[2]}; // Circular shift left

            end
            5'b11111: begin
                temp_result[15:0] = {FlagsOut[2], A[15:1]};
                ALUOut = {FlagsOut[2], A[15:1]}; // Circular shift right
            end
            
            default: begin
                temp_result[15:0] = 16'b0;
                ALUOut = 16'b0; // Default behavior
            end
        endcase
    end
    
    always @(posedge Clock & WF) begin
        
        if (WF == 1'b1) begin
            case (FunSel)
            
                5'b00000: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00001: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00010: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00011: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b00100: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[16];
                    FlagsOut[0] = (A[7] == B[7] && A[7] != temp_result[7]) ? 1 : 0; // Overflow flag
                end
                5'b00101: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[16];
                    FlagsOut[0] = (A[7] == B[7] && A[7] != temp_result[7]) ? 1 : 0;
                end
                5'b00110: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[16];
                    FlagsOut[0] = (A[7] != B[7] && B[7] == temp_result[7]) ? 1 : 0;
                end
                5'b00111: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01000: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01001: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01010: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b01011: begin // ?? Carry Flag ??
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[7];
                end
                5'b01100: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[0];
                end
                5'b01101: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[2] = A[0];
                end
                5'b01110: begin // ?? Carry Flag ??
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[7];
                end
                5'b01111: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[0];
                end
                5'b10000: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b10001: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b10010: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b10011: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b10100: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[16];
                    FlagsOut[0] = (A[15] == B[15] && A[15] != temp_result[15]) ? 1 : 0;
                end
                5'b10101: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[16];
                    FlagsOut[0] = (A[15] == B[15] && A[15] != temp_result[15]) ? 1 : 0;
                end
                5'b10110: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = temp_result[16];
                    FlagsOut[0] = (A[15] != B[15] && B[15] == temp_result[15]) ? 1 : 0;
                end
                5'b10111: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b11000: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b11001: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b11010: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                end
                5'b11011: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[15];
                end
                5'b11100: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[0];
                end
                5'b11101: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[2] = A[0];
                end
                5'b11110: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[15];
                end
                5'b11111: begin
                    FlagsOut[3] = (temp_result[15:0] == 16'b0) ? 1 : 0; // Zero flag
                    FlagsOut[1] = temp_result[15]; // Negative flag
                    FlagsOut[2] = A[0];
                end
            endcase
        end
    end
    
    
        
endmodule