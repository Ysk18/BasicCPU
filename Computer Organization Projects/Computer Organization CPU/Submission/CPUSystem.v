`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Muhammed Ali Acikgoz and Yavuz Selim Kara
// Project Name: BLG222E Project 2
//////////////////////////////////////////////////////////////////////////////////
module CPUSystem(
    input wire Clock,
    input wire Reset,
    output reg [7:0] T
    );
    
    // Registers for Instruction
    reg [5:0] Opcode;
    reg [7:0] Address;
    reg [2:0] DSTREG;
    reg [2:0] SREG1;
    reg [2:0] SREG2;
    reg [1:0] RSel;
    reg S;
    
    // Registers (or Wires?) for ALUSystem
    reg [2:0] RF_OutASel, RF_OutBSel, RF_FunSel;
    reg [3:0] RF_RegSel, RF_ScrSel;
    reg [4:0] ALU_FunSel;
    reg ALU_WF;
    reg [1:0] ARF_OutCSel, ARF_OutDSel;
    reg [2:0] ARF_FunSel, ARF_RegSel;
    reg IR_LH, IR_Write, Mem_WR, Mem_CS;
    reg [1:0] MuxASel, MuxBSel;
    reg MuxCSel;
    
    //reg [15:0] MuxAOut, MuxBOut;
    //reg [7:0] MuxCOut;
    
    wire [15:0] IROut;
    
    ArithmeticLogicUnitSystem _ALUSystem (
        .RF_OutASel(RF_OutASel), .RF_OutBSel(RF_OutBSel), .RF_FunSel(RF_FunSel),
        .RF_RegSel(RF_RegSel), .RF_ScrSel(RF_ScrSel),
        .ALU_FunSel(ALU_FunSel),
        .ALU_WF(ALU_WF),
        .ARF_OutCSel(ARF_OutCSel), .ARF_OutDSel(ARF_OutDSel),
        .ARF_FunSel(ARF_FunSel), .ARF_RegSel(ARF_RegSel),
        .IR_LH(IR_LH), .IR_Write(IR_Write), .Mem_WR(Mem_WR), .Mem_CS(Mem_CS),
        .MuxASel(MuxASel), .MuxBSel(MuxBSel),
        .MuxCSel(MuxCSel),
        .Clock(Clock),
        .IROut(IROut)
        );
    

    
      function [1:0] ARF_SourceSel;
        input [2:0] in;
        begin
            ARF_SourceSel = in[1:0];
        end
      endfunction
      
      // Define the function
        function [2:0] ARF_DestSel;
          input [2:0] in;
          begin
              case(in[1:0])
                  2'b00: begin
                      ARF_DestSel = 3'b011;
                  end
                  2'b01: begin
                      ARF_DestSel = 3'b011;          
                  end
                  2'b10: begin
                      ARF_DestSel = 3'b101;           
                  end
                  2'b11: begin
                      ARF_DestSel = 3'b110;      
                  end
              endcase
          end
        endfunction
        
        // Define the function
            function [2:0] RF_SourceSel;
              input [2:0] in;
              begin
                  case(in[1:0])
                      2'b00: begin
                          RF_SourceSel = 3'b000;
                      end
                      2'b01: begin
                          RF_SourceSel = 3'b001;             
                      end
                      2'b10: begin
                          RF_SourceSel = 3'b010;                
                      end
                      2'b11: begin
                          RF_SourceSel = 3'b011;                
                      end
                  endcase
              end
            endfunction
        
        // Define the function
            function [3:0] RF_DestSel;
              input [2:0] in;
              begin
                  case(in[1:0])
                      2'b00: begin
                          RF_DestSel = 4'b0111;
                      end
                      2'b01: begin
                          RF_DestSel = 4'b1011;                
                      end
                      2'b10: begin
                          RF_DestSel = 4'b1101;                
                      end
                      2'b11: begin
                          RF_DestSel = 4'b1110;                
                      end
                  endcase
              end
            endfunction

            function [3:0] RF_RSel;
              input [1:0] in;
              begin
                  case(in)
                      2'b00: begin
                          RF_RSel = 4'b0111;
                      end
                      2'b01: begin
                          RF_RSel = 4'b1011;                
                      end
                      2'b10: begin
                          RF_RSel = 4'b1101;                
                      end
                      2'b11: begin
                          RF_RSel = 4'b1110;                
                      end
                  endcase
              end
            endfunction

            function [2:0] RF_OutASel_RSel;
              input [1:0] in;
              begin
                  case(in)
                      2'b00: begin
                          RF_OutASel_RSel = 3'b000;
                      end
                      2'b01: begin
                          RF_OutASel_RSel = 3'b001;                
                      end
                      2'b10: begin
                          RF_OutASel_RSel = 3'b010;                
                      end
                      2'b11: begin
                          RF_OutASel_RSel = 3'b011;                
                      end
                  endcase
              end
            endfunction
      
      // State encoding: One-hot encoding for an 8-state counter
      localparam T0 = 8'b0000_0001;
      localparam T1 = 8'b0000_0010;
      localparam T2 = 8'b0000_0100;
      localparam T3 = 8'b0000_1000;
      localparam T4 = 8'b0001_0000;
      localparam T5 = 8'b0010_0000;
      localparam T6 = 8'b0100_0000;
      localparam T7 = 8'b1000_0000;
      
      initial begin
          ARF_RegSel = 3'b011;
          ARF_FunSel = 3'b011;
          T = T0;
      end
      
      always @(*) begin
          if (!Reset) begin
              ARF_RegSel = 3'b011;
              ARF_FunSel = 3'b011;
              T = T0;
          end
      end
      
      always @(posedge Clock) begin
            if(T == T0) begin
                T <= T1;
            end else if (T == T1) begin
                T <= T2;
            end else if (T == T2) begin
                T <= T3;
            end else if (T == T3) begin
                T <= T4;
            end else if (T == T4) begin
                T <= T5;
            end else if (T == T5) begin
                T <= T6;
            end else if (T == T6) begin
                T <= T7;
            end else if (T == T7) begin
                T <= T0;
            end
        end


    
    always @(*) begin
            case(T)
                T0: begin
                    
                    
                    RF_RegSel <= 4'b1111; // disable Rx
                    RF_ScrSel <= 4'b1111; // disable Sx
                    
                    ALU_WF <= 1'b0; // disable ALU flags

                    // PC to Memory
                    ARF_OutDSel <= 2'b0;
                    
                    // Memory write and enable flags
                    Mem_CS <= 1'b0;
                    Mem_WR <= 1'b0;
                    
                    // IR[7:0]
                    IR_Write <= 1'b1;
                    IR_LH <= 1'b0;
                    
                    // PC Increment
                    ARF_RegSel <= 3'b011;
                    ARF_FunSel <= 3'b001;
                end
                T1: begin
                    
                    
                    RF_RegSel <= 4'b1111; // disable Rx
                    RF_ScrSel <= 4'b1111; // disable Sx
                    
                    ALU_WF <= 1'b0; // disable ALU flags

                    // PC to Memory
                    ARF_OutDSel <= 2'b0;
                    
                    // Memory write and enable flags
                    Mem_CS <= 1'b0;
                    Mem_WR <= 1'b0;
                    
                    // IR[15:8]
                    IR_Write <= 1'b1;
                    IR_LH <= 1'b1;
                    
                    // PC Increment
                    ARF_RegSel <= 3'b011;
                    ARF_FunSel <= 3'b001;
                    
                end
                T2: begin
                    // Disable Memory
                    Mem_CS <= 1'b1;
                    // Stop PC Increment
                    // ifle ARF_RegSel atanan durumlarda çalýþmamasý saðlanacak
                    //ARF_RegSel <= 3'b111;
                    // Disable IR
                    IR_Write <= 1'b0;
                    
                    // Fetch the instruction
                    Opcode <= IROut[15:10];
                    RSel <= IROut[9:8];
                    Address <= IROut[7:0];
                    S <= IROut[9];
                    DSTREG <= IROut[8:6];
                    SREG1 <= IROut[5:3];
                    SREG2 <= IROut[2:0];
                    
                    case(Opcode)
                        6'b000000: begin // BRA

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // VALUE TO S1

                            // Choose Output of MUX A
                            MuxASel <= 2'b11;
                            
                            // Load MuxAOut to S1
                            RF_ScrSel <= 4'b0111;
                            RF_FunSel <= 3'b111;
                        end

                        6'b000001: begin // BNE
                            
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            if (_ALUSystem.ALU.FlagsOut[3] == 0) begin
                                // VALUE TO S1

                                // Choose Output of MUX A
                                MuxASel <= 2'b11;
                                
                                // Load MuxAOut to S1
                                RF_ScrSel <= 4'b0111;
                                RF_FunSel <= 3'b111;
                            end else begin
                                T <= T0;
                            end
                        end
                        
                        6'b000010: begin // BEQ
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            if (_ALUSystem.ALU.FlagsOut[3] == 1) begin
                                // VALUE TO S1

                                // Choose Output of MUX A
                                MuxASel <= 2'b11;
                                
                                // Load MuxAOut to S1
                                RF_ScrSel <= 4'b0111;
                                RF_FunSel <= 3'b111;
                            end else begin
                                T <= T0;
                            end
                        end
                        
                        6'b000011: begin // POP

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags    

                            // Increment SP
                            ARF_RegSel = 3'b110; // SP Enable
                            ARF_FunSel = 3'b001; // FunSel = 001 / Inc
                        end
                        
                        6'b000100: begin // PSH

                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags
                            
                            // M[SP] <= Rx[15:8]
                            RF_OutASel <= RF_OutASel_RSel(RSel); // OutA <= Rx
                            ALU_FunSel <= 5'b10000; // ALU_Out <= OutA
                            MuxCSel <= 1'b1;// MuxC_Out <= ALUOut[15:8]
                            ARF_OutDSel <= 2'b11; // OutD <= SP
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b110; // SP Decrement (-1)
                            ARF_FunSel <= 3'b000; 
                        end
                        
                        6'b000101: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end                            
                        end

                        6'b000110: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end                                                      
                        end

                        6'b000111: begin // LSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001000: begin // LSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001001: begin // ASR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001010: begin // CSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001011: begin // CSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, In T3 send s1 to alu and to the dest
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001100: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001101: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001110: begin // NOT
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1,                             
                            // In T3 send s1 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b001111: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b010000: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b010001: begin // MOVH 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Rsel <= IMMEDIATE (8-bit)// immediate = IR[7:0]
                            // LH - choose Least part of IR (IR[7:0])
                            IR_LH <= 1'b0; // IR[7:0]
                            MuxASel <= 2'b11;
                            RF_RegSel <= RF_RSel(RSel);
                            RF_FunSel <= 3'b110; // Only Write High
                        end

                        6'b010010: begin // LDR (16-bit) 
                            
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            MuxASel <= 2'b10; // MuxAOut <= MemOut
                            RF_RegSel <= RF_RSel(RSel); // Rx <= MuxAOut
                            RF_FunSel <= 3'b101; // Only Write Least
                            // AR INC
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 3'b001;
                        end

                        6'b010011: begin // STR (16-bit)
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutASel <= RF_OutASel_RSel(RSel);
                            ALU_FunSel <= 5'b10000; // Rx to ALUOut
                            MuxCSel <= 1'b1; // first write most part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Decrement
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 3'b000;
                        end

                        6'b010100: begin // MOVL 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // RSel <= IMMEDIATE (8-bit)
                            // LH - choose Least part of IR (IR[7:0])
                            IR_LH <= 1'b0; // IR[7:0]
                            MuxASel <= 2'b11;
                            RF_RegSel <= RF_RSel(RSel);
                            RF_FunSel <= 3'b101; // Only Write Low
                        end

                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b011000: begin // MOVS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // Source larý önce S1 S2 ye atacaðýz
                            // sonra iþlem görmüþ halini uygun yere atacaðýz
                            if (SREG1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end       
                        end

                        6'b011001: begin // ADDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end

                        end

                        6'b011010: begin // SUBS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b011011: begin // ANDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b011100: begin // ORRS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b011101: begin // XORS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG1[2] == 0) begin // sent to s1
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG1); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG1);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b0111; // write to S1 - RF_ScrSel = 0111 - RF_FunSel 010 - 
                                RF_FunSel <= 3'b010; // load I to s1
                            end
                        end

                        6'b011110: begin // BX
                            
                            IR_Write <= 1'b0; // disable IR_write
                            
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[SP] <= PC, PC <= Rx

                            // Address <= SP
                            ARF_OutDSel <= 2'b11; // OutD <= SP

                            
                            ARF_OutCSel <= 2'b00; // OutC <= PC
                            MuxASel <= 2'b01; // MuxAOut <= OutC (PC)
                            RF_ScrSel <= 4'b0111; // S1 enabled
                            RF_FunSel <= 3'b010; // RF FunSel <= Load
                            RF_OutASel <= 3'b100; // OutASel <= S1
                            ALU_FunSel <= 5'b10000;// ALUOut <= OutA
                            MuxCSel <= 1'b1; // MuxCOut <= ALUOut[15:8]

                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement // because it is PSH
                            RF_RegSel <= 3'b110;
                            RF_FunSel <= 3'b000;
                        end

                        6'b011111: begin // BL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags
                            // PC <= M[SP]

                            // Increment SP  - Like POP
                            ARF_RegSel = 3'b110; // SP Enable
                            ARF_FunSel = 3'b001; // FunSel = 001 / Inc
                        end

                        6'b100000: begin // LDRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= VALUE (VALUE defined in ADDRESS bits)
                            MuxASel <= 2'b11; // Mux AOut <= IR[7:0]
                            RF_RegSel <= RF_RSel(RSel); // RF RegSel <= RF RSel (RSel)
                            RF_FunSel <= 3'b111; // RF FunSel <= Extend 111
                        end

                        6'b100001: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[AR+OFFSET] <= Rx (AR is 16-bit register)
                            MuxASel <= 2'b11; // Choose offset (IR)
                            RF_ScrSel <= 4'b0111; // load to S1
                            RF_FunSel <= 3'b111;  // Extend load
                        end
                    endcase
                    
                    
                end
                T3: begin
                    case(Opcode)
                        6'b000000: begin // BRA
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load PC to MUX A
                            ARF_RegSel <= 3'b011;
                            ARF_FunSel <= 3'b010;
                            ARF_OutCSel <= 2'b00;
                                
                            // Choose Output of MUX A
                            MuxASel <= 2'b01;
                                
                            // Load MuxAOut to S2
                            RF_ScrSel <= 4'b1011;
                            RF_FunSel <= 3'b010;
                                
                            // Choose OutA as S1
                            RF_OutASel <= 3'b100;
                            
                            // Choose OutB as S2
                            RF_OutBSel <= 3'b101;
                            
                            // A + B
                            
                            ALU_FunSel <= 5'b10100;   
                        end
                        6'b000001: begin // BNE
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                           
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load PC to MUX A
                            ARF_RegSel <= 3'b011;
                            ARF_FunSel <= 3'b010;
                            ARF_OutCSel <= 2'b00;
                                
                            // Choose Output of MUX A
                            MuxASel <= 2'b01;
                                
                            // Load MuxAOut to S2
                            RF_ScrSel <= 4'b1011;
                            RF_FunSel <= 3'b010;
                                
                            // Choose OutA as S1
                            RF_OutASel <= 3'b100;
                            
                            // Choose OutB as S2
                            RF_OutBSel <= 3'b101;
                            
                            // A + B
                            
                            ALU_FunSel <= 5'b10100;   
                            
                            
                        end
                        6'b000010: begin // BEQ
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load PC to MUX A
                            ARF_RegSel <= 3'b011;
                            ARF_FunSel <= 3'b010;
                            ARF_OutCSel <= 2'b00;
                                
                            // Choose Output of MUX A
                            MuxASel <= 2'b01;
                                
                            // Load MuxAOut to S2
                            RF_ScrSel <= 4'b1011;
                            RF_FunSel <= 3'b010;
                                
                            // Choose OutA as S1
                            RF_OutASel <= 3'b100;
                            
                            // Choose OutB as S2
                            RF_OutBSel <= 3'b101;
                            
                            // A + B
                            
                            ALU_FunSel <= 5'b10100;   
                            
                            
                        end
                        6'b000011: begin // POP

                            
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            
                            // Load SP to Memory
                            ARF_OutDSel <= 2'b11; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            MuxASel <= 2'b10; // Mux_AOut <= MemOut
                            RF_RegSel <= RF_RSel(RSel); // choose Rx with RSel func
                            RF_FunSel <=  3'b101; //  Write to least part [7:0] or Load all because we ll update the most part

                            // Increment SP
                            ARF_RegSel <= 3'b110;
                            ARF_FunSel <= 3'b001;

                            // MemOut to Rx[7:0]
                        end
                        6'b000100: begin // PSH
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[SP] <= Rx[7:0]
                            RF_OutASel <= RF_OutASel_RSel(RSel); // OutA <= Rx
                            ALU_FunSel <= 5'b10000; // ALU_Out <= OutA
                            MuxCSel <= 1'b0;// MuxC_Out <= ALUOut[7:0]
                            ARF_OutDSel <= 2'b11; // OutD <= SP
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement (-1)
                            ARF_RegSel <= 3'b110; // SP Decrement (-1)
                            ARF_FunSel <= 3'b000; 
                            
                            
                        end
                        6'b000101: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // S1 Increment
                            RF_ScrSel <= 4'b0111; // S1 enabled
                            RF_FunSel <= 3'b001; // Rx + 1 / increment        
                        end
                        6'b000110: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // S1 Decrement
                            RF_ScrSel <= 4'b0111; // S1 enabled
                            RF_FunSel <= 3'b000; // Rx - 1 / decrement        
                        end
                        6'b000111: begin // LSL

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b11011; // LSL(S1) to ALUOut
                            if(DSTREG[2] == 0) begin // LSL(S1) to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // LSL(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // LSL(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // LSL(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b001000: begin // LSR

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b11100; // LSR(S1) to ALUOut
                            if(DSTREG[2] == 0) begin // LSR(S1) to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // LSR(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // LSR(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // LSR(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b001001: begin // ASR

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b11101; // ASR(S1) to ALUOut
                            if(DSTREG[2] == 0) begin // ASR(S1) to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // ASR(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // ASR(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // ASR(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b001010: begin // CSL

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b11110; // CSL(S1) to ALUOut
                            if(DSTREG[2] == 0) begin // LSL(S1) to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // CSL(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // CSL(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // CSL(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b001011: begin // CSR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b11111; // CSR(S1) to ALUOut
                            if(DSTREG[2] == 0) begin // CSR(S1) to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // CSR(S1) to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // CSR(S1) loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // CSR(S1) to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b001100: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b001101: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b001110: begin // NOT
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b10010; // NOT S1 to ALUOut
                            if(DSTREG[2] == 0) begin // NOT to ARF
                                MuxBSel <= 2'b00; // NOT to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // NOT loaded destination                               
                            end else begin
                                MuxASel <= 2'b00; // NOT to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b001111: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010000: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010001: begin // MOVH
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010010: begin // LDR (16-bit)
                            
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Rx <= M[AR] (AR is 16-bit register) // In 2 steps
                            ARF_OutDSel <= 2'b10; // OutD <= AR
                            Mem_CS <= 1'b0; // MEMORY READ
                            Mem_WR <= 1'b0;
                            MuxASel <= 2'b10; // MuxAOut <= MemOut
                            RF_RegSel <= RF_RSel(RSel); // Rx <= MuxAOut
                            RF_FunSel <= 3'b110; // Only Write High
                            // AR DEC / to keep the first value same
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 3'b000;
                        end
                        6'b010011: begin // STR (16-bit)
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[AR] <= Rx (AR is 16-bit register)
                            RF_OutASel <= RF_OutASel_RSel(RSel);
                            ALU_FunSel <= 5'b10000; // Rx to ALUOut
                            MuxCSel <= 1'b0; // then write least part
                            // AR to Address
                            ARF_OutDSel <= 2'b10; // AR to OutD
                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 3'b001;
                        end
                        6'b010100: begin // MOVL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011000: begin // MOVS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            
                            
                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b10000; // S1 to ALUOut
                            ALU_WF <= S; // ALU_Flag
                            if(DSTREG[2] == 0) begin // S1 to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // S1 to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // S1 loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // S1 to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011001: begin // ADDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            

                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011010: begin // SUBS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011011: begin // ANDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011100: begin // ORRS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011101: begin // XORS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            
                            // In T2 send sreg1 to s1, 
                            // In T3 send sreg2 to s2, 
                            // In T4 send s1 and s2 to alu and to the dest
                            if (SREG2[2] == 0) begin // sent to S2 !!
                                ALU_WF <= 1'b0; // disable ALU flags
                                ARF_OutCSel <= ARF_SourceSel(SREG2); // Select OutC
                                MuxASel <= 2'b01; // MuxAOut = OutC
                                RF_ScrSel <= 4'b1011; // write to S2  
                                RF_FunSel <= 3'b010; // load I to s2
                            end else begin
                                RF_OutASel <= RF_SourceSel(SREG2);
                                ALU_FunSel = 5'b10000; // ALUOut = A - MuxASel = 00
                                ALU_WF <= S; // ALU_Flag
                                MuxASel = 2'b00; // MuxAOut = A
                                RF_ScrSel <= 4'b1011; // write to S2 
                                RF_FunSel <= 3'b010; // load I to s2
                            end
                        end
                        6'b011110: begin // BX
                            
                            IR_Write <= 1'b0; // disable IR_write
                            
                            
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // M[SP] <= PC, PC <= Rx

                            // Address <= SP
                            ARF_OutDSel <= 2'b11; // OutD <= SP

                            ARF_OutCSel <= 2'b00; // OutC <= PC
                            MuxASel <= 2'b01; // MuxAOut <= OutC (PC)
                            RF_ScrSel <= 4'b0111; // S1 enabled
                            RF_FunSel <= 3'b010; // RF FunSel <= Load
                            RF_OutASel <= 3'b100; // OutASel <= S1
                            ALU_FunSel <= 5'b10000;// ALUOut <= OutA
                            MuxCSel <= 1'b0; // MuxCOut <= ALUOut[7:0]

                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // SP Decrement // because it is PSH
                            RF_RegSel <= 3'b110;
                            RF_FunSel <= 3'b000;

                        end
                        6'b011111: begin // BL
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // PC <= M[SP]

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b11; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            MuxBSel <= 2'b10; // MuxBOut <= MemOut 
                            
                            ARF_RegSel <= 3'b011; // PC Enable
                            ARF_FunSel <=  3'b101; // Only Write Low [7:0]
                            
                            
                        end
                        6'b100000: begin // LDRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                             // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100001: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            ARF_OutCSel <= 2'b10; // AR
                            MuxASel <= 2'b01;
                            RF_ScrSel <= 4'b1011; // S2
                            RF_FunSel <= 3'b010; // LOAD
                            RF_OutASel <= 3'b100; // S1
                            RF_OutBSel <= 3'b101; // S2
                            ALU_FunSel <= 5'b10100; // S1 + S2
                            MuxBSel <= 2'b00; 
                            ARF_RegSel <= 3'b101; // AR enabled
                            ARF_FunSel <= 3'b010; // load
                        end
                    endcase
                    
                end
                T4: begin
                    case(Opcode)
                        6'b000000: begin //BRA
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Choose output MUX B as ALUOut
                            MuxBSel <= 2'b00;
                            
                            // Write to PC
                            ARF_RegSel <= 3'b011;
                            ARF_FunSel <= 3'b010;
                            
                            // Reset S1, S2, and ALUOut
                            RF_ScrSel = 4'b0000;
                            RF_FunSel = 3'b011;
                                               
                        end
                        6'b000001: begin // BNE
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000010: begin // BEQ
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000011: begin // POP
                            
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b11; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            MuxASel <= 2'b10; // Mux_AOut <= MemOut
                            RF_RegSel <= RF_RSel(RSel); // choose Rx with RSel func
                            RF_FunSel <=  3'b110; // Only Write High
                           

                            // MemOut to Rx[15:8]
                            
                        end
                        6'b000100: begin // PUSH 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000101: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b10000; // S1 to ALUOut
                            if(DSTREG[2] == 0) begin // S1 to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // S1 to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // S1 loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // S1 to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b000110: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags

                            RF_OutASel <= 3'b100; // S1 to OutA
                            ALU_FunSel <= 5'b10000; // S1 to ALUOut
                            if(DSTREG[2] == 0) begin // S1 to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // S1 to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // S1 loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // S1 to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            

                        end
                        6'b000111: begin // LSL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;        
                        end
                        6'b001000: begin
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;        
                        end
                        6'b001001: begin 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;       
                        end
                        6'b001010: begin 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;       
                        end
                        6'b001011: begin  

                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;      
                        end
                        6'b001100: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10111; // S1 AND S2 to ALUOut
                            if(DSTREG[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b001101: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11000; // S1 OR S2 to ALUOut
                            if(DSTREG[2] == 0) begin // OR to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // OR to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // OR loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // OR to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b001110: begin // NOT 
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;     
                        end
                        6'b001111: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11001; // S1 XOR S2 to ALUOut
                            if(DSTREG[2] == 0) begin // XOR to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // XOR to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // XOR loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // XOR to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                            
                        end
                        6'b010000: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter


                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11010; // S1 NAND S2 to ALUOut
                            if(DSTREG[2] == 0) begin // NAND to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // NAND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // NAND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // NAND to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b010001: begin // MOVH
                        end
                        6'b010010: begin // LDR (16-bit)
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010011: begin // STR (16-bit)
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010100: begin // MOVL
                        end
                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10100; // S1 + S2 to ALUOut
                            if(DSTREG[2] == 0) begin // ADD to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // ADD to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // ADD loaded destination                               
                            end else begin  
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // ADD to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10101; // S1 + S2 + Carry to ALUOut
                            if(DSTREG[2] == 0) begin // ADC to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // ADC to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // ADC loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // ADC to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10110; // S1 - S2 to ALUOut
                            if(DSTREG[2] == 0) begin // SUB to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // SUB to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // SUB loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // SUB to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b011000: begin // MOVS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011001: begin // ADDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10100; // S1 + S2 to ALUOut
                            ALU_WF <= S; // ALU_Flag
                            if(DSTREG[2] == 0) begin // ADD to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // ADD to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // ADD loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // ADD to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011010: begin // SUBS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10110; // S1 - S2 to ALUOut
                            ALU_WF <= S; // ALU_Flag
                            if(DSTREG[2] == 0) begin // SUB to ARF  
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // SUB to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // SUB loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // SUB to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end

                        end
                        6'b011011: begin // ANDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b10111; // S1 AND S2 to ALUOut
                            ALU_WF <= S; // ALU_Flag
                            if(DSTREG[2] == 0) begin // AND to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // AND to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // AND loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // AND to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011100: begin // ORRS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11000; // S1 OR S2 to ALUOut
                            ALU_WF <= S; // ALU_Flag
                            if(DSTREG[2] == 0) begin // OR to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // OR to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // OR loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // OR to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011101: begin // XORS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            



                            RF_OutASel <= 3'b100; // S1 to OutA
                            RF_OutBSel <= 3'b101; // S2 to OutB
                            ALU_FunSel <= 5'b11001; // S1 XOR S2 to ALUOut
                            ALU_WF <= S; // ALU_Flag
                            if(DSTREG[2] == 0) begin // XOR to ARF
                                RF_RegSel <= 4'b1111; // disable Rx
                                MuxBSel <= 2'b00; // XOR to MuxBOut
                                ARF_RegSel <= ARF_DestSel(DSTREG);
                                ARF_FunSel <= 3'b010; // XOR loaded destination                               
                            end else begin
                                ARF_RegSel <= 3'b111; // disable arf regs
                                MuxASel <= 2'b00; // XOR to MuxAOut
                                RF_RegSel <= RF_DestSel(DSTREG);
                                RF_FunSel <= 3'b010;
                            end
                        end
                        6'b011110: begin // BX
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            // PC <= Rx
                            RF_OutASel <= RF_OutASel_RSel(RSel); // OutASel <= OutA_RSel ()
                            ALU_FunSel <= 5'b10000; // ALUOut <= OutA
                            MuxBSel <= 2'b00; // MuxBOut <= ALUOut
                            ARF_RegSel <= 3'b011; // ARF RegSel <= PC
                            ARF_FunSel <= 3'b010;  // ARF FunSel <= Load
                        end
                        6'b011111: begin // BL
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags



                            // PC <= M[SP]

                            // Load SP to Memory
                            ARF_OutDSel <= 2'b11; // SP to OutD(Address)

                            // Read memory
                            Mem_WR <= 1'b0;
                            Mem_CS <= 1'b0;

                            MuxBSel <= 2'b10; // Mux_BOut <= MemOut
                            ARF_RegSel <= 3'b011; // Enable PC
                            ARF_FunSel <= 3'b110; // Only Write High - PC Load [15:8]
                        end
                        6'b100000: begin // LDRIM
                        end
                        6'b100001: begin // STRIM
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            ARF_OutDSel <= 2'b10; // AR to Address
                            RF_OutASel <= RF_OutASel_RSel(RSel); // Rx
                            ALU_FunSel <= 5'b10000;
                            MuxCSel <= 1'b0; // Write Least

                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;

                            // AR Increment
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 3'b001;

                        end

                    endcase
                end
                T5: begin
                    case(Opcode)
                        6'b000000: begin // BRA
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000011: begin // POP
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000101: begin // INC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b000110: begin // DEC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b001100: begin // AND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b001101: begin // ORR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b001111: begin // XOR
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010000: begin // NAND
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010101: begin // ADD
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010110: begin // ADC
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b010111: begin // SUB
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011000: begin // MOVS
                        end
                        6'b011001: begin // ADDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011010: begin // SUBS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011011: begin // ANDS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011100: begin // ORRS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011101: begin // XORS
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011110: begin // BX
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b011111: begin // BL
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                        6'b100000: begin // LDRIM
                        end
                        6'b100001: begin // STRIM
                            
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags


                            ARF_OutDSel <= 2'b10; // AR to Address
                            RF_OutASel <= RF_OutASel_RSel(RSel); // Rx
                            ALU_FunSel <= 5'b10000;
                            MuxCSel <= 1'b1; // Write Most

                            // Write to memory
                            Mem_WR <= 1'b1;
                            Mem_CS <= 1'b0;
                        end

                        
                    endcase
                end
                T6: begin
                    case(Opcode)
                        6'b100001: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            
                            ALU_WF <= 1'b0; // disable ALU flags


                            RF_OutASel <= 3'b101; // AR Value we recorded
                            ALU_FunSel <= 5'b10000;
                            MuxBSel <= 2'b00;
                            ARF_RegSel <= 3'b101;
                            ARF_FunSel <= 3'b010;
                        end
                    endcase
                end
                T7: begin
                    case(Opcode)
                        6'b100001: begin // STRIM
                            Mem_CS <= 1'b1; // disable memory
                            IR_Write <= 1'b0; // disable IR_write
                            RF_RegSel <= 4'b1111; // disable Rx
                            RF_ScrSel <= 4'b1111; // disable Sx
                            ARF_RegSel <= 3'b111; // disable arf regs
                            ALU_WF <= 1'b0; // disable ALU flags
                            // Reset Sequence Counter
                            T <= T0;
                        end
                    endcase
                end
                default: begin
                    T <= T0;
                end
            endcase
    end

endmodule