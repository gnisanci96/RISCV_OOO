`timescale 1ns / 1ps
`include "headers.vh"



module ALU_I
  (
    input  logic [31:0] Num1,
    input  logic [31:0] Num2, 
    input  logic [4:0]  OpSel, 
    output logic [31:0] Result
  );
  
  logic [31:0] add_result;
  logic        add_or_sub;
  logic [31:0] add_op2;
  //
  logic signed [31:0] Num1_signed;
  logic signed [31:0] Num2_signed;
  
  
  
  /////////////////////////////////////////////////////////
  // ADD, ADDI, SUB Operations 
  assign add_or_sub = OpSel[0]; // 1:SUB 0:ADD
  assign add_op2 = ( ({32{add_or_sub}}^Num2)+add_or_sub); // Take 2s compliment if it is SUB instruction
  assign add_result = Num1 + add_op2;  
  
  // SLT 
  logic sltu_result = Num1 < Num2; 
  
  
  assign Num1_signed = Num1;
  assign Num2_signed = Num2;

  
 always_comb
  begin
     if(~OpSel[4]) // --> NOT CONDITIONAL BRANCH INSTRUCTION
     begin
       case(OpSel[2:0])
        `FUNCT3_ADD : Result = add_result; 
        `FUNCT3_SRL : Result = OpSel[3] ? (Num1 >> Num2[4:0]) : (Num1 >>> Num2[4:0]);
        `FUNCT3_OR  : Result = Num1 | Num2;
        `FUNCT3_AND : Result = Num1 & Num2;           
        `FUNCT3_XOR : Result = Num1 ^ Num2;
        `FUNCT3_SLT : Result =(Num1_signed < Num2_signed);
        `FUNCT3_SLTU: Result =(Num1 < Num2);
        `FUNCT3_SLL : Result = Num1 << Num2[4:0];
       endcase 
     end else begin // --> CONDITIONAL BRANCH INSTRUCTION
       case(OpSel[2:0]) 
        `FUNCT3_BEQ : Result = (Num1==Num2); 
        `FUNCT3_BNE : Result = (Num1!=Num2);
        `FUNCT3_BLT : Result = (Num1_signed < Num2_signed);
        `FUNCT3_BGE : Result = (Num1_signed >= Num2_signed);           
        `FUNCT3_BLTU: Result = (Num1 < Num2);
        `FUNCT3_BGEU: Result = (Num1 >= Num2);
       endcase 
     end 
  end 
 

endmodule :ALU_I


module ALU_I_tb();
  
  logic [31:0] Num1;
  logic [31:0] Num2;
  logic [31:0] Result;  
                            // ALU OPERATION SELECT SIGNALS 
                            //   FUNC7[5],  FUNC3   OPCODE[5]
  typedef enum logic [5:0] {ADDI   ={2'b0,`FUNCT3_ADDI ,1'b0},
                            SLTI   ={2'b0,`FUNCT3_SLTI ,1'b0},
                            SLTIU  ={2'b0,`FUNCT3_SLTIU,1'b0},
                            XORI   ={2'b0,`FUNCT3_XORI ,1'b0},
                            ORI    ={2'b0,`FUNCT3_ORI  ,1'b0},
                            ANDI   ={2'b0,`FUNCT3_ANDI ,1'b0},
                            SLLI   ={2'b0,`FUNCT3_SLLI ,1'b0},
                            SRLI   ={2'b0,`FUNCT3_SRLI ,1'b0},
                            SRAI   ={2'b1,`FUNCT3_SRAI ,1'b0},
                            ADD    ={2'b0,`FUNCT3_ADD  ,1'b1},
                            SUB    ={2'b1,`FUNCT3_SUB  ,1'b1},
                            SLL    ={2'b0,`FUNCT3_SLL  ,1'b1},
                            SLT    ={2'b0,`FUNCT3_SLT  ,1'b1},
                            SLTU   ={2'b0,`FUNCT3_SLTU ,1'b1},
                            XORop  ={2'b0,`FUNCT3_XOR  ,1'b1},
                            SRL    ={2'b0,`FUNCT3_SRL  ,1'b1},
                            SRA    ={2'b1,`FUNCT3_SRA  ,1'b1},
                            ORop   ={2'b0,`FUNCT3_OR   ,1'b1},
                            ANDop  ={2'b0,`FUNCT3_AND  ,1'b1},
                            BEQ    ={2'b10,`FUNCT3_BEQ  ,1'b0},
                            BNE    ={2'b10,`FUNCT3_BNE  ,1'b0},
                            BLT    ={2'b10,`FUNCT3_BLT  ,1'b0},
                            BGE    ={2'b10,`FUNCT3_BGE  ,1'b0},
                            BLTU   ={2'b10,`FUNCT3_BLTU ,1'b0},
                            BGEU   ={2'b10,`FUNCT3_BGEU ,1'b0}
                            } OpSelType; 
  
  OpSelType op;
  

  
  initial begin
  Num1 = 32'hf0000000;
  Num2 = 32'b111;
  op = BEQ;   
  #100;  
  op = BNE;   
  #100;    
  op = BLT;   
  #100;  
  op = BGE;   
  #100;    
  op = BLTU;   
  #100;
  op = BGEU;   
  #100;
          
  $finish;  
  end 


  // DUT Instantiation 
  ALU_I uALU_I(
    .Num1   (Num1),
    .Num2   (Num2),
    .OpSel  (op),
    .Result (Result)
  );
  
  
  initial begin
    // Dump waves
    $dumpfile("dump.vcd");
    $dumpvars(1); 
  end 
  
endmodule 



  //logic zero_flag;
  //logic negative_flag;
  
  /////////////////////////////////////
  // Arithmetic OPERATIONS   ADD-ADDI-SUB-SLTI-SLTIU-SLT-SLTU
  ////////////////////////////////////
  // ALU operation selection signals 
  // assign alu_op_sel_o [0] = BRANCH_INSTS | SUB | SLTI | SLTIU | SLT | SLTU  ; // Subtraction Enable
  // assign alu_op_sel_o [1] = SLTI | SLTIU | SLT   | SLTU ;                             // Comparison Enable 
  // assign alu_op_sel_o [2] = BLTU | BGEU  | SLTIU | SLTU ;                             // Unsigned Enable 
  // assign alu_op_sel_o [3] = SRLI | SRAI  | SRL   | SRL  | SRA ;                       // Right shift enable
  // assign alu_op_sel_o [4] = SRAI | SRA ;                                              // Arithmetic shift 
  // assign alu_op_sel_o [5] = SLL  | SLLI  | SRLI  | SRAI | SRL   | SRL | SRA;          // Shift Operation
  // assign alu_op_sel_o [6] = ADD  | ADDI  | SUB   | SLTI | SLTIU | SLT | SLTIU | LUI | AUIPC | JAL | JALR;  // Arithmetic out
  // assign alu_op_sel_o [7] = AND  | ANDI; 
  // assign alu_op_sel_o [8] = OR   | ORI ; 
  // assign alu_op_sel_o [9] = XOR  | XORI; 
  //logic [32:0] alu_in1 = {1'b0,Num1}; 
  //logic [32:0] alu_in2 = {1'b0,Num2}; 
  //
  //logic [32:0]  add_inp1     =  OpSel[2]  ?  { 1'b0 ,Num1 }  :  { Num1[31] , Num1 } ;
  //logic [32:0]  add_inp2     =  {33{OpSel[0]}}  ^  ( OpSel[2] ?  { 1'b0 , Num2 }  :  { Num2[31] , Num2 } ) ;
  //logic c_inp                =  OpSel[0];
  //logic [32:0]  add_out      =  add_inp1  +  add_inp2  +  c_inp;
  //logic [31:0]  arith_out    =  OpSel[1]  ?  { 31'd0 , add_out[32] }  :  add_out[31:0];
  /////////////////////////////////////
  // Flag Generation-BEQ-BNE-BLT-BGE-BLTU-BGEU
  ////////////////////////////////////
  //assign  zero_flag      =  ( add_out[31:0] == 32'd0 ) ;
  //assign  negative_flag  =    add_out[32] ;
  


