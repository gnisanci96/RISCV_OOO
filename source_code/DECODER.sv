
module DECODER(

    input logic [6:0]  OPCODE,
    input logic        FUNCT7_5,
    input logic [6:0]  FUNCT7,
    input logic [2:0]  FUNCT3,
    input logic [1:0]  IADDER_OUT_1_TO_0,
    input logic        TRAP_TAKEN,
    //
    output logic [4:0] ALU_OPCODE,
    output logic [2:0] ALU_M_OPCODE,
    output logic       ALU_M_INST,         // Multiplication Instruction Flag 
    output logic       MEM_WR_REQ,
    output logic [1:0] LOAD_SIZE,
    output logic       LOAD_UNSIGNED,
    output logic [1:0] LOAD_STORE,
    output logic       ALU_SRC,
    output logic       IADDER_SRC,
    output logic       CSR_WR_EN,
    output logic       RD_INST,
    output logic [2:0] WB_MUX_SEL,
    output logic [2:0] IMM_TYPE,
    output logic [2:0] CSR_OP,
    output logic       ILLEGAL_INSTR,
    output logic       MISALIGNED_LOAD,
    output logic       MISALIGNED_STORE,
    output logic [2:0] CONT_TRA_INST
    );
    
    logic is_branch;
    logic is_jal;
    logic is_jalr;
    logic is_auipc;
    logic is_lui;
    logic is_load;
    logic is_store;
    logic is_system;
    logic is_csr;
    logic is_op;
    logic is_op_imm;
    logic is_misc_mem;
    logic is_addi;
    logic is_slti;
    logic is_sltiu;
    logic is_andi;
    logic is_ori;
    logic is_xori;
    logic is_addiw;
    logic is_implemented_instr;
    logic is_mul;
    logic is_mulh;
    logic is_mulhsu;
    logic is_mulhu;
    logic is_div;
    logic is_divu;
    logic is_rem;
    logic is_remu;
    logic mal_word;
    logic mal_half;
    logic misaligned;
        
    //LOADSIZE 00 --> LB, LBU 
    //         01 --> LH, LHU 
    //         10 --> LW
    //         11 --> XX 
    
        
    assign LOAD_SIZE[0]  = FUNCT3[0];
    assign LOAD_SIZE[1]  = FUNCT3[1];
    assign LOAD_UNSIGNED = FUNCT3[2];
    assign ALU_SRC   = OPCODE[5];
    assign is_branch = OPCODE[6] & OPCODE[5] & ~OPCODE[4] & ~OPCODE[3] & ~OPCODE[2]; // Conditional Branch Instructions (BEQ,BNE,BLT,BGE,BLTU,BGEU)
    assign is_jal    = OPCODE[6] & OPCODE[5] & ~OPCODE[4] & OPCODE[3] & OPCODE[2];
    assign is_jalr   = OPCODE[6] & OPCODE[5] & ~OPCODE[4] & ~OPCODE[3] & OPCODE[2];
    assign is_auipc  = ~OPCODE[6] & ~OPCODE[5] & OPCODE[4] & ~OPCODE[3] & OPCODE[2];
    assign is_lui    = ~OPCODE[6] & OPCODE[5] & OPCODE[4] & ~OPCODE[3] & OPCODE[2];
    assign is_op     = ~OPCODE[6] & OPCODE[5] & OPCODE[4] & ~OPCODE[3] & ~OPCODE[2];
    assign is_op_imm = ~OPCODE[6] & ~OPCODE[5] & OPCODE[4] & ~OPCODE[3] & ~OPCODE[2];
    assign is_addi   = is_op_imm & ~FUNCT3[2] & ~FUNCT3[1] & ~FUNCT3[0]; 
    assign is_slti   = is_op_imm & ~FUNCT3[2] & FUNCT3[1] & ~FUNCT3[0];
    assign is_sltiu  = is_op_imm & ~FUNCT3[2] & FUNCT3[1] & FUNCT3[0];
    assign is_andi   = is_op_imm & FUNCT3[2] & FUNCT3[1] & FUNCT3[0];
    assign is_ori    = is_op_imm & FUNCT3[2] & FUNCT3[1] & ~FUNCT3[0];
    assign is_xori   = is_op_imm & FUNCT3[2] & ~FUNCT3[1] & ~FUNCT3[0];
    assign is_load   = ~OPCODE[6] & ~OPCODE[5] & ~OPCODE[4] & ~OPCODE[3] & ~OPCODE[2];
    assign is_store  = ~OPCODE[6] & OPCODE[5] & ~OPCODE[4] & ~OPCODE[3] & ~OPCODE[2];
    assign is_system = OPCODE[6] & OPCODE[5] & OPCODE[4] & ~OPCODE[3] & ~OPCODE[2];
    assign is_misc_mem = ~OPCODE[6] & ~OPCODE[5] & ~OPCODE[4] & OPCODE[3] & OPCODE[2];
    assign is_csr    = is_system & (FUNCT3[2] | FUNCT3[1] | FUNCT3[0]);
    // 
    assign is_m_extension = FUNCT7[0] & ~OPCODE[6] & OPCODE[5] & OPCODE[4] & ~OPCODE[3] & ~OPCODE[2]; 
    assign is_mul     = ~FUNCT3[2] & ~FUNCT3[1] & ~FUNCT3[0] & is_m_extension;
    assign is_mulh    = ~FUNCT3[2] & ~FUNCT3[1] &  FUNCT3[0] & is_m_extension;
    assign is_mulhsu  = ~FUNCT3[2] &  FUNCT3[1] & ~FUNCT3[0] & is_m_extension;
    assign is_mulhu   = ~FUNCT3[2] &  FUNCT3[1] &  FUNCT3[0] & is_m_extension;
    assign is_div     =  FUNCT3[2] & ~FUNCT3[1] & ~FUNCT3[0] & is_m_extension;
    assign is_divu    =  FUNCT3[2] & ~FUNCT3[1] &  FUNCT3[0] & is_m_extension;
    assign is_rem     =  FUNCT3[2] &  FUNCT3[1] & ~FUNCT3[0] & is_m_extension;
    assign is_remu    =  FUNCT3[2] &  FUNCT3[1] &  FUNCT3[0] & is_m_extension; 
    assign ALU_M_OPCODE = FUNCT3;
    //
    assign IADDER_SRC = is_load | is_store | is_jalr;
    assign RD_INST    = is_lui  | is_auipc | is_jalr | is_jal | is_op | is_load | is_csr | is_op_imm | is_m_extension;
    //
    assign LOAD_STORE[0] = is_store;
    assign LOAD_STORE[1] = is_load;
    //
    assign ALU_OPCODE[2:0] = |LOAD_STORE ? 2'b000 : FUNCT3;
    assign ALU_OPCODE[3]   = |LOAD_STORE ? 1'b0   : FUNCT7_5 & ~(is_addi | is_slti | is_sltiu | is_andi | is_ori | is_xori);
    assign ALU_OPCODE[4]   = is_branch;
    //
    //assign ALU_OP_SEL [0] = is_branch | is_sub   | is_slti  | is_sltiu | is_slt | is_sltu  ;        // subtraction Enable
    //assign ALU_OP_SEL [1] = is_slti   | is_sltiu | is_slt   | is_sltu ;                             // Comparison Enable 
    //assign ALU_OP_SEL [2] = is_bltu   | is_bgeu  | is_sltiu | is_sltu ;                             // unsigned Enable 
    //assign ALU_OP_SEL [3] = is_srli   | is_srai  | is_srl   | is_srl  | is_sra ;                    // Right shift enable
    //assign ALU_OP_SEL [4] = is_srai   | is_sra ;                                                    // Arithmetic shift 
    //assign ALU_OP_SEL [5] = is_sll    | is_slli  | is_srli  | is_srai | is_srl   | is_srl | is_sra; // Shift Operation
    //assign ALU_OP_SEL [6] = is_add    | is_addi  | is_sub   | is_slti | is_sltiu | is_slt | is_sltiu | is_lui | is_auipc | is_jal | is_jalr;  // Arithmetic out
    //assign ALU_OP_SEL [7] = is_and    | is_andi; // Logical AND
    //assign ALU_OP_SEL [8] = is_or     | is_ori ; // Logical OR
    //assign ALU_OP_SEL [9] = is_xor    | is_xori; // Logical XOR
    
    
    
    assign ALU_M_INST = is_m_extension;
    assign CONT_TRA_INST[0] = is_branch;
    assign CONT_TRA_INST[1] = is_jal;
    assign CONT_TRA_INST[2] = is_jalr;
    assign CSR_WR_EN = is_csr;
    assign WB_MUX_SEL[0] = is_load | is_auipc | is_jal | is_jalr;
    assign WB_MUX_SEL[1] = is_lui | is_auipc;
    assign WB_MUX_SEL[2] = is_csr | is_jal | is_jalr;
    assign IMM_TYPE[0] = is_op_imm | is_load | is_jalr | is_branch | is_jal;
    assign IMM_TYPE[1] = is_store | is_branch | is_csr;
    assign IMM_TYPE[2] = is_lui | is_auipc | is_jal | is_csr;
    assign CSR_OP = FUNCT3;
    assign is_implemented_instr = is_op | is_op_imm | is_branch | is_jal | is_jalr | is_auipc | is_lui | is_system | is_misc_mem | is_load | is_store;
   // assign ILLEGAL_INSTR = ~OPCODE[1] | ~OPCODE[0] | ~is_implemented_instr;
    assign ILLEGAL_INSTR = (OPCODE[1:0] !== 2'b11); // non-synthesible
    assign mal_word = FUNCT3[1] & ~FUNCT3[0] & (IADDER_OUT_1_TO_0[1] | IADDER_OUT_1_TO_0[0]);
    assign mal_half = ~FUNCT3[1] & FUNCT3[0] & IADDER_OUT_1_TO_0[0];
    assign misaligned = mal_word | mal_half;
    assign MISALIGNED_STORE = is_store & misaligned;
    assign MISALIGNED_LOAD = is_load & misaligned;
    assign MEM_WR_REQ = is_store & ~misaligned & ~TRAP_TAKEN;
        
endmodule