

`ifndef HEADERS_H
`define HEADERS_H







//`define BHT_1BIT
//`define BHT_2BIT
//`define BHT_1BITHIS_2BITCNT
//`define BHT_PSHARED 
`define BHT_GSHARED




`define BOOT_PC         3'd1
`define EPC_PC          3'd2
`define TRAP_PC         3'd3
`define CAL_PC          3'd4
`define PREVIOUS_PC     3'd5

// OPCODE DEFINITIONS 
`define OPCODE_LUI      5'b01101
`define OPCODE_AUIPC    5'b00101
`define OPCODE_JAL      5'b11011
`define OPCODE_JALR     5'b11001
`define OPCODE_BRANCH   5'b11000
`define OPCODE_LOAD     5'b00000
`define OPCODE_STORE    5'b01000
`define OPCODE_INT_IMM  5'b00100
`define OPCODE_INT      5'b01100
`define OPCODE_FENCE    5'b00011
`define OPCODE_ECALL    5'b11100
`define OPCODE_EBREAK   5'b11100
`define OPCODE_CSR      5'b11100
`define OPCODE_M        5'b01100


// FUNCT3 DEFINITIONS 
`define FUNCT3_JALR   3'b000
`define FUNCT3_BEQ    3'b000
`define FUNCT3_BNE    3'b001
`define FUNCT3_BLT    3'b100
`define FUNCT3_BGE    3'b101
`define FUNCT3_BLTU   3'b110
`define FUNCT3_BGEU   3'b111
`define FUNCT3_LB     3'b000
`define FUNCT3_LH     3'b001
`define FUNCT3_LW     3'b010
`define FUNCT3_LBU    3'b100
`define FUNCT3_LHU    3'b101
`define FUNCT3_SB     3'b000
`define FUNCT3_SH     3'b001
`define FUNCT3_SW     3'b010
`define FUNCT3_ADDI   3'b000
`define FUNCT3_SLTI   3'b010
`define FUNCT3_SLTIU  3'b011
`define FUNCT3_XORI   3'b100
`define FUNCT3_ORI    3'b110
`define FUNCT3_ANDI   3'b111
`define FUNCT3_SLLI   3'b001
`define FUNCT3_SRLI   3'b101
`define FUNCT3_SRAI   3'b101
`define FUNCT3_ADD    3'b000
`define FUNCT3_SUB    3'b000
`define FUNCT3_SLL    3'b001
`define FUNCT3_SLT    3'b010
`define FUNCT3_SLTU   3'b011
`define FUNCT3_XOR    3'b100
`define FUNCT3_SRL    3'b101
`define FUNCT3_SRA    3'b101
`define FUNCT3_OR     3'b110
`define FUNCT3_AND    3'b111
`define FUNCT3_FENCE  3'b000
`define FUNCT3_CSRRW  3'b001
`define FUNCT3_CSRRS  3'b010  
`define FUNCT3_CSRRC  3'b011 
`define FUNCT3_CSRRWI 3'b101
`define FUNCT3_CSRRSI 3'b110
`define FUNCT3_CSRRCI 3'b111
`define FUNCT3_MUL    3'b000
`define FUNCT3_MULH   3'b001
`define FUNCT3_MULHSU 3'b010
`define FUNCT3_MULHU  3'b011
`define FUNCT3_DIV    3'b100
`define FUNCT3_DIVU   3'b101
`define FUNCT3_REM    3'b110
`define FUNCT3_REMU   3'b111



// FUNCT7 DEFINITIONS 
`define FUNCT7_SLLI 7'b0000000
`define FUNCT7_SRLI 7'b0000000
`define FUNCT7_SRAI 7'b0100000
`define FUNCT7_ADD  7'b0000000
`define FUNCT7_SUB  7'b0100000
`define FUNCT7_SLL  7'b0000000
`define FUNCT7_SLT  7'b0000000
`define FUNCT7_SLTU 7'b0000000
`define FUNCT7_XOR  7'b0000000
`define FUNCT7_SRL  7'b0000000
`define FUNCT7_SRA  7'b0100000
`define FUNCT7_OR   7'b0000000
`define FUNCT7_AND  7'b0000000
`define FUNCT7_M    7'b0000001 


// INSTRUCTION DEFINITIONS 
`define INST_ECALL  32'b00000000000000000000000001110011
`define INST_EBREAK 32'b00000000000100000000000001110011
`define INST_MRET   32'b00110000001000000000000001110011

// IMMEDIATE SELECTION SIGNAL DEFINITIONS 
`define I_IMM   6'b?????1
`define S_IMM   6'b????1?
`define B_IMM   6'b???1??
`define U_IMM   6'b??1???
`define J_IMM   6'b?1????
`define CSR_IMM 6'b1?????


// ALU_I 
`define ALU_FUNCT_ADD    5'b00000;
`define ALU_FUNCT_Shift  5'b00001; 
`define ALU_FUNCT_AND    5'b00010; 
`define ALU_FUNCT_OR     5'b00011;  
`define ALU_FUNCT_XOR    5'b00100; 


// CSR Constants
`define MCYCLE_RESET        32'h00000000
`define TIME_RESET          32'h00000000
`define MINSTRET_RESET      32'h00000000
`define MCYCLEH_RESET       32'h00000000
`define TIMEH_RESET         32'h00000000
`define MINSTRETH_RESET     32'h00000000
`define MTVEC_BASE_RESET    30'd64
`define MTVEC_MODE_RESET    2'b01
`define MSCRATCH_RESET      32'h00000000
`define MEPC_RESET          32'h00000000
`define MCOUNTINHIBIT_CY_RESET  1'b0 
`define MCOUNTINHIBIT_IR_RESET  1'b0

// CSR Operations 
`define CSR_NOP            2'b00
`define CSR_RW             2'b01
`define CSR_RS             2'b10
`define CSR_RC             2'b11

// Performance Counters
`define CYCLE           12'hC00
`define TIME            12'hC01
`define INSTRET         12'hC02
`define CYCLEH          12'hC80
`define TIMEH           12'hC81
`define INSTRETH        12'hC82

// Machine Trap Setup
`define MSTATUS         12'h300
`define MISA            12'h301
`define MIE             12'h304
`define MTVEC           12'h305

// Machine Trap Handling
`define MSCRATCH        12'h340
`define MEPC            12'h341
`define MCAUSE          12'h342
`define MTVAL           12'h343
`define MIP             12'h344

// Machine Counter / Timers
`define MCYCLE          12'hB00
`define MINSTRET        12'hB02
`define MCYCLEH         12'hB80
`define MINSTRETH       12'hB82

// Machine Counter Setup
`define MCOUNTINHIBIT   12'h320


`endif

