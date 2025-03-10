`timescale 1ns / 1ps
`include "coretb.vh"
`include "headers.vh"


// The Test Bench Stimulus 
// 1 - Fill the Instruction Queue with Instructions 
// GORKEM 5/27/2023
module CORE
#(parameter DATAWITHBIT  = 32,
  parameter FIFOSIZE     = 8,
  parameter ROBSIZE      = 8,
  parameter RSSIZE       = 5,
  parameter LSQSIZE      = 16,
  parameter PROGMEMSIZE  = 10,
  parameter PTENTRYSIZE  = 8   )
(
  input logic                      start,
  input logic                      clk,
  input logic                      rstn
);


//================================
// Local Signals    
//================================
localparam [31:0] PC_BOOT    = 32'h0; 

localparam BRANCHTABLEPCSIZE = 10;
localparam PHISTORYSIZE      = 3;
localparam GHISTORYSIZE      = 3;
//
localparam VASIZE          = 32;
localparam PASIZE          = 12;
localparam L1OFFSETSIZE    = 4;
localparam L1INDEXSIZE     = 2;
localparam PAGEOFFSETSIZE  = (L1INDEXSIZE + L1OFFSETSIZE);
localparam L1WAYSIZE       = 2;
localparam L2INDEXSIZE     = 2;
localparam L2WAYSIZE       = 3;
localparam TLBINDEXSIZE    = 6;
//
localparam L1TAGSIZE        = (VASIZE-L1OFFSETSIZE-L1INDEXSIZE);
localparam L2TAGSIZE        = (PASIZE-L1OFFSETSIZE-L2INDEXSIZE);
localparam VAPAGENUMBERSIZE = (VASIZE - PAGEOFFSETSIZE);
localparam TLBTAGSIZE       = (VAPAGENUMBERSIZE - TLBINDEXSIZE);
localparam TLBDATASIZE      = (PASIZE - PAGEOFFSETSIZE);
//
localparam BTCSIZE          = 3;
localparam BRTSIZE          = 3;


//
logic [31:0]               pc;
logic [31:0]               pc_reg;
logic [31:0]               pc_p_4;
logic [31:0]               btb_pc_target;
logic                      btb_pc_target_valid;
logic                      bht_taken;
logic [31:0]               progmem_out;
logic                      pc_stall;
logic                      branch_taken_decision;
// TARGET PC CALCULATION 
logic [31:0]               b_type_imm;
logic [31:0]               j_type_imm;
logic [31:0]               branch_target;
logic [31:0]               jal_target;
logic [31:0]               pc_target;
logic                      taken_prediction;
logic                      bht_update_en;

//
logic                      flush      ,flush_reg;
logic [31:0]               flush_pc;
logic                      flush_taken,flush_taken_reg;  
//
logic                      iq_rd_en;
logic [(DATAWITHBIT-1):0]  inst;
logic [31:0]               iq_pc,inst_pc;
logic                      iq_full;
logic                      iq_empty;
logic                      iq_wrt_en;
logic  [31:0]              iq_wrt_inst; 
logic                      iq_taken_pre;
logic  [31:0]              iq_target_pre;
//
logic                      iq_taken_pre_out;
logic  [31:0]              iq_target_pre_out;

logic                      rob_full; 
// 
logic [6:0]   opcode;
logic [4:0]   rs1;
logic [4:0]   rs2;
logic [4:0]   rd;
logic [2:0]   funct3;
logic [6:0]   funct7;
//
logic [2:0]                 imm_type; 
logic [31:0]                imm;
logic                       illegal_inst;
//
logic [31:0]                rs1_rf_value;
logic [31:0]                rs2_rf_value;
logic                       rf_we;  
logic [4:0]                 rf_rd;  
logic [31:0]                rf_rd_value;  
//
logic                       rat_issue_we;
logic                       rat_update;
logic                       rat_issue_rob_or_rf;
//
logic [(ROBSIZE-1):0]       rat_rs1_rob_addr;
logic [(ROBSIZE-1):0]       rat_rs2_rob_addr;
logic                       rat_rs1_rob_or_rf;
logic                       rat_rs2_rob_or_rf; 
//
logic                       rob_we;
logic [4:0]                 rob_rd;
logic [(ROBSIZE-1):0]       rob_issue_ptr;
//
logic                       rob_exedone_flag; // It tells that one of the instructions finished execution  so we need to update ROB
logic [(ROBSIZE-1):0]       rob_exedone_tag;  // When the i_exedone_flag is high, it tells which ROB entry finished execution
logic [31:0]                rob_exe_val;      // The instruction that finished the execution generated this value         
// 
logic [31:0]                rob_rs1_value;
logic [31:0]                rob_rs2_value;
logic                       rob_rs1_valid;
logic                       rob_rs2_valid;
//
logic                       rob_commit_ready;
logic [(ROBSIZE-1):0]       rob_commit_rob_addr;
logic [31:0]                rob_commit_inst_pc;
logic [31:0]                rob_commit_value;
logic [4:0]                 rob_commit_rd;
logic                       rob_commit_exception;
logic                       rob_commit_rd_inst;
logic [1:0]                 rob_commit_load_store;
logic [2:0]                 rob_commit_cont_tra_inst;
logic                       commit_en;
logic [(ROBSIZE-1):0]       commit_rob_addr;
logic [31:0]                commit_inst_pc;
logic [31:0]                commit_value;
logic [4:0]                 commit_rd;
logic                       commit_exception;
logic                       commit_rd_inst;
logic [2:0]                 commit_cont_tra_inst;
logic [1:0]                 commit_load_store;
// 
logic [31:0]                rs_rs1_value;
logic [31:0]                rs_rs2_value;
//
logic                       rs1_issue_we;
logic                       rs2_issue_we;
logic                       alu_m_rs_issue_we;
logic [4:0]                 alu_opcode;
logic [2:0]                 alu_m_opcode;
logic [2:0]                 alu_m_rs_issue_opcode;


logic                       rs_rs1_valid; 
logic                       rs_rs2_valid; 
  // Dispatch Signals 
logic                       alu1_busy;
logic [4:0]                 alu1_opcode;
logic [31:0]                alu1_rs1_value;
logic [31:0]                alu1_rs2_value;   
logic [(ROBSIZE-1):0]       alu1_rob_addr;
logic                       alu1_ex_en;
//
logic                       alu2_busy;
logic [4:0]                 alu2_opcode;
logic [31:0]                alu2_rs1_value;
logic [31:0]                alu2_rs2_value;   
logic [(ROBSIZE-1):0]       alu2_rob_addr;
logic                       alu2_ex_en;
//
logic                       alu_m_busy;
logic [4:0]                 alu_m_rs_opcode;
logic [31:0]                alu_m_rs1_value;
logic [31:0]                alu_m_rs2_value;   
logic [(ROBSIZE-1):0]       alu_m_rob_addr;
logic                       alu_m_ex_en;
// BROADCAST SIGNALS 
logic                       alu1_broadcast_en;
logic                       alu1_broadcast_ready; 
logic [31:0]                alu1_broadcast_data;
logic [(ROBSIZE-1):0]       alu1_broadcast_rob_addr;
logic                       alu1_broadcast_addr_cal;
logic                       alu1_broadcast_con_branch_comp;
// 
logic                       alu2_broadcast_en;
logic                       alu2_broadcast_ready; 
logic [31:0]                alu2_broadcast_data;
logic [(ROBSIZE-1):0]       alu2_broadcast_rob_addr;
logic                       alu2_broadcast_addr_cal;
logic                       alu2_broadcast_con_branch_comp;
// 
logic                       alu_m_broadcast_en;
logic                       alu_m_broadcast_ready; 
logic [31:0]                alu_m_broadcast_data;
logic [(ROBSIZE-1):0]       alu_m_broadcast_rob_addr;
// BROADCAST SIGNALS 
logic                       broadcast_en;
logic [(ROBSIZE-1):0]       broadcast_rob_addr;
logic [31:0]                broadcast_data;
logic                       broadcast_addr_cal;
logic                       broadcast_con_branch_comp;
// Reservation Station FULL Flag 
logic                       rs1_full; 
logic                       rs2_full; 
logic                       alu_m_rs_full; 
// DECODER SIGNALS 
logic                       rd_inst; 
logic                       imm_inst;
logic [2:0]                 cont_tra_inst;
logic                       m_inst;
logic [1:0]                 load_store;
logic                       load_unsigned;
logic [1:0]                 load_size;
// LOAD/STORE QUEUE SIGNALS 
logic                       lsq_new_inst_we;
logic [31:0]                lsq_rs2_data;
logic [(ROBSIZE-1):0]       lsq_rs2_rob_addr;
logic                       lsq_rs2_data_valid;
logic                       lsq_load_or_store;   
logic                       lsq_full;
logic                       lsq_empty;
// LSQ COMMIT SIGNALS 
logic                       lsq_commit_en;
logic                       lsq_commit_ready;
logic                       lsq_commit_load_store;
logic [31:0]                lsq_commit_data;  
logic [(ROBSIZE-1):0]       lsq_commit_rob_addr;
// DATA MEMORY SIGNALS 
logic                       data_mem_busy;
logic [31:0]                data_mem_addr;
logic                       data_mem_cs;
logic [(ROBSIZE-1):0]       data_mem_rob_addr;
logic                       data_mem_load_store;
logic [31:0]                data_mem_store_data;
logic [1:0]                 data_mem_size;
logic                       data_mem_signed_unsigned;
//
logic                       mem_broadcast_en;       // Broadcast Enable 
logic                       mem_broadcast_ready;    // Broadcast Data Ready 
logic [(ROBSIZE-1):0]       mem_broadcast_rob_addr; // Broadcast ROB Addr 
logic [31:0]                mem_broadcast_load_data;// Broadcast Data 
// BRANCH TARGET CALCULATOR SIGNALS 
logic                      btc_flush_i;
logic                      btc_wr_en_i;
logic [(ROBSIZE-1):0]      btc_wr_rob_addr_i;
logic [31:0]               btc_wr_op1_i;
logic [(ROBSIZE-1):0]      btc_wr_op1_rob_addr_i;    
logic                      btc_wr_op1_v_i;
logic [31:0]               btc_wr_op2_i;
logic                      btc_wr_op2_v_i;  
logic                      btc_broadcast_i;
logic [(ROBSIZE-1):0]      btc_broadcast_rob_addr_i;
logic [31:0]               btc_broadcast_data_i;
logic                      btc_fifo_emtpy_o;
logic [31:0]               btc_fifo_target_cal_o;
logic [(ROBSIZE-1):0]      btc_fifo_rob_addr_o;  
logic                      btc_fifo_rd_en_i;                         
logic                      btc_full_o;  

// BRT TABLE SIGNALS 
logic                      brt_flush_i;
//=====================================
// ISSUE INTERFACE    
//=====================================   
logic                      brt_wr_en_i;
logic [ROBSIZE-1:0]        brt_wr_rob_addr_i;
logic [31:0]               brt_wr_target_cal_i;
logic                      brt_wr_target_cal_v_i; 
logic [31:0]               brt_wr_target_pre_i;
logic                      brt_wr_taken_cal_i; 
logic                      brt_wr_taken_cal_v_i;
logic                      brt_wr_taken_pre_i;
//
logic                      brt_commit;            // Send Commit signal to BRT and wait "i_brt_commit_ready" signal
logic [(ROBSIZE-1):0]      brt_commit_rob_addr;   // BRT Commit ROB addr
logic                      brt_commit_done;       // BRT Instruction Commit is DONE.  
logic                      brt_commit_miss_pre;   // 1: BRT instruction target is miss-predicted 
logic [31:0]               brt_commit_cal_target; // Correct Calculated Target address for BRT instruction. 
//=====================================
// Update Target Calculated.    
//=====================================
logic                      brt_target_ready_i; 
logic  [31:0]              brt_target_cal_i;
logic [ROBSIZE-1:0]        brt_target_rob_addr_i;
logic                      brt_read_btc_o; 
//=====================================
// Update Taken Calculated    
//=====================================
logic                      brt_taken_ready_i; 
logic                      brt_taken_cal_i;
logic [ROBSIZE-1:0]        brt_taken_rob_addr_i;
logic                      brt_read_jeu_o; 
//=====================================
// COMMIT INTERFACE    
//=====================================
logic                      brt_commit_en_i; 
logic [ROBSIZE-1:0]        brt_commit_rob_addr_i; 
logic                      brt_commit_ready_o; 
logic [31:0]               brt_commit_target_o; 
logic [31:0]               brt_commit_taken_o; 
logic                      brt_commit_target_correct_o;       
logic                      brt_commit_taken_correct_o;       
//=====================================
// STATUS SIGNALS     
//=====================================
logic                      brt_full_o;
//=====================================
// JUMP EXECUTION UNIT     
//=====================================
localparam JEUSIZE = 3; 
logic                      jeu_flush_i; 
//===============================
// ISSUE Interface 
//===============================
logic                      jeu_wr_en_i; 
logic [ROBSIZE-1:0]        jeu_wr_rob_addr_i;      
//
logic [31:0]               jeu_wr_op1_i;  
logic                      jeu_wr_op1_v_i; 
logic [ROBSIZE-1:0]        jeu_wr_op1_rob_addr_i; 
//                
logic [31:0]               jeu_wr_op2_i;  
logic                      jeu_wr_op2_v_i;  
logic [ROBSIZE-1:0]        jeu_wr_op2_rob_addr_i;     
//
logic [2:0]                jeu_wr_funct3_i;    
//===============================
// Broadcast Interface 
//===============================
logic                      jeu_broadcast_i; 
logic [ROBSIZE-1:0]        jeu_broadcast_rob_addr_i; 
logic [31:0]               jeu_broadcast_data_i; 
//===============================
// JEU FIFO INTERFACE 
//===============================
logic                      jeu_fifo_emtpy_o; 
logic                      jeu_fifo_taken_cal_o; 
logic [ROBSIZE-1:0]        jeu_fifo_rob_addr_o;   
logic                      jeu_fifo_rd_en_i;  
//===============================
// JEU TABLE SIGNAL
//===============================
logic                      jeu_full_o; 
//
logic                      btb_write_en;
logic [31:0]               btb_write_pc; 
logic [31:0]               btb_write_target;
logic                      btb_write_is_ret;
logic                      bht_write_en; 
logic [31:0]               bht_write_pc; 
logic                      bht_write_taken;   


//===============================
//  CORE CONTROL 
//===============================
Core_Control_Unit #(.ROBSIZE (ROBSIZE)) uCore_Control_Unit(
 .start                     (start),
 .clk                       (clk), 
 .rstn                      (rstn),
 // CONTROL SIGNALS 
 .rd_inst                   (rd_inst),
 .i_alu_m_inst              (alu_m_inst),
 .i_illegal_inst            (illegal_inst),
 .i_cont_tra_inst           (cont_tra_inst),
 .i_load_store              (load_store),
 .i_alu_opcode              (alu_opcode),
 .o_pc_stall                (pc_stall),
 // INSTRUCTION QUEUE SIGNALS 
 .i_iq_empty                (iq_empty),
 .i_iq_full                 (iq_full),
 .o_iq_rd_en                (iq_rd_en), 
 // RESERVATION STATION SIGNALS 
 .i_rs1_full                (rs1_full),
 .i_rs2_full                (rs2_full), 
 .i_alu_m_rs_full           (alu_m_rs_full),
 .o_rs1_we                  (rs1_issue_we),
 .o_rs2_we                  (rs2_issue_we),
 .o_alu_m_rs_we             (alu_m_rs_issue_we),               
 // ROB SIGNALS 
 .i_rob_full                (rob_full),
 .o_rob_we                  (rob_we),
 // Branch Target Calculator Signals. 
 .i_btc_full_o              (btc_full_o),
 .o_btc_wr_en_i             (btc_wr_en_i),
 // JUMP EXECUTION UNIT SIGNALS 
 .o_jeu_wr_en               (jeu_wr_en_i),
 .i_jeu_full                (jeu_full_o),
 // BRT TABKE SIGNALS 
 .o_brt_wr_en_i             (brt_wr_en_i),
 .i_brt_full_o              (brt_full_o), 
 // RAT SIGNALS 
 .o_rat_issue_we            (rat_issue_we), 
 // ALU_CONTROL1 BROADCAST SIGNALS
 .i_alu1_broadcast_ready           (alu1_broadcast_ready),
 .i_alu1_broadcast_data            (alu1_broadcast_data),
 .i_alu1_broadcast_rob_addr        (alu1_broadcast_rob_addr),
 .i_alu1_broadcast_addr_cal        (alu1_broadcast_addr_cal),
 .i_alu1_broadcast_con_branch_comp (alu1_broadcast_con_branch_comp),
 .o_alu1_broadcast_en              (alu1_broadcast_en),
 // ALU_CONTROL2 BROADCAST SIGNALS
 .i_alu2_broadcast_ready           (alu2_broadcast_ready),
 .i_alu2_broadcast_data            (alu2_broadcast_data), 
 .i_alu2_broadcast_rob_addr        (alu2_broadcast_rob_addr),
 .i_alu2_broadcast_addr_cal        (alu2_broadcast_addr_cal),
 .i_alu2_broadcast_con_branch_comp (alu2_broadcast_con_branch_comp), 
 .o_alu2_broadcast_en              (alu2_broadcast_en),
  // MUL_CONTROL BROADCAST SIGNALS  
 .i_alu_m_broadcast_ready            (alu_m_broadcast_ready),
 .i_alu_m_broadcast_data             (alu_m_broadcast_data), 
 .i_alu_m_broadcast_rob_addr         (alu_m_broadcast_rob_addr),
 .o_alu_m_broadcast_en               (alu_m_broadcast_en),
 //DATA MEMORY BROADCAST SIGNALS 
 .o_mem_broadcast_en          (mem_broadcast_en),       // Broadcast Enable 
 .i_mem_broadcast_ready       (mem_broadcast_ready),    // Broadcast Data Ready 
 .i_mem_broadcast_rob_addr    (mem_broadcast_rob_addr), // Broadcast ROB Addr 
 .i_mem_broadcast_load_data   (mem_broadcast_load_data),// Broadcast Data 
 // BROADCAST BUS SIGNALS 
 .o_broadcast_en              (broadcast_en),
 .o_broadcast_data            (broadcast_data),
 .o_broadcast_rob_addr        (broadcast_rob_addr),
 .o_broadcast_addr_cal        (broadcast_addr_cal),
 .o_broadcast_con_branch_comp (broadcast_con_branch_comp),
   // COMMIT SIGNALS 
 .i_commit_ready            (rob_commit_ready),
 .i_commit_rob_addr         (rob_commit_rob_addr),
 .i_commit_inst_pc          (rob_commit_inst_pc),
 .i_commit_value            (rob_commit_value),
 .i_commit_rd               (rob_commit_rd),
 .i_commit_exception        (rob_commit_exception),
 .i_commit_load_store       (rob_commit_load_store),
 .i_commit_rd_inst          (rob_commit_rd_inst), 
 .i_commit_cont_tra_inst    (rob_commit_cont_tra_inst),
 .o_commit                  (commit_en),
 .o_commit_rob_addr         (commit_rob_addr),
 .o_commit_inst_pc          (commit_inst_pc),
 .o_commit_value            (commit_value),
 .o_commit_rd               (commit_rd),
 .o_commit_exception        (commit_exception),
 .o_commit_rd_inst          (commit_rd_inst),
 .o_commit_cont_tra_inst    (commit_cont_tra_inst),
 .o_commit_load_store       (commit_load_store),
 // BRT SIGNALS 
 .o_brt_commit              (brt_commit),            
 .o_brt_commit_rob_addr     (brt_commit_rob_addr),   
 .i_brt_commit_done         (brt_commit_done),       
 .i_brt_commit_miss_pre     (brt_commit_miss_pre),   
 .i_brt_commit_cal_target   (brt_commit_cal_target),  
 // FLUSH
 .o_flush                   (flush),
 .o_flush_pc                (flush_pc),
 .o_flush_taken             (flush_taken),  
   // COMMIT SIGNALS 
  .o_lsq_commit_en          (lsq_commit_en),
  .i_lsq_commit_ready       (lsq_commit_ready),
  .i_lsq_commit_load_store  (lsq_commit_load_store),
  .i_lsq_commit_data        (lsq_commit_data), 
  .i_lsq_commit_rob_addr    (lsq_commit_rob_addr),
  .i_mem_busy               (data_mem_busy),
 // REGISTER FILE WRITE SIGNALS 
 .o_rf_we                   (rf_we),
 .o_rf_wrd                  (rf_rd),
 .o_rf_rd_wvalue            (rf_rd_value),
 // LOAD/STORE QUEUE SIGNALS
 .i_lsq_full                (lsq_full),
 .o_lsq_new_inst_we         (lsq_new_inst_we)
    );

//===============================
//  Instruction Fetch Unit 
//===============================
INSTRUCTION_FETCH_UNIT2
#(.PC_BOOT         (PC_BOOT       ),
  .ROBSIZE         (ROBSIZE       ),
  .VASIZE          (VASIZE        ),
  .PASIZE          (PASIZE        ),
  .PAGEOFFSETSIZE  (PAGEOFFSETSIZE),
  .L1OFFSETSIZE    (L1OFFSETSIZE  ),
  .L1INDEXSIZE     (L1INDEXSIZE   ),
  .L1WAYSIZE       (L1WAYSIZE     ),
  .L2INDEXSIZE     (L2INDEXSIZE   ),
  .L2WAYSIZE       (L2WAYSIZE     ),
  .TLBINDEXSIZE    (TLBINDEXSIZE  ),
  .PTENTRYSIZE     (PTENTRYSIZE   ))
uINSTRUCTION_FETCH_UNIT2
(
    .clk              (clk), 
    .rstn             (rstn), 
    .i_start          (start),
    .i_flush          (flush),       
    .i_flush_pc       (flush_pc), 
    .i_iq_full        (iq_full),  
    .o_iq_wrt_en      (iq_wrt_en),  
    .o_iq_inst        (iq_wrt_inst), 
    .o_iq_pc          (iq_pc), 
    .o_iq_taken_pre   (iq_taken_pre),
    .o_iq_target_pre  (iq_target_pre),
    // BranchTargetBuffer UPDATE SIGNALS 
    .i_btb_write_en       (btb_write_en),
    .i_btb_write_pc       (btb_write_pc), 
    .i_btb_write_target   (btb_write_target),
    .i_btb_write_is_ret   (btb_write_is_ret),  
    // Branch History Table Update Signals 
    .i_bht_write_en       (bht_write_en), 
    .i_bht_write_pc       (bht_write_pc),  
    .i_bht_write_taken    (bht_write_taken)     
    );


//===============================
//  INSTRUCTION QUEUE 
//===============================
 INSTRUCTION_QUEUE 
 #(.DATAWITHBIT (DATAWITHBIT), .FIFOSIZE (FIFOSIZE))
 uINSTRUCTION_QUEUE 
 ( 
   .clk           (clk), 
   .rstn          (rstn), 
   //
   .i_flush       (flush), 
   // Instruction Fetch Unit to Instruction Queue Signals 
   .i_wrt_en      (iq_wrt_en), 
   .i_wrt_data    (iq_wrt_inst), 
   .i_wrt_inst_pc (iq_pc),
   .i_wrt_taken   (iq_taken_pre), 
   .i_wrt_target  (iq_target_pre),
   //  
   .i_rd_en       (iq_rd_en), 
   //
   .o_rd_data     (inst), 
   .o_rd_inst_pc  (inst_pc),
   .o_rd_taken    (iq_taken_pre_out),
   .o_rd_target   (iq_target_pre_out),
   .o_full        (iq_full), 
   .o_empty       (iq_empty) 
   );
   

//===============================
//  DECODE STAGE   
//===============================

assign opcode = inst[6:0];
assign rd     = inst[11:7];
assign funct3 = inst[14:12];
assign rs1    = inst[19:15];
assign rs2    = inst[24:20];
assign funct7 = inst[31:25];

assign pc_p_4 = inst_pc + 4;

//===============================
//  DECODER 
//===============================
DECODER  uDECODER(
    .OPCODE               (opcode),
    .FUNCT7_5             (funct7[5]),
    .FUNCT7               (funct7),
    .FUNCT3               (funct3),
    .IADDER_OUT_1_TO_0    ('0),
    .TRAP_TAKEN           (1'b0),
     //
    .ALU_OPCODE           (alu_opcode),
    .ALU_M_OPCODE         (alu_m_rs_issue_opcode),
    .ALU_M_INST           (alu_m_inst),
    .MEM_WR_REQ           (  ),
    .LOAD_SIZE            (load_size),
    .LOAD_UNSIGNED        (load_unsigned),
    .LOAD_STORE           (load_store),
    .ALU_SRC              (  ),
    .IADDER_SRC           (  ),
    .CSR_WR_EN            (  ),
    .RD_INST              (rd_inst),
    .WB_MUX_SEL           (  ),
    .IMM_TYPE             (imm_type),
    .CSR_OP               (  ),
    .ILLEGAL_INSTR        (illegal_inst),
    .MISALIGNED_LOAD      (  ),
    .MISALIGNED_STORE     (  ),
    .CONT_TRA_INST        (cont_tra_inst)
    );

assign imm_inst = |imm_type; 

//===============================
//  IMMEDIATE GENERATION (IG) 
//===============================
IMM_GEN uIMM_GEN(
   .i_inst     (inst), 
   .i_imm_type (imm_type),
   .o_imm      (imm)
    );


//===============================
//  REGISTER FILE (RF)
//===============================
REGISTER_FILE uREGISTER_FILE(
  .clk             (clk),
  .rstn            (rstn),
  .i_we            (rf_we),
  .i_rd            (rf_rd),
  .i_rd_Value      (rf_rd_value),
  //
  .i_rs1           (rs1),
  .i_rs2           (rs2),
  .o_rs1_RF_Value  (rs1_rf_value),
  .o_rs2_RF_Value  (rs2_rf_value)
    );
    
    
//===============================
//  REGISTER ALLIASING TABLE (RAT) 
//===============================
RAT
#( .ROBSIZE (ROBSIZE))
uRAT
(
    .clk                (clk),
    .rstn               (rstn),
    .i_flush            (flush),
    // RAT ISSUE WRITE SIGNALS 
    .i_we               (rat_issue_we),
    .i_rd               (rd),
    .i_rob_addr         (rob_issue_ptr),
    .i_rob_or_rf        (1'b1),
    // RAT READ RS1 & RS2 values 
    .i_rs1              (rs1),
    .i_rs2              (rs2),
    .o_rs1_rob_addr     (rat_rs1_rob_addr),
    .o_rs2_rob_addr     (rat_rs2_rob_addr),
    .o_rs1_rob_or_rf    (rat_rs1_rob_or_rf),
    .o_rs2_rob_or_rf    (rat_rs2_rob_or_rf),
    // COMMIT SIGNALS 
    .i_commit_en        (commit_en),
    .i_commit_rd        (commit_rd),
    .i_commit_rob_addr  (commit_rob_addr) 
    );
    
          
//===============================
//  RE-ORDER BUFFER (ROB)
//===============================
ROB
#( .ROBSIZE (ROBSIZE))
uROB
(
     .clk                      (clk),
     .rstn                     (rstn),
     .i_flush                  (flush),
     // Write NEW OP to ROB
     .i_issue_we               (rob_we),
     .i_issue_inst_pc          (inst_pc),
     .i_issue_taken            (iq_taken_pre_out),
     .i_issue_rd               (rd),
     .i_issue_load_store       (load_store),
     .i_issue_rd_inst          (rd_inst),
     .i_issue_cont_tra_inst    (cont_tra_inst),
     .i_issue_pc_p_4           (pc_p_4),     
     .o_issue_ptr              (rob_issue_ptr),
     // READ RS1 RS2 ROB VALUES 
     .i_rs1_ROB_Addr           (rat_rs1_rob_addr),
     .i_rs2_ROB_Addr           (rat_rs2_rob_addr),
     .o_ROB_rs1_Value          (rob_rs1_value),
     .o_ROB_rs2_Value          (rob_rs2_value),
     .o_ROB_rs1_Valid          (rob_rs1_valid),
     .o_ROB_rs2_Valid          (rob_rs2_valid),
     // BROADCAST SIGNALS 
     .i_broadcast              (broadcast_en),
     .i_broadcast_rob_addr     (broadcast_rob_addr),
     .i_broadcast_data         (broadcast_data),
     // COMMIT SIGNALS 
     .o_commit_ready           (rob_commit_ready),
     .o_commit_rob_addr        (rob_commit_rob_addr),
     .o_commit_inst_pc         (rob_commit_inst_pc),
     .o_commit_value           (rob_commit_value),
     .o_commit_rd              (rob_commit_rd),
     .o_commit_exception       (rob_commit_exception),
     .o_commit_load_store      (rob_commit_load_store),
     .o_commit_rd_inst         (rob_commit_rd_inst),
     .o_commit_cont_tra_inst   (rob_commit_cont_tra_inst),
     .i_commit                 (commit_en),
     // ROB FULL FLAG 
     .o_rob_full               (rob_full)
    );

//===============================
//  SELECT RS1 & RS2 VALUES 
//===============================
// Conditional Branch instructions have imm value but ALU compares RS1 and RS2 Values.  
assign rs_rs1_value = rat_rs1_rob_or_rf ? rob_rs1_value : rs1_rf_value; 
assign rs_rs2_value = (imm_inst & ~alu_opcode[4]) ? imm : (rat_rs2_rob_or_rf ? rob_rs2_value : rs2_rf_value); 

assign rs_rs1_valid = ~rat_rs1_rob_or_rf | (rat_rs1_rob_or_rf & rob_rs1_valid); 
assign rs_rs2_valid = (imm_inst & ~alu_opcode[4]) | (~rat_rs2_rob_or_rf | (rat_rs2_rob_or_rf & rob_rs2_valid)); 


//assign CONT_TRA_INST[0] = is_branch;
//assign CONT_TRA_INST[1] = is_jal;
//assign CONT_TRA_INST[2] = is_jalr;
//cont_tra_inst
// btc_wr_op1_i   --> Defined Below.
always_comb
begin
   btc_flush_i              = flush;
   btc_wr_rob_addr_i        = rob_issue_ptr;
   btc_broadcast_i          = broadcast_en;
   btc_broadcast_rob_addr_i = broadcast_rob_addr;
   btc_broadcast_data_i     = broadcast_data;
   //btc_wr_en_i --> Generated By the Control Unit 
   case(cont_tra_inst)
      3'b100: // JALR 
      begin 
         btc_wr_op1_i          = rs_rs1_value; 
         btc_wr_op1_rob_addr_i = rat_rs1_rob_addr;
         btc_wr_op1_v_i        = rs_rs1_valid;
         btc_wr_op2_i          = imm;
         btc_wr_op2_v_i        = 1'b1;  
      end 
      3'b010: // JAL  
      begin 
         btc_wr_op1_i          = inst_pc; 
         btc_wr_op1_rob_addr_i = '0;
         btc_wr_op1_v_i        = 1'b1;
         btc_wr_op2_i          = imm;
         btc_wr_op2_v_i        = 1'b1;  
      end 
      3'b001: // CONDITIONAL BRANCH  
      begin   
         btc_wr_op1_i          = inst_pc; 
         btc_wr_op1_rob_addr_i = '0;
         btc_wr_op1_v_i        = 1'b1;
         btc_wr_op2_i          = imm;
         btc_wr_op2_v_i        = 1'b1;  
      end 
      default: 
      begin  
         btc_wr_op1_i          = '0; 
         btc_wr_op1_rob_addr_i = '0;
         btc_wr_op1_v_i        = '0;
         btc_wr_op2_i          = '0;
         btc_wr_op2_v_i        = '0;  
      end 
   endcase
end 



//==========================================
//   BRANCH TARGET CALCULATOR (BRT)
//==========================================
BRANCH_TARGET_CALCULATOR
#( .BTCSIZE (BTCSIZE),
   .ROBSIZE (ROBSIZE)   )
uBRANCH_TARGET_CALCULATOR
(
   .clk                           (clk),
   .rstn                          (rstn),
   //===============================
   // Flush Command 
   //===============================
   .i_btc_flush                   (btc_flush_i),
   //===============================
   // Write Interface 
   //===============================
   .i_btc_wr_en                   (btc_wr_en_i          ),
   .i_btc_wr_rob_addr             (btc_wr_rob_addr_i    ),
   .i_btc_wr_op1                  (btc_wr_op1_i         ),
   .i_btc_wr_op1_rob_addr         (btc_wr_op1_rob_addr_i),   
   .i_btc_wr_op1_v                (btc_wr_op1_v_i       ),
   .i_btc_wr_op2                  (btc_wr_op2_i         ),
   .i_btc_wr_op2_v                (btc_wr_op2_v_i       ),
   //===============================
   // Broadcast Interface 
   //===============================
   // Catch Broadcast-Compare ROB of op1- Update Values      
   .i_btc_broadcast               (btc_broadcast_i          ),
   .i_btc_broadcast_rob_addr      (btc_broadcast_rob_addr_i ),
   .i_btc_broadcast_data          (btc_broadcast_data_i     ),
   //===============================
   // BRT Table Interface 
   //===============================
   .o_btc_fifo_emtpy              (btc_fifo_emtpy_o),
   .o_btc_fifo_target_cal         (btc_fifo_target_cal_o),
   .o_btc_fifo_rob_addr           (btc_fifo_rob_addr_o), 
   .i_btc_fifo_rd_en              (btc_fifo_rd_en_i),
   .o_btc_full                    (btc_full_o)
);


//========================================
// JUMP EXECUTION UNIT SIGNALS 
//========================================



always_comb
begin
   jeu_flush_i                 = flush; 
   //===============================
   // ISSUE Interface 
   //===============================
   //jeu_wr_en_i                 = '0;  --> This is generated by the Core Control Unit 
   jeu_wr_rob_addr_i           = rob_issue_ptr;      
   jeu_wr_op1_i                = rs_rs1_value;  
   jeu_wr_op1_v_i              = rs_rs1_valid; 
   jeu_wr_op1_rob_addr_i       = rat_rs1_rob_addr; 
   jeu_wr_op2_i                = rs_rs2_value;  
   jeu_wr_op2_v_i              = rs_rs2_valid;  
   jeu_wr_op2_rob_addr_i       = rat_rs2_rob_addr;     
   jeu_wr_funct3_i             = funct3;    
   //===============================
   // Broadcast Interface 
   //===============================
   jeu_broadcast_i             = broadcast_en; 
   jeu_broadcast_rob_addr_i    = broadcast_rob_addr; 
   jeu_broadcast_data_i        = broadcast_data; 
   //===============================
   // JEU FIFO INTERFACE 
   //===============================
   //jeu_fifo_emtpy_o    
   //jeu_fifo_taken_cal_o
   //jeu_fifo_rob_addr_o 
   //jeu_fifo_rd_en_i   = '0;  
   //===============================
   // JEU FIFO SIGNALS 
   //===============================
   //jeu_full_o
end 

//========================================
// JUMP EXECUTION UNIT INSTANTIATION 
//========================================
JUMP_EXECUTION_UNIT
#(.JEUSIZE (JEUSIZE),
  .ROBSIZE (ROBSIZE) )
uJUMP_EXECUTION_UNIT
(
   .clk                                (clk),
   .rstn                               (rstn),
         
   .i_jeu_flush                        (jeu_flush_i),
   //===============================    
   // ISSUE Interface                   
   //===============================    
   .i_jeu_wr_en                        (jeu_wr_en_i),
   .i_jeu_wr_rob_addr                  (jeu_wr_rob_addr_i),     
   //                                   
   .i_jeu_wr_op1                       (jeu_wr_op1_i),
   .i_jeu_wr_op1_v                     (jeu_wr_op1_v_i),
   .i_jeu_wr_op1_rob_addr              (jeu_wr_op1_rob_addr_i),
   //                                   
   .i_jeu_wr_op2                       (jeu_wr_op2_i),
   .i_jeu_wr_op2_v                     (jeu_wr_op2_v_i),
   .i_jeu_wr_op2_rob_addr              (jeu_wr_op2_rob_addr_i),   
   //                                   
   .i_jeu_wr_funct3                    (jeu_wr_funct3_i),  
   //===============================    
   // Broadcast Interface               
   //===============================    
   .i_jeu_broadcast                    (jeu_broadcast_i),
   .i_jeu_broadcast_rob_addr           (jeu_broadcast_rob_addr_i),
   .i_jeu_broadcast_data               (jeu_broadcast_data_i),
   //===============================    
   // JEU FIFO INTERFACE                
   //===============================    
   .o_jeu_fifo_emtpy                   (jeu_fifo_emtpy_o),
   .o_jeu_fifo_taken_cal               (jeu_fifo_taken_cal_o),
   .o_jeu_fifo_rob_addr                (jeu_fifo_rob_addr_o), 
   .i_jeu_fifo_rd_en                   (jeu_fifo_rd_en_i),
   //===============================    
   // JEU                               
   //===============================    
   .o_jeu_full                         (jeu_full_o)
    );  


//====================================
// BTC-BRT CONNECTION   
//==================================== 
assign btc_fifo_rd_en_i      = brt_read_btc_o;                         
assign brt_target_ready_i    = ~btc_fifo_emtpy_o; 
assign brt_target_cal_i      = btc_fifo_target_cal_o;
assign brt_target_rob_addr_i = btc_fifo_rob_addr_o;

//====================================
// JEU-BRT CONNECTION   
//==================================== 
assign jeu_fifo_rd_en_i        = brt_read_jeu_o;
assign brt_taken_ready_i       = ~jeu_fifo_emtpy_o; 
assign brt_taken_cal_i         = jeu_fifo_taken_cal_o;
assign brt_taken_rob_addr_i    = jeu_fifo_rob_addr_o;



//========================================
// BRT TABLE SIGNALS 
//========================================
assign brt_flush_i  = btc_flush_i;
//=====================================
// ISSUE INTERFACE    
//=====================================   
//assign brt_wr_en_i                =     '0;
assign brt_wr_rob_addr_i            = rob_issue_ptr;
assign brt_wr_target_cal_i          = '0;
assign brt_wr_target_cal_v_i        = '0; 
assign brt_wr_target_pre_i          = iq_target_pre_out; // BRING THIS FROM STAGE-2 
assign brt_wr_taken_cal_i           = (cont_tra_inst[1] | cont_tra_inst[2]); // taken_cal is 1 for JAL and JALR instructions  
assign brt_wr_taken_cal_v_i         = (cont_tra_inst[1] | cont_tra_inst[2]); // taken_cal is known and valid at decode stage. 
assign brt_wr_taken_pre_i           = iq_taken_pre_out; 
//=====================================
// Update Taken Calculated-- BRT-JEU INTERFACE    
//=====================================

//=====================================
// COMMIT INTERFACE      
//=====================================
assign brt_commit_en_i              =     '0; 
assign brt_commit_rob_addr_i        =     '0; 



//==========================================
//   BRT TABLE (BRT)
//==========================================
BRT_TABLE
#(.BRTSIZE (BRTSIZE),
  .ROBSIZE (ROBSIZE))
uBRT_TABLE
(
   .clk                              (clk), 
   .rstn                             (rstn), 
   //
   .i_brt_flush                      (brt_flush_i),
   //=====================================
   // ISSUE INTERFACE    
   //=====================================   
   .i_brt_wr_en                      (brt_wr_en_i          ),
   .i_brt_wr_rob_addr                (brt_wr_rob_addr_i    ),
   .i_brt_wr_target_cal              (brt_wr_target_cal_i  ),
   .i_brt_wr_target_cal_v            (brt_wr_target_cal_v_i), 
   .i_brt_wr_target_pre              (brt_wr_target_pre_i  ),
   .i_brt_wr_taken_cal               (brt_wr_taken_cal_i   ), 
   .i_brt_wr_taken_cal_v             (brt_wr_taken_cal_v_i ),
   .i_brt_wr_taken_pre               (brt_wr_taken_pre_i   ), 
   .i_brt_wr_pc_p_4                  (pc_p_4),
   .i_brt_wr_pc                      (inst_pc),
   //=====================================
   // Update Target Calculated.    
   //=====================================
   .i_brt_target_ready               (brt_target_ready_i   ), 
   .i_brt_target_cal                 (brt_target_cal_i     ),
   .i_brt_target_rob_addr            (brt_target_rob_addr_i),
   .o_brt_read_btc                   (brt_read_btc_o       ), 
   //=====================================
   // Update Taken Calculated    
   //=====================================
   .i_brt_taken_ready                (brt_taken_ready_i   ), 
   .i_brt_taken_cal                  (brt_taken_cal_i     ),
   .i_brt_taken_rob_addr             (brt_taken_rob_addr_i),
   .o_brt_read_jeu                   (brt_read_jeu_o      ), 
   //=====================================
   // COMMIT INTERFACE    
   //=====================================
   .i_brt_commit                      (brt_commit),           
   .i_brt_commit_rob_addr             (brt_commit_rob_addr),  
   .o_brt_commit_done                 (brt_commit_done),      
   .o_brt_commit_miss_pre             (brt_commit_miss_pre),  
   .o_brt_commit_cal_target           (brt_commit_cal_target),   
   //=====================================
   // STATUS SIGNALS     
   //=====================================
   .o_brt_full                        (brt_full_o),
   // BTB and BHT UPDATE SIGNALS 
   .o_btb_write_en                    (btb_write_en    ),
   .o_btb_write_pc                    (btb_write_pc    ),
   .o_btb_write_target                (btb_write_target),
   .o_btb_write_is_ret                (btb_write_is_ret),
   .o_bht_write_en                    (bht_write_en    ),
   .o_bht_write_pc                    (bht_write_pc    ), 
   .o_bht_write_taken                 (bht_write_taken )
);


//===============================
//  RESERVATION STATION 1
//===============================
RESERVATION_STATION
#( .RSSIZE (RSSIZE),.ROBSIZE (ROBSIZE))
uRESERVATION_STATION1
(
  .clk                  (clk),
  .rstn                 (rstn),
  .i_flush              (flush),
  //
  .i_issue_we           (rs1_issue_we),
  .i_rob_addr           (rob_issue_ptr),
  .i_alu_opcode         (alu_opcode),
  .i_rs1_value          (rs_rs1_value),
  .i_rs2_value          (rs_rs2_value),
  .i_rs1_valid          (rs_rs1_valid), 
  .i_rs2_valid          (rs_rs2_valid), 
  .i_rs1_rob_addr       (rat_rs1_rob_addr),
  .i_rs2_rob_addr       (rat_rs2_rob_addr),
  .i_addr_cal           (|load_store),
  .i_con_branch_comp    (1'b0),
  //      
  .i_broadcast          (broadcast_en),
  .i_broadcast_rob_addr (broadcast_rob_addr),
  .i_broadcast_data     (broadcast_data),
  //
  .i_alu_busy           (alu1_busy),
  .o_alu_opcode         (alu1_opcode),
  .o_rs1_value          (alu1_rs1_value),
  .o_rs2_value          (alu1_rs2_value), 
  .o_rat_rs1_rob_addr   (alu1_rob_addr), 
  .o_alu_ex_en          (alu1_ex_en),
  .o_addr_cal           (alu1_addr_cal),
  .o_con_branch_comp    (alu1_con_branch_comp),
  //
  .o_rs_full            (rs1_full)
  );

//===============================
//  ALU CONTROL 1 
//===============================
ALU_I_CONTROL_UNIT 
#( .ROBSIZE (ROBSIZE))
uALU_I_CONTROL_UNIT1(
    .clk                  (clk), 
    .rstn                 (rstn),
    //
    .i_flush              (flush),
    //
    .o_busy               (alu1_busy),
    .i_rob_addr           (alu1_rob_addr),
    .i_con_branch_comp    (1'b0),
    .i_addr_cal           (alu1_addr_cal),
    .i_rs1_value          (alu1_rs1_value),
    .i_rs2_value          (alu1_rs2_value),
    .i_alu_opcode         (alu1_opcode),
    .i_ex_en              (alu1_ex_en),   
    //
    .i_broadcast_en       (alu1_broadcast_en),
    .o_broadcast_ready    (alu1_broadcast_ready), 
    .o_broadcast_out      (alu1_broadcast_data),
    .o_broadcast_rob_addr (alu1_broadcast_rob_addr),
    .o_broadcast_addr_cal (alu1_broadcast_addr_cal),
    .o_broadcast_con_branch_comp (alu1_broadcast_con_branch_comp)
    );


//===============================
//  RESERVATION STATION 2
//===============================
RESERVATION_STATION
#( .RSSIZE (RSSIZE),.ROBSIZE (ROBSIZE))
uRESERVATION_STATION2
(
  .clk                  (clk),
  .rstn                 (rstn),
  .i_flush              (flush),
  //
  .i_issue_we           (rs2_issue_we),
  .i_rob_addr           (rob_issue_ptr),
  .i_alu_opcode         (alu_opcode),
  .i_rs1_value          (rs_rs1_value),
  .i_rs2_value          (rs_rs2_value),
  .i_rs1_valid          (rs_rs1_valid), 
  .i_rs2_valid          (rs_rs2_valid), 
  .i_rs1_rob_addr       (rat_rs1_rob_addr),
  .i_rs2_rob_addr       (rat_rs2_rob_addr),
  .i_addr_cal           (|load_store),
  .i_con_branch_comp    (1'b0),
  //      
  .i_broadcast          (broadcast_en),
  .i_broadcast_rob_addr (broadcast_rob_addr),
  .i_broadcast_data     (broadcast_data),
  //
  .i_alu_busy           (alu2_busy),
  .o_alu_opcode         (alu2_opcode),
  .o_rs1_value          (alu2_rs1_value),
  .o_rs2_value          (alu2_rs2_value), 
  .o_rat_rs1_rob_addr   (alu2_rob_addr), 
  .o_alu_ex_en          (alu2_ex_en),
  .o_addr_cal           (alu2_addr_cal),
  .o_con_branch_comp    (alu2_con_branch_comp),
  //
  .o_rs_full            (rs2_full)
  );

//===============================
//  ALU CONTROL 2
//===============================
ALU_I_CONTROL_UNIT 
#( .ROBSIZE (ROBSIZE))
uALU_I_CONTROL_UNIT2(
    .clk                  (clk), 
    .rstn                 (rstn),
    .i_flush              (flush),
    .o_busy               (alu2_busy),
    .i_rob_addr           (alu2_rob_addr),
    .i_con_branch_comp    (1'b0),
    .i_addr_cal           (alu2_addr_cal),
    .i_rs1_value          (alu2_rs1_value),
    .i_rs2_value          (alu2_rs2_value),
    .i_alu_opcode         (alu2_opcode),
    .i_ex_en              (alu2_ex_en),   
    //
    .i_broadcast_en              (alu2_broadcast_en),
    .o_broadcast_ready           (alu2_broadcast_ready), 
    .o_broadcast_out             (alu2_broadcast_data),
    .o_broadcast_rob_addr        (alu2_broadcast_rob_addr),
    .o_broadcast_addr_cal        (alu2_broadcast_addr_cal),
    .o_broadcast_con_branch_comp (alu2_broadcast_con_branch_comp)
    );

//======================================================
// MULTIPLICATION UNIT RESERVATION STATION 
//======================================================
ALU_M_RESERVATION_STATION
#( .RSSIZE (RSSIZE),.ROBSIZE (ROBSIZE))
uMUL_RESERVATION_STATION
(
  .clk                     (clk),
  .rstn                    (rstn),
  .i_flush                 (flush),
  // MUL_RS-ISSUE_LOGIC COM BUS  
  .i_issue_we              (alu_m_rs_issue_we),
  .i_rob_addr              (rob_issue_ptr),
  .i_alu_opcode            (alu_m_rs_issue_opcode),
  .i_rs1_value             (rs_rs1_value),
  .i_rs2_value             (rs_rs2_value),
  .i_rs1_valid             (rs_rs1_valid), 
  .i_rs2_valid             (rs_rs2_valid), 
  .i_rs1_rob_addr          (rat_rs1_rob_addr),
  .i_rs2_rob_addr          (rat_rs2_rob_addr),
  // MUL_RS-BROADCAST_BUS Connection      
  .i_broadcast             (broadcast_en),         // BROADCAST BUS ENABLE SIGNAL  
  .i_broadcast_rob_addr    (broadcast_rob_addr),   // BROADCAST BUS ROB ADDR 
  .i_broadcast_data        (broadcast_data),       // BROADCAST BUS DATA 
  // MUL_RS-MUL_UNIT COM BUS 
  .i_alu_busy              (alu_m_busy),           // Multiplication Unit Busy Flag from Multiplication Unit   
  .o_alu_opcode            (alu_m_opcode),         // Multiplication Operation Opcode to Multiplication Unit       
  .o_rs1_value             (alu_m_rs1_value),        
  .o_rs2_value             (alu_m_rs2_value), 
  .o_rat_rs1_rob_addr      (alu_m_rob_addr), 
  .o_alu_ex_en             (alu_m_ex_en),
  //
  .o_rs_full               (alu_m_rs_full)          // Multiplication Unit Reservation Station Full Flag 
  );

//===============================
//  MULTIPLICATION UNIT CONTROL MODULE
//===============================
MUL_CONTROL_UNIT 
#( .ROBSIZE (ROBSIZE))
uMUL_CONTROL_UNIT(
    .clk                  (clk), 
    .rstn                 (rstn),
    .i_flush              (flush),
    // MUL_RS-MUL_UNIT COM BUS 
    .o_busy               (alu_m_busy),
    .i_rob_addr           (alu_m_rob_addr),
    .i_rs1_value          (alu_m_rs1_value),
    .i_rs2_value          (alu_m_rs2_value),
    .i_alu_m_opcode       (alu_m_opcode),
    .i_ex_en              (alu_m_ex_en),   
    // MUL_UNIT-
    .i_broadcast_en       (alu_m_broadcast_en),
    .o_broadcast_ready    (alu_m_broadcast_ready), 
    .o_broadcast_out      (alu_m_broadcast_data),
    .o_broadcast_rob_addr (alu_m_broadcast_rob_addr)
    );


//===============================
//  LOAD STORE QUEUE INPUT SIGNALS  
//===============================

assign lsq_rs2_data       = rat_rs2_rob_or_rf ? rob_rs2_value : rs2_rf_value;
assign lsq_rs2_rob_addr   = rat_rs2_rob_addr;
assign lsq_rs2_data_valid = (~rat_rs2_rob_or_rf | (rat_rs2_rob_or_rf & rob_rs2_valid));
assign lsq_load_or_store  = load_store[0]; 

//===============================
//  LOAD STORE QUEUE 
//===============================
LOAD_STORE_QUEUE
#(.QUEUESIZE  (LSQSIZE), .ROBSIZE   (ROBSIZE))
uLOAD_STORE_QUEUE
(
    .clk                      (clk),
    .rstn                     (rstn),
    //
    .i_flush                  (flush),
    // NEW INSTRUCTION WRITE SIGNALS 
    .i_new_inst_we            (lsq_new_inst_we),
    .i_rob_addr               (rob_issue_ptr),
    .i_rs2_data               (lsq_rs2_data),
    .i_rs2_rob_addr           (lsq_rs2_rob_addr),
    .i_rs2_data_valid         (lsq_rs2_data_valid),
    .i_load_or_store          (lsq_load_or_store),
    .i_signed_unsigned        (load_unsigned),
    .i_size                   (load_size),
    // BROADCAST BUS CONNECTION
    .i_broadcast_en           (broadcast_en),
    .i_broadcast_data         (broadcast_data),
    .i_broadcast_rob_addr     (broadcast_rob_addr),
    .i_broadcast_addr_flag    (broadcast_addr_cal),
    // COMMIT SIGNALS 
    .i_commit_en              (lsq_commit_en),
    .o_commit_ready           (lsq_commit_ready),
    .o_commit_load_store      (lsq_commit_load_store),
    .o_commit_data            (lsq_commit_data),
    .o_commit_rob_addr        (lsq_commit_rob_addr),    
     //DATA MEMORY SIGNALS 
     //TO MEMORY
    .o_mem_cs                 (data_mem_cs),
    .o_mem_addr               (data_mem_addr),
    .o_mem_rob_addr           (data_mem_rob_addr),
    .o_mem_load_store         (data_mem_load_store),
    .o_mem_store_data         (data_mem_store_data),
    .o_mem_size               (data_mem_size),
    .o_mem_signed_unsigned    (data_mem_signed_unsigned),
    // FROM MEMORY
    .i_load_we                (1'b0),
    .i_mem_busy               (data_mem_busy),
    // 
    .o_full                   (lsq_full),
    .o_empty                  (lsq_empty)   
    );

//===============================
//  DATA MEMORY MANAGEMENT UNIT (DMMU)
//===============================
DMMU
#(.ROBSIZE         (ROBSIZE),
  .VASIZE          (VASIZE),
  .PASIZE          (PASIZE),
  .PAGEOFFSETSIZE  (PAGEOFFSETSIZE),
  .L1OFFSETSIZE    (L1OFFSETSIZE),
  .L1INDEXSIZE     (L1INDEXSIZE),
  .L1WAYSIZE       (L1WAYSIZE),
  .L2INDEXSIZE     (L2INDEXSIZE),
  .L2WAYSIZE       (L2WAYSIZE),
  .TLBINDEXSIZE    (TLBINDEXSIZE) ) // Re-Order Buffer Size  
uDMMU
(
   .clk                    (clk),                     // Clock 
   .rstn                   (rstn),                    // Active-Low Reset  
   .i_cs                   (data_mem_cs),             // CHIP SELECT
   .i_addr                 (data_mem_addr),           // Memory Address 
   .i_rob_addr             (data_mem_rob_addr),       // ROB Addr
   .i_load_store           (data_mem_load_store),     // LOAD OR STORE COMMAND 
   .i_store_data           (data_mem_store_data),     // Store Data 
   .i_funct3               ({data_mem_signed_unsigned,data_mem_size}),
   // 
   .i_broadcast_en         (mem_broadcast_en),       // Broadcast Enable 
   .o_broadcast_ready      (mem_broadcast_ready),    // Broadcast Data Ready 
   .o_broadcast_rob_addr   (mem_broadcast_rob_addr), // Broadcast ROB Addr 
   .o_broadcast_load_data  (mem_broadcast_load_data),// Broadcast Data 
   //
   .o_mem_busy             (data_mem_busy)//Memory Busy Flag   
   
   
    );

endmodule :CORE  





///////////////////////////////////
// CORE TEST BENCH 
///////////////////////////////////
module CORE_tb();

parameter DATAWITHBIT  = 32;
parameter FIFOSIZE     = 32;
parameter ROBSIZE      = 8;
parameter LSQSIZE      = 16;
parameter PROGMEMSIZE  = 10;
//
parameter VASIZE          = 32;
parameter PASIZE          = 20;
parameter PAGEOFFSETSIZE  = 9;
parameter L1OFFSETSIZE    = 6;
parameter L1INDEXSIZE     = 3;
parameter L1WAYSIZE       = 4;
parameter L2INDEXSIZE     = 5;
parameter L2WAYSIZE       = 4;
parameter TLBINDEXSIZE    = 6;
//
localparam L1TAGSIZE        = (PASIZE-L1OFFSETSIZE-L1INDEXSIZE);
localparam L2OFFSETSIZE     =  L1OFFSETSIZE;
localparam L2TAGSIZE        = (PASIZE-L2OFFSETSIZE-L2INDEXSIZE); 
localparam L2ADDRSIZE       =  PASIZE;
localparam VAPAGENUMBERSIZE = (VASIZE - PAGEOFFSETSIZE);
localparam TLBTAGSIZE       = (VAPAGENUMBERSIZE - TLBINDEXSIZE);
localparam TLBDATASIZE      = (PASIZE - PAGEOFFSETSIZE);
// TB SIGNALS 
int block, word;
logic                      clk;
logic                      rstn;
logic [31:0]               ROM_ADDR;
logic                      start;
int                        PC;

////////////////
// Clock Generation 
///////////////
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk; 
end 











//==================
// Instruction Functions 
//==================
task JAL(
  input logic [4:0]  rd,
  input logic [20:0] imm 
);
begin
  localparam [6:0] opcode = 7'b1101111;
  //$display("JAL rd:%d, imm:%d ",rd,imm);
  WRITE({imm[20],imm[10:1],imm[11],imm[19:12],rd,opcode});
end   
endtask



task JALR(
  input logic [4:0]  rd,
  input logic [4:0]  rs1,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b1100111;
  localparam [2:0] funct3 = 3'b000;
  //$display("JAL rd:%d, imm:%d ",rd,imm);
  WRITE({imm[11:0],rs1[4:0],funct3[2:0],rd[4:0],opcode[6:0]});
end   
endtask





//==================
// ADD Instruction 
//==================
task ADD(
  input logic [4:0] rd,
  input logic [4:0] rs1, 
  input logic [4:0] rs2 
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b000;
  localparam [6:0] funct7 = 7'b0000000;
  //$display("ADD rd:%d, rs1:%d, rs2:%d ",rd,rs1,rs2);
  WRITE({funct7,rs2,rs1,funct3,rd,opcode});
end   
endtask


//=========
// ADDI Instruction 
//=========
task ADDI(
  input logic [4:0]  rd,
  input logic [4:0]  rs1, 
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0010011;
  localparam [2:0] funct3 = 3'b000;
  WRITE({imm,rs1,funct3,rd,opcode});
  $display("ADDI rd:%d, rs1:%d, imm:%d --> inst:%h",rd,rs1,imm,{imm,rs1,funct3,rd,opcode});
end   
endtask


//=========
// LOAD LB INSTRUCTION 
//=========
task LB(
  input logic [4:0]  rd,
  input logic [4:0]  rs1,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0000011;
  localparam [2:0] funct3 = 3'b000;
  WRITE({imm,rs1,funct3,rd,opcode});
  //$display("LB rd:%d, rs1:%d, imm:%d ",rd,rs1,imm);
end   
endtask


//=========
// LOAD LH INSTRUCTION 
//=========
task LH(
  input logic [4:0]  rd,
  input logic [4:0]  rs1,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0000011;
  localparam [2:0] funct3 = 3'b001;
  WRITE({imm,rs1,funct3,rd,opcode});
  //$display("LH rd:%d, rs1:%d, imm:%d ",rd,rs1,imm);
end   
endtask

//=========
// LOAD LW INSTRUCTION 
//=========
task LW(
  input logic [4:0]  rd,
  input logic [4:0]  rs1,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0000011;
  localparam [2:0] funct3 = 3'b010;
  WRITE({imm,rs1,funct3,rd,opcode});
  $display("PC=0x%h LW rd:%d, rs1:%d, imm:%d ",PC,rd,rs1,imm);
end   
endtask

//=========
// LOAD LBU INSTRUCTION 
//=========
task LBU(
  input logic [4:0]  rd,
  input logic [4:0]  rs1,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0000011;
  localparam [2:0] funct3 = 3'b100;
  WRITE({imm,rs1,funct3,rd,opcode});
  //$display("LBU rd:%d, rs1:%d, imm:%d ",rd,rs1,imm);
end   
endtask

//=========
// LOAD LHU INSTRUCTION 
//=========
task LHU(
  input logic [4:0]  rd,
  input logic [4:0]  rs1,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0000011;
  localparam [2:0] funct3 = 3'b101;
  WRITE({imm,rs1,funct3,rd,opcode});
  //$display("LHU rd:%d, rs1:%d, imm:%d ",rd,rs1,imm);
end   
endtask

//=========
// LOAD SB INSTRUCTION 
//=========
// imm[11:5] rs2 rs1 000 imm[4:0] 0100011 SB
// imm[11:5] rs2 rs1 001 imm[4:0] 0100011 SH
// imm[11:5] rs2 rs1 010 imm[4:0] 0100011 SW
task SB(
  input logic [4:0]  rs1,
  input logic [4:0]  rs2,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0100011;
  localparam [2:0] funct3 = 3'b000;
  WRITE ({imm[11:5],rs2,rs1,funct3,imm[4:0],opcode});
  //$display("SB rs1:%d, rs2:%d, imm:%d ",rs1,rs2,imm);
end   
endtask

//=========
// LOAD SH INSTRUCTION 
//=========
// imm[11:5] rs2 rs1 000 imm[4:0] 0100011 SB
// imm[11:5] rs2 rs1 001 imm[4:0] 0100011 SH
// imm[11:5] rs2 rs1 010 imm[4:0] 0100011 SW
task SH(
  input logic [4:0]  rs1,
  input logic [4:0]  rs2,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0100011;
  localparam [2:0] funct3 = 3'b001;
  WRITE ({imm[11:5],rs2,rs1,funct3,imm[4:0],opcode});
  //$display("SH rs1:%d, rs2:%d, imm:%d ",rs1,rs2,imm);
end   
endtask

//=========
// LOAD SH INSTRUCTION 
//=========
// imm[11:5] rs2 rs1 000 imm[4:0] 0100011 SB
// imm[11:5] rs2 rs1 001 imm[4:0] 0100011 SH
// imm[11:5] rs2 rs1 010 imm[4:0] 0100011 SW
task SW(
  input logic [4:0]  rs1,
  input logic [4:0]  rs2,
  input logic [11:0] imm 
);
begin
  localparam [6:0] opcode = 7'b0100011;
  localparam [2:0] funct3 = 3'b010;
  WRITE ({imm[11:5],rs2,rs1,funct3,imm[4:0],opcode});
  $display("PC=0x%h SW rs1:%d, rs2:%d, imm:%d   ",PC,rs1,rs2,imm);
end   
endtask


//=========
// BEQ INSTRUCTION 
//=========
task BEQ(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [12:0] imm  
);
begin
  localparam [6:0] opcode = 7'b1100011;
  localparam [2:0] funct3 = 3'b000;
  WRITE ({imm[12],imm[10:5],rs2,rs1,funct3,imm[4:1],imm[11],opcode});
  $display("BEQ rs1:%d, rs2:%d, imm:%b",rs1,rs2,imm);   
end 
endtask 

//=========
// BNE INSTRUCTION 
//=========
task BNE(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [12:0] imm  
);
begin
  localparam [6:0] opcode = 7'b1100011;
  localparam [2:0] funct3 = 3'b001;
  WRITE ({imm[12],imm[10:5],rs2,rs1,funct3,imm[4:1],imm[11],opcode});
  //$display("BNE rs1:%d, rs2:%d, imm:%d",rs1,rs2,imm);   
end 
endtask 

//=========
// BLT INSTRUCTION 
//=========
task BLT(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [12:0] imm  
);
begin
  localparam [6:0] opcode = 7'b1100011;
  localparam [2:0] funct3 = 3'b100;
  WRITE ({imm[12],imm[10:5],rs2,rs1,funct3,imm[4:1],imm[11],opcode});
  //$display("BLT rs1:%d, rs2:%d, imm:%d",rs1,rs2,imm);   
end 
endtask 

//=========
// BLT INSTRUCTION 
//=========
task BGE(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [12:0] imm  
);
begin
  localparam [6:0] opcode = 7'b1100011;
  localparam [2:0] funct3 = 3'b101;
  WRITE ({imm[12],imm[10:5],rs2,rs1,funct3,imm[4:1],imm[11],opcode});
  //$display("BGE rs1:%d, rs2:%d, imm:%d",rs1,rs2,imm);   
end 
endtask 

//=========
// BLTU INSTRUCTION 
//=========
task BLTU(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [12:0] imm  
);
begin
  localparam [6:0] opcode = 7'b1100011;
  localparam [2:0] funct3 = 3'b110;
  WRITE ({imm[12],imm[10:5],rs2,rs1,funct3,imm[4:1],imm[11],opcode});
  //$display("BLTU rs1:%d, rs2:%d, imm:%d",rs1,rs2,imm);   
end 
endtask 

//=========
// BGEU INSTRUCTION 
//=========
task BGEU(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [12:0] imm  
);
begin
  localparam [6:0] opcode = 7'b1100011;
  localparam [2:0] funct3 = 3'b111;
  WRITE ({imm[12],imm[10:5],rs2,rs1,funct3,imm[4:1],imm[11],opcode});
  //$display("BGEU rs1:%d, rs2:%d, imm:%d",rs1,rs2,imm);   
end 
endtask 


//=======================
// MUL
//=======================
task MUL(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b000;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  $display("MUL rd:%d, rs1:%d, rs2:%d  --> 0x%h",rd,rs1,rs2,{funct7,rs2,rs1,funct3,rd,opcode});   
end 
endtask 

//=======================
// MULH
//=======================
task MULH(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd 
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b001;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("MULH rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask 

//=======================
// MULHSU
//=======================
task MULHSU(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b010;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("MULHSU rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask 


//=======================
// MULHU
//=======================
task MULHU(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b011;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("MULHU rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask 

//=======================
// DIV
//=======================
task DIV(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd 
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b100;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("DIV rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask 

//=======================
// DIVU
//=======================
task DIVU(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b101;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("DIVU rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask 

//=======================
// DIVU
//=======================
task REM(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b110;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("REM rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask 

//=======================
// REMU
//=======================
task REMU(
   input logic [4:0]  rs1, 
   input logic [4:0]  rs2,
   input logic [4:0]  rd
);
begin
  localparam [6:0] opcode = 7'b0110011;
  localparam [2:0] funct3 = 3'b111;
  localparam [6:0] funct7 = 7'b0000001;
  WRITE ({funct7,rs2,rs1,funct3,rd,opcode});
  //$display("REMU rd:%d, rs1:%d, rs2:%d",rd,rs1,rs2);   
end 
endtask


//=======================
// INSTRUCTION QUEUE WRITE FUNCTION 
//=======================

task WRITE(
   input logic [31:0] wrt_data
);
begin
   @(posedge clk);    
   uCORE.uINSTRUCTION_FETCH_UNIT2.uEXTERNAL_MEMORY.MEM[block][word] <= wrt_data;
   word = word + 1;
   if(word==4) 
   begin 
      word  = 0; 
      block = block + 1; 
   end  
   PC = PC + 4;    
end 
endtask 


task RESET_IMEM();
begin
      for(int i=0;i<256;i++)
      begin
         for(int j=0;j<256;j++)
         begin
            uCORE.uINSTRUCTION_FETCH_UNIT2.uEXTERNAL_MEMORY.MEM[j][i] = 32'h13;
         end 
      end 
end 
endtask 

////////////////
// RESET Task 
///////////////
task RESET();
begin
   PC    = '0;
   start = '0;
   block = '0;
   word  = '0;
   // Asyncronous Active Low Reset 
   rstn = 1'b1;
   repeat(2) @(posedge clk);
   rstn = 1'b0;
   repeat(2) @(posedge clk);
   rstn = 1'b1; 
   repeat(2) @(posedge clk);   
end 
endtask 

int error;
logic [31:0][31:0] ex_reg_file;


task CHECK_REGISTER(input logic [31:0][31:0] ex_reg_file, input int test_num = 0);
begin
   error = 0;
   for(int i = 0 ;i<32;i++)
   begin
      if(uCORE.uREGISTER_FILE.reg_mem[i] != ex_reg_file[i]) 
      begin 
      $display("Register %d Error. Expected :%d Actual:%d Time: %t ",i,ex_reg_file[i],uCORE.uREGISTER_FILE.reg_mem[i],$time);  
      error = error + 1;
      end 
   end 
   
   $display("*************************************************");
   if(error == 0) begin $display("TEST %d PASS",test_num); end 
   else           begin $display("TEST %d FAIL",test_num); end 
   $display("*************************************************");
end
endtask 






//=============================================
// TEST 1: SELF-CHECK  
//=============================================
task TEST1(input int start_data = '0);
begin
   error = 0;
   start = 1'b0;
   for(int i =0;i<32;i++)
   begin
      ADDI(i,0,(i+start_data));
   end 
   //
   RESET();  
   repeat(10) @(posedge clk);
   start = 1'b1;
   //
   repeat(3000) @(posedge clk);
   //
   // CHECK REGISTER VALUES AFTER THE INSTRUCTIONS ARE DONE. 
   if(uCORE.uREGISTER_FILE.reg_mem[0]!=0)
   begin
      error = error + 1;
      $display("TEST1 ERROR REG:0 Time:%t Actual:%d Expected:0",$time,uCORE.uREGISTER_FILE.reg_mem[0]);
      $display("Register 0 is ALWAYS 0.");
   end 
   
   for(int i =1;i<32;i++)
   begin
      if(uCORE.uREGISTER_FILE.reg_mem[i]!=(i+start_data))
      begin
         error = error + 1;
         $display("TEST1 ERROR REG:%d Time:%t Actual:%d Expected:%d",i,$time,uCORE.uREGISTER_FILE.reg_mem[i],(i+start_data));
      end 
   end  
   //
   $display("**************************************************");
   if(error==0) $display("TEST 1 PASSED");
   else         $display("TEST 1 FAILED. %d ERRORS",error);
   $display("**************************************************");
end 
endtask


//=============================================
// TEST 2: SELF CHECK
//=============================================
//BEQ
task TEST2();
begin
   error = '0;
   //
   ADDI(1,0,33);
   ADDI(2,0,33);
   BEQ( .rs1  (1), .rs2 (2),  .imm (16) );
   ADDI(3,0,33);
   ADDI(4,0,34);
   ADDI(5,0,35);
   ADDI(6,0,36);
   ADDI(7,0,37);
   ADDI(8,0,38);
   //
   //
   RESET();  
   repeat(10) @(posedge clk);
   start = 1'b1;
   repeat(1000) @(posedge clk);
   
   // CHECK THE REGISTERS 
   ex_reg_file[1] = 'd33;  
   ex_reg_file[2] = 'd33;  
   ex_reg_file[6] = 'd36;  
   ex_reg_file[7] = 'd37;  
   ex_reg_file[8] = 'd38;  
   
   CHECK_REGISTER(.ex_reg_file (ex_reg_file),.test_num (2));
   
end 
endtask 



//=============================================
// TEST 3: SELF CHECK
//=============================================
// STORE/LOAD WORD TEST 
task TEST3();
begin
   RESET_IMEM();
   start = '0;
   //
   ADDI(1,0,8);                          // RF[1] = 8 
   ADDI(2,0,5);                          // RF[2] = 5 
   ADDI(3,0,10);                         // RF[3] = 10 
   ADDI(4,0,15);                         // RF[4] = 15 
   ADDI(5,0,20);                         // RF[5] = 20 
   ADDI(6,0,25);                         // RF[6] = 25 
   ADDI(7,0,30);                         // RF[7] = 30 
   ADDI(8,0,35);                         // RF[8] = 35 
   //
   SW( .rs1 (1), .rs2 (2), .imm (0) );   ///  MEM[RF[1]+0] = RF[2]
   SW( .rs1 (1), .rs2 (3), .imm (4) );
   SW( .rs1 (1), .rs2 (4), .imm (8) );
   SW( .rs1 (1), .rs2 (5), .imm (12) );
   SW( .rs1 (1), .rs2 (6), .imm (16) );
   SW( .rs1 (1), .rs2 (7), .imm (20) );
   SW( .rs1 (1), .rs2 (8), .imm (24) );
   //
   LW( .rd  (9),  .rs1 (1),  .imm (0) );  //RF[9] = MEM[RF[1]+0]
   LW( .rd  (10), .rs1 (1),  .imm (4) );
   LW( .rd  (11), .rs1 (1),  .imm (8) );
   LW( .rd  (12), .rs1 (1),  .imm (12) );
   LW( .rd  (13), .rs1 (1),  .imm (16) );
   LW( .rd  (14), .rs1 (1),  .imm (20) );
   LW( .rd  (15), .rs1 (1),  .imm (24) );
   //
   RESET();  
   repeat(10) @(posedge clk);
   start = 1'b1;
   repeat(5000) @(posedge clk);
   // CHECK THE REGISTERS 
   ex_reg_file = '0;
   ex_reg_file[1]  ='d8;
   ex_reg_file[2]  ='d5;
   ex_reg_file[3]  ='d10;
   ex_reg_file[4]  ='d15;
   ex_reg_file[5]  ='d20;
   ex_reg_file[6]  ='d25;
   ex_reg_file[7]  ='d30;
   ex_reg_file[8]  ='d35;
   ex_reg_file[9]  ='d5;
   ex_reg_file[10] ='d10;
   ex_reg_file[11] ='d15;
   ex_reg_file[12] ='d20;
   ex_reg_file[13] ='d25;
   ex_reg_file[14] ='d30;
   ex_reg_file[15] ='d35;
  
   CHECK_REGISTER(.ex_reg_file (ex_reg_file),.test_num (3));
   
   
end 
endtask 



//=============================================
// TEST 4: SELF CHECK
//=============================================
// MULTUPLICATION UNIT TEST 
task TEST4();
begin
   ADDI(1,0,8);
   ADDI(2,0,5);
   MUL( .rs1(1) , .rs2 (2) , .rd(3) );
   MUL( .rs1(3) , .rs2 (3) , .rd(3) );
   MUL( .rs1(3) , .rs2 (3) , .rd(3) );
   /*MULH(  .rs1(1) , .rs2 (2) , .rd(4) );
   MULHSU(.rs1(1) , .rs2 (2) , .rd(5) );
   MULHU( .rs1(1) , .rs2 (2) , .rd(6) );
   DIV(   .rs1(1) , .rs2 (2) , .rd(7) );
   DIVU(  .rs1(1) , .rs2 (2) , .rd(8) );
   REM(   .rs1(1) , .rs2 (2) , .rd(9) );
   REMU(  .rs1(1) , .rs2 (2) , .rd(10) );*/
   RESET();  
   repeat(10)   @(posedge clk);
   start = 1'b1;
   repeat(2000) @(posedge clk);
   // CHECK THE REGISTERS 
   ex_reg_file = '0;
   ex_reg_file[1] = 'd8;
   ex_reg_file[2] = 'd5;
   ex_reg_file[3] = 'd2560000;

   CHECK_REGISTER(.ex_reg_file (ex_reg_file),.test_num (4));
   
end 
endtask 

//=============================================
// TEST 5:
//=============================================
task TEST5();
begin
   ADDI(1 ,0,8);
   ADDI(2 ,0,8);
   ADDI(3 ,0,8);
   ADDI(4 ,0,8);
   ADDI(5 ,0,8);
   ADDI(6 ,0,8);
   ADDI(7 ,0,8);
   ADDI(8 ,0,8);
   ADDI(9 ,0,8);
   ADDI(10,0,8);
   ADDI(11,0,8);
   ADDI(12,0,8);
   ADDI(13,0,8);
   ADDI(14,0,8);
   ADDI(15,0,8);
   ADDI(16,0,8);
   ADDI(17,0,8);
   ADDI(18,0,8);
   ADDI(19,0,8);
   ADDI(20,0,8);
   ADDI(21,0,8);
   ADDI(22,0,8);
   ADDI(23,0,8);
   ADDI(24,0,8);
   ADDI(25,0,8);
   ADDI(26,0,8);
   ADDI(27,0,8);
   ADDI(28,0,8);
   ADDI(29,0,8);
   ADDI(30,0,8);
   ADDI(31,0,8);
   RESET();  
   repeat(10) @(posedge clk);
   start = 1'b1;
   repeat(2000) @(posedge clk);
   // CHECK THE REGISTERS 
   if(uCORE.uREGISTER_FILE.reg_mem[0]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[1]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[2]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[3]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[4]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[5]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[6]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[7]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[8]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[9]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[10] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[11] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[12] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[13] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[14] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[15] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[16] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[17] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[18] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[19] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[20] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[21] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[22] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[23] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[24] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[25] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[26] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[27] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[28] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[29] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[30] !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[31] !='d8)         error++;
   //
   $display("**************************************************");
   if(error==0) $display("TEST 5 PASSED");
   else         $display("TEST 5 FAILED. %d ERRORS",error);
   $display("**************************************************");
end 
endtask 


task TEST6();
begin
   ADDI(1,0,8);   // 0
   ADDI(1,1,8);   // 4
   ADDI(1,1,8);   // 8
   ADDI(1,1,8);   // 12
   ADDI(1,1,8);   // 16
   ADDI(1,1,8);   // 20
   ADDI(1,1,8);   // 24
   ADDI(1,1,8);   // 28
   ADDI(1,1,8);   // 32
   ADDI(1,1,8);   // 36
   ADDI(1,1,8);   // 40
   ADDI(1,1,8);   // 44
   ADDI(1,1,8);   // 48
   ADDI(1,1,8);   // 52
   ADDI(1,1,8);   // 56
   ADDI(1,1,8);   // 60
   ADDI(1,1,8);   // 64
   ADDI(1,1,8);   // 68
   ADDI(1,1,8);   // 72
   ADDI(1,1,8);   // 76
   ADDI(1,1,8);   // 80
   ADDI(1,1,8);   // 84
   ADDI(1,1,8);   // 88
   ADDI(1,1,8);   // 92
   ADDI(1,1,8);   // 96
   ADDI(1,1,8);   // 100
   ADDI(1,1,8);   // 104
   ADDI(1,1,8);   // 108
   ADDI(1,1,8);   // 112
   ADDI(1,1,8);   // 116
   ADDI(1,1,8);   // 120
   RESET();  
   repeat(10) @(posedge clk);
   start = 1'b1;
   repeat(2000) @(posedge clk);
   // CHECK THE REGISTERS 
   if(uCORE.uREGISTER_FILE.reg_mem[0]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[1]  !='d248)       error++;
   if(uCORE.uREGISTER_FILE.reg_mem[2]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[3]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[4]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[5]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[6]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[7]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[8]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[9]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[10] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[11] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[12] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[13] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[14] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[15] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[16] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[17] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[18] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[19] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[20] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[21] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[22] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[23] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[24] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[25] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[26] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[27] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[28] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[29] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[30] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[31] !='d0)         error++;
   //
   $display("**************************************************");
   if(error==0) $display("TEST 6 PASSED");
   else         $display("TEST 6 FAILED. %d ERRORS",error);
   $display("**************************************************");
end 
endtask 



task TEST7();
begin
     JAL(.rd ('d2) , .imm ('d120) );
     JAL(.rd ('d3) , .imm ('d124) );
     JAL(.rd ('d4) , .imm ('d128) );
     JAL(.rd ('d5) , .imm ('d132) );
     JAL(.rd ('d6) , .imm ('d136) );
     JAL(.rd ('d7) , .imm ('d140) );
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
end 
endtask 

 

task TEST8();
begin
     RESET_IMEM();
     BEQ(.rs1('d1) ,.rs2('d1) , .imm ('d120) );
     BEQ(.rs1('d2) ,.rs2('d2) , .imm ('d100) );
     BEQ(.rs1('d3) ,.rs2('d3) , .imm ('d80)  );
     BEQ(.rs1('d4) ,.rs2('d4) , .imm ('d60)  );
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
     
     
end 
endtask 


task TEST9();
begin
     RESET_IMEM();
     BEQ(.rs1('d1) ,.rs2('d1) , .imm ('d12) );  // 0    
     ADDI(1,0,8);                               // 4
     ADDI(2,0,9);                               // 8  
     ADDI(3,0,10);                              // 12
     ADDI(4,0,11);                              // 16
     ADDI(5,0,12);                              // 20
     ADDI(6,0,13);                              // 24
     ADDI(7,0,14);                              // 28
     ADDI(8,0,15);                              // 32
     ADDI(9,0,16);                              // 36
     BEQ(.rs1('d2) ,.rs2('d2) , .imm ('d20) );  // 40
     ADDI(10,0,17);                             // 44
     ADDI(11,0,18);                             // 48  
     ADDI(12,0,19);                             // 52
     ADDI(13,0,20);                             // 56
     ADDI(14,0,21);                             // 60
     ADDI(15,0,22);                             // 64
     ADDI(16,0,23);                             // 68
     ADDI(17,0,24);                             // 72
     ADDI(18,0,25);                             // 76     
     //BEQ(.rs1('d1) ,.rs2('d1) , .imm (13'b11111_1101_1000) );  // 40
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
     
        repeat(2000) @(posedge clk);
   // CHECK THE REGISTERS 
   if(uCORE.uREGISTER_FILE.reg_mem[0]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[1]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[2]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[3]  !='d10)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[4]  !='d11)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[5]  !='d12)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[6]  !='d13)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[7]  !='d14)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[8]  !='d15)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[9]  !='d16)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[10] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[11] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[12] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[13] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[14] !='d21)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[15] !='d22)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[16] !='d23)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[17] !='d24)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[18] !='d25)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[19] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[20] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[21] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[22] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[23] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[24] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[25] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[26] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[27] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[28] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[29] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[30] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[31] !='d0)         error++;     

   $display("**************************************************");   
   if(error==0) $display("TEST 9 PASSED");
   else         $display("TEST 9 FAILED. %d ERRORS",error);
   $display("**************************************************");
      
end 
endtask 



task TEST10();
begin
     RESET_IMEM();  
     ADDI(1,1,8);                                              // 0
     ADDI(2,2,9);                                              // 4
     ADDI(3,3,10);                                             // 8 
     ADDI(4,4,11);                                             // 12
     ADDI(5,5,12);                                             // 16
     ADDI(6,6,13);                                             // 20
     ADDI(7,7,14);                                             // 24
     ADDI(8,8,15);                                             // 28
     ADDI(9,9,16);                                             // 32
     BEQ(.rs1('d2) ,.rs2('d2) , .imm ('d12) );                 // 36
     ADDI(10,10,17);                                           // 40 0
     ADDI(11,11,18);                                           // 44 0 
     ADDI(12,12,19);                                           // 48 
     ADDI(13,13,20);                                           // 52 
     ADDI(14,14,21);                                           // 56   <-- Return Back 
     ADDI(15,18,22);                                           // 60
     ADDI(16,0,22);                                            // 64
     ADDI(17,17,24);                                           // 68
     ADDI(18,18,25);                                           // 72 
     BEQ(.rs1('d15) ,.rs2('d16) , .imm (-20) );                // 76
     ADDI(19,19,26);                                           // 80
     ADDI(20,20,27);                                           // 84      
     ADDI(21,21,28);                                           // 88          
     ADDI(22,22,29);                                           // 92  
     ADDI(23,23,30);                                           // 96           
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
     
        repeat(2000) @(posedge clk);
   // CHECK THE REGISTERS 
   if(uCORE.uREGISTER_FILE.reg_mem[0]  !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[1]  !='d8)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[2]  !='d9)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[3]  !='d10)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[4]  !='d11)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[5]  !='d12)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[6]  !='d13)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[7]  !='d14)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[8]  !='d15)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[9]  !='d16)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[10] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[11] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[12] !='d19)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[13] !='d20)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[14] !='d42)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[15] !='d47)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[16] !='d22)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[17] !='d48)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[18] !='d50)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[19] !='d26)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[20] !='d27)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[21] !='d28)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[22] !='d29)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[23] !='d30)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[24] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[25] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[26] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[27] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[28] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[29] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[30] !='d0)         error++;
   if(uCORE.uREGISTER_FILE.reg_mem[31] !='d0)         error++;     

   $display("**************************************************");   
   if(error==0) $display("TEST 10 PASSED");
   else         $display("TEST 10 FAILED. %d ERRORS",error);
   $display("**************************************************");   
end 
endtask 


task TEST11();
begin
     RESET_IMEM();  
     ADDI(1,1,8);                                              // 0
     ADDI(2,2,9);                                              // 4
     ADDI(3,3,10);                                             // 8 
     ADDI(4,4,11);                                             // 12
     ADDI(5,5,12);                                             // 16
     ADDI(6,6,13);                                             // 20
     ADDI(7,7,14);                                             // 24
     ADDI(8,8,15);                                             // 28
     ADDI(9,9,16);                                             // 32 <-- Return Back 
     BEQ(.rs1('d2) ,.rs2('d2) , .imm ('d12) );                 // 36
     ADDI(10,10,17);                                           // 40 0
     ADDI(11,11,18);                                           // 44 0 
     ADDI(12,12,19);                                           // 48 
     ADDI(13,13,20);                                           // 52 
     ADDI(14,14,21);                                           // 56   
     ADDI(15,18,22);                                           // 60
     ADDI(16,0,22);                                            // 64
     ADDI(17,17,24);                                           // 68
     ADDI(18,18,25);                                           // 72 
     BEQ(.rs1('d15) ,.rs2('d16) , .imm (-44) );                // 76
     ADDI(19,19,26);                                           // 80
     ADDI(20,20,27);                                           // 84      
     ADDI(21,21,28);                                           // 88          
     ADDI(22,22,29);                                           // 92  
     ADDI(23,23,30);                                           // 96           
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
     
        repeat(2000) @(posedge clk);
   // CHECK THE REGISTERS 
   if(uCORE.uREGISTER_FILE.reg_mem[0]  !='d0)    begin $display("ERROR REGISTER TEST11 0");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[1]  !='d8)    begin $display("ERROR REGISTER TEST11 1");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[2]  !='d9)    begin $display("ERROR REGISTER TEST11 2");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[3]  !='d10)   begin $display("ERROR REGISTER TEST11 3");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[4]  !='d11)   begin $display("ERROR REGISTER TEST11 4");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[5]  !='d12)   begin $display("ERROR REGISTER TEST11 5");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[6]  !='d13)   begin $display("ERROR REGISTER TEST11 6");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[7]  !='d14)   begin $display("ERROR REGISTER TEST11 7");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[8]  !='d15)   begin $display("ERROR REGISTER TEST11 8");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[9]  !='d32)   begin $display("ERROR REGISTER TEST11 9");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[10] !='d0)    begin $display("ERROR REGISTER TEST11 10");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[11] !='d0)    begin $display("ERROR REGISTER TEST11 11");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[12] !='d38)   begin $display("ERROR REGISTER TEST11 12");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[13] !='d40)   begin $display("ERROR REGISTER TEST11 13");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[14] !='d42)   begin $display("ERROR REGISTER TEST11 14");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[15] !='d47)   begin $display("ERROR REGISTER TEST11 15");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[16] !='d22)   begin $display("ERROR REGISTER TEST11 16");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[17] !='d48)   begin $display("ERROR REGISTER TEST11 17");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[18] !='d50)   begin $display("ERROR REGISTER TEST11 18");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[19] !='d26)   begin $display("ERROR REGISTER TEST11 19");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[20] !='d27)   begin $display("ERROR REGISTER TEST11 20");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[21] !='d28)   begin $display("ERROR REGISTER TEST11 21");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[22] !='d29)   begin $display("ERROR REGISTER TEST11 22");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[23] !='d30)   begin $display("ERROR REGISTER TEST11 23");    error++;     end 
   if(uCORE.uREGISTER_FILE.reg_mem[24] !='d0)    begin $display("ERROR REGISTER TEST11 24");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[25] !='d0)    begin $display("ERROR REGISTER TEST11 25");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[26] !='d0)    begin $display("ERROR REGISTER TEST11 26");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[27] !='d0)    begin $display("ERROR REGISTER TEST11 27");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[28] !='d0)    begin $display("ERROR REGISTER TEST11 28");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[29] !='d0)    begin $display("ERROR REGISTER TEST11 29");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[30] !='d0)    begin $display("ERROR REGISTER TEST11 30");   error++;      end 
   if(uCORE.uREGISTER_FILE.reg_mem[31] !='d0)    begin $display("ERROR REGISTER TEST11 31");   error++;    	 end 	 
   
   
   $display("**************************************************");
   if(error==0) $display("TEST 11 PASSED");
   else         $display("TEST 11 FAILED. %d ERRORS",error);
   $display("**************************************************");
end 
endtask 





task TEST12();
begin
     RESET_IMEM();  
     ADDI(1,1,8);                                              // 0
     ADDI(2,2,9);                                              // 4
     ADDI(3,3,10);                                             // 8 
     ADDI(4,4,11);                                             // 12
     ADDI(5,5,12);                                             // 16
     ADDI(6,6,13);                                             // 20
     ADDI(7,7,14);                                             // 24
     ADDI(8,8,15);                                             // 28
     ADDI(9,9,16);                                             // 32  <-- BEQ returns here 
     JAL(.rd (31),.imm (16));
     ADDI(10,10,17);                                           // 40 
     ADDI(11,11,18);                                           // 44 
     ADDI(12,12,19);                                           // 48 
     ADDI(13,13,20);                                           // 52 
     ADDI(14,14,21);                                           // 56   
     ADDI(15,18,22);                                           // 60
     ADDI(16,0,22);                                            // 64
     ADDI(17,17,24);                                           // 68
     ADDI(18,18,25);                                           // 72 
     BEQ(.rs1('d15) ,.rs2('d16) , .imm (-44) );                // 76
     ADDI(19,19,26);                                           // 80
     ADDI(20,20,27);                                           // 84      
     ADDI(21,21,28);                                           // 88          
     ADDI(22,22,29);                                           // 92  
     ADDI(23,23,30);                                           // 96           
     ADDI(24,24,31);                                           // 96           
     ADDI(25,25,32);                                           // 96           
     ADDI(26,26,33);                                           // 96           
     ADDI(27,27,34);                                           // 96           
     ADDI(28,28,35);                                           // 96 
       
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
     repeat(2000) @(posedge clk);

      // CHECK THE REGISTERS 
      ex_reg_file = '0;
      ex_reg_file[1]  ='d8;
      ex_reg_file[2]  ='d9;
      ex_reg_file[3]  ='d10;
      ex_reg_file[4]  ='d11;
      ex_reg_file[5]  ='d12;
      ex_reg_file[6]  ='d13;
      ex_reg_file[7]  ='d14;
      ex_reg_file[8]  ='d15;
      ex_reg_file[9]  ='d32;
      ex_reg_file[10] ='d0;
      ex_reg_file[11] ='d0;
      ex_reg_file[12] ='d0;
      ex_reg_file[13] ='d40;
      ex_reg_file[14] ='d42;
      ex_reg_file[15] ='d47;
      ex_reg_file[16] ='d22;
      ex_reg_file[17] ='d48;
      ex_reg_file[18] ='d50;
      ex_reg_file[19] ='d26;
      ex_reg_file[20] ='d27;
      ex_reg_file[21] ='d28;
      ex_reg_file[22] ='d29;
      ex_reg_file[23] ='d30;
      ex_reg_file[24] ='d31;
      ex_reg_file[25] ='d32;
      ex_reg_file[26] ='d33;
      ex_reg_file[27] ='d34;
      ex_reg_file[28] ='d35;
      ex_reg_file[31] ='d40; // PC + 4 for the JAL instruction
      
      
      CHECK_REGISTER(.ex_reg_file (ex_reg_file),.test_num (12));  
end 
endtask 



task TEST13();
begin
     RESET_IMEM();  
     ADDI(1,1,8);                                              // 0
     ADDI(2,2,9);                                              // 4
     ADDI(3,3,10);                                             // 8 
     ADDI(4,4,11);                                             // 12
     ADDI(5,5,12);                                             // 16
     ADDI(6,6,13);                                             // 20
     ADDI(7,7,14);                                             // 24
     ADDI(8,8,15);                                             // 28
     ADDI(9,9,16);                                             // 32  <-- BEQ returns here 
     JALR(.rd (31),.rs1(5), .imm(48) );
     ADDI(10,10,17);                                           // 40 
     ADDI(11,11,18);                                           // 44 
     ADDI(12,12,19);                                           // 48 
     ADDI(13,13,20);                                           // 52 
     ADDI(14,14,21);                                           // 56   
     ADDI(15,18,22);                                           // 60
     ADDI(16,0,22);                                            // 64
     ADDI(17,17,24);                                           // 68
     ADDI(18,18,25);                                           // 72 
     BEQ(.rs1('d15) ,.rs2('d16) , .imm (-44) );                // 76
     ADDI(19,19,26);                                           // 80
     ADDI(20,20,27);                                           // 84      
     ADDI(21,21,28);                                           // 88          
     ADDI(22,22,29);                                           // 92  
     ADDI(23,23,30);                                           // 96           
     ADDI(24,24,31);                                           // 96           
     ADDI(25,25,32);                                           // 96           
     ADDI(26,26,33);                                           // 96           
     ADDI(27,27,34);                                           // 96           
     ADDI(28,28,35);                                           // 96 
       
     RESET();  
     repeat(10) @(posedge clk);
     start = 1'b1;
     repeat(2000) @(posedge clk);

      // CHECK THE REGISTERS 
      ex_reg_file = '0;
      ex_reg_file[1]  ='d8;
      ex_reg_file[2]  ='d9;
      ex_reg_file[3]  ='d10;
      ex_reg_file[4]  ='d11;
      ex_reg_file[5]  ='d12;
      ex_reg_file[6]  ='d13;
      ex_reg_file[7]  ='d14;
      ex_reg_file[8]  ='d15;
      ex_reg_file[9]  ='d32;
      ex_reg_file[10] ='d0;
      ex_reg_file[11] ='d0;
      ex_reg_file[12] ='d0;
      ex_reg_file[13] ='d0;
      ex_reg_file[14] ='d0;
      ex_reg_file[15] ='d47;
      ex_reg_file[16] ='d22;
      ex_reg_file[17] ='d48;
      ex_reg_file[18] ='d50;
      ex_reg_file[19] ='d26;
      ex_reg_file[20] ='d27;
      ex_reg_file[21] ='d28;
      ex_reg_file[22] ='d29;
      ex_reg_file[23] ='d30;
      ex_reg_file[24] ='d31;
      ex_reg_file[25] ='d32;
      ex_reg_file[26] ='d33;
      ex_reg_file[27] ='d34;
      ex_reg_file[28] ='d35;
      ex_reg_file[31] ='d40; // PC + 4 for the JAL instruction
      
      
      CHECK_REGISTER(.ex_reg_file (ex_reg_file),.test_num (13));  
end 
endtask 













//==========================================
// MAIN STIMULUS 
//==========================================
initial begin
    TEST8();
  //TEST1();//PASS 
  //TEST2();//PASS 
  //TEST3(); //PASS 
  //TEST4(); //PASS
  //TEST5(); //PASS
  //TEST6(); //PASS
  
  //TEST9();   //PASS
  //TEST10();  //PASS 
  //TEST11();  //PASS 
  //TEST12();  //PASS 
  //TEST13();  //PASS   
  repeat(500) @(posedge clk);
  $finish;
end 



//===============================
//  CORE Instantiation  
//===============================
CORE
#( .DATAWITHBIT   (DATAWITHBIT), 
   .FIFOSIZE      (FIFOSIZE), 
   .ROBSIZE       (ROBSIZE),
   .LSQSIZE       (LSQSIZE),
   .PROGMEMSIZE   (PROGMEMSIZE))
uCORE
(
  .start         (start),
  .clk           (clk),
  .rstn          (rstn)
);

endmodule





