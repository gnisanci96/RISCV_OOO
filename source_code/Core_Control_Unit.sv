`timescale 1ns / 1ps

module Core_Control_Unit
#(parameter ROBSIZE = 8,
  parameter LSQSIZE = 16)
(
  input logic                          start,
  input logic                          clk, 
  input logic                          rstn,
  // DECODER SIGNALS 
  input  logic                         rd_inst, // 1: The instruction writes to rd
  input  logic                         i_alu_m_inst,
  input  logic                         i_illegal_inst,
  input  logic [2:0]                   i_cont_tra_inst,
  input  logic [1:0]                   i_load_store,
  input  logic [4:0]                   i_alu_opcode,
  output logic                         o_pc_stall,          
  // INSTRUCTION QUEUE SIGNALS 
  input  logic                         i_iq_empty,
  input  logic                         i_iq_full,
  output logic                         o_iq_rd_en,
  // RESERVATION STATION SIGNALS 
  input  logic                         i_rs1_full,
  input  logic                         i_rs2_full, 
  input  logic                         i_alu_m_rs_full, 
  output logic                         o_rs1_we, 
  output logic                         o_rs2_we, 
  output logic                         o_alu_m_rs_we, 
  // ROB SIGNALS 
  input  logic                         i_rob_full, 
  output logic                         o_rob_we, 
  // BTC SIGNALS 
  input  logic                         i_btc_full_o,
  output logic                         o_btc_wr_en_i,
  input  logic                         i_jeu_full,
  output logic                         o_jeu_wr_en,
  // BRT TABLE SIGNALS 
  input  logic                         i_brt_full_o,
  output logic                         o_brt_wr_en_i,
  // RAT SIGNALS 
  output logic                         o_rat_issue_we, 
  // ALU_CONTROL1 BROADCAST SIGNALS
  input  logic                         i_alu1_broadcast_ready,
  input  logic [31:0]                  i_alu1_broadcast_data,
  input  logic [(ROBSIZE-1):0]         i_alu1_broadcast_rob_addr,
  input  logic                         i_alu1_broadcast_addr_cal,
  input  logic                         i_alu1_broadcast_con_branch_comp, 
  output logic                         o_alu1_broadcast_en,
  // ALU_CONTROL2 BROADCAST SIGNALS
  input  logic                         i_alu2_broadcast_ready,
  input  logic [31:0]                  i_alu2_broadcast_data,
  input  logic [(ROBSIZE-1):0]         i_alu2_broadcast_rob_addr,
  input  logic                         i_alu2_broadcast_addr_cal,
  input  logic                         i_alu2_broadcast_con_branch_comp, 
  output logic                         o_alu2_broadcast_en,
  // MUL_CONTROL BROADCAST SIGNALS 
  input  logic                         i_alu_m_broadcast_ready,
  input  logic [31:0]                  i_alu_m_broadcast_data,
  input  logic [(ROBSIZE-1):0]         i_alu_m_broadcast_rob_addr,
  output logic                         o_alu_m_broadcast_en,
  // DATA MEMORY BROADCAST SIGNALS 
  output logic                         o_mem_broadcast_en,        // Broadcast Enable 
  input  logic                         i_mem_broadcast_ready,     // Broadcast Data Ready 
  input  logic [(ROBSIZE-1):0]         i_mem_broadcast_rob_addr,  // Broadcast ROB Addr 
  input  logic [31:0]                  i_mem_broadcast_load_data, // Broadcast Data 
  // BROADCAST BUS SIGNALS 
  output logic                         o_broadcast_en,
  output logic [31:0]                  o_broadcast_data,  
  output logic [(ROBSIZE-1):0]         o_broadcast_rob_addr,
  output logic                         o_broadcast_addr_cal,
  output logic                         o_broadcast_con_branch_comp, 
  // COMMIT INPUT SIGNALS 
  input   logic                        i_commit_ready,
  input   logic [(ROBSIZE-1):0]        i_commit_rob_addr,
  input   logic [31:0]                 i_commit_inst_pc,
  input   logic [31:0]                 i_commit_value,
  input   logic [4:0]                  i_commit_rd,
  input   logic                        i_commit_exception,
  input   logic                        i_commit_rd_inst, 
  input   logic [2:0]                  i_commit_cont_tra_inst,
  input   logic [1:0]                  i_commit_load_store,
  // COMMIT OUTPUT SIGNALS 
  output  logic                        o_commit,
  output  logic [(ROBSIZE-1):0]        o_commit_rob_addr,
  output  logic [31:0]                 o_commit_inst_pc,
  output  logic [31:0]                 o_commit_value,
  output  logic [4:0]                  o_commit_rd,
  output  logic                        o_commit_exception,
  output  logic                        o_commit_rd_inst, 
  output  logic [2:0]                  o_commit_cont_tra_inst,  
  output  logic [1:0]                  o_commit_load_store,
  // BRT signals 
  output  logic                        o_brt_commit,            // Send Commit signal to BRT and wait "i_brt_commit_ready" signal
  output  logic [(ROBSIZE-1):0]        o_brt_commit_rob_addr,   // BRT Commit ROB addr
  input   logic                        i_brt_commit_done,       // BRT Instruction Commit is DONE.  
  input   logic                        i_brt_commit_miss_pre,   // 1: BRT instruction target is miss-predicted 
  input   logic [31:0]                 i_brt_commit_cal_target, // Correct Calculated Target address for BRT instruction. 
  // FLUSH 
  output  logic                        o_flush,       
  output  logic [31:0]                 o_flush_pc,
  output  logic                        o_flush_taken,                    
  // LSQ COMMIT SIGNALS 
  output logic                         o_lsq_commit_en,
  input  logic                         i_lsq_commit_ready,
  input  logic                         i_lsq_commit_load_store,
  input  logic [31:0]                  i_lsq_commit_data,  
  input  logic [(ROBSIZE-1):0]         i_lsq_commit_rob_addr,
  input  logic                         i_mem_busy,
  // REGISTER FILE WRITE SIGNALS 
  output logic                         o_rf_we,
  output logic [4:0]                   o_rf_wrd,
  output logic [31:0]                  o_rf_rd_wvalue,
  // LOAD/STORE QUEUE SIGNALS 
  input logic                          i_lsq_full,     
  output logic                         o_lsq_new_inst_we
    );


//================
//  Local Signals 
//================
logic wrt_r11_or_r12;
logic rs_issue_we;
logic broadcast_alu_sel, broadcast_alu_sel_reg ;
    
//============================= 
//  PC STALL SIGNAL GENERATION  
//=============================
assign o_pc_stall = i_iq_full;    

  
//===============================   
// Instruction Queue Read Enable Signal Generation    
//===============================         
assign o_iq_rd_en = ~i_rs1_full & ~i_rs2_full &  ~i_rob_full & ~i_iq_empty & ~i_lsq_full & ~i_alu_m_rs_full  & ~i_btc_full_o & ~i_jeu_full & ~i_brt_full_o & ~i_illegal_inst & start;  
    
//================================
// RESERVATION STATION WRITE-ENABLE SIGNAL 
//================================  

//============
// Choose the Reservation Station to write the Instruction 
//============

// RS write enable Generation
// We fetched an instruction that writes to rd OR Store Instruction for Address Calculation AND not an M instruction.
// rs_issue_we writes the ALU instructions. 
// (~|i_cont_tra_inst) -->Dont Write Conditional Branch Instructions to ALU Reservation Station. ROB calculates the PC+4,BTC calculates the target, and JEU performs the comparison. 
assign rs_issue_we       = o_iq_rd_en & (rd_inst | i_load_store[0] ) & ~i_illegal_inst & ~i_alu_m_inst & (~|i_cont_tra_inst);
assign o_lsq_new_inst_we = o_iq_rd_en & |i_load_store & ~i_illegal_inst;
assign o_alu_m_rs_we     = o_iq_rd_en &  i_alu_m_inst & ~i_illegal_inst;
 
// Switch the Reservation Station on every Instruction Read.
// ALU Reservation Station Arbitration. 
 
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      wrt_r11_or_r12 <= '0;
   end else begin
      if(o_iq_rd_en)
      begin
         wrt_r11_or_r12 <= ~wrt_r11_or_r12;
      end 
   end 
end 

assign o_rs1_we =  wrt_r11_or_r12  & rs_issue_we ;
assign o_rs2_we = ~wrt_r11_or_r12  & rs_issue_we ;

//================================
// ROB WRITE ENABLE SIGNAL GENERATION 
//================================  
// Generate a ROB enable signal everytime we fetch an instruction from iq 
assign o_rob_we = o_iq_rd_en & ~i_illegal_inst;

//================================
// BRANCH TARGET CALCULATOR WRITE ENABLE SIGNAL GENERATION 
//================================ 
//input  logic                         i_btc_full_o,
//output logic                         o_btc_wr_en_i,
//i_cont_tra_inst: {is_branch,is_jal,is_jalr}
assign o_btc_wr_en_i = o_iq_rd_en & (|i_cont_tra_inst) & ~i_illegal_inst;
 
 
//================================
// Jump Execution Unit Write Enable Signal Generation 
//================================ 
assign o_jeu_wr_en   = o_iq_rd_en & i_cont_tra_inst[0] & ~i_illegal_inst;     
 
//================================
// BRT TABLE WRITE ENABLE SIGNAL GENERATION 
//================================ 
assign o_brt_wr_en_i = o_btc_wr_en_i;

//================================
// RAT Issue Write-Enable Signal 
//================================  
assign o_rat_issue_we = o_iq_rd_en & rd_inst & ~i_illegal_inst;

//================================
// BROADCAST BUS SIGNAL DRIVE 
//================================ 
// 1: Select ALU1 0: Select ALU2 //Always Select ALU 1 for now 

localparam REQSIZE = 4;
logic [(REQSIZE-1):0] req;
logic [(REQSIZE-1):0] grant;
assign req[0] = i_alu1_broadcast_ready;
assign req[1] = i_alu2_broadcast_ready;
assign req[2] = i_alu_m_broadcast_ready;
assign req[3] = i_mem_broadcast_ready;
//
always_comb
begin
   o_alu1_broadcast_en  = 1'b0;
   o_alu2_broadcast_en  = 1'b0;
   o_alu_m_broadcast_en = 1'b0;
   o_mem_broadcast_en   = 1'b0;
   case(grant)
    'b0001: begin
        o_alu1_broadcast_en  = 1'b1;
        o_broadcast_en       = 1'b1;  
        o_broadcast_data     = i_alu1_broadcast_data; 
        o_broadcast_rob_addr = i_alu1_broadcast_rob_addr; 
        o_broadcast_addr_cal = i_alu1_broadcast_addr_cal;  
        o_broadcast_con_branch_comp = i_alu1_broadcast_con_branch_comp;   
           end 
    'b0010: begin
        o_alu2_broadcast_en  = 1'b1;
        o_broadcast_en       = 1'b1;  
        o_broadcast_data     = i_alu2_broadcast_data; 
        o_broadcast_rob_addr = i_alu2_broadcast_rob_addr; 
        o_broadcast_addr_cal = i_alu2_broadcast_addr_cal;   
        o_broadcast_con_branch_comp = i_alu2_broadcast_con_branch_comp;  
           end 
    'b0100: begin
        o_alu_m_broadcast_en  = 1'b1;
        o_broadcast_en        = 1'b1;  
        o_broadcast_data      = i_alu_m_broadcast_data; 
        o_broadcast_rob_addr  = i_alu_m_broadcast_rob_addr; 
           end 
    'b1000: begin
        o_mem_broadcast_en   = 1'b1;
        o_broadcast_en       = 1'b1; 
        o_broadcast_data     = i_mem_broadcast_load_data;
        o_broadcast_rob_addr = i_mem_broadcast_rob_addr;
        o_broadcast_addr_cal = 1'b0;   
        o_broadcast_con_branch_comp = 1'b0;                 
            end
     default: begin 
        o_alu1_broadcast_en  = '0;
        o_alu2_broadcast_en  = '0;
        o_mem_broadcast_en   = '0;
        o_broadcast_en       = '0;
        o_broadcast_data     = '0;
        o_broadcast_rob_addr = '0;
        o_broadcast_addr_cal = '0;  
        o_broadcast_con_branch_comp = '0;    
              end                        
   endcase 
end 



LSGF_ARBITER
#( .REQSIZE (REQSIZE)) 
uLSGF_ARBITER
  (
    .clk     (clk), 
    .rstn    (rstn), 
    .i_req   (req),
    .o_grant (grant)
);



/*
//================================
// COMMIT SIGNAL GENERATION & EXCEPTION HANDLING 
//================================
always_comb
begin
           o_commit            = '0; 
           o_rf_we             = '0;
           o_lsq_commit_en     = '0;  
           o_flush             = '0;
           //    
           o_commit_rob_addr      = i_commit_rob_addr;  
           o_commit_inst_pc       = i_commit_inst_pc;
           o_commit_value         = i_commit_value;
           o_commit_rd            = i_commit_rd;
           o_commit_exception     = i_commit_exception;
           o_commit_rd_inst       = i_commit_rd_inst; 
           o_commit_cont_tra_inst = i_commit_cont_tra_inst;
           o_commit_load_store    = i_commit_load_store;           
           o_rf_wrd               = i_commit_rd;
           o_rf_rd_wvalue         = '0; 
           //                           
   //====================
   // EXCEPTION HANDLING          
   //====================       
   if(i_commit_exception) begin  
      //====================
      // EXCEPTION HANDLING FOR CONDITIONAL BRANCH INSTRUCTIONS        
      //====================   
      if(i_commit_cont_tra_inst[0])
      begin      
         o_flush       = 1'b1;              // Generate Flush Flag 
         o_flush_pc    = i_commit_inst_pc;  // Set the PC to the PC of the COMMIT BRANCH
         o_flush_taken = i_commit_value[0]; // Take the Calculated Taken-NotTaken Value and Force the PC logic decision.             
      end 
           
   end else begin
   //====================
   // COMMIT SIGNAL GENERATION          
   //====================            
       case(i_commit_load_store)
         2'b00: // NO LOAD|STORE INSTRUCTION
           begin
              o_commit               = i_commit_ready & (~i_commit_exception);  
              //================================
              // Register File Write Signal Generation 
              //================================   
              o_rf_we             = o_commit & i_commit_rd_inst;  
              o_rf_rd_wvalue      = i_commit_value;       
           end 
         2'b01: // STORE INSTRUCTION
           begin 
              if(i_lsq_commit_ready) // Wait for the Instruction to be ready at LSQ
              begin
                 if(~i_mem_busy) // Wait Memory to be Available 
                 begin
                    o_lsq_commit_en = 1'b1;  // LSQ COMMIT ENABLE 
                    //
                    o_commit        = 1'b1;  // ROB/RAT COMMIT ENABLE                
                 end 
              end 
           end 
         2'b10:// LOAD INSTRUCTION  
           begin
              if(i_lsq_commit_ready)
              begin
                 o_lsq_commit_en     = 1'b1;  // LSQ COMMIT ENABLE 
                 //
                 o_commit            = 1'b1;  
                 //================================
                 // Register File Write Signal Generation 
                 //================================   
                 o_rf_we             = 1'b1;
                 o_rf_rd_wvalue      = i_lsq_commit_data;              
              end       
           end 
         2'b11: begin $display("[CONTROL UNIT]-LOAD|STORE ERROR"); end 
       endcase   
   end 
end 
*/
logic [2:0]            commit_cont_tra_inst_next,commit_cont_tra_inst_reg;
logic [31:0]           commit_value_next,commit_value_reg;
logic [4:0]            commit_rd_next,commit_rd_reg;
logic [(ROBSIZE-1):0]  commit_rob_addr_next,commit_rob_addr_reg;

 typedef enum {
   commit_wait,
   commit_brt
 } state_type;  

state_type commit_state,commit_state_n;


always_comb begin
           o_commit            = '0; 
           o_rf_we             = '0;
           o_lsq_commit_en     = '0;  
           o_flush             = '0;
           //    
           o_commit_rob_addr      = i_commit_rob_addr;  
           o_commit_inst_pc       = i_commit_inst_pc;
           o_commit_value         = i_commit_value;
           o_commit_rd            = i_commit_rd;
           o_commit_exception     = i_commit_exception;
           o_commit_rd_inst       = i_commit_rd_inst; 
           o_commit_cont_tra_inst = i_commit_cont_tra_inst;
           o_commit_load_store    = i_commit_load_store;           
           o_rf_wrd               = i_commit_rd;
           o_rf_rd_wvalue         = '0; 
           //       
           o_brt_commit           = '0;
	if(i_commit_ready)begin
	    
	   case(commit_state)
	    
		   commit_wait: begin  
		      
                //===========================================
				// COMMIT SIGNAL GENERATION
		        //===========================================
		        if(i_commit_cont_tra_inst) begin                // CONTROL TRANSFER INSTRUCTION  
				   commit_state_n            = commit_brt;
				   commit_cont_tra_inst_next = i_commit_cont_tra_inst;
				   commit_value_next         = i_commit_value;
				   commit_rd_next            = i_commit_rd;
				   commit_rob_addr_next      = i_commit_rob_addr;
				end else if(i_commit_load_store == 2'b01) begin // STORE INSTRUCTION 
                   //
				   if(i_lsq_commit_ready) // Wait for the Instruction to be ready at LSQ
                   begin
                      if(~i_mem_busy) // Wait Memory to be Available 
                      begin
                         o_lsq_commit_en = 1'b1;  // LSQ COMMIT ENABLE 
                         o_commit        = 1'b1;  // ROB/RAT COMMIT ENABLE                
                      end 
                   end 
				   //
                end else if(i_commit_load_store == 2'b10) begin // LOAD INSTRUCTION
                   //
				   if(i_lsq_commit_ready)
                   begin
                      o_lsq_commit_en     = 1'b1;  // LSQ COMMIT ENABLE 
                      o_commit            = 1'b1;  
                      //================================
                      // Register File Write Signal Generation 
                      //================================   
                      o_rf_we             = 1'b1;
                      o_rf_rd_wvalue      = i_lsq_commit_data;              
                   end  
                   //  
                end else if(i_commit_load_store == 2'b00) begin // ALU INSTRUCTION 				
                   o_commit            = i_commit_ready & (~i_commit_exception);  
                   //================================
                   // Register File Write Signal Generation 
                   //================================   
                   o_rf_we             = o_commit & i_commit_rd_inst;  
                   o_rf_rd_wvalue      = i_commit_value;    
		        end 
		   
		               end // commit_wait
					
                //=========================================================
				// COMMIT CONTROL TRANSFER INSTRUCTIONS 
				//=========================================================	
				commit_brt: begin  
                    o_brt_commit          = 1'b1;  
                    o_brt_commit_rob_addr = commit_rob_addr_reg;
					if(i_brt_commit_done)
                    begin
                       commit_state_n    = commit_wait;
					   
					   if(i_brt_commit_miss_pre) begin 
					      o_flush       = 1'b1;                     // Generate Flush Flag. 
                          o_flush_pc    = i_brt_commit_cal_target;  // Set the PC to Calculated Target Address.  
                          o_flush_taken = i_commit_value[0];        // Take the Calculated Taken-NotTaken Value and Force the PC logic decision.					   
					   end 
					   
                       o_commit          = 1'b1;
                         
					   // For JAL and JALR, write PC+4 to RF.  	 
                       if(commit_cont_tra_inst_reg==3'b100 || commit_cont_tra_inst_reg==3'b010) begin 
					      o_rf_we        = 1'b1;
					      o_rf_rd_wvalue = commit_value_reg; 
						  o_commit_rd    = commit_rd_reg;
					   end
					   
                    end 					
                end 				
	
	
	   endcase 
	end  
end 



always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn) begin
      commit_state             <= commit_wait;
      commit_cont_tra_inst_reg <= '0;
      commit_value_reg         <= '0;
      commit_rd_reg            <= '0;   
      commit_rob_addr_reg      <= '0;
   end else begin
      commit_state             <= commit_state_n;
      commit_cont_tra_inst_reg <= commit_cont_tra_inst_next;
      commit_value_reg         <= commit_value_next;
      commit_rd_reg            <= commit_rd_next;
      commit_rob_addr_reg      <= commit_rob_addr_next;
   end 
end 


     



 

  
   
endmodule :Core_Control_Unit



//========================
// CORE CONTROL UNIT TEST BENCH 
//========================
module Core_Control_Unit_tb();

parameter ROBSIZE = 8;

logic start;
logic clk; 
logic rstn;
// CONTROL SIGNALS 
logic rd_inst;
// INSTRUCTION QUEUE SIGNALS 
logic i_iq_empty;
logic o_iq_rd_en;
// RESERVATION STATION SIGNALS 
logic i_rs1_full;
logic i_rs2_full; 
logic o_rs1_we; 
logic o_rs2_we; 
// ROB SIGNALS 
logic i_rob_full; 
logic o_rob_we;
// RAT SIGNALS 
logic o_rat_issue_we; 
logic o_rat_update;
// ALU_CONTROL1 SIGNALS
logic                       i_alu1_broadcast_ready;
logic [31:0]                i_alu1_broadcast_data;
logic                       o_alu1_broadcast_en;
logic [(ROBSIZE-1):0]       i_alu1_broadcast_rob_addr;
// ALU_CONTROL2 SIGNALS
logic                       i_alu2_broadcast_ready;
logic [31:0]                i_alu2_broadcast_data; 
logic                       o_alu2_broadcast_en;
logic [(ROBSIZE-1):0]       i_alu2_broadcast_rob_addr;
// BROADCAST BUS SIGNALS 
logic                       o_broadcast_en;
logic [31:0]                o_broadcast_data; 
logic [(ROBSIZE-1):0]       o_broadcast_rob_addr;
// COMMIT SIGNALS 
logic                        i_commit_ready;
logic [31:0]                 i_commit_value;
logic [4:0]                  i_commit_rd;
logic                        i_commit_exception;
logic                        i_commit_rd_inst;  
logic                        o_commit;
// LSQ COMMIT SIGNALS 
// COMMIT SIGNALS 
logic                        o_lsq_commit_en;
logic                        i_lsq_commit_ready;
logic                        i_lsq_commit_load_store;
logic [31:0]                 i_lsq_commit_data;  
logic [(ROBSIZE-1):0]        i_lsq_commit_rob_addr;
logic                        i_mem_busy;
// REGISTER FILE WRITE SIGNALS 
logic                        o_rf_we;
logic [4:0]                  o_rf_wrd;
logic [31:0]                 o_rf_rd_wvalue;



//========================
// CLOCK GENERATION 
//========================
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 

//========================
// RESET TASK: Generates an asyncrhonous, active low reset   
//========================
task RESET();
begin
  start                   = 1'b1;
  i_commit_ready          = '0;
  i_commit_value          = '0;
  i_commit_rd             = '0;
  i_commit_exception      = '0;
  i_commit_rd_inst        = '0;  
  i_iq_empty              = 1'b1; // IQ is empty initially
  i_rs1_full              = '0;
  i_rs2_full              = '0;
  i_rob_full              = '0;
  rd_inst                 = '0;
  i_alu1_broadcast_ready  = '0;
  i_alu2_broadcast_ready  = '0;
  i_lsq_commit_ready      = '0;
  i_lsq_commit_load_store = '0;
  i_lsq_commit_data       = '0;  
  i_lsq_commit_rob_addr   = '0;
  i_mem_busy              = '0;
  // 
  rstn = 1'b1;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b0;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b1;
  repeat(2) @(posedge clk); #3;
end 
endtask

//========================
// ALU CONTROL READY TO BROADCAST SIGNAL 
//========================
task ALU_BROADCAST(input logic         ALU1 = 1'b0,
                   input logic         ALU2 = 1'b0,
                   input logic [31:0]  ALU1_DATA = '0,
                   input logic [31:0]  ALU2_DATA = '0
                   );
begin
    @(posedge clk);
      i_alu1_broadcast_ready = ALU1;
      i_alu1_broadcast_data  = ALU1_DATA;
      i_alu2_broadcast_ready = ALU2;
      i_alu2_broadcast_data  = ALU2_DATA;    
    @(posedge clk);
      i_alu1_broadcast_ready = '0;
      i_alu1_broadcast_data  = '0;       
      i_alu2_broadcast_ready = '0;
      i_alu2_broadcast_data  = '0; 
end 
endtask

//========================
// KEEP THE IQ_EMPTY SIGNAL LOW FOR ready_cc CC to allow fetch signals 
//========================
task IQ_FETCH_READY (input int ready_cc,
                     input logic rd_inst_flg = 1'b1);
begin
     @(posedge clk);
       i_iq_empty <= 1'b0;
       rd_inst    <= rd_inst_flg;
     repeat(ready_cc) @(posedge clk);
       i_iq_empty <= 1'b1;            
end 
endtask


//========================
// CREATE delay_cc CC delay. 
//========================
task DELAY (input int delay_cc);
begin
     repeat(delay_cc) @(posedge clk);
end 
endtask



//================================
// MAIN STIMULUS 
//================================
/*initial begin
  RESET();
  //
  IQ_FETCH_READY ( .ready_cc (5));
  //
  DELAY(5);
  //
  ALU_BROADCAST( .ALU1 (1'b1), .ALU2 (1'b0), .ALU1_DATA  (32'hDEAD), .ALU2_DATA (32'hBEEF) );
  DELAY(5);
  //
  ALU_BROADCAST( .ALU1 (1'b0), .ALU2 (1'b1), .ALU1_DATA  (32'hDEAD), .ALU2_DATA (32'hBEEF) );
  DELAY(5);
  //
  ALU_BROADCAST( .ALU1 (1'b1), .ALU2 (1'b1), .ALU1_DATA  (32'hDEAD), .ALU2_DATA (32'hBEEF) );
  DELAY(5);
  //
  ALU_BROADCAST( .ALU1 (1'b1), .ALU2 (1'b1), .ALU1_DATA  (32'hDEAD), .ALU2_DATA (32'hBEEF) );
  DELAY(5);
  //
  repeat(10) @(posedge clk);
  $finish;
end */



initial begin 
   RESET();




end 




//================================
// Module Instantiation 
//================================
Core_Control_Unit #(.ROBSIZE (ROBSIZE)) uCore_Control_Unit (.*);




endmodule :Core_Control_Unit_tb 




