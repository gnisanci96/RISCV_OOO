`timescale 1ns / 1ps



module BRT_TABLE
#(parameter BRTSIZE = 4,
  parameter ROBSIZE = 8)
(
   input logic                 clk, 
   input logic                 rstn, 
   //
   input logic                 i_brt_flush,
   //=====================================
   // ISSUE INTERFACE    
   //=====================================   
   input logic                 i_brt_wr_en,
   input logic [ROBSIZE-1:0]   i_brt_wr_rob_addr,
   input logic [31:0]          i_brt_wr_target_cal,
   input logic                 i_brt_wr_target_cal_v, 
   input logic [31:0]          i_brt_wr_target_pre,
   input logic                 i_brt_wr_taken_cal, 
   input logic                 i_brt_wr_taken_cal_v,
   input logic                 i_brt_wr_taken_pre, 
   input logic [31:0]          i_brt_wr_pc_p_4,
   input logic [31:0]          i_brt_wr_pc,
   //=====================================
   // Update Target Calculated.    
   //=====================================
   input  logic                i_brt_target_ready, 
   input  logic  [31:0]        i_brt_target_cal,
   input  logic [ROBSIZE-1:0]  i_brt_target_rob_addr,
   output logic                o_brt_read_btc, 
   //=====================================
   // Update Taken Calculated    
   //=====================================
   input  logic                i_brt_taken_ready, 
   input  logic                i_brt_taken_cal,
   input  logic [ROBSIZE-1:0]  i_brt_taken_rob_addr,
   output logic                o_brt_read_jeu, 
   //=====================================
   // COMMIT INTERFACE    
   //=====================================
   input  logic                 i_brt_commit,            // Send Commit signal to BRT and wait "i_brt_commit_ready" signal
   input  logic [(ROBSIZE-1):0] i_brt_commit_rob_addr,   // BRT Commit ROB addr
   output logic                 o_brt_commit_done,       // BRT Instruction Commit is DONE.  
   output logic                 o_brt_commit_miss_pre,   // 1: BRT instruction target is miss-predicted 
   output logic [31:0]          o_brt_commit_cal_target, // Correct Calculated Target address for BRT instruct   
   //=====================================
   // STATUS SIGNALS     
   //=====================================
   output logic                 o_brt_full,
   output logic                 o_btb_write_en,
   output logic [31:0]          o_btb_write_pc,
   output logic [31:0]          o_btb_write_target,
   output logic                 o_btb_write_is_ret,
   output logic                 o_bht_write_en,
   output logic [31:0]          o_bht_write_pc, 
   output logic                 o_bht_write_taken   
);


logic [(2**BRTSIZE-1):0]                      T_VALID;
logic [(2**BRTSIZE-1):0][(ROBSIZE-1):0]       T_ROB_ADDR;
logic [(2**BRTSIZE-1):0][31:0]                T_TARGET_CAL;
logic [(2**BRTSIZE-1):0]                      T_TARGET_CAL_V;
logic [(2**BRTSIZE-1):0][31:0]                T_TARGET_PRE;
logic [(2**BRTSIZE-1):0]                      T_TAKEN_CAL;
logic [(2**BRTSIZE-1):0]                      T_TAKEN_CAL_V;
logic [(2**BRTSIZE-1):0]                      T_TAKEN_PRE;
logic [(2**BRTSIZE-1):0][31:0]                T_PC_P_4;
logic [(2**BRTSIZE-1):0][31:0]                T_PC;
logic [(2**BRTSIZE-1):0]                      T_READY;
//
logic                 loop_break0,loop_break1,loop_break2,loop_break3;
logic [(BRTSIZE-1):0] issue_addr, update_target_addr,update_taken_addr;
logic read_btc_fifo,read_jeu_fifo, update_target_en,update_taken_en ;
//
logic [(ROBSIZE-1):0] commit_rob_addr;




//=====================================
// ISSUE LOGIC 
//=====================================
always_comb 
begin
   loop_break0 = '0;
   for(int i0=0;i0<(2**BRTSIZE);i0++)
   begin
      if((~T_VALID[i0]) && (~loop_break0))
      begin
         issue_addr = i0;
         loop_break0 = 1'b1;    
      end 
   end 
end 


//=====================================
// READY ARRAY GENERATION 
//=====================================
always_comb 
begin
   for(int i0=0;i0<(2**BRTSIZE);i0++)
   begin
      T_READY[i0] = T_TARGET_CAL_V[i0] & T_TAKEN_CAL_V[i0];
   end 
end 



//=====================================
// UPDATE TARGET CAL LOGIC 
//=====================================
always_comb
begin
   loop_break1 = '0; 
   loop_break1 = '0;
   update_target_addr = '0;
   read_btc_fifo      = '0;
   update_target_en   = '0;
   //  
   if(i_brt_target_ready)
   begin
      for(int i1=0;i1<(2**BRTSIZE);i1++)
      begin
         if((T_ROB_ADDR[i1]==i_brt_target_rob_addr) && T_VALID[i1] && (~loop_break1))
         begin
            update_target_addr  = i1;
            loop_break1         = 1'b1;  
            read_btc_fifo       = 1'b1;  
            update_target_en    = 1'b1;
         end 
      end 
   end 
   //
end 


//=====================================
// UPDATE TAKEN CAL LOGIC 
//=====================================
always_comb
begin
   loop_break2       = '0;
   read_jeu_fifo     = '0;
   update_taken_en   = '0;
   update_taken_addr = '0;
   if(i_brt_taken_ready)
   begin
      for(int i2=0;i2<(2**BRTSIZE);i2++)
      begin
         if((T_ROB_ADDR[i2]==i_brt_taken_rob_addr) && T_VALID[i2] && (~loop_break2))
         begin
            update_taken_addr   = i2;
            loop_break2         = 1'b1;  
            read_jeu_fifo       = 1'b1;  
            update_taken_en     = 1'b1;
         end 
      end 
   end 





end 


always_ff @(posedge clk or negedge rstn)
begin
   // 
   if(!rstn || i_brt_flush)
   begin
      T_VALID         <= '0;
      T_ROB_ADDR      <= '0;
      T_TARGET_CAL    <= '0;
      T_TARGET_CAL_V  <= '0;
      T_TARGET_PRE    <= '0;
      T_TAKEN_CAL     <= '0;
      T_TAKEN_CAL_V   <= '0;
      T_TAKEN_PRE     <= '0;
      T_PC_P_4        <= '0;
      T_PC            <= '0;
   end else begin
      // 
      if(i_brt_wr_en && ~o_brt_full)
      begin
         T_VALID[issue_addr]         <= 1'b1;
         T_ROB_ADDR[issue_addr]      <= i_brt_wr_rob_addr;
         T_TARGET_CAL[issue_addr]    <= i_brt_wr_target_cal;
         T_TARGET_CAL_V[issue_addr]  <= i_brt_wr_target_cal_v;
         T_TARGET_PRE[issue_addr]    <= i_brt_wr_target_pre;
         T_TAKEN_CAL[issue_addr]     <= i_brt_wr_taken_cal; 
         T_TAKEN_CAL_V[issue_addr]   <= i_brt_wr_taken_cal_v;
         T_TAKEN_PRE[issue_addr]     <= i_brt_wr_taken_pre; 
         T_PC_P_4[issue_addr]        <= i_brt_wr_pc_p_4;  
         T_PC                        <= i_brt_wr_pc;        
      end 
      
      if(o_brt_commit_done) begin 
         T_VALID[commit_rob_addr]       <= '0;
         T_ROB_ADDR[commit_rob_addr]    <= '0;
         T_TARGET_CAL[commit_rob_addr]  <= '0;
         T_TARGET_CAL_V[commit_rob_addr]<= '0;
         T_TARGET_PRE[commit_rob_addr]  <= '0;
         T_TAKEN_CAL[commit_rob_addr]   <= '0;
         T_TAKEN_CAL_V[commit_rob_addr] <= '0;
         T_TAKEN_PRE[commit_rob_addr]   <= '0;
         T_PC_P_4[commit_rob_addr]      <= '0;
         T_PC[commit_rob_addr]          <= '0;
      end 
      
      // 
      if(update_target_en)
      begin
         T_TARGET_CAL[update_target_addr]    <= i_brt_target_cal;
         T_TARGET_CAL_V[update_target_addr]  <= 1'b1;
      end 
      // 
      if(update_taken_en)
      begin
         T_TAKEN_CAL[update_taken_addr]    <= i_brt_taken_cal;
         T_TAKEN_CAL_V[update_taken_addr]  <= 1'b1;
      end 
     // 
   end
   // 
end 




assign o_brt_read_btc              = read_btc_fifo;
assign o_brt_full                  = &T_VALID;
assign o_brt_read_jeu              = read_jeu_fifo;
assign o_brt_commit_ready          = '0; 
assign o_brt_commit_target         = '0;  
assign o_brt_commit_taken          = '0;  
assign o_brt_commit_target_correct = '0;        
assign o_brt_commit_taken_correct  = '0;     


always_comb 
begin
   loop_break3             = '0;
   o_brt_commit_done       = '0;
   o_brt_commit_miss_pre   = '0;
   o_brt_commit_cal_target = '0;
   commit_rob_addr         = '0;
   //
   o_btb_write_en          = '0;
   o_btb_write_pc          = '0;
   o_btb_write_target      = '0;
   o_btb_write_is_ret      = '0;
   o_bht_write_en          = '0;
   o_bht_write_pc          = '0; 
   o_bht_write_taken       = '0;
     
   if(i_brt_commit)
   begin  
   for(int i =0;i<(2**BRTSIZE);i++)
   begin
      if((T_ROB_ADDR[i] == i_brt_commit_rob_addr) && ~loop_break3) 
      begin
         loop_break3           = 1'b1;
         commit_rob_addr       = i;
         if(T_READY[i]) 
         begin
            o_brt_commit_done       = 1'b1;
            o_brt_commit_miss_pre   = ~((T_TARGET_CAL[i] == T_TARGET_PRE[i]) && (T_TAKEN_CAL[i] == T_TAKEN_PRE[i])); 
            o_brt_commit_cal_target = T_TAKEN_CAL[i] ? T_TARGET_CAL[i] : T_PC_P_4[i];
            o_btb_write_en      = 1'b1;
            o_btb_write_pc      = T_PC[i];
            o_btb_write_target  = T_TARGET_CAL[i];
            o_btb_write_is_ret  = '0;
            o_bht_write_en      = 1'b1;
            o_bht_write_pc      = T_PC[i]; 
            o_bht_write_taken   = T_TAKEN_CAL[i];            
            
            
         end 
         
      end  
   end 
   end 





end 












endmodule :BRT_TABLE



module BRT_TABLE_tb();

   parameter BRTSIZE = 4;
   parameter ROBSIZE = 8;

   logic                 clk; 
   logic                 rstn; 
   logic                 i_brt_flush;
   //=====================================
   // ISSUE INTERFACE    
   //=====================================   
   logic                 i_brt_wr_en;
   logic [ROBSIZE-1:0]   i_brt_wr_rob_addr;
   logic [31:0]          i_brt_wr_target_cal;
   logic                 i_brt_wr_target_cal_v; 
   logic [31:0]          i_brt_wr_target_pre;
   logic                 i_brt_wr_taken_cal; 
   logic                 i_brt_wr_taken_cal_v;
   logic                 i_brt_wr_taken_pre; 
   //=====================================
   // Update Target Calculated.    
   //=====================================
   logic                i_brt_btc_ready; 
   logic  [31:0]        i_brt_target_cal;
   logic [ROBSIZE-1:0]  i_brt_rob_addr;
   logic                o_brt_read_btc; 
   logic                o_brt_full;



//=======================================
// RESET TASK 
//=======================================
task RESET();
begin
   i_brt_flush            <=  '0;
   i_brt_wr_en            <=  '0;
   i_brt_wr_rob_addr      <=  '0;
   i_brt_wr_target_cal    <=  '0;
   i_brt_wr_target_cal_v  <=  '0; 
   i_brt_wr_target_pre    <=  '0;
   i_brt_wr_taken_cal     <=  '0; 
   i_brt_wr_taken_cal_v   <=  '0;
   i_brt_wr_taken_pre     <=  '0; 
   i_brt_btc_ready        <=  '0; 
   i_brt_target_cal       <=  '0;
   i_brt_rob_addr         <=  '0;
   //==================================
   rstn = 1'b1;
   @(posedge clk); 
   rstn = 1'b0;
   repeat(2) @(posedge clk);
   rstn = 1'b1;
end 
endtask 

//===================================
// ISSUE TASK 
//===================================
task ISSUE( input logic [ROBSIZE-1:0]   brt_wr_rob_addr,
            input logic [31:0]          brt_wr_target_cal,
            input logic                 brt_wr_target_cal_v, 
            input logic [31:0]          brt_wr_target_pre,
            input logic                 brt_wr_taken_cal, 
            input logic                 brt_wr_taken_cal_v,
            input logic                 brt_wr_taken_pre      );
begin
   @(posedge clk);
   i_brt_wr_en           <=  1'b1;
   i_brt_wr_rob_addr     <=  brt_wr_rob_addr;
   i_brt_wr_target_cal   <=  brt_wr_target_cal;
   i_brt_wr_target_cal_v <=  brt_wr_target_cal_v; 
   i_brt_wr_target_pre   <=  brt_wr_target_pre;
   i_brt_wr_taken_cal    <=  brt_wr_taken_cal; 
   i_brt_wr_taken_cal_v  <=  brt_wr_taken_cal_v;
   i_brt_wr_taken_pre    <=  brt_wr_taken_pre;
   @(posedge clk);
   i_brt_wr_en           <=  '0;
   i_brt_wr_rob_addr     <=  '0;
   i_brt_wr_target_cal   <=  '0;
   i_brt_wr_target_cal_v <=  '0; 
   i_brt_wr_target_pre   <=  '0;
   i_brt_wr_taken_cal    <=  '0; 
   i_brt_wr_taken_cal_v  <=  '0;
   i_brt_wr_taken_pre    <=  '0;
end 
endtask

//===================================
// TEST-1 
//===================================
task test1();
begin
ISSUE( .brt_wr_rob_addr       ('d5), 
       .brt_wr_target_cal     ('d5), 
       .brt_wr_target_cal_v   ('d1),  
       .brt_wr_target_pre     ('d5), 
       .brt_wr_taken_cal      ('d1),  
       .brt_wr_taken_cal_v    ('d1),
       .brt_wr_taken_pre      ('d1)     );



end 
endtask 

//===================================
// MAIN STIMULUS 
//===================================
initial begin
  RESET(); 
  test1(); 
  
 
  repeat(10) @(posedge clk);
  $finish();
end 


//==================================
// DUT Instantiation 
//==================================
BRT_TABLE
#(.BRTSIZE (BRTSIZE),
  .ROBSIZE (ROBSIZE)  )
uBRT_TABLE
(.*);	   

endmodule :BRT_TABLE_tb 



