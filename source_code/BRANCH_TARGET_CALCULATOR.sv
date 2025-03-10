`timescale 1ns / 1ps

/*
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
   .i_btc_flush                   (btc_flush),
   //===============================
   // Write Interface 
   //===============================
   .i_btc_wr_en                   (btc_wr_en          ),
   .i_btc_wr_rob_addr             (btc_wr_rob_addr    ),
   .i_btc_wr_op1                  (btc_wr_op1         ),
   .i_btc_wr_op1_rob_addr         (btc_wr_op1_rob_addr),   
   .i_btc_wr_op1_v                (btc_wr_op1_v       ),
   .i_btc_wr_op2                  (btc_wr_op2         ),
   .i_btc_wr_op2_v                (btc_wr_op2_v       ),
   //===============================
   // Broadcast Interface 
   //===============================
   // Catch Broadcast-Compare ROB of op1- Update Values      
   .i_btc_broadcast               (btc_broadcast          ),
   .i_btc_broadcast_rob_addr      (btc_broadcast_rob_addr ),
   .i_btc_broadcast_data          (btc_broadcast_data     ),
   //===============================
   // BRT Table Interface 
   //===============================
   .o_brt_target_cal_ready        (brt_target_cal_ready  ),
   .o_brt_target_cal              (brt_target_cal        ),
   .o_btc_target_rob_addr         (btc_target_rob_addr   ), 
   .i_brt_target_receive_ack      (brt_target_receive_ack),
   //
   .o_brt_full                    (brt_full)
);
*/


module BRANCH_TARGET_CALCULATOR
#(  parameter BTCSIZE = 4,
    parameter ROBSIZE = 8)
(
   input logic                       clk, 
   input logic                       rstn, 
   //===============================
   // Flush Command 
   //===============================
   input logic                     i_btc_flush,
   //===============================
   // Write Interface 
   //===============================
   input logic                     i_btc_wr_en,       // Issue new Instruction 
   input logic [(ROBSIZE-1):0]     i_btc_wr_rob_addr, // ROB address of the instruction  
   input logic [31:0]              i_btc_wr_op1,
   input logic [(ROBSIZE-1):0]     i_btc_wr_op1_rob_addr,    
   input logic                     i_btc_wr_op1_v,
   input logic [31:0]              i_btc_wr_op2,
   input logic                     i_btc_wr_op2_v,
   //===============================
   // Broadcast Interface 
   //===============================
   // Catch Broadcast-Compare ROB of op1- Update Values      
   input logic                      i_btc_broadcast,
   input logic [(ROBSIZE-1):0]      i_btc_broadcast_rob_addr,
   input logic [31:0]               i_btc_broadcast_data,
   //===============================
   // BRT Table Interface 
   //===============================
   output logic                     o_btc_fifo_emtpy,
   output logic [31:0]              o_btc_fifo_target_cal,
   output logic [(ROBSIZE-1):0]     o_btc_fifo_rob_addr,  
   input  logic                     i_btc_fifo_rd_en, 
   //
   output logic                     o_btc_full
);


localparam FIFODEPTH     = 3;
localparam FIFODATA0SIZE = 32; 
localparam FIFODATA1SIZE = ROBSIZE; 



logic [(2**BTCSIZE-1):0]                      T_VALID;
logic [(2**BTCSIZE-1):0] [(ROBSIZE-1):0]      T_ROB_ADDR;
logic [(2**BTCSIZE-1):0] [31:0]               T_OP1;
logic [(2**BTCSIZE-1):0] [(ROBSIZE-1):0]      T_OP1_ROB_ADDR;
logic [(2**BTCSIZE-1):0]                      T_OP1_V;
logic [(2**BTCSIZE-1):0] [31:0]               T_OP2;
logic [(2**BTCSIZE-1):0]                      T_OP2_V;
logic [(2**BTCSIZE-1):0]                      T_READY; 


logic loop_break, loop_break2,loop_break3;
logic [(BTCSIZE-1):0] issue_addr;
logic                 dispatch_en; 
logic [(BTCSIZE-1):0] dispatch_addr; 
logic                 broadcast_en; 
logic [(BTCSIZE-1):0] broadcast_addr; 


logic                          fifo_WrtEn_i;
logic [(FIFODATA0SIZE-1):0]    fifo_data0_i;
logic [(FIFODATA1SIZE-1):0]    fifo_data1_i; 
logic                          fifo_RdEn_i;
logic [(FIFODATA0SIZE-1):0]    fifo_data0_o;      
logic [(FIFODATA1SIZE-1):0]    fifo_data1_o;        
logic                          fifo_Full_o;
logic                          fifo_Empty_o;


always_comb 
begin
   loop_break = '0;
   for(int i=0;i<(2**BTCSIZE);i++)
   begin
      if((T_VALID[i]==0) && (loop_break==0))
      begin
         issue_addr = i;
         loop_break = 1'b1;    
      end 
   end 
end 



assign T_READY = T_OP1_V & T_OP2_V;







always_comb
begin
   // 
   dispatch_addr = '0;
   loop_break2   = '0;
   fifo_WrtEn_i  = '0;
   fifo_data0_i  = '0;
   fifo_data1_i  = '0;
   dispatch_en   = '0;
   //
   for(int i0=0;i0<(2**BTCSIZE);i0++)
   begin
      if((T_VALID[i0]==1) && (T_READY[i0]==1) && (loop_break2==0))
      begin
         dispatch_addr = i0;
         loop_break2   = 1'b1;
         //
         fifo_WrtEn_i  = 1'b1;
         fifo_data0_i  = T_OP1[dispatch_addr] + T_OP2[dispatch_addr];
         fifo_data1_i  = T_ROB_ADDR[dispatch_addr];
         // 
         dispatch_en   = 1'b1;
      end 
   end 


end 






always_comb
begin
  loop_break3    = '0;
  broadcast_addr = '0;
  broadcast_en   = '0;
  if(i_btc_broadcast)
  begin
   for(int i1=0;i1<(2**BTCSIZE);i1++)
   begin
      if((T_VALID[i1]==1) && (T_OP1_ROB_ADDR[i1]==i_btc_broadcast_rob_addr)&& (~T_OP1_V[i1])  && (loop_break3==0))
      begin
         broadcast_addr = i1;
         loop_break3    = 1'b1;
         broadcast_en   = 1'b1;
      end 
   end 
  end 
  
end 






always_ff @(posedge clk or negedge rstn)
begin

   if(!rstn)
   begin
      T_VALID         <=  '0; 
      T_ROB_ADDR      <=  '0; 
      T_OP1           <=  '0; 
      T_OP1_ROB_ADDR  <=  '0; 
      T_OP1_V         <=  '0; 
      T_OP2           <=  '0; 
      T_OP2_V         <=  '0;  
   end else if (i_btc_flush)
   begin
      T_VALID         <=  '0; 
      T_ROB_ADDR      <=  '0; 
      T_OP1           <=  '0; 
      T_OP1_ROB_ADDR  <=  '0; 
      T_OP1_V         <=  '0; 
      T_OP2           <=  '0; 
      T_OP2_V         <=  '0; 
   end else begin
      
      //=====================================
      // NEW INSTRUCTION ISSUE 
      //=====================================
      if(i_btc_wr_en && ~o_btc_full)
      begin
         T_VALID[issue_addr]        <=  1'b1;
         T_ROB_ADDR[issue_addr]     <=  i_btc_wr_rob_addr;
         T_OP1[issue_addr]          <=  i_btc_wr_op1;
         T_OP1_ROB_ADDR[issue_addr] <=  i_btc_wr_op1_rob_addr;
         T_OP1_V[issue_addr]        <=  i_btc_wr_op1_v;
         T_OP2[issue_addr]          <=  i_btc_wr_op2;
         T_OP2_V[issue_addr]        <=  i_btc_wr_op2_v;
      end 
      //=====================================
      // Dispatched Instruction to the Execution Unit. 
      //=====================================
      if(dispatch_en)
      begin
         T_VALID[dispatch_addr]     <= 1'b0;
      end
      //=====================================
      // Catch Broadcasted Data From for OP1. 
      //=====================================
      if(broadcast_en)
      begin
         T_OP1[broadcast_addr]     <= i_btc_broadcast_data;
         T_OP1_V[broadcast_addr]   <= 1'b1;
      end 
      
   end 
   
end 





//=======================================
// FIFO INSTANTIATION 
//=======================================
FIFO_DUAL_DATA 
#(.FIFODEPTH (FIFODEPTH),
  .DATA0SIZE (FIFODATA0SIZE), 
  .DATA1SIZE (FIFODATA1SIZE) )
uFIFO_DUAL_DATA
(
  .clk      (clk),
  .rstn     (rstn),
  //
  .i_WrtEn  (fifo_WrtEn_i),
  .i_data0  (fifo_data0_i),
  .i_data1  (fifo_data1_i),
  //         
  .i_RdEn   (fifo_RdEn_i),
  .o_data0  (fifo_data0_o),   
  .o_data1  (fifo_data1_o),       
  //         
  .o_Full   (fifo_Full_o),
  .o_Empty  (fifo_Empty_o)
    );



assign o_btc_fifo_emtpy       = fifo_Empty_o;
assign o_btc_fifo_target_cal  = fifo_data0_o;
assign o_btc_fifo_rob_addr    = fifo_data1_o;  
assign fifo_RdEn_i            = i_btc_fifo_rd_en;

assign o_btc_full             = &T_VALID;



endmodule :BRANCH_TARGET_CALCULATOR









module BRANCH_TARGET_CALCULATOR_tb();

parameter BTCSIZE = 4;
parameter ROBSIZE = 8;

logic                     clk; 
logic                     rstn; 
//===============================
// Flush Command 
//===============================
logic                     i_btc_flush;
//===============================
// Write Interface 
//===============================
logic                     i_btc_wr_en;
logic [(ROBSIZE-1):0]     i_btc_wr_rob_addr;
logic [31:0]              i_btc_wr_op1;
logic [(ROBSIZE-1):0]     i_btc_wr_op1_rob_addr;    
logic                     i_btc_wr_op1_v;
logic [31:0]              i_btc_wr_op2;
logic                     i_btc_wr_op2_v;
//===============================
// Broadcast Interface 
//===============================
// Catch Broadcast-Compare ROB of op1- Update Values      
logic                      i_btc_broadcast;
logic [(ROBSIZE-1):0]      i_btc_broadcast_rob_addr;
logic [31:0]               i_btc_broadcast_data;
//===============================
// BRT Table Interface 
//===============================
logic                     o_brt_target_cal_ready;
logic [31:0]              o_brt_target_cal;
logic [(ROBSIZE-1):0]     o_btc_target_rob_addr;  
logic                     i_brt_target_receive_ack; 
//
logic                     o_brt_full;
//
int error = '0;

initial begin
   clk  = 1'b0;
   rstn = 1'b1;
end 

always #10 clk = ~clk;



task RESET(); 
begin
   i_btc_flush            = '0;
   i_btc_wr_en            = '0;
   i_btc_wr_rob_addr      = '0;
   i_btc_wr_op1           = '0;
   i_btc_wr_op1_rob_addr  = '0;    
   i_btc_wr_op1_v         = '0;
   i_btc_wr_op2           = '0;
   i_btc_wr_op2_v         = '0;
   //
   i_btc_broadcast           = '0;
   i_btc_broadcast_rob_addr  = '0;
   i_btc_broadcast_data      = '0;
   // 
   i_brt_target_receive_ack  = '0;
   //
   rstn = 1'b1;   
   repeat(2)  @(posedge clk); 
   rstn = 1'b0;   
   repeat(2)  @(posedge clk); 
   rstn = 1'b1;   
end 
endtask 


//=========================================
// ISSUE TASK 1 
//=========================================
task ISSUE(
           input logic [(ROBSIZE-1):0]     btc_wr_rob_addr,
           input logic [31:0]              btc_wr_op1,
           input logic [(ROBSIZE-1):0]     btc_wr_op1_rob_addr,    
           input logic                     btc_wr_op1_v,
           input logic [31:0]              btc_wr_op2,
           input logic                     btc_wr_op2_v);
begin
   @(posedge clk);
   i_btc_wr_en           = 1'b1;
   i_btc_wr_rob_addr     = btc_wr_rob_addr;
   i_btc_wr_op1          = btc_wr_op1;
   i_btc_wr_op1_rob_addr = btc_wr_op1_rob_addr;    
   i_btc_wr_op1_v        = btc_wr_op1_v;
   i_btc_wr_op2          = btc_wr_op2;
   i_btc_wr_op2_v        = btc_wr_op2_v;
   @(posedge clk);
   i_btc_wr_en           = '0;
   i_btc_wr_rob_addr     = '0;
   i_btc_wr_op1          = '0;
   i_btc_wr_op1_rob_addr = '0;    
   i_btc_wr_op1_v        = '0;
   i_btc_wr_op2          = '0;
   i_btc_wr_op2_v        = '0;
   
end 
endtask 


task BROADCAST(
                 input logic [(ROBSIZE-1):0]      btc_broadcast_rob_addr,
                 input logic [31:0]               btc_broadcast_data);
begin
   @(posedge clk);
      i_btc_broadcast           <= 1'b1;
      i_btc_broadcast_rob_addr  <= btc_broadcast_rob_addr;
      i_btc_broadcast_data      <= btc_broadcast_data;
   @(posedge clk);
      i_btc_broadcast           <= '0;
      i_btc_broadcast_rob_addr  <= '0;
      i_btc_broadcast_data      <= '0;
end 
endtask 



logic [7:0][(ROBSIZE-1):0]     rand_rob_addr;
logic [7:0][31:0]              rand_op1;
logic [7:0][(ROBSIZE-1):0]     rand_op1_rob_addr;
logic [7:0][31:0]              rand_op2;




//=========================================
// TEST 1 TASK  
//=========================================
task test1();
begin
   error = 0;
   for(int i=0;i<8;i++)
   begin
      rand_rob_addr[i]     = $urandom();
      rand_op1[i]          = $urandom();
      rand_op1_rob_addr[i] = $urandom();
      rand_op2[i]          = $urandom();
      
   ISSUE(
       .btc_wr_rob_addr     (rand_rob_addr[i]),
       .btc_wr_op1          (rand_op1[i]),
       .btc_wr_op1_rob_addr (rand_op1_rob_addr[i]),    
       .btc_wr_op1_v        ('d1),
       .btc_wr_op2          (rand_op2[i]),
       .btc_wr_op2_v        ('d1) 
	   );  
      
   end 
   repeat(8) @(posedge clk); 
   
   for(int i=0;i<8;i++)
   begin
      if(uBRANCH_TARGET_CALCULATOR.uBTC_FIFO.T_DATA0[i] != (rand_op1[i]+rand_op2[i]) || 
         uBRANCH_TARGET_CALCULATOR.uBTC_FIFO.T_DATA1[i] != rand_rob_addr[i])
         begin
            error++;  
            $display("Test1 Error at index %0d ",i);
         end 
   end
   
   if(error == 0) begin $display("TEST1 PASS"); end   
	   
end 
endtask


//=========================================
// TEST 2 TASK  
//=========================================
task test2();
begin
   error = 0;
   
   for(int i=0;i<8;i++)
   begin
      rand_rob_addr[i]     = $urandom();
      rand_op1[i]          = $urandom()%20;
      rand_op1_rob_addr[i] = $urandom();
      rand_op2[i]          = $urandom()%20;
      
   ISSUE(
       .btc_wr_rob_addr     (rand_rob_addr[i]),
       .btc_wr_op1          (rand_op1[i]),
       .btc_wr_op1_rob_addr (rand_op1_rob_addr[i]),    
       .btc_wr_op1_v        ('d0),
       .btc_wr_op2          (rand_op2[i]),
       .btc_wr_op2_v        ('d1) 
	   );  
      
   end 
   repeat(8) @(posedge clk); 
   //
   //
   for(int i=0;i<8;i++)
   begin
       BROADCAST(
           .btc_broadcast_rob_addr  (rand_op1_rob_addr[i]),
           .btc_broadcast_data      (rand_op1[i]) );
   end 
   //
   //
   repeat(8) @(posedge clk); 

   for(int i=0;i<8;i++)
   begin
      if(uBRANCH_TARGET_CALCULATOR.uBTC_FIFO.T_DATA0[i] != (rand_op1[i]+rand_op2[i]) || 
         uBRANCH_TARGET_CALCULATOR.uBTC_FIFO.T_DATA1[i] != rand_rob_addr[i])
         begin
            error++;  
            $display("Test2 Error at index %0d ",i);
         end 
   end 


         
         
           if(error == 0) begin $display("TEST2 PASS"); end    
         

end 
endtask




initial begin
   RESET();    
   repeat(10) @(posedge clk);
   test2();
	   
	   
	   
   repeat(100) @(posedge clk);
   $finish();
end 




//=================================
// DUT Instantiation 
//=================================
BRANCH_TARGET_CALCULATOR
#( .BTCSIZE (4),
   .ROBSIZE (8)   )
uBRANCH_TARGET_CALCULATOR
(.*);


endmodule :BRANCH_TARGET_CALCULATOR_tb 
