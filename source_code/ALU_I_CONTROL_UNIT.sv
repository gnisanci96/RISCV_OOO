`timescale 1ns / 1ps
`include "headers.vh"

module ALU_I_CONTROL_UNIT
#(parameter ROBSIZE = 8)
(
    input  logic                     clk, 
    input  logic                     rstn,
    input  logic                     i_flush,
    output logic                     o_busy,
    input  logic [(ROBSIZE-1):0]     i_rob_addr,
    input  logic                     i_con_branch_comp,
    input  logic                     i_addr_cal,
    input  logic [31:0]              i_rs1_value,
    input  logic [31:0]              i_rs2_value,
    input  logic [4:0]               i_alu_opcode,
    input  logic                     i_ex_en,   
    // BROADCAST SIGNALS 
    input  logic                     i_broadcast_en,
    output logic                     o_broadcast_ready, 
    output logic [31:0]              o_broadcast_out,
    output logic [(ROBSIZE-1):0]     o_broadcast_rob_addr,
    output logic                     o_broadcast_addr_cal,
    output logic                     o_broadcast_con_branch_comp
    );
  
typedef enum{
   IDLE,
   EX,
   BROADCAST
} state_type;
 
  
state_type state, state_next;
logic [(ROBSIZE-1):0] ex_rob_addr,  ex_rob_addr_next;
logic [31:0]              alu_out, alu_out_reg,alu_out_reg_next;
//
int delay, delay_next;
int cnt, cnt_next;
//
logic [31:0]              o_broadcast_out_next;
logic [(ROBSIZE-1):0]     o_broadcast_rob_addr_next;
logic [(ROBSIZE-1):0]     o_broadcast_rob_addr_next;
logic                     o_broadcast_addr_cal_next;
logic                     o_broadcast_con_branch_comp_next;
//
always_comb
begin
    //
    state_next  = state;
    cnt_next    = cnt;
    delay_next  = delay;
    o_broadcast_out_next      = o_broadcast_out;
    o_broadcast_rob_addr_next = o_broadcast_rob_addr;
    o_broadcast_addr_cal_next = o_broadcast_addr_cal;
    o_broadcast_con_branch_comp_next = o_broadcast_con_branch_comp;
    o_broadcast_ready = 1'b0;
    o_busy = 1'b1;
    //
   case(state)
      IDLE:
      begin
           o_busy = 1'b0;
           if(i_ex_en)
           begin
                                               state_next = EX;
             ex_rob_addr_next                 = i_rob_addr;
             cnt_next                         = '0;   
             alu_out_reg_next                 = alu_out;
             o_broadcast_addr_cal_next        = i_addr_cal;
             o_broadcast_con_branch_comp_next = i_con_branch_comp;
             //
             case(i_alu_opcode[2:0])
              `FUNCT3_ADD : delay_next = 5; 
              `FUNCT3_SRL : delay_next = 5; 
              `FUNCT3_OR  : delay_next = 5; 
              `FUNCT3_AND : delay_next = 5;           
              `FUNCT3_XOR : delay_next = 5; 
              `FUNCT3_SLT : delay_next = 5; 
              `FUNCT3_SLTU: delay_next = 5; 
              `FUNCT3_SLL : delay_next = 5; 
             endcase 
            //
           end  
      
       end 
      EX:begin 
         if(cnt < delay)
         begin
            cnt_next = cnt + 1;
         end else begin
            cnt_next = '0;
            o_broadcast_out_next      = alu_out_reg;
            o_broadcast_rob_addr_next = ex_rob_addr;
                                               state_next = BROADCAST;      
         end 
      
        end
      BROADCAST: begin
         
         if(i_broadcast_en)begin
            o_broadcast_ready = 1'b0;
                                               state_next = IDLE;                   
         end else begin
            o_broadcast_ready = 1'b1;
         end 
      
                 end   
   endcase 
end 


//=========================
// State Register   
//=========================  
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      state <= IDLE;
   end else if (i_flush)
   begin
      state <= IDLE;   
   end else begin
      state <= state_next;
   end 
end 


//=========================
// REGISTERS    
//=========================  
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
     cnt                         <= '0;
     delay                       <= '0;
     alu_out_reg                 <= '0;
     ex_rob_addr                 <= '0;
     o_broadcast_out             <= '0;
     o_broadcast_rob_addr        <= '0;
     o_broadcast_addr_cal        <= '0;
     o_broadcast_con_branch_comp <= '0; 
   end else begin
     cnt                         <= cnt_next;
     delay                       <= delay_next;
     alu_out_reg                 <= alu_out_reg_next;
     ex_rob_addr                 <= ex_rob_addr_next;
     o_broadcast_out             <= o_broadcast_out_next;
     o_broadcast_rob_addr        <= o_broadcast_rob_addr_next;
     o_broadcast_addr_cal        <= o_broadcast_addr_cal_next;
     o_broadcast_con_branch_comp <= o_broadcast_con_branch_comp_next;
   end 
end 

//===============================
// ALU INSTANTIATION 
//===============================
ALU_I uALU_I
  (
    .Num1   (i_rs1_value),
    .Num2   (i_rs2_value), 
    .OpSel  (i_alu_opcode), 
    .Result (alu_out)
  );    
    
    
endmodule



module ALU_I_CONTROL_UNIT_tb();

parameter ROBSIZE = 8;
logic                     clk; 
logic                     rstn;
logic                     i_flush;
logic                     o_busy;
logic [(ROBSIZE-1):0]     i_rob_addr;
logic [31:0]              i_rs1_value;
logic [31:0]              i_rs2_value;
logic [3:0]               i_alu_opcode;
logic                     i_ex_en;   
// BROADCAST SIGNALS 
logic                     i_broadcast_en;
logic                     o_broadcast_ready; 
logic [31:0]              o_broadcast_out;
logic [(ROBSIZE-1):0]     o_broadcast_rob_addr;

initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 

task RESET();
begin
   i_ex_en        = '0;
   i_broadcast_en = '0;
   i_flush        = '0;
   // 
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2)@(posedge clk);
   rstn = 1'b1;
end 
endtask 


    task SEND_EX(
    input logic [(ROBSIZE-1):0]     rob_addr,
    input logic [31:0]              rs1_value,
    input logic [31:0]              rs2_value,
    input logic [3:0]               alu_opcode
);
begin
    @(posedge clk);
    i_ex_en       <= 1'b1;
    i_rob_addr    <= rob_addr;
    i_rs1_value   <= rs1_value;
    i_rs2_value   <= rs2_value;
    i_alu_opcode  <= alu_opcode;
    @(posedge clk);
    i_ex_en       <= '0;
    i_rob_addr    <= '0;
    i_rs1_value   <= '0;
    i_rs2_value   <= '0;
    i_alu_opcode  <= '0;   
end 
endtask
//
task BROADCAST();
begin
     @(posedge clk);
        i_broadcast_en = 1'b1;
        $display("BROADCAST DATA: %h ROB: %h",o_broadcast_out,o_broadcast_rob_addr);
     @(posedge clk);
        i_broadcast_en = 1'b0;
end 
endtask


task FLUSH();
begin
   @(posedge clk);
     i_flush = 1'b1;
   @(posedge clk);
     i_flush = 1'b0;     
end 
endtask


initial begin
  RESET();
  //
SEND_EX(
    .rob_addr   (2), 
    .rs1_value  (3), 
    .rs2_value  (5),
    .alu_opcode (4'b1000)
);

FLUSH();


  //
repeat(10) @(posedge clk);
//BROADCAST();  
//

$finish;
end 




ALU_I_CONTROL_UNIT #( .ROBSIZE (ROBSIZE)) uALU_I_CONTROL_UNIT
(.*);

endmodule 


