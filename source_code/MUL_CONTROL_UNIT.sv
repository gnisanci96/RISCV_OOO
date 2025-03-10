`timescale 1ns / 1ps
`include "headers.vh"
// SINGLE-CYCLE RV32M Execution Unit 
//////////////////////////////////////////////////////////////////////////////////
// RV32M Standard Extension
// funct7  rs2 rs1 funct3  rd  opcode
// 0000001 rs2 rs1  000    rd 0110011 MUL       lower32-x-rs1 x-rs2
// 0000001 rs2 rs1  001    rd 0110011 MULH      Upper32-signed-rs1           signed-rs1 
// 0000001 rs2 rs1  010    rd 0110011 MULHSU    Upper32-unsigned-rs1 unsigned-rs2 
// 0000001 rs2 rs1  011    rd 0110011 MULHU     Upper32-signed-rs1   unsigned-rs2 
// 0000001 rs2 rs1  100    rd 0110011 DIV
// 0000001 rs2 rs1  101    rd 0110011 DIVU
// 0000001 rs2 rs1  110    rd 0110011 REM
// 0000001 rs2 rs1  111    rd 0110011 REMU
//////////////////////////////////////////////////////////////////////////////////
// Instruction Functionality 
// MUL: 
// MUL performs an XLEN-bit×XLEN-bit multiplication of rs1 by rs2 and places the lower XLEN bits
// in the destination register.
// MULH:
// Returns the upper 32-bit for signed×signed Multiplication. 
// MULHU: 
// Returns the upper 32-bit for unsigned×unsigned Multiplication. 
// MULHSU: 
// Returns the upper 32-bit for signed rs1×unsigned rs2 Multiplication. 
// DIV:
// It performs a 32 bits by 32 bits signed integer division of rs1 by
// rs2, rounding towards zero. 
// DIVU:
// It performs a 32 bits by 32 bits unsigned integer division of rs1 by
// rs2, rounding towards zero. 
// REM:
// REM provides the remainder of the DIV division operation.
// REMU:
// REMU provides the remainder of the DIV division operation.



module MUL_CONTROL_UNIT
#(parameter ROBSIZE = 8)
(
    input  logic                     clk, 
    input  logic                     rstn,
    input  logic                     i_flush,
    output logic                     o_busy,
    input  logic [(ROBSIZE-1):0]     i_rob_addr,
    input  logic [31:0]              i_rs1_value,
    input  logic [31:0]              i_rs2_value,
    input  logic [2:0]               i_alu_m_opcode,
    input  logic                     i_ex_en,   
    // BROADCAST SIGNALS 
    input  logic                     i_broadcast_en,
    output logic                     o_broadcast_ready, 
    output logic [31:0]              o_broadcast_out,
    output logic [(ROBSIZE-1):0]     o_broadcast_rob_addr
    );
  
typedef enum{
   IDLE,
   EX,
   BROADCAST
} state_type;
 
  
state_type state, state_next;
logic [(ROBSIZE-1):0]     ex_rob_addr_reg,  ex_rob_addr_next;
logic [63:0]              ex_result;
logic                     ex_start,ex_ready;
logic                     ex_num1_signed,ex_num2_signed;
logic [1:0]               ex_opcode_next,ex_opcode_reg;
//
int delay, delay_next;
int cnt, cnt_next;
//
logic [31:0]              o_broadcast_out_next;
logic [(ROBSIZE-1):0]     o_broadcast_rob_addr_next;
//
always_comb
begin
    //
    state_next  = state;
    ex_opcode_next = ex_opcode_reg;


    o_broadcast_out_next      = o_broadcast_out;
    o_broadcast_rob_addr_next = o_broadcast_rob_addr;
    o_broadcast_ready = 1'b0;
    o_busy = 1'b1;
    //
    ex_start = 1'b0;
   case(state)
      IDLE:
      begin
           o_busy = 1'b0;
           if(i_ex_en && ~i_flush)
           begin
              //
              ex_opcode_next  = i_alu_m_opcode;
              ex_rob_addr_next = i_rob_addr;
              //
                 ex_start = 1'b1;
                 case(i_alu_m_opcode)
                    3'b000: begin ex_num1_signed = 1'b0; ex_num2_signed = 1'b0; end 
                    3'b001: begin ex_num1_signed = 1'b1; ex_num2_signed = 1'b1; end 
                    3'b010: begin ex_num1_signed = 1'b0; ex_num2_signed = 1'b0; end 
                    3'b011: begin ex_num1_signed = 1'b1; ex_num2_signed = 1'b0; end 
                    3'b100: begin ex_num1_signed = 1'b1; ex_num2_signed = 1'b1; end 
                    3'b101: begin ex_num1_signed = 1'b0; ex_num2_signed = 1'b0; end 
                    3'b110: begin ex_num1_signed = 1'b1; ex_num2_signed = 1'b1; end 
                    3'b111: begin ex_num1_signed = 1'b0; ex_num2_signed = 1'b0; end 
                 endcase 
                                               state_next = EX;                           
              
           end 
       end 
      EX:
        begin 
           if(i_flush)
           begin 
                                               state_next = IDLE;                   
           end else if(ex_ready)
            begin
               
               if(ex_opcode_reg[2])
               begin
                  o_broadcast_out_next      =  ex_opcode_reg[1]   ? ex_result[64:32]: ex_result[31:0];
               end else begin
                  o_broadcast_out_next      = |ex_opcode_reg[1:0] ? ex_result[64:32]: ex_result[31:0];
               end 
               
               
               o_broadcast_rob_addr_next = ex_rob_addr_reg;
                                               state_next = BROADCAST;
            end 
        
        end    
      BROADCAST: begin
         
           if(i_flush)
           begin 
                                               state_next = IDLE;                   
           end else if(i_broadcast_en)begin
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
     ex_rob_addr_reg             <= '0;
     o_broadcast_out             <= '0;
     o_broadcast_rob_addr        <= '0;
     ex_opcode_reg               <= '0;
   end else begin
     ex_rob_addr_reg             <= ex_rob_addr_next;
     o_broadcast_out             <= o_broadcast_out_next;
     o_broadcast_rob_addr        <= o_broadcast_rob_addr_next;
     ex_opcode_reg               <= ex_opcode_next;
   end 
end 

//===============================
// ALU INSTANTIATION 
//===============================
`ifdef BOOTH_MUL_ALGO
//
BOOTH_MUL_ALGO
#(.SIZE (32))
MUL
(
   .clk          (clk),
   .rstn         (rstn),
   .flush        (i_flush), 
   .start        (mul_start),
   .num1_signed  (mul_num1_signed),
   .num2_signed  (mul_num2_signed),
   .num1         (i_rs1_value),
   .num2         (i_rs2_value),
   .result       (mul_result),
   .ready        (mul_ready)
    );
    
`else
 
MUL_DIV
#(.SIZE (32))
uMUL_DIV
(
   .clk          (clk),
   .rstn         (rstn),
   .flush        (i_flush),         // 1: Flush the ongoing MUL_DIV execution.
   .start        (ex_start),       // 1: Input parameters are ready to be sampled.
   .mul_div      (~i_alu_m_opcode[2]),     // 1: Multiplication Operation 0:Division Operation
   .num1_signed  (ex_num1_signed), // 1: num1 is a signed number. 0: num1 is an unsigned number.
   .num2_signed  (ex_num2_signed), // 1: num2 is a signed number. 0: num2 is an unsigned number.
   .num1         (i_rs1_value),
   .num2         (i_rs2_value),
   .result       (ex_result),      // Multiplication:  result is product. Division: Higher-half: Remainder Lower-half is quotient. 
   .ready        (ex_ready)        // Result is ready to be sampled.
    );
    
 `endif    
    
endmodule :MUL_CONTROL_UNIT



module MUL_CONTROL_UNIT_tb();

parameter ROBSIZE = 8;
logic                     clk; 
logic                     rstn;
logic                     i_flush;
logic                     o_busy;
logic [(ROBSIZE-1):0]     i_rob_addr;
logic [31:0]              i_rs1_value;
logic [31:0]              i_rs2_value;
logic [3:0]               i_alu_m_opcode;
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
    input logic [31:0]              rs1,
    input logic [31:0]              rs2,
    input logic [3:0]               opcode,
    input logic [31:0]              expected
);
begin
    @(posedge clk);
    i_ex_en       <= 1'b1;
    i_rob_addr    <= rob_addr;
    i_rs1_value   <= rs1;
    i_rs2_value   <= rs2;
    i_alu_m_opcode  <= opcode;
    @(posedge clk);
    i_ex_en       <= '0;
    i_rob_addr    <= '0;
    i_rs1_value   <= '0;
    i_rs2_value   <= '0;
    i_alu_m_opcode<= '0; 
    @(posedge clk iff o_broadcast_ready);
    if(o_broadcast_out == expected) begin $display("GOOD JOB !");  end   
    else begin  $display("!!ERROR!! Expected :%h Actual:%h",expected,o_broadcast_out); end 
    repeat(2)@(posedge clk);
    i_broadcast_en = 1'b1;
    @(posedge clk);
    i_broadcast_en = 1'b0;
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
  SEND_EX(.rob_addr   (1), .rs1  (3), .rs2  (5),.opcode (2'b000), .expected (15) );
//FLUSH();


  //
repeat(10) @(posedge clk);
//BROADCAST();  
//

$finish;
end 




MUL_CONTROL_UNIT #( .ROBSIZE (ROBSIZE)) uMUL_CONTROL_UNIT
(.*);

endmodule 


