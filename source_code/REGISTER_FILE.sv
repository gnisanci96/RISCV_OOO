`timescale 1ns / 1ps

module REGISTER_FILE(
  input  logic  clk,
  input  logic  rstn,
  //
  input  logic        i_we,
  input  logic [4:0]  i_rd,
  input  logic [31:0] i_rd_Value,
  //
  input  logic [4:0]  i_rs1,
  input  logic [4:0]  i_rs2,
  output logic [31:0] o_rs1_RF_Value,
  output logic [31:0] o_rs2_RF_Value
    );
       
logic [31:0][31:0] reg_mem;    
 
//---------------------------------- DATA WRITE    
always_ff @(posedge clk or negedge rstn) begin
  if(!rstn)begin
     reg_mem <= '0;
  end else if(i_we && (i_rd!=5'd0)) begin 
    reg_mem[i_rd] <= i_rd_Value;    
  end
end 

//------------------------------------READ DATA at address i_rs1  
assign o_rs1_RF_Value = reg_mem[i_rs1];

//------------------------------------READ DATA at address i_rs2  
assign o_rs2_RF_Value = reg_mem[i_rs2];  
  
    
    
endmodule :REGISTER_FILE 



module REGISTER_FILE_tb();


  logic        clk;
  logic [4:0]  i_rs1;
  logic [4:0]  i_rs2;
  //
  logic        i_we;
  logic [4:0]  i_rd;
  logic [31:0] i_rd_Value;
  //
  logic [31:0] o_rs1_RF_Value;
  logic [31:0] o_rs2_RF_Value;
  
///////////////////
// CLOCK GENERATION 
//////////////////  
initial begin
 clk = 1'b0;
 forever #10 clk = ~clk;
end   
  

///////////////////
// RESET TASK  
//////////////////
task RESET();
begin
   i_we = 1'b0;
end 
endtask 

///////////////////
// WRITE TASK 
//////////////////
task WRITE(input logic [4:0] rd, input logic [31:0] data);
begin
   @(posedge clk);
      i_we       = 1'b1;
      i_rd       = rd;
      i_rd_Value = data; 
   @(posedge clk);
      i_we       = 1'b0;
end 
endtask 

///////////////////
// READ TASK 
//////////////////
task READ(input  logic [4:0]  rs1, 
          input  logic [4:0]  rs2,
          output logic [31:0] rs1_value,
          output logic [31:0] rs2_value);
begin
   i_rs1 = rs1;
   i_rs2 = rs2;
   #1;
   rs1_value = o_rs1_RF_Value;
   rs2_value = o_rs2_RF_Value;
end
endtask 





initial begin
RESET();
// Write to the Registers 
  for(int i=0;i<32;i++)
  begin
    WRITE(.rd (i)   , .data (i+1));
  end 
$finish; 
end 










//////////////////////
// DUT Instantiation 
//////////////////////
REGISTER_FILE uREGISTER_FILE(.*);



endmodule :REGISTER_FILE_tb 



