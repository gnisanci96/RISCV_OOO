`timescale 1ns / 1ps

////////////////////////////////////////
/// RAT FUNCTIONALITY 
///////////////////////////////////////
// Function 1: ROB address Read 
// It gets the source register addresses rs1 and rs2 and we provide ROB address 
// Also it outputs the flags that says if the ROB address is valid or not. 
// Inputs : rs1, rs2 
// Outputs: o_rs1_ROB_Addr, o_rs2_ROB_Addr, o_rs1_ROB_or_RF, o_rs2_ROB_or_RF
// Function 2:
// We get the write enable, register address, ROB address and ROB_or_RF flag 
// Input: i_we,i_rd, i_ROB_Addr, i_ROB_or_RF
// output: Update the RAT table 
// Function3: 
// For Register 0, it should always Return Read from RF. 
// Function4: 
// 1-Get the Commit Signals (Commit_en, Commit_rd, Commit_rob)
// 2- If the if the rob_addr_table[commit_rd]==commit_rob 
//    --> Update rob_or_rf_table[commit_rd]=1'b0


module RAT
#(parameter ROBSIZE = 8)
(
    input logic clk,
    input logic rstn,
    //
    input logic                      i_flush,
    //
    input logic                      i_we,
    input logic [4:0]                i_rd,
    input logic [(ROBSIZE-1):0]      i_rob_addr,
    input logic                      i_rob_or_rf,
    //
    input  logic [4:0]               i_rs1,
    input  logic [4:0]               i_rs2,
    output logic [(ROBSIZE-1):0]     o_rs1_rob_addr,
    output logic [(ROBSIZE-1):0]     o_rs2_rob_addr,
    output logic                     o_rs1_rob_or_rf,
    output logic                     o_rs2_rob_or_rf,
    // COMMIT SIGNALS 
    input logic                      i_commit_en,
    input logic [4:0]                i_commit_rd,
    input logic [(ROBSIZE-1):0]      i_commit_rob_addr
    );
    
// RAT TABLE DEFINITION : ADDR | VALID    
logic [(ROBSIZE-1):0]     rob_addr_table [31:0]; // Address Table Keeps the Address of The ROB Value table 
logic [31:0]              rob_or_rf_table;         // IF ROB_or_RF row is high, it means that value is stored in ROB, otherwise it is in RF          

//////////////////////    
// ADDRESS READ 
/////////////////////
assign o_rs1_rob_addr  = rob_addr_table[i_rs1];
assign o_rs2_rob_addr  = rob_addr_table[i_rs2];
assign o_rs1_rob_or_rf = rob_or_rf_table[i_rs1];   
assign o_rs2_rob_or_rf = rob_or_rf_table[i_rs2];   
    
	
//////////////////////    
// UPDATE THE RAT ADDR COLUMN 
/////////////////////    
    
always_ff @(posedge clk)
begin
   if(i_we & ~(i_rd == 0)) begin
    rob_addr_table[i_rd] <= i_rob_addr;
   end 
end     
    
//////////////////////    
// UPDATE THE RAT rob_or_rf_table 
/////////////////////                
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn) begin
      rob_or_rf_table <= '0;
   end else begin
    if(i_flush)
    begin
      rob_or_rf_table <= '0;
    end else 
       /////// RAT WRITE
       if(i_we && ~(i_rd == 0))
       begin
          rob_or_rf_table[i_rd] <= i_rob_or_rf;
       end
       /////// RAT UPDATE ON COMMIT
       if(i_commit_en && (rob_addr_table[i_commit_rd] == i_commit_rob_addr))
       begin
             rob_or_rf_table[i_commit_rd] <= 1'b0;  
       end 
       ///////
    end   
    
end       
    
    
    
    
    
endmodule :RAT 








module RAT_tb();


parameter ROBSIZE = 8;
    //  
    logic clk;
    logic rstn;
    //
    logic                     i_flush;
    //
    logic                     i_we;
    logic [4:0]               i_rd;
    logic [(ROBSIZE-1):0]     i_rob_addr;
    logic                     i_rob_or_rf;
    //
    logic [4:0]               i_rs1;
    logic [4:0]               i_rs2;
    logic [(ROBSIZE-1):0]     o_rs1_rob_addr;
    logic [(ROBSIZE-1):0]     o_rs2_rob_addr;
    logic                     o_rs1_rob_or_rf;
    logic                     o_rs2_rob_or_rf;
    // COMMIT SIGNALS 
    logic                     i_commit_en;
    logic [4:0]               i_commit_rd;
    logic [(ROBSIZE-1):0]     i_commit_rob_addr;
        
/////////////////////////////////////
// CLOCK GENERATION 
/////////////////////////////////////
initial begin
 clk = 1'b0;
 fork 
  forever #10 clk = ~clk;
 join
end 

////////////////////////////////
// RESET TASK: Generates an asyncrhonous, active low reset   
////////////////////////////////
task RESET();
begin
  i_we    = 1'b0;
  i_flush = 1'b0;
  //
  rstn = 1'b1;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b0;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b1;
  repeat(2) @(posedge clk); #3;
end 
endtask

////////////////////////////
// WRITE 
///////////////////////////
task WRITE(input logic [4:0]                rd,
           input logic [(ROBSIZE-1):0]      rob_addr,
           input logic                      rob_or_rf);
begin          
     @(posedge clk); 
       i_we        <= 1'b1;
       i_rd        <= rd;
       i_rob_addr  <= rob_addr;
       i_rob_or_rf <= rob_or_rf;
     @(posedge clk);
       i_we        <= 1'b0;     
       i_rd        <= 0;    
       i_rob_or_rf <= 0;        
     @(posedge clk);
end            
endtask

////////////////////////////
// RAT COMMIT 
///////////////////////////
task COMMIT (
   input logic [4:0]               commit_rd, 
   input logic [(ROBSIZE-1):0]     commit_rob_addr
);
begin
   @(posedge clk);
      i_commit_en       <= 1'b1;
      i_commit_rd       <= commit_rd;
      i_commit_rob_addr <= commit_rob_addr;
   @(posedge clk);  
      i_commit_en       <= 1'b0;
      i_commit_rd       <= '0;
      i_commit_rob_addr <= '0;     
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



/////////////////////////////////////
// RAT TEST STIMULU 
/////////////////////////////////////
initial begin
RESET();

for(int i=0;i<32;i++)
begin
  WRITE(.rd (i), .rob_addr (i+2), .rob_or_rf (1));
end
DELAY(5);
//
COMMIT ( .commit_rd (5), .commit_rob_addr (12) );
DELAY(5);
//
WRITE(.rd (10), .rob_addr (5), .rob_or_rf (1));
DELAY(5);
COMMIT ( .commit_rd (10), .commit_rob_addr (5) );
//

$finish;
end 




RAT #( .ROBSIZE (ROBSIZE)) uRAT (.*);


endmodule :RAT_tb 

