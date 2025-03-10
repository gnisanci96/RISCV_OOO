`timescale 1ns / 1ps
///////////////////////////////////////////
// Synchronous FIFO Design With Wrap Around(When the Fifo is FULL in continous Writing by Wrapping Around the FIFO)


module RAS
#(parameter DATAWIDTH = 40,
  parameter FIFODEPTH = 4)
(
  input logic clk,
  input logic rstn,
  //
  input logic                    i_ras_write_en,
  input logic [(DATAWIDTH-1):0]  i_ras_write_pc,
  //
  input  logic                   i_ras_read_en,
  output logic [(DATAWIDTH-1):0] o_ras_read_pc,             
  //
  output logic                   o_full,
  output logic                   o_empty
    );
    
    
logic [(2**FIFODEPTH-1):0][(DATAWIDTH-1):0]    T_PC; 
    
logic [(FIFODEPTH):0]                          counter;

logic [(FIFODEPTH-1):0]                        read_ptr;
logic [(FIFODEPTH-1):0]                        write_ptr;


//////////////////////////////////////
//  MEMORY READ/WRITE REGISTER 
/////////////////////////////////////
always_ff @(posedge clk)
begin
   if(i_ras_write_en)
   begin
     T_PC[write_ptr] <= i_ras_write_pc;
   end 
end 
    

assign o_ras_read_pc = T_PC[read_ptr];


//////////////////////////////////////
//  Pointer Calculations 
/////////////////////////////////////
///////////
//  WRITE POINTER  
///////////
always_ff @(posedge clk or negedge rstn)
begin 
   if(!rstn)
   begin
         write_ptr <= 0;
   end else begin
      if(i_ras_write_en)
      begin
         write_ptr <= write_ptr + 1;
      end 
   end 
end 
///////////
//  READ POINTER  
///////////
always_ff @(posedge clk or negedge rstn)
begin 
   if(!rstn)
   begin
         read_ptr <= 0;
   end else begin
      if(i_ras_read_en)
      begin
         read_ptr <= read_ptr + 1;
      end 
   end 
end 

//=========================
//-----Counter Update------ 
//=========================
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
     counter <= 0;
   end else if ( (i_ras_write_en && i_ras_read_en) && o_empty)
   begin
     counter <= counter + 1;
   end else if ( (i_ras_write_en && i_ras_read_en))
   begin
     counter <= counter;
   end else if (i_ras_write_en && ~o_full) 
   begin
     counter <= counter + 1;
   end else if ( i_ras_read_en && ~o_empty)
   begin
     counter <= counter - 1;
   end 
end 

assign o_full  = (counter == (2**FIFODEPTH));
assign o_empty = (counter == 0);



    
    
endmodule :RAS




//===================================
//----------RAS TEST BENCH-----------
//===================================
module RAS_tb();

parameter DATAWIDTH = 40;
parameter FIFODEPTH = 3;


logic                   clk;
logic                   rstn;
//
logic                   i_ras_write_en;
logic [(DATAWIDTH-1):0] i_ras_write_pc;
//
logic                   i_ras_read_en;
logic [(DATAWIDTH-1):0] o_ras_read_pc;            
//
logic                   o_full;
logic                   o_empty;
//
int error;

////////////////////
// CLOCK GENERATION
///////////////////
initial begin
clk = 1'b0;
fork
 forever #10 clk = ~clk;
join
end 

////////////////////
// RESET TASK
///////////////////
task RESET();
begin
   error = '0;
   //
   i_ras_write_en = '0;
   i_ras_write_pc = '0;
   i_ras_read_en  = '0;
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2) @(posedge clk);
   rstn = 1'b1;
end 
endtask 

////////////////////////
//  PUSH TASK (Pushes Data Into the FIFO Mem)
//////////////////////
task PUSH(input logic [(DATAWIDTH-1):0] pc);
begin
  @(posedge clk);
   i_ras_write_en <= 1'b1;
   i_ras_write_pc <= pc;
   i_ras_read_en  <= 1'b0;
  @(posedge clk);
   i_ras_write_en <= '0;
   i_ras_write_pc <= '0;
   i_ras_read_en  <= '0;
end 
endtask 

/////////////////////
//  PULL TASK 
////////////////////
task PULL(input logic [(DATAWIDTH-1):0] ex_pc);
begin
  @(posedge clk);
   i_ras_read_en  <= 1'b1;
     if(~o_empty) 
     begin  
        if(ex_pc != o_ras_read_pc) begin   error++; $display("DATA ERROR ! Expected %h Actual:%h ",ex_pc,o_ras_read_pc); end 
        else                       begin            $display("THE DATA PULLED CORRECTLY %h",o_ras_read_pc);              end 
     end 
  @(posedge clk);
   i_ras_read_en  <= 1'b0;
end 
endtask 


task test1();
begin
//parameter DATAWIDTH = 40;
//parameter FIFODEPTH = 3;
  RESET();
  //
  PUSH(1);
  PUSH(2);
  PUSH(3);
  PUSH(4);
  PUSH(5);
  PUSH(6);
  PUSH(7);
  PUSH(8);
  //-----OverWrite-----
  PUSH(9);
  PUSH(10);
  PUSH(11);
  PUSH(12);
  PUSH(13);
  PUSH(14);
  PUSH(15);
  PUSH(16);
  //-----PULLDATA------ 
  PULL(9);
  PULL(10);
  PULL(11);
  PULL(12);
  PULL(13);
  PULL(14);
  PULL(15);
  PULL(16);
  //
  if(error==0) begin $display("TEST-1 PASS");  end 
  else         begin $display("TEST-2 FAIL");  end 
  //
end 
endtask 






initial begin
   test1();
  // 
  /*PULL(1);
  PULL(2);
  PULL(3);
  PULL(4);
  PULL(5);
  PULL(6);
  PULL(7);
  PULL(8);
  PULL(9);
  PULL(10);
  //
  PUSH('hB);
  PUSH('hA);
  PUSH('hB);
  PUSH('hE);
  PUSH('hC);
  PUSH('hA);
  PUSH('hF);
  PUSH('hE);
  PUSH('hA);
  PUSH('hB);
  //
  PULL();
  PULL();
  PULL();
  PULL();
  PULL();
  PULL();
  PULL();
  PULL();
  PULL();
  PULL();*/
  //
  $finish; 
end 


////////////////////////
//  DUT INSTANTIATION 
///////////////////////
RAS
#( .DATAWIDTH (DATAWIDTH),
   .FIFODEPTH (FIFODEPTH))
uRAS
(.*);
    


endmodule :RAS_tb 