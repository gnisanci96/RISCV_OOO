`timescale 1ns / 1ps
///////////////////////////////////////////
// Synchronous FIFO Memory 

module FIFO_DUAL_DATA 
#(parameter FIFODEPTH = 64,
  parameter DATA0SIZE = 32, 
  parameter DATA1SIZE = 8 )
(
  input logic clk,
  input logic rstn,
  //
  input logic                      i_WrtEn,
  input logic [(DATA0SIZE-1):0]    i_data0,
  input logic [(DATA1SIZE-1):0]    i_data1, 
  //
  input  logic                     i_RdEn,
  output logic [(DATA0SIZE-1):0]   o_data0,      
  output logic [(DATA1SIZE-1):0]   o_data1,        
  //
  output logic                     o_Full,
  output logic                     o_Empty
    );
    
    
logic [(2**FIFODEPTH-1):0][(DATA0SIZE-1):0] T_DATA0; 
logic [(2**FIFODEPTH-1):0][(DATA1SIZE-1):0] T_DATA1; 
    
logic [(FIFODEPTH):0]      counter;

logic [(FIFODEPTH-1):0]    RdPtr;
logic [(FIFODEPTH-1):0]    WrtPtr;


//////////////////////////////////////
//  MEMORY READ/WRITE REGISTER 
/////////////////////////////////////
always_ff @(posedge clk)
begin
   if(i_WrtEn && ~o_Full)
   begin
     T_DATA0[WrtPtr] <= i_data0;
     T_DATA1[WrtPtr] <= i_data1;
   end 
end 
    

assign o_data0 = T_DATA0[RdPtr];
assign o_data1 = T_DATA1[RdPtr];


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
         WrtPtr <= 0;
   end else begin
      if(i_WrtEn && ~o_Full)
      begin
         WrtPtr <= WrtPtr + 1;
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
         RdPtr <= 0;
   end else begin
      if(i_RdEn && ~o_Empty)
      begin
         RdPtr <= RdPtr + 1;
      end 
   end 
end 

///////////
//  Counter Calculation 
///////////
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
     counter <= 0;
   end else if ( (i_WrtEn && i_RdEn) && o_Full)
   begin
     counter <= counter -1;
   end else if ( (i_WrtEn && i_RdEn) && o_Empty)
   begin
     counter <= counter + 1;
   end else if ( (i_WrtEn && i_RdEn))
   begin
     counter <= counter;
   end else if ( i_WrtEn && ~o_Full) 
   begin
     counter <= counter + 1;
   end else if (i_RdEn && ~o_Empty)
   begin
     counter <= counter - 1;
   end 
end 

assign o_Full  = (counter == (2**FIFODEPTH));
assign o_Empty = (counter == 0);



    
    
endmodule :FIFO_DUAL_DATA


module FIFO_DUAL_DATA_tb ();

parameter FIFODEPTH = 64;
parameter DATA0SIZE = 32; 
parameter DATA1SIZE = 8; 


logic                       clk;
logic                       rstn;
//
logic                     i_WrtEn;
logic [(DATA0SIZE-1):0]   i_data0;
logic [(DATA1SIZE-1):0]   i_data1; 
//								 
logic                     i_RdEn;
logic [(DATA0SIZE-1):0]   o_data0;      
logic [(DATA1SIZE-1):0]   o_data1;         
//
logic                     o_Full;
logic                     o_Empty;

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
   i_WrtEn = '0;
   i_data0 = '0;
   i_data1 = '0;
   i_RdEn  = '0;
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
task PUSH(input logic [(DATA0SIZE-1):0] data0, input logic [(DATA0SIZE-1):0] data1);
begin
  @(posedge clk);
   i_WrtEn = 1'b1;
   i_data0 = data0;
   i_data1 = data1;
   i_RdEn  = '0;
  @(posedge clk);
   i_WrtEn = '0;
   i_data0 = '0;
   i_data1 = '0;
   i_RdEn  = '0;
end 
endtask 

/////////////////////
//  PULL TASK 
////////////////////
task PULL();
begin
  @(posedge clk); #1;
     i_RdEn = 1'b1;
     if(~o_Empty) begin $display("THE DATA0 PULLED %h THE DATA1 PULLED %h",o_data0,o_data1); end 
  @(posedge clk); #1;
     i_RdEn = 1'b0; 
     
end 
endtask 


initial begin
  RESET();
  PUSH(1,2);
  PUSH(2,3);
  PUSH(3,4);
  PUSH(4,5);
  PUSH(5,6);
  PUSH(6,7);
  PUSH(7,8);
  PUSH(8,9);
  PUSH(9,10);
  PUSH(10,11);
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
  PULL();
  //
  $finish; 
end 


////////////////////////
//  DUT INSTANTIATION 
///////////////////////
FIFO_DUAL_DATA
#( .FIFODEPTH (FIFODEPTH),
   .DATA0SIZE (DATA0SIZE), 
   .DATA1SIZE (DATA1SIZE) )
uFIFO_DUAL_DATA
(.*);
    


endmodule :FIFO_DUAL_DATA_tb 