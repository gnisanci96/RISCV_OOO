`timescale 1ns / 1ps
/////////////////////////////////////////
// INSTRUCTION_QUEUE MODULE FUNCTIONALITY 
/////////////////////////////////////////
// The Queue will work as Fifo with a Flush Functinality 
//

module INSTRUCTION_QUEUE
#(parameter DATAWITHBIT=32,
  parameter FIFOSIZE=8)
(
    input logic clk,
    input logic rstn,
    //
    input logic                      i_flush,
    //
    input logic                      i_wrt_en,
    input logic [(DATAWITHBIT-1):0]  i_wrt_data,
    input logic [31:0]               i_wrt_inst_pc,
    input logic                      i_wrt_taken,
    input logic [31:0]               i_wrt_target,
    //
    input logic                      i_rd_en,
    output logic [(DATAWITHBIT-1):0] o_rd_data,
    output logic [31:0]              o_rd_inst_pc,
    output logic                     o_rd_taken,
    output logic [31:0]              o_rd_target,
    //
    output logic                     o_full,
    output logic                     o_empty
 );

///////////////////////////////
//  LOGIC DECLERATION 
///////////////////////////////
logic [(FIFOSIZE-1):0][(DATAWITHBIT-1):0] fifomem   ;
logic [(FIFOSIZE-1):0][31:0]              T_INST_PC ;
logic [(FIFOSIZE-1):0]                    T_TAKEN;
logic [(FIFOSIZE-1):0][31:0]              T_TARGET;

logic [($clog2(FIFOSIZE)-1):0]   read_pointer;
logic [($clog2(FIFOSIZE)-1):0]   write_pointer;
logic [($clog2(FIFOSIZE)):0]     count, count_next;
//

///////////////////////////////
//  FIFO MEMORY READ & WRITE OPERATION 
///////////////////////////////

always_ff @(posedge clk or negedge rstn)
begin
    if(!rstn)
    begin
        fifomem   <= '0;
        T_INST_PC <= '0;
        T_TAKEN   <= '0;
        T_TARGET  <= '0;
    end else if(i_wrt_en && (~o_full) && (~i_flush))
     begin
        fifomem[write_pointer]    <= i_wrt_data; 
        T_INST_PC[write_pointer]  <= i_wrt_inst_pc;
        T_TAKEN[write_pointer]    <= i_wrt_taken;
        T_TARGET[write_pointer]   <= i_wrt_target;     
     end
end 

assign o_rd_data    = fifomem[read_pointer]; 
assign o_rd_inst_pc = T_INST_PC[read_pointer];
assign o_rd_taken   = T_TAKEN[read_pointer]; 
assign o_rd_target  = T_TARGET[read_pointer]; 


///////////////////////////////
//  FIFO MEMORY READ POINTER GENERATION
///////////////////////////////
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
    read_pointer <= 0 ;
   end else if(i_flush) begin
    read_pointer <= 0 ;
   end else if(i_rd_en && ~o_empty) begin
    read_pointer <= read_pointer + 1;
   end 
end 

///////////////////////////////
//  FIFO MEMORY Write POINTER GENERATION
///////////////////////////////

always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
    write_pointer <= 0 ;
   end else if(i_flush) begin
    write_pointer <= 0 ;
   end else if(i_wrt_en && ~o_full) begin
    write_pointer <= write_pointer + 1;
   end 
end 

///////////////////////////////
//  FIFO MEMORY Count Generation 
///////////////////////////////
// Count Functionality 
//(1) increments on write operations
//(2) decrements on read operations
//(3) neither increments nor decrements on simultaneous write-read operations
//(4) will not increment on write when already full
//(5) will not decrement on read when already empty
//(6) simultaneous write-read operations will only increment or decrement if FIFO is already full or empty
//(7) FIFO is empty when count=0
//(8) FIFO is full when count=(2**n)

always_comb 
begin
   count_next = count;
   
   if(i_flush) begin
     count_next = 0;
   end else if ( (i_wrt_en && i_rd_en) && o_full)
   begin
     count_next = count -1;
   end else if ( (i_wrt_en && i_rd_en) && o_empty)
   begin
     count_next = count + 1;
   end else if ( (i_wrt_en && i_rd_en))
   begin
     count_next = count;
   end else if ( i_wrt_en && ~o_full) 
   begin
     count_next = count + 1;
   end else if (i_rd_en && ~o_empty)
   begin
     count_next = count - 1;
   end 
end 

always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      count <= '0;
   end else begin
      count <= count_next;
   end 
end 




assign o_full  = (count == (FIFOSIZE));
assign o_empty = (count == 0);

endmodule :INSTRUCTION_QUEUE




integer testcount =0;



//////////////////////////////////////////
//////////////////////////////////////////
//              TEST BENCH 
//////////////////////////////////////////
//////////////////////////////////////////


module INSTRUCTION_QUEUE_tb ();


parameter DATAWITHBIT=32;
parameter FIFOSIZE=4;

logic clk;
logic rstn;
    //
logic                      i_flush;
    //
logic                      i_wrt_en;
logic [(DATAWITHBIT-1):0]  i_wrt_data;
logic [31:0]               i_wrt_inst_pc;
logic                      i_wrt_taken;
    //
logic                      i_rd_en;
logic [(DATAWITHBIT-1):0]  o_rd_data;
logic [31:0]               o_rd_inst_pc;
logic                      o_rd_taken;
    //
logic                      o_full;
logic                      o_empty;


logic flag; 
logic [($clog2(FIFOSIZE)):0]     count;
 ////////////////////////////////
// CLOCK GENERATION 
////////////////////////////////
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
  rstn = 1'b1;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b0;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b1;
  repeat(2) @(posedge clk); #3;
end 
endtask

////////////////////////////////
// FLUSH TASK: Generates a synchronous Flush Signal 
////////////////////////////////
task FLUSH();
begin
  @(posedge clk);
    i_flush = 1'b1;
  @(posedge clk);
    i_flush = 1'b0;
end 
endtask




//////////////////////////////////////
// TASK: Write to the Fifo 
/////////////////////////////////////
task PUSH( input logic  [(DATAWITHBIT-1):0]  pushdata);
begin
   @(posedge clk);
     i_wrt_en   = 1'b1;
     i_wrt_data = pushdata;
   @(posedge clk);
     i_wrt_en   = 1'b0;
end 
endtask
//////////////////////////////////////
// TASK: PULL data from the FIFO 
/////////////////////////////////////
task PULL();
begin
  @(posedge clk); #1;
    i_rd_en = 1'b1;
    $display("PULL: %h",o_rd_data);  
  @(posedge clk); #1; 
    i_rd_en = 1'b0;
end 
endtask
//////////////////////////////////////
// TASK: PULL data from the FIFO 
/////////////////////////////////////
task PUSHPULL( input logic  [(DATAWITHBIT-1):0]  pushdata);
begin
    @(posedge clk); #1;
      i_rd_en  = 1'b1;
      i_wrt_en = 1'b1;
      i_wrt_data = pushdata;
      $display("PULL: %h",o_rd_data);  
    @(posedge clk); #1; 
      i_rd_en  = 1'b0;
      i_wrt_en = 1'b0;
end 
endtask





//////////////////////////////////////
// TASK: TEST FLAG CHECK
/////////////////////////////////////
task flagcheck( input logic flag);
begin
    if(flag) 
      $display("TEST %d :PASS",testcount ); 
    else 
      $display("TEST %d :FAIL",testcount ); 
      
    testcount++;
end 
endtask 




//////////////////////////////////////
// TEST STIMULUS 
/////////////////////////////////////

/////////////////
// TEST 1
/////////////////

initial begin
// INITIALIZE ALL THE INPUTS 
i_flush    =0;
i_wrt_en   =0;
i_wrt_data =0;
i_rd_en    =0;
RESET();
flag = 0;
//
// TEST 1
//
// After Reset The FIFO should be empty 
$display("THE TEST %d",testcount);
flag = (o_empty == 1'b1 && o_full == 1'b0);
flagcheck(flag);
//
// TEST 2
//
// PUSH DATA and try to overflow 
//
$display("THE TEST %d",testcount);
for(int i=1;i<FIFOSIZE+5;i++)
begin
  PUSH(i);
end 
//
flag = 1'b1;
for(int i =0;i<FIFOSIZE;i++)
begin
  flag = (flag & uINSTRUCTION_QUEUE.fifomem[i] == (i+1));
end 
flagcheck(flag);
//
// TEST 3
//
// PULL DATA and try pulling more than pushed  
//
$display("THE TEST %d",testcount);
for(int i=0;i<(FIFOSIZE+5);i++)
begin
  PULL();
end 
//
// TEST 4 
//
//
// PULL PUSHED At the same time when the FIFO is empty 
//
$display("THE TEST %d",testcount);
PUSHPULL(32'h34);
flag = (uINSTRUCTION_QUEUE.fifomem[0] == 32'h34);
flagcheck(flag);
//
// TEST 4 
//
//
// PULL PUSHED At the same time when the FIFO is not Empty or Full
// We should both push and pull and FIFO uINSTRUCTION_QUEUE.count should not change. 
//
$display("THE TEST %d",testcount);
count = uINSTRUCTION_QUEUE.count;
PUSHPULL(32'h35);
flag = ((uINSTRUCTION_QUEUE.fifomem[1] == 32'h35) && (count == uINSTRUCTION_QUEUE.count));
flagcheck(flag);


//
// TEST  
//
//
// PULL PUSHED At the same time when the FIFO is Full (It should read it only)
//
$display("THE TEST %d",testcount);
for(int i=1;i<FIFOSIZE+5;i++)
begin
  PUSH(i+35);
end 
PUSHPULL(32'h55);
//
// TEST  
//
//
// Flush The FIFO
//
repeat(10) @(posedge clk);
FLUSH();
flag = ((uINSTRUCTION_QUEUE.read_pointer==0) & (uINSTRUCTION_QUEUE.write_pointer==0) & (uINSTRUCTION_QUEUE.count==0));
flagcheck(flag);




$display("THE END");
repeat(10) @(posedge clk);
$finish;
end 



 
 
 
 
 INSTRUCTION_QUEUE 
 #(.DATAWITHBIT (DATAWITHBIT), 
   .FIFOSIZE (FIFOSIZE)     )
 uINSTRUCTION_QUEUE 
 (.*);




endmodule :INSTRUCTION_QUEUE_tb 
