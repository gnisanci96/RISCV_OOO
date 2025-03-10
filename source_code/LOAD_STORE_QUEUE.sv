`timescale 1ns / 1ps
/////////////////////////////////////////////////////////
//          The Functionality of the Load/Store Queue      
/////////////////////////////////////////////////////////
// 1- New Instruction Write 
//  1.1-If the Data is ready for the Store, Write the Data and set DATA_READY.
// 2- Get Broadcasted Address for Load and Store from ALU. 
// 3- Get Broadcasted Data for Store from ALU.
// 4- Choose a Load and Store and send it to Memory. 
// 5- Get DATA for Load from Memory.
// 6- Send Ready To Commit Signals to Control Unit. [READ ROW will commit] 
// 7- Get Commit Flag and move the Commit Pointer. 
 


module LOAD_STORE_QUEUE
#(parameter QUEUESIZE     = 8,
  parameter ROBSIZE       = 8
  )
(
  input logic                           clk,
  input logic                           rstn,
  // FLUSH THE LOAD STORE QUEUE 
  input logic                           i_flush,
  // NEW INSRUCTION WRITE 
  input logic                           i_new_inst_we,
  input logic [(ROBSIZE-1):0]           i_rob_addr,
  input logic [31:0]                    i_rs2_data,
  input logic [(ROBSIZE-1):0]           i_rs2_rob_addr,   // If i_rs2_data_valid==0, wait for the Broadcasted Data with this ROB 
  input logic                           i_rs2_data_valid, // 1: "i_rs2_data" is ready, no need to wait for Broadcast  
  input logic                           i_load_or_store,  // 0:Load 1:Store 
  input logic                           i_signed_unsigned, 
  input logic [1:0]                     i_size,
  // BROADCAST SIGNALS 
  input logic                           i_broadcast_en,       // An instruction completed execution and waiting for 
  input logic [31:0]                    i_broadcast_data,     // Broadcasted 32-bit Data 
  input logic [(ROBSIZE-1):0]           i_broadcast_rob_addr, // ROB address of the instruction broadcasted 
  input logic                           i_broadcast_addr_flag,// The Broadcasted data is address 
  // COMMIT SIGNALS 
  input  logic                          i_commit_en,
  output logic                          o_commit_ready,
  output logic                          o_commit_load_store,
  output logic [31:0]                   o_commit_data,  
  output logic [(ROBSIZE-1):0]          o_commit_rob_addr,
  //DATA MEMORY SIGNALS 
    // TO MEMORY
  output logic                          o_mem_cs,
  output logic [31:0]                   o_mem_addr,
  output logic [(ROBSIZE-1):0]          o_mem_rob_addr,
  output logic                          o_mem_load_store,
  output logic [31:0]                   o_mem_store_data,
  output logic [1:0]                    o_mem_size,
  output logic                          o_mem_signed_unsigned,
    // FROM MEMORY
  input logic                           i_load_we,
  input  logic                          i_mem_busy,
  // 
  output logic                          o_full, 
  output logic                          o_empty
    );
    
 /////////////////////////////////////////////////
 //  Local Declerations 
 /////////////////////////////////////////////////   
 logic [(ROBSIZE-1):0]           T_ROB_ADDR       [(QUEUESIZE-1):0]; 
 logic [(ROBSIZE-1):0]           T_RS2_ROB_ADDR   [(QUEUESIZE-1):0]; 
 logic [31:0]                    T_ADDRESS        [(QUEUESIZE-1):0]; 
 logic [31:0]                    T_DATA           [(QUEUESIZE-1):0]; 
 logic [(QUEUESIZE-1):0]         T_LOAD_STORE; 
 logic [(QUEUESIZE-1):0]         T_SIGNED_UNSIGNED; 
 logic [1:0]                     T_SIZE           [(QUEUESIZE-1):0]; 
 logic [(QUEUESIZE-1):0]         T_ADDR_READY; 
 logic [(QUEUESIZE-1):0]         T_DATA_READY; 
 logic [(QUEUESIZE-1):0]         T_READY; 
 logic [(QUEUESIZE-1):0]         T_SENT, T_SENT_next; 
 logic [(QUEUESIZE-1):0]         T_VALID; 
 //  
 logic [(ROBSIZE-1):0]           commit_rs2_rob_addr;
 logic [31:0]                    commit_address; 
 logic                           commit_signed_unsigned; 
 logic [1:0]                     commit_size; 
 logic                           commit_addr_ready; 
 logic                           commit_data_ready; 
 logic                           commit_valid;  
 //
 logic                          o_mem_cs_next;
 logic [31:0]                   o_mem_addr_next;
 logic [(ROBSIZE-1):0]          o_mem_rob_addr_next;
 logic                          o_mem_load_store_next;
 logic [31:0]                   o_mem_store_data_next;
 logic [1:0]                    o_mem_size_next;
 logic                          o_mem_signed_unsigned_next;
 
 
// 
//  FIFO Queue Pointers 
//
logic [($clog2(QUEUESIZE)-1):0]   commit_pointer;
logic [($clog2(QUEUESIZE)-1):0]   write_pointer;
logic [($clog2(QUEUESIZE)):0]     count;


///////////////////////////////
//  LOAD/STORE QUEUE TABLE UPDATE 
///////////////////////////////
always_ff @(posedge clk or negedge rstn)
begin
    if(!rstn)
    begin
       T_LOAD_STORE         <= '0; 
       T_SIGNED_UNSIGNED    <= '0;  
       T_ADDR_READY         <= '0; 
       T_DATA_READY         <= '0; 
       T_VALID              <= '0;  
       for(int i =0;i<QUEUESIZE;i++)
       begin
          T_ROB_ADDR[i]     <= '0; 
          T_RS2_ROB_ADDR[i] <= '0; 
          T_ADDRESS[i]      <= '0;  
          T_DATA[i]         <= '0; 
          T_SIZE[i]         <= '0;  
       end 
    end else if(i_flush)   
    begin
       T_ADDR_READY      <= '0; 
       T_DATA_READY      <= '0; 
       T_VALID           <= '0;   
    end else 
    begin
       // NEW INSTRUCTION WRITE 
       if(i_new_inst_we && (~o_full))
       begin
          T_LOAD_STORE[write_pointer]      <= i_load_or_store; 
          T_SIGNED_UNSIGNED[write_pointer] <= i_signed_unsigned;  
          T_ADDR_READY[write_pointer]      <= '0; 
          // 
          if(i_load_or_store) // Check rs2 data for STORE inst.
          begin
             T_DATA_READY[write_pointer]      <= i_rs2_data_valid;
             //
             if(i_rs2_data_valid)
             begin
                T_RS2_ROB_ADDR[write_pointer]    <= '0;
                T_DATA[write_pointer]            <= i_rs2_data;              
             end else begin
                T_RS2_ROB_ADDR[write_pointer]    <= i_rs2_rob_addr;
                T_DATA[write_pointer]            <= '0;     
             end 
             //
          end else begin
             // Data will will come from memory for LOAD insts.
             T_DATA_READY[write_pointer]      <= '0; 
             T_RS2_ROB_ADDR[write_pointer]    <= '0;
             T_DATA[write_pointer]            <= '0;           
          end 
          //
          T_VALID[write_pointer]           <= 1'b1; 
          T_ROB_ADDR[write_pointer]        <= i_rob_addr; 
          T_ADDRESS[write_pointer]         <= '0;  
          T_SIZE[write_pointer]            <= i_size;  
       end
       ////////
       // BROADCAST    
       ////////
       if(i_broadcast_en)
       begin 
          if(i_broadcast_addr_flag)
          begin
          
            for(int k =0;k<QUEUESIZE;k++)
            begin
               if((i_broadcast_rob_addr==T_ROB_ADDR[k]) && T_VALID[k] &&  ~T_ADDR_READY[k])
               begin
                  T_ADDRESS[k]    <= i_broadcast_data;
                  T_ADDR_READY[k] <= 1'b1;
               end 
            end 
            
          end else 
          begin
               // GET THE RS2 VALUE FOR STORE INSTRUCTIONS 
               for(int k =0;k<QUEUESIZE;k++)
               begin
                  if((i_broadcast_rob_addr==T_RS2_ROB_ADDR[k]) && T_VALID[k] &&  ~T_DATA_READY[k] && T_LOAD_STORE[k])
                  begin
                     T_DATA[k]       <= i_broadcast_data;
                     T_DATA_READY[k] <= 1'b1;
                  end 
               end 
             // GET THE DATA FOR LOAD INSTRUCTIONS 
               for(int k =0;k<QUEUESIZE;k++)
               begin
                  if((i_broadcast_rob_addr==T_ROB_ADDR[k]) && T_VALID[k] &&  ~T_DATA_READY[k] && ~T_LOAD_STORE[k])
                  begin
                     T_DATA[k]       <= i_broadcast_data;
                     T_DATA_READY[k] <= 1'b1;
                  end 
               end      
     
          end 
          //
       end // if(i_broadcast_en)

    end 
end 


//=========
// READY SIGNAL GENERATION  
//=========
always_comb
begin
   for(int j =0;j<QUEUESIZE;j++)
   begin
      T_READY[j] <= T_ADDR_READY[j] & T_DATA_READY[j];
   end 
end 


///////
//  COMMIT SIGNALS 
//////
assign o_commit_rob_addr      = T_ROB_ADDR[commit_pointer];
assign commit_rs2_rob_addr    = T_RS2_ROB_ADDR[commit_pointer];
assign commit_address         = T_ADDRESS[commit_pointer];
assign o_commit_data          = T_DATA[commit_pointer];
assign o_commit_load_store    = T_LOAD_STORE[commit_pointer];
assign commit_signed_unsigned = T_SIGNED_UNSIGNED[commit_pointer]; 
assign commit_size            = T_SIZE[commit_pointer];
assign commit_addr_ready      = T_ADDR_READY[commit_pointer];
assign commit_data_ready      = T_DATA_READY[commit_pointer]; 
assign o_commit_ready         = T_READY[commit_pointer];
assign commit_valid           = T_VALID[commit_pointer];
   
///////
//  |CHOOSE INSTRUCTION TO SEND TO THE DATA MEMORY| 
//////  
// If the commit instruction is store --> Wait for the commit_en  
// If the commit instruction is load  --> Send it to memory if address is ready or wait for the address  
// 
// 
// 
//////   


always_comb
begin
   o_mem_cs_next               = 1'b0;
   o_mem_addr_next             = commit_address;
   o_mem_rob_addr_next         = o_commit_rob_addr;
   o_mem_load_store_next       = o_commit_load_store;
   o_mem_store_data_next       = o_commit_data;
   o_mem_size_next             = commit_size;
   o_mem_signed_unsigned_next  = commit_signed_unsigned;
   T_SENT_next                 = T_SENT;
   //
   if(o_commit_load_store) // --> Commit inst is STORE 
   begin
      if(i_commit_en)    // --> Wait for the commit_en from the control_unit
      begin
         o_mem_cs_next               = 1'b1;
      end 
   end else begin       // --> Commit inst is LOAD
      // No need to wait for the Commit_en from the control unit 
      // 
      if(commit_addr_ready && ~i_mem_busy && ~T_SENT[commit_pointer])
      begin
         o_mem_cs_next                = 1'b1;
         T_SENT_next[commit_pointer]  = 1'b1;      
      end 
      //
      if(i_commit_en)
      begin
         T_SENT_next[commit_pointer]  = 1'b0;
      end 
   end 
   //
end 




always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      o_mem_cs              <= '0;
      o_mem_rob_addr        <= '0;
      o_mem_load_store      <= '0;
      o_mem_store_data      <= '0;
      o_mem_size            <= '0;
      o_mem_signed_unsigned <= '0; 
      T_SENT                <= '0;
      o_mem_addr            <= '0;
   end else begin
      o_mem_cs              <= o_mem_cs_next;
      o_mem_rob_addr        <= o_mem_rob_addr_next;
      o_mem_load_store      <= o_mem_load_store_next;
      o_mem_store_data      <= o_mem_store_data_next;
      o_mem_size            <= o_mem_size_next;
      o_mem_signed_unsigned <= o_mem_signed_unsigned_next;
      T_SENT                <= T_SENT_next;
      o_mem_addr            <= o_mem_addr_next;
   end 
end 



  
  
  
/////////////////////////////////////////////////////////
//  Update Read Pointer  
/////////////////////////////////////////////////////////
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
    commit_pointer <= 0 ;
   end else if(i_flush) begin
    commit_pointer <= 0 ;
   end else if(i_commit_en && ~o_empty) begin
    commit_pointer <= commit_pointer + 1;
   end 
end 
 
/////////////////////////////////////////////////////////
//  Update Write Pointer  
///////////////////////////////////////////////////////// 
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
    write_pointer <= 0 ;
   end else if(i_flush) begin
    write_pointer <= 0 ;
   end else if(i_new_inst_we && ~o_full) begin
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

always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
     count <= 0;
   end else if(i_flush) begin
     count <= 0;
   end else if ( (i_new_inst_we && i_commit_en) && o_full)
   begin
     count <= count -1;
   end else if ( (i_new_inst_we&& i_commit_en) && o_empty)
   begin
     count <= count + 1;
   end else if ( (i_new_inst_we&& i_commit_en))
   begin
     count <= count;
   end else if ( i_new_inst_we && ~o_full) 
   begin
     count <= count + 1;
   end else if (i_commit_en && ~o_empty)
   begin
     count <= count - 1;
   end 
end 

assign o_full  = (count == (QUEUESIZE));
assign o_empty = (count == 0);
     
endmodule :LOAD_STORE_QUEUE



//////////////////////////////////////
//  LOAD_STORE_QUEUE_tb 
/////////////////////////////////////


module LOAD_STORE_QUEUE_tb();


parameter QUEUESIZE     = 8;
parameter ADDRSIZE_bit  = 32;
parameter VALUESIZE_bit = 32;
parameter ROBSIZE       = 8;

logic                             clk;
logic                             rstn;
logic                           i_flush;
// NEW INSRUCTION WRITE 
logic                           i_new_inst_we;
logic [(ROBSIZE-1):0]       i_rob_addr;
logic [31:0]                    i_rs2_data;
logic [(ROBSIZE-1):0]       i_rs2_rob_addr;
logic                           i_rs2_data_valid;
logic                           i_load_or_store; // 0:Load 1:Store 
logic                           i_signed_unsigned;
logic [1:0]                     i_size;
//BROADCAST SIGNALS 
logic                           i_broadcast_en;
logic [31:0]                    i_broadcast_data;
logic [(ROBSIZE-1):0]       i_broadcast_rob_addr;
logic                           i_broadcast_addr_flag;
// COMMIT SIGNALS 
logic                           i_commit_en;
logic                           o_commit_ready;
logic                           o_commit_load_store;
logic [31:0]                    o_commit_data;  
logic [(ROBSIZE-1):0]       o_commit_rob_addr;
//DATA MEMORY SIGNALS 
// TO MEMORY
logic                           o_mem_cs;
logic [(ROBSIZE-1):0]       o_mem_rob_addr;
logic [31:0]                    o_mem_addr;
logic                           o_mem_load_store;
logic [31:0]                    o_mem_store_data;
logic [1:0]                     o_mem_size;
logic                           o_mem_signed_unsigned;
// FROM MEMORY
logic                           i_load_we;
logic                           i_mem_busy;
logic [(ROBSIZE-1):0]       i_load_rob_addr; 
logic [31:0]                    i_load_data;
// 
logic                           o_full; 
logic                           o_empty;



 
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
   i_flush         <= 1'b0;
   i_new_inst_we   <= 1'b0;
   i_load_or_store <= 1'b0;
   i_rob_addr      <= '0;
   i_mem_busy      <= '0;
   i_commit_en     <= 1'b0; 
   //BROADCAST SIGNALS 
   i_broadcast_en        <= '0;
   i_broadcast_data      <= '0;
   i_broadcast_rob_addr  <= '0;
   i_broadcast_addr_flag <= '0;
   i_load_we             <= '0;
   i_load_rob_addr       <= '0;
   i_load_data           <= '0;
  // 
  rstn = 1'b1;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b0;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b1;
  repeat(2) @(posedge clk); #3;
end 
endtask

//==============================
// WRITE NEW INSTRUCTION TO THE LOAD & STORE QUEUE   
//==============================
task WRITE(
 input logic [(ROBSIZE-1):0]       rob_addr,
 input logic [31:0]                    rs2_data,
 input logic [(ROBSIZE-1):0]       rs2_rob_addr,
 input logic                           rs2_data_valid,
 input logic                           load_or_store,   // 0:Load 1:Store 
 input logic                           signed_unsigned,
 input logic [1:0]                     size
);
begin
   @(posedge clk);
      i_new_inst_we      <= 1'b1;
      i_rob_addr         <= rob_addr;
      i_load_or_store    <= load_or_store; // 0:Load 1:Store 
      i_signed_unsigned  <= signed_unsigned;
      i_size             <= size;
   @(posedge clk);
      i_new_inst_we      <= 1'b0;
      i_rob_addr         <= '0;
      i_load_or_store    <= '0; // 0:Load 1:Store 
      i_signed_unsigned  <= '0;
      i_size             <= '0;
end 
endtask

//==============================
// BROADCAST DATA from ALU. The Data maybe address for load/store or int value for store.  
//==============================
task BROADCAST (
  input logic [31:0]                    broadcast_data,
  input logic [(ROBSIZE-1):0]       broadcast_rob_addr,
  input logic                           broadcast_addr_flag // 1: Data is an Address for Load/Store 
);
begin
   @(posedge clk);
     //BROADCAST SIGNALS 
     i_broadcast_en        <= 1'b1;
     i_broadcast_data      <= broadcast_data;
     i_broadcast_rob_addr  <= broadcast_rob_addr;
     i_broadcast_addr_flag <= broadcast_addr_flag;
   @(posedge clk); 
     i_broadcast_en        <= 1'b0;
     i_broadcast_data      <= '0;
     i_broadcast_rob_addr  <= '0;
     i_broadcast_addr_flag <= '0; 
end 
endtask 

//==============================
// LOAD DATA WRITE 
//==============================
task LOAD_DATA_WRITE(
   input logic [(ROBSIZE-1):0]       load_rob_addr,
   input logic [31:0]                    load_data
);
begin
    @(posedge clk);
       i_load_we             <= 1'b1;
       i_load_rob_addr       <= load_rob_addr;
       i_load_data           <= load_data;  
    @(posedge clk);
       i_load_we             <= 1'b0;
       i_load_rob_addr       <= load_rob_addr;
       i_load_data           <= load_data;  
end 
endtask 



//==============================
// DELAY  
//==============================
task DELAY(input int delay_cc);
begin
  repeat(delay_cc) @(posedge clk);
end 
endtask

//==============================
// FLUSH 
//==============================
task FLUSH();
begin
  @(posedge clk);
     i_flush = 1'b1;
  @(posedge clk);     
     i_flush = 1'b0;  
end 
endtask

/////////////////////////////////////
// LOAD/STORE QUEUE TEST STIMULU 
/////////////////////////////////////
initial begin
  RESET();
  //
  WRITE( .rob_addr  (0), .rs2_data  ('0), .rs2_rob_addr (5),.rs2_data_valid (1),.load_or_store (0), .signed_unsigned (0),.size (2) );
  WRITE( .rob_addr  (1), .rs2_data  ('0), .rs2_rob_addr (5),.rs2_data_valid (1),.load_or_store (0), .signed_unsigned (0),.size (2) );
  WRITE( .rob_addr  (2), .rs2_data  ('0), .rs2_rob_addr (5),.rs2_data_valid (1),.load_or_store (0), .signed_unsigned (0),.size (2) );
  //
  BROADCAST ( .broadcast_data (32'h1234), .broadcast_rob_addr (0), .broadcast_addr_flag (1) );
  BROADCAST ( .broadcast_data (32'h5678), .broadcast_rob_addr (1), .broadcast_addr_flag (1) );
  BROADCAST ( .broadcast_data (32'h9abc), .broadcast_rob_addr (2), .broadcast_addr_flag (1) );
  //
  LOAD_DATA_WRITE( .load_rob_addr (5), .load_data (12) );
  //
  DELAY(10);
  //
  FLUSH();
  //
  $finish;
end 








///////////////////////////////////
// Module Instantiation 
//////////////////////////////////

LOAD_STORE_QUEUE
#( .QUEUESIZE       (8),
   .ADDRSIZE_bit    (32),
   .VALUESIZE_bit   (32)
  )
uLOAD_STORE_QUEUE (.*);





endmodule :LOAD_STORE_QUEUE_tb


