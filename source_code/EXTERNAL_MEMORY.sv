`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// EXTERNAL MEMORY 


module EXTERNAL_MEMORY
#(parameter BLOCKSIZE=64,    // Number of Blocks 
  parameter BLOCKWORDSIZE=4) // Number of Words(32-bit) in each Block  
(
   input  logic        clk,            // Clock    
   input  logic        rstn,           // Active low Asyncronous Reset 
   input  logic        i_cs,           // CHIP SELECT  
   input  logic        i_write_read,   // 1:Write 0:Read 
   input  logic [31:0] i_addr,         // READ/WRITE ADDR 
   input  logic [31:0] i_write_data,   // WRITE DATA WORD
   input  logic        i_write_data_v, // WRITE DATA VALID
   output logic        o_write_ready,
   output logic [31:0] o_read_data,    // READ DATA WORD 
   output logic        o_read_data_v,  // READ DATA VALID 
   output logic        o_burst_done 
    );


//===========================
// Memory Organization 
//===========================
//         |                 |
//---------------------------|
//         |           word0 |
//         |BLOCK 0    word1 |
//         |        word(M-1)|
//---------------------------|
//         |                 |
//         |  BLOCK 1        |
//         |                 |
//        ......
//         |                 |
//         |  BLOCK (N-1)    |
//         |                 |
//---------------------------|



//===========================
// LOCAL SIGNALS
//===========================
localparam OFFSETSIZE = $clog2(BLOCKWORDSIZE*4);        // Number of Bits we need to represent each Byte in a Block
logic [(BLOCKSIZE-1):0][(BLOCKWORDSIZE-1):0][31:0] MEM;// The External Memory 
logic [($clog2(BLOCKSIZE)-1):0]   block_addr;           // The Address of the BLock
logic [$clog2(BLOCKWORDSIZE):0]   word_cnt;             // Counter used for Burst Read/Write Operation    
 
 
    
typedef enum{
   IDLE, 
   BURST_READ, 
   BURST_WRITE 
} state_type;

state_type state, state_next; 







//=========================
// STATE DIAGRAM
//=========================
always_ff @(posedge clk or negedge rstn) 
begin
   
   if(!rstn)
   begin
      word_cnt      <= '0;
      block_addr    <= '0;
      o_read_data   <= '0;
      o_read_data_v <= '0;
      o_burst_done  <= '0;
      o_write_ready <= '0;
      //    
   end else begin
      case(state)
         IDLE:
            begin  
               o_read_data_v <= 1'b0;
               o_burst_done  <= 1'b0;
               if(i_cs)
               begin
                  block_addr <= i_addr[($clog2(BLOCKSIZE)+OFFSETSIZE-1):OFFSETSIZE];
                  word_cnt   <= '0;
                  if(i_write_read)
                  begin
                                         state <= BURST_WRITE;
                  o_write_ready <= 1'b1;
                                         
                  end else begin
                                         state <= BURST_READ;   
                  
                  end 
               end 
            end 
         BURST_READ: 
            begin  
            //
               o_read_data   <= MEM[block_addr][word_cnt];
               o_read_data_v <= 1'b1;
               if(word_cnt == (BLOCKWORDSIZE-1))
               begin
                  word_cnt <= '0;
                  o_burst_done  <= 1'b1;
                                         state <= IDLE;
               end else begin
                  word_cnt <= word_cnt + 1;
               end 
            //   
            end 
         BURST_WRITE:
            begin  
               //
               if(i_write_data_v)
               begin
                  MEM[block_addr][word_cnt] <= i_write_data;
                  if(word_cnt == (BLOCKWORDSIZE-1))
                  begin
                     word_cnt <= '0;
                     o_burst_done  <= 1'b1;
                     o_write_ready <= 1'b0;
                                         state <= IDLE;
                  end else begin
                     word_cnt <= word_cnt + 1;
                  end 
               end 
               //
            end 
      endcase 
   end 
   




end 


endmodule :EXTERNAL_MEMORY



//===============================
// EXTERNAL MEMORY TEST BENCH 
//===============================
module EXTERNAL_MEMORY_tb();



parameter BLOCKSIZE     = 64;    
parameter BLOCKWORDSIZE = 4;
parameter OFFSETSIZE    = $clog2(BLOCKWORDSIZE*4); 

// LOCAL SIGNALS 
logic        clk; 
logic        rstn; 
logic        i_cs;           // CHIP SELECT  
logic        i_write_read;   // 1:Write 0:Read 
logic [31:0] i_addr;         // READ/WRITE ADDR 
logic [31:0] i_write_data;   // WRITE DATA WORD
logic        i_write_data_v; // WRITE DATA VALID
logic        o_write_ready;  // Ready to Recive Write Signals 
logic [31:0] o_read_data;    // READ DATA WORD 
logic        o_read_data_v;  // READ DATA VALID 
logic        o_burst_done;

//====================
// CLOCK GENERATION 
//====================
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 

//====================
// RESET TASK 
//====================
task RESET();
begin
      i_cs           <= '0 ; // CHIP SELECT  
      i_write_read   <= '0 ; // 1:Write 0:Read 
      i_addr         <= '0 ; // READ/WRITE ADDR 
      i_write_data   <= '0 ; // WRITE DATA WORD
      i_write_data_v <= '0 ; // WRITE DATA VALID
      rstn = 1'b1;
   @(posedge clk);
      rstn = 1'b0;
   repeat(2)@(posedge clk);
      rstn = 1'b1;
end 
endtask 



//============================
// READ BLOCK TASK
//============================
task READ(input logic [((32-OFFSETSIZE)-1):0] block_addr,
          input logic [(OFFSETSIZE-1):0] offset = '0);
begin
   @(posedge clk);
      i_cs           <= 1'b1; // CHIP SELECT  
      i_write_read   <= 1'b0; // 1:Write 0:Read 
      i_addr         <= {block_addr,offset}; // READ/WRITE ADDR 
      i_write_data   <= '0;   // WRITE DATA WORD
      i_write_data_v <= '0;   // WRITE DATA VALID
   @(posedge clk);
      i_cs           <= '0; // CHIP SELECT  
      i_write_read   <= '0; // 1:Write 0:Read 
      i_addr         <= '0; // READ/WRITE ADDR 
   repeat(5) @(posedge clk);
end 
endtask


//============================
// WRITE BLOCK TASK
//============================
task WRITE(input logic [((32-OFFSETSIZE)-1):0] block_addr,
           input logic [(OFFSETSIZE-1):0] offset = '0,
           input int start_data = '0);
begin
   @(posedge clk);
      i_cs           <= 1'b1; // CHIP SELECT  
      i_write_read   <= 1'b1; // 1:Write 0:Read 
      i_addr         <= {block_addr,offset}; // READ/WRITE ADDR 
      i_write_data   <= '0;   // WRITE DATA WORD
      i_write_data_v <= '0;   // WRITE DATA VALID
   @(posedge clk);
      i_cs           <= '0; // CHIP SELECT  
      i_write_read   <= '0; // 1:Write 0:Read 
      i_addr         <= '0; // READ/WRITE ADDR 
      for(int i0=0;i0<BLOCKWORDSIZE;i0++)
      begin
         @(posedge clk);
         i_write_data   <= start_data+i0;   // WRITE DATA WORD
         i_write_data_v <= 1'b1;            // WRITE DATA VALID
      end 
      @(posedge clk);
      i_write_data   <= '0;   // WRITE DATA WORD
      i_write_data_v <= '0;   // WRITE DATA VALID    
      repeat(BLOCKWORDSIZE) @(posedge clk);  
end 
endtask


//===============================
// TEST 1 
//===============================
// READ MEMORY TEST 
// 1- Initialize the MEmory 
// 2- Read the MEmory 
task test1();
begin
   RESET();
   // Initialize the first Block 
   uEXTERNAL_MEMORY.MEM[0][0] <= 32'h11112222; 
   uEXTERNAL_MEMORY.MEM[0][1] <= 32'h33334444; 
   uEXTERNAL_MEMORY.MEM[0][2] <= 32'h55556666; 
   uEXTERNAL_MEMORY.MEM[0][3] <= 32'h77778888; 
   // Initialize the Second Block 
   uEXTERNAL_MEMORY.MEM[12][0] <= 32'h9999AAAA; 
   uEXTERNAL_MEMORY.MEM[12][1] <= 32'hBBBBCCCC; 
   uEXTERNAL_MEMORY.MEM[12][2] <= 32'hDDDDEEEE; 
   uEXTERNAL_MEMORY.MEM[12][3] <= 32'hFFFF1111; 
   // READ THE BLOCK 0. 
   READ(.block_addr ('d0));
   // READ THE BLOCK 12. 
   READ(.block_addr ('d12));
end 
endtask 



//===============================
// TEST 2 
//===============================
// READ MEMORY TEST 
// 1- Write to the Memory 
// 2- Read the MEmory 
task test2();
begin
   RESET();
   WRITE(.block_addr ('d0),.start_data (5));
   WRITE(.block_addr ('d1),.start_data (10));
   WRITE(.block_addr ('d2),.start_data (15));
   WRITE(.block_addr ('d3),.start_data (20));
   WRITE(.block_addr ('d4),.start_data (25));
   WRITE(.block_addr ('d5),.start_data (30));
   WRITE(.block_addr ('d6),.start_data (35));
   WRITE(.block_addr ('d7),.start_data (40));
   
   // READ THE BLOCKS
   READ(.block_addr ('d0));
   READ(.block_addr ('d1));
   READ(.block_addr ('d2));
   READ(.block_addr ('d3));
   READ(.block_addr ('d4));
   READ(.block_addr ('d5));
   READ(.block_addr ('d6));
   READ(.block_addr ('d7));
end 
endtask 

//====================
// MAIN STIMULUS 
//====================
initial begin
   //test1();
   test2();


   $finish;
end 





//====================
// DUT INSTANTIATION 
//====================
EXTERNAL_MEMORY uEXTERNAL_MEMORY(.*);




endmodule :EXTERNAL_MEMORY_tb  

