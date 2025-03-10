`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Cache Based 2-bit Branch Predictor  
//////////////////////////////////////////////////////////////////////////////////


module BHT__2bit_predictor
#(parameter WAYSIZE    = 4,             
  parameter OFFSETBIT  = 10,
  parameter INDEXBIT   = 8, 
  parameter TAGBIT     = 20,
  parameter PASIZE     = (TAGBIT + INDEXBIT + OFFSETBIT) )
   (
   input logic          clk, 
   input logic          rstn,
   //======================= 
   // READ SIGNALS 
   //=======================
   input  logic         i_bht_read_en,
   input  logic [31:0]  i_bht_read_pc, 
   output logic         o_bht_taken_pre,
   output logic         o_bht_hit,
   //======================= 
   // WRITE SIGNALS 
   //=======================
   input logic          i_bht_write_en, 
   input logic [31:0]   i_bht_write_pc, 
   input logic          i_bht_write_taken 
    );
    
    

    
// CACHE MEMORY
// cache_mem[set][line][line_row][line_column]
//            SET            LINE            LINE_ROW       LINE_COLMN    
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(2**OFFSETBIT-1):0][1:0]    T_CNT; 
 //            SET            LINE
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                             T_VALID; 
//            SET            LINE           TAG                                                       
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(TAGBIT-1):0]               T_TAG;  
//  
logic  [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                            T_PLRU_bits; 
//
logic  [(OFFSETBIT-1):0]                read_offset;  
logic  [(INDEXBIT-1):0]                 read_index;    
logic  [(TAGBIT-1):0]                   read_tag;   
//
logic  [(OFFSETBIT-1):0]                write_offset;  
logic  [(INDEXBIT-1):0]                 write_index;    
logic  [(TAGBIT-1):0]                   write_tag;   
//
logic write_hit,plru_found, read_hit;
logic [(2**WAYSIZE-1):0] write_line, read_line, plru_line;


//==============================================
// ---------------DECODING---------------------- 
//==============================================
assign read_offset  = i_bht_read_pc[(OFFSETBIT-1):0]; 
assign read_index   = i_bht_read_pc[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];;    
assign read_tag     = i_bht_read_pc[(PASIZE-1):(OFFSETBIT+INDEXBIT)];   
//
assign write_offset = i_bht_write_pc[(OFFSETBIT-1):0];   
assign write_index  = i_bht_write_pc[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];    
assign write_tag    = i_bht_write_pc[(PASIZE-1):(OFFSETBIT+INDEXBIT)]; 
//



always_comb 
begin
   read_hit   = 1'b0;
   read_line  = '0;
   //
   write_hit  = 1'b0;
   write_line = 1'b0;
   //
   plru_found = 1'b0;
   plru_line  = '0;
   //===============================        
   //---------CHECK READ HIT-------- 
   //===============================
   // IN i_bht_read_en --> Module input 
   // IN WAYSIZE       --> parameter
   // IN read_tag      --> Decode CL 
   // IN read_index    --> Decode CL 
   // IN T_VALID       --> SL  
   // IN T_TAG         --> SL 
   // 
   // OUT read_hit
   // OUT read_line
   
   if(i_bht_read_en)
   begin
      for(int line=0;line<(2**WAYSIZE);line++)
      begin
          if((T_TAG[read_index ][line] == read_tag) && T_VALID[read_index][line]) 
          begin
             read_hit  = 1'b1;
             read_line = line;
          end   
      end 
   end 
   
   //===============================        
   //---------CHECK WRITE HIT-------- 
   //===============================
   // WAYSIZE       --> parameter 
   // write_index   --> Decode CL 
   // write_tag     --> Decode CL 
   // 
   // OUT write_hit 
   // OUT write_line 
   for(int l=0;l<(2**WAYSIZE);l++)
   begin
       if((T_TAG[write_index][l] == write_tag) && T_VALID[write_index][l] && ~write_hit) 
       begin
          write_hit  = 1'b1; 
          write_line = l;    // HIT LINE 
       end
   end 
   
   // If there is a WRITE MISS, choose the PLRU line as the write_line. 			  
  
   if(~write_hit)
   begin 
      // Check for an invalid line. 
      for(int i=0;i<(2**WAYSIZE);i++)
      begin
         if((T_VALID[write_index ][i] == 1'b0) && ~plru_found)
         begin
            plru_found = 1'b1;
            plru_line  = i;
         end   
      end 
      // if all the lines are valid, choose the PLRU line based on the PLRU bits 
      if(~plru_found)
      begin 
         // Choose the first line, which has the PLRU bit set. 
         for(int i=0;i<(2**WAYSIZE);i++)
         begin
            if((T_VALID[write_index ][i] == 1'b1) && ~plru_found)
            begin
               plru_found = 1'b1;
               plru_line  = i;
            end   
         end 
         // 
      end 
      
      write_line = plru_line;
      
   end // if(~write_hit)  
   
   
   
   
end 



//==============================================
// ---------------READ-WRITE---------------------- 
//==============================================
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      for(int set_cnt =0;set_cnt<(2**INDEXBIT);set_cnt++)begin
         for(int way_cnt =0;way_cnt<(2**WAYSIZE);way_cnt++)begin
            for(int offset_cnt =0;offset_cnt<(2**OFFSETBIT);offset_cnt++)begin
               T_CNT[set_cnt][way_cnt][offset_cnt] <= 'd1;
            end     
         end     
      end 
      // READ SIGNALS 
      o_bht_taken_pre <= '0;
      o_bht_hit       <= '0;
      //
      T_PLRU_bits     <= '0;
      T_VALID         <= '0;
   end else begin
      //=========================================
      //------------- WRITE OPERATION -----------
      //========================================= 
      // IN i_bht_write_en    --> Module Input 
      // IN i_bht_write_taken --> Module Input 
      // IN write_tag         --> Decode CL 
      // IN write_index       --> Decode CL
      // IN write_offset      --> Decode CL 
      // IN write_line        --> CL 
      // IN write_hit         --> CL 
      //
      // OUT T_TAG
      // OUT T_VALID
      // OUT T_CNT
      if(i_bht_write_en)
      begin
         // Update TAG and VALID bit if this is a WRITE MISS.  
         if(~write_hit)
         begin
            T_TAG[write_index][write_line]   <= write_tag;
            T_VALID[write_index][write_line] <= 1'b1;
         end 
         // Write 
         if(i_bht_write_taken)
         begin
            $display("--WRITE_TAKEN  write_index:%d write_line:%d write_index:%d ",write_index,write_line,write_offset);
            T_CNT[write_index][write_line][write_offset] <= &T_CNT[write_index][write_line][write_offset]  ? T_CNT[write_index][write_line][write_offset] : (T_CNT[write_index][write_line][write_offset] + 1); // MSB of the CNT is the prediction. 
         end else begin
            $display("--WRITE NOT TAKEN write_index:%d write_line:%d write_index:%d ",write_index,write_line,write_offset);
            T_CNT[write_index][write_line][write_offset] <= ~|T_CNT[write_index][write_line][write_offset] ? T_CNT[write_index][write_line][write_offset] : (T_CNT[write_index][write_line][write_offset] - 1); // MSB of the CNT is the prediction. 
         end 
         // 
         
         
      end 
      
      //=========================================
      //------------- READ OPERATION ------------
      //========================================= 
      // IN i_bht_read_en  --> Module Input 
      // IN read_index     --> Decode CL 
      // IN read_line      --> CL 
      // IN read_offset    --> Decode CL 
      // IN read_hit       --> CL 
      //
      // OUT o_bht_taken_pre
      // OUT o_bht_hit
      if(i_bht_read_en)
      begin
         if(read_hit)
         begin
            o_bht_taken_pre <= T_CNT[read_index][read_line][read_offset][1]; // MSB of the CNT is the prediction. 
            o_bht_hit       <= 1'b1;
         end else begin
            o_bht_taken_pre <= '0;
            o_bht_hit       <= '0;
         end 
      end 
      
      
      //=========================================
      //------------- UPDATE PLRU ------------
      //=========================================
      if(i_bht_read_en)
      begin
         T_PLRU_bits[read_index][read_line] = 1'b1;
         if(&T_PLRU_bits) 
         begin
           T_PLRU_bits = '0;
           T_PLRU_bits[read_index][read_line] = 1'b1;
         end 
      end    
      
      
      if(i_bht_write_en)
      begin
         T_PLRU_bits[write_index][write_line] = 1'b1;
         if(&T_PLRU_bits) 
         begin
           T_PLRU_bits = '0;
           T_PLRU_bits[write_index][write_line] = 1'b1;
         end 
      end 
      
      

      
      
      
            
   end 
end 













    
endmodule :BHT__2bit_predictor


//=====================================
//------------TEST BENCH---------------
//=====================================
module BHT__2bit_predictor_tb();


parameter WAYSIZE    = 4;             
parameter OFFSETBIT  = 4;
parameter INDEXBIT   = 5; 
parameter TAGBIT     = 23;
parameter PASIZE     = (TAGBIT + INDEXBIT + OFFSETBIT) ;
//
logic          clk; 
logic          rstn;
//======================= 
// READ SIGNALS 
//=======================
logic         i_bht_read_en;
logic [31:0]  i_bht_read_pc; 
logic         o_bht_taken_pre;
logic         o_bht_hit;
//======================= 
// WRITE SIGNALS 
//=======================
logic          i_bht_write_en; 
logic [31:0]   i_bht_write_pc; 
logic          i_bht_write_taken; 

//======================= 
// CLOCK GENERATION  
//=======================
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 



//======================= 
// RESET Task 
//=======================
task RESET();
begin
   i_bht_read_en     = 1'b0;
   i_bht_read_pc     = 1'b0;
   i_bht_write_en    = 1'b0;
   i_bht_write_pc    = 1'b0;
   i_bht_write_taken = 1'b0;
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0; 
   repeat(2) @(posedge clk);
   rstn = 1'b1;
end
endtask 

//======================= 
//  Functionality 1 
//=======================
//input logic          i_bht_write_en, 
//input logic [31:0]   i_bht_write_pc, 
//input logic          i_bht_write_taken 
task WRITE(input logic [31:0] bht_write_pc,
           input logic        bht_write_taken);
begin
   @(posedge clk);
      i_bht_write_en    <= 1'b1;
      i_bht_write_pc    <= bht_write_pc; 
      i_bht_write_taken <= bht_write_taken;  
   @(posedge clk);
      i_bht_write_en <= 1'b0;
      i_bht_write_pc <= '0;
end 
endtask 


//======================= 
//  Functionality 2
//=======================
//input  logic         i_bht_read_en,
//input  logic [31:0]  i_bht_read_pc, 
//output logic         o_bht_taken_pre,
//output logic         o_bht_hit,

task READ(input logic [31:0] bht_read_pc);
begin
   @(posedge clk);
      i_bht_read_en <= 1'b1;
      i_bht_read_pc <= bht_read_pc; 
   @(posedge clk);
      i_bht_read_en <= '0;
      i_bht_read_pc <= '0;
end 
endtask 



task test1();
begin
   RESET();
   WRITE(.bht_write_pc ('h1234), .bht_write_taken(1'b1) );
   WRITE(.bht_write_pc ('h1234), .bht_write_taken(1'b1) );
   WRITE(.bht_write_pc ('h1234), .bht_write_taken(1'b1) );   
   WRITE(.bht_write_pc ('h1234), .bht_write_taken(1'b1) ); 
   WRITE(.bht_write_pc ('h1234), .bht_write_taken(1'b1) );
   WRITE(.bht_write_pc ('h1234), .bht_write_taken(1'b1) );

end 
endtask :test1 




//======================= 
//  TEST STIMULUS 
//=======================
initial begin
   test1();

   $finish; 
end 




//======================= 
// DUT INSTANTIATION 
//=======================
BHT__2bit_predictor
#(.WAYSIZE    (WAYSIZE),             
  .OFFSETBIT  (OFFSETBIT),   
  .INDEXBIT   (INDEXBIT),   
  .TAGBIT     (TAGBIT),   
  .PASIZE     (PASIZE))
uBHT__2bit_predictor
   (.*);
    



endmodule :BHT__2bit_predictor_tb









