`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// BRANCH TARGET BUFFER 
//////////////////////////////////////////////////////////////////////////////////


module BTB
#(parameter PCSIZE     = 32,
  parameter WAYSIZE    = 4,             
  parameter OFFSETBIT  = 10,
  parameter INDEXBIT   = 8, 
  parameter TAGBIT     = 20,
  parameter PASIZE     = (TAGBIT + INDEXBIT + OFFSETBIT))
(
  input  logic                clk, 
  input  logic                rstn,
  // FLUSH FLAG
  input logic                 i_flush,
  //======================================= 
  //------------- READ SIGNALS ------------  
  //=======================================  
  input  logic                i_btb_read_en,
  input  logic [(PCSIZE-1):0] i_btb_read_pc, 
  output logic                o_btb_read_hit,
  output logic [(PCSIZE-1):0] o_btb_target_pre, 
  output logic                o_btb_is_ret,        
  //======================================= 
  //------------- WRITE SIGNALS ------------  
  //=======================================
  input  logic                i_btb_write_en,
  input  logic [(PCSIZE-1):0] i_btb_write_pc, 
  input  logic [(PCSIZE-1):0] i_btb_write_target,
  input  logic                i_btb_write_is_ret    // It is RETURN instruction so RAS table will predict the target. 
    );
    
    
    
// CACHE MEMORY
// cache_mem[set][line][line_row][line_column]
//            SET            LINE            LINE_ROW       LINE_COLMN    
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(2**(OFFSETBIT)-1):0][(PCSIZE-1):0]    T_TARGET; 
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(2**(OFFSETBIT)-1):0][(PCSIZE-1):0]    T_IS_RET; 
 //            SET            LINE
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                                        T_VALID; 
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                                        T_DIRTY; 
//            SET            LINE           TAG                                                       
logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(TAGBIT-1):0]                          T_TAG;  
//  
logic  [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                                       T_PLRU_bits; 
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
assign read_offset  = i_btb_read_pc[(OFFSETBIT-1):0]; 
assign read_index   = i_btb_read_pc[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];;    
assign read_tag     = i_btb_read_pc[(PCSIZE-1):(OFFSETBIT+INDEXBIT)];   
//
assign write_offset = i_btb_write_pc[(OFFSETBIT-1):0];   
assign write_index  = i_btb_write_pc[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];    
assign write_tag    = i_btb_write_pc[(PCSIZE-1):(OFFSETBIT+INDEXBIT)]; 
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
   if(i_btb_read_en)
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
      //
      T_TARGET            <= '0;
      T_IS_RET            <= '0; 
      T_PLRU_bits         <= '0;
      T_VALID             <= '0;
      T_DIRTY             <= '0;
      T_TAG               <= '0;
      // READ SIGNALS 
      o_btb_read_hit      <= '0;
      o_btb_target_pre    <= '0;
      o_btb_is_ret        <= '0;      
      //
   end else begin
      //=========================================
      //------------- WRITE OPERATION -----------
      //========================================= 
      // IN i_btb_write_en     --> Module Input 
      // IN i_btb_write_target --> Module Input 
      // IN i_btb_write_is_ret --> Module Input 
      // IN write_tag          --> Decode CL 
      // IN write_index        --> Decode CL
      // IN write_offset       --> Decode CL 
      // IN write_line         --> CL 
      // IN write_hit          --> CL 
      //
      // OUT T_TAG
      // OUT T_VALID
      // OUT T_TARGET
      // OUT T_IS_RET 
      if(i_btb_write_en)
      begin
         // Update TAG and VALID bit if this is a WRITE MISS.  
         if(~write_hit)
         begin
            T_TAG[write_index][write_line]   <= write_tag;
            T_VALID[write_index][write_line] <= 1'b1;
         end 
         // Write
         $display("WRITE_TAKEN");
         T_TARGET[write_index][write_line][write_offset] <= i_btb_write_target;
         T_IS_RET[write_index][write_line][write_offset] <= i_btb_write_is_ret;
      end 
      
      //=========================================
      //------------- READ OPERATION ------------
      //========================================= 
      // IN i_btb_read_en  --> Module Input 
      // IN read_index     --> Decode CL 
      // IN read_line      --> CL 
      // IN read_offset    --> Decode CL 
      // IN read_hit       --> CL 
      //
      // OUT o_bht_taken_pre
      // OUT o_bht_hit
      if(i_btb_read_en)
      begin
         if(read_hit)
         begin
            o_btb_read_hit    <= read_hit;
            o_btb_target_pre  <= T_TARGET[read_index][read_line][read_offset];
            o_btb_is_ret      <= T_IS_RET[read_index][read_line][read_offset]; 
         end else begin
            o_btb_read_hit    <= '0;
            o_btb_target_pre  <= '0;
            o_btb_is_ret      <= '0; 
         end 
      end 
      
      
      //=========================================
      //------------- UPDATE PLRU ------------
      //=========================================
      if(i_btb_read_en)
      begin
         T_PLRU_bits[read_index][read_line] = 1'b1;
         if(&T_PLRU_bits) 
         begin
           T_PLRU_bits = '0;
           T_PLRU_bits[read_index][read_line] = 1'b1;
         end 
      end    
      
      
      if(i_btb_write_en)
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

   
endmodule :BTB





//==================================
//---------TEST BENCH--------------- 
//==================================
module BTB_tb();

parameter PCSIZE     = 32;
parameter WAYSIZE    = 4;             
parameter OFFSETBIT  = 4;
parameter INDEXBIT   = 3; 
parameter TAGBIT     = 32 - (OFFSETBIT + INDEXBIT + WAYSIZE);
parameter PASIZE     = (TAGBIT + INDEXBIT + OFFSETBIT);


logic                clk; 
logic                rstn;
// FLUSH FLAG
logic                i_flush;
//======================================= 
//------------- READ SIGNALS ------------  
//=======================================  
logic                i_btb_read_en;
logic [(PCSIZE-1):0] i_btb_read_pc; 
logic                o_btb_read_hit;
logic [(PCSIZE-1):0] o_btb_target_pre;    
logic                o_btb_is_ret;    
//======================================= 
//------------- WRITE SIGNALS ------------  
//=======================================
logic                i_btb_write_en;
logic [(PCSIZE-1):0] i_btb_write_pc; 
logic [(PCSIZE-1):0] i_btb_write_target;
logic                i_btb_write_is_ret;    // It is RETURN instruction so RAS table will predict the target. 
//
int error;

//======================================= 
//------------- Clock Generation --------  
//=======================================
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 

//======================================= 
//----------- RESET TASK ----------------  
//=======================================
task RESET();
begin
   error = '0;
   //
   i_flush            = '0;
   i_btb_read_en      = '0;
   i_btb_read_pc      = '0;
   i_btb_write_en     = '0;
   i_btb_write_pc     = '0; 
   i_btb_write_target = '0;
   i_btb_write_is_ret = '0;
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2)@(posedge clk);
   rstn = 1'b1;
end 
endtask 

//============================
// READ TARGET 
//============================
task WRITE( input logic [(PCSIZE-1):0] btb_write_pc, 
            input logic [(PCSIZE-1):0] btb_write_target,
            input logic                btb_write_is_ret    );
begin
   
   @(posedge clk); 
      i_btb_write_en     <= 1'b1;
      i_btb_write_pc     <= btb_write_pc; 
      i_btb_write_target <= btb_write_target;
      i_btb_write_is_ret <= btb_write_is_ret;
   @(posedge clk);
      i_btb_write_en     <= '0;
      i_btb_write_pc     <= '0; 
      i_btb_write_target <= '0;
      i_btb_write_is_ret <= '0;
end 
endtask 






//============================
// WRITE NEW PC TARGET TASK 
//============================
// READ INTERFACE 
//logic                i_btb_read_en;
//logic [(PCSIZE-1):0] i_btb_read_pc; 
//logic                o_btb_read_hit;
//logic [(PCSIZE-1):0] o_btb_target_pre;    
//logic                o_btb_is_ret;  
task READ(input logic [(PCSIZE-1):0] btb_read_pc,
          input logic                ex_btb_read_hit,
          input logic [(PCSIZE-1):0] ex_btb_target_pre,
          input logic                ex_btb_is_ret);
begin

   @(posedge clk);
      i_btb_read_en <= 1'b1;
      i_btb_read_pc <= btb_read_pc;   
   @(posedge clk);   
      i_btb_read_en <= '0;
      i_btb_read_pc <= '0;   
      #2 
   if(o_btb_read_hit   != ex_btb_read_hit)   begin $display("ERROR: o_btb_read_hit   != ex_btb_read_hit");   error++; end    
   if(o_btb_target_pre != ex_btb_target_pre) begin $display("ERROR: o_btb_target_pre != ex_btb_target_pre"); error++; end    
   if(o_btb_is_ret     != ex_btb_is_ret)     begin $display("ERROR: o_btb_is_ret     != ex_btb_is_ret");     error++; end    
      
end 
endtask 

//========================================
//------------ TEST1: READ/WRITE TEST -----------
// Write to BTB and then Read the same addresses and check if written data
// and read data match.   
//========================================
task test1();
begin
   RESET();
   //
   WRITE( .btb_write_pc('d0 ),  .btb_write_target('d100), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d4 ),  .btb_write_target('d200), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d8 ),  .btb_write_target('d300), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d12),  .btb_write_target('d400), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d16),  .btb_write_target('d104), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d20),  .btb_write_target('d108), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d24),  .btb_write_target('d112), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d28),  .btb_write_target('d204), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d32),  .btb_write_target('d304), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d36),  .btb_write_target('d408), .btb_write_is_ret(1'b1) );
   //
   READ(.btb_read_pc ('d0 ), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d100), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d4 ), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d200), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d8 ), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d300), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d12), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d400), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d16), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d104), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d20), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d108), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d24), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d112), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d28), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d204), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d32), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d304), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d36), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d408), .ex_btb_is_ret (1'b1) );
   //
   if(error==0) begin $display(" TEST-1 PASS");  end
   else         begin $display(" TEST-1 FAIL");  end  
end 
endtask 


//========================================
//------------ TEST2: READ/WRITE TEST -----------
// 1- Write to BTB
// 2- Overwrite the same address fields 
// 3- Read the same address 
// 4- Compare the written values and read values. If they match, test PASSes. 
//========================================
task test2();
begin
   RESET();
   // WRITE TO BTB 
   WRITE( .btb_write_pc('d0 ),  .btb_write_target('d100), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d4 ),  .btb_write_target('d200), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d8 ),  .btb_write_target('d300), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d12),  .btb_write_target('d400), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d16),  .btb_write_target('d104), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d20),  .btb_write_target('d108), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d24),  .btb_write_target('d112), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d28),  .btb_write_target('d204), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d32),  .btb_write_target('d304), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d36),  .btb_write_target('d408), .btb_write_is_ret(1'b1) );
   // 2- Overwrite the same values with different values 
   WRITE( .btb_write_pc('d0 ),  .btb_write_target('d500), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d4 ),  .btb_write_target('d600), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d8 ),  .btb_write_target('d700), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d12),  .btb_write_target('d800), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d16),  .btb_write_target('d904), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d20),  .btb_write_target('d114), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d24),  .btb_write_target('d118), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d28),  .btb_write_target('d122), .btb_write_is_ret(1'b0) );
   WRITE( .btb_write_pc('d32),  .btb_write_target('d126), .btb_write_is_ret(1'b1) );
   WRITE( .btb_write_pc('d36),  .btb_write_target('d130), .btb_write_is_ret(1'b0) );
   //
   READ(.btb_read_pc ('d0 ), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d500), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d4 ), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d600), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d8 ), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d700), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d12), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d800), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d16), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d904), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d20), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d114), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d24), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d118), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d28), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d122), .ex_btb_is_ret (1'b0) );
   READ(.btb_read_pc ('d32), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d126), .ex_btb_is_ret (1'b1) );
   READ(.btb_read_pc ('d36), .ex_btb_read_hit (1'b1), .ex_btb_target_pre ('d130), .ex_btb_is_ret (1'b0) );
   //
   if(error==0) begin $display(" TEST-2 PASS");  end
   else         begin $display(" TEST-2 FAIL");  end  
end 
endtask 



initial begin
   test2();
   repeat(100) @(posedge clk);
   $finish;
end 

//================================
//------------BTB-----------------
//================================
BTB
#(.PCSIZE     (PCSIZE),
  .WAYSIZE    (WAYSIZE),
  .OFFSETBIT  (OFFSETBIT),
  .INDEXBIT   (INDEXBIT),
  .TAGBIT     (TAGBIT),
  .PASIZE     (PASIZE))
uBTB
(.*);

endmodule :BTB_tb  
