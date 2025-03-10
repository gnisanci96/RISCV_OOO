`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// VirtuallyIndexed-Physically-Tagged L1 Cache. 
// imm[11:0] rs1     000 rd       0000011 LB
// imm[11:0] rs1     001 rd       0000011 LH
// imm[11:0] rs1     010 rd       0000011 LW
// imm[11:0] rs1     100 rd       0000011 LBU
// imm[11:0] rs1     101 rd       0000011 LHU
// imm[11:5] rs2 rs1 000 imm[4:0] 0100011 SB
// imm[11:5] rs2 rs1 001 imm[4:0] 0100011 SH
// imm[11:5] rs2 rs1 010 imm[4:0] 0100011 SW

//================================
// L2 COMMAND DEFINITIONS 
//================================
`define  CACHE_HIT_CHECK_CMD  3'b000
`define  LOAD_CMD             3'b001
`define  STORE_CMD            3'b010
`define  BURST_READ_CMD       3'b011
`define  BURST_WRITE_CMD      3'b100
`define  UPDATE_INCLUSION_CMD 3'b101

module L2_CACHE
#(parameter ADDRSIZE    = 32,
  parameter WAYSIZE     = 4,             
  parameter OFFSETSIZE  = 10,
  parameter INDEXSIZE   = 8, 
  parameter TAGSIZE     = 20)
(
   input logic clk,
   input logic rstn,
   input logic i_flush,
   //  
   input  logic                                   i_l2_cs,
   input  logic [2:0]                             i_l2_cmd, 
   output logic                                   o_l2_cmd_done,                // Outputs are ready for the command 
   //
   input  logic  [(ADDRSIZE-1):0]                 i_l2_addr,                // set num   
   input  logic  [(WAYSIZE-1):0]                  i_l2_line_num,            // line num
   input  logic  [2:0]                            i_l2_funct3,              // Size (byte,half-byte,word) 
   // CMD0: CACHE HIT CHECK. 
   output logic                                   o_l2_hit,                 // 0:Cache Miss 1: Cache Hit  
   output logic [(WAYSIZE-1):0]                   o_l2_hit_or_victim_line,     // o_l2_hit == 1 ? Hit Line # : LRU_line 
   output logic                                   o_l2_hit_or_victim_line_v,       // Victim Line Valid Bit 
   output logic                                   o_l2_hit_or_victim_line_d,       // Victim Line Dirty Bit
   output logic  [(ADDRSIZE-1):0]                 o_l2_victim_line_addr,
   // CMD1: Line SIGLE-DATA Read 
   output logic  [31:0]                           o_l2_rd_line_data,          // Data  
   output logic                                   o_l2_rd_valid,              // Read Data,tags,valid_bits VALID.        
   input  logic                                   i_l2_wrt_valid,             // Burst Read Value Valid.
   input  logic                                   i_l2_burst_done,            //   
   // CMD2: Line SINGLE-DATA Write
   input  logic  [31:0]                           i_l2_wrt_line_data,          // Data  
   // CMD3: Line Burst Read 
   input  logic                                   i_l2_wrt_line_dirty, 
   // CMD4: Line Burst Write   
   output logic                                   o_l2_burst_write_ready,
   output logic                                   o_l2_busy,
   //CMD6 : UPDATE INCLUSION BIT 
   input logic                                    i_l2_inclusion_bit       
    );
    
    
 // CACHE MEMORY
 // cache_mem[set][line][line_row][line_column]
 //            SET            LINE            LINE_ROW       LINE_COLMN    
 logic [(2**INDEXSIZE-1):0][(2**WAYSIZE-1):0][(2**(OFFSETSIZE-2)-1):0][31:0]  T_CACHE_DATA; 
 //            SET            LINE
 logic [(2**INDEXSIZE-1):0][(2**WAYSIZE-1):0]                                 T_LINE_VALID; 
 logic [(2**INDEXSIZE-1):0][(2**WAYSIZE-1):0]                                 T_LINE_DIRTY; 
 //            SET            LINE           TAG                                                       
 logic [(2**INDEXSIZE-1):0][(2**WAYSIZE-1):0][(TAGSIZE-1):0]                  T_LINE_TAG;  
//  
logic  [(2**INDEXSIZE-1):0][(2**WAYSIZE-1):0][(WAYSIZE-1):0]                  T_LRU_CNTS; 
logic  [(2**INDEXSIZE-1):0][(2**WAYSIZE-1):0]                                 T_INCLUSION; 

//
logic [(OFFSETSIZE-1):0]    burst_cnt;
logic [(INDEXSIZE-1):0]     l2_set_num_reg;  // set num   
logic [(WAYSIZE-1):0]       l2_line_num_reg; // line num
logic                       loop_break; 
// CMD3: Burst-Line Read 
// CMD4: Burst-Data Write
  
  
//==================================================
// Local Signals   
//==================================================  
logic [(TAGSIZE-1):0]    l2_tag, l2_tag_reg;  
logic [(INDEXSIZE-1):0]  l2_index, l2_index_reg;   
logic [(OFFSETSIZE-1):0] l2_offset;   
logic                    l2_wrt_line_dirty_reg;  
logic                    first_non_inclusion; 
  
  
typedef enum{
   IDLE,
   BURST_READ,
   BURST_WRITE
} state_type;
 
state_type cstate,nstate;


assign l2_index  = i_l2_addr[(OFFSETSIZE+INDEXSIZE-1):OFFSETSIZE];
assign l2_offset = i_l2_addr[(OFFSETSIZE-1):0];
assign l2_tag    = i_l2_addr[(ADDRSIZE-1):(OFFSETSIZE+INDEXSIZE)];



always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn || i_flush)
   begin
       cstate                     <= IDLE;
       //  
 	   o_l2_hit                   <= '0;
       o_l2_hit_or_victim_line    <= '0;
       o_l2_cmd_done              <= '0;
       //
       o_l2_rd_line_data          <= '0;
       //
       l2_set_num_reg             <= '0;   
       l2_line_num_reg            <= '0;  
       //
       o_l2_rd_valid              <= '0;
       //
       T_LINE_VALID               <= '0;
       T_LINE_DIRTY               <= '0;
       T_INCLUSION                <= '0;
	   o_l2_hit_or_victim_line_v  <= '0;
	   o_l2_hit_or_victim_line_d  <= '0;
	   o_l2_burst_write_ready     <= '0;
	   o_l2_busy                  <= '0;
	   l2_tag_reg                 <= '0;
	   l2_index_reg               <= '0;
	   l2_wrt_line_dirty_reg      <= '0; 
	   first_non_inclusion        <= 1'b1; 
	   o_l2_victim_line_addr      <= '0;  
   end else begin
         // 
      o_l2_cmd_done = 1'b0;
      o_l2_hit      = 1'b0;
      case(cstate)
         IDLE:
         begin
            o_l2_burst_write_ready = 1'b0;
         //
         if(i_l2_cs)
         begin
            l2_tag_reg    <= l2_tag;
            l2_index_reg  <= l2_index;
            l2_wrt_line_dirty_reg      <= i_l2_wrt_line_dirty;
            case(i_l2_cmd)
                `CACHE_HIT_CHECK_CMD: // HIT CHECK -->  HIT OR VICTIM LINE RETURN
                begin
                   o_l2_cmd_done = 1'b1; 
				   loop_break = 1'b0;
				   //////////
                   for(int line=0;line<(2**WAYSIZE);line++)
                   begin
                      if( (T_LINE_TAG[l2_index][line] == l2_tag) && T_LINE_VALID[l2_index][line] && ~loop_break)
                      begin   
                         loop_break = 1'b1;
                         o_l2_hit                   = 1'b1;
                         o_l2_hit_or_victim_line    = line;                      
				      end
				   end 	  
                   /////////
				   if(~o_l2_hit)
				   begin
				      // VICTIM LINE PICK
				      /////////  IF THERE IS AN INVALID LINE, INVALID LINE IS THE VICTIM LINE 
                      for(int line=0;line<(2**WAYSIZE);line++)
                      begin
                         if((~T_LINE_VALID[l2_index][line]) && ~loop_break)
                         begin   
                            loop_break = 1'b1;
                            o_l2_hit_or_victim_line = line; 
                         end 
                      end     
                      // IF ALL THE LINES ARE VALID, CHECK FOR THE LRU Line Among the Lines that are not in L1.(Inclusion bit = 0)
                     first_non_inclusion = 1'b1;
                     for(int line=0;line<(2**WAYSIZE);line++)
                     begin
                        if(~T_INCLUSION[l2_index][line])
                        begin
                            //
                            if(first_non_inclusion)
                            begin
                               o_l2_hit_or_victim_line = line;
                               first_non_inclusion = 1'b0;
                            end 
                            //
                            if(T_LRU_CNTS[l2_index][o_l2_hit_or_victim_line]>T_LRU_CNTS[l2_index][line])
                            begin
                               o_l2_hit_or_victim_line = line;
                            end
                            // 
                        end 
                     end //   for(int line=0;line<(2**WAYSIZE);line++)  
                     o_l2_victim_line_addr <= {T_LINE_TAG[l2_index][o_l2_hit_or_victim_line],l2_index,{OFFSETSIZE{1'b0}} };
                                
                   end
				   //
				   o_l2_hit_or_victim_line_v  =  T_LINE_VALID[l2_index][o_l2_hit_or_victim_line];
				   o_l2_hit_or_victim_line_d  =  T_LINE_DIRTY[l2_index][o_l2_hit_or_victim_line];
				   //
                end 
                `LOAD_CMD : // LOAD 
                begin 
                     o_l2_cmd_done = 1'b1;
                     o_l2_rd_line_data = T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)];
                     case(i_l2_funct3)
                        3'b000: begin  o_l2_rd_line_data = { {24{o_l2_rd_line_data[7]}},o_l2_rd_line_data[7:0]};   end 
                        3'b001: begin  o_l2_rd_line_data = { {16{o_l2_rd_line_data[15]}},o_l2_rd_line_data[15:0]}; end 
                        3'b100: begin  o_l2_rd_line_data = { {24{1'b0}},o_l2_rd_line_data[7:0]};                   end 
                        3'b101: begin  o_l2_rd_line_data = { {16{1'b0}},o_l2_rd_line_data[15:0]};                  end 
                     endcase
                end 
                `STORE_CMD: // STORE 
                begin 
                    o_l2_cmd_done = 1'b1;
                    case(i_l2_funct3[1:0])
                    2'b00: // SB
                    begin 
                       case(l2_offset[1:0])
                       2'b00: begin T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)][7:0]   <= i_l2_wrt_line_data[7:0];  end 
                       2'b01: begin T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)][15:8]  <= i_l2_wrt_line_data[7:0];  end 
                       2'b10: begin T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)][23:16] <= i_l2_wrt_line_data[7:0];  end 
                       2'b11: begin T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)][31:24] <= i_l2_wrt_line_data[7:0];  end            
                       endcase
                    end 
                   2'b01: // SH
                   begin
                      case(l2_offset[1])
                         1'b0: begin T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)][15:0]   <= i_l2_wrt_line_data[15:0];  end 
                         1'b1: begin T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)][31:16]  <= i_l2_wrt_line_data[15:0];  end          
                      endcase
                  end 
                  2'b10: // SW
                  begin
                     T_CACHE_DATA[l2_index][i_l2_line_num][(l2_offset>>2)] <= i_l2_wrt_line_data;  
                  end 
                  endcase
                     T_LINE_DIRTY[l2_index][i_l2_line_num] = 1'b1;
                end 
                `BURST_READ_CMD: 
                begin 
                   l2_set_num_reg  <= l2_index;  // set num   
                   l2_line_num_reg <= i_l2_line_num; // line num
                   burst_cnt       <= '0; 
                   o_l2_busy       <= 1'b1; 
                                                                                       cstate <= BURST_READ;
                end 
                `BURST_WRITE_CMD: // BURST WRITE 
                begin 
                   l2_set_num_reg  <= l2_index;  // set num   
                   l2_line_num_reg <= i_l2_line_num; // line num
                   burst_cnt       <= '0;  
                   o_l2_burst_write_ready = 1'b1;
                   o_l2_busy       <= 1'b1;
                                                                                       cstate <= BURST_WRITE;
                end  
               `UPDATE_INCLUSION_CMD: // UPDATE INCLUSION BIT 
               begin
                  T_INCLUSION[l2_index][i_l2_line_num] <= i_l2_inclusion_bit;
               end 
            endcase 
            //
            end
         end
      BURST_READ:        
      begin
         if(~(burst_cnt==(2**(OFFSETSIZE-2))) && ~i_l2_burst_done)
         begin
               o_l2_rd_line_data <= T_CACHE_DATA[l2_set_num_reg][l2_line_num_reg][burst_cnt]; 
               o_l2_rd_valid     <= 1'b1;  
               burst_cnt         <= burst_cnt + 1;               
         end else begin
               o_l2_cmd_done         = 1'b1;        
               o_l2_rd_valid     <= 1'b0; 
               o_l2_busy         <= 1'b0;
                                                         cstate <= IDLE; 
         end 
      end
      BURST_WRITE:
      begin
         if(~(burst_cnt==(2**(OFFSETSIZE-2))) && ~i_l2_burst_done)
         begin
             if(i_l2_wrt_valid)
             begin
               T_CACHE_DATA[l2_set_num_reg][l2_line_num_reg][burst_cnt] <= i_l2_wrt_line_data; 
               burst_cnt                                                <= burst_cnt + 1;
             end                
         end else begin        
              o_l2_cmd_done         = 1'b1;     
              T_LINE_VALID[l2_set_num_reg][l2_line_num_reg] <= 1'b1;
              T_LINE_DIRTY[l2_set_num_reg][l2_line_num_reg] <= l2_wrt_line_dirty_reg;
              T_LINE_TAG[l2_set_num_reg][l2_line_num_reg]   <= l2_tag_reg; 
              o_l2_busy                                     <= 1'b0;
                                                     cstate <= IDLE; 
         end 
      end
   endcase 
   
end 
 

end 


 
//=========================================================    
// LRU CIRCUIT 
//=========================================================  
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      for(int set = 0 ; set<(2**INDEXSIZE) ;set++)
      begin
         for(int line = 0 ; line<(2**WAYSIZE) ;line++)
         begin
            T_LRU_CNTS[set][line] <= line;
         end 
      end 
   end else begin
      if(i_l2_cs & (i_l2_cmd==`LOAD_CMD || i_l2_cmd==`STORE_CMD ||i_l2_cmd==`BURST_READ_CMD ||i_l2_cmd==`BURST_WRITE_CMD ) )
      begin
         for(int line = 0 ; line<(2**WAYSIZE) ;line++)
         begin
            if(T_LRU_CNTS[l2_index][line]> T_LRU_CNTS[l2_index][i_l2_line_num])
            begin
               T_LRU_CNTS[l2_index][line] <= (T_LRU_CNTS[l2_index][line] - 1);
            end 
         end 
         T_LRU_CNTS[l2_index][i_l2_line_num] = (2**WAYSIZE-1);
      end 
   end 
end  
//

endmodule :L2_CACHE



//=================================================
// l2_CAHCE TEST BENCH 
//=================================================
module L2_CACHE_tb();


parameter ADDRSIZE    = 32;
parameter WAYSIZE     = 4;             
parameter OFFSETSIZE  = 8;
parameter INDEXSIZE   = 3; 
parameter TAGSIZE     = 20;




//
logic                                    clk;
logic                                    rstn;
//  
logic                                    i_l2_cs;
logic [2:0]                              i_l2_cmd; 
logic                                    o_l2_cmd_done;
//
logic [(ADDRSIZE-1):0]                   i_l2_addr; 
logic [(WAYSIZE-1):0]                    i_l2_line_num;  // line num
logic [2:0]                              i_l2_funct3;
// CMD0: SET READ
logic                                    o_l2_hit;
logic [(2**WAYSIZE-1):0]                 o_l2_hit_or_victim_line;
logic                                    o_l2_hit_or_victim_line_v;       // Victim Line Valid Bit 
logic                                    o_l2_hit_or_victim_line_d;      // Victim Line Dirty Bit// CMD1: Line SIGLE-DATA Read 
logic  [31:0]                            o_l2_rd_line_data;          // Data       
// CMD2: Line SINGLE-DATA Write  
logic  [31:0]                            i_l2_wrt_line_data;         // Data  
logic                                    o_l2_rd_valid;              // Read Data Is Valid  
logic                                    i_l2_wrt_valid;             // Write Data is Valid  
logic                                    i_l2_burst_done;            // Burst Done Signal 
// CMD3: Line Burst Read 
// Set Number 
// Line Number 
// CMD4: Line Burst Write   
logic                                   i_l2_wrt_line_dirty;
logic                                   o_l2_burst_write_ready;
logic                                   o_l2_busy;
logic                                   i_l2_inclusion_bit;

int error;
logic [(2**(OFFSETSIZE-2)-1):0][31:0]  Test_Line;


logic [(TAGSIZE-1):0]    l2_tag;  
logic [(INDEXSIZE-1):0]  l2_index;   
logic [(OFFSETSIZE-1):0] l2_offset;   
  
assign l2_index  = i_l2_addr[(OFFSETSIZE+INDEXSIZE-1):OFFSETSIZE];
assign l2_offset = i_l2_addr[(OFFSETSIZE-1):0];
assign l2_tag    = i_l2_addr[(ADDRSIZE-1):(OFFSETSIZE+INDEXSIZE)];

//====================================================
// Clock Generation 
//====================================================
initial begin
 clk = 1'b0; 
 fork
  forever #10 clk = ~clk;
 join
end 

//====================================================
// INITIALIZE task: It is used to initialize the cache with random numbers.  
//====================================================
task INITIALIZE();
begin
/* // CACHE MEMORY
 // cache_mem[set][line][line_row][line_column]
 //            SET            LINE            LINE_ROW       LINE_COLMN    
 logic   [(INDEXSIZE-1):0][(WAYSIZE-1):0][((OFFSETSIZE-2)-1):0][31:0]  T_CACHE_DATA; 
 //            SET            LINE
 logic   [(INDEXSIZE-1):0][(WAYSIZE-1):0]              T_LINE_VALID; 
 logic   [(INDEXSIZE-1):0][(WAYSIZE-1):0]              T_LINE_DIRTY; 
 //            SET            LINE           TAG                                                       
 logic [(INDEXSIZE-1):0][(WAYSIZE-1):0][(TAGSIZE-1):0] T_LINE_TAG; */
 for(int set=0;set<(2**INDEXSIZE);set++)
 begin
    for(int line=0;line<(2**WAYSIZE);line++)
    begin 
       DUT.T_LINE_VALID[set][line] <= $urandom%2;
       DUT.T_LINE_DIRTY[set][line] <= $urandom%2;
       DUT.T_LINE_TAG[set][line]   <= $urandom;
       for(int line_row=0;line_row<(2**(OFFSETSIZE-2));line_row++)
       begin  
          DUT.T_CACHE_DATA[set][line][line_row] <= $urandom;
       end
    end  
 end 
end
endtask


//====================================================
// RESET TASK: Resets the DUT 
//====================================================
task RESET();
begin
   error = '0;
   // 
   i_l2_cs             = '0;
   i_l2_cmd            = '0;
   i_l2_wrt_line_data  = '0;
   l2_index            = '0;
   i_l2_line_num       = '0;
   i_l2_funct3         = '0;
   l2_offset           = '0;
   i_l2_burst_done     = '0;
   i_l2_wrt_valid      = '0;
   i_l2_wrt_line_dirty = '0;
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2)@(posedge clk);
   rstn = 1'b1;
end 
endtask


task HIT_CHECK(input logic  [(ADDRSIZE-1):0] l2_addr);
begin
   @(posedge clk);
      i_l2_cs        <= 1'b1;
      i_l2_cmd       <= `CACHE_HIT_CHECK_CMD;      
      i_l2_addr      <= l2_addr;
   @(posedge clk);   
      i_l2_cs        <= '0;
      i_l2_cmd       <= '0;    
      i_l2_addr      <= l2_addr;
      //
end 
endtask


task READ_SINGLE_DATA(input logic  [(ADDRSIZE-1):0]       l2_addr,
                      input logic  [($clog2(WAYSIZE)):0]  l2_line_num,  // line num
                      input logic  [2:0]                  l2_funct3,
                      input logic  [31:0]                 ex_rd_data);  
begin
   l2_index  = l2_addr[(OFFSETSIZE+INDEXSIZE-1):OFFSETSIZE];
   @(posedge clk);
      i_l2_cs        <= 1'b1;
      i_l2_cmd       <= `LOAD_CMD;      
      i_l2_addr      <= l2_addr;
      i_l2_funct3    <= l2_funct3;
      l2_offset      <= l2_offset;
   @(posedge clk);   
      i_l2_cs        <= '0;
      i_l2_cmd       <= '0;    
      l2_index       <= '0;
      i_l2_line_num  <= '0;
      i_l2_funct3    <= '0;
      l2_offset      <= '0;
      //
      #2;
      if(o_l2_rd_line_data != ex_rd_data)         begin error++; end     
      //
end 
endtask


task WRITE_SINGLE_DATA(input logic  [31:0]                l2_wrt_line_data,
                       input logic  [(ADDRSIZE-1):0]      l2_addr,
                       input logic  [($clog2(WAYSIZE)):0] l2_line_num,  // line num
                       input logic  [2:0]                 l2_funct3);   // offset );
begin
   @(posedge clk);
      i_l2_cs            <= 1'b1;
      i_l2_cmd           <= `STORE_CMD;    
      i_l2_wrt_line_data <= l2_wrt_line_data; 
      i_l2_addr          <= l2_addr;
      i_l2_line_num      <= l2_line_num;
      i_l2_funct3        <= l2_funct3;
   @(posedge clk);   
      i_l2_cs            <= '0;
      i_l2_cmd           <= '0;  
      i_l2_addr          <= '0;  
      i_l2_wrt_line_data <= '0;   
      i_l2_line_num      <= '0;
      i_l2_funct3        <= '0;
      //
      //#2;
      //if(o_l2_rd_set_tags != DUT.T_LINE_TAG[l2_set_num])         begin error++; end     
      //
end 
endtask

// BURST READ TASK 
task BURST_READ (input logic  [(INDEXSIZE-1):0]     l2_addr,
                 input logic  [(WAYSIZE-1):0]       l2_line_num);
begin
   l2_index  = l2_addr[(OFFSETSIZE+INDEXSIZE-1):OFFSETSIZE];
   @(posedge clk);
      i_l2_cs            <= 1'b1;
      i_l2_cmd           <= `BURST_READ_CMD;    
      i_l2_addr          <= l2_addr;
      i_l2_line_num      <= l2_line_num;
   @(posedge clk);   
      i_l2_cs            <= '0;
      i_l2_cmd           <= '0;   
      i_l2_addr           <= '0;
      i_l2_line_num      <= '0; 
      //
   @(posedge clk);   
      for(int i=0;i<(2**(OFFSETSIZE-2));i++)
      begin
         #2; 
         if(o_l2_rd_line_data != DUT.T_CACHE_DATA[l2_index][l2_line_num][i]) begin error++; end 
         @(posedge clk);
      end 
end 
endtask 


task BURST_WRITE (input logic  [(INDEXSIZE-1):0]     l2_addr,
                  input logic  [(WAYSIZE-1):0]       l2_line_num);
begin
   // Initialize the test array
   for(int i=0;i<(2**(OFFSETSIZE-2));i++)
   begin
       Test_Line[i] <= $urandom;
   end 
   // START BURST WRITE 
   @(posedge clk);
      i_l2_cs            <= 1'b1;
      i_l2_cmd           <= `BURST_WRITE_CMD;    
      i_l2_addr          <= l2_addr;
      i_l2_line_num      <= l2_line_num;
   @(posedge clk);   
      i_l2_cs            <= '0;
      i_l2_cmd           <= '0;   
      l2_index           <= '0;
      i_l2_line_num      <= '0;  
   for(int i=0;i<(2**(OFFSETSIZE-2));i++)
   begin
       i_l2_wrt_valid      <= 1'b1;
       i_l2_wrt_line_data  <= Test_Line[i]; 
       @(posedge clk);
   end 
       i_l2_wrt_valid      <= 1'b0;
   // CHECK THE CACHE DATA LINE     
   @(posedge clk);
   #2;
   for(int i=0;i<(2**(OFFSETSIZE-2));i++)
   begin
        if(DUT.T_CACHE_DATA[l2_index][l2_line_num][i] != Test_Line[i]) begin error++; end 
   end  
   //
end 
endtask 


task UPDATE_INCLUSION(input logic  [(ADDRSIZE-1):0] l2_addr,
                      input logic  [(WAYSIZE-1):0]  l2_line_num,
                      input logic                   l2_inclusion_bit);
begin
   @(posedge clk);
      i_l2_cs            <= 1'b1;
      i_l2_cmd           <= `UPDATE_INCLUSION_CMD;      
      i_l2_addr          <= l2_addr;
      i_l2_line_num      <= l2_line_num;
      i_l2_inclusion_bit <= l2_inclusion_bit; 
   @(posedge clk);   
      i_l2_cs            <= '0;
      i_l2_cmd           <= '0;    
      i_l2_addr          <= '0;
      i_l2_line_num      <= '0;
      i_l2_inclusion_bit <= '0;
      //
end 
endtask


// Read Write Test 
task test2();
begin
   RESET();
   INITIALIZE();
   
   WRITE_SINGLE_DATA( .l2_wrt_line_data ('hdeadbeef),     .l2_addr   ('d2),   .l2_line_num ('d0),   .l2_funct3  ('d010));   // offset );
   READ_SINGLE_DATA(  .l2_addr ('d0), .l2_line_num ('d0), .l2_funct3 ('d010), .ex_rd_data ('hdeadbeef)  );   // offset );

   //
   if(error==0) begin  $display("TEST2: ALL GOOD! NO ERROR!");        end 
   else         begin  $display("TEST2: !!ERROR!! %d Errors", error); end 

end 
endtask


// Read Write Test 
task test3();
begin
   RESET();
   INITIALIZE();
   //
   BURST_READ ( .l2_addr  ('d1), .l2_line_num ('d2) );
   //
   if(error==0) begin  $display("TEST3: ALL GOOD! NO ERROR!");        end 
   else         begin  $display("TEST3: !!ERROR!! %d Errors", error); end 
end 
endtask


// Read Write Test 
task test4();
begin
   RESET();
   //INITIALIZE();
   //
   BURST_WRITE (.l2_addr ('d0), .l2_line_num ('d0));
   BURST_WRITE (.l2_addr ('d0), .l2_line_num ('d4));
   BURST_WRITE (.l2_addr ('d0), .l2_line_num ('d8));
   BURST_WRITE (.l2_addr ('d0), .l2_line_num ('d12));
   //
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d0), .l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d1), .l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d2), .l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d3),.l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d4), .l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d5), .l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d6), .l2_inclusion_bit(1'b1));
   UPDATE_INCLUSION(.l2_addr ('h0),.l2_line_num ('d7),.l2_inclusion_bit(1'b1));
   //
   HIT_CHECK(.l2_addr ('h12340000));
   if(error==0) begin  $display("TEST4: ALL GOOD! NO ERROR!");        end 
   else         begin  $display("TEST4: !!ERROR!! %d Errors", error); end 
end 
endtask



// Read Write Test 
task test5();
begin
   RESET();
   INITIALIZE();
   //
   for(int set=0;set<(2**INDEXSIZE);set++)
   begin
      for(int line=0;line<WAYSIZE;line++)
      begin
         READ_SINGLE_DATA(.l2_addr  (set), .l2_line_num (line), .l2_funct3   ('d010), .ex_rd_data (DUT.T_CACHE_DATA[set][line][0]));   // offset );
      end
   end 
   //   
   //
   for(int set=0;set<(2**INDEXSIZE);set++)
   begin
      for(int line=0;line<WAYSIZE;line++)
      begin
         READ_SINGLE_DATA(.l2_addr  (set), .l2_line_num (line), .l2_funct3 ('d010), .ex_rd_data (DUT.T_CACHE_DATA[set][line][0]));   // offset );
      end
   end 
   // NO SELF CHECK
   //if(error==0) begin  $display("TEST5: ALL GOOD! NO ERROR!");        end 
   //else         begin  $display("TEST5: !!ERROR!! %d Errors", error); end 
end 
endtask


// MAIN TEST FLOW 
initial begin
   RESET();
   //INITIALIZE();
   //test1();
   //test2();
   //
   //test3();
   test4();
   //test5();
   //HIT_CHECK( .l2_addr ('hdeadbeef) );
   
   
   
   repeat(4) @(posedge clk);
   $finish;
end 







  
  
// DUT Instantiation 
L2_CACHE
#(.ADDRSIZE    (ADDRSIZE),
  .WAYSIZE     (WAYSIZE),             
  .OFFSETSIZE  (OFFSETSIZE),
  .INDEXSIZE   (INDEXSIZE), 
  .TAGSIZE     (TAGSIZE))
DUT  
(.*);


endmodule :L2_CACHE_tb
