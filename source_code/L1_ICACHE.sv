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


module L1_ICACHE
#(parameter WAYSIZE    = 4,             
  parameter OFFSETBIT  = 10,
  parameter INDEXBIT   = 8, 
  parameter TAGBIT     = 32 - INDEXBIT - OFFSETBIT,
  parameter PASIZE     = (TAGBIT + INDEXBIT + OFFSETBIT))
(
   input logic clk,
   input logic rstn,
   // 
   input logic i_flush,
   //  
   input  logic                                   i_l1_cs,
   input  logic [2:0]                             i_l1_cmd, 
   output logic                                   o_l1_cmd_done,
   //
 
   input  logic  [(PASIZE-1):0]                   i_l1_addr,
   input  logic  [(WAYSIZE-1):0]                  i_l1_line_num,              // line num
   input  logic  [2:0]                            i_l1_funct3,                // Size (byte,half-byte,word) 
   // CMD0: SET READ
   input  logic                                   i_l1_rd_set,
   input  logic [(PASIZE-1):0]                    i_l1_rd_set_addr,
   output logic [(2**WAYSIZE-1):0][(TAGBIT-1):0]  o_l1_rd_set_tags, 
   output logic [(2**WAYSIZE-1):0]                o_l1_rd_set_valid_bits, 
   output logic [(2**WAYSIZE-1):0]                o_l1_rd_set_dirty_bits,
   output logic [(WAYSIZE-1):0]                   o_l1_plru_line_num,
   // CMD1: Line SIGLE-DATA Read 
   output logic  [31:0]                           o_l1_rd_line_data,          // Data  
   output logic                                   o_l1_rd_valid,              // Read Data,tags,valid_bits VALID.        
   input  logic                                   i_l1_wrt_valid,             // Burst Read Value Valid.
   input  logic                                   i_l1_burst_done,            //   
   // CMD2: Line SINGLE-DATA Write  
   input  logic  [31:0]                           i_l1_wrt_line_data,          // Data  
   // CMD3: Line Burst Read 
   // CMD4: Line Burst Write   
   output logic                                   o_l1_burst_write_ready,
   output logic                                   o_l1_busy
    );
    
    
 // CACHE MEMORY
 // cache_mem[set][line][line_row][line_column]
 //            SET            LINE            LINE_ROW       LINE_COLMN    
 logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(2**(OFFSETBIT-2)-1):0][31:0]  T_CACHE_DATA; 
 //            SET            LINE
 logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                                T_LINE_VALID; 
 logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                                T_LINE_DIRTY; 
 //            SET            LINE           TAG                                                       
 logic [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0][(TAGBIT-1):0]                  T_LINE_TAG;  
//  
logic  [(2**INDEXBIT-1):0][(2**WAYSIZE-1):0]                                T_PLRU_bits; 
//
logic [(OFFSETBIT-1):0]    burst_cnt;
logic [(WAYSIZE-1):0]      l1_line_num_reg; // line num
logic                      loop_break; 
// CMD3: Burst-Line Read 
// CMD4: Burst-Data Write
logic  [(INDEXBIT-1):0]                 i_index,i_index_reg, i_index_set;    
logic  [(OFFSETBIT-1):0]                i_offset,i_offset_reg;  
logic  [(TAGBIT-1):0]                   i_tag,i_tag_reg;  
  
  
typedef enum{
   IDLE,
   BURST_READ,
   BURST_WRITE
} state_type;
 
state_type cstate,nstate;

			    
assign i_offset =  i_l1_addr[(OFFSETBIT-1):0];
assign i_index  =  i_l1_addr[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];
assign i_tag    =  i_l1_addr[(PASIZE-1):(OFFSETBIT+INDEXBIT)];
		
assign i_index_set  =  i_l1_rd_set_addr[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];			    

always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      o_l1_rd_set_tags        = '0;
      o_l1_rd_set_valid_bits  = '0;
      o_l1_rd_set_dirty_bits  = '0;
   end else begin
      if(i_l1_rd_set)
      begin
         o_l1_rd_set_tags        <= T_LINE_TAG[i_index_set];
         o_l1_rd_set_valid_bits  <= T_LINE_VALID[i_index_set];
         o_l1_rd_set_dirty_bits  <= T_LINE_DIRTY[i_index_set];
         // Select the Pseudo Least Recently Algorithm 
         loop_break = 1'b0;
         for(int line=0;line<(2**WAYSIZE);line++)
         begin
            if((~T_LINE_VALID[i_index][line]) && ~loop_break)
            begin   
               loop_break = 1'b1;
               o_l1_plru_line_num = line; 
            end 
         end // for(    
         //
         for(int line=0;line<(2**WAYSIZE);line++)
         begin
            if((~T_PLRU_bits[i_index][line]) && ~loop_break)
            begin   
               loop_break = 1'b1;
               o_l1_plru_line_num = line; 
            end 
         end // for(    
                 
      end 
   end 
end 








always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn || i_flush)
   begin
   
       cstate                  <= IDLE;
       o_l1_cmd_done           <= '0;
       o_l1_burst_write_ready  <= '0;
       //
       o_l1_rd_line_data       <= '0;
       //
       l1_line_num_reg         <= '0;  
       //
       o_l1_rd_valid           <= '0;
       //
       T_LINE_VALID            <= '0;
       T_LINE_DIRTY            <= '0;
       o_l1_busy               <= '0;
	   i_index_reg             <= '0; 
	   i_offset_reg            <= '0; 
	   i_tag_reg               <= '0;
   end else begin
      o_l1_cmd_done = '0;
         // 
      case(cstate)
         IDLE:
         begin
         o_l1_burst_write_ready = 1'b0;
         //
         if(i_l1_cs)
         begin
		    // 
		    i_offset_reg    <=  i_l1_addr[(OFFSETBIT-1):0];
		    i_index_reg     <=  i_l1_addr[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];
			i_tag_reg       <=  i_l1_addr[(PASIZE-1):(OFFSETBIT+INDEXBIT)];
			l1_line_num_reg <=  i_l1_line_num;
			//
            case(i_l1_cmd)
                3'b000: 
                begin 
                   // DO  nothing                  
                end 
                3'b001: 
                begin 
                     o_l1_cmd_done = 1'b1;
                     //
                     o_l1_rd_line_data = T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)];
                end 
                3'b010: 
                begin 
                     o_l1_cmd_done = 1'b1;
                     //
                    case(i_l1_funct3[1:0])
                    2'b00: // SB
                    begin 
                       case(i_offset[1:0])
                       2'b00: begin T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)][7:0]   <= i_l1_wrt_line_data[7:0];  end 
                       2'b01: begin T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)][15:8]  <= i_l1_wrt_line_data[7:0];  end 
                       2'b10: begin T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)][23:16] <= i_l1_wrt_line_data[7:0];  end 
                       2'b11: begin T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)][31:24] <= i_l1_wrt_line_data[7:0];  end            
                       endcase
                    end 
                   2'b01: // SH
                   begin
                      case(i_offset[1])
                         1'b0: begin T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)][15:0]   <= i_l1_wrt_line_data[15:0];  end 
                         1'b1: begin T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)][31:16]  <= i_l1_wrt_line_data[15:0];  end          
                      endcase
                  end 
                  2'b10: // SW
                  begin
                     T_CACHE_DATA[i_index][i_l1_line_num][(i_offset>>2)] <= i_l1_wrt_line_data;  
                  end 
                  endcase
                     T_LINE_DIRTY[i_index][i_l1_line_num] = 1'b1;
                end 
                3'b011: 
                begin 
                   burst_cnt       <= '0;  
                   o_l1_busy       <= 1'b1;
                                                                                       cstate <= BURST_READ;
                end 
                3'b100: 
                begin 
                   burst_cnt              <= '0;  
                   o_l1_burst_write_ready  = 1'b1;
                   o_l1_busy              <= 1'b1;
                                                                                       cstate <= BURST_WRITE;
                end 
            endcase 
            //
            end
         end
      BURST_READ:        
      begin
         if(~(burst_cnt==(2**(OFFSETBIT-2))) && ~i_l1_burst_done)
         begin
               o_l1_rd_line_data <= T_CACHE_DATA[i_index_reg][l1_line_num_reg][burst_cnt]; 
               o_l1_rd_valid     <= 1'b1;  
               burst_cnt         <= burst_cnt + 1;               
         end else begin   
               o_l1_cmd_done = 1'b1;
               //     
               o_l1_rd_valid     <= 1'b0; 
               o_l1_busy         <= 1'b0;
                                                         cstate <= IDLE; 
         end 
      end
      BURST_WRITE:
      begin
         if(~(burst_cnt==(2**(OFFSETBIT-2))) && ~i_l1_burst_done)
         begin
             if(i_l1_wrt_valid)
             begin
               T_CACHE_DATA[i_index_reg][l1_line_num_reg][burst_cnt] <= i_l1_wrt_line_data; 
               burst_cnt         <= burst_cnt + 1;
             end                
         end else begin 
              o_l1_cmd_done = 1'b1;
              //    
              T_LINE_VALID[i_index_reg][l1_line_num_reg] <= 1'b1;
              T_LINE_DIRTY[i_index_reg][l1_line_num_reg] <= 1'b0;
			  T_LINE_TAG[i_index_reg][l1_line_num_reg]   <= i_tag_reg;
              o_l1_busy              <= 1'b0;
                                                         cstate <= IDLE; 
         end 
      end
   endcase 
   
end 
 

end 


 
//=========================================================    
// Bit-Pseudo-LRU CIRCUIT 
//=========================================================  
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      T_PLRU_bits <= '0;    
   end else begin
      if(i_l1_cs & (i_l1_cmd=='d1 || i_l1_cmd=='d2 ||i_l1_cmd=='d3 || i_l1_cmd=='d4) )
      begin
         T_PLRU_bits[i_index_reg][i_l1_line_num] = 1'b1;
            if(&T_PLRU_bits[i_index])
            begin
               T_PLRU_bits[i_index] = '0; 
               T_PLRU_bits[i_index][i_l1_line_num] = 'b1;                
            end  
      end 
   end 
end  
//
 
 
 
    
endmodule :L1_ICACHE



//=================================================
// L1_CAHCE TEST BENCH 
//=================================================
module L1_ICACHE_tb();

   parameter WAYSIZE    = 4;             
   parameter OFFSETBIT  = 10;
   parameter INDEXBIT   = 8; 
   parameter TAGBIT     = 20;
   parameter PASIZE     = (TAGBIT + INDEXBIT + OFFSETBIT);


   logic clk;
   logic rstn;
   //  
   logic                                   i_l1_cs;
   logic [2:0]                             i_l1_cmd; 
   logic                                   o_l1_cmd_done;
   //
   logic  [(PASIZE-1):0]                   i_l1_addr;
   logic  [(WAYSIZE-1):0]                  i_l1_line_num;          
   logic  [2:0]                            i_l1_funct3;            
   // CMD0: SET READ
   logic                                   i_l1_rd_set;
   logic [(PASIZE-1):0]                    i_l1_rd_set_addr;
   logic [(2**WAYSIZE-1):0][(TAGBIT-1):0]  o_l1_rd_set_tags; 
   logic [(2**WAYSIZE-1):0]                o_l1_rd_set_valid_bits; 
   logic [(2**WAYSIZE-1):0]                o_l1_rd_set_dirty_bits;
   logic [(WAYSIZE-1):0]                   o_l1_plru_line_num;
   // CMD1: Line SIGLE-DATA Read 
   logic  [31:0]                           o_l1_rd_line_data;      
   logic                                   o_l1_rd_valid;          
   logic                                   i_l1_wrt_valid;         
   logic                                   i_l1_burst_done;        
   // CMD2: Line SINGLE-DATA Write  
   logic  [31:0]                           i_l1_wrt_line_data;     
   // CMD3: Line Burst Read 
   // CMD4: Line Burst Write   
   logic                                   o_l1_burst_write_ready;
   logic                                   o_l1_busy;           


  logic  [(INDEXBIT-1):0]                 index;    
  logic  [(OFFSETBIT-1):0]                offset;  
  logic  [(TAGBIT-1):0]                   tag;  

int error;
logic [(2**(OFFSETBIT-2)-1):0][31:0]  Test_Line;
//
initial begin
 clk = 1'b0; 
 fork
  forever #10 clk = ~clk;
 join
end 


task INITIALIZE();
begin
/* // CACHE MEMORY
 // cache_mem[set][line][line_row][line_column]
 //            SET            LINE            LINE_ROW       LINE_COLMN    
 logic   [(i_index_regSIZE-1):0][(WAYSIZE-1):0][((i_offset_regSIZE-2)-1):0][31:0]  T_CACHE_DATA; 
 //            SET            LINE
 logic   [(i_index_regSIZE-1):0][(WAYSIZE-1):0]              T_LINE_VALID; 
 logic   [(i_index_regSIZE-1):0][(WAYSIZE-1):0]              T_LINE_DIRTY; 
 //            SET            LINE           i_tag_reg                                                       
 logic [(i_index_regSIZE-1):0][(WAYSIZE-1):0][(i_tag_regSIZE-1):0] T_LINE_i_tag_reg; */
 for(int set=0;set<(2**INDEXBIT);set++)
 begin
    for(int line=0;line<(2**WAYSIZE);line++)
    begin 
       DUT.T_LINE_VALID[set][line] <= $urandom%2;
       DUT.T_LINE_DIRTY[set][line] <= $urandom%2;
       DUT.T_LINE_TAG[set][line]   <= $urandom;
       for(int line_row=0;line_row<(2**(OFFSETBIT-2));line_row++)
       begin  
          DUT.T_CACHE_DATA[set][line][line_row] <= $urandom;
       end
    end  
 end 
end
endtask



task RESET();
begin
   error = '0;
   // 
   i_l1_cs            = '0;
   i_l1_cmd           = '0;
   i_l1_addr          = '0;
   i_l1_line_num      = '0;
   i_l1_funct3        = '0;
   i_l1_wrt_valid     = '0;
   i_l1_burst_done    = '0;
   i_l1_wrt_line_data = '0;
   i_l1_rd_set        = '0;
   i_l1_rd_set_addr   = '0;
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2)@(posedge clk);
   rstn = 1'b1;
end 
endtask


task READ_SET(input logic  [(PASIZE-1):0] l1_addr);
begin
   @(posedge clk);    
      i_l1_rd_set        <= 1'b1;
      i_l1_rd_set_addr   <= l1_addr; 
      index              <= l1_addr[((OFFSETBIT+INDEXBIT)-1):OFFSETBIT];
   @(posedge clk);   
      i_l1_rd_set        <= '0;
      i_l1_rd_set_addr   <= '0;
      #2;
      if(o_l1_rd_set_tags       != DUT.T_LINE_TAG[index])       begin error++; end    
      if(o_l1_rd_set_valid_bits != DUT.T_LINE_VALID[index])     begin error++; end  
      if(o_l1_rd_set_dirty_bits != DUT.T_LINE_DIRTY[index])     begin error++; end  
end 
endtask



/*
// BURST READ TASK 
task BURST_READ (input logic  [(i_index_regBIT-1):0]      l1_set_num,
                 input logic  [(WAYSIZE-1):0]       l1_line_num);
begin
   @(posedge clk);
      i_l1_cs            <= 1'b1;
      i_l1_cmd           <= 3'b011;    
      i_index_reg       <= l1_set_num;
      i_l1_line_num      <= l1_line_num;
   @(posedge clk);   
      i_l1_cs            <= '0;
      i_l1_cmd           <= '0;   
      i_index_reg       <= '0;
      i_l1_line_num      <= '0; 
      //
   @(posedge clk);   
      for(int i=0;i<(2**(i_offset_regBIT-2));i++)
      begin
         #2; 
         if(o_l1_rd_line_data != DUT.T_CACHE_DATA[l1_set_num][l1_line_num][i]) begin error++; end 
         @(posedge clk);
      end 
end 
endtask 


task BURST_WRITE (input logic  [(i_index_regBIT-1):0]      l1_set_num,
                  input logic  [(WAYSIZE-1):0]       l1_line_num);
begin
   // Initialize the test array
   for(int i=0;i<(2**(i_offset_regBIT-2));i++)
   begin
       Test_Line[i] <= $urandom;
   end 
   // START BURST WRITE 
   @(posedge clk);
      i_l1_cs            <= 1'b1;
      i_l1_cmd           <= 3'b100;    
      i_index_reg       <= l1_set_num;
      i_l1_line_num      <= l1_line_num;
   @(posedge clk);   
      i_l1_cs            <= '0;
      i_l1_cmd           <= '0;   
      i_index_reg       <= '0;
      i_l1_line_num      <= '0;  
   for(int i=0;i<(2**(i_offset_regBIT-2));i++)
   begin
       i_l1_wrt_valid      <= 1'b1;
       i_l1_wrt_line_data  <= Test_Line[i]; 
       @(posedge clk);
   end 
       i_l1_wrt_valid      <= 1'b0;
   // CHECK THE CACHE DATA LINE     
   @(posedge clk);
   #2;
   for(int i=0;i<(2**(i_offset_regBIT-2));i++)
   begin
        if(DUT.T_CACHE_DATA[l1_set_num][l1_line_num][i] != Test_Line[i]) begin error++; end 
   end  
   //
end 
endtask 
*/


//===========================
// READ SET TEST 
//===========================
task test1();
begin
   RESET();
   INITIALIZE();
   
   for(int i=0; i<(2**INDEXBIT); i++)
   begin
      READ_SET( .l1_addr (i) );
   end 
   //
   if(error==0) begin  $display("TEST1: ALL GOOD! NO ERROR!");        end 
   else         begin  $display("TEST1: !!ERROR!! %d Errors", error); end  
end 
endtask


/*
// Read Write Test 
task test3();
begin
   RESET();
   INITIALIZE();
   //
   BURST_READ ( .l1_set_num  ('d1), .l1_line_num ('d2) );
   //
   if(error==0) begin  $display("TEST3: ALL GOOD! NO ERROR!");        end 
   else         begin  $display("TEST3: !!ERROR!! %d Errors", error); end 
end 
endtask


// Read Write Test 
task test4();
begin
   RESET();
   INITIALIZE();
   //
   BURST_WRITE (.l1_set_num ('d1), .l1_line_num ('d2));
   BURST_WRITE (.l1_set_num ('d3), .l1_line_num ('d0));
   //
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
   for(int set=0;set<(2**i_index_regBIT);set++)
   begin
      for(int line=0;line<WAYSIZE;line++)
      begin
         READ_SINGLE_DATA(.l1_set_num  (set), .l1_line_num (line), .l1_funct3   ('d010), .l1_i_offset_reg   ('d0), .ex_rd_data (DUT.T_CACHE_DATA[set][line][0]));   // i_offset_reg );
      end
   end 
   //   
   for(int set=0;set<(2**i_index_regBIT);set++)
   begin
      for(int line=0;line<WAYSIZE;line++)
      begin
         READ_SINGLE_DATA(.l1_set_num  (set), .l1_line_num (line), .l1_funct3   ('d010), .l1_i_offset_reg   ('d0), .ex_rd_data (DUT.T_CACHE_DATA[set][line][0]));   // i_offset_reg );
      end
   end 
   //
   // NO SELF CHECK
   //if(error==0) begin  $display("TEST5: ALL GOOD! NO ERROR!");        end 
   //else         begin  $display("TEST5: !!ERROR!! %d Errors", error); end 
end 
endtask
*/

// MAIN TEST FLOW 
initial begin
   test1();

   repeat(4) @(posedge clk);
   $finish;
end 

//=========================
// DUT Instantiation 
//=========================
L1_ICACHE
#(.WAYSIZE    (WAYSIZE),             
  .OFFSETBIT  (OFFSETBIT),   
  .INDEXBIT   (INDEXBIT),   
  .TAGBIT     (TAGBIT))
DUT  
(.*);


endmodule :L1_ICACHE_tb
