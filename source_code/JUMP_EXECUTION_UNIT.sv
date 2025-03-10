`timescale 1ns / 1ps
`include "headers.vh"
//========================================
// JUMP EXECUTION UNIT 
//========================================
// Calculates the Taken/Non-Taken result for the Jump Instruction 
//========================================


//////////////////////////////////////////////////////////////////////////////////
// Jump Execution Unit  
// 1- Compare Rs1 and Rs2 for Conditional Branch Instructions. 
// 2- Based on the Result of the Comparison and the Decode signals create "taken_cal" 
// 3- taken_cal is always 1 for JAL and JALR instructions. 
// imm[20|10:1|11|19:12]             rd 1101111 JAL 
// imm[11:0]        rs1 000          rd 1100111 JALR 		
// imm[12|10:5] rs2 rs1 000 imm[4:1|11] 1100011 BEQ		
// imm[12|10:5] rs2 rs1 001 imm[4:1|11] 1100011 BNE 	
// imm[12|10:5] rs2 rs1 100 imm[4:1|11] 1100011 BLT 	
// imm[12|10:5] rs2 rs1 101 imm[4:1|11] 1100011 BGE 	
// imm[12|10:5] rs2 rs1 110 imm[4:1|11] 1100011 BLTU 	
// imm[12|10:5] rs2 rs1 111 imm[4:1|11] 1100011 BGEU
//////////////////////////////////////////////////////////////////////////////////
/*
logic signed [31:0] op1_signed;
logic signed [31:0] op2_signed;
assign op1_signed = jeu_op1;
assign op2_signed = jeu_op2;  
      
always_comb
begin
   // 
   casex({jeu_control_transfer_inst,jeu_funct3})
     6'b1xxxxx:            jeu_taken_cal = 1'b1;
     6'bx1xxxx:            jeu_taken_cal = 1'b1;
    {3'b001,`FUNCT3_BEQ }: jeu_taken_cal = (jeu_op1==jeu_op2); 
    {3'b001,`FUNCT3_BNE }: jeu_taken_cal = (jeu_op1!=jeu_op2);
    {3'b001,`FUNCT3_BLT }: jeu_taken_cal = (op1_signed < jeu_op2);
    {3'b001,`FUNCT3_BGE }: jeu_taken_cal = (op1_signed >= jeu_op2);           
    {3'b001,`FUNCT3_BLTU}: jeu_taken_cal = (jeu_op1 < jeu_op2);
    {3'b001,`FUNCT3_BGEU}: jeu_taken_cal = (jeu_op1 >= jeu_op2);
    default:               jeu_taken_cal = '0;
   endcase 
   //  
end 
*/
/*

   
 JUMP_EXECUTION_UNIT
#(.JEUSIZE (JEUSIZE),
  .ROBSIZE (ROBSIZE) )
uJUMP_EXECUTION_UNIT
(
   .clk                                (clk),
   .rstn                               (rstn),
         
   .i_jeu_flush,                       (jeu_flush_i),
   //===============================    
   // ISSUE Interface                   
   //===============================    
   .i_jeu_wr_en                        (jeu_wr_en_i),
   .i_jeu_wr_rob_addr                  (jeu_wr_rob_addr_i),     
   //                                   
   .i_jeu_wr_op1                       (jeu_wr_op1_i),
   .i_jeu_wr_op1_v                     (jeu_wr_op1_v_i),
   .i_jeu_wr_op1_rob_addr,             (jeu_wr_op1_rob_addr_i),
   //                                   
   .i_jeu_wr_op2                       (jeu_wr_op2_i),
   .i_jeu_wr_op2_v                     (jeu_wr_op2_v_i),
   .i_jeu_wr_op2_rob_addr              (jeu_wr_op2_rob_addr_i),   
   //                                   
   .i_jeu_wr_funct3                    (jeu_wr_funct3_i),  
   //===============================    
   // Broadcast Interface               
   //===============================    
   .i_jeu_broadcast                    (jeu_broadcast_i),
   .i_jeu_broadcast_rob_addr           (jeu_broadcast_rob_addr_i),
   .i_jeu_broadcast_data               (jeu_broadcast_data_i),
   //===============================    
   // JEU FIFO INTERFACE                
   //===============================    
   .o_jeu_fifo_emtpy                   (jeu_fifo_emtpy_i),
   .o_jeu_fifo_taken_cal               (jeu_fifo_taken_cal_i),
   .o_jeu_fifo_rob_addr                (jeu_fifo_rob_addr_i), 
   .i_jeu_fifo_rd_en                   (jeu_fifo_rd_en_i),
   //===============================    
   // JEU                               
   //===============================    
   .o_jeu_full                         (jeu_full_i)
    );  
   
*/



module JUMP_EXECUTION_UNIT
#(parameter JEUSIZE = 3, 
  parameter ROBSIZE = 4 )
(
   input logic                      clk,  // Main Clock 
   input logic                      rstn, // Active Low Reset 
   
   input logic                      i_jeu_flush, // Flush the JEU 
   //===============================
   // ISSUE Interface 
   //===============================
   input logic                      i_jeu_wr_en,       // Write new Instruction 
   input logic [ROBSIZE-1:0]        i_jeu_wr_rob_addr, // ROB address of the instruction      
   //
   input logic [31:0]               i_jeu_wr_op1,   // 32-bit RS1 value 
   input logic                      i_jeu_wr_op1_v, // 1:RS1 value is valid, no need to wait for Broadcast
   input logic [ROBSIZE-1:0]        i_jeu_wr_op1_rob_addr, // ROB address that is going to generate the RS1 in case "i_jeu_wr_op1_v==0"
   //                
   input logic [31:0]               i_jeu_wr_op2, // 32-bit RS2 value 
   input logic                      i_jeu_wr_op2_v, // 1:RS2 value is valid, no need to wait for Broadcast
   input logic [ROBSIZE-1:0]        i_jeu_wr_op2_rob_addr,// ROB address that is going to generate the RS2 in case "i_jeu_wr_op1_v==0"    
   //
   input logic [2:0]                i_jeu_wr_funct3, // Function 3 of the instruction for Branch instruction idendification  
   //===============================
   // Broadcast Interface 
   //===============================
   input logic                      i_jeu_broadcast, // There is a broadcasted result on the BUS 
   input logic [ROBSIZE-1:0]        i_jeu_broadcast_rob_addr, // ROB address of the instruction broadcasting
   input logic [31:0]               i_jeu_broadcast_data, // Data Broadcasted 
   //===============================
   // JEU FIFO INTERFACE 
   //===============================
   output logic                     o_jeu_fifo_emtpy,
   output logic                     o_jeu_fifo_taken_cal,
   output logic [ROBSIZE-1:0]       o_jeu_fifo_rob_addr,  
   input  logic                     i_jeu_fifo_rd_en, 
   //===============================
   // JEU 
   //===============================
   output logic                     o_jeu_full // JEU unit is full. 
    );
    
    
localparam FIFODEPTH     = 3;
localparam FIFODATA0SIZE = 1; 
localparam FIFODATA1SIZE = ROBSIZE;    


//============================
// Jump Execution Unit 
//============================
logic [(2**JEUSIZE-1):0]                  T_VALID;
logic [(2**JEUSIZE-1):0] [(ROBSIZE-1):0]  T_ROB_ADDR;
logic [(2**JEUSIZE-1):0] [31:0]           T_OP1;
logic [(2**JEUSIZE-1):0] [(ROBSIZE-1):0]  T_OP1_ROB_ADDR;
logic [(2**JEUSIZE-1):0]                  T_OP1_V;
logic [(2**JEUSIZE-1):0] [31:0]           T_OP2;
logic [(2**JEUSIZE-1):0] [(ROBSIZE-1):0]  T_OP2_ROB_ADDR;
logic [(2**JEUSIZE-1):0]                  T_OP2_V;
logic [(2**JEUSIZE-1):0] [2:0]            T_FUNCT3;

logic [(2**JEUSIZE-1):0]                  T_READY; 
//============================
// LOCAL SIGNALS 
//============================
logic                         fifo_wr_en_i;
logic [(FIFODATA0SIZE-1):0]   fifo_data0_i;
logic [(FIFODATA1SIZE-1):0]   fifo_data1_i; 
logic                         fifo_rd_en_i;
logic [(FIFODATA0SIZE-1):0]   fifo_data0_o;      
logic [(FIFODATA1SIZE-1):0]   fifo_data1_o;        
logic                         fifo_full_o;
logic                         fifo_empty_o;
//
logic signed [31:0] op1_signed;
logic signed [31:0] op2_signed;
logic jeu_taken_cal;


logic loop_break0,loop_break1,loop_break2;
logic [(JEUSIZE-1):0] issue_addr, broadcast_addr_op1,broadcast_addr_op2, dispatch_addr; 
logic broadcast_en_op1,broadcast_en_op2, dispatch_en;





//===================================
// ISSUE LOGIC 
//===================================
always_comb
begin
   issue_addr  = '0; 
   loop_break0 = '0;
   for(int i0=0;i0<(2**JEUSIZE);i0++)
   begin
      if((~T_VALID[i0]) && ~loop_break0)
      begin
         issue_addr  = i0;
         loop_break0 = 1'b1;
      end 
   end 
end 


//===================================
// BROADCAST DATA CAPTURE LOGIC
//===================================
always_comb
begin
   loop_break1      = '0;
   loop_break2      = '0;
   broadcast_en_op1 = '0; 
   broadcast_en_op2 = '0;
   if(i_jeu_broadcast)
   begin
      for(int i1=0;i1<(2**JEUSIZE);i1++)
      begin
         //
         if((T_VALID[i1]) && (T_OP1_ROB_ADDR[i1]==i_jeu_broadcast_rob_addr)&& ~T_OP1_V[i1] && ~loop_break1)
         begin
            broadcast_addr_op1  = i1;
            loop_break1         = 1'b1;
            broadcast_en_op1    = 1'b1;  
         end 
         //
         if((T_VALID[i1]) && (T_OP2_ROB_ADDR[i1]==i_jeu_broadcast_rob_addr)&& ~T_OP2_V[i1] && ~loop_break2)
         begin
            broadcast_addr_op2  = i1;
            loop_break2         = 1'b1;
            broadcast_en_op2    = 1'b1; 
         end 
         //
      end 
   end 
end 


//===================================
// DISPATCH LOGIC 
//===================================
assign T_READY = T_OP1_V & T_OP2_V;

always_comb 
begin
   dispatch_addr = '0;
   dispatch_en   = '0;
   fifo_wr_en_i  = '0; 
   fifo_data0_i  = '0;
   fifo_data1_i  = '0;
   //  
   for(int i2=0;i2<(2**JEUSIZE);i2++)
   begin
      //
      if(T_VALID[i2] && T_READY[i2]) 
      begin
         dispatch_addr = i2;
         dispatch_en   = 1'b1;
         
         
         //=====================
         // TAKEN CALCULATION LOGIC 
         //=====================
         op1_signed = T_OP1[dispatch_addr];
         op2_signed = T_OP2[dispatch_addr];  
         //
         case({T_FUNCT3[dispatch_addr]})
          `FUNCT3_BEQ : jeu_taken_cal = (T_OP1[dispatch_addr] ==  T_OP2[dispatch_addr]); 
          `FUNCT3_BNE : jeu_taken_cal = (T_OP1[dispatch_addr] !=  T_OP2[dispatch_addr]);
          `FUNCT3_BLT : jeu_taken_cal = (op1_signed           <   T_OP2[dispatch_addr]);
          `FUNCT3_BGE : jeu_taken_cal = (op1_signed           >=  T_OP2[dispatch_addr]);           
          `FUNCT3_BLTU: jeu_taken_cal = (T_OP1[dispatch_addr]  <  T_OP2[dispatch_addr]);
          `FUNCT3_BGEU: jeu_taken_cal = (T_OP1[dispatch_addr] >=  T_OP2[dispatch_addr]);
           default:     jeu_taken_cal = '0;
         endcase      
         //
         //=====================
         // FIFO SIGNAL GENERATION 
         //=====================
         fifo_wr_en_i = 1'b1;
         fifo_data0_i = jeu_taken_cal;
         fifo_data1_i = T_ROB_ADDR[dispatch_addr];
         //  
       end 
           
      end    
      //
   end 







//===================================
// REGISTER 
//===================================
always_ff @(posedge clk or negedge rstn)
begin

   if(!rstn || i_jeu_flush)
   begin
      T_VALID          <= '0;
      T_ROB_ADDR       <= '0;
      T_OP1            <= '0;
      T_OP1_ROB_ADDR   <= '0;
      T_OP1_V          <= '0;
      T_OP2            <= '0;
      T_OP2_ROB_ADDR   <= '0;
      T_OP2_V          <= '0;
      T_FUNCT3         <= '0;  
   end else begin
      //===================================
      // ISSUE REGISTER  
      //===================================
      if(i_jeu_wr_en && ~o_jeu_full)
      begin
          T_VALID[issue_addr]         <= 1'b1;
          T_ROB_ADDR[issue_addr]      <= i_jeu_wr_rob_addr;
          T_OP1[issue_addr]           <= i_jeu_wr_op1;
          T_OP1_ROB_ADDR[issue_addr]  <= i_jeu_wr_op1_rob_addr;
          T_OP1_V[issue_addr]         <= i_jeu_wr_op1_v;
          T_OP2[issue_addr]           <= i_jeu_wr_op2;
          T_OP2_ROB_ADDR[issue_addr]  <= i_jeu_wr_op2_rob_addr;
          T_OP2_V[issue_addr]         <= i_jeu_wr_op2_v;
          T_FUNCT3                    <= i_jeu_wr_funct3;
      end 
      //===================================
      // BROADCAST REGISTERS 
      //===================================
      if(broadcast_en_op1)
      begin
         T_OP1[broadcast_addr_op1]    <= i_jeu_broadcast_data;
         T_OP1_V[broadcast_addr_op1]  <= 1'b1;
      end 
      // 
      if(broadcast_en_op2)
      begin
         T_OP2[broadcast_addr_op2]    <= i_jeu_broadcast_data;
         T_OP2_V[broadcast_addr_op2]  <= 1'b1;
      end 
      // 
      //===================================
      // DISPATCH REGISTERS 
      //===================================
      if(dispatch_en)
      begin
         T_VALID[dispatch_addr] <= 1'b0;
      end
   
   end 

end 


//=======================================
// FIFO INPUTS 
//=======================================
assign fifo_rd_en_i = i_jeu_fifo_rd_en;



//=======================================
// FIFO INSTANTIATION 
//=======================================
FIFO_DUAL_DATA 
#(.FIFODEPTH (FIFODEPTH),
  .DATA0SIZE (FIFODATA0SIZE), 
  .DATA1SIZE (FIFODATA1SIZE) )
uFIFO_DUAL_DATA
(
  .clk      (clk),
  .rstn     (rstn),
  //
  .i_WrtEn  (fifo_wr_en_i),
  .i_data0  (fifo_data0_i),
  .i_data1  (fifo_data1_i),
  //         
  .i_RdEn   (fifo_rd_en_i),
  .o_data0  (fifo_data0_o),   
  .o_data1  (fifo_data1_o),       
  //         
  .o_Full   (fifo_full_o),
  .o_Empty  (fifo_empty_o)
    );



//===================================
// Jump Execution Unit Output Signals 
//===================================
assign o_jeu_fifo_emtpy     = fifo_empty_o;
assign o_jeu_fifo_taken_cal = fifo_data0_o;
assign o_jeu_fifo_rob_addr  = fifo_data1_o;  
//
assign o_jeu_full = &T_VALID;


    
endmodule :JUMP_EXECUTION_UNIT



module JUMP_EXECUTION_UNIT_tb();


parameter JEUSIZE = 3; 
parameter ROBSIZE = 4;

logic                      clk; 
logic                      rstn;
					      
logic                      i_jeu_flush;
//===============================
// ISSUE Interface 
//===============================
logic                      i_jeu_wr_en; 
logic [ROBSIZE-1:0]        i_jeu_wr_rob_addr;      
				          
logic [31:0]               i_jeu_wr_op1; 
logic                      i_jeu_wr_op1_v; 
logic [ROBSIZE-1:0]        i_jeu_wr_op1_rob_addr;
				          
logic [31:0]               i_jeu_wr_op2; 
logic                      i_jeu_wr_op2_v; 
logic [ROBSIZE-1:0]        i_jeu_wr_op2_rob_addr;    
				          
logic [2:0]                i_jeu_wr_funct3;   
//===============================
// Broadcast Interface 
//===============================
logic                      i_jeu_broadcast;
logic [(ROBSIZE-1):0]      i_jeu_broadcast_rob_addr;
logic [31:0]               i_jeu_broadcast_data;
//===============================
// JEU FIFO INTERFACE 
//===============================
logic                     o_jeu_fifo_emtpy;
logic [31:0]              o_jeu_fifo_taken_cal;
logic [ROBSIZE-1:0]       o_jeu_fifo_rob_addr;  
logic                     i_jeu_fifo_rd_en; 
//===============================
// JEU 
//===============================
logic                     o_jeu_full; 
	
	
	
	
	
	
	
//===============================
// CLOCK GENERATION 
//===============================
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 



	
	
task RESET();
begin
i_jeu_flush              <= '0;
i_jeu_wr_en              <= '0; 
i_jeu_wr_rob_addr        <= '0;      
i_jeu_wr_op1             <= '0; 
i_jeu_wr_op1_v           <= '0; 
i_jeu_wr_op1_rob_addr    <= '0;
i_jeu_wr_op2             <= '0; 
i_jeu_wr_op2_v           <= '0; 
i_jeu_wr_op2_rob_addr    <= '0;    
i_jeu_wr_funct3          <= '0;   
i_jeu_broadcast          <= '0;
i_jeu_broadcast_rob_addr <= '0;
i_jeu_broadcast_data     <= '0;
i_jeu_fifo_rd_en         <= '0; 
//
   rstn <= 1'b1;
@(posedge clk); 
   rstn <= 1'b0;
repeat(2) @(posedge clk);
   rstn <= 1'b1;

end 
endtask 	
	
	
	
task ISSUE(
   input logic [ROBSIZE-1:0] jeu_wr_rob_addr,      
   input logic [31:0]        jeu_wr_op1, 
   input logic               jeu_wr_op1_v, 
   input logic [ROBSIZE-1:0] jeu_wr_op1_rob_addr,
   input logic [31:0]        jeu_wr_op2, 
   input logic               jeu_wr_op2_v, 
   input logic [ROBSIZE-1:0] jeu_wr_op2_rob_addr,    
   input logic [2:0]         jeu_wr_funct3   
);
begin
   @(posedge clk);
      i_jeu_wr_en              <= 1'b1; 
      i_jeu_wr_rob_addr        <= jeu_wr_rob_addr;      
      i_jeu_wr_op1             <= jeu_wr_op1; 
      i_jeu_wr_op1_v           <= jeu_wr_op1_v; 
      i_jeu_wr_op1_rob_addr    <= jeu_wr_op1_rob_addr;
      i_jeu_wr_op2             <= jeu_wr_op2; 
      i_jeu_wr_op2_v           <= jeu_wr_op2_v; 
      i_jeu_wr_op2_rob_addr    <= jeu_wr_op2_rob_addr;    
      i_jeu_wr_funct3          <= jeu_wr_funct3; 
  @(posedge clk);
      i_jeu_wr_en              <= 1'b0; 
      i_jeu_wr_rob_addr        <= '0;      
      i_jeu_wr_op1             <= '0; 
      i_jeu_wr_op1_v           <= '0; 
      i_jeu_wr_op1_rob_addr    <= '0;
      i_jeu_wr_op2             <= '0; 
      i_jeu_wr_op2_v           <= '0; 
      i_jeu_wr_op2_rob_addr    <= '0;    
      i_jeu_wr_funct3          <= '0; 
end 
endtask 


task BROADCAST(
   input logic [$clog2(ROBSIZE):0]  jeu_broadcast_rob_addr,
   input logic [31:0]               jeu_broadcast_data
);
begin
   @(posedge clk);
   i_jeu_broadcast          <= 1'b1;
   i_jeu_broadcast_rob_addr <= jeu_broadcast_rob_addr;
   i_jeu_broadcast_data     <= jeu_broadcast_data;
   @(posedge clk);
   i_jeu_broadcast          <= '0;
   i_jeu_broadcast_rob_addr <= '0;
   i_jeu_broadcast_data     <= '0;
end 
endtask 




//`define FUNCT3_BEQ    3'b000
//`define FUNCT3_BNE    3'b001
//`define FUNCT3_BLT    3'b100
//`define FUNCT3_BGE    3'b101
//`define FUNCT3_BLTU   3'b110
//`define FUNCT3_BGEU   3'b111
task test1();
begin
ISSUE(
   .jeu_wr_rob_addr     ('d0),      
   .jeu_wr_op1          ('d5), 
   .jeu_wr_op1_v        ('d1), 
   .jeu_wr_op1_rob_addr ('d3),
   .jeu_wr_op2          ('d5), 
   .jeu_wr_op2_v        ('d1), 
   .jeu_wr_op2_rob_addr ('d4),    
   .jeu_wr_funct3       (`FUNCT3_BEQ) 
);
ISSUE(
   .jeu_wr_rob_addr     ('d1),      
   .jeu_wr_op1          ('d5), 
   .jeu_wr_op1_v        ('d1), 
   .jeu_wr_op1_rob_addr ('d3),
   .jeu_wr_op2          ('d5), 
   .jeu_wr_op2_v        ('d1), 
   .jeu_wr_op2_rob_addr ('d4),    
   .jeu_wr_funct3       (`FUNCT3_BEQ) 
);
ISSUE(
   .jeu_wr_rob_addr     ('d2),      
   .jeu_wr_op1          ('d5), 
   .jeu_wr_op1_v        ('d1), 
   .jeu_wr_op1_rob_addr ('d3),
   .jeu_wr_op2          ('d5), 
   .jeu_wr_op2_v        ('d1), 
   .jeu_wr_op2_rob_addr ('d4),    
   .jeu_wr_funct3       (`FUNCT3_BEQ) 
);
ISSUE(
   .jeu_wr_rob_addr     ('d3),      
   .jeu_wr_op1          ('d5), 
   .jeu_wr_op1_v        ('d1), 
   .jeu_wr_op1_rob_addr ('d3),
   .jeu_wr_op2          ('d5), 
   .jeu_wr_op2_v        ('d1), 
   .jeu_wr_op2_rob_addr ('d4),    
   .jeu_wr_funct3       (`FUNCT3_BEQ) 
);
ISSUE(
   .jeu_wr_rob_addr     ('d4),      
   .jeu_wr_op1          ('d5), 
   .jeu_wr_op1_v        ('d1), 
   .jeu_wr_op1_rob_addr ('d3),
   .jeu_wr_op2          ('d5), 
   .jeu_wr_op2_v        ('d1), 
   .jeu_wr_op2_rob_addr ('d4),    
   .jeu_wr_funct3       (`FUNCT3_BEQ) 
);
ISSUE(
   .jeu_wr_rob_addr     ('d5),      
   .jeu_wr_op1          ('d5), 
   .jeu_wr_op1_v        ('d1), 
   .jeu_wr_op1_rob_addr ('d3),
   .jeu_wr_op2          ('d5), 
   .jeu_wr_op2_v        ('d1), 
   .jeu_wr_op2_rob_addr ('d4),    
   .jeu_wr_funct3       (`FUNCT3_BEQ) 
);


end 
endtask 




task test2();
begin
   ISSUE(
      .jeu_wr_rob_addr     ('d0),      
      .jeu_wr_op1          ('d0), 
      .jeu_wr_op1_v        ('d0), 
      .jeu_wr_op1_rob_addr ('d3),
      .jeu_wr_op2          ('d0), 
      .jeu_wr_op2_v        ('d0), 
      .jeu_wr_op2_rob_addr ('d4),    
      .jeu_wr_funct3       (`FUNCT3_BEQ) 
   );
   
   ISSUE(
      .jeu_wr_rob_addr     ('d1),      
      .jeu_wr_op1          ('d0), 
      .jeu_wr_op1_v        ('d0), 
      .jeu_wr_op1_rob_addr ('d5),
      .jeu_wr_op2          ('d0), 
      .jeu_wr_op2_v        ('d0), 
      .jeu_wr_op2_rob_addr ('d6),    
      .jeu_wr_funct3       (`FUNCT3_BEQ) 
   );
   
   ISSUE(
      .jeu_wr_rob_addr     ('d2),      
      .jeu_wr_op1          ('d0), 
      .jeu_wr_op1_v        ('d0), 
      .jeu_wr_op1_rob_addr ('d7),
      .jeu_wr_op2          ('d0), 
      .jeu_wr_op2_v        ('d0), 
      .jeu_wr_op2_rob_addr ('d8),    
      .jeu_wr_funct3       (`FUNCT3_BEQ) 
   );   
   
   ISSUE(
      .jeu_wr_rob_addr     ('d3),      
      .jeu_wr_op1          ('d0), 
      .jeu_wr_op1_v        ('d0), 
      .jeu_wr_op1_rob_addr ('d9),
      .jeu_wr_op2          ('d0), 
      .jeu_wr_op2_v        ('d0), 
      .jeu_wr_op2_rob_addr ('d10),    
      .jeu_wr_funct3       (`FUNCT3_BEQ) 
   );     


BROADCAST( .jeu_broadcast_rob_addr ('d3), .jeu_broadcast_data ('d33));
BROADCAST( .jeu_broadcast_rob_addr ('d4), .jeu_broadcast_data ('d33));
BROADCAST( .jeu_broadcast_rob_addr ('d5), .jeu_broadcast_data ('d55));
BROADCAST( .jeu_broadcast_rob_addr ('d6), .jeu_broadcast_data ('d55));
BROADCAST( .jeu_broadcast_rob_addr ('d7), .jeu_broadcast_data ('d66));
BROADCAST( .jeu_broadcast_rob_addr ('d8), .jeu_broadcast_data ('d66));
BROADCAST( .jeu_broadcast_rob_addr ('d9), .jeu_broadcast_data ('d77));
BROADCAST( .jeu_broadcast_rob_addr ('d10), .jeu_broadcast_data ('d77));


end 
endtask 




initial begin
   RESET(); 
   test2();


   $finish(); 
end 




//====================================
// DUT Instantiation 
//====================================
JUMP_EXECUTION_UNIT
#(.JEUSIZE (JEUSIZE), 
  .ROBSIZE (ROBSIZE)    )
uJUMP_EXECUTION_UNIT
(.*);


endmodule 





