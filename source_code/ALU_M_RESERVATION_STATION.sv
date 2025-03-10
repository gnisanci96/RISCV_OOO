`timescale 1ns / 1ps

////========
// RESERVATION STATION TABLE 
////========
// | ROB_ADDR | ALU_OPCODE | RS1_VALUE | RS2_VALUE | RS1_ROB_ADDR | RS2_ROB_ADDR | RS1_VALUE_VALID | RS2_VALUE_VALID |ADDR_CAL|READY  | VALID


module ALU_M_RESERVATION_STATION
#(parameter RSSIZE    = 16,
  parameter ROBSIZE   = 8)
(
  input logic                      clk,
  input logic                      rstn,
  // FLUSH 
  input logic                      i_flush,
  // Add new instruction to the Reservation Station 
  input logic                      i_issue_we,
  input logic [(ROBSIZE-1):0]      i_rob_addr,
  input logic [2:0]                i_alu_opcode,
  input logic [31:0]               i_rs1_value,
  input logic [31:0]               i_rs2_value,
  input logic                      i_rs1_valid, 
  input logic                      i_rs2_valid, 
  input logic [(ROBSIZE-1):0]      i_rs1_rob_addr,
  input logic [(ROBSIZE-1):0]      i_rs2_rob_addr,
  // Catch Broadcast-Compare ROB- Update Values      
  input logic                      i_broadcast,
  input logic [(ROBSIZE-1):0]      i_broadcast_rob_addr,
  input logic [31:0]               i_broadcast_data,
  // Dispatch Signals 
  input  logic                     i_alu_busy,
  output logic [2:0]               o_alu_opcode,
  output logic [31:0]              o_rs1_value,
  output logic [31:0]              o_rs2_value,
  output logic [(ROBSIZE-1):0]     o_rat_rs1_rob_addr,    
  output logic                     o_alu_ex_en,
  // Reservation Station FULL Flag 
  output logic                     o_rs_full 
  );
 
 
// ================================================    
// RESERVATION STATION TABLE COLUMN 
// ================================================  
logic [(ROBSIZE-1):0]     ROB_ADDR        [(RSSIZE-1):0]; // The ROB address the instruction is stored  
logic [2:0]               ALU_OPCODE      [(RSSIZE-1):0]; // ALU Instruction Selection bits    
logic [31:0]              RS1_VALUE       [(RSSIZE-1):0]; // 32-bit RS1 Value 
logic [31:0]              RS2_VALUE       [(RSSIZE-1):0]; // 32-bit RS2 value 
logic [(ROBSIZE-1):0]     RS1_ROB_ADDR    [(RSSIZE-1):0]; // The ROB address responsible to generate the rs1 value  
logic [(ROBSIZE-1):0]     RS2_ROB_ADDR    [(RSSIZE-1):0]; // The ROB address responsible to generate the rs2 value    
logic [(RSSIZE-1):0]      RS1_VALUE_VALID;                // RS1 value is ready and valid 
logic [(RSSIZE-1):0]      RS2_VALUE_VALID;                // RS2 value is ready and valid 
logic [(RSSIZE-1):0]      READY;                          // INSTRUCTION is ready to execute 
logic [(RSSIZE-1):0]      VALID;                          // COLUMN VALID FLAG 



// ================================================    
// LOCAL SIGNALS 
// ================================================  
logic [($clog2(RSSIZE)-1):0] issue_addr;
logic [($clog2(RSSIZE)-1):0] dispatch_addr;
logic                        dispatch_flag;  
// ================================================    
// Add new instruction to the Reservation Station --- ISSUE      
// ================================================     
 
/////////////////
// Choose available Row to Write the new instruction 
// Write it to the first available row -- Search low Index --> High Index 
//////////////// 
logic break_flag1;

always_comb 
begin
   break_flag1 = '0;
   for(int i=0 ; i<RSSIZE ;i++)
   begin
      if((break_flag1==0) && (VALID[i]==0))
      begin
         issue_addr  = i;
         break_flag1 = 1'b1;
         //$display("[RS] THE ISSUE ADDR: %d",issue_addr);
      end 
   end
end 

 
 
 
 
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
         RS1_VALUE_VALID <= '0;    
         RS2_VALUE_VALID <= '0;    
         READY           <= '0;   
         VALID           <= '0;       
      for(int i=0;i<RSSIZE;i++)
      begin
         ROB_ADDR[i]        <= '0;       
         ALU_OPCODE[i]      <= '0;         
         RS1_VALUE[i]       <= '0;          
         RS2_VALUE[i]       <= '0;          
         RS1_ROB_ADDR[i]    <= '0;       
         RS2_ROB_ADDR[i]    <= '0;   
      end 
   end else if(i_flush)
   begin
         RS1_VALUE_VALID <= '0;    
         RS2_VALUE_VALID <= '0;    
         READY           <= '0;   
         VALID           <= '0; 
   end else begin
    
     if(i_issue_we )
     begin
         ROB_ADDR[issue_addr]        <= i_rob_addr;       
         ALU_OPCODE[issue_addr]      <= i_alu_opcode;         
         RS1_VALUE[issue_addr]       <= i_rs1_value;          
         RS2_VALUE[issue_addr]       <= i_rs2_value;  
         RS1_VALUE_VALID[issue_addr] <= i_rs1_valid;    
         RS2_VALUE_VALID[issue_addr] <= i_rs2_valid;         
         RS1_ROB_ADDR[issue_addr]    <= i_rs1_rob_addr;       
         RS2_ROB_ADDR[issue_addr]    <= i_rs2_rob_addr; 
         VALID[issue_addr]           <= 1'b1; 
        // IF THE VALUE NEEDED IS BROADCASTED AT THE SAME CYCLE AS THE INSTRUCTION IS WRITTEN TO THE RS. 
        if(i_broadcast)
        begin
           if(i_broadcast_rob_addr == i_rs1_rob_addr)
           begin
              RS1_VALUE[issue_addr]       <= i_broadcast_data;
              RS1_VALUE_VALID[issue_addr] <= 1'b1;
           end 
           //
           if(i_broadcast_rob_addr == i_rs2_rob_addr)
           begin
              RS2_VALUE[issue_addr]       <= i_broadcast_data;
              RS2_VALUE_VALID[issue_addr] <= 1'b1;
           end 
        end 
         
     end 
     ////  
     if(dispatch_flag)
     begin
         ROB_ADDR[dispatch_addr]        <= '0;      
         ALU_OPCODE[dispatch_addr]      <= '0;        
         RS1_VALUE[dispatch_addr]       <= '0;       
         RS2_VALUE[dispatch_addr]       <= '0; 
         RS1_VALUE_VALID[dispatch_addr] <= '0;   
         RS2_VALUE_VALID[dispatch_addr] <= '0;        
         RS1_ROB_ADDR[dispatch_addr]    <= '0;      
         RS2_ROB_ADDR[dispatch_addr]    <= '0;
         VALID[dispatch_addr]           <= '0;  
     end 
     //================================= 
     // BROADCAST UPDATE 
     //=================================
     // When there is a broadcast signal: 
     //  1- Read all the VALID rows
     //  2- Check if the 32-bit Source Value is Valid 
     //  3- If the Source value is not valid (so an entry in ROB will produce the value) 
     //  4- Check the ROB value and if it matches the Broadcasted ROB addr, update the 32-bit Source value
     //  5- Now the Source value is valid.  
     if(i_broadcast)
     begin
      
      for(int i1= 0;i1<RSSIZE;i1++)
      begin
         //
         if( (VALID[i1] == 1) && (RS1_VALUE_VALID[i1] == 0) && (RS1_ROB_ADDR[i1]==i_broadcast_rob_addr))
         begin
             RS1_VALUE[i1]       <= i_broadcast_data;   
             RS1_VALUE_VALID[i1] <= 1'b1;
         end 
         //
         if( (VALID[i1] == 1) && (RS2_VALUE_VALID[i1] == 0) && (RS2_ROB_ADDR[i1] == i_broadcast_rob_addr))
         begin
             RS2_VALUE[i1]       <= i_broadcast_data;   
             RS2_VALUE_VALID[i1] <= 1'b1;
         end 
         //         
      end 
      
     end 
     
   end 
end  
 
// ================================================    
// Reservation ROW READY signal generation 
// ================================================     
always_comb 
begin
   for(int j=0;j<RSSIZE;j++)
   begin
      READY[j] = RS1_VALUE_VALID[j] & RS2_VALUE_VALID[j] & VALID[j]; 
   end 
end  
    
   

// ================================================    
// Decide Which one to Dispatch 
// IDEA: Find the first one that is ready on the table: Search lowIndex to HighIndex 
// ================================================      
logic break_flag2;

always_comb 
begin
  break_flag2 = 1'b0;
  for(int k =0;k<RSSIZE;k++)
  begin
     if((break_flag2==1'b0) && (READY[k]==1'b1))
     begin
        dispatch_addr = k;
        break_flag2   = 1'b1;
        //$display("[RS] DISPATCH ADDR: %d ",dispatch_addr );
     end  
  end
end 
  
// There is at least one ready-to-execute Operation  & ALU is not busy --> Ready to dispatch   
assign dispatch_flag = (|READY) & (~i_alu_busy);
  

always_comb
begin
       o_alu_ex_en = dispatch_flag;
       if(dispatch_flag)
       begin
         o_alu_opcode         = ALU_OPCODE[dispatch_addr];
         o_rs1_value          = RS1_VALUE[dispatch_addr];
         o_rs2_value          = RS2_VALUE[dispatch_addr];
         o_rat_rs1_rob_addr   = ROB_ADDR[dispatch_addr];
       end  
end   
  
// ================================================    
// Reservation Station FULL Flag
// ================================================    
assign o_rs_full = &VALID;  
  
    
endmodule :ALU_M_RESERVATION_STATION


// ================================================    
// ================================================    
// RESERVATION STATION TEST BENCH 
// ================================================    
// ================================================    
module ALU_M_RESERVATION_STATION_tb();

parameter RSSIZE  = 16;
parameter ROBSIZE = 8;

logic                     clk;
logic                     rstn;
logic                     i_flush;
// Add new instruction to the Reservation Station 
logic                     i_issue_we;
logic [(ROBSIZE-1):0]     i_rob_addr;
logic [3:0]               i_alu_opcode;
logic [31:0]              i_rs1_value;
logic [31:0]              i_rs2_value;
logic                     i_rs1_valid; 
logic                     i_rs2_valid; 
logic [(ROBSIZE-1):0]     i_rs1_rob_addr;
logic [(ROBSIZE-1):0]     i_rs2_rob_addr;
logic                     i_addr_cal;
// Catch Broadcast-Compare ROB- Update Values      
logic                     i_broadcast;
logic [(ROBSIZE-1):0]     i_broadcast_rob_addr;
logic [31:0]              i_broadcast_data;
// Dispatch Signals 
logic                     i_alu_busy;
logic [3:0]               o_alu_opcode;
logic [31:0]              o_rs1_value;
logic [31:0]              o_rs2_value;
logic [(ROBSIZE-1):0]     o_rat_rs1_rob_addr;    
logic                     o_alu_ex_en;
logic                     o_addr_cal;
// Reservation Station FULL Flag 
logic                     o_rs_full; 


//===========
// Clock Generation
//===========
initial begin
 clk = 1'b0;
 forever #10 clk = ~clk;
end 


//===========
// RESET TASK 
//===========
task RESET();
begin
  i_issue_we           = '0;
  i_rob_addr           = '0;
  i_alu_opcode         = '0;
  i_rs1_value          = '0;
  i_rs2_value          = '0;
  i_rs1_valid          = '0;
  i_rs2_valid          = '0; 
  i_rs1_rob_addr       = '0;
  i_rs2_rob_addr       = '0;
  i_broadcast          = '0;
  i_broadcast_rob_addr = '0;
  i_broadcast_data     = '0;
  i_alu_busy           = '0;  
  i_addr_cal           = '0; 
  i_flush              = '0;          
  //
  rstn = 1'b1;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b0;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b1;
  repeat(2) @(posedge clk); #3;
end 
endtask

//================================
// WRITE RESERVATION STATION TASK 
//================================
task WRITE(
   input logic [(ROBSIZE-1):0]     rob_addr,
   input logic [3:0]               alu_opcode,
   input logic [31:0]              rs1_value,
   input logic [31:0]              rs2_value,
   input logic                     rs1_valid, 
   input logic                     rs2_valid, 
   input logic [(ROBSIZE-1):0]     rs1_rob_addr,
   input logic [(ROBSIZE-1):0]     rs2_rob_addr,
   input logic                     addr_cal   
);
begin
   @(posedge clk);
   i_issue_we     <= 1'b1 ;
   i_rob_addr     <= rob_addr;
   i_alu_opcode   <= alu_opcode;
   i_rs1_value    <= rs1_value;
   i_rs2_value    <= rs2_value;
   i_rs1_valid    <= rs1_valid; 
   i_rs2_valid    <= rs2_valid; 
   i_rs1_rob_addr <= rs1_rob_addr;
   i_rs2_rob_addr <= rs2_rob_addr;
   i_addr_cal     <= addr_cal;
   @(posedge clk);
   i_issue_we     <= 1'b0;
   i_rob_addr     <= '0;
   i_alu_opcode   <= '0;
   i_rs1_value    <= '0;
   i_rs2_value    <= '0;
   i_rs1_valid    <= '0; 
   i_rs2_valid    <= '0; 
   i_rs1_rob_addr <= '0;
   i_rs2_rob_addr <= '0;
   i_addr_cal     <= '0;
end 
endtask

//================================
// BROADCAST TASK 
//================================
task BROADCAST(
  input logic [(ROBSIZE-1):0]     rob_addr,
  input logic [31:0]              data  
);
begin
  @(posedge clk);
  i_broadcast          <= 1'b1;
  i_broadcast_rob_addr <= rob_addr;
  i_broadcast_data     <= data;
  @(posedge clk);
  i_broadcast          <= 1'b0;
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





//==============
// MAIN STIMULUS 
//==============
initial begin
RESET();
//
WRITE(.rob_addr  (1), .alu_opcode (2), .rs1_value    (3) , .rs2_value    (4), 
      .rs1_valid (0), .rs2_valid  (0), .rs1_rob_addr (5),  .rs2_rob_addr (6), .addr_cal (1'b1));
WRITE(.rob_addr  (1), .alu_opcode (2), .rs1_value    (3) , .rs2_value    (4), 
      .rs1_valid (0), .rs2_valid  (0), .rs1_rob_addr (5),  .rs2_rob_addr (6), .addr_cal (1'b1));
WRITE(.rob_addr  (1), .alu_opcode (2), .rs1_value    (3) , .rs2_value    (4), 
      .rs1_valid (0), .rs2_valid  (0), .rs1_rob_addr (5),  .rs2_rob_addr (6), .addr_cal (1'b1));
WRITE(.rob_addr  (1), .alu_opcode (2), .rs1_value    (3) , .rs2_value    (4), 
      .rs1_valid (0), .rs2_valid  (0), .rs1_rob_addr (5),  .rs2_rob_addr (6), .addr_cal (1'b1));
      
      
//
BROADCAST( .rob_addr (5), .data (30) );
BROADCAST( .rob_addr (6), .data (15) );
//
FLUSH();




repeat(10) @(posedge clk);
$finish;
end 


// ================================================    
// RESERVATION STATION Instantiation 
// ================================================    
ALU_M_RESERVATION_STATION
#(.RSSIZE (RSSIZE),.ROBSIZE   (ROBSIZE))uALU_M_RESERVATION_STATION (.*);


endmodule :ALU_M_RESERVATION_STATION_tb


