`timescale 1ns / 1ps

module ROB
#(parameter ROBSIZE=8)
(
     input  logic                         clk,
     input  logic                         rstn,
     input  logic                         i_flush,
     // Add a new instruction to the table 
     input  logic                         i_issue_we,
     input  logic [31:0]                  i_issue_inst_pc,
     input  logic                         i_issue_taken,
     input  logic [4:0]                   i_issue_rd,
     input  logic [1:0]                   i_issue_load_store,
     input  logic                         i_issue_rd_inst,
     input  logic [2:0]                   i_issue_cont_tra_inst,
     input  logic [31:0]                  i_issue_pc_p_4,
     output logic [(ROBSIZE-1):0]         o_issue_ptr,   
     // Return the rob values if they are valid 
     input  logic [(ROBSIZE-1):0]         i_rs1_ROB_Addr,
     input  logic [(ROBSIZE-1):0]         i_rs2_ROB_Addr,
     output logic [31:0]                  o_ROB_rs1_Value,
     output logic [31:0]                  o_ROB_rs2_Value,
     output logic                         o_ROB_rs1_Valid,
     output logic                         o_ROB_rs2_Valid,
     // BROADCAST SIGNALS 
     input  logic                         i_broadcast,
     input  logic [(ROBSIZE-1):0]         i_broadcast_rob_addr,
     input  logic [31:0]                  i_broadcast_data,
     // Generate the Commit Output Values   
     output logic                         o_commit_ready,
     output logic [(ROBSIZE-1):0]         o_commit_rob_addr,
     output logic [31:0]                  o_commit_inst_pc,
     output logic [31:0]                  o_commit_value,
     output logic [4:0]                   o_commit_rd,
     output logic                         o_commit_exception,
     output logic [1:0]                   o_commit_load_store,
     output logic                         o_commit_rd_inst,
     output logic [2:0]                   o_commit_cont_tra_inst,
     input  logic                         i_commit, 
     // ROB FULL FLAG 
     output logic                         o_rob_full
    );    
    
    
// ROB TABLE COLUMNS     
logic [(2**ROBSIZE-1):0][4:0]  RD;
logic [(2**ROBSIZE-1):0][31:0] VALUE;    
logic [(2**ROBSIZE-1):0][31:0] PC;    
logic [(2**ROBSIZE-1):0]       BRANCH_TAKEN; 
logic [(2**ROBSIZE-1):0]       DONE;    
logic [(2**ROBSIZE-1):0]       EXCEPTION;  
logic [(2**ROBSIZE-1):0]       VALID;  
logic [(2**ROBSIZE-1):0][1:0]  LOAD_STORE ;
logic [(2**ROBSIZE-1):0]       RD_INST; 
logic [(2**ROBSIZE-1):0][2:0]  CONT_TRA_INST ;



// ROB TABLE POINTERS 
logic [(ROBSIZE-1):0] COMMIT;   
logic [(ROBSIZE-1):0] ISSUE;   
// FIFO Signals 
logic  [(ROBSIZE-1):0]      count, count_next;
logic empty;



//===========================================================
// Add a new instruction to the table 
//===========================================================
always_ff @(posedge clk or negedge rstn)
begin
  //
  if(!rstn || i_flush)
  begin
     //
     for(int i=0;i<2**ROBSIZE;i++)
     begin
            RD[i]              <= '0;
            DONE[i]            <= '0;
            VALUE[i]           <= '0;   
            PC[i]              <= '0;
            BRANCH_TAKEN[i]    <= '0; 
            EXCEPTION[i]       <= '0;  
            VALID[i]           <= '0;
            LOAD_STORE[i]      <= '0;
            RD_INST[i]         <= '0;
            CONT_TRA_INST[i]   <= '0;
     end 
    //
  end else begin
    ////
         // ISSUE INSTRUCTION WRITE 
         if(i_issue_we && (~o_rob_full))
         begin
            RD[ISSUE]              <= i_issue_rd;
            DONE[ISSUE]            <= '0;
            VALUE[ISSUE]           <= '0;   
            PC[ISSUE]              <= i_issue_inst_pc;
            BRANCH_TAKEN[ISSUE]    <= i_issue_taken;  
            EXCEPTION[ISSUE]       <= '0;  
            VALID[ISSUE]           <= 1'b1;
            LOAD_STORE[ISSUE]      <= i_issue_load_store; 
            RD_INST[ISSUE]         <= i_issue_rd_inst; 
            CONT_TRA_INST[ISSUE]   <= i_issue_cont_tra_inst;
            
            //========================
            // The value for JAL and JALR will be written at the ISSUE stage
            // The Conditional Branch Instruction Doesnt Have a value so it will be DONE as soon as it is ussued. 
            //========================
            case(i_issue_cont_tra_inst)
               3'b001: begin DONE[ISSUE]           <= 1'b1; end 
               3'b010: begin DONE[ISSUE]           <= 1'b1; VALUE[ISSUE]          <= i_issue_pc_p_4; end 
               3'b100: begin DONE[ISSUE]           <= 1'b1; VALUE[ISSUE]          <= i_issue_pc_p_4; end 
               default: begin  end 
            endcase 
            
            
             
            
            // Store and LOAD Instructions 
            // Will check LOAD STORE QUEUE for the MEMORY ADDRESS AND DATA.So Load-Store Queue Makes the decision of when the Store instruction is Ready. 
            // For Load Instruction, Control Unit Takes the Data from Load-Store queue to write to the Register File. 
            if(i_issue_load_store != 2'b00)
            begin
               DONE[ISSUE]           <= 1'b1;
            end 
            
            
              
        end
         // BROADCAST WRITE 
         if( i_broadcast && VALID[i_broadcast_rob_addr] && (LOAD_STORE[i_broadcast_rob_addr]==2'b00))
         begin
            DONE[i_broadcast_rob_addr]  <= 1'b1;
            VALUE[i_broadcast_rob_addr] <= i_broadcast_data; 
            
            // FOR CONDITIONAL BRANCH INSTRUCTIONS 
            if(CONT_TRA_INST[i_broadcast_rob_addr][0])
            begin
               // Generate Exception if the Prediction and the Actual are different
               EXCEPTION[i_broadcast_rob_addr] <= BRANCH_TAKEN[i_broadcast_rob_addr] ^ i_broadcast_data[0];
            end 
              
         end 
         
         
         
         
         
         
         // COMMIT UPDATE 
         if( i_commit && VALID[COMMIT])
         begin
           VALID[COMMIT] <= 1'b0;
         end 
    ////
  end     
    
end 




///////////////////////////////
//  COMMIT POINTER GENERATION
///////////////////////////////
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
    COMMIT <= 0;
   end else if(i_flush) begin
    COMMIT <= 0;
   end else if(i_commit && ~empty) begin
    if(COMMIT==(2**ROBSIZE-1)) COMMIT <= 0;
    else                       COMMIT <= COMMIT + 1;
   end 
end 

//================================
// Generate the Commit Output Values   
//================================
assign o_commit_ready           = DONE[COMMIT] & VALID[COMMIT];
assign o_commit_value           = VALUE[COMMIT];
assign o_commit_rd              = RD[COMMIT];
assign o_commit_inst_pc         = PC[COMMIT];
assign o_commit_exception       = EXCEPTION[COMMIT];
assign o_commit_rob_addr        = COMMIT;
assign o_commit_load_store      = LOAD_STORE[COMMIT];
assign o_commit_rd_inst         = RD_INST[COMMIT];
assign o_commit_cont_tra_inst   = CONT_TRA_INST[COMMIT];



///////////////////////////////
//  ISSUE POINTER GENERATION
///////////////////////////////
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
    ISSUE <= 0 ;
   end else if(i_flush) begin
    ISSUE <= 0 ;
   end else if(i_issue_we && ~o_rob_full) begin
    if(ISSUE == (2**ROBSIZE-1)) ISSUE <= '0;
    else                        ISSUE <= ISSUE + 1;
   end 
end 

assign o_issue_ptr = ISSUE; // Pointer to the current ISSUE pointer. 


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
   
   if ( (i_issue_we && i_commit) && o_rob_full)
   begin
     count_next = count -1;
   end else if ( (i_issue_we && i_commit) && empty)
   begin
     count_next = count + 1;
   end else if ( (i_issue_we && i_commit))
   begin
     count_next = count;
   end else if ( i_issue_we && ~o_rob_full) 
   begin
     count_next = count + 1;
   end else if (i_commit && ~empty)
   begin
     count_next = count - 1;
   end 
end 

always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      count <= '0;
   end else if(i_flush) begin
      count <= '0;
   end else begin
      count <= count_next;
   end 
end 


//================================
// ROB FULL FLAG 
//================================
assign o_rob_full = (count == (2**ROBSIZE-1));
assign empty      = (count == 0);


//========================
// Return the rob values if they are valid 
//========================
assign o_ROB_rs1_Value = VALUE[i_rs1_ROB_Addr];
assign o_ROB_rs2_Value = VALUE[i_rs2_ROB_Addr]; 
assign o_ROB_rs1_Valid = DONE[i_rs1_ROB_Addr]; 
assign o_ROB_rs2_Valid = DONE[i_rs2_ROB_Addr];


    
endmodule :ROB

///////////////////////////////////////////////////////////////////
//// ROB TABLE 
///////////////////////////////////////////////////////////////////

module ROB_tb();

localparam ROBSIZE = 8; // Defines the number of entries in the 

// INPUTS 
logic                         clk;
logic                         rstn;
logic                         i_flush;
// Add a new instruction to the table 
logic                         i_issue_we;
logic [31:0]                  i_issue_inst_pc;
logic                         i_issue_taken;
logic [1:0]                   i_issue_load_store;
logic [4:0]                   i_issue_rd;
logic                         i_issue_rd_inst;
logic [2:0]                   i_issue_cont_tra_inst;
logic [(ROBSIZE-1):0]         o_issue_ptr;   
// Return the rob values if they are valid 
logic [(ROBSIZE-1):0]         i_rs1_ROB_Addr;
logic [(ROBSIZE-1):0]         i_rs2_ROB_Addr;
logic [31:0]                  o_ROB_rs1_Value;
logic [31:0]                  o_ROB_rs2_Value;
logic                         o_ROB_rs1_Valid;
logic                         o_ROB_rs2_Valid;
// BROADCAST SIGNALS 
logic                         i_broadcast;
logic [(ROBSIZE-1):0]         i_broadcast_rob_addr;
logic [31:0]                  i_broadcast_data;
// Generate the Commit Output Values   
logic                         o_commit_ready;
logic [(ROBSIZE-1):0]         o_commit_rob_addr;
logic [31:0]                  o_commit_inst_pc;
logic [31:0]                  o_commit_value;
logic [1:0]                   o_commit_load_store;
logic [4:0]                   o_commit_rd;
logic                         o_commit_rd_inst;
logic                         o_commit_exception;
logic [2:0]                   o_commit_cont_tra_inst;
logic                         i_commit; 
// ROB FULL FLAG 
logic                         o_rob_full;

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
  i_issue_we           = '0;
  i_issue_rd           = '0;
  i_rs1_ROB_Addr       = '0;
  i_rs2_ROB_Addr       = '0;
  i_commit             = '0;
  i_flush              = '0; 
  i_broadcast          = '0;   
  i_broadcast_rob_addr = '0; 
  i_broadcast_data     = '0;  
  i_issue_rd_inst      = '0;
  i_issue_taken        = '0;
  i_issue_load_store   = '0;
  //
  rstn = 1'b1;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b0;
  repeat(2) @(posedge clk); #2;
  rstn = 1'b1;
  repeat(2) @(posedge clk); #3;
end 
endtask

//////////////////////////////
// ADD NEW INSTRUCTION (ISSUE) FUNCTION 
//////////////////////////////
task ISSUE(
input logic [4:0] i_rd
);
begin
   @(posedge clk);
   i_issue_we <= 1'b1;
   i_issue_rd <=  i_rd;
   @(posedge clk); 
   i_issue_we <=  1'b0;
end 
endtask

//================================
// BROADCAST TASK 
//================================
task BROADCAST(
  input logic [(ROBSIZE-1):0]     rob_addr,
  input logic [31:0]              value  
);
begin
  @(posedge clk);
  i_broadcast          <= 1'b1;
  i_broadcast_rob_addr <= rob_addr;
  i_broadcast_data     <= value;
  @(posedge clk);
  i_broadcast          <= 1'b0;
end 
endtask

//================================
// COMMIT 
//================================
task COMMIT();
begin
   wait(o_commit_ready)
   @(posedge clk);
   $display("COMMIT READY:%d RD:%d VALUE:%d EXCEPTION: %d ",o_commit_ready, o_commit_rd , o_commit_value,o_commit_exception);
   i_commit = 1'b1;
   @(posedge clk);
   i_commit = 1'b0;  
end 
endtask 

//================================
// FLUSH 
//================================
task FLUSH();
begin
   repeat(30) @(posedge clk);
   @(posedge clk);
     i_flush = 1'b1;
   @(posedge clk);
     i_flush = 1'b0;   
end 
endtask

//////////////////////////////////////
// TEST STIMULUS 
/////////////////////////////////////
initial begin
// INITIALIZE ALL THE INPUTS 
RESET();
ISSUE(.i_rd(5));
ISSUE(.i_rd(6));
ISSUE(.i_rd(7));
ISSUE(.i_rd(8));
ISSUE(.i_rd(9));
//
//BROADCAST( .rob_addr (0), .value (30) );
//BROADCAST( .rob_addr (1), .value (15) );
//BROADCAST( .rob_addr (2), .value (45) );
//
//COMMIT();
//COMMIT();
//COMMIT();
//
FLUSH();


repeat(10) @(posedge clk);
$finish;
end 






// ROB MODULE INSTANTIATION
ROB #(.ROBSIZE (ROBSIZE)) uROB (.*);

endmodule :ROB_tb



