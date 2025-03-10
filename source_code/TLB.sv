`timescale 1ns / 1ps
// Direct Map TLB CACHE 

module TLB
#(parameter INDEXSIZE    = 6,
  parameter VPNSIZE      = 16,
  parameter PPNSIZE      = 12
  )
(
    input  logic                            clk,                // Clock 
    input  logic                            rstn,               // Asynchronous Reset 
    // Control Commands  
    input  logic                            i_tlb_cs,           // Chip Select Signal 
    input  logic                            i_tlb_write_read,   // 1: WRITE  0: READ TLB    
    // 
    input  logic [(VPNSIZE-1):0]            i_tlb_vpn,          //  Virtual  Page Number Input  
    input  logic [(PPNSIZE-1):0]            i_tlb_ppn,          //  Physical Page Number Input                        
    
    output logic [(PPNSIZE-1):0]            o_tlb_ppn,          // Pyhsical Page Number Output                      
    output logic                            o_tlb_hit,          // 1: TLB hit 0:TLB miss
    output logic                            o_tlb_output_valid   
    );
    
    
logic [(2**INDEXSIZE-1):0][(VPNSIZE-INDEXSIZE-1):0]  T_VPN;    
logic [(2**INDEXSIZE-1):0][(PPNSIZE-1):0]            T_PPN;    
logic [(2**INDEXSIZE-1):0]                           T_VALID;    
logic                                                loop_break;    
     
logic [(INDEXSIZE-1):0]          victim_entry; 
logic [(VPNSIZE-INDEXSIZE-1):0]  tag;  
logic [(INDEXSIZE-1):0]          index;
//  
// Bit-Pseudo LRU Signals     
logic [(2**INDEXSIZE-1):0]       plru_bits;    
//   
    

//=================================
//   READ/WRITE CIRCUIT 
//=================================
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      T_VALID    <= '0;
      T_VPN      <= '0;
      T_PPN      <= '0;
      plru_bits  <= '0;
      o_tlb_ppn  <= '0;
      o_tlb_hit  <= '0;
      o_tlb_output_valid <= '0;
      victim_entry <= '0;
      loop_break   <= '0;
   end else begin
         o_tlb_output_valid = 1'b0;
         o_tlb_hit  = 1'b0;
      if(i_tlb_cs) begin
         o_tlb_output_valid = 1'b1;
         // 1- DECODE THE VPN 
         tag = i_tlb_vpn[(VPNSIZE-1):INDEXSIZE];
         index = i_tlb_vpn[(INDEXSIZE-1):0];
         // 2- CHECK FOR HIT 
         //$display("[TLB] INDEX: %d T_VPN[index]: %h  i_tlb_vpn:%h tag:%h",index,T_VPN[index],i_tlb_vpn,tag);
         if((T_VPN[index] == tag) && T_VALID[index])
         begin
            o_tlb_hit  = 1'b1;
         end      
         //$display("TLB HIT:%b",o_tlb_hit);
      
         //3- If it is a HIT OR MISS  
         if(o_tlb_hit) begin
            // TLB HIT READ/WRITE
            // 4- UPDATE THE PLRU BITS 
            plru_bits[index] = 1'b1;
               if(&plru_bits) begin 
                  plru_bits = '0;   
                  plru_bits[index] = 1'b1;
               end //if(&plru_bits) 
           // 5- READ/WRITE 
            if(i_tlb_write_read) begin
               // 6- HIT-WRITE
               T_PPN[index] <= i_tlb_ppn;  
            end  else begin
              // 7-HIT-READ
              o_tlb_ppn <= T_PPN[index];
            end // else(i_tlb_write_read)
            //
         end else begin
            // 8-TLB MISS 
            //$display("TLB MISS with tag:%h i_tlb_ppn:%h",tag,i_tlb_ppn);
            if(i_tlb_write_read) begin
               loop_break = 1'b0;
               // 9- Choose the First Invalid as the Victim Line if there is any invalid. 
               // 10- UPDATE THE TLB TABLE 
               T_VPN[index]   <= tag; 
               T_PPN[index]   <= i_tlb_ppn;  
               T_VALID[index] <= 1'b1;
               // 11- UPDATE PLRU BITS 
               plru_bits[index] <= 1'b1;
               if(&plru_bits) begin 
                  plru_bits = '0;   
                  plru_bits[index] <= 1'b1;
               end //if(&plru_bits) 
            end
         end // else (o_tlb_hit)
      end // if(i_tlb_cs) 
      //
   end 
end 








    
endmodule :TLB 


//==============================================
// TLB Test Bench 
//==============================================
module TBL_tb();

parameter INDEXSIZE    = 6;
parameter VPNSIZE      = 16;
parameter PPNSIZE      = 12;

logic                            clk;                // Clock 
logic                            rstn;               // Asynchronous Reset 
// Control Commands  
logic                            i_tlb_cs;           // Chip Select Signal 
logic                            i_tlb_write_read;   // 1: WRITE  0: READ TLB    
// 
logic [(VPNSIZE-1):0]            i_tlb_vpn;          //  Virtual  Page Number Input  
logic [(PPNSIZE-1):0]            i_tlb_ppn;          //  Physical Page Number Input                        
//   
logic [(PPNSIZE-1):0]            o_tlb_ppn;          // Pyhsical Page Number Output                      
logic                            o_tlb_hit;          // 1: TLB hit 0:TLB miss
logic                            o_tlb_output_valid;   
//
int error;
logic [(INDEXSIZE-1):0]     test_index;

// Clock Generation 
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;  
end 





// RESET TASK 
task RESET();
begin
   error            = '0;
   i_tlb_cs         = '0; // Chip Select Signal 
   i_tlb_write_read = '0; // 1: WRITE  0: READ TLB    
   i_tlb_vpn        = '0; //  Virtual  Page Number Input  
   i_tlb_ppn        = '0; //  Physical Page Number Input    
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2) @(posedge clk);
   rstn = 1'b1;
end
endtask 


task READ( input logic [(VPNSIZE-1):0]            vpn,
           input logic                            ex_tlb_hit,
           input logic [(PPNSIZE-1):0]            ex_tlb_ppn );
begin
    @(posedge clk);
      i_tlb_cs         <= 1'b1; // Chip Select Signal 
      i_tlb_write_read <= 1'b0; // 1: WRITE  0: READ TLB    
      i_tlb_vpn        <= vpn;  //  Virtual  Page Number Input  
      i_tlb_ppn        <= '0;   //  Physical Page Number Input   
    @(posedge clk);
      i_tlb_cs         <= 1'b0; // Chip Select Signal 
      i_tlb_write_read <= 1'b0; // 1: WRITE  0: READ TLB    
      i_tlb_vpn        <= '0;   //  Virtual  Page Number Input  
      i_tlb_ppn        <= '0;   //  Physical Page Number Input   
       // SELF-CHECK 
       #2;
       if(o_tlb_hit != ex_tlb_hit)    begin $display("HIT MISS-MATCH! %0t",$time); error++;  end
       else                           begin $display("HIT MATCH %0t",$time);                 end 
       //
       if(ex_tlb_hit) begin
          if(o_tlb_ppn != ex_tlb_ppn) begin $display("DATA MISS-MATCH!"); error++;           end 
       end   
end 
endtask 


//==================================
// WRITE TASK 
//==================================
task WRITE( input logic [(VPNSIZE-1):0] vpn, 
            input logic [(PPNSIZE-1):0] ppn );
begin
    @(posedge clk);
      i_tlb_cs         <= 1'b1; // Chip Select Signal 
      i_tlb_write_read <= 1'b1; // 1: WRITE  0: READ TLB    
      i_tlb_vpn        <= vpn;  //  Virtual  Page Number Input  
      i_tlb_ppn        <= ppn;  //  Physical Page Number Input   
    @(posedge clk);
      i_tlb_cs         <= 1'b0; // Chip Select Signal 
      i_tlb_write_read <= 1'b0; // 1: WRITE  0: READ TLB    
      i_tlb_vpn        <= '0;   //  Virtual  Page Number Input  
      i_tlb_ppn        <= '0;   //  Physical Page Number Input   
end 
endtask 

//=================
// STIMULUS 
//=================
initial begin
   //test1();
   RESET();  
   WRITE( .vpn ('hBABE), .ppn ('h123) );  
   WRITE( .vpn ('hDEAD), .ppn ('hAAC) ); 
   WRITE( .vpn ('hCAFE), .ppn ('h112) ); 
   //
   repeat(5) @(posedge clk);
   READ( .vpn ('hBABE), .ex_tlb_hit (1'b1), .ex_tlb_ppn ('h123) );             
   READ( .vpn ('hDEAD), .ex_tlb_hit (1'b1), .ex_tlb_ppn ('hAAC) );             
   READ( .vpn ('hCAFE), .ex_tlb_hit (1'b1), .ex_tlb_ppn ('h112) );  
   //           
   $finish; 
end 

// =========================
// DUT-INSTANTIATION 
// =========================
TLB
#( .INDEXSIZE    (INDEXSIZE),
   .VPNSIZE      (VPNSIZE),
   .PPNSIZE      (PPNSIZE)
  )
uTLB
(.*);


endmodule :TBL_tb