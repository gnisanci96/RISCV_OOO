`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// PAGE_TABLE MEMORY MODULE 

module PAGE_TABLE
#(parameter ENTRYSIZE = 8,
  parameter VPNSIZE = 23, 
  parameter PPNSIZE   = 11)
(
    input  logic                 clk, 
    input  logic                 rstn, 
    input  logic                 i_cs, 
    input  logic                 i_write_read,
    input  logic [(VPNSIZE-1):0] i_VPN, 
    input  logic [(PPNSIZE-1):0] i_PPN,
    output logic                 o_output_v,
    output logic                 o_page_fault, 
    output logic [(PPNSIZE-1):0] o_PPN
    );
    
    
    
    
logic [(2**ENTRYSIZE-1):0]                T_VALID;      
logic [(2**ENTRYSIZE-1):0][(VPNSIZE-1):0] T_VPN;    
logic [(2**ENTRYSIZE-1):0][(PPNSIZE-1):0] T_PPN;    
logic break_loop;   
    
    
    
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      T_VALID       <= '0;
      o_output_v    <= '0;
      o_PPN         <= '0;
      o_page_fault  <= '0;
   end else begin
      //
      o_output_v = 1'b0;
      o_page_fault = 1'b0;
      if(i_cs)
      begin
         //
         if(i_write_read)
         begin
         break_loop = 1'b0;
            for(int i=0;i<(2**ENTRYSIZE);i++)
            begin
               if(~T_VALID[i] && ~break_loop )
               begin
                  T_VPN[i]   <= i_VPN;
                  T_PPN[i]   <= i_PPN;
                  T_VALID[i] <= 1'b1;
                  break_loop = 1'b1;
               end 
            end 
         end else begin
            o_page_fault = 1'b1;
            for(int i=0;i<(2**ENTRYSIZE);i++)
            begin
               if((T_VPN[i]==i_VPN) && T_VALID[i])
               begin
                  o_PPN        <= T_PPN[i];
                  o_page_fault  = 1'b0;
               end 
            end 
         end 
         o_output_v = 1'b1;
      end
   end 
end 




    
    
    
    
endmodule : PAGE_TABLE 



module PAGE_TABLE_tb();

parameter ENTRYSIZE = 8;
parameter VPNSIZE   = 23; 
parameter PPNSIZE   = 11;
logic                 clk; 
logic                 rstn; 
logic                 i_cs; 
logic                 i_write_read;
logic [(VPNSIZE-1):0] i_VPN;
logic [(PPNSIZE-1):0] i_PPN; 
logic                 o_output_v;
logic                 o_page_fault; 
logic [(PPNSIZE-1):0] o_PPN;


initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 


task RESET();
begin
      i_cs = 1'b0;
      i_write_read = 1'b0;
      i_VPN = '0;
      i_PPN = '0;
      //
      rstn = 1'b1;
   repeat(2) @(posedge clk); 
      rstn = 1'b0;
   repeat(2) @(posedge clk); 
      rstn = 1'b1;
end 
endtask 


task READ(input logic [(VPNSIZE-1):0] VPN);
begin
       i_cs         <= 1'b1;
       i_write_read <= 1'b0;
       i_VPN        <= VPN;
    @(posedge clk);
       i_cs         <= 1'b0;
       i_write_read <= 1'b0;
       i_VPN        <= '0;
    wait(o_output_v == 1'b1);
    $display("THE OUTPUT %02t",$time);
    
end 
endtask :READ 


task WRITE(input logic [(VPNSIZE-1):0] VPN, 
           input logic [(PPNSIZE-1):0] PPN);
begin
       i_cs         <= 1'b1;
       i_write_read <= 1'b1;
       i_VPN        <= VPN;
       i_PPN        <= PPN;
    @(posedge clk);
       i_cs         <= 1'b0;
       i_write_read <= 1'b0;
       i_VPN        <= '0;
       i_PPN        <= '0;
end 
endtask :WRITE 






initial begin
   RESET();
   READ(.VPN('h1234567));
   WRITE(.VPN('h1234567) , .PPN ('h010));
   WRITE(.VPN('h1234568) , .PPN ('h011));
   WRITE(.VPN('h1234569) , .PPN ('h012));
   WRITE(.VPN('h123456A) , .PPN ('h013));
   WRITE(.VPN('h123456B) , .PPN ('h014));
   READ(.VPN('h1234567));
  $finish;
end 


//=========================
// DUT INSTANTIATION 
//=========================
PAGE_TABLE
#(.ENTRYSIZE (ENTRYSIZE),
  .VPNSIZE   (VPNSIZE), 
  .PPNSIZE   (PPNSIZE) )
uPAGE_TABLE
(.*);



endmodule :PAGE_TABLE_tb 


