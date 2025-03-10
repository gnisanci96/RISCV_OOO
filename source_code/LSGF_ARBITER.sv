`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// LEAST RECENTLY GRANTED FIRST ARBITER 

module LSGF_ARBITER
#(parameter REQSIZE = 3) 
  (
    input  logic                   clk, 
    input  logic                   rstn, 
    input  logic [(REQSIZE-1):0]   i_req,
    output logic [(REQSIZE-1):0]   o_grant
);
  


  logic [$clog2(REQSIZE-1):0] LRG       [(REQSIZE-1):0];
  logic [$clog2(REQSIZE-1):0] LRG_next  [(REQSIZE-1):0];  
  logic [$clog2(REQSIZE-1):0] granted; 
  logic [(REQSIZE-1):0] grant_next;
  
  
  
//============================  
// ASSIGN grant
//============================  
logic break_flag;   

always_comb
  begin
    break_flag = '0;
    grant_next = '0;
    for(int k =0;k<4;k++)
      begin
        for(int m =0;m<REQSIZE;m++)
       begin
         if((LRG[m]==k) && (i_req[m]==1'b1) && ~break_flag)
           begin
             grant_next[m] = 1'b1; 
             granted       = m;
             break_flag    = 1'b1;
           end 
       end                  
      end 
  end 
  
  
  
always_ff @(posedge clk or negedge rstn)
   begin
      if(!rstn)
        begin
          o_grant   <= '0;
        end else begin
          o_grant   <= grant_next;          
        end 
    end 
  
  
  
  
  
  
  
  
  
//============================  
// UPDATE least Recently Granted Array   
//============================   

 always_comb
   begin
     LRG_next = LRG;
    for(int j=0;j<REQSIZE;j++)
     begin
      if(LRG[j]> LRG[granted])
       begin
         LRG_next[j] = LRG_next[j] - 1; 
       end          
     end
     LRG_next[granted] = (REQSIZE-1);
   end 
  
  
  
  always_ff @(posedge clk or negedge rstn)
    begin
      if(!rstn)
        begin
           for(int i=0;i<REQSIZE;i++)
            begin
              LRG[i]=i;
            end 
        end else begin
            LRG  <= LRG_next; 
        end 
    end 
  
  
  
  
  
  
endmodule

//=============================
//=============================
// LSGF TEST BENCH 
//=============================
//=============================
module LSGF_ARBITER_tb();

parameter REQSIZE = 3;
logic                 clk;
logic                 rstn;
logic [(REQSIZE-1):0] i_req;
logic [(REQSIZE-1):0] o_grant;


//=====================
// CLOCK GENERATION
//=====================
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 

//=====================
// RESET GENERATION
//=====================
task RESET();
begin
   i_req = '0;
   //
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2)@(posedge clk);
   rstn = 1'b1;
end 
endtask 


//=====================
// TEST FLOW 
//=====================
initial begin
 RESET();
 repeat(2)@(posedge clk);
 i_req = 3'b010;
 repeat(2)@(posedge clk);
 i_req = 3'b000;
 repeat(2)@(posedge clk);
 i_req = 3'b011;
 repeat(2)@(posedge clk);
 i_req = 3'b111;
 repeat(5)@(posedge clk);  
 $finish;
end 


LSGF_ARBITER
#(.REQSIZE (REQSIZE))
uLSGF_ARBITER
(.*);

endmodule 


