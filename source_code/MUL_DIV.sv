`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Multiply-Divide Algorithm with individual signed/unsigned number selection flags. 
// Algorithm Flow: num1/num2 Signed to Unsigned Conversion --> Unsigned Mul/Div --> result unsigned to signed conversion.  


module MUL_DIV
#(parameter SIZE = 4)
(
   input  logic                clk,
   input  logic                rstn,
   input  logic                start,        // 1: Input parameters are ready to be sampled. 
   input  logic                flush,        // 1: Flush the ongoing MUL_DIV execution.  
   input  logic                mul_div,      // 1: Multiplication Operation 0:Division Operation  
   input  logic                num1_signed,  // 1: num1 is a signed number. 0: num1 is an unsigned number. 
   input  logic                num2_signed,  // 1: num2 is a signed number. 0: num2 is an unsigned number. 
   input  logic [(SIZE-1):0]   num1,         //   
   input  logic [(SIZE-1):0]   num2,
   output logic [(2*SIZE-1):0] result,        // Multiplication:  result is product. Division: Higher-half: Remainder Lower-half is quotient.  
   output logic                ready         // Result is ready to be sampled.  
    );
    
    
enum logic {IDLE,EXECUTION} cstate,nstate;        
    
logic [(SIZE-1):0]     A, A_next;
logic [(SIZE-1):0]     A_test;
logic [(2*SIZE):0]     B, B_next;
logic                  ready_next;
logic [(2*SIZE-1):0]   result_next;
logic [$clog2(SIZE):0] cnt, cnt_next;
logic                  result_sign_next,result_sign_reg; 
logic                  mul_div_next,mul_div_reg;
logic [(SIZE-1):0]     uns_num1;
logic [(SIZE-1):0]     uns_num2;   
logic num1_comp;
logic num2_comp;
  

assign num1_comp = (num1_signed & num1[SIZE-1]);  
assign num2_comp = (num2_signed & num2[SIZE-1]);  
  
  


assign uns_num1 = ({SIZE{num1_comp}} ^ num1) + num1_comp;  
assign uns_num2 = ({SIZE{num2_comp}} ^ num2) + num2_comp;  


always_comb begin
    nstate   = cstate;
    A_next   = A;
    B_next   = B;
    cnt_next = cnt;
    ready_next = ready;
    mul_div_next            = mul_div_reg;
case(cstate)
   IDLE:
    begin
      ready_next = '0;
      if(start && (~flush))
      begin
         A_next                  = uns_num2; 
         B_next[(SIZE-1):0]      = uns_num1;
         B_next[(2*SIZE-1):SIZE] = '0;
         cnt_next                = '0;
         mul_div_next            = mul_div;
         result_sign_next        = num1_comp ^ num2_comp;
                                            nstate <= EXECUTION;
      end 
    end 
   EXECUTION:
      begin
      
      if(flush) begin                       nstate <= IDLE;  end 
      
      
      if(mul_div_reg&B[0] || (~mul_div_reg)&&(B[(2*SIZE-1):SIZE] >= A) )
      begin    
         B_next[(2*SIZE):SIZE] = B[(2*SIZE-1):SIZE] + ({SIZE{~mul_div_reg}} ^ A) + {{(SIZE-1){1'b0}},(~mul_div_reg)} ;
      end
      
      if(mul_div_reg)
      begin
         // MULTIPLICATION SECTION            
         B_next = {1'b0,B_next[(2*SIZE):1]};
         result_next = B[(2*SIZE-1):0];
         result_next = ({2*SIZE{result_sign_reg}} ^ result_next) + result_sign_reg;
         //
      end else begin
         // DIVISION SECTION 
         if(B[(2*SIZE-1):SIZE] >= A)
         begin
            result_next[(2*SIZE-1):SIZE] = B_next[(2*SIZE-1):SIZE];
            B_next = {B_next[(2*SIZE-2):0],1'b1};
         end else begin
            result_next[(2*SIZE-1):SIZE] = B[(2*SIZE-1):SIZE];
            B_next = {B[(2*SIZE-2):0],1'b0};
         end  
            result_next[(SIZE-1):0]      = B_next[(SIZE-1):0];
            result_next[(SIZE-1):0]      = ({SIZE{result_sign_reg}} ^ result_next[(SIZE-1):0]) + result_sign_reg;
            result_next[(2*SIZE-1):SIZE] = ({SIZE{result_sign_reg}} ^ result_next[(2*SIZE-1):SIZE]) + result_sign_reg;
         //       
      end 
         cnt_next = cnt + 1;
           
         if(cnt == SIZE) 
         begin 
            ready_next  = 1'b1; 
                                           nstate <= IDLE;    
         end 
         
      end 
endcase 

end 
    
    
    
    
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      cstate      <= IDLE;
      A           <= '0;
      B           <= '0;
      ready       <= '0;
      cnt         <= '0;
      result      <= '0;
      mul_div_reg <= '0;
      result_sign_reg <= '0;
   end else begin
      cstate          <= nstate;
      A               <= A_next;
      B               <= B_next;
      ready           <= ready_next;
      cnt             <= cnt_next;
      result          <= result_next;
      mul_div_reg     <= mul_div_next;
      result_sign_reg <= result_sign_next; 
   end 
end     
    
    
        
endmodule :MUL_DIV




module MUL_DIV_tb();

parameter SIZE = 4;
logic               clk;
logic               rstn;
logic               start;  
logic               mul_div;
logic               flush;
logic               num1_signed;
logic               num2_signed;
logic [(SIZE-1):0]  num1;
logic [(SIZE-1):0]  num2;
logic [(2*SIZE-1):0]    result;
logic               ready;
//
logic [(SIZE-1):0]  t_num1;
logic [(SIZE-1):0]  t_num2;
logic t_num1_signed,t_num2_signed;
logic [(2*SIZE-1):0]   t_result;
int t_mod;
int error = 0;
// Clock Generation
initial begin
   clk = 1'b0;
   forever #10 clk = ~clk;
end 


// Reset Task 
task RESET();
   flush   = '0;
   mul_div = 1;
   error   = 0;
   rstn    = 1'b1;
   repeat(2) @(negedge clk);
   rstn    = 1'b0;   
   repeat(2) @(negedge clk);
   rstn    = 1'b1;   
endtask 



task SEND(   input  logic [(SIZE-1):0]   inum1,
             input  logic [(SIZE-1):0]   inum2,
             input  logic                imul_div,
             input  logic      inum1_signed,
             input  logic      inum2_signed,
             input  logic [(2*SIZE-1):0]   expected);
begin
    @(posedge clk);
       start       <= 1'b1;
       num1        <= inum1;
       num2        <= inum2;
       mul_div     <= imul_div;
       num1_signed <= inum1_signed;
       num2_signed <= inum2_signed;
    @(posedge clk);
       start       <= '0;
       num1        <= '0;
       num2        <= '0;   
    @(posedge clk iff ready);
    if (result != expected)begin error++; $display("ERROR! sign1:%b num1:%b sign2:%b num2:%b EXPECTED:%h != ACTUAL:%h",inum1_signed,inum1,inum2_signed,inum2,expected,result); end
    else $display("ALL GOOD! EXPECTED:%h == ACTUAL:%h",expected,result);   
end 
endtask 



initial begin
   RESET();
   
   
   // UNSIGNED-MULTIPLICATION 
      SEND( .inum1        ('b1011),
            .inum2        ('b1101),
            .imul_div     (1'b1),
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b10001111));
                      
    SEND(   .inum1        ('b0110),
            .inum2        ('b0010),
            .imul_div     (1'b1),
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b00001100));
            
      SEND( .inum1        ('b1111),
            .inum2        ('b1111),
            .imul_div     (1'b1),
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b11100001));  
                      
   // UNSIGNED-UNSIGNED DIVISION 
      SEND( .inum1        ('b0110),
            .inum2        ('b0010),
            .imul_div     (1'b0),
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b00000011));
            
      SEND( .inum1        ('b1111),
            .inum2        ('b0110),
            .imul_div     (1'b0),            
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b00110010));            

            
      SEND( .inum1        ('b1000),
            .inum2        ('b0111),
            .imul_div     (1'b0),            
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b00010001));              
            
       SEND( .inum1       ('b0001),
            .inum2        ('b1111),
            .imul_div     (1'b0),            
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b00010000));              
                       
       SEND( .inum1       ('b0011),
            .inum2        ('b0111),
            .imul_div     (1'b0),            
            .inum1_signed ('b0),
            .inum2_signed ('b0),
            .expected     ('b00110000)); 
   // SIGNED-SIGNED MULTIPLICATION 
       SEND(.inum1        ('b0011),
            .inum2        ('b0111),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b00010101)); 
            
       SEND(.inum1        ('b1000),
            .inum2        ('b1000),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b01000000));             

       SEND(.inum1        ('b1000),
            .inum2        ('b1011),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b00101000));             
            
       SEND(.inum1        ('b1000),
            .inum2        ('b1011),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b00101000));              
            
       SEND(.inum1        ('b1000),
            .inum2        ('b0101),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b11011000));                
            
            
       SEND(.inum1        ('b1011),
            .inum2        ('b0101),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b11100111));                
            
      // SIGNED-UNSIGNED MULTIPLICATION  
               
        SEND(.inum1       ('b1011),
            .inum2        ('b1111),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b0),
            .expected     ('b1011_0101));              
         
         
        SEND(.inum1       ('b1000),
            .inum2        ('b1111),
            .imul_div     (1'b1),            
            .inum1_signed ('b1),
            .inum2_signed ('b0),
            .expected     ('b1000_1000));          
         
     // SIGNED-SIGNED DIVISION  
     
         SEND(.inum1      ('b1000),
            .inum2        ('b1100),
            .imul_div     (1'b0),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b0000_0010));          
         
         
         SEND(.inum1      ('b1000),
            .inum2        ('b0100),
            .imul_div     (1'b0),            
            .inum1_signed ('b1),
            .inum2_signed ('b1),
            .expected     ('b0000_1110));    
            
            
            
            
   $finish;
end 



mul_div 
#(.SIZE (SIZE))
DUT
(.*);


endmodule:MUL_DIV_tb