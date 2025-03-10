`timescale 1ns / 1ps
`define R_TYPE              3'b000
`define I_TYPE              3'b001
`define S_TYPE              3'b010
`define B_TYPE              3'b011
`define U_TYPE              3'b100
`define J_TYPE              3'b101


module IMM_GEN(
 input  logic [31:0]  i_inst, 
 input  logic [2:0]   i_imm_type,
 output logic [31:0]  o_imm
    );
    
    
logic [31:0] i_type;
logic [31:0] s_type;
logic [31:0] b_type;
logic [31:0] u_type;
logic [31:0] j_type;
logic [31:0] csr_type;

assign i_type = { {20{i_inst[31]}}, i_inst[31:20] };
assign s_type = { {20{i_inst[31]}}, i_inst[31:25], i_inst[11:7] };
assign b_type = { {19{i_inst[31]}}, i_inst[31], i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0 };
assign u_type = { i_inst[31:12], 12'h000 };
assign j_type = { {11{i_inst[31]}}, i_inst[31], i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0 };   
  

    always_comb
    begin
       case (i_imm_type)
            3'b000: o_imm = i_type; 
           `I_TYPE: o_imm = i_type;
           `S_TYPE: o_imm = s_type;
           `B_TYPE: o_imm = b_type;
           `U_TYPE: o_imm = u_type;
           `J_TYPE: o_imm = j_type;
            3'b111: o_imm = i_type;
       endcase
    end


endmodule
