`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// INSTRUCTION FETCH UNIT  
//////////////////////////////////////////////////////////////////////////////////
//================================
// L2 COMMAND DEFINITIONS 
//================================
`define  L2_CACHE_HIT_CHECK_CMD  3'b000
`define  L2_LOAD_CMD             3'b001
`define  L2_STORE_CMD            3'b010
`define  L2_BURST_READ_CMD       3'b011
`define  L2_BURST_WRITE_CMD      3'b100
`define  L2_UPDATE_INCLUSION_CMD 3'b101
 
module INSTRUCTION_FETCH_UNIT2
#(parameter PC_BOOT         = 32'h0,
  parameter ROBSIZE         = 8,
  parameter VASIZE          = 32,
  parameter PASIZE          = 12,
  parameter PAGEOFFSETSIZE  = 6,
  parameter L1OFFSETSIZE    = 4,
  parameter L1INDEXSIZE     = 2,
  parameter L1WAYSIZE       = 2,
  parameter L2INDEXSIZE     = 2,
  parameter L2WAYSIZE       = 4,
  parameter TLBINDEXSIZE    = 6,
  parameter PTENTRYSIZE     = 8)
(
    input logic                 clk, 
    input logic                 rstn, 
    //
    input logic                 i_start, 
    // FLUSH SIGNALS 
    input logic                 i_flush,       
    input logic [31:0]          i_flush_pc,
    
    
    // Instruction Queue Signals 
    input  logic                i_iq_full, 
    output logic                o_iq_wrt_en, 
    output logic [31:0]         o_iq_inst,
    output logic [31:0]         o_iq_pc,
    output logic                o_iq_taken_pre,
    output logic [31:0]         o_iq_target_pre,
    // Branch Target Buffer Signals 
    input  logic                i_btb_write_en,
    input  logic [31:0]         i_btb_write_pc, 
    input  logic [31:0]         i_btb_write_target,
    input  logic                i_btb_write_is_ret,    // It is RETURN instruction so RAS table will predict the target.    
    // Branch History Table Signals 
    input logic                 i_bht_write_en, 
    input logic [31:0]          i_bht_write_pc, 
    input logic                 i_bht_write_taken   
    );
    

logic [31:0] pc_next,pc_reg,pc_reg2_next,pc_reg2,pc_reg3_next,pc_reg3;


//==========
// SIGNAL-DEFINTION START 
//==========      
localparam L1TAGSIZE        = (PASIZE-L1OFFSETSIZE-L1INDEXSIZE);
localparam L2OFFSETSIZE     =  L1OFFSETSIZE;
localparam L2TAGSIZE        = (PASIZE-L2OFFSETSIZE-L2INDEXSIZE); 
localparam L2ADDRSIZE       =  PASIZE;
localparam VAPAGENUMBERSIZE = (VASIZE - PAGEOFFSETSIZE);
localparam TLBTAGSIZE       = (VAPAGENUMBERSIZE - TLBINDEXSIZE);
localparam TLBDATASIZE      = (PASIZE - PAGEOFFSETSIZE);
localparam VPNSIZE          = (VASIZE - PAGEOFFSETSIZE);
localparam PPNSIZE          = (PASIZE - PAGEOFFSETSIZE);
//
/*
localparam BHTWAYSIZE    = 4;             
localparam BHTOFFSETBIT  = 10;
localparam BHTINDEXBIT   = 8; 
localparam BHTTAGBIT     = 20;
//
localparam BTBPCSIZE     = 32;
localparam BTBWAYSIZE    = 4;            
localparam BTBOFFSETBIT  = 10;
localparam BTBINDEXBIT   = 8; 
localparam BTBTAGBIT     = 20;
*/
localparam BHTWAYSIZE    = 3;             
localparam BHTOFFSETBIT  = 3;
localparam BHTINDEXBIT   = 3; 
localparam BHTTAGBIT     = 10;
//
localparam BTBPCSIZE     = 32;
localparam BTBWAYSIZE    = 2;            
localparam BTBOFFSETBIT  = 3;
localparam BTBINDEXBIT   = 3; 
localparam BTBTAGBIT     = 10;

//
localparam RASDATAWIDTH = 40;
localparam RASFIFODEPTH = 4;
//======================= 
// BHT READ SIGNALS
//=======================
logic              bht_read_en_i;
logic [31:0]       bht_read_pc_i; 
logic              bht_taken_pre_o;
logic              bht_hit_o;
//======================= 
// BHT WRITE SIGNALS 
//=======================
logic              bht_write_en_i; 
logic [31:0]       bht_write_pc_i; 
logic              bht_write_taken_i; 
//======================================= 
//------------- BTB READ SIGNALS -------- 
//=======================================  
logic                   btb_read_en_i; 
logic [(BTBPCSIZE-1):0] btb_read_pc_i;  
logic                   btb_read_hit_o; 
logic [(BTBPCSIZE-1):0] btb_target_pre_o;  
logic                   btb_is_ret_o;       
//======================================= 
//------------- WRITE SIGNALS ------------  
//=======================================
logic                   btb_write_en_i; 
logic [(BTBPCSIZE-1):0] btb_write_pc_i;  
logic [(BTBPCSIZE-1):0] btb_write_target_i; 
logic                   btb_write_is_ret_i; 




//======================================= 
//-------------RAS WRITE SIGNALS --------  
//=======================================
logic                      ras_write_en_i;
logic [(RASDATAWIDTH-1):0] ras_write_pc_i;
//                         
logic                      ras_read_en_i;
logic [(RASDATAWIDTH-1):0] ras_read_pc_o;   
//                         
logic                      ras_full_o;
logic                      ras_empty_o;



//======================
// L1 Cache Signals 
//====================== 
// Control Signals  
logic                                       l1_cs_i;
logic  [2:0]                                l1_cmd_i; 
logic                                       l1_cmd_done_o;
logic  [(PASIZE-1):0]                       l1_addr_i;
logic  [(L1WAYSIZE-1):0]                    l1_line_num_i;  // line num
logic  [2:0]                                l1_funct3_i;    // Size (byte;half-byte;word) 
// CMD0: SET READ
logic                                       l1_rd_set_i;
logic [(PASIZE-1):0]                        l1_rd_set_addr_i;
logic [(2**L1WAYSIZE-1):0][(L1TAGSIZE-1):0] l1_rd_set_tags_o; 
logic [(2**L1WAYSIZE-1):0]                  l1_rd_set_valid_bits_o; 
logic [(2**L1WAYSIZE-1):0]                  l1_rd_set_dirty_bits_o;
logic [($clog2(L1WAYSIZE)):0]               l1_plru_line_num_o;
// CMD1: Line SIGLE-DATA Read   
logic [31:0]                                l1_inst_o,l1_inst_o_reg_next, l1_inst_o_reg;        // Data  
logic                                       l1_rd_valid_o;            // Read Data;tags;valid_bits VALID.        
logic                                       l1_wrt_valid_i;           // Burst Read Value Valid.
logic                                       l1_burst_done_i;          //   
// CMD2: Line SINGLE-DATA Write  
logic  [31:0]                               l1_wrt_line_data_i;       // Data  
// CMD3: Line Burst Read 
// CMD4: Line Burst Write   
logic                                       l1_burst_write_ready_o;
logic                                       l1_busy_o;

logic                                       l1_hit_or_victim_line_v_next, l1_hit_or_victim_line_v_reg;
logic                                       l1_hit_or_victim_line_d_next, l1_hit_or_victim_line_d_reg;
logic [(L1WAYSIZE-1):0]                     l1_hit_or_lru_line_o_next   , l1_hit_or_lru_line_o_reg;
logic [(PASIZE-1):0]                        l1_victim_line_addr_next    , l1_victim_line_addr_reg;         

//======================
// TLB SIGNALS 
//======================
logic                            tlb_cs_i;           // Chip Select Signal 
logic                            tlb_write_read_i;   // 1: WRITE  0: READ TLB    
logic [(VPNSIZE-1):0]            tlb_vpn_i;          // Virtual  Page Number Input  
logic [(PPNSIZE-1):0]            tlb_ppn_i;          // Physical Page Number Input                        
logic [(PPNSIZE-1):0]            tlb_ppn_o;          // Pyhsical Page Number Output                      
logic                            tlb_hit_o;          // 1: TLB hit 0:TLB miss
logic                            tlb_output_valid_o; // 1:TLB Output is valid.    
//======================
// PAGE TABLE 
//======================
logic                    pt_cs_i;
logic                    pt_write_read_i;
logic [(VPNSIZE-1):0]    pt_VPN_i; 
logic [(PPNSIZE-1):0]    pt_PPN_i;
logic                    pt_output_v_o;
logic                    pt_page_fault_o;
logic [(PPNSIZE-1):0]    pt_PPN_o;
//======================
// L2 Signals 
//======================
logic                           l2_cs_i;
logic [2:0]                     l2_cmd_i; 
logic                           l2_cmd_done_o;        
							   
logic  [(L2ADDRSIZE-1):0]       l2_addr_i;         
logic  [(L2WAYSIZE-1):0]        l2_line_num_i;     
logic  [2:0]                    l2_funct3_i;       
// CMD0: CHECH HIT                
logic                           l2_hit_o;          
logic [(L2WAYSIZE-1):0]         l2_hit_or_victim_line_o;
logic                           l2_hit_or_victim_line_v_o;
logic                           l2_hit_or_victim_line_d_o;
logic [(PASIZE-1):0]            l2_victim_line_addr_o,l2_victim_line_addr_o_next,l2_victim_line_addr_o_reg;
// CMD1: Line SIGLE-DATA Read   
logic  [31:0]                   l2_rd_line_data_o; 
logic                           l2_rd_valid_o;     
logic                           l2_wrt_valid_i;    
logic                           l2_burst_done_i;  
logic                           l2_wrt_line_dirty_i;
logic                           l2_hit_pa_next, l2_hit_pa_reg;
// CMD2: Line SINGLE-DATA Write 
logic                           l2_burst_write_ready_o;
logic                           l2_busy_o;
logic  [31:0]                   l2_wrt_line_data_i;
//
logic                           l2_write_back_next,l2_write_back_reg;
logic [(L2WAYSIZE-1):0]         l2_hit_or_victim_line_pa_next,l2_hit_or_victim_line_pa_reg;
logic [(L2WAYSIZE-1):0]         l2_hit_or_victim_line_l1victim_next,l2_hit_or_victim_line_l1victim_reg;
logic                           l2_inclusion_bit_i;
//======================
// External Memory Signals 
//======================
logic        mem_cs_i;           // CHIP SELECT  
logic        mem_write_read_i;   // 1:Write 0:Read 
logic [31:0] mem_addr_i;         // READ/WRITE ADDR 
logic [31:0] mem_write_data_i;   // WRITE DATA WORD
logic        mem_write_data_v_i; // WRITE DATA VALID
logic        mem_write_ready_o;
logic [31:0] mem_read_data_o;    // READ DATA WORD 
logic        mem_read_data_v_o;  // READ DATA VALID 
logic        mem_burst_done_o; 


//======================
// Local Signals 
//======================
    
 typedef enum {
    IDLE,
    //
    TLB_MISS,
    READ_PAGE_TABLE_CMD,
    WAIT_READ_PAGE_TABLE_CMD,
    WRITE_TLB_CMD,
    WRITE_PAGE_TABLE_CMD,
    CHECK_TLB_L1_CMD,
    //
    FETCH,
    L1_SINGLE_DATA_READ_CMD,
    L1_SINGLE_DATA_WRITE_CMD,
    WAIT_L1_SINGLE_DATA_READ_CMD,
    WAIT_L1_SINGLE_DATA_WRITE_CMD,
    L1_BROADCAST,
    //
    L2_CHECK_HIT_CMD_S0,
    WAIT_L2_CHECK_HIT_CMD_S0,
    L2_CHECK_HIT_CMD_VICTIM_S0,
    WAIT_L2_CHECK_HIT_CMD_VICTIM_S0,    
    L2_CLEAR_INCLUSION_BIT_CMD_S0,
    MUX_1,
    // STACK 1 
    L2_BURST_WRITE_CMD_S1, 
    WAIT_L2_BURST_WRITE_READY_S1,
    L1_BURST_READ_CMD_S1,
    WAIT_L1_L2_BUSY_S1,
    MUX_2,
    // STACK 2
    MEM_BURST_WRITE_CMD_S2,
    WAIT_MEM_BURST_WRITE_READY_S2,
    L2_BURST_READ_CMD_S2,
    WAIT_MEM_L2_BUSY_S2,
    MUX_3,
    // STACK 3
    L1_BURST_WRITE_CMD_S3,
    WAIT_L1_BURST_WRITE_READY_S3,
    L2_BURST_READ_CMD_S3,
    WAIT_L1_L2_BUSY_S3,
    // STACK 4
    L1_L2_BURST_WRITE_CMD_S4,
    WAIT_L1_L2_BURST_WRITE_READY_S4,
    EX_MEM_BURST_READ_CMD_S4,
    WAIT_MEM_L2_BUSY_S4,
    L2_SET_INCLUSION_BIT
 } state_type;     
    
state_type cstate,nstate; 
    

logic [(VASIZE-1):0]            va_reg     ,va_next;            // Memory Address 
logic [(ROBSIZE-1):0]           i_rob_addr_reg       ,i_rob_addr_next;        // ROB Addr
logic                           i_load_store_reg     ,i_load_store_next;      // LOAD OR STORE COMMAND 
logic [31:0]                    i_store_data_reg     ,i_store_data_next;      // Store Data 
logic [1:0]                     i_funct3_reg         ,i_funct3_next;          // Data Size-Byte|Half-Word|Word
//
logic                           l1_hit;              
logic [(PAGEOFFSETSIZE-1):0]    page_offset;
logic [(VAPAGENUMBERSIZE-1):0]  virtual_page_number;
logic [(TLBINDEXSIZE-1):0]      tlb_index;
logic [(TLBTAGSIZE-1):0]        tlb_tag;
logic [(PASIZE-1):0]            PA_next, PA_reg;
logic [(PPNSIZE-1):0]           ppa_count_next, ppa_count_reg;
logic [(PPNSIZE-1):0]           tlb_ppn_in_next, tlb_ppn_in_reg;
//
logic                           o_broadcast_ready_next;    // Broadcast Data Ready 
logic [(ROBSIZE-1):0]           o_broadcast_rob_addr_next; // Broadcast ROB Addr 
logic [31:0]                    o_broadcast_load_data_next;// Broadcast Data 
//
logic                           l1_write_back_next, l1_write_back_reg;
logic                           loop_break;
// Pipeline Signals 
logic  inst_valid_next,inst_valid,inst_valid_reg_next,inst_valid_reg;
logic  stall; 
logic  taken_pre;
logic  taken_pre_reg2,taken_pre_reg3;
logic  [31:0] pc_target_pre, pc_target_pre_reg2;


//=======================END OF SIGNAL DEFINITIONS===============================    
    
//===========================================
//    STATE-DIAGRAM 
//===========================================

//======================
// STATE DIAGRAM 
//======================
always_comb
begin
   stall = i_iq_full;
   //
   loop_break = 1'b0;
   nstate = cstate;
   //
   pc_next      = pc_reg;
   pc_reg2_next = pc_reg2; 
   pc_reg3_next = pc_reg3;
   //
   virtual_page_number        = pc_reg[(VASIZE-1):PAGEOFFSETSIZE];
   tlb_index                  = virtual_page_number[(TLBINDEXSIZE-1):0];
   tlb_tag                    = virtual_page_number[(VAPAGENUMBERSIZE-1):TLBINDEXSIZE];
   //
   i_rob_addr_next            = i_rob_addr_reg;
   i_load_store_next          = i_load_store_reg;    
   i_store_data_next          = i_store_data_reg;    
   i_funct3_next              = i_funct3_reg;
   l1_hit_or_lru_line_o_next  = l1_hit_or_lru_line_o_reg;
   l1_write_back_next         = l1_write_back_reg; 
   PA_next                    = PA_reg;
   ppa_count_next             = ppa_count_reg;
   tlb_ppn_in_next            = tlb_ppn_in_reg;
   //======================= 
   // INITIALIZE BHT READ SIGNALS
   //=======================
   bht_read_en_i      = '0;
   bht_read_pc_i      = '0; 
   //======================= 
   // INITIALIZE BHT WRITE SIGNALS
   //=======================
   bht_write_en_i     = '0;
   bht_write_pc_i     = '0;
   bht_write_taken_i  = '0; 
   //=======================
   // INITIALIZE BTB SIGNALS
   //======================= 
   btb_read_en_i      = '0; 
   btb_read_pc_i      = '0;  
   btb_write_en_i     = '0; 
   btb_write_pc_i     = '0;  
   btb_write_target_i = '0;
   btb_write_is_ret_i = '0; 
   //=====================   
   // INITIAL L1 SIGNALS  
   //=====================  
   l1_cs_i           = '0;
   l1_cmd_i          = '0;   
   l1_addr_i         = '0;  
   l1_line_num_i     = '0;           
   l1_funct3_i       = '0;                       
   l1_wrt_valid_i    = '0;        
   l1_burst_done_i   = '0;           
   l1_wrt_line_data_i= '0; 
   //
   l1_rd_set_i       = '0;
   l1_rd_set_addr_i  = '0;
   //
   l1_hit_or_victim_line_v_next = l1_hit_or_victim_line_v_reg;
   l1_hit_or_victim_line_d_next = l1_hit_or_victim_line_d_reg;
   l1_victim_line_addr_next     = l1_victim_line_addr_reg;
   l1_hit                       = '0;
   //=====================  
   // INITIAL TLB SIGNALS 
   //===================== 
   tlb_cs_i         = '0;       
   tlb_write_read_i = '0;
   tlb_vpn_i        = '0;      
   tlb_ppn_i        = '0;       
   //=====================  
   // INITIAL PAGE TABLE SIGNALS 
   //===================== 
   pt_cs_i         = '0;
   pt_write_read_i = '0;
   pt_VPN_i        = '0;
   pt_PPN_i        = '0;
   //===================== 
   // INITIAL L2 SIGNALS  
   //===================== 
   l2_cs_i            = '0;
   l2_cmd_i           = '0; 
   l2_addr_i          = '0;         
   l2_line_num_i      = '0;     
   l2_funct3_i        = '0;       
   l2_wrt_valid_i     = '0;    
   l2_burst_done_i    = '0;   
   l2_wrt_line_data_i = '0;
   l2_hit_pa_next     = l2_hit_pa_reg;
   l2_victim_line_addr_o_next = l2_victim_line_addr_o_reg;
   //
   l2_hit_or_victim_line_pa_next        = l2_hit_or_victim_line_pa_reg;
   l2_hit_or_victim_line_l1victim_next  = l2_hit_or_victim_line_l1victim_reg;
   l2_wrt_line_dirty_i = '0;
   l2_inclusion_bit_i  = '0;
   //===================== 
   // INITIAL MEMORY SIGNALS  
   //===================== 
   mem_cs_i             = '0;
   mem_write_read_i     = '0;
   mem_addr_i           = '0;
   mem_write_data_i     = '0;
   mem_write_data_v_i   = '0;
   //
   inst_valid_reg_next  = inst_valid;
   l1_inst_o_reg_next   = l1_inst_o;
   inst_valid_next      = 1'b0;
   
   case(cstate)
      IDLE: 
       begin
           if(i_start)
           begin
               //============================
               // L1 Signals: Read the set for the last PC. 
               //============================
               l1_rd_set_i       = 1'b1;
               l1_rd_set_addr_i  = pc_reg;
               //============================
               // SET TLB Signals to Read the Physical Address
               //============================
               tlb_cs_i          = 1'b1;     
               tlb_write_read_i  = 1'b0;
               tlb_vpn_i         = virtual_page_number;      
               tlb_ppn_i         = '0;       
               //============================
               // Set the BHT Signals to read the BHT 
               //============================
               bht_read_en_i      = 1'b1;
               bht_read_pc_i      = pc_reg;
               //============================
               // Set the BTB Signals to read the BTB
               //============================               
               btb_read_en_i      = 1'b1; 
               btb_read_pc_i      = pc_reg;
                 
               
                                                                        nstate = FETCH;
           end
       end 
      FETCH: 
      begin

         //=================================================================
    if(stall)
    begin
       inst_valid_next      = inst_valid;
       inst_valid_reg_next  = inst_valid_reg;
       l1_inst_o_reg_next   = l1_inst_o_reg;
	end else begin
	   //
	   l1_write_back_next   = 1'b0;  
       pc_reg2_next         = pc_reg; 
       pc_reg3_next         = pc_reg2;  
       //
         if(tlb_hit_o)
         begin
             //$display("TLB HIT");
             PA_next = {tlb_ppn_o,pc_reg[(PAGEOFFSETSIZE-1):0]};
            //================================================================
            // Check L1 HIT 
            for(int i =0;i<(2**L1WAYSIZE);i++)
            begin
               if((tlb_ppn_o == l1_rd_set_tags_o[i]) && l1_rd_set_valid_bits_o[i] &&~loop_break)
               begin
               l1_hit_or_lru_line_o_next  = i;
               l1_hit     = 1'b1;
               loop_break = 1'b1;
               end 
            end 
            //================================================================
            if(l1_hit)
            begin
               //$display("L1 HIT");
               
               //=============== 
               // TLB_ HIT && L1_HIT 
               //=============== 
               // READ THE INSTRUCTION FROM L1 
               l1_cs_i           = 1'b1;
               l1_cmd_i          = 3'd1; 
               l1_addr_i         = PA_next; 
               l1_line_num_i     = l1_hit_or_lru_line_o_next;
               //============================
               // Set the BHT Signals to read the BHT 
               //============================
               bht_read_en_i      = 1'b1;
               bht_read_pc_i      = pc_reg;
               //============================
               // Set the BTB Signals to read the BTB
               //============================               
               btb_read_en_i      = 1'b1; 
               btb_read_pc_i      = pc_reg;   
                
               // BRANCH TARGET/TAKEN DECISION 
               taken_pre     = (bht_hit_o & btb_read_hit_o) & bht_taken_pre_o; 
               pc_target_pre =  btb_is_ret_o ? ras_read_pc_o : btb_target_pre_o; 
               
               
               
               //=============== PC MUX IN CASE OF TLB and L1 HIT =====================
               case(taken_pre)
                  1'b0: begin 
                  pc_next = pc_reg + 4;       
                  inst_valid_next = 1'b1;
                  end 
                  1'b1: begin 
                  pc_next = pc_target_pre;    
                  inst_valid_next = 1'b0;
                  end 
               endcase 
               
               
                
                
               //============================
               // L1 Signals: Read the set for the last PC. 
               //============================
               l1_rd_set_i       = 1'b1;
               l1_rd_set_addr_i  = pc_next;  
               
               
               //=============== STATE SELECT ===================== 
                                                                                       nstate = FETCH;
               //=============== pc_next Decoding =====================  
               page_offset           = pc_next[(PAGEOFFSETSIZE-1):0];
               virtual_page_number   = pc_next[(VASIZE-1):PAGEOFFSETSIZE];
               tlb_index             = virtual_page_number[(TLBINDEXSIZE-1):0];
               //=============== TLB SIGNALS =====================  
               tlb_cs_i              = 1'b1;     
               tlb_write_read_i      = 1'b0;
               tlb_vpn_i             = virtual_page_number;      
               tlb_ppn_i             = '0;      
               // 
               
               
               
               
            end else begin 
               //$display("L1 MISS");
               // L1 MISS
               l1_hit_or_lru_line_o_next    = l1_plru_line_num_o;
               l1_hit_or_victim_line_v_next = l1_rd_set_valid_bits_o[l1_plru_line_num_o];
               l1_hit_or_victim_line_d_next = l1_rd_set_dirty_bits_o[l1_plru_line_num_o];
               l1_write_back_next           = l1_rd_set_valid_bits_o[l1_plru_line_num_o]  & l1_rd_set_dirty_bits_o[l1_plru_line_num_o];
               l1_victim_line_addr_next     = {l1_rd_set_tags_o[l1_plru_line_num_o],va_reg[(L1INDEXSIZE+L1OFFSETSIZE-1):0]};
               /*$display(" l1_hit_or_lru_line_o_next=%h l1_hit_or_victim_line_v_next =%h l1_hit_or_victim_line_d_next =%h  l1_write_back_next =%h l1_victim_line_addr_next = %h" ,l1_hit_or_lru_line_o_next,   
                           l1_hit_or_victim_line_v_next,
                           l1_hit_or_victim_line_d_next,
                           l1_write_back_next,          
                           l1_victim_line_addr_next);      */
                                                                                       nstate = L2_CHECK_HIT_CMD_S0;
            end 
            //
         end else begin
            // TLB MISS   
            //$display("-TLB MISS");
            
            
            
                                                                                        nstate = READ_PAGE_TABLE_CMD;
	     
         end // TLB MISS
    end // if(~stall)		        
            

      end //FETCH
      READ_PAGE_TABLE_CMD:
      begin
        pt_cs_i         = 1'b1;
        pt_write_read_i = 1'b0;
        pt_VPN_i        = virtual_page_number;
        //$display("-READ PAGE TABLE WITH --> pt_VPN_i = %h ",pt_VPN_i);
         
                                                                                     nstate = WAIT_READ_PAGE_TABLE_CMD;
      end 
      WAIT_READ_PAGE_TABLE_CMD:
      begin
         if(pt_output_v_o) begin

            //
            if(pt_page_fault_o)
            begin
            //$display("-PAGE TABLE FAULT");
                                                                                     nstate = WRITE_PAGE_TABLE_CMD;   
            end else begin
            //$display("-Physical Address FOUND in Page Table pt_PPN_o:%h",pt_PPN_o);
            tlb_ppn_in_next = pt_PPN_o;
            
                                                                                     nstate = WRITE_TLB_CMD;   
            end 
            //
         end 
      end 
      WRITE_PAGE_TABLE_CMD:
      begin
        pt_cs_i         = 1'b1;
        pt_write_read_i = 1'b1;
        pt_VPN_i        = virtual_page_number;
        pt_PPN_i        = ppa_count_reg;
        tlb_ppn_in_next = ppa_count_reg; 
        ppa_count_next  = ppa_count_reg + 1;
        //$display("Write Page Table: pt_VPN_i:%h-pt_PPN_i:%h",pt_VPN_i,pt_PPN_i);
                                                                                     nstate = WRITE_TLB_CMD; 
      end 
      WRITE_TLB_CMD:
      begin
         //$display("Write TLB");
         tlb_cs_i         = 1'b1;       
         tlb_write_read_i = 1'b1;
         tlb_vpn_i        = virtual_page_number;    
         tlb_ppn_i        = tlb_ppn_in_reg; 
                                                                                     nstate = CHECK_TLB_L1_CMD; 
      end 
      CHECK_TLB_L1_CMD:
      begin
         //$display("PC REG: %h ",pc_reg);
         //============================
         // L1 Signals: Read the set for the last PC. 
         //============================
         l1_rd_set_i       = 1'b1;
         l1_rd_set_addr_i  = pc_reg;
         //============================
         // SET TLB Signals to Read the Physical Address
         //============================
         tlb_cs_i          = 1'b1;     
         tlb_write_read_i  = 1'b0;
         tlb_vpn_i         = virtual_page_number;      
         tlb_ppn_i         = '0;      
                                                                                     nstate = FETCH;
      end 
      
      L2_CHECK_HIT_CMD_S0:
      begin
         l2_cs_i            = 1'b1;
         l2_cmd_i           = 'd0; 
         l2_addr_i          = PA_reg;     
                                                                        nstate = WAIT_L2_CHECK_HIT_CMD_S0;
      end 
      WAIT_L2_CHECK_HIT_CMD_S0:
      begin
         if(l2_cmd_done_o)begin
            l2_write_back_next            = ~l2_hit_o & (l2_hit_or_victim_line_v_o & l2_hit_or_victim_line_d_o); 
            l2_hit_pa_next                = l2_hit_o;
            l2_hit_or_victim_line_pa_next = l2_hit_or_victim_line_o;
            l2_victim_line_addr_o_next    = l2_victim_line_addr_o;
            
            if(l1_hit_or_victim_line_v_reg) begin
            // L1 VICTIM LINE IS VALID so we need to write it back to L2 
                                                                        nstate = L2_CHECK_HIT_CMD_VICTIM_S0; 
            end else begin
            // L1 VICTIM LINE IS NO VALID so we dont need to write it back 
                                                                        nstate = MUX_1;
            end 
            
         end 
      end //WAIT_L2_CHECK_HIT_CMD_S0
      L2_CHECK_HIT_CMD_VICTIM_S0:
      begin
         l2_cs_i            = 1'b1;
         l2_cmd_i           = 'd0; 
         l2_addr_i          = l1_victim_line_addr_reg; 
                                                                        nstate = WAIT_L2_CHECK_HIT_CMD_VICTIM_S0; 
      end //L2_CHECK_HIT_CMD_VICTIM_S0
      WAIT_L2_CHECK_HIT_CMD_VICTIM_S0:
      begin
         if(l2_cmd_done_o)begin
            l2_hit_or_victim_line_l1victim_next = l2_hit_or_victim_line_o;
                                                                        nstate = L2_CLEAR_INCLUSION_BIT_CMD_S0;
         end
      end //WAIT_L2_CHECK_HIT_CMD_VICTIM_S0   
      L2_CLEAR_INCLUSION_BIT_CMD_S0:
      begin
         //
         l2_cs_i               = 1'b1;
         l2_cmd_i              = `L2_UPDATE_INCLUSION_CMD; 
         l2_addr_i             = l1_victim_line_addr_reg; 
         l2_line_num_i         = l2_hit_or_victim_line_l1victim_reg;
         l2_inclusion_bit_i    = 1'b0;
                                                                        nstate = MUX_1;
         //
      end //L2_CLEAR_INCLUSION_BIT_CMD_S0   
      MUX_1:
      begin
         if(l1_write_back_reg)
         begin
                                                                        nstate = L2_BURST_WRITE_CMD_S1;
         end else begin
                                                                        nstate = MUX_2;
         end 
      end 
      ///////////////////////////////// 
      // STACK 1 
      ///////////////////////////////// 
      L2_BURST_WRITE_CMD_S1:
      begin
          l2_cs_i       = 1'b1;
          l2_cmd_i      = 'd4; 
          l2_addr_i     = l1_victim_line_addr_reg; 
          l2_line_num_i = l2_hit_or_victim_line_l1victim_reg;
          l2_wrt_line_dirty_i = l1_hit_or_victim_line_d_reg; // This should be always 1  
                                                                       nstate = WAIT_L2_BURST_WRITE_READY_S1;                                                     
      end // L2_BURST_WRITE_CMD_S1
      WAIT_L2_BURST_WRITE_READY_S1:
      begin
         if(l2_burst_write_ready_o)
         begin
                                                                       nstate = L1_BURST_READ_CMD_S1;                                                     
         end 
      end 
      L1_BURST_READ_CMD_S1:
      begin
         l1_cs_i           = 1'b1;
         l1_cmd_i          = 'd3; 
         l1_addr_i         = l1_victim_line_addr_reg; 
         l1_line_num_i     = l1_hit_or_lru_line_o_reg;
                                                                       nstate = WAIT_L1_L2_BUSY_S1;                                                     
      end //L1_BURST_READ_CMD_S1
      WAIT_L1_L2_BUSY_S1:
      begin
         // 
         l2_wrt_line_data_i = l1_inst_o;
         l2_wrt_valid_i     = l1_rd_valid_o;
         //
         if(~l1_busy_o  && ~l2_busy_o)
         begin
            nstate = MUX_2;
         end                                                              
      end 
      MUX_2:
      begin
         if(l2_write_back_reg)
         begin
         // WRITE L2 VICTIM LINE BACK
                                                               nstate = MEM_BURST_WRITE_CMD_S2;
         end else begin
         // NO WRITE L2 VICTIM LINE BACK 
                                                               nstate = MUX_3;
         end   
      end // MUX_3 
      //==============================
      // STACK 2
      //============================== 
      MEM_BURST_WRITE_CMD_S2:
      begin
         mem_cs_i           <= 1'b1; // CHIP SELECT  
         mem_write_read_i   <= 1'b1; // 1:Write 0:Read 
         mem_addr_i         <= l2_victim_line_addr_o_reg; // READ/WRITE ADDR 
         mem_write_data_i   <= '0;   // WRITE DATA WORD
         mem_write_data_v_i <= '0;   // WRITE DATA VALID
                                                               nstate = WAIT_MEM_BURST_WRITE_READY_S2;
      end                          
      WAIT_MEM_BURST_WRITE_READY_S2:
      begin
         if(mem_write_ready_o)
         begin
                                                               nstate = L2_BURST_READ_CMD_S2;
         end 
      end 
      L2_BURST_READ_CMD_S2:
      begin
         l2_cs_i               = 1'b1;
         l2_cmd_i              = `L2_BURST_READ_CMD; 
         l2_addr_i             = l2_victim_line_addr_o_reg; 
         l2_line_num_i         = l2_hit_or_victim_line_pa_reg;
                                                                        nstate = WAIT_MEM_L2_BUSY_S2;
      end 
      WAIT_MEM_L2_BUSY_S2:
      begin
         mem_write_data_i    = l2_rd_line_data_o;
         mem_write_data_v_i  = l2_rd_valid_o;
         if(~l2_busy_o)
         begin
                                                                        nstate = MUX_3;
         end 
      end 
      MUX_3:
      begin
         if(l2_hit_pa_reg)
         begin
                                                                        nstate = L1_BURST_WRITE_CMD_S3;
         end else begin
                                                                        nstate = L1_L2_BURST_WRITE_CMD_S4;
         end 
      end // MUX_3
      //==============================
      // STACK 3
      //==============================
      L1_BURST_WRITE_CMD_S3:
      begin
         l1_cs_i               = 1'b1;
         l1_cmd_i              = 'd4;
         l1_addr_i             = PA_reg; 
         l1_line_num_i         = l1_hit_or_lru_line_o_reg;
                                                                        nstate = WAIT_L1_BURST_WRITE_READY_S3;
      end 
      WAIT_L1_BURST_WRITE_READY_S3:
      begin
         if(l1_burst_write_ready_o)
         begin
                                                                        nstate = L2_BURST_READ_CMD_S3;
         end   
      end 
      L2_BURST_READ_CMD_S3:
      begin
         l2_cs_i               = 1'b1;
         l2_cmd_i              = 'd3; 
         l2_addr_i             = PA_reg; 
         l2_line_num_i         = l2_hit_or_victim_line_pa_reg;
                                                                          nstate = WAIT_L1_L2_BUSY_S3;       
      end 
      WAIT_L1_L2_BUSY_S3:
      begin
        l1_wrt_valid_i      = l2_rd_valid_o;
        l1_wrt_line_data_i  = l2_rd_line_data_o;
         if(~l1_busy_o  && ~l2_busy_o)
         begin
            nstate = L2_SET_INCLUSION_BIT;
         end   
      end 
      //==============================
      // STACK 4
      //==============================
      L1_L2_BURST_WRITE_CMD_S4:
      begin
         l1_cs_i               = 1'b1;
         l1_cmd_i              = 'd4;
         l1_addr_i             = PA_reg; 
         l1_line_num_i         = l1_hit_or_lru_line_o_reg;

         l2_cs_i               = 1'b1;
         l2_cmd_i              = 'd4; 
         l2_addr_i             = PA_reg; 
         l2_line_num_i         = l2_hit_or_victim_line_pa_reg;
         l2_wrt_line_dirty_i   = 1'b0;
                                                                        nstate = WAIT_L1_L2_BURST_WRITE_READY_S4;
      end //L1_L2_BURST_WRITE_CMD_S4
      WAIT_L1_L2_BURST_WRITE_READY_S4:
      begin
         if(l1_burst_write_ready_o && l2_burst_write_ready_o)
         begin
                                                                        nstate = EX_MEM_BURST_READ_CMD_S4;
         end       
      end // WAIT_L1_L2_BURST_WRITE_READY_S4  
      EX_MEM_BURST_READ_CMD_S4:
      begin
         mem_cs_i           = 1'b1;
         mem_write_read_i   = 1'b0;
         mem_addr_i         = PA_reg;
      
                                                                        nstate = WAIT_MEM_L2_BUSY_S4;
      end 
      WAIT_MEM_L2_BUSY_S4:
      begin
         l1_wrt_line_data_i = mem_read_data_o;
         l1_wrt_valid_i     = mem_read_data_v_o;
         l2_wrt_line_data_i = mem_read_data_o;
         l2_wrt_valid_i     = mem_read_data_v_o;
         if(~l1_busy_o && ~l2_busy_o)
         begin
                                                                        nstate = L2_SET_INCLUSION_BIT;
         end 
      end 
      L2_SET_INCLUSION_BIT:
      begin
         l2_cs_i               = 1'b1;
         l2_cmd_i              = `L2_UPDATE_INCLUSION_CMD; 
         l2_addr_i             = PA_reg; 
         l2_line_num_i         = l2_hit_or_victim_line_pa_reg;
         l2_inclusion_bit_i    = 1'b1;
         // The Data is stored in L1.
         // READ L1 sets again and check for hit. 
         nstate = FETCH; 
      end 
   endcase
end 


//========================================
// State Diagram Registers 
//========================================
always_ff @(posedge clk or negedge rstn)
begin
   if(!rstn)
   begin
      cstate                       <= IDLE;
      pc_reg                       <= '0;
      va_reg                       <= '0;
      i_rob_addr_reg               <= '0;
      i_load_store_reg             <= '0;    
      i_store_data_reg             <= '0;    
      i_funct3_reg                 <= '0;
      l2_write_back_reg            <= '0;  
      l1_write_back_reg            <= '0;
      l1_hit_or_victim_line_v_reg  <= '0;
      l1_hit_or_victim_line_d_reg  <= '0;
      l1_victim_line_addr_reg      <= '0;  
      PA_reg                       <= '0;   
      l1_hit_or_lru_line_o_reg     <= '0;
      l2_hit_pa_reg                <= '0;
      l2_hit_or_victim_line_pa_reg <= '0;
      l2_hit_or_victim_line_l1victim_reg <= '0;
      l2_victim_line_addr_o_reg    <= '0;
      ppa_count_reg                <= '0;
      tlb_ppn_in_reg               <= '0;
      l1_inst_o_reg                <= '0;            
      inst_valid                   <= '0;
      inst_valid_reg               <= '0;
      pc_reg2                      <= '0; 
      pc_reg3                      <= '0;
      taken_pre_reg2               <= '0;
      taken_pre_reg3               <= '0;
      pc_target_pre_reg2           <= '0;
   end else if(i_flush) begin
      cstate                       <= IDLE;
      pc_reg                       <= i_flush_pc;
      va_reg                       <= '0;
      i_rob_addr_reg               <= '0;
      i_load_store_reg             <= '0;    
      i_store_data_reg             <= '0;    
      i_funct3_reg                 <= '0;
      l2_write_back_reg            <= '0;  
      l1_write_back_reg            <= '0;
      l1_hit_or_victim_line_v_reg  <= '0;
      l1_hit_or_victim_line_d_reg  <= '0;
      l1_victim_line_addr_reg      <= '0;  
      PA_reg                       <= '0;   
      l1_hit_or_lru_line_o_reg     <= '0;
      l2_hit_pa_reg                <= '0;
      l2_hit_or_victim_line_pa_reg <= '0;
      l2_hit_or_victim_line_l1victim_reg <= '0;
      l2_victim_line_addr_o_reg    <= '0;
      tlb_ppn_in_reg               <= '0;
      l1_inst_o_reg                <= '0;            
      inst_valid                   <= '0;
      inst_valid_reg               <= '0;
      pc_reg2                      <= '0; 
      pc_reg3                      <= '0;
      taken_pre_reg2               <= '0;
      taken_pre_reg3               <= '0;
      pc_target_pre_reg2           <= '0;
   end else begin
      cstate                       <= nstate; 
      pc_reg                       <= pc_next;
      pc_reg2                      <= pc_reg2_next; 
      pc_reg3                      <= pc_reg3_next;
      taken_pre_reg2               <= taken_pre;
      taken_pre_reg3               <= taken_pre_reg2;
      pc_target_pre_reg2           <= pc_target_pre;
      va_reg                       <= va_next;
      i_rob_addr_reg               <= i_rob_addr_next;
      i_load_store_reg             <= i_load_store_next;    
      i_store_data_reg             <= i_store_data_next;    
      i_funct3_reg                 <= i_funct3_next;   
      //
      l1_write_back_reg            <= l1_write_back_next;
      //
      l1_hit_or_victim_line_v_reg  <= l1_hit_or_victim_line_v_next;
      l1_hit_or_victim_line_d_reg  <= l1_hit_or_victim_line_d_next;
      l1_victim_line_addr_reg      <= l1_victim_line_addr_next;
      l2_write_back_reg            <= l2_write_back_next; 
      //
      PA_reg                       <= PA_next;
      l1_hit_or_lru_line_o_reg     <= l1_hit_or_lru_line_o_next;
      l2_hit_pa_reg                <= l2_hit_pa_next;
      l2_hit_or_victim_line_pa_reg <= l2_hit_or_victim_line_pa_next;
      l2_hit_or_victim_line_l1victim_reg <= l2_hit_or_victim_line_l1victim_next;
      l2_victim_line_addr_o_reg    <= l2_victim_line_addr_o_next;
      //
      ppa_count_reg                <= ppa_count_next;
      //
      tlb_ppn_in_reg               <= tlb_ppn_in_next;
      //
      l1_inst_o_reg                <= l1_inst_o;
      inst_valid                   <= inst_valid_next;
      inst_valid_reg               <= inst_valid_reg_next;
      //
    
   end 
end 


//========================================
// BHT-Instantiation 
//========================================
BHT__2bit_predictor
#(.WAYSIZE   (BHTWAYSIZE),            
  .OFFSETBIT (BHTOFFSETBIT),
  .INDEXBIT  (BHTINDEXBIT),
  .TAGBIT    (BHTTAGBIT))
uBHT__2bit_predictor
   (
   .clk                 (clk), 
   .rstn                (rstn),
   //======================= 
   // READ SIGNALS 
   //=======================
   .i_bht_read_en       (bht_read_en_i),
   .i_bht_read_pc       (bht_read_pc_i), 
   .o_bht_taken_pre     (bht_taken_pre_o),
   .o_bht_hit           (bht_hit_o),
   //======================= 
   // WRITE SIGNALS 
   //=======================
   .i_bht_write_en      (i_bht_write_en), 
   .i_bht_write_pc      (i_bht_write_pc), 
   .i_bht_write_taken   (i_bht_write_taken) 
    );
	

//========================================
// BTB-Instantiation 
//========================================	
BTB
#(.PCSIZE     (BTBPCSIZE),
  .WAYSIZE    (BTBWAYSIZE),            
  .OFFSETBIT  (BTBOFFSETBIT),
  .INDEXBIT   (BTBINDEXBIT),
  .TAGBIT     (BTBTAGBIT) )
uBTB
(
  .clk                    (clk), 
  .rstn                   (rstn),
  // FLUSH FLAG
  .i_flush                (1'b0),
  //======================================= 
  //------------- READ SIGNALS ------------  
  //=======================================  
  .i_btb_read_en          (btb_read_en_i),
  .i_btb_read_pc          (btb_read_pc_i), 
  .o_btb_read_hit         (btb_read_hit_o),
  .o_btb_target_pre       (btb_target_pre_o), 
  .o_btb_is_ret           (btb_is_ret_o),        
  //======================================= 
  //------------- WRITE SIGNALS ------------  
  //=======================================
  .i_btb_write_en         (i_btb_write_en),
  .i_btb_write_pc         (i_btb_write_pc), 
  .i_btb_write_target     (i_btb_write_target),
  .i_btb_write_is_ret     (i_btb_write_is_ret)   
    );


//========================================
// RAS-Instantiation 
//========================================
RAS
#(.DATAWIDTH (RASDATAWIDTH),
  .FIFODEPTH (RASFIFODEPTH)  )
uRAS
(
  .clk             (clk),
  .rstn            (rstn ),
  //
  .i_ras_write_en  (ras_write_en_i),
  .i_ras_write_pc  (ras_write_pc_i),
  //
  .i_ras_read_en   (ras_read_en_i),
  .o_ras_read_pc   (ras_read_pc_o),            
  //
  .o_full          (ras_full_o),
  .o_empty         (ras_empty_o)
    );



//========================================
// L1-Instantiation 
//========================================
L1_ICACHE
#(.WAYSIZE    (L1WAYSIZE),             
  .OFFSETBIT  (L1OFFSETSIZE),
  .INDEXBIT   (L1INDEXSIZE), 
  .TAGBIT     (L1TAGSIZE) 
  )
uL1_ICACHE
(
   .clk                     (clk),
   .rstn                    (rstn),
   .i_flush                 (i_flush),
   //
   .i_l1_cs                 (l1_cs_i),
   .i_l1_cmd                (l1_cmd_i),
   .o_l1_cmd_done           (l1_cmd_done_o), 
   //
   .i_l1_addr               (l1_addr_i),         
   .i_l1_line_num           (l1_line_num_i),          
   .i_l1_funct3             (l1_funct3_i),                       
   //
   .i_l1_rd_set             (l1_rd_set_i),
   .i_l1_rd_set_addr        (l1_rd_set_addr_i),
   .o_l1_rd_set_tags        (l1_rd_set_tags_o), 
   .o_l1_rd_set_valid_bits  (l1_rd_set_valid_bits_o),
   .o_l1_rd_set_dirty_bits  (l1_rd_set_dirty_bits_o),
   .o_l1_plru_line_num      (l1_plru_line_num_o),
   //
   .o_l1_rd_line_data       (l1_inst_o),        
   .o_l1_rd_valid           (l1_rd_valid_o),          
   .i_l1_wrt_valid          (l1_wrt_valid_i),         
   .i_l1_burst_done         (l1_burst_done_i),          
   //
   .i_l1_wrt_line_data      (l1_wrt_line_data_i),
   .o_l1_burst_write_ready  (l1_burst_write_ready_o),
   .o_l1_busy               (l1_busy_o)         
);
    

   
   
    
//========================================
// TLB-Instantiation 
//======================================== 
TLB
#(.INDEXSIZE  (TLBINDEXSIZE),
  .VPNSIZE    (VPNSIZE),
  .PPNSIZE    (PPNSIZE)
  )
uTLB
  (
  .clk                     (clk),               // Clock 
  .rstn                    (rstn),              // Asynchronous Reset 
  .i_tlb_cs                (tlb_cs_i),          // Chip Select Signal 
  .i_tlb_write_read        (tlb_write_read_i),  // 1: WRITE  0: READ TLB    
  .i_tlb_vpn               (tlb_vpn_i),         //  Virtual  Page Number Input  
  .i_tlb_ppn               (tlb_ppn_i),         //  Physical Page Number Input                        
  .o_tlb_ppn               (tlb_ppn_o),         // Pyhsical Page Number Output                      
  .o_tlb_hit               (tlb_hit_o),         // 1: TLB hit 0:TLB miss
  .o_tlb_output_valid      (tlb_output_valid_o) // 1: TLB output is valid  
    );
    







//========================================
// PAGE TABLE 
//========================================       

PAGE_TABLE
#(.ENTRYSIZE (PTENTRYSIZE),
  .VPNSIZE   (VPNSIZE), 
  .PPNSIZE   (PPNSIZE))
uPAGE_TABLE
(
    .clk              (clk), 
    .rstn             (rstn), 
    .i_cs             (pt_cs_i), 
    .i_write_read     (pt_write_read_i), 
    .i_VPN            (pt_VPN_i),  
    .i_PPN            (pt_PPN_i), 
    .o_output_v       (pt_output_v_o), 
    .o_page_fault     (pt_page_fault_o), 
    .o_PPN            (pt_PPN_o)
    );    
   
//========================================
// L2 CACHE-Instantiation 
//========================================     
L2_CACHE
#(.ADDRSIZE   (L2ADDRSIZE  ),
  .WAYSIZE    (L2WAYSIZE   ),           
  .OFFSETSIZE (L2OFFSETSIZE),
  .INDEXSIZE  (L2INDEXSIZE ),
  .TAGSIZE    (L2TAGSIZE   ) )
uL2_CACHE
(
   .clk                        (clk),
   .rstn                       (rstn),
   .i_flush                    (i_flush),
   .i_l2_cs                    (l2_cs_i),
   .i_l2_cmd                   (l2_cmd_i),
   .o_l2_cmd_done              (l2_cmd_done_o), 
   .i_l2_addr                  (l2_addr_i), 
   .i_l2_line_num              (l2_line_num_i), 
   .i_l2_funct3                (l2_funct3_i), 
   .o_l2_hit                   (l2_hit_o), 
   .o_l2_hit_or_victim_line    (l2_hit_or_victim_line_o), 
   .o_l2_hit_or_victim_line_v  (l2_hit_or_victim_line_v_o),       // Victim Line Valid Bit 
   .o_l2_hit_or_victim_line_d  (l2_hit_or_victim_line_d_o),       // Victim Line Dirty Bit
   .o_l2_victim_line_addr      (l2_victim_line_addr_o),
   .o_l2_rd_line_data          (l2_rd_line_data_o), 
   .o_l2_rd_valid              (l2_rd_valid_o), 
   .i_l2_wrt_valid             (l2_wrt_valid_i), 
   .i_l2_burst_done            (l2_burst_done_i), 
   .i_l2_wrt_line_data         (l2_wrt_line_data_i),
   .i_l2_wrt_line_dirty        (l2_wrt_line_dirty_i),
   .o_l2_burst_write_ready     (l2_burst_write_ready_o),
   .i_l2_inclusion_bit         (l2_inclusion_bit_i),
   .o_l2_busy                  (l2_busy_o)  
    );

//========================================
// L2 CACHE-Instantiation 
//========================================  
EXTERNAL_MEMORY
#(.BLOCKSIZE    (256),  // Number of Blocks 
  .BLOCKWORDSIZE((2**L1OFFSETSIZE)/4) )   // Number of Words(32-bit) in each Block  
uEXTERNAL_MEMORY
(
   .clk            (clk),                 // Clock    
   .rstn           (rstn),                // Active low Asyncronous Reset 
   .i_cs           (mem_cs_i          ),  // CHIP SELECT  
   .i_write_read   (mem_write_read_i  ),  // 1:Write 0:Read 
   .i_addr         (mem_addr_i        ),  // READ/WRITE ADDR 
   .i_write_data   (mem_write_data_i  ),  // WRITE DATA WORD
   .i_write_data_v (mem_write_data_v_i),  // WRITE DATA VALID
   .o_write_ready  (mem_write_ready_o ),  
   .o_read_data    (mem_read_data_o   ),  // READ DATA WORD 
   .o_read_data_v  (mem_read_data_v_o ),  // READ DATA VALID 
   .o_burst_done   (mem_burst_done_o  )
    );










//===========================================
//    STAGE-1 
//===========================================











//===========================================
//    STAGE-2 
//===========================================


   
   
//===========================================
//    OUTPUT ASSIGNMENT 
//===========================================
assign o_iq_wrt_en     = ~stall & inst_valid_reg; 
assign o_iq_inst       = l1_inst_o_reg;
assign o_iq_pc         = pc_reg3;
assign o_iq_taken_pre  = taken_pre_reg2;    
assign o_iq_target_pre = pc_target_pre_reg2;  
   
    
    
endmodule :INSTRUCTION_FETCH_UNIT2

//====================================================
// TEST BENCH 
//====================================================
module INSTRUCTION_FETCH_UNIT2_tb();

  parameter PC_BOOT         = 32'h0;
  parameter ROBSIZE         = 8;
  parameter VASIZE          = 32;
  parameter PASIZE          = 12;
  parameter PAGEOFFSETSIZE  = 6;
  parameter L1OFFSETSIZE    = 4;
  parameter L1INDEXSIZE     = 2;
  parameter L1WAYSIZE       = 2;
  parameter L2INDEXSIZE     = 2;
  parameter L2WAYSIZE       = 4;
  parameter TLBINDEXSIZE    = 6;
  parameter PTENTRYSIZE     = 8;


logic clk; 
logic rstn; 
//
logic i_start; 
// Instruction Queue Signals 
logic        i_iq_full; 
logic        o_iq_wrt_en; 
logic [31:0] o_iq_inst;
logic [31:0] o_iq_pc;
logic        o_iq_taken_pre;  

// Instruction Queue Signals. 
parameter DATAWITHBIT = 32;
parameter FIFOSIZE    = 128;

logic                      i_flush;
logic                      i_wrt_en;
logic [(DATAWITHBIT-1):0]  i_wrt_data;
logic [31:0]               i_wrt_inst_pc;
logic                      i_wrt_taken;

logic                      i_rd_en;
logic [(DATAWITHBIT-1):0]  o_rd_data;
logic [31:0]               o_rd_inst_pc;
logic                      o_rd_taken;
						   
logic                      o_full;
logic                      o_empty;

						   
initial begin
clk =1'b0;
forever #10 clk = ~clk;
end 

//================================
//   RESET TASK 
//================================
task RESET();
begin
  rstn      = 1'b1;
  i_start   = 1'b0;
  i_rd_en   = '0;
  //
  repeat(2) @(posedge clk);
  rstn = 1'b0;
  repeat(2) @(posedge clk);
  rstn = 1'b1;
  repeat(2) @(posedge clk);
end 
endtask 

int counter;
//================================
//   Initialize External Memory 
//================================
task INITIALIZE_EX_MEM();
begin
   counter = 'd1;
   for(int i=0;i<256;i++)
   begin
      for(int j=0;j<4;j++)
      begin
         //uINSTRUCTION_FETCH_UNIT.uEXTERNAL_MEMORY.MEM[i][j] = counter;
         counter = counter + 1;
      end 
   end 
end
endtask 





//================================
//   START FETCH
//================================
task START();
begin
  @(negedge clk);
  i_start   = 1'b1; 
  @(negedge clk);
  i_start   = 1'b0; 
end 
endtask 

//================================
//   MAIN STIMULUS 
//================================
initial begin
   RESET();
   INITIALIZE_EX_MEM();
   START();
   wait(o_full == 1'b1);
   @(posedge clk);
   i_rd_en = 1'b1;
   wait(o_empty == 1'b1);
  @(posedge clk);
   i_rd_en = 1'b0;
   $finish;
end 

assign  i_flush       = '0;
assign  i_wrt_en      = o_iq_wrt_en;
assign  i_wrt_data    = o_iq_inst;
assign  i_wrt_inst_pc = o_iq_pc;
assign  i_wrt_taken   = o_iq_taken_pre;
assign  i_iq_full     = o_full;

//================================
// DUT Instantiation 
//================================
INSTRUCTION_FETCH_UNIT2
#(.PC_BOOT         (PC_BOOT       ),
  .ROBSIZE         (ROBSIZE       ),
  .VASIZE          (VASIZE        ),
  .PASIZE          (PASIZE        ),
  .PAGEOFFSETSIZE  (PAGEOFFSETSIZE),
  .L1OFFSETSIZE    (L1OFFSETSIZE  ),
  .L1INDEXSIZE     (L1INDEXSIZE   ),
  .L1WAYSIZE       (L1WAYSIZE     ),
  .L2INDEXSIZE     (L2INDEXSIZE   ),
  .L2WAYSIZE       (L2WAYSIZE     ),
  .TLBINDEXSIZE    (TLBINDEXSIZE  ),
  .PTENTRYSIZE     (PTENTRYSIZE   ) )
uINSTRUCTION_FETCH_UNIT2
(.*);

//================================
// Instruction Queue Instantiation 
//================================
 INSTRUCTION_QUEUE 
 #(.DATAWITHBIT (DATAWITHBIT), .FIFOSIZE (FIFOSIZE))
 uINSTRUCTION_QUEUE 
 ( .*);


endmodule :INSTRUCTION_FETCH_UNIT2_tb