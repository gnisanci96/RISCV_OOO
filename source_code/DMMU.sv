`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 
//================================
// L2 COMMAND DEFINITIONS 
//================================
`define  L2_CACHE_HIT_CHECK_CMD  3'b000
`define  L2_LOAD_CMD             3'b001
`define  L2_STORE_CMD            3'b010
`define  L2_BURST_READ_CMD       3'b011
`define  L2_BURST_WRITE_CMD      3'b100
`define  L2_UPDATE_INCLUSION_CMD 3'b101

module DMMU 
#(parameter ROBSIZE         = 8,
  parameter VASIZE          = 32,
  parameter PASIZE          = 12,
  parameter PAGEOFFSETSIZE  = 6,
  parameter L1OFFSETSIZE    = 4,
  parameter L1INDEXSIZE     = 2,
  parameter L1WAYSIZE       = 2,
  parameter L2INDEXSIZE     = 2,
  parameter L2WAYSIZE       = 4,
  parameter TLBINDEXSIZE    = 6,
  parameter PTENTRYSIZE     = 8
  ) 
(
   input logic                          clk,
   input logic                          rstn, 
   input logic                          i_cs,                 // CHIP SELECT
   input logic [(VASIZE-1):0]           i_addr,               // Memory Address 
   input logic [(ROBSIZE-1):0]          i_rob_addr,           // ROB Addr
   input logic                          i_load_store,         // LOAD OR STORE COMMAND 
   input logic [31:0]                   i_store_data,         // Store Data 
   input logic [2:0]                    i_funct3,               // function 3 
   // 
   input  logic                         i_broadcast_en,       // Broadcast Enable 
   output logic                         o_broadcast_ready,    // Broadcast Data Ready 
   output logic [(ROBSIZE-1):0]         o_broadcast_rob_addr, // Broadcast ROB Addr 
   output logic [31:0]                  o_broadcast_load_data,// Broadcast Data 
   //
   output logic                         o_mem_busy,            // Memory Busy Flag
   output logic                         o_PAGE_FAULT 
    );
 

//         
localparam L1TAGSIZE        = (PASIZE-L1OFFSETSIZE-L1INDEXSIZE);
localparam L2OFFSETSIZE     =  L1OFFSETSIZE;
localparam L2TAGSIZE        = (PASIZE-L2OFFSETSIZE-L2INDEXSIZE); 
localparam L2ADDRSIZE       =  PASIZE;
localparam VAPAGENUMBERSIZE = (VASIZE - PAGEOFFSETSIZE);
localparam TLBTAGSIZE       = (VAPAGENUMBERSIZE - TLBINDEXSIZE);
localparam TLBDATASIZE      = (PASIZE - PAGEOFFSETSIZE);
localparam VPNSIZE          = (VASIZE - PAGEOFFSETSIZE);
localparam PPNSIZE          = (PASIZE - PAGEOFFSETSIZE);


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
logic [(2**L1WAYSIZE-1):0][(L1TAGSIZE-1):0] l1_rd_set_tags_o; 
logic [(2**L1WAYSIZE-1):0]                  l1_rd_set_valid_bits_o; 
logic [(2**L1WAYSIZE-1):0]                  l1_rd_set_dirty_bits_o;
logic [($clog2(L1WAYSIZE)):0]               l1_plru_line_num_o;
// CMD1: Line SIGLE-DATA Read   
logic [31:0]                                l1_rd_line_data_o;        // Data  
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
    TLB_MISS,
    READ_PAGE_TABLE_CMD,
    WAIT_READ_PAGE_TABLE_CMD,
    WRITE_TLB_CMD,
    WRITE_PAGE_TABLE_CMD,
    CHECK_TLB_L1_CMD,
    //
    CHECK_L1_TLB_HIT,
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
//=======================END OF SIGNAL DEFINITIONS===============================    
    


//======================
// STATE DIAGRAM 
//======================
always_comb
begin
   o_PAGE_FAULT = 1'b0;
   //
   loop_break = 1'b0;
   nstate = cstate;
   //
   va_next                    = va_reg;
   virtual_page_number        = va_reg[(VASIZE-1):PAGEOFFSETSIZE];
   tlb_index                  = virtual_page_number[(TLBINDEXSIZE-1):0];
   tlb_tag                    = virtual_page_number[(VAPAGENUMBERSIZE-1):TLBINDEXSIZE];
   i_rob_addr_next            = i_rob_addr_reg;
   i_load_store_next          = i_load_store_reg;    
   i_store_data_next          = i_store_data_reg;    
   i_funct3_next              = i_funct3_reg;
   l1_hit_or_lru_line_o_next  = l1_hit_or_lru_line_o_reg;
   l1_write_back_next         = l1_write_back_reg; 
   o_broadcast_ready_next     = o_broadcast_ready;     
   o_broadcast_rob_addr_next  = o_broadcast_rob_addr; 
   o_broadcast_load_data_next = o_broadcast_load_data;
   PA_next                    = PA_reg;
   ppa_count_next             = ppa_count_reg;
   tlb_ppn_in_next            = tlb_ppn_in_reg;
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
   mem_cs_i           = '0;
   mem_write_read_i   = '0;
   mem_addr_i         = '0;
   mem_write_data_i   = '0;
   mem_write_data_v_i = '0;
   //
   o_mem_busy = 1'b1;
   //
   case(cstate)
      IDLE: 
       begin
           if(i_cs)
           begin
              o_mem_busy = 1'b1;
              // Sample the inputs 
              va_next               = i_addr;
              //
              page_offset           = i_addr[(PAGEOFFSETSIZE-1):0];
              virtual_page_number   = i_addr[(VASIZE-1):PAGEOFFSETSIZE];
              tlb_index             = virtual_page_number[(TLBINDEXSIZE-1):0];
              //
              i_rob_addr_next       = i_rob_addr;
              i_load_store_next     = i_load_store;    
              i_store_data_next     = i_store_data;    
              i_funct3_next         = i_funct3;    
              //============================
              // Set L1 signals to Read the Tags,Valid bits, Dirty Bits for the set  
              //============================
              l1_cs_i           = 1'b1;
              l1_cmd_i          = 3'd0; 
              l1_addr_i         = i_addr; 
              //============================
              // SET TLB Signals to Read the Physical Address
              //============================
              tlb_cs_i         = 1'b1;     
              tlb_write_read_i = 1'b0;
              tlb_vpn_i        = virtual_page_number;      
              tlb_ppn_i        = '0;       
                                                                        nstate = CHECK_L1_TLB_HIT;
           end  else begin
              o_mem_busy = 1'b0;
           end 
           
       end 
      CHECK_L1_TLB_HIT: 
      begin
         l1_write_back_next = 1'b0;  

      if(tlb_hit_o)
      begin
      
          PA_next = {tlb_ppn_o,va_reg[(PAGEOFFSETSIZE-1):0]};
         //================================================================
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
            // L1 HIT 
            if(i_load_store_reg)
            begin
                                                                        nstate = L1_SINGLE_DATA_WRITE_CMD;
            end else begin
                                                                        nstate = L1_SINGLE_DATA_READ_CMD;
            end
            // 
         end else begin 
            // L1 MISS
            l1_hit_or_lru_line_o_next    = l1_plru_line_num_o;
            l1_hit_or_victim_line_v_next = l1_rd_set_valid_bits_o[l1_plru_line_num_o];
            l1_hit_or_victim_line_d_next = l1_rd_set_dirty_bits_o[l1_plru_line_num_o];
            l1_write_back_next           = l1_rd_set_valid_bits_o[l1_plru_line_num_o]  & l1_rd_set_dirty_bits_o[l1_plru_line_num_o];
            l1_victim_line_addr_next     = {l1_rd_set_tags_o[l1_plru_line_num_o],va_reg[(L1INDEXSIZE+L1OFFSETSIZE-1):0]};
                                                                                    nstate = L2_CHECK_HIT_CMD_S0;
         end 
         //
      end else begin
         // TLB MISS   
                                                                                     nstate = READ_PAGE_TABLE_CMD;

      end  
      end 
      READ_PAGE_TABLE_CMD:
      begin
        pt_cs_i         = 1'b1;
        pt_write_read_i = 1'b0;
        pt_VPN_i        = virtual_page_number;
         
                                                                                     nstate = WAIT_READ_PAGE_TABLE_CMD;
      end 
      WAIT_READ_PAGE_TABLE_CMD:
      begin
         if(pt_output_v_o) begin
            //
            if(pt_page_fault_o)
            begin
                                                                                     nstate = WRITE_PAGE_TABLE_CMD;   
            end else begin
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
      
                                                                                     nstate = WRITE_TLB_CMD; 
      end 
      WRITE_TLB_CMD:
      begin
         tlb_cs_i         = 1'b1;       
         tlb_write_read_i = 1'b1;
         tlb_vpn_i        = virtual_page_number;    
         tlb_ppn_i        = tlb_ppn_in_reg; 
                                                                                     nstate = CHECK_TLB_L1_CMD; 
      end 
      CHECK_TLB_L1_CMD:
      begin 
          //============================
		  // Set L1 signals to Read the Tags,Valid bits, Dirty Bits for the Set  
          //============================
          l1_cs_i           = 1'b1;
          l1_cmd_i          = 3'd0; 
          l1_addr_i         = va_reg; 
          //============================
          // SET TLB Signals to Read the Physical Address
          //============================
          tlb_cs_i         = 1'b1;     
          tlb_write_read_i = 1'b0;
          tlb_vpn_i        = virtual_page_number;      
          tlb_ppn_i        = '0;     
                                                                                     nstate = CHECK_L1_TLB_HIT;  
      end 
      //===================================================
      L1_SINGLE_DATA_READ_CMD:
      begin
         l1_cs_i           = 1'b1;
         l1_cmd_i          = 'd1;
         l1_addr_i         = PA_reg;         
         l1_line_num_i     = l1_hit_or_lru_line_o_reg; 
         l1_funct3_i       = i_funct3_reg;              
                                                                        nstate = WAIT_L1_SINGLE_DATA_READ_CMD;
      end 
      WAIT_L1_SINGLE_DATA_READ_CMD:
      begin
         if(l1_cmd_done_o)
         begin
            o_broadcast_ready_next     = 1'b1;     
            o_broadcast_rob_addr_next  = i_rob_addr_reg; 
            o_broadcast_load_data_next = l1_rd_line_data_o;
                                                                        nstate = L1_BROADCAST;
         end 
      end 
      L1_SINGLE_DATA_WRITE_CMD:
      begin
         l1_cs_i            = 1'b1;
         l1_cmd_i           = 'd2; 
         l1_addr_i          = PA_reg;          
         l1_line_num_i      = l1_hit_or_lru_line_o_reg;             
         l1_funct3_i        = i_funct3_reg;           
         l1_wrt_line_data_i = i_store_data_reg;      
                                                                        nstate = WAIT_L1_SINGLE_DATA_WRITE_CMD;
      end 
      WAIT_L1_SINGLE_DATA_WRITE_CMD:
      begin
         if(l1_cmd_done_o)
         begin
                                                                        nstate = IDLE;
         end 
      end 
      L1_BROADCAST:
      begin
         if(i_broadcast_en)
         begin
            o_broadcast_ready_next     = '0;     
            o_broadcast_rob_addr_next  = '0; 
            o_broadcast_load_data_next = '0;
                                                                        nstate = IDLE;
         end
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
         l2_wrt_line_data_i = l1_rd_line_data_o;
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
         // L1 HIT 
         if(i_load_store_reg)
         begin
                                                                        nstate = L1_SINGLE_DATA_WRITE_CMD;
         end else begin
                                                                        nstate = L1_SINGLE_DATA_READ_CMD;
         end
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
      va_reg                       <= '0;
      i_rob_addr_reg               <= '0;
      i_load_store_reg             <= '0;    
      i_store_data_reg             <= '0;    
      i_funct3_reg                 <= '0;
      o_broadcast_ready            <= '0; 
      o_broadcast_rob_addr         <= '0;
      o_broadcast_load_data        <= '0;
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
      end else begin
      cstate                       <= nstate; 
      va_reg                       <= va_next;
      i_rob_addr_reg               <= i_rob_addr_next;
      i_load_store_reg             <= i_load_store_next;    
      i_store_data_reg             <= i_store_data_next;    
      i_funct3_reg                 <= i_funct3_next;   
      //
      o_broadcast_ready            <= o_broadcast_ready_next;     
      o_broadcast_rob_addr         <= o_broadcast_rob_addr_next; 
      o_broadcast_load_data        <= o_broadcast_load_data_next;
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
   end 
end 






//========================================
// L1-Instantiation 
//========================================
L1_CACHE
#(.WAYSIZE    (L1WAYSIZE),             
  .OFFSETBIT  (L1OFFSETSIZE),
  .INDEXBIT   (L1INDEXSIZE), 
  .TAGBIT     (L1TAGSIZE) 
  )
uL1_CACHE
(
.clk                     (clk),
.rstn                    (rstn),
//
.i_l1_cs                 (l1_cs_i),
.i_l1_cmd                (l1_cmd_i),
.o_l1_cmd_done           (l1_cmd_done_o), 
//
.i_l1_addr               (l1_addr_i),         
.i_l1_line_num           (l1_line_num_i),          
.i_l1_funct3             (l1_funct3_i),                       
//
.o_l1_rd_set_tags        (l1_rd_set_tags_o), 
.o_l1_rd_set_valid_bits  (l1_rd_set_valid_bits_o),
.o_l1_rd_set_dirty_bits  (l1_rd_set_dirty_bits_o),
.o_l1_plru_line_num      (l1_plru_line_num_o),
//
.o_l1_rd_line_data       (l1_rd_line_data_o),        
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



    
    
endmodule :DMMU



module DMMU_tb();
    
 //
parameter ROBSIZE         = 8;
parameter VASIZE          = 32;
parameter PASIZE          = 12;
parameter L1OFFSETSIZE    = 4;
parameter L1INDEXSIZE     = 2;
parameter PAGEOFFSETSIZE  = (L1INDEXSIZE + L1OFFSETSIZE);
parameter L1WAYSIZE       = 2;
parameter L2INDEXSIZE     = 2;
parameter L2WAYSIZE       = 3;
parameter TLBINDEXSIZE    = 6;
parameter PTENTRYSIZE     = 8;
parameter PTVPNSIZE       = 23; 
parameter PTPPNSIZE       = 11;
//
localparam L1TAGSIZE        = (VASIZE-L1OFFSETSIZE-L1INDEXSIZE);
localparam L2TAGSIZE        = (PASIZE-L1OFFSETSIZE-L2INDEXSIZE);
localparam VAPAGENUMBERSIZE = (VASIZE - PAGEOFFSETSIZE);
localparam TLBTAGSIZE       = (VAPAGENUMBERSIZE - TLBINDEXSIZE);
localparam TLBDATASIZE      = (PASIZE - PAGEOFFSETSIZE);



//
logic [(PASIZE-1):0]       t_pa;
//
logic [(L1TAGSIZE-1):0]    t_l1tag;
logic [(L1INDEXSIZE-1):0]  t_l1index;
logic [(L1OFFSETSIZE-1):0] t_l1offset;
//
logic [(L2TAGSIZE-1):0]    t_l2tag;
logic [(L2INDEXSIZE-1):0]  t_l2index;
logic [(L1OFFSETSIZE-1):0] t_l2offset;
//
logic [(TLBINDEXSIZE-1):0] t_tlbindex;
logic [(TLBTAGSIZE-1):0]   t_tlbtag;
//   
//
logic                            clk;                  // Clock
logic                            rstn;                 // rstn
logic                            i_cs;                 // CHIP SELECT
logic [(VASIZE-1):0]             i_addr;               // Memory Address 
logic                            o_l1_cmd_done;
logic [(ROBSIZE-1):0]            i_rob_addr;           // ROB Addr
logic                            i_load_store;         // LOAD OR STORE COMMAND 
logic [31:0]                     i_store_data;         // Store Data 
logic [2:0]                      i_funct3;             // function 3 
//
logic                            i_broadcast_en;        // Broadcast Enable 
logic                            o_broadcast_ready;     // Broadcast Data Ready 
logic [(ROBSIZE-1):0]            o_broadcast_rob_addr;  // Broadcast ROB Addr 
logic [31:0]                     o_broadcast_load_data; // Broadcast Data 
logic                            o_PAGE_FAULT;
//
logic                            o_mem_busy;            // Memory Busy Flag 
int error;
//
logic [255:0][(VASIZE-1):0]     test_addr;
int cnt;





task INITIALIZEL1();
begin
/* // CACHE MEMORY
 // cache_mem[set][line][line_row][line_column]
 //            SET            LINE            LINE_ROW       LINE_COLMN    
 logic   [(INDEXSIZE-1):0][(WAYSIZE-1):0][((OFFSETSIZE-2)-1):0][31:0]  T_CACHE_DATA; 
 //            SET            LINE
 logic   [(INDEXSIZE-1):0][(WAYSIZE-1):0]              T_LINE_VALID; 
 logic   [(INDEXSIZE-1):0][(WAYSIZE-1):0]              T_LINE_DIRTY; 
 //            SET            LINE           TAG                                                       
 logic [(INDEXSIZE-1):0][(WAYSIZE-1):0][(TAGSIZE-1):0] T_LINE_TAG; */
 for(int set=0;set<(2**L1INDEXSIZE);set++)
 begin
    for(int line=0;line<(2**L1WAYSIZE);line++)
    begin 
       DUT.uL1_CACHE.T_LINE_VALID[set][line] <= $urandom%2;
       DUT.uL1_CACHE.T_LINE_DIRTY[set][line] <= $urandom%2;
       DUT.uL1_CACHE.T_LINE_TAG[set][line]   <= $urandom;
       //
       for(int line_row=0;line_row<(2**(L1OFFSETSIZE-2));line_row++)
       begin  
          DUT.uL1_CACHE.T_CACHE_DATA[set][line][line_row] <= $urandom;
       end
       //
    end  
 end 
end
endtask



task INITIALIZETLB();
begin
    for(int entry=0;entry<(2**TLBINDEXSIZE);entry++)
    begin 
       DUT.uTLB.T_VPN[entry]   <= $urandom;
       DUT.uTLB.T_PPN[entry]   <= entry;
       DUT.uTLB.T_VALID[entry] <= 1'b1;
    end 
 /////==========/////////////=============/////   
end 
endtask  

task INITIALIZEMEM();
begin
    cnt = 1;
    for(int entry=0;entry<256;entry++)
    begin 
       for(int word=0;word<(2**L1OFFSETSIZE)/4;word++)
       begin 
          DUT.uEXTERNAL_MEMORY.MEM[entry][word] <= cnt;
          cnt = cnt + 1;
       end   
    end 
end 
endtask 


//==============================
// CLOCK GENERATION 
//==============================

initial begin
 clk = 1'b0; 
 fork
  forever #10 clk = ~clk;
 join
end 
 
//==============================
// RESET TASK 
//==============================
task RESET();
begin
   error = '0;
   i_cs  = '0;
   // 
   rstn = 1'b1;
   @(posedge clk);
   rstn = 1'b0;
   repeat(2)@(posedge clk);
   rstn = 1'b1;
end 
endtask


//==============================
// STORE TASK 
//==============================
task STORE(
   input logic [(VASIZE-1):0]              addr,
   input logic [(ROBSIZE-1):0]             rob_addr,
   input logic [31:0]                      store_data,
   input logic [2:0]                       funct3,
   input logic                             signed_unsigned
);
begin


//
t_tlbindex = addr[(L1INDEXSIZE+L1OFFSETSIZE+TLBINDEXSIZE-1):(L1INDEXSIZE+L1OFFSETSIZE)];
t_tlbtag   = addr[(VASIZE-1):(L1INDEXSIZE+L1OFFSETSIZE+TLBINDEXSIZE)];
//
if(t_tlbtag == DUT.uTLB.T_VPN[t_tlbindex])
begin
   $display("----------------------------------------------------");
   $display("!!TLB HIT!!");
   t_pa = {DUT.uTLB.T_PPN[t_tlbindex],addr[(L1INDEXSIZE+L1OFFSETSIZE-1) :0]};
   $display("PHYSICAL ADDR: %h",t_pa);
   t_l1offset = t_pa[(L1OFFSETSIZE-1):0];
   t_l1index  = t_pa[(L1OFFSETSIZE+L1INDEXSIZE-1):L1OFFSETSIZE];
   t_l1tag    = t_pa[(PASIZE-1):(L1OFFSETSIZE+L1INDEXSIZE)];
   //
   t_l2offset = t_pa[(L1OFFSETSIZE-1):0];
   t_l2index  = t_pa[(L1OFFSETSIZE+L2INDEXSIZE-1):L1OFFSETSIZE];
   t_l2tag    = t_pa[(PASIZE-1):(L1OFFSETSIZE+L2INDEXSIZE)];
   //
   $display("L1 OFFSET: %h",t_l1offset);
   $display("L1 INDEX:  %h",t_l1index);
   $display("L1 TAG:    %h",t_l1tag);
   //
   $display("L2 OFFSET: %h",t_l2offset);
   $display("L2 INDEX:  %h",t_l2index);
   $display("L2 TAG:    %h",t_l2tag);
   //
   $display("DATA: %h",store_data);
   //
end else begin
   $display("!!!PAGE FAULT!!!", );
   $display("--t_tlbtag : %h ",t_tlbtag);
   $display("--DUT.uTLB.T_TAG[t_tlbindex]:%h t_tlbindex:%d",DUT.uTLB.T_VPN[t_tlbindex],t_tlbindex);
end 

        wait(~o_mem_busy)
        @(posedge clk);
          i_cs              <= 1'b1;
          i_addr            <= addr;
          i_rob_addr        <= rob_addr;
          i_load_store      <= 1'b1;
          i_store_data      <= store_data;
          i_funct3          <= funct3;
        @(posedge clk);          
          i_cs              <= '0;
          i_addr            <= '0;
          i_rob_addr        <= '0;
          i_load_store      <= '0;
          i_store_data      <= '0;
          i_funct3          <= funct3;
end 
endtask




//==============================
// LOAD TASK 
//==============================
task LOAD(
   input logic [(VASIZE-1):0]              addr,
   input logic [31:0]                      ex_data,
   input logic [(ROBSIZE-1):0]             rob_addr,
   input logic [2:0]                       funct3,
   input logic                             signed_unsigned
);
begin 
        wait(~o_mem_busy)
        @(posedge clk);
          i_cs              <= 1'b1;
          i_addr            <= addr;
          i_rob_addr        <= rob_addr;
          i_load_store      <= 1'b0;
          i_store_data      <= '0;
          i_funct3          <= funct3;
        @(posedge clk);          
          i_cs              <= '0;
          i_addr            <= '0;
          i_rob_addr        <= '0;
          i_load_store      <= '0;
          i_store_data      <= '0;
          i_funct3          <= '0;
        wait(o_broadcast_ready)
        @(posedge clk);  
        if(o_broadcast_load_data != ex_data) error++;
        i_broadcast_en = 1'b1;
        @(posedge clk);
        i_broadcast_en = 1'b0;  
end 
endtask





//=======================================================
//                      TEST 1
//=======================================================
task test1();
begin
RESET();
INITIALIZETLB();
INITIALIZEMEM();
#20
//
STORE(
   .addr             ({DUT.uTLB.T_VPN[5],6'b000101,3'b111,6'b001100}),    // --> Virtual Address 
   .rob_addr         ('d5),
   .store_data       ('hdeadbeef),
   .funct3           (3'b010),
   .signed_unsigned  (1'b0)
);
//
LOAD(
   .addr            ({DUT.uTLB.T_VPN[5],6'b000101,3'b111,6'b001100}),
   .ex_data         ('hdeadbeef),
   .rob_addr        ('d5),
   .funct3          (3'b010),
   .signed_unsigned (1'b0)
);
//

if(error==0) begin $display("---TEST 1 PASS---  ");             end 
else         begin $display("---TEST 1 FAIL--- %d Errors ",error);  end 

end 
endtask 
logic [(TLBINDEXSIZE-1):0] t_tlb_index;
logic [7:0] store_data;


//=======================================================
//                      TEST 2
//=======================================================
task test2(input logic [(L1OFFSETSIZE-1):0] offset,
           input logic [(L1INDEXSIZE-1):0]  index,
           input int                        read_write_num = 4, 
           input int                        test_num = 0);
begin
RESET();
INITIALIZETLB();
INITIALIZEMEM();



#20
//
   for(int line =0; line<read_write_num;line++)
   begin
      t_tlb_index = line;
      store_data =  line;
      $display("TLB TAG : %h Index:%d",DUT.uTLB.T_VPN[line],t_tlb_index);
      STORE(
         .addr             ({DUT.uTLB.T_VPN[line],t_tlb_index,index,offset}),    // --> Virtual Address 
         .rob_addr         ('d5),
         .store_data       ({24'hdeadbe,store_data}),
         .funct3           (3'b010),
         .signed_unsigned  (1'b0)
       );
   end 


   for(int line =0; line<read_write_num;line++)
   begin
      t_tlb_index = line;
      store_data =  line;
      //
      LOAD(
         .addr             ({DUT.uTLB.T_VPN[line],t_tlb_index,index,offset}),
         .ex_data          ({24'hdeadbe,store_data}),
         .rob_addr         ('d5),
          .funct3          (3'b010),
          .signed_unsigned (1'b0)
       );
       //
    end 




if(error==0) begin $display("---TEST %d PASS---  ",test_num);             end 
else         begin $display("---TEST %d FAIL--- %d Errors ",test_num,error);  end 

end 
endtask 

//=======================================================
//                      TEST 3
//=======================================================  
task test3(input int                        read_write_num = 4, 
           input int                        test_num = 0);
begin
RESET();
INITIALIZEMEM();



#20
//
      test_addr[0]  = {20'h11111,6'b000101,2'b11,4'b0011};
      store_data = 0;
      STORE(
         .addr             (test_addr[0]),    // --> Virtual Address 
         .rob_addr         ('d5),
         .store_data       ({24'hdeadbe,store_data}),
         .funct3           (3'b010),
         .signed_unsigned  (1'b0)
       );
     
      LOAD(
         .addr             (test_addr[0]),
         .ex_data          ({24'hdeadbe,store_data}),
         .rob_addr         ('d5),
          .funct3          (3'b010),
          .signed_unsigned (1'b0)
       );
///////////////////////////////////////////////////////////////
      test_addr[1]  = {20'h2222,6'b000111,2'b11,4'b0011};
      store_data = 1;
      STORE(
         .addr             (test_addr[1]),    // --> Virtual Address 
         .rob_addr         ('d5),
         .store_data       ({24'hdeadbe,store_data}),
         .funct3           (3'b010),
         .signed_unsigned  (1'b0)
       );
      // 
      LOAD(
         .addr             (test_addr[1]),
         .ex_data          ({24'hdeadbe,store_data}),
         .rob_addr         ('d5),
          .funct3          (3'b010),
          .signed_unsigned (1'b0)
       );
///////////////////////////////////////////////////////////////
      test_addr[2]  = {20'h33333,6'b001000,2'b11,4'b0011};
      store_data = 2;
      STORE(
         .addr             (test_addr[2]),    // --> Virtual Address 
         .rob_addr         ('d5),
         .store_data       ({24'hdeadbe,store_data}),
         .funct3           (3'b010),
         .signed_unsigned  (1'b0)
       );
      // 
      LOAD(
         .addr             (test_addr[2]),
         .ex_data          ({24'hdeadbe,store_data}),
         .rob_addr         ('d5),
          .funct3          (3'b010),
          .signed_unsigned (1'b0)
       );




if(error==0) begin $display("---TEST %d PASS---  ",test_num);             end 
else         begin $display("---TEST %d FAIL--- %d Errors ",test_num,error);  end 

end 
endtask 






    
//=======================================================
//                      TEST 4
//=======================================================  
task test4(input int read_write_num = 4, 
           input int test_num = 0);
begin
RESET();
INITIALIZEMEM();
//
#20
//
   for(int line =0; line<read_write_num;line++)
   begin
      test_addr[line]  = $urandom;
      store_data = line;
      STORE(
         .addr             (test_addr[line]),    // --> Virtual Address 
         .rob_addr         ('d5),
         .store_data       ({24'hdeadbe,store_data}),
         .funct3           (3'b010),
         .signed_unsigned  (1'b0)
       );
   end 


   for(int line =0; line<read_write_num;line++)
   begin
      store_data = line;
      LOAD(
         .addr             (test_addr[line]),
         .ex_data          ({24'hdeadbe,store_data}),
         .rob_addr         ('d5),
          .funct3          (3'b010),
          .signed_unsigned (1'b0)
       );
    end 




if(error==0) begin $display("---TEST %d PASS---  ",test_num);             end 
else         begin $display("---TEST %d FAIL--- %d Errors ",test_num,error);  end 

end 
endtask   
  
  
  
  
  
  
    
 initial begin
 $display("START");
   RESET();
   /* STORE(
      .addr             ({20'h12345,6'b000101,2'b11,4'b1100}),    // --> Virtual Address 
      .rob_addr         ('d5),
      .store_data       ('hdeadbeef),
      .funct3           (3'b010),
      .signed_unsigned  (1'b0)
    );
    */
 //test2(.offset ('h0),.index  ('h0));
   test4(.read_write_num (56) );
 $finish;
 end    
    
    
 DMMU
#(.ROBSIZE         (ROBSIZE),
  .VASIZE          (VASIZE),
  .PASIZE          (PASIZE),
  .PAGEOFFSETSIZE  (PAGEOFFSETSIZE),
  .L1OFFSETSIZE    (L1OFFSETSIZE),
  .L1INDEXSIZE     (L1INDEXSIZE),
  .L1WAYSIZE       (L1WAYSIZE),
  .L2INDEXSIZE     (L2INDEXSIZE),
  .L2WAYSIZE       (L2WAYSIZE),
  .TLBINDEXSIZE    (TLBINDEXSIZE),
  .PTENTRYSIZE     (PTENTRYSIZE))
DUT
(.*);   
    
    
endmodule :DMMU_tb






