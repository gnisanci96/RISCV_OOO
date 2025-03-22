# RISCV32IM
This repository contains the design and source code for an Out-of-Order (OOO) RISC-V implementation.

# I-Instructions 
<figure>
  <img src="/figures/rv32i_base_insts.png" alt="Description" width="500"/>
  <figcaption>RISC-V 32-bit I-instructions </figcaption>
</figure>


## LUI (Load Upper Immediate)
This instruction is used to build 32-bit contants.
The LUI places the 20-bit immediate value in the top 20-bit of the destination register rd, filling the lowest 12-bits with zeros.
## AUIPC (add upper immediate to pc)
It is used to build pc-relative addresses.
The AUIPC forms a 32-bit offset from the 20-bit immediate, filling the lowest bits with zeros.
It adds the offset to the PC of the AUIPC instruction and adds the result to the destination register rd.
## JAL (Jump and Link)
Unconditional Jump instruction.
Immediate value is sign-extended and added to the address of the JAL instruction to form the target address.
JAL stores the pc+4 into the register rd.
x1 is the standard return address register and x5 is the alternate link register.
## JALR (Indirect Jump instruction)
Unconditional Jump instruction.
Target address is calculated by adding the sign-extended 12-bit immediate to the register rs1 and then setting the least-significant bit of the result to zero.
The address of the instruction following the jump pc+4 is written to the register rd.
## Conditional Branches (BEQ, BNE, BLT, BLTU, BGE, BGEU)
- These instructions compare two registers and jump or not depending on the result.
- BEQ and BNE take the branch if registers rs1 and rs2 are equal or unequal respectively.
- BLT and BLTU take the branch if rs1 is less than rs2, using signed and unsigned comparison respectively.
- BGE and BGEU take the branch if rs1 is greater than or equal to rs2, using signed and unsigned comparison respectively.
- Load and Store Instructions
- RISC-V is load-store architecture, where only load and store instructions access memory and arithmetic instructions only operate on CPU registers.
- Loads copy a value from memory to register rd.
- Stores copy the value in register rs2 to memory.
- The effective address of the loads are obtained by adding register rs1 to the sign-extended 12-bit offset.
- The LW instruction loads a 32-bit value from memory into rd.
- LH loads a 16-bit value from memory, then sign-extends to 32-bits before storing in rd.
- LHU loads a 16-bit value from memory but then zero extends to 32-bits before storing in rd.
- LB and LBU are defined analogously for 8-bit values.
## Fence
- Fence instruction is used to order device I/O and memory accesses.
- THIS processor orders the memory load and store instructions strictly so the memory accesses are IN-ORDER.
## ECALL
The ECALL instruction is used to make a service request to the execution environment.
EBREAK
EBREAK instruction is used to return the control to the debugging environment.
# M-Instructions
## MUL
MUL performs an 32-bit x 32-bit multiplication of rs1 and rs2 and places the LOWER 32-bit of the 64-bit result to the destination register.
## MULH
MULH performs an 32-bit(signed)x32-bit(signed) multiplication of rs1 and rs2 and places the HIGHER 32-bit of the 64-bit result to the destination register.
## MULHSU
MULHSU performs an 32-bit(unsigned)x32-bit(unsigned) multiplication of rs1 and rs2 and places the HIGHER 32-bit of the 64-bit result to the destination register.
## MULHU
MULHSU performs an 32-bit(signed)x32-bit(unsigned) multiplication of rs1 and rs2 and places the HIGHER 32-bit of the 64-bit result to the destination register.
## DIV, DIVU
DIV and DIVU perform an XLEN bits by XLEN bits signed and unsigned integer division of rs1 by rs2, rounding towards zero.
## REM, REMU
REM and REMU provide the remainder of the corresponding division operation.
For REM, the sign of the result equals the sign of the dividend. -For both signed and unsigned division, it holds that dividend = divisor × quotient + remainder.
# Instruction Fetch Stage
- Instruction Fetch unit is responsible from converting virtual address to physical address, reading instructions from external memory and writing them into the L1,L2 cache, updating the TLB table in case of a page fault, and perform branch prediction using Branch Target Buffer(BTB) and Branch History Table(BHT).
- The design uses 32-bit virtual address.
- The design uses Virtually-Indexed-Physically-Tagged (VIPT) Cache. The benefit of VIPT cache is that we can read the tag and data of the L1 cache while reading the physical page number from the TLB table. If we use Physically indexed physically Tagged(PIPT) Cache, L1 cache would need to wait TLB read first and then start reading the tag and data.
- After reading the TLB and L1 Tag, we can compare the physical TAG and L1 Tag.
- If Both L1 and L2 have hit, take the instructions from L1 cache and send them into the pipeline, where they will go to instruction queue eventually with their predicted branch target address and predicted taken/not-taken result.
- If there is no TLB tag that matches the TLB tag of the virtual address, Instruction Fetch Unit generates PAGE FAULT signal and stops instruction fetch until the PAGE FAULT is resolved.
- The pysical memory is divided into 2^(physical page number) pages and virtual address is divided into 2^(Virtual Page Number) pages. Since the number of virtual pages are more than the number of physical pages, there needs to be a mapping between virtual and physical pages, which is done using TLB and page table. Each page has 2^(Page offset) bytes.
- The PAGE FAULT is handled as follows;
   - Check if the PAGE TABLE has the Virtual to Physical address conversion.
   - If the Page Table doesn't have the conversion, assign the next available Pysical Page number to the Virtual Page Number that caused to the PAGE FAULT.
   - Then update the TLB with the conversion, generate the pysical address, perform the L1 Tag check and continue the fetch process.

<figure>
  <img src="/figures/TLB_L1_L2_addr.png" alt="Description" width="500"/>
</figure>

# BRANCH PREDICTION 
### How do branch prediction and Branch Target Buffer help performance of a cpu? 
- When a branch instruction is fetched, the CPU needs to quickly determine:
- Will the branch be taken or not? (Handled by branch predictors like 2-bit predictors, TAGE, etc.)
- If taken, where does it go? (Handled by the BTB).
- Without a Branch Prediction, the CPU would need to decode the branch instruction and compute the target address, causing a stall. The BTB probabilistically eliminates this delay by caching branch target addresses and taken/not-taken result.

## Branch Prediction Modules 
### Branch History Table (2-bit Predictor) 
- This project implements 2-bit branch predictor for speculative execution of control transfe instructions.
- Why 2-bit predictor ? 
  - A 1-bit predictor only remembers the last outcome of a branch (Taken/Not Taken). This causes problems in loops where the branch is mostly Taken, except for the last iteration (causing a misprediction).
  - A 2-bit predictor helps by allowing the CPU to "hesitate" before changing predictions, reducing mispredictions caused by occasional fluctuations.
- How 2-bit predictor work ?
  - Each branch is associated with a 2-bit saturating counter.
  - The counter is stored in a Branch History Table (BHT), indexed by the branch address (PC).
  - The 2 bits track the branch's recent behavior, allowing a decision that is more stable than a simple 1-bit predictor.
  - The diagram below shows how the 2bit predictor FSM works. 
<figure>
  <img src="/figures/BHT_2bit_predictor_fsm.png" alt="Description" width="500"/>
</figure>
- BHT is designed as a set-associative cache, where the data stored is the 2-bit current FSM value.
- The BHT is addressed using PC of the instruction. 
- If there is a hit taken/not-taken prediction is returned.  
- If there is no-hit, BHT miss is reported to the instruction fetch unit.  

### Branch Target Buffer 
- A Branch Target Buffer (BTB) is a cache-like structure used in modern CPUs to store branch instruction PC and their predicted target addresses. It helps speed up branch prediction by providing the next instruction’s address before the actual branch instruction is executed.
- In this project, BTB is designed as a set-associative cache.

### Branch Prediction Flow 
- Send PC to BHT and BTB and check if there is any prediction or not.
- If there is a taken prediction, start fetching from the address stored in the BTB
- If BHT and BTB doesn't hit or If the prediction is not-taken, jump to (PC + 4).
- If there is a prediction, prediction(target address with taken/not-taken flag) is going to be sent to OOO pipeline through instruction queue with the PC of the instruction. 
- Branch Target Calculator module is going to calculate the target address(all control transfer instructions) and taken/not-taken result(in case of conditional branches).
- BRT module is going to compare the predictions with the calculated values.
- When the ROB Commit pointer points to the Control Transfer Instruction, ROB sends flag to Core Control Unit(CCU) with information about the instruction.
- If there is a Commit Ready signal, CCU checks if the instruction is control transfer instruction
- The CCU checks the output of the BRT, which reports miss-prediction result and calculated target.  
- If there is a Miss-Prediction, CCU performs the following actions;
  - Flush the Pipeline.
  - Let the Front End know that Flush occured, what is the flush_pc (Exception PC) and calculated taken/not-taken result of the control transfer instruction.
- Front end Unit stops fetching and jumps to the Exception PC sent by the CCU.
- BTB and BHT are then updated with the new information. 
  


# Instruction Queue Read / Decode / Renaming Stage
### Step1: Instruction Fetch Stage (All instructions)

### Step2: If Instruction queue is not empty Core_Control_Unit checks if any one of reservation stations, ROB, BTC, JEU, BRT is full. If none of the modues are full, it reads the instruction queue package.
Future Work: Since the instruction isn't pre-decoded, Core_Control_Unit doesn't know anything about the instruction so it checks all types of "instructions buffer". This is in-efficiency and can be resolved by 
adding pre-decode at the front-end of the processor. 

### Step3: Instruction Queue package contains the following (All instructions);
- 32-bit instruction
- 32-bit instruction address
- 32-bit instruction target
- 1-bit instruction taken prediction
### Step4: Instruction Decode (All instructions);

- Decode opcode, rd, funct3, rs1, rs2, funct7
- Check if instruction is M-extension, I-extension, control transfer, legal instruction, generate operation and immidiate type select signals.
- Decode load store size and if the operation is signed or unsigned.

### Step5: 
Immidiate Generation Unit takes the immediate type from the decoder and generates the 32-bit immediate value if the instruction is immediate instruction.
### Step6: Source Register Value Read (All instructions);
- Read the rs1 and rs2 values from the Register File
- Read RAT table to learn if core will take the rs1 and rs2 values from the Register File or ROB.
- Read ROB addresses that will provide the rs1 and rs2 values. If rs1 and rs2 values are in register file, these ROB adresses will be discarded.
- Read the Data and Valid values from the ROB using the ROB addresses provided by the RAT table.
- If the rs1 and rs2 values are in register file, the values are read from the register file and they are ready to be used.
- If the values are in ROB, check if the values are ready or not.
  - If ROB values are ready, read the values from ROB.
  - If values are not ready, the values will be brodcasted with the ROB addresses on the Core Data Bus.

### Step7: Write Instruction to the ROB (All instructions)
- Every instruction is written to the ROB and assigned a unique ROB address, which serves as its identification number throughout its lifecycle.

### Step8: Write instruction to the Reservation Station (For ALU instruction)
- This design uses Distributed reservation station, which means that, every execuion unit has its own local resercation station.
- The Reservation station stores ALU opcode, rs1, rs2 values, their valid signals and ROB address for the instruction and ROB address for the rs1, rs2 values.
- If rs1 or rs2 values of the instruction are not ready, reservation station wait for the value(s) to be brodcasted. So everytime there is a broadcast on the data bus, it compares the ROB address of the brodcasted value with the ROB address of the rs1 and rs2 values stored. If there is a match, it stores the value and sets the valid bit of the matching rs1 or rs2.
- If the rs1 and rs2 values of the instruction are ready and the ALU control unit is not busy, the instruction is sent to the ALU control unit, which will perform the execution. This step is called DISPATCH.
- If there are multiple instructions ready to be dispatched, the reservation station chooses the instruction with the lowest index on the table.
Future WORK: There are multiple algorithms to choose which instruction to send to the execution unit. Here are the 2 of the algorithms; 
   - Oldest-Ready First: Dispatches the oldest instruction that is ready to execute.
   - Critical Path First: Prioritizes instructions that are on the program’s critical path, reducing execution latency. It needs to trach the dependencies to the ROB of the instructions.

### Step9: Execution and Broadcast for ALU instructions; - The executution unit takes N-CC to execute the instruction.
- When the result is ready, it sets the broadcast_ready signal and wait for its turn to broadcast the result.
- Since there can be multiple execution units that wants to broadcast at the same time and the boardcast bus is shared among multiple execution units, core_control_unit uses Least Recently Granted First (LRGF) Arbiter to choose the next excution unit to brodcast its value.
- The Value is broadcasted to all the all the reservation stations, load_store queue and the ROB. The broadcast contains the data, ROB address of the instruction and if the data is a memory address, which will be captured by the load_store queue.

### Step10: Execution and broadcast for LOAD/STORE instructions;

- After Decoding, the LOAD and STORE instructions are written to Load/Store queue.
- The Load Instruction reads an memory address and write the value to a destination register.
- Store instruction reads value of a register and writes it to a memory location.
- Load/Store Queue keeps track of the broadcasted data. The data may be register value for store to be stored to the memory or memory address for store or load.
- Load Store queue sends the load/store instructions to the memory IN-ORDER.
- There is NO DATA FORWARDING between load and store instructions.
- The queue works as a FIFO.
- When the read pointer points to a LOAD instruction, memory is not busy, and address of the load is ready, the load instruction is sent to memory unit for memory read operation immediately.
- When the read pointer points to a store instruction, memory is not busy, store address and data are ready, the load store queue waits ROB commit pointer to point the store instruction. Only in commit, store instruction is sent to memory for memory write operation.
- When memory read operation is complete, Data Memory Management unit (DMMU) sets its broadcast signal and waits grant from the core control unit to get the access to the Core Data Bus for broadcast operation.
### STEP11: Execution for Conditional Branch Instructions:

- Conditional Branch instruction is read from the instruction queue with the taken/not-taken and target prediction of the instruction.
### THE JUMP EXECUTION UNIT.
- The Conditional Branch Instructions(CBI) (BEQ,BNE,BLT,BGE,BLTU,BGEU) are issue to this module to calculate if the branch is taken or not.
- As explained above, CBIs performs comparison operation between RS1 and RS2.   
- The JEU writes the instruction to its reservation table and waits for the rs1 and rs2 values to be ready.
- The JEU waits for the broadcast bus to broadcast rs1 and rs2 values if they are not ready. It uses ROB addreses of the rs1 and rs2 values to check.
- The JEU writes the ROB address of the branch instruction and taken/not-taken result to its FIFO.
- The JEU connects outputs of the FIFO(taken/non-taken bit, ROB addr, fifo empty ) to its own output.
### The Branch Target Calculator(BTC).
- All the control transfer instructions are written to the BTC unit, where the target address of the instructions are calculated.
- The target address of the instuctions are calculated and written to a FIFO with ROB address of the instruction.
### The BRT table.
- The BRT table collects the taken/not-taken and target address information of the control transfer instructions and stores them until commit.
- The BRT talbe reads the taken/not-taken information from the JEU.
- The BRT table reads the target address from the BTC unit.
- BRT table gets the predicted taken/not-taken result for branch instructions and predicted target address for all the branch instructions from instruction queue.
- When ROB points to a control transfer instruction(CTI), Core Control Unit sends commit signal with a ROB address to check if the CTI is ready to be committed.
- When the CTI is ready(target address and taken/not-taken) for the instruction with ROB sent from CCU, CTI responses with the comparison between predicted values and the calculated values.
- If all the predictions are correct, the instructions are committed.
- If at least one of the predictions(target address or taken/not-taken) are wrong, the core control unit sends FLUSH signal to all the units.
- When Branch Instrunctions commit, there is no register write. So brach instructions are ready as soon as they are written to the ROB.
- When JUMP instructions are committed, PC+4 are written to the rd register. So JUMP instructions wait for the PC+4 to be calculated and broadcasted to be ready in ROB.
# The MODULES:
1-) Translation Lookaside Buffer(TLB):

TLB is a fully-associative cache memory that stores the recent virtual to physical memory transitions.
It uses Pseudo-Least Recently Used Algorithm to decide the victim Cache.

# EXECUTION UNIT 

## ALU 


## MUL/DIV Unit 

### MUL

The multiplication algorithm uses Partial-Sum Aproach to calculate the result sequencially. The waveform below shows the internal signals for the 4-bit version of the multiplication. The CPU uses the 32-bit version of the 
same module. Since it uses a single adder, calculation takes 32 Clock Cycle to execute. 

<figure>
  <img src="/figures/mul_10x7.png" alt="Description" width="500"/>
</figure>

### DIV 

The division uses the common registers and addition unit with the multiplication unit. 


The figures below show the division operation (101000/000011) = 001101 with REM=1
<figure>
  <img src="/figures/div_0.png" alt="Description" width="500"/>
</figure>

<figure>
  <img src="/figures/div_1.png" alt="Description" width="500"/>
</figure>

<figure>
  <img src="/figures/div_2.png" alt="Description" width="500"/>
</figure>

Before the unsigned multiplication is performed, algorithm takes the 2's complement of the negative signed numbers and performs unsigned multiplication. 
For example; 
- num1= 1011 (signed) --> -5
- num2= 0111 (signed) -->  7
- 
- Take the 2's complement of the num1 --> 2's(1011) = 0101  --> 5 
- Perform the unsigned multiplication --> 0101x0111 = 0010_0011 --> 35 
- Result needs to be negative number, therefore 2's(0010_0011) = 1101_1101 --> -35


# The Algorithms:

1-) Least Recently Used Algorithm: The algorithm is used as a cache replacement and access grant algorithm in this project. In a 4-way set associative cache, we assign initial number so lines from 0 to 3. The line with number 0 is the least recently accessed line and line with number 3 is the most recently accessed line. So when we have to choose a valid line to replace, we choose the line that is least recently used. Everytime we access a cache line, we update its number to highest available number(3 in this case) and update the number of other lines accordingly. The figure below shows how to update the LRU counters.

<figure>
  <img src="/figures/LRU.png" alt="Description" width="500"/>
</figure>

2-) Pseudo Least Recently Used (PLRU) Algorithm: The LRU algoritm assigns log2(WayNumber) bit counters for each line, which is expensive. So we may instead use Pseudo Least Recently Used algorithm to save power and area.

Each Cache Line uses 1-bit intead of Log2(WayNumber) used in LRU algorithm.
We initialize all the bits to 0.
Each time the line is accessed, we set the bit to one.
If all the bits becomes 1, we leave the last set bit as 1 and convert the other ones to 0.

<figure>
  <img src="/figures/PLRU.png" alt="Description" width="500"/>
</figure>
