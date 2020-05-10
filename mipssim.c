/*************************************************************************************|
|   1. YOU ARE NOT ALLOWED TO SHARE/PUBLISH YOUR CODE (e.g., post on piazza or online)|
|   2. Fill main.c and memory_hierarchy.c files                                       |
|   3. Do not use any other .c files neither alter main.h or parser.h                 |
|   4. Do not include any other library files                                         |
|*************************************************************************************/

#include "mipssim.h"

#define BREAK_POINT 440 // exit after so many cycles -- useful for debugging

// Global variables
char mem_init_path[1000];
char reg_init_path[1000];

uint32_t cache_size = 0;
struct architectural_state arch_state;

//Instruction types definitions completed
#define I_TYPE 2
#define J_TYPE 3

static inline uint8_t get_instruction_type(int opcode)
{
    switch (opcode) {
        /// opcodes are defined in mipssim.h

        case SPECIAL:
            return R_TYPE;
            break;
        case EOP:
            return EOP_TYPE;
            break;

        ///@students: fill in the rest

        //I type instructions
        case LW:
        case SW:
        case BEQ:
        case ADDI:
          return I_TYPE;
          break;

        //J type instructions
        case J:
          return J_TYPE;
          break;

        default: assert(false);
          break;
    }
    assert(false);
}

void FSM()
{
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;

    //reset control signals
    memset(control, 0, (sizeof(struct ctrl_signals)));

    int opcode = IR_meta->opcode;
    int state = arch_state.state;

    switch (state) {
        case INSTR_FETCH:
            control->MemRead = 1;
            control->ALUSrcA = 0;
            control->IorD = 0;
            control->IRWrite = 1;
            control->ALUSrcB = 1;
            control->ALUOp = 0;
            control->PCWrite = 1;
            control->PCSource = 0;
            state = DECODE;
            break;
        case DECODE:
            control->ALUSrcA = 0;
            control->ALUSrcB = 3;
            control->ALUOp = 0;
            if (opcode == SPECIAL) state = EXEC;
            else if (opcode == EOP) state = EXIT_STATE; ///
            else if ((opcode == SW) || (opcode == LW)) state = MEM_ADDR_COMP;
            else if (opcode == BEQ) state = BRANCH_COMPL;
            else if (opcode == J) state = JUMP_COMPL;
            else if (opcode == ADDI) state = I_TYPE_EXEC;
            else assert(false);
            break;
        case MEM_ADDR_COMP:
            control->ALUSrcA = 1;
            control->ALUSrcB = 2;
            control->ALUOp = 0;
            if (opcode == SW) state = MEM_ACCESS_ST;
            else if (opcode == LW) state = MEM_ACCESS_LD;
            else assert(false);
            break;
        case MEM_ACCESS_LD:
            control->MemRead = 1;
            control->IorD = 1;
            state = WB_STEP;
            break;
        case WB_STEP:
            control->RegDst = 0;
            control->RegWrite = 1;
            control->MemtoReg = 1;
            state = INSTR_FETCH;
            break;
        case MEM_ACCESS_ST:
            control->MemWrite = 1;
            control->IorD = 1;
            state = INSTR_FETCH;
            break;
        case EXEC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 0;
            control->ALUOp = 2;
            state = R_TYPE_COMPL;
            break;
        case R_TYPE_COMPL:
            control->RegDst = 1;
            control->RegWrite = 1;
            control->MemtoReg = 0;
            state = INSTR_FETCH;
            break;
        case BRANCH_COMPL:
            control->ALUSrcA = 1;
            control->ALUSrcB = 0;
            control->ALUOp = 1;
            control->PCWriteCond = 1;
            control->PCSource = 1;
            state = INSTR_FETCH;
            break;
        case JUMP_COMPL:
            control->PCWrite = 1;
            control->PCSource = 2;
            state = INSTR_FETCH;
            break;
        case I_TYPE_EXEC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 2;
            control->ALUOp = 0;
            state = I_TYPE_COMPL;
            break;
        case I_TYPE_COMPL:
            control->MemtoReg = 0;
            control->RegWrite = 1;
            control->RegDst = 0 ;
            state = INSTR_FETCH;
            break;
        default:
            assert(false);
            break;
    }

    arch_state.state = state;
}


void instruction_fetch()
{
  //printf("%s\n", "instruction_fetch" );
    if ((arch_state.control.MemRead) && (arch_state.control.IorD == 0) && (arch_state.control.IRWrite)) {
        int address = arch_state.curr_pipe_regs.pc;
        arch_state.next_pipe_regs.IR = memory_read(address);
    }
}

void decode_and_read_RF()
{
  //printf("%s\n", "decode_and_read_RF" );
    int read_register_1 = arch_state.IR_meta.reg_21_25;
    int read_register_2 = arch_state.IR_meta.reg_16_20;
    check_is_valid_reg_id(read_register_1);
    check_is_valid_reg_id(read_register_2);
    arch_state.next_pipe_regs.A = arch_state.registers[read_register_1];
    arch_state.next_pipe_regs.B = arch_state.registers[read_register_2];
}

void execute()
{
  //printf("%s\n", "execute" );
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;

    int alu_opB = 0;
    int immediate = IR_meta->immediate;
    int shifted_immediate = (immediate) << 2;
    int opcode = IR_meta->opcode;

    int alu_opA = control->ALUSrcA == 1 ? curr_pipe_regs->A : curr_pipe_regs->pc;
    switch (control->ALUSrcB) {
        case 0:
            alu_opB = curr_pipe_regs->B;
            break;
        case 1:
            alu_opB = WORD_SIZE;
            break;
        case 2:
            alu_opB = immediate;
            break;
        case 3:
            alu_opB = shifted_immediate;
            break;
        default:
            assert(false);
            break;
    }
            //BEQ: has to check equality then calculates ta, how does this work in the ALU
    switch (control->ALUOp) {
        case 0:
            //printf("ALU performs add operation. %u + %u \n", alu_opA, alu_opB);
            next_pipe_regs->ALUOut = alu_opA + alu_opB;
            //printf("B = %u  A = %u , A + B = %u\n", alu_opB, alu_opA, next_pipe_regs->ALUOut);
            break;
        case 1:
            //printf("ALU performs subtract operation.\n");
            next_pipe_regs->ALUOut = alu_opA - alu_opB;
            printf("BEQ: A = %u B = %u \n", alu_opA, alu_opB);
            break;
        case 2:
            //printf("Funct field determines ALU operation.\n");
            if (IR_meta->function == ADD) {
                next_pipe_regs->ALUOut = alu_opA + alu_opB;
                printf("ADD: A = %u B = %u \n", alu_opA, alu_opB);
            } else if (IR_meta->function == SLT) {
                next_pipe_regs->ALUOut = alu_opA < alu_opB ? 1 : 0;
            }
            break;
        default:
            assert(false);
            break;
    }

    // PC calculation
    switch (control->PCSource) {
        case 0: //INSTR_FETCH
            next_pipe_regs->pc = next_pipe_regs->ALUOut; /// next -> next is vital for pc counter
            //printf("PCsrc 0: %u state %u\n", next_pipe_regs->pc, arch_state.state);
            break;
        case 1: //BRANCH_COMPL
            //next_pipe_regs->pc = curr_pipe_regs->ALUOut;
            next_pipe_regs->pc = (shifted_immediate) + (curr_pipe_regs->pc);
            //printf("PCsrc 1: %u state %u\n", next_pipe_regs->pc, arch_state.state);
            break;
        case 2: //JUMP_COMPL
            next_pipe_regs->pc = (IR_meta->jmp_offset << 2) | (curr_pipe_regs->pc & 0xf0000000);
            //printf("PCsrc 2: %u state %u\n", next_pipe_regs->pc, arch_state.state);
            break;
        default:
            assert(false);
            break;
    }

}


void memory_access() {
  //printf("%s\n", "memory_access" );
  ///@students: appropriate calls to functions defined in memory_hierarchy.c must be added
  if (arch_state.control.IorD) {
    if (arch_state.control.MemRead) {
      printf("READING FROM MEMORY: ");
      int address = arch_state.curr_pipe_regs.ALUOut;
      arch_state.curr_pipe_regs.MDR = memory_read(address);
      printf("MDR(%u) READ FROM MEMORY\n", arch_state.curr_pipe_regs.MDR);
    }
    if (arch_state.control.MemWrite) {
      printf("WRITING TO MEMORY: ");
      int address = arch_state.curr_pipe_regs.ALUOut;
      printf("ALUout = %u", arch_state.curr_pipe_regs.ALUOut);
      memory_write(address, arch_state.curr_pipe_regs.B);
      printf("SAVING %u TO ADDRESS %u\n", arch_state.curr_pipe_regs.B, arch_state.curr_pipe_regs.ALUOut);
    }
  }
}

void write_back() {
  //printf("%s\n", "write_back" );
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct ctrl_signals *control = &arch_state.control;
    int opcode = IR_meta->opcode;
    int write_reg_id;
    int write_data;

    if (arch_state.control.RegWrite) {
        if (arch_state.control.RegDst) {
            write_reg_id = arch_state.IR_meta.reg_11_15;
            write_data = arch_state.curr_pipe_regs.ALUOut;
        } else {
            write_reg_id = arch_state.IR_meta.reg_16_20;

            if (arch_state.control.MemtoReg) {
                write_data = arch_state.curr_pipe_regs.MDR;
            } else {
                write_data = arch_state.curr_pipe_regs.ALUOut;
            }
        }
        check_is_valid_reg_id(write_reg_id);

        if (write_reg_id > 0) {
            arch_state.registers[write_reg_id] = write_data;
            printf("Reg $%u = %d \n", write_reg_id, write_data);
        } else printf("Attempting to write reg_0. That is likely a mistake \n");
    }
}


void set_up_IR_meta(int IR, struct instr_meta *IR_meta)
{
  //printf("%s\n", "set_up_IR_meta" );
    IR_meta->opcode = get_piece_of_a_word(IR, OPCODE_OFFSET, OPCODE_SIZE);
    IR_meta->immediate = get_sign_extended_imm_id(IR, IMMEDIATE_OFFSET);
    IR_meta->function = get_piece_of_a_word(IR, 0, 6);
    IR_meta->jmp_offset = get_piece_of_a_word(IR, 0, 26);
    IR_meta->reg_11_15 = (uint8_t) get_piece_of_a_word(IR, 11, REGISTER_ID_SIZE);
    IR_meta->reg_16_20 = (uint8_t) get_piece_of_a_word(IR, 16, REGISTER_ID_SIZE);
    IR_meta->reg_21_25 = (uint8_t) get_piece_of_a_word(IR, 21, REGISTER_ID_SIZE);
    IR_meta->type = get_instruction_type(IR_meta->opcode);

    switch (IR_meta->opcode) {
        case SPECIAL:
            if (IR_meta->function == ADD)
                printf("Executing ADD(%u), $%u = $%d + $%d (function: %u) \n",
                      IR_meta->opcode,  IR_meta->reg_11_15, IR_meta->reg_16_20,  IR_meta->reg_21_25, IR_meta->function);
            else if (IR_meta->function == SLT)
                printf("Executing SLT(%u), $%u = 1 if $%d < $%d else $%u = 0 (function: %u) \n",
                      IR_meta->opcode, IR_meta->reg_11_15, IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->reg_11_15, IR_meta->function);
            break;
        case ADDI:
            printf("Executing ADDI(%u), $%u = $%u + %d \n",
                IR_meta->opcode, IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
            break;
        case LW:
            printf("Executing LW(%u), a word is loaded into $%u from the memory address of $%u with offset %p \n",
                IR_meta->opcode, IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
            break;
        case SW:
            printf("Executing SW(%u), the contents of $%u is stored at the address of $%u with offset %p \n",
                IR_meta->opcode, IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
            break;
        case BEQ:
            printf("Executing BEQ(%u), if $%d = $%d then branch to the address %p \n",
                IR_meta->opcode, IR_meta->reg_16_20, IR_meta->reg_21_25, IR_meta->immediate);
            break;
        case J:
            printf("Executing J(%u), jumps to address %p \n",
                IR_meta->opcode, IR_meta->jmp_offset);
            break;
        case EOP:
            printf("Executing EOP(%u) \n", IR_meta->opcode);
            break;
        default: assert(false);
    }
}

void assign_pipeline_registers_for_the_next_cycle()
{
  //printf("%s\n", "assign_pipeline_registers_for_the_next_cycle" );
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;

    if (control->IRWrite) {
        curr_pipe_regs->IR = next_pipe_regs->IR;
        printf("PC %d: ", curr_pipe_regs->pc / 4);
        set_up_IR_meta(curr_pipe_regs->IR, IR_meta);
    }

        //curr_pipe_regs->MDR = next_pipe_regs->MDR;
    curr_pipe_regs->ALUOut = next_pipe_regs->ALUOut;
    curr_pipe_regs->A = next_pipe_regs->A;
    curr_pipe_regs->B = next_pipe_regs->B;

    if (control->PCWrite) {
        check_address_is_word_aligned(next_pipe_regs->pc);
        curr_pipe_regs->pc = next_pipe_regs->pc;
    }
    if (control->PCWriteCond && !arch_state.curr_pipe_regs.ALUOut) {
      check_address_is_word_aligned(next_pipe_regs->pc);
      curr_pipe_regs->pc = next_pipe_regs->pc;
    }
}


int main(int argc, const char* argv[])
{
    /*--------------------------------------
    /------- Global Variable Init ----------
    /--------------------------------------*/
    parse_arguments(argc, argv);
    arch_state_init(&arch_state);
    ///@students WARNING: Do NOT change/move/remove main's code above this point!
    while (true) {

        ///@students: Fill/modify the function bodies of the 7 functions below,
        /// Do NOT modify the main() itself, you only need to
        /// write code inside the definitions of the functions called below.

        FSM();

        instruction_fetch();

        decode_and_read_RF();

        execute();

        memory_access();

        write_back();

        assign_pipeline_registers_for_the_next_cycle();


       ///@students WARNING: Do NOT change/move/remove code below this point!
        marking_after_clock_cycle();
        arch_state.clock_cycle++;
        // Check exit statements
        if (arch_state.state == EXIT_STATE) { // I.E. EOP instruction!
            printf("\nExiting because the exit state was reached \n");
            printf("\nMEMORY STATS:\n");
            printf("Total LWs: %u\n", arch_state.mem_stats.lw_total);
            printf("Total LW cache hits: %u\n", arch_state.mem_stats.lw_cache_hits);
            printf("Total SWs: %u\n", arch_state.mem_stats.sw_total);
            printf("Total SW cache hits: %u\n", arch_state.mem_stats.sw_cache_hits);
            break;
        }
        if (arch_state.clock_cycle == BREAK_POINT) {
            printf("\nExiting because the break point (%u) was reached. \n", BREAK_POINT);
            printf("\nMEMORY STATS:\n");
            printf("Total LWs: %u\n", arch_state.mem_stats.lw_total);
            printf("Total LW cache hits: %u\n", arch_state.mem_stats.lw_cache_hits);
            printf("Total SWs: %u\n", arch_state.mem_stats.sw_total);
            printf("Total SW cache hits: %u\n", arch_state.mem_stats.sw_cache_hits);
            break;
        }

    }
    marking_at_the_end();
}
