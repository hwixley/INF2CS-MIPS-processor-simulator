/*************************************************************************************|
|   1. YOU ARE NOT ALLOWED TO SHARE/PUBLISH YOUR CODE (e.g., post on piazza or online)|
|   2. Fill main.c and memory_hierarchy.c files                                       |
|   3. Do not use any other .c files neither alter main.h or parser.h                 |
|   4. Do not include any other library files                                         |
|*************************************************************************************/
#include "mipssim.h"

/// @students: declare cache-related structures and variables here

//direct mapped cache definitive characteristics:
const int address_bit_length = 32;
const int block_bytes = 16;
const int offset_bits = 4;
const int data_bits = 8;
//initialized in cache init (nested inside memory init)
int total_lines;
int tag_bits;
int index_bits;

typedef struct Block_ {
  int valid;
  int tag;
  int data[4];
}block;

block *cache;

void cache_state_init() {
  index_bits = log(cache_size/16)/log(2);
  tag_bits = address_bit_length - offset_bits - index_bits;
  assert(tag_bits + index_bits + offset_bits == 32);

  total_lines = cache_size/block_bytes;

  cache = (block *) malloc(total_lines * sizeof(block));

  for (int i = 0; i < total_lines; i++) {
    cache[i].valid = 0;
    cache[i].tag = 0;
    for (int a = 0; a < 4; a++) {
      cache[i].data[a] = 0;
    }
  }

  if (cache != NULL) {
    printf("%u bytes of memory allocated to the cache.\n", cache_size);
  } else {
    printf("Memory not allocated to the cache.\n");
  }
}

void memory_state_init(struct architectural_state* arch_state_ptr) {
    arch_state_ptr->memory = (uint32_t *) malloc(sizeof(uint32_t) * MEMORY_WORD_NUM);
    memset(arch_state_ptr->memory, 0, sizeof(uint32_t) * MEMORY_WORD_NUM);
    if(cache_size == 0){
        // CACHE DISABLED
        memory_stats_init(arch_state_ptr, 0); // WARNING: we initialize for no cache 0
    }else {
        // CACHE ENABLED
        cache_state_init();
        memory_stats_init(arch_state_ptr, tag_bits);
        /// @students: memory_stats_init(arch_state_ptr, X); <-- fill # of tag bits for cache 'X' correctly
    }
}

// returns data on memory[address / 4]
int memory_read(int address){
    arch_state.mem_stats.lw_total++;
    check_address_is_word_aligned(address);

    if(cache_size == 0){
        // CACHE DISABLED
        //printf("Read from memory:\n");
        return (int) arch_state.memory[address / 4];
    }else{
        // CACHE ENABLED
        printf("Attempting to read from cache\n");

        int a_offset = get_piece_of_a_word(address, 0, offset_bits);
        int a_index = get_piece_of_a_word(address, offset_bits, index_bits);
        int a_tag = get_piece_of_a_word(address, index_bits + offset_bits, tag_bits);
        int offset = (int) (a_offset)/4;

        if ((cache[a_index].valid) && (cache[a_index].tag == a_tag)) {
            //FOUND WORD IN CACHE
            printf("Found word in cache.\n");
            arch_state.mem_stats.lw_cache_hits++;

            return (int) cache[a_index].data[offset];

        } else {
          //COULD NOT FIND WORD IN CACHE
            cache[a_index].tag = a_tag;
            cache[a_index].valid = 1;

            for (int a = 0; a < 4; a++) {
                cache[a_index].data[a] = arch_state.memory[((address/4)-((address/4)%4)+a)];
            }

            printf("Could not find word in cache.\n");
            return (int) arch_state.memory[address / 4];
        }
    }
    //return 0;
}

// writes data on memory[address / 4]
void memory_write(int address, int write_data){
    arch_state.mem_stats.sw_total++;
    check_address_is_word_aligned(address);

    if(cache_size == 0){
        // CACHE DISABLED
        arch_state.memory[address / 4] = (uint32_t) write_data;
        //return void;
        printf("    Data stored in the memory.\n");

    }else{
        // CACHE ENABLED
        int a_offset = get_piece_of_a_word(address, 0, offset_bits);
        int a_index = get_piece_of_a_word(address, offset_bits, index_bits);
        int a_tag = get_piece_of_a_word(address, index_bits + offset_bits, tag_bits);

        cache[a_index].valid = 1;
        cache[a_index].tag = a_tag;
        int offset = (int) a_offset/4;

        if ((cache[a_index].tag == a_tag) && (cache[a_index].valid)) {
            arch_state.mem_stats.sw_cache_hits++;
            cache[a_index].data[offset] = write_data;

            arch_state.memory[address/4] = (uint32_t) write_data;
            printf("    Data successfully stored in cache.\n");
            //for (int a = 0; a < 4; a++) {
            //    arch_state.memory[(address/4)+a] = cache[a_index].data[a];
          //}
        } else {
            // cache[a_index].tag == a_tag;
            // for (int a = 0; a < 4; a++) {
            //     cache[a_index].data[a] = arch_state.memory[((address/4)-((address/4)%4)+a)];
            // }
            // cache[a_index].valid = 1;
            //
            // for (int b = 0; b < 4; b++) {
            //   arch_state.memory[(address/4)+b] = cache[a_index].data[b];
            // }
            arch_state.memory[address/4] = (uint32_t) write_data;
            printf("    Data unsuccessfully stored in the cache, data written to memory.\n");
        }
        //return void;
        //assert(0); /// @students: Remove assert(0); and implement Memory hierarchy w/ cache

        /// @students: your implementation must properly increment: arch_state_ptr->mem_stats.sw_cache_hits
    }
}
