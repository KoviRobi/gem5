#include <stdio.h>
#include <stdint.h>
#include "zero.h"
#include <gem5/m5ops.h>

char mem[SIZE];

int main()
{
  printf("Hello, world\n");
  m5_reset_stats(0,0);
  zero_instr(&mem[0], SIZE);
  m5_dump_stats(0,0);
  m5_reset_stats(0,0);
  zero_instr(&mem[0], SIZE);
  m5_dump_stats(0,0);
  m5_reset_stats(0,0);
  zero_instr(&mem[0], SIZE);
  m5_dump_stats(0,0);
#if defined(USE_zero_instr)
  zero_instr(&mem[0], SIZE);
#elif defined(USE_zero_loop)
  zero_loop(&mem[0], SIZE);
#else
# error "Unknown target"
#endif
}
