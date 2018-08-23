#include <stdio.h>
#include <stdint.h>
#include "zero.h"
#include <gem5/m5ops.h>

char mem[16<<20]; // 16M

void *
linaro_memset(void*p, int c, size_t s);

#define work(offset,size) \
  /* To warm caches */ \
  printf("Doing inst "#offset" "#size"\n"); \
  zero_instr(&mem[offset], size); \
  m5_reset_stats(0,0); \
  zero_instr(&mem[offset], size); \
  m5_dump_stats(0,0); \
  printf("Doing loop "#offset" "#size"\n"); \
  m5_reset_stats(0,0); \
  zero_loop(&mem[offset], size); \
  m5_dump_stats(0,0); \
  printf("Doing linaro memset "#offset" "#size"\n"); \
  linaro_memset(&mem[offset], 0, size); \
  m5_reset_stats(0,0); \
  linaro_memset(&mem[offset], 0, size); \
  m5_dump_stats(0,0);


int main()
{
  printf("Hello, world\n");
  work(0, 16);
  work(0, 16<<3);
  work(0, 16<<6);
  work(0, 16<<9);
  work(0, 16<<12);
  work(0, 16<<15);
  work(0, 16<<18);
  printf("Bye, world\n");
  return 0;
}
