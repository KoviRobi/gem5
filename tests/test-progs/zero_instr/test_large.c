#include <gem5/m5ops.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "zero.h"

char mem[16<<20]; // 16M

void *
newlib_memset(void*p, int c, size_t s);

void *
newlib_no_dc_zva_memset(void*p, int c, size_t s);

#define work(offset,size) \
  /* To warm caches */ \
  printf("Doing inst "#offset" "#size"\n"); \
  zero_instr(&mem[offset], size); \
  m5_reset_stats(0,0); \
  zero_instr(&mem[offset], size); \
  m5_dump_stats(0,0); \
                      \
  printf("Doing loop "#offset" "#size"\n"); \
  zero_loop(&mem[offset], size); \
  m5_reset_stats(0,0); \
  zero_loop(&mem[offset], size); \
  m5_dump_stats(0,0); \
                    \
  printf("Doing glibc "#offset" "#size"\n"); \
  memset(&mem[offset],0, size); \
  m5_reset_stats(0,0); \
  memset(&mem[offset], 0, size); \
  m5_dump_stats(0,0); \
                      \
  printf("Doing newlib memset "#offset" "#size"\n"); \
  newlib_memset(&mem[offset], 0, size); \
  m5_reset_stats(0,0); \
  newlib_memset(&mem[offset], 0, size); \
  m5_dump_stats(0,0); \
                      \
  printf("Doing newlib memset no DC ZVA "#offset" "#size"\n"); \
  newlib_no_dc_zva_memset(&mem[offset], 0, size); \
  m5_reset_stats(0,0); \
  newlib_no_dc_zva_memset(&mem[offset], 0, size); \
  m5_dump_stats(0,0);


int main()
{
  printf("Hello, world\n");
  uintptr_t aligned_4K = (uintptr_t)&mem[0];
  aligned_4K = aligned_4K + 4095 & ~0<<12;
  memset(&mem[0], 0, 4096);
  work(0,     16            );
  work(0,     16<<3         );
  work(0,     16<<6         );
  work(0,     16<<9         );
  work(0,     16<<12        );
  work(0,     16<<15        );
  work(0,     16<<18        );
  work(1,    (16    )-2     );
  work(1,    (16<<3 )-2     );
  work(1,    (16<<6 )-2     );
  work(1,    (16<<9 )-2     );
  work(1,    (16<<12)-2     );
  work(1,    (16<<15)-2     );
  work(1,    (16<<18)-2     );
  // 0x824 is 2048+32+4, i.e. it is halfway aligned on 4K, 64B and 8B
  // alignments
  work(0x824, (16    )+0x824);
  work(0x824, (16<<3 )+0x824);
  work(0x824, (16<<6 )+0x824);
  work(0x824, (16<<9 )+0x824);
  work(0x824, (16<<12)+0x824);
  work(0x824, (16<<15)+0x824);
  work(0x824, (16<<18)+0x824);
  printf("Bye, world\n");
  return 0;
}
