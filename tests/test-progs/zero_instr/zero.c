#include <stdint.h>
#include <string.h>
#include "zero.h"

#ifdef DEBUG
# include <stdio.h>
# define dbg(...) {fprintf(stdout, "%s\n", #__VA_ARGS__);\
  __VA_ARGS__}
#else
# define dbg(...) __VA_ARGS__
#endif

/// Size is in bytes
void zero_instr(void *start, size_t size)
{
  uintptr_t istart = (uintptr_t)start;
  uintptr_t iend   = (uintptr_t)istart + size;
  uintptr_t start_8B  =  istart+7 & ~1<<2;
  uintptr_t start_64B = istart+63 & ~1<<5;
  uintptr_t start_4KB = istart+4095 & ~1<<11;
  uintptr_t end_4KB   = iend & ~1<<11;
  uintptr_t end_64B   = iend & ~1<<5;
  uintptr_t end_8B    = iend & ~1<<2;
  uintptr_t register pos asm("x0") = istart;
  if (size > 8192) {
    for (; pos < start_8B;  pos += sizeof(uint8_t))  dbg(*(uint8_t*)pos = 0;);
    for (; pos < start_64B; pos += sizeof(uint64_t)) dbg(*(uint64_t*)pos = 0;);
    if (pos < start_4KB) {
      uintptr_t register size asm("x1") = start_4KB-pos;
      dbg(asm volatile(".word 0x38200000 | (0 << 5) | 1" : : : "memory"););
      pos = start_4KB;
    }
    for (; pos < end_4KB;   pos += 4096) {
      uintptr_t register size asm("x1") = 4096;
      dbg(asm volatile(".word 0x38200000 | (0 << 5) | 1" : : : "memory"););
    }
    if (pos < end_64B) {
      uintptr_t register size asm("x1") = end_64B-pos;
      dbg(asm volatile(".word 0x38200000 | (0 << 5) | 1" : : : "memory"););
      pos = end_64B;
    }
    for (; pos < end_8B; pos += sizeof(uint64_t)) dbg(*(uint64_t*)pos = 0;);
    for (; pos < iend;   pos += sizeof(uint8_t))  dbg(*(uint8_t*)pos = 0;);
  } else if (size > 128) {
    for (; pos < start_8B;  pos += sizeof(uint8_t))  dbg(*(uint8_t*)pos = 0;);
    for (; pos < start_64B; pos += sizeof(uint64_t)) dbg(*(uint64_t*)pos = 0;);
    if (pos < start_4KB) {
      uintptr_t register size asm("x1") = start_4KB-pos;
      dbg(asm volatile(".word 0x38200000 | (0 << 5) | 1" : : : "memory"););
      pos = start_4KB;
    }
    if (pos < end_64B) {
      uintptr_t register size asm("x1") = end_64B-pos;
      dbg(asm volatile(".word 0x38200000 | (0 << 5) | 1" : : : "memory"););
      pos = end_64B;
    }
    for (; pos < end_8B; pos += sizeof(uint64_t)) dbg(*(uint64_t*)pos = 0;);
    for (; pos < iend;   pos += sizeof(uint8_t))  dbg(*(uint8_t*)pos = 0;);
  } else {
    dbg(memset(start, 0, size););
  }
}

void zero_loop(void *start, size_t size)
{
  memset(start, 0, size);
#if 0
  cast_assign(uintptr_t, istart,   start);
  cast_assign(uintptr_t, iend,     istart + size);
  cast_assign(uint64_t*, start_8B, istart+7 & ~1<<3);
  cast_assign(uint64_t*, end_8B,   iend & ~1<<3);
  for (uint8_t *p = start;     p < (uint8_t*)start_8B; ++p) *p = 0;
  for (uint64_t *p = start_8B; p < end_8B;  ++p) *p = 0;
  for (uint8_t *p = (uint8_t*)end_8B; p < (uint8_t*)iend;  ++p) *p = 0;
#endif
}

