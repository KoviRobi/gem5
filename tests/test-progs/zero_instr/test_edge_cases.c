#include <stdio.h>
#include <stdint.h>
#include "zero.h"
#include <gem5/m5ops.h>
#define DEBUG
#include "zero.c"

//char mem[16<<20]; // 16M
char mem[4<<12]; // 4 * 4K pages


int main()
{
  printf("Hello, world\n");
  uintptr_t aligned = (uintptr_t)&mem[0];
  aligned = aligned + 4096 & ~1<<11;
  zero_instr((void*)aligned+1, (2<<12)+4094);
  printf("Bye, world\n");
}
