#include <stdio.h>
#include <stdint.h>
#include "zero.h"

char mem[4096];

void print_mem()
{
  printf("\n");
  for (int i = 0; i < sizeof(mem); ++i) {
    putchar(mem[i]%64+'?'); // ?@ A-Z [\]^_` a-z {|}~
    if (i%64==63) putchar('\n');
  }
  putchar('\n');
  putchar('\n');
}

int main()
{
  printf("mem is at 0x%p\n", &mem[0]);
  for (int i = 0; i < sizeof(mem); ++i)
    mem[i] = i;
  zero_instr(&mem[34], 64*10);
  print_mem();
  return 0;
}
