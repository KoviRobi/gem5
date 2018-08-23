#include <stdio.h>
#include <stdint.h>

char mem[4096];

void zero(void *start, unsigned size)
{
  printf("calling zero at %p size 0x%08x\n", start, size);
  asm(
      "mov x2, %[start]\n"
      "mov x3, %[size]\n"
      ".word (0b0011100000100000000000 << 10) | (2 << 5) | 3"
      :
      : [start]"r"(start), [size]"r"(size)
      : "x2", "x3"
     );
}

void print_mem()
{
  printf("\n");
  for (int i = 0; i < sizeof(mem); ++i)
    putchar(mem[i]%26+'a');
  printf("\n");
  printf("\n");
}

int main()
{
  printf("mem is at 0x%p\n", &mem[0]);
  for (int i = 0; i < sizeof(mem); ++i)
    mem[i] = i;
  printf("Hello world!\n");
  intptr_t aligned = (intptr_t)&mem[0x40];
  aligned = aligned & (~1<<6);
  zero((void*)aligned, 128);
  printf("Bye world!\n");
}
