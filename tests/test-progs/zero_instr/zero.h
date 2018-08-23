#include <stddef.h>

#ifndef __included_zero_h__
#define __included_zero_h__

/// Size is in bytes
void zero_instr(void *start, size_t size);
void zero_loop(void *start, size_t size);

#endif
