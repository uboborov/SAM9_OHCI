#ifndef MEMORY_H_
#define MEMORY_H_
#include <stddef.h>

#define safe_malloc malloc

void *aligned_malloc(size_t bytes, size_t alignment);
void aligned_free(void *ptr);

#endif /*MEMORY_H_*/
