#include "memory.h"

#include <stdlib.h>

#include "drv_interrupt.h"

void *memory_alloc(size_t size) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    return malloc(size);
  }
  return NULL;
}

void *memory_realloc(void *ptr, size_t size) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    return realloc(ptr, size);
  }
  return NULL;
}

void memory_free(void *ptr) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    free(ptr);
  }
}