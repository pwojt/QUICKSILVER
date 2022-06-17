#include "memory.h"

#include <stdlib.h>

#include "drv_interrupt.h"
#include "failloop.h"

void *memory_alloc(size_t size) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    void *ptr = malloc(size);
    if (ptr == NULL) {
      failloop(FAILLOOP_HEAP);
    }
    return ptr;
  }
  return NULL;
}

void *memory_realloc(void *ptr, size_t size) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    ptr = realloc(ptr, size);
    if (ptr == NULL) {
      failloop(FAILLOOP_HEAP);
    }
    return ptr;
  }
  return NULL;
}

void memory_free(void *ptr) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    free(ptr);
  }
}