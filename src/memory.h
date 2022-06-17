#pragma once

#include <stddef.h>

void *memory_alloc(size_t size);
void *memory_realloc(void *ptr, size_t size);
void memory_free(void *ptr);
