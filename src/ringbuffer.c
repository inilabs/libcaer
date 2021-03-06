#include "libcaer/ringbuffer.h"

#include "portable_aligned_alloc.h"

#include <stdalign.h> // To get alignas() macro.
#include <stdatomic.h>

// Alignment specification support (with defines for cache line alignment).
#if !defined(CACHELINE_SIZE)
#	define CACHELINE_SIZE 128 // Default (big enough for most processors), must be power of two!
#endif

struct caer_ring_buffer {
	alignas(CACHELINE_SIZE) size_t putPos;
	alignas(CACHELINE_SIZE) size_t getPos;
	alignas(CACHELINE_SIZE) size_t size;
	atomic_uintptr_t elements[];
};

caerRingBuffer caerRingBufferInit(size_t size) {
	// Force multiple of two size for performance.
	if ((size == 0) || ((size & (size - 1)) != 0)) {
		return (NULL);
	}

	caerRingBuffer rBuf
		= portable_aligned_alloc(CACHELINE_SIZE, sizeof(struct caer_ring_buffer) + (size * sizeof(atomic_uintptr_t)));
	if (rBuf == NULL) {
		return (NULL);
	}

	// Initialize counter variables.
	rBuf->putPos = 0;
	rBuf->getPos = 0;
	rBuf->size   = size;

	// Initialize pointers.
	for (size_t i = 0; i < size; i++) {
		atomic_store_explicit(&rBuf->elements[i], (uintptr_t) NULL, memory_order_relaxed);
	}

	atomic_thread_fence(memory_order_release);

	return (rBuf);
}

void caerRingBufferFree(caerRingBuffer rBuf) {
	portable_aligned_free(rBuf);
}

bool caerRingBufferPut(caerRingBuffer rBuf, void *elem) {
	if (elem == NULL) {
		// NULL elements are disallowed (used as place-holders).
		// Critical error, should never happen -> exit!
		exit(EXIT_FAILURE);
	}

	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->putPos], memory_order_acquire);

	// If the place where we want to put the new element is NULL, it's still
	// free and we can use it.
	if (curr == NULL) {
		atomic_store_explicit(&rBuf->elements[rBuf->putPos], (uintptr_t) elem, memory_order_release);

		// Increase local put pointer.
		rBuf->putPos = ((rBuf->putPos + 1) & (rBuf->size - 1));

		return (true);
	}

	// Else, buffer is full.
	return (false);
}

bool caerRingBufferFull(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->putPos], memory_order_acquire);

	// If the place where we want to put the new element is NULL, it's still
	// free and thus the buffer still has available space.
	if (curr == NULL) {
		return (false);
	}

	// Else, buffer is full.
	return (true);
}

void *caerRingBufferGet(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->getPos], memory_order_acquire);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, and reset the place to NULL.
	if (curr != NULL) {
		atomic_store_explicit(&rBuf->elements[rBuf->getPos], (uintptr_t) NULL, memory_order_release);

		// Increase local get pointer.
		rBuf->getPos = ((rBuf->getPos + 1) & (rBuf->size - 1));

		return (curr);
	}

	// Else, buffer is empty.
	return (NULL);
}

void *caerRingBufferLook(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->getPos], memory_order_acquire);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, without removing it from the
	// ring buffer.
	if (curr != NULL) {
		return (curr);
	}

	// Else, buffer is empty.
	return (NULL);
}

bool caerRingBufferEmpty(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->getPos], memory_order_acquire);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, without removing it from the
	// ring buffer.
	if (curr != NULL) {
		return (false);
	}

	// Else, buffer is empty.
	return (true);
}
