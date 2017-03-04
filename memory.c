#include <stdlib.h>
#include <stddef.h>
#include "memory.h"
/* Returns a piece of memory aligned to the given
 * alignment parameter. Alignment must be a power of
 * 2.
 * This function returns memory of length 'bytes' or more
 */
void *aligned_malloc(size_t bytes, size_t alignment)
{

        /* Check if alignment is a power of 2
         * as promised by the caller.
         */
        if ( alignment & (alignment-1)) /* If not a power of 2 */
                return NULL;
        
        /* Determine how much more to allocate
         * to make room for the alignment:
         * 
         * We need (alignment - 1) extra locations 
         * in the worst case - i.e., malloc returns an
         * address off by 1 byte from an aligned
         * address.
         */
        size_t size = bytes + alignment - 1; 

        /* Additional storage space for storing a delta. */
        size += sizeof(size_t);

        /* Allocate memory using malloc() */
        void *malloc_ptr = safe_malloc(size);

        if (NULL == malloc_ptr)
                return NULL;

        /* Move pointer to account for storage of delta */
        void *new_ptr = (void *) ((char *)malloc_ptr + sizeof(size_t));

        /* Make ptr a multiple of alignment,
         * using the standard trick. This is
         * used everywhere in the Linux kernel
         * for example.
         */
        void *aligned_ptr = (void *) (((size_t)new_ptr + alignment - 1) & ~(alignment -1));

        size_t delta = (size_t)aligned_ptr - (size_t)malloc_ptr;

        /* write the delta just before the place we return to user */
        *((size_t *)aligned_ptr - 1) = delta;

        return aligned_ptr;
}


/* Frees a chunk of memory returned by aligned_malloc() */
void aligned_free(void *ptr)
{

        if (NULL == ptr)
                return;

        /* Retrieve delta */
        size_t delta = *( (size_t *)ptr - 1);

        /* Calculate the original ptr returned by malloc() */
        void *malloc_ptr = (void *) ( (size_t)ptr - delta);

        safe_free(malloc_ptr);
}
