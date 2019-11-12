/*
 * mm_kr_heap.c
 *
 * Based on C dynamic memory manager code from
 * Brian Kernighan and Dennis Richie (K&R)
 *
 *  @since Feb 13, 2019
 *  @author philip gust
 */


#include <stdio.h>
#include <unistd.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include "memlib.h"
#include "mm_heap.h"


// Allocation unit for header of memory blocks
typedef union Header {
    struct {
        union Header *ptr;  // next block if on free list
        size_t size;        // size of this block including header
                            // measured in multiple of header size
    } s;
    max_align_t _align;     // force alignment to max align boundary */
} Header;

// forward declarations
static Header *morecore(size_t);
void visualize(const char*);

/** Empty list to get started */
static Header base;

/** Start of free memory list */
static Header *freep = NULL;

/**
 * Initialize memory allocator
 */
void mm_init(void) {
	mem_init();

    base.s.ptr = freep = &base;
    base.s.size = 0;
}

/**
 * Reset memory allocator.
 */
void mm_reset(void) {
	mem_reset_brk();

	base.s.ptr = freep = &base;
    base.s.size = 0;
}

/**
 * De-initialize memory allocator.
 */
void mm_deinit(void) {
	mem_deinit();

	base.s.ptr = freep = &base;
    base.s.size = 0;
}

/**
 * Allocation units for nbytes bytes.
 *
 * @param nbytes number of bytes
 * @return number of units for nbytes
 */
inline static size_t mm_units(size_t nbytes) {
    /* smallest count of Header-sized memory chunks */
    /*  (+1 additional chunk for the Header itself) needed to hold nbytes */
    return (nbytes + sizeof(Header) - 1) / sizeof(Header) + 1;
}

/**
 * Allocation bytes for nunits allocation units.
 *
 * @param nunits number of units
 * @return number of bytes for nunits
 */
inline static size_t mm_bytes(size_t nunits) {
    return nunits * sizeof(Header);
}

/**
 * Get pointer to block payload.
 *
 * @param bp the block
 * @return pointer to allocated payload
 */
inline static void *mm_payload(Header *bp) {
	return bp + 1;
}

/**
 * Get pointer to block for payload.
 *
 * @param ap the allocated payload pointer
 */
inline static Header *mm_block(void *ap) {
	return (Header*)ap - 1;
}



/**
 * Allocates size bytes of memory and returns a pointer to the
 * allocated memory, or NULL if request storage cannot be allocated.
 *
 * @param nbytes the number of bytes to allocate
 * @return pointer to allocated memory or NULL if not available.
 */
void *mm_malloc(size_t nbytes) {
    if (freep == NULL) {
    	mm_init();
    }

    Header *prevp = freep;

    // smallest count of Header-sized memory chunks
    //  (+1 additional chunk for the Header itself) needed to hold nbytes
    size_t nunits = mm_units(nbytes);

    // traverse the circular list to find a block
    for (Header *p = prevp->s.ptr; ; prevp = p, p = p->s.ptr) {
        if (p->s.size >= nunits) {          /* found block large enough */
            if (p->s.size == nunits) {
				// free block exact size
                prevp->s.ptr = p->s.ptr;
            } else {
            	// split and allocate tail end
                p->s.size -= nunits; // adjust the size to split the block
                /* find the address to return */
                p += p->s.size;		 // address upper block to return
                p->s.size = nunits;	 // set size of block
            }
            p->s.ptr = NULL;  // no longer on free list
            freep = prevp;  /* move the head */
            return mm_payload(p);
        }

        /* back where we started and nothing found - we need to allocate */
        if (p == freep) {                    /* wrapped around free list */
        	p = morecore(nunits);
        	if (p == NULL) {
                errno = ENOMEM;
                return NULL;                /* none left */
            }
        }
    }

    assert(false);  // shouldn't happen
	return NULL;
}


/**
 * Deallocates the memory allocation pointed to by ap.
 * If ap is a NULL pointer, no operation is performed.
 *
 * @param ap the memory to free
 */
void mm_free(void *ap) {
	// ignore null pointer
    if (ap == NULL) {
        return;
    }

    Header *bp = mm_block(ap);   /* point to block header */

    // validate size field of header block
    assert(bp->s.size > 0 && mm_bytes(bp->s.size) <= mem_heapsize());

    // find where to insert the free space
    // (bp > p && bp < p->s.ptr) => between two nodes
    // (p > p->s.ptr)            => this is the end of the list
    // (p == p->p.ptr)           => list is one element only
    Header *p = freep;
    for ( ; !(bp > p && bp < p->s.ptr); p = p->s.ptr) {
        if (p >= p->s.ptr && (bp > p || bp < p->s.ptr)) {
        	// freed block at start or end of arena
            break;
        }
	}

    if (bp + bp->s.size == p->s.ptr) {
		// coalesce if adjacent to upper neighbor
        bp->s.size += p->s.ptr->s.size;
        bp->s.ptr = p->s.ptr->s.ptr;
    } else {
    	// link in before upper block
        bp->s.ptr = p->s.ptr;
    }

    if (p + p->s.size == bp) {
		// coalesce if adjacent to lower block
        p->s.size += bp->s.size;
        p->s.ptr = bp->s.ptr;

    } else {
		// link in after lower block
        p->s.ptr = bp;
    }

    /* reset the start of the free list */
    freep = p;
}

/**
 * Tries to change the size of the allocation pointed to by ap
 * to size, and returns ap.
 *
 * If there is not enough room to enlarge the memory allocation
 * pointed to by ap, realloc() creates a new allocation, copies
 * as much of the old data pointed to by ptr as will fit to the
 * new allocation, frees the old allocation, and returns a pointer
 * to the allocated memory.
 *
 * If ap is NULL, realloc() is identical to a call to malloc()
 * for size bytes.  If size is zero and ptr is not NULL, a minimum
 * sized object is allocated and the original object is freed.
 */
void* mm_realloc(void *ap, size_t newsize) {
	// NULL ap acts as malloc for size newsize bytes
	if (ap == NULL) {
		return mm_malloc(newsize);
	}

	Header* bp = mm_block(ap);    // point to block header
	if (newsize > 0) {
		// return this ap if allocated block large enough
		if (bp->s.size >= mm_units(newsize)) {
			return ap;
		}
	}

	// allocate new block
	void *newap = mm_malloc(newsize);
	if (newap == NULL) {
		return NULL;
	}
	// copy old block to new block
	size_t oldsize = mm_bytes(bp->s.size-1);
	memcpy(newap, ap, (oldsize < newsize) ? oldsize : newsize);
	mm_free(ap);
	return newap;
}


/**
 * Request additional memory to be added to this process.
 *
 * @param nu the number of Header-chunks to be added
 */
static Header *morecore(size_t nu) {
	// nalloc based on page size
	size_t nalloc = mem_pagesize()/sizeof(Header);

    /* get at least NALLOC Header-chunks from the OS */
    if (nu < nalloc) {
        nu = nalloc;
    }

    size_t nbytes = mm_bytes(nu); // number of bytes
    void* p = mem_sbrk(nbytes);
    if (p == (char *) -1) {	// no space
        return NULL;
    }

    Header* bp = (Header*)p;
    bp->s.size = nu;

    // add new space to the circular list
    mm_free(bp+1);

    return freep;
}

/**
 * Print the free list (debugging only)
 *
 * @msg the initial message to print
 */
void visualize(const char* msg) {
    fprintf(stderr, "\n--- Free list after \"%s\":\n", msg);

    if (freep == NULL) {                   /* does not exist */
        fprintf(stderr, "    List does not exist\n\n");
        return;
    }

    if (freep == freep->s.ptr) {          /* self-pointing list = empty */
        fprintf(stderr, "    List is empty\n\n");
        return;
    }

/*
    tmp = freep;                           // find the start of the list
    char* str = "    ";
    do {           // traverse the list
        fprintf(stderr, "%sptr: %10p size: %-3lu blks - %-5lu bytes\n", 
        	str, (void *)tmp, tmp->s.size, mm_bytes(tmp->s.size));
        str = " -> ";
        tmp = tmp->s.ptr;
    }  while (tmp->s.ptr > freep);
*/
    char* str = "    ";
    for (Header *p = base.s.ptr; p != &base; p = p->s.ptr) {
        fprintf(stderr, "%sptr: %10p size: %3lu blks - %5lu bytes\n", 
        	str, (void *)p, p->s.size, mm_bytes(p->s.size));
        str = " -> ";
    }


    fprintf(stderr, "--- end\n\n");
}


/**
 * Calculate the total amount of available free memory.
 *
 * @return the amount of free memory in bytes
 */
size_t mm_getfree(void) {
    if (freep == NULL) {
        return 0;
    }

    Header *tmp = freep;
    size_t res = tmp->s.size;

    while (tmp->s.ptr > tmp) {
        tmp = tmp->s.ptr;
        res += tmp->s.size;
    }

    return mm_bytes(res);
}
