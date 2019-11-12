/*
 * mm_dlink_heap.c
 *
 * Dynamic memory allocator based on a pool of contiguously
 * stored blocks of free and allocated memory. The pool is
 * delimited by a prologue and an epilogue block.
 *
 * begin                                                  end
 * pool                                                  pool
 *  --------------------------------------------------------
 * | prologue | zero or more allocated/free blks | epilogue |
 * |    blk   |                                  |    blk   |
 *  --------------------------------------------------------
 *
 * Each block consists of allocated or free memory, delimited
 * by fixed-size headers that serve as boundary tags for the
 * block. The block is a multiple of these fixed-size header
 * storage units to help manage fragmentation.
 *
 *  allocated block
 *  --------------------------------------------------------
 * | hdr |    two or more header-size storage units   | hdr |
 *  --------------------------------------------------------
 *
 * The headers record the number of header-size storage units
 * in the block, including the headers themselves, and a flag
 * marking whether the block is allocated or free. The headers
 * allow traversal of the blocks in the memory pool in either
 * direction, and simplify coalescing of adjacent free blocks.
 *
 * The two fields can be packed into bit fields of a single
 * size_t word, since the size field is the number of header
 * units rather than the number of bytes in a block.
 *
 *     | n-1                   3  2  1  0  |  0  |
 *      -----------------------------------------
 *     | s  s  s  s  ... s  s  s  s  s  s  | a/f |
 *      -----------------------------------------
 *
 * Free blocks are also managed as a doubly-linked circular
 * list to make allocation and deallocation more efficient.
 * The links are stored in the first two storage units in the
 * free space within a free block. Consequently, a block must
 * have at least two header-size storage units plus a header
 * and a footer.
 *
 *  free block
 *  --------------------------------------------------------
 * | hdr | prv | nxt |     unused storage units       | hdr |
 *  --------|-----|-----------------------------------------
 *       <--'     '-->
 *  prev free     next free
 *
 * The header is represented as a union of the size_t field,
 * a pointer to the next or previous free block in the list,
 * and a max_align_t that ensures blocks in the pool are
 * properly aligned.
 *
 *         header union
 *         -----------------------------------
 *        |      blk size     | a/f  |   **   |
 *        |-----------------------------------|
 *        |  next/prev free blk ptr  |   **   |
 *        |-----------------------------------|
 *        |            max align_t            |
 *         -----------------------------------
 *
 * The prologue of the memory pool is a block used as a
 * dummy node in the circular free list to simplify list
 * management. This block is marked as allocated to ensure
 * that it cannot be freed. The epilogue of the memory pool
 * is a header block to simplify traversal and coalescing
 * algorithms. A epilogue is a single header to simplify
 * coalescing algorithms. Prologue and epilogue headers
 * are permanently marked as allocated.
 *
 * This mm_free() and mm_realloc() check whether the void*
 * pointer parameter is to memory that is not part of the
 * pool or has not been allocated. It also checks whether
 * the pointer is within the block, and locates the block
 * boundry by searching the heap blocks until the one that
 * contains the pointer is found. This relies on a special
 * technique that detects a pointer that is not maximally
 * aligned. C does not provide such a capability, so an
 * approximation is used.
 *
 *  @since March 4, 2019
 *  @author philip gust
 */


#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>
#include "memlib.h"
#include "mm_heap.h"


// Allocation unit for header/footer of m blocks
// and for free list pointers within free block payload
typedef union Header {          /* block header/footer */
    struct {
        size_t blksize: (8*sizeof(size_t)-1); // size of this block including header+footer
                                              // measured in multiples of header size;
        size_t isalloc: 1;                    // 1 if block allocated, 0 if free
    } s;
    union Header *blkp;						  // pointer to adjacent block on free list
    max_align_t _align;              		  // force alignment to max align boundary
} Header;

// 2 header units for freelist ptrs.
#define MIN_PAYLOAD_SIZE 2

// header + footer + MIN_PAYLOAD_SIZE
#define MIN_BLOCK_SIZE (2 + MIN_PAYLOAD_SIZE)

//static const size_t MIN_PAYLOAD_SIZE = 2;
//static const size_t MIN_BLOCK_SIZE = 2 + MIN_PAYLOAD_SIZE;

// forward declarations
static void do_reset(void);
static void put_free_block(Header *bp);
static Header *get_free_block(size_t nunits);
static Header *find_alloc_block(void *ap);
static Header *morecore(size_t);
void visualize(const char*);

/** Start of free memory list */
static Header *freep = NULL;

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
 * Allocation units for nbytes in units of header size
 *
 * @param nbytes number of bytes
 * @return number of units for nbytes
 */
inline static size_t mm_units(size_t nbytes) {
    /* smallest count of Header-sized memory chunks */
    return (nbytes + sizeof(Header) - 1) / sizeof(Header);
}

/**
 * Allocation nbytes in units of header size
 *
 * @param nunits number of units
 * @return number of bytes for nunits
 */
inline static size_t mm_bytes(size_t nunits) {
    return nunits * sizeof(Header);
}

/**
 * Unlink free block from free list.
 *
 * @param bp the block pointer
 */
inline static void unlink_free_block(Header *bp) {
	Header *nextp = bp[2].blkp;
	Header *prevp = bp[1].blkp;
	prevp[2].blkp = nextp;	 // link prev block to next block
    nextp[1].blkp = prevp;	 // link next block to prev block
}

/**
 * Link free block after specified block in free list.
 *
 * @param bp the block to link in
 * @param afterp the block to link after
 */
inline static void link_free_block_after(Header *bp, Header *afterp) {
	Header *nextp = afterp[2].blkp;
	bp[1].blkp = afterp;
	bp[2].blkp = nextp;
	afterp[2].blkp = nextp[1].blkp = bp;
}

/**
 * Returns true if ap is a maximally aligned pointer.
 * Note: C provides no direct way to do this, but this
 * approximation should be relatively portable if the
 * size of max_align_t is a power of 2.
 *
 * @param ap an allocated pointer
 * @return true if pointer is a header-aligned block pointer
 */
inline static bool is_max_aligned_pointer(void* ap) {
	// if max_align_t is 16-bytes, addresses must be
	// a multiple of 16, so bottom 4 bits of address
	// are always 0. Subtracting 1 creates a mask with
	// the required number of bits that must be 0 for
	// alignment: (16-1) == 15 or 0x1111. Cast of ap
	// to uintptr_t required to allow mask operation.
	//
    return ((uintptr_t)ap & (sizeof(max_align_t)-1)) == 0;
}
/**
 * Initialize memory allocator.
 */
void mm_init(void) {
	if (freep == NULL) {
		mem_init();
		do_reset();
	}
}

/**
 * Reset memory allocator
 */
void mm_reset(void) {
	if (freep == NULL) {
		mm_init();  // not previously initialized
	} else {
		mem_reset_brk();	// reset memlib
		do_reset();			// rebuild heap structure
	}
}

/**
 * Reset heap and free list
 */
static void do_reset(void) {
    // create initial empty heap
	// free list dummy block + epilogue header
	if (mem_sbrk((MIN_BLOCK_SIZE + 1) * sizeof(Header)) == NULL) {
		return;
	}

	// dummy block in doubly-linked circular free list
	freep = mem_heap_lo();
	freep[0].s.blksize = freep[MIN_BLOCK_SIZE-1].s.blksize = MIN_BLOCK_SIZE;
	freep[0].s.isalloc = freep[MIN_BLOCK_SIZE-1].s.isalloc = 1; // protect block
	freep[1].blkp = freep[2].blkp = freep;	// circular link pre and next

	// epilogue header
	Header *epilogue = freep + MIN_BLOCK_SIZE; // point past free list block
	epilogue[0].s.blksize = 1;
	epilogue[0].s.isalloc = 1;
}

/**
 * De-initialize memory allocator
 */
void mm_deinit(void) {
	mem_deinit();
	freep = NULL;
}

/**
 * Allocates size bytes of memory and returns a pointer to
 * allocated memory, or returns NULL and sets errno to ENOMEM
 * if storage cannot be allocated.
 *
 * @param nbytes the number of bytes to allocate
 * @return pointer to allocated memory or NULL if not available
 */
void *mm_malloc(size_t nbytes) {
    if (freep == NULL) {
    	mm_init();
    }

    // number of Header-sized memory units
    size_t nunits = mm_units(nbytes) + 2;
    if (nunits < MIN_BLOCK_SIZE) {
    	nunits = MIN_BLOCK_SIZE;
    }

    // get free blocks of required size
    Header *bp = get_free_block(nunits);
    if (bp == NULL) {
    	errno = ENOMEM;  // per spec
    	return NULL;
    }
    return mm_payload(bp);  // address of payload
}


/**
 * Deallocates the memory allocation pointed to by ap.
 * If ap is NULL, no operation is performed. If ap points
 * to memory not allocated or already free, no operation
 * is performed and errno is set to EFAULT. Pointer can be
 * anywhere within previously allocated space.
 *
 * @param ap the allocated storage to free
 */
void mm_free(void *ap) {
	if (ap != NULL) {
		// find block from pointer within allocated block payload
		Header *bp = find_alloc_block(ap);

		if (bp == NULL) {
			errno = EFAULT;  // bad address
		} else {
			// add blocks to free list
			put_free_block(bp);
		}
	}
}

/**
 * Reallocates size bytes of memory and returns a pointer
 * to the allocated memory, or NULL if memory cannot be
 * allocated, points to memory not allocated or already
 * free. Pointer can be anywhere within previously allocated
 * space.
 *
 * @param ap the currently allocated storage
 * @param nbytes the number of bytes to allocate
 * @return pointer to allocated memory or NULL if not available.
 */
void *mm_realloc(void *ap, size_t nbytes) {
	if (ap == NULL) {
		return mm_malloc(nbytes);
	}

	// find block from pointer within allocated block payload
	Header *bp = find_alloc_block(ap);
	if (bp == NULL) {
		errno = EFAULT;
		return NULL;
	}

	// get current block size
    size_t curunits = bp->s.blksize;

    // already enough units for request
    size_t nunits = mm_units(nbytes)+2; // +2 for header+footer
    if (nunits <= curunits) {
    	return ap;
    }

    // allocate new block for request
    Header *newbp = get_free_block(nunits);
    if (newbp == NULL) {
    	return NULL;
    }
    void *newap = mm_payload(newbp);  // pointer to new payload

    // copy current payload to new payload area
    size_t apbytes = mm_bytes(curunits-2); // not header or footer
    memcpy(newap, ap, apbytes);

    put_free_block(bp);  // free current storage

    return newap;  // pointer to new payload
}

/**
 * Get block from free block list, splitting free blocks
 * and requesting additional system space if necessary.
 *
 * Blocks have header and footer blocks with size and
 * allocation flags set.
 *
 * @param nunits the number of free units required
 * @return pointer to free blocks
 */
static Header *get_free_block(size_t nunits) {
    /* traverse the circular list to find a block */
    Header *bp = freep;
    while (true) {
    	// find first fit
    	if (   (bp[0].s.isalloc == 0) 	// dummy node marked allocated
    		&& (bp[0].s.blksize >= nunits)) {

            if (bp->s.blksize < nunits+MIN_BLOCK_SIZE) { // cannot split if too small
            	// if freep is here, move it to previous free block
            	if (freep == bp) {
            		freep = bp[1].blkp;
            	}

            	// unlink allocated block from free list
            	unlink_free_block(bp);

                // set block size and mark allocated
                size_t blkoff = bp[0].s.blksize;  // offset to following block
                bp[0].s.isalloc = bp[blkoff-1].s.isalloc = 1;  // mark allocated
            } else {		// split and allocate tail end
            	// offset to allocated part of split block
            	size_t blkoff = bp[0].s.blksize - nunits;

            	// adjust size of initial free part of split block
                bp[blkoff-1].s.blksize = bp[0].s.blksize -= nunits;

                // adjust size of remaining allocated part of split block
                bp[blkoff].s.blksize = bp[blkoff+nunits-1].s.blksize = nunits;

                // mark block allocated
                bp[blkoff].s.isalloc = bp[blkoff+nunits-1].s.isalloc = 1;

                // get address of header of allocated part
                bp+= blkoff;
            }

            // return pointer to block payload
            return bp;
        }

    	// advance to next free block
    	bp = bp[2].blkp;

    	// back where we started and nothing found
        // so we need to get more storage
        if (bp == freep) {                    /* wrapped around free list */
        	bp = morecore(nunits);
        	if (bp == NULL) {
                return NULL;                /* none left */
            }
        	// uses new storage on next iteration
        }
    }
}

/**
 * Put block onto free block list, coalescing adjacent blocks
 * where possible. Sets freep to freed block after coalescing.
 *
 * @param bp the blocks to free
 */
static void put_free_block(Header *bp) {
	// number of units in freed block
	size_t nunits = bp->s.blksize;

    // mark blocks free
	bp[0].s.isalloc = bp[nunits-1].s.isalloc = 0;

	if (bp[-1].s.isalloc == 0) {  // coalesce with lower adjacent block
		// point to lower block
		bp-= bp[-1].s.blksize;

		// set combined block size
		nunits+= bp[0].s.blksize;  // combined units
		bp[0].s.blksize = bp[nunits-1].s.blksize = nunits;
	} else  { // add block to free list after freep
		link_free_block_after(bp, freep);
	}
	freep = bp;

	// coalesce with upper adjacent block
	if (bp[nunits].s.isalloc == 0) {
		// unlink upper adjacent block from free list
		unlink_free_block(bp+nunits);

		// set combined block size
		nunits+= bp[nunits].s.blksize;  // combined units
		bp[0].s.blksize = bp[nunits-1].s.blksize = nunits;
	}
}

/**
 * Find allocated block from pointer.
 *
 * Note: This function tries to detect whether pointer is
 * to start of block payload by an approximation that is
 * very reliable but not 100% reliable If pointer is
 * within heap, is maximally aligned, and the header and
 * footer match, it is to start of payload. Otherwise,
 * searches blocks for one that contains pointer.
 *
 * @param ap pointer to allocated storage
 * @return pointer to allocated block or NULL if pointer
 * 		is not within allocated block returned by mm_malloc()
 */
static Header *find_alloc_block(void *ap) {
    if (ap == NULL) {	// cannot be null
        return NULL;
    }

    // pointer must be within heap
    if (ap <= mem_heap_lo() || ap >= mem_heap_hi()) {
    	return NULL;
    }

    // must be aligned
    Header *bp;
    if (is_max_aligned_pointer(ap)) {
		// must be allocated
		bp = mm_block(ap);  // point to block header
		if (bp[0].s.isalloc == 1) {
			// must have minimum block size
			size_t nunits = bp[0].s.blksize;
			if (   (nunits >= MIN_BLOCK_SIZE) 				// must have min. size
				&& ((void*)(bp+nunits) < mem_heap_hi())) { 	// must be within heap
				// header and footer must match
				if (   (bp[0].s.isalloc == bp[nunits-1].s.isalloc)
					|| (bp[0].s.blksize == bp[nunits-1].s.blksize)) {
					return bp;
				}
			}
		}
    }

    // find block that contains ap
    bp = mem_heap_lo();
    for (Header *nxtp = bp + bp[0].s.blksize;
    	(void*)nxtp <= ap; bp = nxtp, nxtp += bp[0].s.blksize) {
    }

    // return block if was allocated
    return (bp[0].s.isalloc == 1) ? bp : NULL;
}

/**
 * Request additional memory to be added to this process.
 *
 * @param nunits the number of Header-chunks to be added
 * @return pointer to a block that is large enough.
 */
static Header *morecore(size_t nunits) {
	// nalloc based on page size
	size_t nalloc = mm_units(mem_pagesize());

	// get at least a page-size of chunk from the OS
    if (nunits < nalloc) {
    	nunits = nalloc;
    }

    // sbrk specified number of bytes
    size_t nbytes = mm_bytes(nunits);
    void *cp = (void *) mem_sbrk(nbytes);
    if (cp == (void *) -1) {                 /* no space at all */
        return NULL;
    }

    // initialize new block header and footer
    Header *bp = mm_block(cp);   // adjust for old epilogue
    bp[0].s.blksize = bp[nunits-1].s.blksize = nunits;
    bp[0].s.isalloc = bp[nunits-1].s.isalloc = 0;

    // add epilogue header
	bp[nunits].s.blksize = 1;  // add new epilogue header
	bp[nunits].s.isalloc = 1;

	/* add the new space to free list */
    put_free_block(bp);

    return freep;  // set by put_free_block()
}

/**
 * Print the free list (educational purpose)
 *
 * @msg the initial message to print
 */
void visualize(const char* msg) {
    fprintf(stderr, "\n--- Free list after \"%s\":\n", msg);

    if (freep == NULL) {                   /* does not exist */
        fprintf(stderr, "    List does not exist\n\n");
        return;
    }

    if (freep == freep[1].blkp) {          /* self-pointing list = empty */
        fprintf(stderr, "    List is empty\n\n");
        return;
    }

    Header *tmp = freep;
    char *str = "    ";
    do {           /* traverse the list */
		fprintf(stderr, "0x%p: %s blocks: %zu alloc: %d prev: 0x%p next: 0x%p\n", tmp, str, tmp[0].s.blksize, tmp[0].s.isalloc, tmp[1].blkp, tmp[2].blkp);
		str = " -> ";
		tmp = tmp[2].blkp;
    }  while (tmp != freep);
    fprintf(stderr, "--- end\n\n");
}


/**
 * Calculate the total amount of available free memory
 * excluding headers and footers.
 *
 * @return the amount of free memory in bytes
 */
size_t mm_getfree(void) {
    if (freep == NULL) {
        return 0;
    }

    Header *tmp = freep;
    size_t res = tmp[0].s.blksize;

    while (tmp[1].blkp != freep) {
    	if (tmp[0].s.isalloc == 0) {  // not dummy node
			res += tmp[0].s.blksize - 2;  // not headers/footers
			tmp = tmp[1].blkp;
    	}
    }

    return mm_bytes(res);
}
