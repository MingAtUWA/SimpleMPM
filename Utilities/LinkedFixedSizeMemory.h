#ifndef _LINKED_FIXED_SIZE_MEMORY_H_
#define _LINKED_FIXED_SIZE_MEMORY_H_

#include <cassert>

/*=============================================================
Class LinkedFixedSizeMemory
	1. Each class maintain a list of memory blocks;
	2. Each memory blocks has its own memory pool;
	3. Each memory pools is an array of fixed size memory slot.
 =============================================================*/
class LinkedFixedSizeMemory
{
protected:
	// header of each memory blocks
	struct MemBlockHeader
	{
		size_t slot_num;
		MemBlockHeader *next;
	};
	// header of each slots in memory block
	struct MemSlotHeader
	{
		union
		{
			/* pointer and size_t have same
			   size in 32bit and 64bit os. */
			MemSlotHeader *next;
			size_t mem_id;
		};
	};

protected:
	// used to initialize mem_id;
	static size_t cur_mem_id;
	// identifier of this memory
	const size_t mem_id;

	// size of each slot in memory pool 
	const size_t slot_size;

	// total memory size (include all memory block)
	size_t total_slot_num;
	// list for memory block
	MemBlockHeader *mem_block_list;
	// number of empty slot
	size_t empty_slot_num;
	// list for empty slot that can be allocated
	MemSlotHeader *empty_slot_list;

public:
	LinkedFixedSizeMemory(size_t item_size);
	~LinkedFixedSizeMemory();

	// get new item
	inline void *alloc(void)
	{
		MemSlotHeader *pmem_slot_tmp;

		// allocate space if no empty slot
		if (!empty_slot_list && expand(total_slot_num + 1)) return nullptr;

		pmem_slot_tmp = empty_slot_list;
		empty_slot_list = empty_slot_list->next;
		pmem_slot_tmp->mem_id = mem_id;
		--empty_slot_num;
		return (char *)pmem_slot_tmp + sizeof(MemSlotHeader);
	}
	// free item
	inline void free(void *item)
	{
		assert(item);
		MemSlotHeader *pslot = (MemSlotHeader *)((char *)item - sizeof(MemSlotHeader));
		// check if item belongs to this block
		if (pslot->mem_id != mem_id) return;
		pslot->next = empty_slot_list;
		empty_slot_list = pslot;
		++empty_slot_num;
	}

	// expand memory by item_num
	int expand(size_t item_num);
	// release all memory
	void release(void);
	// get rid of uncessary (and the smallest) memory block
	int squeeze(void);
};

#endif