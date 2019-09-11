#include "Utilities_pcp.h"

#include <cstdlib>

#include "LinkedFixedSizeMemory.h"

size_t LinkedFixedSizeMemory::cur_mem_id = 0;

LinkedFixedSizeMemory::LinkedFixedSizeMemory(size_t item_size) :
	mem_id(++cur_mem_id), slot_size(item_size + sizeof(MemSlotHeader)),
	total_slot_num(0), mem_block_list(nullptr), 
	empty_slot_num(0), empty_slot_list(nullptr) {}

LinkedFixedSizeMemory::~LinkedFixedSizeMemory() { release(); }

int LinkedFixedSizeMemory::expand(size_t item_num)
{
	MemBlockHeader *pmem_block_tmp;
	char *pmem_pool_tmp;

	// only need to expand memory when empty slot is not enough
	if (item_num > empty_slot_num)
	{
		item_num -= empty_slot_num;

		// add memory block
		pmem_block_tmp = (MemBlockHeader *)malloc(sizeof(MemBlockHeader) + slot_size * item_num);
		if (!pmem_block_tmp) return -1;
		pmem_block_tmp->next = mem_block_list;
		mem_block_list = pmem_block_tmp;
		pmem_block_tmp->slot_num = item_num;
		total_slot_num += item_num;

		// initialize each empty slots in the pool
		pmem_pool_tmp = (char *)pmem_block_tmp + sizeof(MemBlockHeader);
		for (size_t i = 0; i < item_num; i++)
		{
			((MemSlotHeader *)pmem_pool_tmp)->next = empty_slot_list;
			empty_slot_list = (MemSlotHeader *)pmem_pool_tmp;
			pmem_pool_tmp += slot_size;
		}
		empty_slot_num += item_num;
	}
	return 0;
}

void LinkedFixedSizeMemory::release(void)
{
	MemBlockHeader *pmem_block_tmp;
	while (mem_block_list)
	{
		pmem_block_tmp = mem_block_list;
		mem_block_list = mem_block_list->next;
		free(pmem_block_tmp);
	}
	total_slot_num = 0;
	empty_slot_list = nullptr;
	empty_slot_num = 0;
}

int LinkedFixedSizeMemory::squeeze(void)
{
	if (!mem_block_list) return 0;

	/*
	size_t i;
	MemBlockHeader *pmem_block_tmp;
	char *pmem_pool;
	MemSlotHeader *pmem_slot_tmp;

	// mark all unused slots as 0
	for (pmem_slot_tmp = empty_slot_list; pmem_slot_tmp;
		 pmem_slot_tmp = pmem_slot_tmp->next)
	{
		pmem_slot_tmp->mem_id = 0;
	}
	// mark all used slots as 1
	for (pmem_block_tmp = mem_block_list; pmem_block_tmp;
		 pmem_block_tmp = pmem_block_tmp->next)
	{
		pmem_pool = (char*)pmem_block_tmp + sizeof(MemBlockHeader);
		for (i = 0; i < pmem_block_tmp->slot_num; i++)
		{
			pmem_slot_tmp = (MemSlotHeader *)pmem_pool;
			if (pmem_slot_tmp->mem_id) pmem_slot_tmp->mem_id = 1;
			pmem_pool += slot_size;
		}
	}

	// organize helper data structure
	size_t mem_block_num = 0;
	for (pmem_block_tmp = mem_block_list; pmem_block_tmp;
		 pmem_block_tmp = pmem_block_tmp->next)
	{
		++mem_block_num;
	}
	MemBlockHeader **mem_block_array = (MemBlockHeader **)malloc(sizeof(MemBlockHeader *) * mem_block_num);
	if (!mem_block_array) return -1;
	for (pmem_block_tmp = mem_block_list, i = 0; pmem_block_tmp;
		 pmem_block_tmp = pmem_block_tmp->next, i++)
	{
		mem_block_array[i] = pmem_block_tmp;
	}
	mem_block_list = nullptr;
	// sort mem_block_array according to slot_num
	for (i = 0; i < mem_block_num; i++)
	{

	}

	// squeeze the memory blocks
	size_t mem_largest_id = 0;
	size_t mem_largest_unused_slot_num = 0;
	size_t mem_smallest_id = mem_block_num - 1;
	size_t mem_smallest_used_slot_num = 0;
	while (true)
	{
		while (mem_largest_id <= mem_smallest_id)
		{
			// get number of unused slots in largest memory block
			mem_largest_unused_slot_num = mem_block_array[mem_largest_id]->slot_num;
			pmem_pool = (char *)mem_block_array[mem_largest_id] + sizeof(MemBlockHeader);
			for (i = 0; i < mem_block_array[mem_largest_id]->slot_num; i++)
			{
				if (((MemSlotHeader *)pmem_pool)->mem_id) --mem_largest_unused_slot_num;
				pmem_pool += slot_size;
			}
			
			if (mem_largest_unused_slot_num) break;
			
			// add back to list
			mem_block_array[mem_largest_id]->next = mem_block_list;
			mem_block_list = mem_block_array[mem_largest_id];
			mem_block_array[mem_largest_id] = nullptr;
			
			++mem_largest_id;
		}

		while(mem_largest_id <= mem_smallest_id)
		{
			// get number of used slots in smallest memory block
			mem_smallest_used_slot_num = mem_block_array[mem_smallest_id]->slot_num;
			pmem_pool = (char *)mem_block_array[mem_smallest_id] + sizeof(MemBlockHeader);
			for (i = 0; i < mem_block_array[mem_smallest_id]->slot_num; i++)
			{
				if (((MemSlotHeader *)pmem_pool)->mem_id) ++mem_smallest_used_slot_num;
				pmem_pool += slot_size;
			}

			if (mem_smallest_used_slot_num) break;

			// free memory block
			free(mem_block_array[mem_smallest_id]);
			mem_block_array[mem_smallest_id] = nullptr;

			--mem_smallest_id;
		}

		//// complete if no more largest or smallest memory block
		//if (mem_largest_id >= mem_smallest_id)
		//{
		//	if (mem_block_array[mem_largest_id])
		//	{
		//		mem_block_array[mem_largest_id]->next = mem_block_list;
		//		mem_block_list = mem_block_array[mem_largest_id];
		//	}
		//	free(mem_block_array);
		//	break;
		//}
		
		// move slot from pmem_block_smallest to pmem_block_largest


		// post-process
		if (!mem_largest_unused_slot_num)
		{
			mem_block_array[mem_largest_id]->next = mem_block_list;
			mem_block_list = mem_block_array[mem_largest_id];
			mem_block_array[mem_largest_id] = nullptr;
			++mem_largest_id;
		}

		if (!mem_smallest_used_slot_num)
		{
			free(mem_block_array[mem_smallest_id]);
			mem_block_array[mem_smallest_id] = nullptr;
			--mem_smallest_id;
		}
	}

	// recover empty slot list and mem_id fields of each slots
	empty_slot_num = 0;
	for (pmem_block_tmp = mem_block_list; pmem_block_tmp;
		pmem_block_tmp = pmem_block_tmp->next)
	{
		pmem_pool = (char*)pmem_block_tmp + sizeof(MemBlockHeader);
		for (size_t i = 0; i < pmem_block_tmp->slot_num; i++)
		{
			pmem_slot_tmp = (MemSlotHeader *)pmem_pool;
			if (pmem_slot_tmp->mem_id) // used
			{
				pmem_slot_tmp->mem_id = mem_id;
			}
			else // unused
			{
				pmem_slot_tmp->next = empty_slot_list;
				empty_slot_list = pmem_slot_tmp;
				++empty_slot_num;
			}
			pmem_pool += slot_size;
		}
	}
	*/
	return 0;
}