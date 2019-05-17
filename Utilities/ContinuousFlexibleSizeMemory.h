#ifndef _MEMORY_UTILITIES_CONTINUOUS_FLEXIBLE_SIZE_MEMORY_H_
#define _MEMORY_UTILITIES_CONTINUOUS_FLEXIBLE_SIZE_MEMORY_H_

#include <string>

// must be power of two
#ifndef MEMORY_ALIGNMENT
#define MEMORY_ALIGNMENT sizeof(void *)
#endif

// round up the cloest power of two
#ifndef MEMORY_ALIGNMENT_PADDING
#define MEMORY_ALIGNMENT_PADDING(address) \
	((MEMORY_ALIGNMENT - ((address) & (MEMORY_ALIGNMENT - 1))) & (MEMORY_ALIGNMENT - 1))
#endif

#ifndef ITEM_HEADER_ALIGNED_SIZE
#define ITEM_HEADER_ALIGNED_SIZE (sizeof(size_t) + MEMORY_ALIGNMENT_PADDING(sizeof(size_t)))
#endif

// problematic!!!! needed to be changed
namespace MemoryUtilities
{
/*=============================================
Class ContinuousFixedSizeMemory
 ----------------------------------------------
	Resize by approximately 2 fold each time
	run out of memory.
Note:
	1. "item_size" must be larger than 0.
	2. The memory address *changes* dynamically
	   with reallocation.
 ==============================================*/
class ContinuousFlexibleSizeMemory
{
protected:
	char *mem;
	size_t capacity;
	// position for next allocation
	size_t pos;

public:
	ContinuousFlexibleSizeMemory(size_t init_size = 0) :
		mem(nullptr), capacity(0), pos(0) { reserve(init_size); }
	~ContinuousFlexibleSizeMemory() { if (mem) delete[] mem; }

	inline void *get_mem(void) const { return mem; }
	inline size_t get_capacity(void) const { return capacity; }
	inline void *get_first(void) const { return mem + sizeof(size_t); }
	inline void *get_next(void *p_item) const
	{
		size_t offset = *reinterpret_cast<size_t *>((char *)p_item - ITEM_HEADER_ALIGNED_SIZE);
		return (char *)p_item + offset + sizeof(size_t);
	};
	inline void *get_end(void) const { return mem + pos; }

	// allocate memory for new item
	inline void *alloc(const size_t item_size)
	{
		pos += MEMORY_ALIGNMENT_PADDING(pos) + ITEM_HEADER_ALIGNED_SIZE;
		size_t item_pos = pos;
		pos += item_size;
		// increase the capacity by 2 fold
		if (pos > capacity)
		{
			size_t new_capacity = capacity + capacity;
			new_capacity = new_capacity > pos ? new_capacity : pos;
			reserve(new_capacity);
		}
		*reinterpret_cast<size_t *>(mem + item_pos - ITEM_HEADER_ALIGNED_SIZE) = item_size;
		return mem + item_pos;
	}

	// resize memory capacity to size
	void reserve(size_t reserve_size)
	{
		if (reserve_size > capacity)
		{
			char *tmp = new char[reserve_size];
			if (mem)
			{
				if (pos)
					memcpy(tmp, mem, pos);
				delete[] mem;
			}
			mem = tmp;
			capacity = reserve_size;
		}
	}
	// expand memory capacity by size
	inline void expand(size_t expand_size)
	{
		reserve(capacity + expand_size);
	}
	// reset memory (clear data without releasing memory)
	inline void reset(void) { pos = 0; }
	// clear memory
	void clear(void)
	{
		if (mem)
		{
			delete[] mem;
			mem = nullptr;
			capacity = 0;
			pos = 0;
		}
	}
};

};

#undef MEMORY_ALIGNMENT
#undef MEMORY_ALIGNMENT_PADDING
#undef ITEM_HEADER_ALIGNED_SIZE

#endif