#ifndef _MEMORY_UTILITIES_CONTINUOUS_FIXED_SIZE_MEMORY_H_
#define _MEMORY_UTILITIES_CONTINUOUS_FIXED_SIZE_MEMORY_H_

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

namespace MemoryUtilities
{
/*===========================================
Class ContinuousFixedSizeMemory
---------------------------------------------
	Resize by approximately 2 fold each
	time run out of memory.
	Has memory alignment.
Note:
	1. "item_size" must be larger than 0.
	2. The memory address *changes* dynamically
	   with reallocation.
 ===========================================*/
class ContinuousFixedSizeMemory
{
protected:
	const size_t item_size;
	char *mem;
	// maximum_item_num * item_size
	size_t capacity;
	// position for next allocation
	size_t pos;

public:
	ContinuousFixedSizeMemory(const size_t size_of_item, size_t init_size = 0) :
		item_size(size_of_item + MEMORY_ALIGNMENT_PADDING(size_of_item)),
		mem(nullptr), capacity(0), pos(0) { reserve(init_size); }
	~ContinuousFixedSizeMemory() { if (mem) delete[] mem; }

	inline void *get_mem(void) const { return mem; }
	inline size_t get_capacity(void) const { return capacity; }
	inline size_t get_item_size(void) const { return item_size; }
	inline size_t get_item_num(void) const { return pos / item_size; }

	inline void *get_nth_item(size_t id) { return mem + id * item_size; }
	inline void *get_first(void) const { return mem; }
	inline void *get_next(void *p_item) const { return static_cast<char *>(p_item) + item_size; };
	inline void *get_end(void) const { return mem + pos; }

	// allocate memory for new item
	inline void *alloc(void)
	{
		size_t item_pos = pos;
		pos += item_size;
		// increase the capacity by 2 fold
		if (pos > capacity)
		{
			size_t new_capacity = capacity + capacity;
			new_capacity = new_capacity > pos ? new_capacity : pos;
			reserve_raw(new_capacity);
		}
		return mem + item_pos;
	}

	// resize memory capacity to "reserve_num" items
	inline void reserve(size_t reserve_num)
	{
		reserve_raw(item_size * reserve_num);
	}
	// expand memory capacity by "expand_num" items
	inline void expand(size_t expand_num)
	{
		reserve_raw(capacity + item_size * expand_num);
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

private:
	// resize memory capacity to "reserve_size"
	void reserve_raw(size_t new_capacity)
	{
		if (new_capacity > capacity)
		{
			char *tmp = new char[new_capacity];
			if (mem)
			{
				if (pos)
					memcpy(tmp, mem, pos);
				delete[] mem;
			}
			mem = tmp;
			capacity = new_capacity;
		}
	}
};
}

#undef MEMORY_ALIGNMENT
#undef MEMORY_ALIGNMENT_PADDING

#endif