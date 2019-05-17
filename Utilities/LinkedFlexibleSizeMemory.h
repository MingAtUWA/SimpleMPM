#ifndef _LINKED_FLEXIBLE_SIZE_MEMORY_H_
#define _LINKED_FLEXIBLE_SIZE_MEMORY_H_

#include <cassert>

// must be power of two
#ifndef MEMORY_ALIGNMENT
#define MEMORY_ALIGNMENT sizeof(char *)
#endif

// round up the cloest power of two
#ifndef MEMORY_ALIGNMENT_PADDING
#define MEMORY_ALIGNMENT_PADDING(address) \
	((MEMORY_ALIGNMENT - ((address) & (MEMORY_ALIGNMENT - 1))) & (MEMORY_ALIGNMENT - 1))
#endif

#ifndef MEMORY_POOL_HEADER_SIZE
#define MEMORY_POOL_HEADER_SIZE \
	(sizeof(MemoryPoolHeader) + MEMORY_ALIGNMENT_PADDING(sizeof(MemoryPoolHeader)))
#endif

/*
Cannot free memory
*/
class LinkedFlexibleSizeMemory
{
public:
	LinkedFlexibleSizeMemory(size_t init_chunk_size = 0) :
		top_pool(nullptr), total_pool_size(0),
		pool_cur_pos(nullptr), pool_end_pos(nullptr)
	{
		if (init_chunk_size)
			expand(init_chunk_size);
	}
	~LinkedFlexibleSizeMemory() { clear(); }

	inline void *alloc(size_t size)
	{
		assert(size);

		pool_cur_pos += MEMORY_ALIGNMENT_PADDING(size_t(pool_cur_pos));
		char *res = pool_cur_pos;
		pool_cur_pos += size;
		if (pool_cur_pos < pool_end_pos)
			return res;

		size_t expand_size = size > total_pool_size ? size : total_pool_size;
		// approximately expand by 2 fold
		expand(expand_size);
		res = pool_cur_pos;
		pool_cur_pos += size;
		return res;
	}

	void clear(void)
	{
		MemoryPoolHeader *tmp_pool;
		while (top_pool)
		{
			tmp_pool = top_pool;
			top_pool = top_pool->prev_pool;
			delete[] tmp_pool;
		}
	}

private:
	void expand(size_t chunk_size)
	{
		if (!chunk_size) return;

		total_pool_size += chunk_size;
		chunk_size += MEMORY_POOL_HEADER_SIZE;
		char *pool = new char[chunk_size];
		MemoryPoolHeader *header = reinterpret_cast<MemoryPoolHeader *>(pool);
		header->prev_pool = top_pool;
		top_pool = header;

		cur_pool = header;
		pool_cur_pos = pool + MEMORY_POOL_HEADER_SIZE;
		pool_end_pos = pool + chunk_size;
	}

	struct MemoryPoolHeader
	{
		MemoryPoolHeader *prev_pool;
	};
	MemoryPoolHeader *top_pool;
	size_t total_pool_size;

	MemoryPoolHeader *cur_pool;
	char *pool_cur_pos;
	char *pool_end_pos;
};

#undef MEMORY_ALIGNMENT
#undef MEMORY_ALIGNMENT_PADDING
#undef MEMORY_POOL_HEADER_SIZE

#endif