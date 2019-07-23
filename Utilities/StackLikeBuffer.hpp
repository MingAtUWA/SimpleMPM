#ifndef _STACK_LIKE_BUFFER_HPP_
#define _STACK_LIKE_BUFFER_HPP_

namespace MemoryUtilities
{

/***************************************
 * StackLikeBuffer
 * can alloc and reset
 * cannot pop or free
 ***************************************/
template <class Item>
class StackLikeBuffer
{
protected:
	struct MemPoolHeader
	{
		MemPoolHeader *next;
		size_t size;
		Item *mem;
	};
	MemPoolHeader *top_pool; // memory pools managed as stack
	size_t pool_size; // default size of new memory pool
	
	size_t pool_num; // number of memory pool allocated
	size_t total_size; // total capacity of all memory pool

	MemPoolHeader *next_pool;
	Item *cur, *end;

public:
	StackLikeBuffer(size_t default_pool_size) :
		top_pool(nullptr), pool_size(default_pool_size),
		pool_num(0), total_size(0),
		next_pool(nullptr), cur(nullptr), end(nullptr) {}
	~StackLikeBuffer() { clear(); }

	inline void set_default_pool_size(size_t default_pool_size) { pool_size = default_pool_size; }

	inline Item *alloc(size_t num)
	{
		Item *res;
		res = cur;
		cur += num;
		if (cur > end) // not enough memory in this slot
		{
			while (next_pool)
			{
				if (next_pool->size < num)
				{
					next_pool = next_pool->next;
					continue;
				}
				cur = next_pool->mem;
				end = cur + next_pool->size;
				next_pool = next_pool->next;
				goto memory_found;
			}
			new_mem_pool(num);
			next_pool = nullptr;
			cur = top_pool->mem;
			end = cur + top_pool->size;
		memory_found:	
			res = cur;
			cur += num;
		}
		return res;
	}

	// clear items in buffer
	// preserve allocated memory
	inline void reset(void)
	{
		// reset and delete all items
		if (top_pool)
		{
			next_pool = top_pool->next;
			cur = top_pool->mem;
			end = cur + top_pool->size;
		}
		else
		{
			next_pool = nullptr;
			cur = nullptr;
			end = nullptr;
		}
	}
	// Optimize performance by reallocating
	// the whole chunk of memory
	inline void reset_optimize(void)
	{
		// optimize performance
		if (pool_num > 1)
		{
			size_t merged_pool_size = total_size;
			clear();
			new_mem_pool(merged_pool_size);
		}
		// reset and delete all items
		if (top_pool)
		{
			next_pool = top_pool->next;
			cur = top_pool->mem;
			end = cur + top_pool->size;
		}
		else
		{
			next_pool = nullptr;
			cur = nullptr;
			end = nullptr;
		}
	}

	// clear allocated memory
	void clear(void)
	{
		char *tmp;
		while (top_pool)
		{
			tmp = reinterpret_cast<char *>(top_pool);
			top_pool = top_pool->next;
			delete[] tmp;
		}
		pool_num = 0;
		total_size = 0;
		next_pool = nullptr;
		cur = nullptr;
		end = nullptr;
	}

protected:
// must be power of two
#define MEMORY_ALIGNMENT sizeof(void *)
// round up the cloest power of two
#define MEMORY_ALIGNMENT_PADDING(address) ((MEMORY_ALIGNMENT - ((address) & (MEMORY_ALIGNMENT - 1))) & (MEMORY_ALIGNMENT - 1))

	// allocate new memeory pool with size >= "size"
	void new_mem_pool(size_t size)
	{
		if (size == 0) return;
		if (size < pool_size)
			size = pool_size;
		size_t mem_size = sizeof(MemPoolHeader) + size * sizeof(Item) + MEMORY_ALIGNMENT;
		char *mem = new char[mem_size];
		MemPoolHeader *pool_header = reinterpret_cast<MemPoolHeader *>(mem);
		pool_header->next = top_pool;
		top_pool = pool_header;
		pool_header->size = size;
		// memory alignment
		mem += sizeof(MemPoolHeader);
		mem += MEMORY_ALIGNMENT_PADDING(size_t(mem));
		pool_header->mem = reinterpret_cast<Item *>(mem);
		++pool_num;
		total_size += size;
	}
#undef MEMORY_ALIGNMENT
#undef MEMORY_ALIGNMENT_PADDING
};

};

#endif