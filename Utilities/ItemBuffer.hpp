#ifndef _MEMORY_UTILITIES_ITEM_BUFFER_H_
#define _MEMORY_UTILITIES_ITEM_BUFFER_H_

#include "LinkedMemoryPool.h"

namespace MemoryUtilities
{
	/*===============================================
	Class ItemBuffer
	-------------------------------------------------
	This buffer just store item and is not iterable.
	Use linked memory pool.
	Note:
	1. "default_pool_item_num" must be larger than 0.
	================================================*/
	template<typename Item>
	class ItemBuffer
	{
	protected:
		struct EmptySlot
		{
			EmptySlot *next;
		};
		union Slot
		{
			Item item;
			EmptySlot *next;
		};

		LinkedMemoryPool mem;
		// number of item in new pool
		size_t default_pool_item_num;
		// assist in allocation
		LinkedMemoryPool::Header *cur_pool;
		LinkedMemoryPool::Header *next_pool;
		char *pos, *end;

		// head of allocated item
		EmptySlot *empty;

	public:
		ItemBuffer(size_t default_num = 1) : 
			default_pool_item_num(default_num),
			cur_pool(nullptr), next_pool(nullptr),
			pos(nullptr), end(nullptr),
			empty(nullptr) {}
		~ItemBuffer() { clear(); }

		inline void set_default_pool_item_num(size_t default_num)
		{
			default_pool_item_num = default_num;
		}

		// allocate memory for new item
		inline Item *alloc(void)
		{
			Item *tmp;
			if (empty)
			{
				tmp = reinterpret_cast<Item *>(empty);
				empty = empty->next;
			}
			else if (pos < end)
			{
				tmp = reinterpret_cast<Item *>(pos);
				pos += sizeof(Slot);
			}
			else
			{
				tmp = alloc_from_other_pool();
			}
			return tmp;
		}

		inline void del(Item *pitem)
		{
			EmptySlot *tmp = reinterpret_cast<EmptySlot *>(pitem);
			// add to empty list
			tmp->next = empty;
			empty = tmp;
		}

		void reset(void)
		{
			if (mem.head)
			{
				cur_pool = mem.head;
				next_pool = cur_pool->next;
				pos = static_cast<char *>(mem.get_mem_from_header(cur_pool));
				end = pos + cur_pool->size;
			}
			else
			{
				cur_pool = nullptr;
				next_pool = nullptr;
				pos = nullptr;
				end = nullptr;
			}
			empty = nullptr;
		}

		void clear(void)
		{
			mem.clear();
			cur_pool = nullptr;
			next_pool = nullptr;
			pos = nullptr;
			end = nullptr;
			empty = nullptr;
		}

	private:
		// allocate memory from other memory pool
		Item *alloc_from_other_pool(void)
		{
			Item *tmp;
			if (next_pool)
			{
				cur_pool = next_pool;
				next_pool = cur_pool->next;
				pos = static_cast<char *>(mem.get_mem_from_header(cur_pool));
				end = pos + cur_pool->size;
				tmp = reinterpret_cast<Item *>(pos);
				pos += sizeof(Slot);
				return tmp;
			}

			if (cur_pool)
			{
				pos = static_cast<char *>(mem.add_pool(default_pool_item_num * sizeof(Slot), cur_pool));
				cur_pool = cur_pool->next;
			}
			else // the first time
			{
				pos = static_cast<char *>(mem.add_pool(default_pool_item_num * sizeof(Slot)));
				cur_pool = mem.head;
			}
			next_pool = cur_pool->next;
			end = pos + cur_pool->size;
			tmp = reinterpret_cast<Item *>(pos);
			pos += sizeof(Slot);
			return tmp;
		}
	};
}

#endif