#ifndef _MEMORY_UTILITIES_ITEM_STACK_HPP_
#define _MEMORY_UTILITIES_ITEM_STACK_HPP_

#include <string>
#include "LinkedMemoryPool.h"

namespace MemoryUtilities
{
	/*================================================
	Class ItemStack
	--------------------------------------------------
	Note:
	1. "default_pool_size" must be larger than 0.
	2. Use linked memory pool.
	3. Iteration: 
		for (cur = top(); cur; cur = prev(cur)) {..}
	=================================================*/

	template<typename Item>
	class ItemStack
	{
	protected:
		struct Slot
		{
			Item item;
			Slot *next;
		};

		LinkedMemoryPool mem;
		// number of item in new pool
		size_t default_pool_item_num;
		// assist in allocation
		LinkedMemoryPool::Header *cur_pool;
		LinkedMemoryPool::Header *next_pool;
		char *pos, *end;

		// head of allocated item
		Slot *head; 
		
	public:
		ItemStack(size_t default_num = 1) :
			default_pool_item_num(default_num),
			cur_pool(nullptr), next_pool(nullptr),
			pos(nullptr), end(nullptr),
			head(nullptr) {}
		~ItemStack() { clear(); }
		
		inline void set_default_pool_item_num(size_t default_num)
		{
			if (default_num) default_pool_item_num = default_num;
		}

		// allocate memory for new item
		inline Item *alloc(void)
		{
			if (pos < end)
			{
				Slot *tmp = reinterpret_cast<Slot *>(pos);
				pos += sizeof(Slot);
				tmp->next = head;
				head = tmp;
				return &(tmp->item);
			}
			return alloc_from_other_pool();
		}

		void reset(void)
		{
			if (mem.head)
			{
				cur_pool = mem.head;
				next_pool = cur_pool->next;
				pos = static_cast<char *>(mem.get_mem_from_header(cur_pool));
				end = pos + cur_pool->size;
				head = nullptr;
			}
			else
			{
				cur_pool = nullptr;
				next_pool = nullptr;
				pos = nullptr;
				end = nullptr;
				head = nullptr;
			}
		}

		// clear memory
		void clear(void)
		{
			mem.clear();
			cur_pool = nullptr;
			next_pool = nullptr;
			pos = nullptr;
			end = nullptr;
			head = nullptr;
		}

		inline Item *top(void) const
		{
			return reinterpret_cast<Item *>(head);
		}
		inline Item *prev(Item *cur) const
		{
			return reinterpret_cast<Item *>(reinterpret_cast<Slot *>(cur)->next);
		}
		inline Item *eoi(void) const { return nullptr; }

	private:
		Item *alloc_from_other_pool(void)
		{
			if (next_pool)
			{
				cur_pool = next_pool;
				next_pool = cur_pool->next;
				pos = static_cast<char *>(mem.get_mem_from_header(cur_pool));
				end = pos + cur_pool->size;
				Slot *tmp = reinterpret_cast<Slot *>(pos);
				pos += sizeof(Slot);
				tmp->next = head;
				head = tmp;
				return &(tmp->item);
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
			Slot *tmp = reinterpret_cast<Slot *>(pos);
			pos += sizeof(Slot);
			tmp->next = head;
			head = tmp;
			return &(tmp->item);
		}
	};
}

#endif