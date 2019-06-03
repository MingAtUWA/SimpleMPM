#ifndef _MEMORY_UTILITIES_ITEM_LIST_HPP_
#define _MEMORY_UTILITIES_ITEM_LIST_HPP_

#include <string>
#include "LinkedMemoryPool.h"

namespace MemoryUtilities
{
	/*===========================================
	Class ItemList
	---------------------------------------------
	Double linked list
	Note:
	1. "default_pool_size" must be larger than 0.
	2. Use linked memory pool.
	===========================================*/
	template<typename Item>
	class ItemList
	{
	protected:
		struct Slot
		{
			Slot *prev;
			Slot *next;
			Item item;
		};
		struct SlotTag
		{
			Slot *prev;
			Slot *next;
			SlotTag() : 
				prev(reinterpret_cast<Slot *>(this)),
				next(reinterpret_cast<Slot *>(this)) {}
		};

		LinkedMemoryPool mem;
		// number of item in new pool
		size_t default_pool_item_num;
		// assist in allocation
		LinkedMemoryPool::Header *cur_pool;
		LinkedMemoryPool::Header *next_pool;
		char *pos, *end;

		// head of allocated item
		SlotTag head;

		// stack for empty slot
		Slot *empty;

	public:
		ItemList(size_t default_num = 1) : 
			default_pool_item_num(default_num),
			cur_pool(nullptr), next_pool(nullptr),
			pos(nullptr), end(nullptr),
			empty(nullptr) {}
		~ItemList() { clear(); }

		inline void set_default_pool_item_num(size_t default_num)
		{
			if (default_num) default_pool_item_num = default_num;
		}

		inline Item *alloc(void)
		{
			Slot *tmp;
			if (empty)
			{
				tmp = empty;
				empty = empty->prev;
			}
			else if (pos < end)
			{
				tmp = (Slot *)pos;
				pos += sizeof(Slot);	
			}
			else
			{
				tmp = alloc_from_other_pool();
			}
			append_to_list(tmp);
			return &(tmp->item);
		}

		inline void del(Item *pitem)
		{
			Slot *tmp;
			tmp = (Slot *)((char *)pitem - offsetof(Slot, item));
			del_from_list(tmp);
			// add to empty list
			tmp->prev = empty;
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
			head.prev = reinterpret_cast<Slot *>(&head);
			head.next = reinterpret_cast<Slot *>(&head);
		}

		void clear(void)
		{
			mem.clear();
			cur_pool = nullptr;
			next_pool = nullptr;
			pos = nullptr;
			end = nullptr;
			empty = nullptr;
			head.prev = reinterpret_cast<Slot *>(&head);
			head.next = reinterpret_cast<Slot *>(&head);
		}
		
		inline Item *first(void) const { return &(head.next->item); }
		inline Item *next(Item *cur) const
		{
#define NEXT_POINTER(cur) ((Slot *)(*(char **)((char *)(cur)-(offsetof(Slot, item) - offsetof(Slot, next)))))
			return &(NEXT_POINTER(cur)->item);
#undef  NEXT_POINTER
		}
		inline Item *last(void) const { return &(head.prev->item); }
		inline Item *prev(Item *cur) const
		{
#define PREV_POINTER(cur) ((Slot *)(*(char **)((char *)(cur)-(offsetof(Slot, item) - offsetof(Slot, prev)))))
			return &(PREV_POINTER(cur)->item);
#undef  PREV_POINTER
		}
		// end of iteration
		inline Item *eoi(void) const { return (Item *)((char *)&head + offsetof(Slot, item)); }
	
	protected:
		// add at the end of the list
		inline void append_to_list(Slot *cur)
		{
			cur->next = reinterpret_cast<Slot *>(&head);
			cur->prev = head.prev;
			head.prev->next = cur;
			head.prev = cur;
		}
		// add at the beginning of the list
		inline void prepend_to_list(Slot *cur)
		{
			cur->prev = reinterpret_cast<Slot *>(&head);
			cur->next = head.next;
			head.next->prev = cur;
			head.next = cur;
		}
		inline void del_from_list(Slot *cur)
		{
			cur->next->prev = cur->prev;
			cur->prev->next = cur->next;
		}

	private:
		// allocate memory from other memory pool
		Slot *alloc_from_other_pool(void)
		{
			Slot *tmp;
			if (next_pool)
			{
				cur_pool = next_pool;
				next_pool = cur_pool->next;
				pos = static_cast<char *>(mem.get_mem_from_header(cur_pool));
				end = pos + cur_pool->size;
				tmp = reinterpret_cast<Slot *>(pos);
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
			tmp = reinterpret_cast<Slot *>(pos);
			pos += sizeof(Slot);
			return tmp;
		}
	};
}

#endif