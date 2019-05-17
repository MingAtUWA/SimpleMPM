#ifndef _MEMORY_UTILITIES_ITEM_ARRAY_HPP_
#define _MEMORY_UTILITIES_ITEM_ARRAY_HPP_

#include <string>

namespace MemoryUtilities
{
	/*===========================================
	Class ItemArray
	---------------------------------------------
	No memory allignment
	Resize by approximately 2 fold each
	time run out of memory.
	Note:
	1. "item_size" must be larger than 0.
	2. The memory address *changes* dynamically
	with reallocation.
	============================================*/
	template<typename Item>
	class ItemArray
	{
	protected:
		Item *mem;
		Item *pos;
		Item *end;

	public:
		ItemArray(size_t init_size = 0) :
			mem(nullptr), pos(nullptr), end(nullptr) { reserve(init_size); }
		~ItemArray() { clear(); }

		inline Item *get_mem(void) const { return mem; }
		inline size_t get_num(void) const { return size_t(pos - mem); }
		inline size_t get_capacity(void) const { return size_t(end - mem); }
		inline Item *get_nth_item(size_t id) { return mem + id; }
		inline Item &operator[] (size_t id) { return mem[id]; }

		// allocate memory for new item
		inline Item *alloc(void)
		{
			if (pos < end)
				return pos++;
			// increase the capacity by around 2 fold
			reserve(get_capacity() * 2 + 1);
			return pos++;
		}
		inline void add(Item &data)
		{
			if (pos < end)
			{
				*(pos++) = data;
				return;
			}
			// increase the capacity by around 2 fold
			reserve(get_capacity() * 2 + 1);
			*(pos++) = data;
		}
		inline void add(Item *data)
		{
			if (pos < end)
			{
				*(pos++) = *data;
				return;
			}
			// increase the capacity by around 2 fold
			reserve(get_capacity() * 2 + 1);
			*(pos++) = *data;
		}
		void reserve(size_t reserve_num)
		{
			if (reserve_num > get_capacity())
			{
				size_t cur_num = size_t(pos - mem);
				Item *tmp = new Item[reserve_num];
				if (mem)
				{
					if (cur_num)
						memcpy(tmp, mem, size_t(pos) - size_t(mem));
					delete[] mem;
				}
				mem = tmp;
				pos = mem + cur_num;
				end = mem + reserve_num;
			}
		}
		inline void expand(size_t expand_num)
		{
			reserve(get_capacity() + expand_num);
		}
		inline void reset(void) { pos = mem; }
		// clear memory
		void clear(void)
		{
			if (mem)
			{
				delete[] mem;
				mem = nullptr;
			}
			pos = nullptr;
			end = nullptr;
		}
		// get memory
		operator Item*()
		{
			return reinterpret_cast<Item *>(mem);
		}
	};
}

#endif