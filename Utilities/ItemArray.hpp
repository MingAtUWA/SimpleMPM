#ifndef _MEMORY_UTILITIES_ITEM_ARRAY_HPP_
#define _MEMORY_UTILITIES_ITEM_ARRAY_HPP_

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
	/*=============================================================
	Class ItemArray
	---------------------------------------------------------------
	* Continuous memory address, iterable;
	* Resize by approximately 2 fold after reallocation;
	* Memory address subjects to **change** after reallocation;
	* Can emulate stack with push(), pop() or alloc(), free().
	==============================================================*/
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
		inline Item *get_nth_item(size_t id) const { return mem + id; }
		inline Item &operator[] (size_t id) const { return mem[id]; }

		// allocate memory for new item
		inline Item *alloc(void)
		{
			if (pos < end)
				return pos++;
			// increase the capacity by around 2 fold
			reserve(get_capacity() * 2 + 1);
			return pos++;
		}
		// allocate memory for multiple items
		inline Item *alloc(size_t num)
		{
			size_t capacity = get_capacity();
			if (pos + num > end)
				reserve(num > capacity ? capacity + num : capacity * 2 + 1);
			Item *tmp = pos;
			pos += num;
			return tmp;
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
		// add chunk of data, call copy function
		inline void add(size_t num, Item *data)
		{
			size_t capacity = get_capacity();
			if (pos + num > end)
				reserve(num > capacity ? capacity + num : capacity * 2 + 1);
			for (size_t i = 0; i < num; ++i)
				pos[i] = data[i];
			pos += num;
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

#undef MEMORY_ALIGNMENT
#undef MEMORY_ALIGNMENT_PADDING

#endif