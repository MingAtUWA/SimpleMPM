#ifndef _MEMORY_UTILITIES_LINKED_MEMORY_POOL_H_
#define _MEMORY_UTILITIES_LINKED_MEMORY_POOL_H_

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
	struct LinkedMemoryPool
	{
		struct Header
		{
			size_t size;
			Header *next;
		};
		Header *head;

		LinkedMemoryPool() : head(nullptr) {}
		~LinkedMemoryPool() { clear(); }
		
		void *add_pool(size_t size)
		{
			if (!size) return nullptr;

			size_t chunk_size = size + sizeof(Header)
							  + MEMORY_ALIGNMENT_PADDING(sizeof(Header));
			char *mem = new char[chunk_size];
			Header *header = reinterpret_cast<Header *>(mem);
			header->size = size;
			header->next = head;
			head = header;
			return get_mem_from_header(header);
		}
		// add memory pool after "current pool"
		void *add_pool(size_t size, Header *cur_pool)
		{
			if (!size) return nullptr;

			size_t chunk_size = size + sizeof(Header)
								+ MEMORY_ALIGNMENT_PADDING(sizeof(Header));
			char *mem = new char[chunk_size];
			Header *header = reinterpret_cast<Header *>(mem);
			header->size = size;
			header->next = cur_pool->next;
			cur_pool->next = header;
			return get_mem_from_header(header);
		}

		void clear(void)
		{
			Header *tmp;
			while (head)
			{
				tmp = head;
				head = head->next;
				delete[] tmp;
			}
		}

		inline void *get_mem_from_header(Header *header) const
		{
			return reinterpret_cast<char *>(header)
				+ sizeof(Header)
				+ MEMORY_ALIGNMENT_PADDING(sizeof(Header));
		}
	};
}

#undef MEMORY_ALIGNMENT
#undef MEMORY_ALIGNMENT_PADDING

#endif