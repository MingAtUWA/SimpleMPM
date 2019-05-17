#include "Test_pcp.h"

#include "ItemArray.hpp"
#include "test_post_processor.h"

void test_item_array(void)
{
	MemoryUtilities::ItemArray<size_t> test_array;
	
	test_array.reserve(10);
	for (size_t i = 0; i < 10; i++)
		test_array.add(i);

	test_array.reserve(20);
}