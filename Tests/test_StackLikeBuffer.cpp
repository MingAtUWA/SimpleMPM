#include "Test_pcp.h"

#include "StackLikeBuffer.hpp"
#include "test_post_processor.h"

void test_StackLikeBuffer(void)
{
	MemoryUtilities::StackLikeBuffer<size_t> buf(2);
	size_t *tmp1[10];

	for (size_t i = 0; i < 10; ++i)
	{
		tmp1[i] = buf.alloc(1);
		*tmp1[i] = i;
	}
	for (size_t i = 0; i < 10; ++i)
		std::cout << tmp1[i] << ": " << *tmp1[i] << "\n";
	std::cout << "\n";

	buf.reset();

	tmp1[0] = buf.alloc(10);
	for (size_t i = 0; i < 10; ++i)
	{
		tmp1[0][i] = i + 10;
	}
	for (size_t i = 0; i < 10; ++i)
		std::cout << tmp1[0]+i << ": " << tmp1[0][i] << "\n";
	std::cout << "\n";

	buf.reset();
	for (size_t i = 0; i < 6; ++i)
	{
		tmp1[i] = buf.alloc(2);
		tmp1[i][0] = 2*i + 20;
		tmp1[i][1] = 2*i + 21;
	}
	for (size_t i = 0; i < 6; ++i)
	{
		std::cout << tmp1[i] << ": " << tmp1[i][0] << "\n";
		std::cout << tmp1[i]+1 << ": " << tmp1[i][1] << "\n";
	}
	std::cout << "\n";

	buf.reset_optimize();
	for (size_t i = 0; i < 10; ++i)
	{
		tmp1[i] = buf.alloc(1);
		*tmp1[i] = i;
	}
	for (size_t i = 0; i < 10; ++i)
	{
		std::cout << tmp1[i] << ": " << *tmp1[i] << "\n";
	}

	//buf.clear();
	//buf.reset();
}