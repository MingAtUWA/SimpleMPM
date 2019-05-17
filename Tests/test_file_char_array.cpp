#include "Test_pcp.h"

#include "FileCharArray.h"
#include "test_post_processor.h"

void test_file_char_array(void)
{
	//FileCharArray file("test.txt");
	FileCharArray file("test2.txt");
	if (!file.is_valid())
		return;
	size_t size =  file.get_file_size();
	for (size_t i = 0; i < size + 1; i++)
	{
		char dd = file[i];
	}
	if (!file.is_valid())
		return;
	for (size_t i = 0; i < 100; i++)
	{
		std::cout << file[i];
	}
}
