#include "Test_pcp.h"

#include <iostream>

#include "PreAllocStringBuffer.hpp"
#include "StringBuffer.h"
#include "MemoryBuffer.h"
#include "ContinuousFixedSizeMemory.h"
#include "FileCharArray.h"

#include "TextResultFileParser.hpp"
#include "FieldDataParser.hpp"

#include "test_post_processor.h"

void test_result_file_parser(void)
{
	TextResultFileInfo res_file_info;
	TextResultFile::Parser<PFileCharArray> parser;

	const char *th_file1_path = "res_file\\TimeHistory1-test_out1.txt";
	FileCharArray th_file1;
	th_file1.open_file(th_file1_path);
	if (!th_file1.is_valid())
		return;

	parser.parse(&res_file_info, th_file1);

	const char *ms_file1_path = "res_file\\ModelState.txt";
	FileCharArray ms_file1(ms_file1_path);
	if (!ms_file1.is_valid())
		return;

	parser.parse(&res_file_info, ms_file1);

	res_file_info.print(std::cout);

	TimeHistoryNode *time_history = res_file_info.first_time_history();
	TimeRecordNode *time_rcd = time_history->first_time_rcd();
	//th_file1.open_file(th_file1_path);
	
	//for (size_t i = time_rcd->field_data.begin_pos;
	//	 i < time_rcd->field_data.end_pos; i++)
	//{
	//	std::cout << th_file1[i];
	//}

	TextResultFile::FieldDataParser<PFileCharArray> fld_parser;
	fld_parser.init(time_rcd->field_data, th_file1, time_history->get_field_num());
	// parse field data
	double *fld_data;
	size_t fld_data_num;
	fld_data = fld_parser.parse_next_point();
	while (fld_data)
	{
		fld_data_num = fld_parser.get_field_num();
		for (size_t i = 0; i < fld_data_num; i++)
			std::cout << fld_data[i] << " ";
		std::cout << "\n";
		fld_data = fld_parser.parse_next_point();
	}

}
