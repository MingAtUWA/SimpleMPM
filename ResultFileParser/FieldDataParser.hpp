#ifndef _FIELD_DATA_PARSER_HPP_
#define _FIELD_DATA_PARSER_HPP_

#include "ItemArray.hpp"
#include "PreAllocStringBuffer.hpp"
#include "ParserException.h"

namespace TextResultFile
{

#ifndef PRE_ALLOC_BUFFER_SIZE
#define PRE_ALLOC_BUFFER_SIZE 128
namespace Internal
{
	typedef MemoryUtilities::PreAllocStringBuffer<PRE_ALLOC_BUFFER_SIZE> StrBuf;
}
#undef PRE_ALLOC_BUFFER_SIZE
#endif

template<typename PChar>
class FieldDataParser
{
protected:
	PChar file_content;
	
	struct State
	{
	public:
		static PChar end_file_pos;
		PChar file_pos;
		size_t line_index;
		size_t column_index;
	public:
		State() {}
		State(const State &another) :
			file_pos(another.file_pos),
			line_index(another.line_index),
			column_index(another.column_index) {}
		inline char operator* (void) { return *file_pos; }
		inline char operator[] (size_t id) { return file_pos[id]; }
		inline State& operator++ (void)
		{
			if (*file_pos == '\n')
			{
				++line_index;
				column_index = 0;
			}
			++file_pos;
			++column_index;
			if (file_pos < end_file_pos)
				return *this;

			// exit due to end of field data
			throw ParserException(line_index, column_index, nullptr,
				ParserException::Type::Note,
				ParserException::Reason::EndOfContent);
		}
	};
	State cur_state;
	// buffer
	MemoryUtilities::ItemArray<double> field_data;
	Internal::StrBuf double_str;

public:
	/*----------------------------------------------------
	init()
	Parameters:
		1. fc - head of buffer of file content;
		2. begin - the first index of field data;
		3. end - the last index of field data.
	 ----------------------------------------------------*/
	void init(FieldDataNode &fld_data, PChar fc,
					 size_t suggested_field_num = 0)
	{
		file_content = fc;

		State::end_file_pos.offset(fc, fld_data.end_pos);
		cur_state.file_pos.offset(fc, fld_data.begin_pos);
		cur_state.line_index = fld_data.begin_line;
		cur_state.column_index = fld_data.begin_column;
		if (suggested_field_num)
			field_data.reserve(suggested_field_num);
	}
	/*------------------------------------
	parse_next_point()
	Return values:
		1. pointer to field data;
		2. null if no more data.
	 ------------------------------------*/
	double *parse_next_point(void)
	{
		double double_tmp;
		field_data.reset();
		try
		{
			while (true)
			{
				skip_divider();
				switch (*cur_state)
				{
				case '\n':
					++cur_state;
				case '\0':
					if (field_data.get_num())
						return static_cast<double *>(field_data.get_mem());
					else if (cur_state.file_pos.is_eof())
						return nullptr;
					// skip blank line
					break;
				default:
					double_tmp = get_double();
					field_data.add(double_tmp);
					break;
				}
			}	
		}
		catch (const ParserException& error)
		{
			// Just end of field data, not error.
			if (error.get_type() == ParserException::Type::Note &&
				error.get_reason() == ParserException::Reason::EndOfContent)
			{
				if (field_data.get_num())
					return static_cast<double *>(field_data.get_mem());
			}
			else
			{
				std::cerr << error.get_message() << std::endl;
			}
		}

		return nullptr;
	}
	inline size_t get_field_num(void) {	return field_data.get_num(); }

private:
	inline void skip_divider(void)
	{
		while (*cur_state == ' '
			|| *cur_state == ','
			|| *cur_state == '\r')
			++cur_state;
	}
	inline void skip_divider(State &state)
	{
		while (*state == ' '
			|| *state == ','
			|| *state == '\r')
			++state;
	}

	bool parse_double(Internal::StrBuf &buffer)
	{
		State tmp_state = cur_state;
		buffer.reset();
		//before_period
		if (*tmp_state == '.')
			goto after_period;
		if (*tmp_state == '-' || *tmp_state == '+')
		{
			if (!isdigit(tmp_state[1]))
				return false;
			buffer << *tmp_state;
			++tmp_state;
		}
		while (isdigit(*tmp_state))
		{
			buffer << *tmp_state;
			++tmp_state;
		}
		// after period
		if (*tmp_state != '.')
			return false;
	after_period:
		buffer << '.';
		++tmp_state;
		while (isdigit(*tmp_state))
		{
			buffer << *tmp_state;
			++tmp_state;
		}
		// after exponential
		if (*tmp_state != 'e' && *tmp_state != 'E')
			goto complete_parsing;
		if (tmp_state[1] == '-' || tmp_state[1] == '+')
		{
			if (!isdigit(tmp_state[2]))
				goto complete_parsing;
		}
		else
		{
			if (!isdigit(tmp_state[1]))
				goto complete_parsing;
		}
		buffer << 'e';
		++tmp_state;
		buffer << *tmp_state; // '-', '+' or number
		++tmp_state;
		while (isdigit(*tmp_state))
		{
			buffer << *tmp_state;
			++tmp_state;
		}
		// complete parsing
	complete_parsing:
		cur_state = tmp_state;
		return true;
	}

	double get_double(void)
	{
		if (!parse_double(double_str))
			throw ParserException(cur_state.line_index,  cur_state.column_index,
								  "Expect double number.",
								  ParserException::Reason::TypeError);
		return std::atof(double_str.c_str());
	}
};

template<typename PChar>
PChar FieldDataParser<PChar>::State::end_file_pos;

};

#endif