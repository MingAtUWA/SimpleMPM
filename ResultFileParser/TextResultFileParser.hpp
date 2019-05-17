#ifndef _TEXT_RESULT_FILE_PARSER_HPP_
#define _TEXT_RESULT_FILE_PARSER_HPP_

//#include <cctype>
#include <string>
#include <iostream>

#include "PreAllocStringBuffer.hpp"
#include "ParserException.h"
#include "TextResultFileInfo.h"

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
	class Parser
	{
	protected:
		TextResultFileInfo *file_info;
		PChar file_content;
		struct State
		{
		public:
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
				return *this;
			}
		};
		State cur_state;
		Internal::StrBuf token;

	public:
		Parser() {}
		~Parser() {}

		int parse(TextResultFileInfo *res_file, PChar fc)
		{
			// initialization
			file_info = res_file;
			file_content = fc;
			cur_state.file_pos = fc;
			cur_state.line_index = 1;
			cur_state.column_index = 1;

			// begin parsing
			try
			{
				while (true)
				{
					skip_blank_char();
					switch (*cur_state)
					{
					case '\0': // finish parsing
						return 0;
					case '*':
						++cur_state;
						if (*cur_state == '*')
						{
							++cur_state;
							CommentNode *comment_node = res_file->new_comment();
							parse_comment(comment_node);
							// res_file.add_comment();
							break;
						}
						if (!get_token(token))
							throw ParserException(cur_state.line_index, cur_state.column_index,
								"Expect section name or \"end\" after \"*\".",
								ParserException::Reason::SyntaxError);
						if (token == "TimeHistory")
						{
							TimeHistoryNode *time_history_node = res_file->new_time_history();
							parse_time_history(time_history_node);
							file_info->add_time_history(time_history_node);
						}
						else if (token == "ModelState")
						{
							ModelStateNode *model_state_node = res_file->new_model_state();
							parse_model_state(model_state_node);
							file_info->add_model_state(model_state_node);
						}
						else
						{
							UnknownNode unknown_node(token.c_str());
							parse_unknown(&unknown_node);
							// print unknown
						}
						break;
					default:
						++cur_state;
						break;
					}
				}
			}
			catch (const ParserException& error)
			{
				std::cerr << error.get_message() << std::endl;
				return -1;
			}
			return 0;
		}

	protected:
		int parse_comment(CommentNode *comment_node)
		{
			comment_node->begin_pos = size_t(cur_state.file_pos - file_content);
			comment_node->begin_line = cur_state.line_index;
			comment_node->begin_column = cur_state.column_index;
			while (true)
			{
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of comment section, expecting \"**\"");
				case '*':
					if (cur_state[1] == '*') // end of comment_node
					{
						comment_node->end_pos = size_t(cur_state.file_pos - file_content);
						comment_node->end_line = cur_state.line_index;
						comment_node->end_column = cur_state.column_index;
						++cur_state;
						++cur_state;
						return 0;
					}
					++cur_state;
					break;
				default:
					++cur_state;
					break;
				}
			}
			return 0;
		}

		int parse_unknown(UnknownNode *unknown_node)
		{
			size_t end_pos_tmp;
			size_t end_line_tmp;
			size_t end_column_tmp;

			unknown_node->begin_pos = size_t(cur_state.file_pos - file_content);
			unknown_node->begin_line = cur_state.line_index;
			unknown_node->begin_column = cur_state.column_index;
			size_t depth = 1;
			while (true)
			{
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of this section, expecting \"**\"");
				case '*':
					end_pos_tmp = size_t(cur_state.file_pos - file_content);
					end_line_tmp = cur_state.line_index;
					end_column_tmp = cur_state.column_index;
					++cur_state;
					if (get_token(token))
					{
						if (token == "end")
						{
							--depth;
							if (depth == 0)
							{
								unknown_node->end_pos = end_pos_tmp;
								unknown_node->end_line = end_line_tmp;
								unknown_node->end_column = end_column_tmp;
								return 0;
							}
						}
						else
						{
							++depth;
						}
					}
					break;
				default:
					++cur_state;
					break;
				}
			}
			return 0;
		}

		int parse_step(StepNode *step_node)
		{
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of step section, expecting \"*end\"",
						ParserException::Reason::SyntaxError);
				case '*':
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Expect section name or \"end\" after \"*\".",
							ParserException::Reason::SyntaxError);
					if (token == "end")
					{
						return 0;
					}
					else // unknown node
					{
						UnknownNode unknown_node(token.c_str());
						parse_unknown(&unknown_node);
						// print unknown_node
					}
					break;
				default:
					get_attribute_name(token);
					skip_blank_char();
					if (token == "Name")
					{
						get_attribute_value(token);
						step_node->name = token.c_str();
					}
					else if (token == "Index")
					{
						get_attribute_value(token);
						step_node->index = std::atoi(token.c_str());
					}
					else if (token == "StartTime")
					{
						get_attribute_value_double(token);
						step_node->start_time = std::atof(token.c_str());
					}
					else if (token == "StepTime")
					{
						get_attribute_value_double(token);
						step_node->step_time = std::atof(token.c_str());
					}
					else if (token == "StartSubstepIndex")
					{
						get_attribute_value(token);
						step_node->start_substep_index = std::atoi(token.c_str());
					}
					else if (token == "SubstepNum")
					{
						get_attribute_value(token);
						step_node->substep_num = std::atoi(token.c_str());
					}
					else // unknown attribute
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unknown Step Attribute",
							ParserException::Reason::ContentOfUnknownMeaning);
					}
					break;
				}
			}
			return 0;
		}

		int parse_field_data(FieldDataNode *fld_data_node)
		{
			size_t end_pos_tmp;
			size_t end_line_tmp;
			size_t end_column_tmp;

			fld_data_node->begin_pos = size_t(cur_state.file_pos - file_content);
			fld_data_node->begin_line = cur_state.line_index;
			fld_data_node->begin_column = cur_state.column_index;
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of field data section, expecting \"*end\"",
						ParserException::Reason::SyntaxError);
				case '*':
					end_pos_tmp = size_t(cur_state.file_pos - file_content);
					end_line_tmp = cur_state.line_index;
					end_column_tmp = cur_state.column_index;
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Expect \"end\" after \"*\".",
							ParserException::Reason::SyntaxError);
					if (token == "end")
					{
						fld_data_node->end_pos = end_pos_tmp;
						fld_data_node->end_line = end_line_tmp;
						fld_data_node->end_column = end_column_tmp;
						return 0;
					}
					else
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"No section is allowd in *FieldData.",
							ParserException::Reason::SyntaxError);
					}
					break;
				default:
					++cur_state;
					break;
				}
			}

		}

		int parse_time_rcd(TimeRecordNode *time_rcd)
		{
			bool fld_data_parsed = false;
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of time record section, expecting \"*end\"",
						ParserException::Reason::SyntaxError);
				case '*':
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Expect section name or \"end\" after \"*\".",
							ParserException::Reason::SyntaxError);
					if (token == "end")
					{
						return 0;
					}
					else if (token == "FieldData")
					{
						if (fld_data_parsed)
							throw ParserException(cur_state.line_index, cur_state.column_index,
								"Each time record should only have one field data.",
								ParserException::Reason::ContentError);
						fld_data_parsed = true;
						parse_field_data(&(time_rcd->field_data));
					}
					else
					{
						UnknownNode unknown_node(token.c_str());
						parse_unknown(&unknown_node);
					}
					break;
				default:
					// get time record properties
					get_attribute_name(token);
					skip_blank_char();
					if (token == "Index")
					{
						get_attribute_value(token);
						time_rcd->index = atoi(token.c_str());
					}
					else if (token == "StepIndex")
					{
						get_attribute_value(token);
						time_rcd->step_index = atoi(token.c_str());
					}
					else if (token == "SubstepNum")
					{
						get_attribute_value(token);
						time_rcd->substep_num = atoi(token.c_str());
					}
					else if (token == "TotalSubstepNum")
					{
						get_attribute_value(token);
						time_rcd->total_substep_num = atoi(token.c_str());
					}
					else if (token == "CurrentTime")
					{
						get_attribute_value_double(token);
						time_rcd->current_time = atof(token.c_str());
					}
					else if (token == "TotalTime")
					{
						get_attribute_value_double(token);
						time_rcd->total_time = atof(token.c_str());
					}
					else if (token == "PointNum")
					{
						get_attribute_value(token);
						time_rcd->point_num = atoi(token.c_str());
					}
					else if (token == "PointIndex")
					{
						size_t pid_tmp;

						if (time_rcd->point_num == 0)
							throw ParserException(cur_state.line_index, cur_state.column_index,
								"Can not find PointNum attribute before PointIndex attribute.",
								ParserException::Reason::ContentError);
						time_rcd->point_index.reserve(time_rcd->point_num);

						get_attribute_value(token);
						pid_tmp = atoi(token.c_str());
						time_rcd->point_index.add(pid_tmp);
						skip_blank_char();

						while (get_more_attribute_value(token))
						{
							pid_tmp = atoi(token.c_str());
							time_rcd->point_index.add(pid_tmp);
							skip_blank_char();
						}

						time_rcd->point_num = time_rcd->point_index.get_num();
					}
					else // unknown attribute
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unknown time record attribute.",
							ParserException::Reason::ContentError);
					}
					break;
				}
			}
			return 0;
		}

		int parse_time_history(TimeHistoryNode *time_history_node)
		{
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unexpected end of time history section, expecting \"*end\"",
							ParserException::Reason::SyntaxError);
				case '*':
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
								"Expect section name or \"end\" after \"*\".",
								ParserException::Reason::SyntaxError);
					if (token == "end") // end
					{
						return 0;
					}
					else if (token == "Step")
					{
						StepNode *step_node = file_info->new_step();
						parse_step(step_node);
						time_history_node->add_step(step_node);
					}
					else if (token == "TimeRecord")
					{
						TimeRecordNode *time_rcd_node = file_info->new_time_rcd();
						parse_time_rcd(time_rcd_node);
						time_history_node->add_time_rcd(time_rcd_node);
					}
					else
					{
						UnknownNode unknown_node(token.c_str());
						parse_unknown(&unknown_node);
						// print unknown_node
					}
					break;
				default:
					get_attribute_name(token);
					skip_blank_char();
					if (token == "Name")
					{
						get_attribute_value(token);
						time_history_node->name = token.c_str();
					}
					else if (token == "Index")
					{
						get_attribute_value(token);
						time_history_node->index = std::atoi(token.c_str());
					}
					else if (token == "Type")
					{
						get_attribute_value(token);
						time_history_node->type = token.c_str();
					}
					else if (token == "IntervalNum")
					{
						get_attribute_value(token);
						time_history_node->interval_num = std::atoi(token.c_str());
					}
					else if (token == "FieldNum")
					{
						get_attribute_value(token);
						time_history_node->field_num = std::atoi(token.c_str());
					}
					else if (token == "Field")
					{
						get_attribute_value(token);
						skip_blank_char();
						while (get_more_attribute_value(token))
							skip_blank_char();
					}
					else // unknown attribute
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unknown Output Attribute",
							ParserException::Reason::ContentError);
					}
					break;
				}
			}
			return 0;
		}

// ------------------------------- Parse model state -----------------------------------
		int parse_background_mesh(BackgroundMeshNode *background_mesh_node)
		{
			double double_tmp;
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of background mesh section, expecting \"*end\"",
						ParserException::Reason::SyntaxError);
				case '*':
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
								"Expect section name or \"end\" after \"*\".",
								ParserException::Reason::SyntaxError);
					if (token == "end") // end
					{
						return 0;
					}
					else
					{
						UnknownNode unknown_node(token.c_str());
						parse_unknown(&unknown_node);
						// print unknown_node
					}
					break;
				default:
					get_attribute_name(token);
					skip_blank_char();
					if (token == "Type")
					{
						get_attribute_value(token);
						background_mesh_node->type = token.c_str();
					}
					else if (token == "XCoordinateNum")
					{
						get_attribute_value_unsigned_int(token);
						background_mesh_node->x_coord_num = size_t(std::atoi(token.c_str()));
					}
					else if (token == "YCoordinateNum")
					{
						get_attribute_value_unsigned_int(token);
						background_mesh_node->y_coord_num = size_t(std::atoi(token.c_str()));
					}
					else if (token == "XCoordinates")
					{
						background_mesh_node->x_coords.reserve(background_mesh_node->x_coord_num);
						background_mesh_node->x_coords.reset();
						get_attribute_value_double(token);
						double_tmp = std::atof(token.c_str());
						background_mesh_node->x_coords.add(double_tmp);
						while (get_more_attribute_value_double(token))
						{
							double_tmp = std::atof(token.c_str());
							background_mesh_node->x_coords.add(double_tmp);
						}
						background_mesh_node->x_coord_num 
							= background_mesh_node->x_coords.get_num();
					}
					else if (token == "YCoordinates")
					{
						background_mesh_node->y_coords.reserve(background_mesh_node->y_coord_num);
						background_mesh_node->y_coords.reset();
						get_attribute_value_double(token);
						double_tmp = std::atof(token.c_str());
						background_mesh_node->y_coords.add(double_tmp);
						while (get_more_attribute_value_double(token))
						{
							double_tmp = std::atof(token.c_str());
							background_mesh_node->y_coords.add(double_tmp);
						}
						background_mesh_node->y_coord_num = 
							background_mesh_node->y_coords.get_num();
					}
					else // unknown attribute
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unknown Output Attribute",
							ParserException::Reason::ContentError);
					}
					break;
				}
			}
		}

		int parse_object(ObjectNode *object_node)
		{
			double double_tmp;
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of background mesh section, expecting \"*end\"",
						ParserException::Reason::SyntaxError);
				case '*':
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Expect section name or \"end\" after \"*\".",
							ParserException::Reason::SyntaxError);
					if (token == "end") // end
					{
						return 0;
					}
					else
					{
						UnknownNode unknown_node(token.c_str());
						parse_unknown(&unknown_node);
						// print unknown_node
					}
					break;
				default:
					get_attribute_name(token);
					skip_blank_char();
					if (token == "Type")
					{
						get_attribute_value(token);
						object_node->type = token.c_str();
					}
					else if (token == "ParticleNum")
					{
						get_attribute_value_unsigned_int(token);
						object_node->particle_num = size_t(std::atoi(token.c_str()));
					}
#define             ELSE_IF_TOKEN_EQUAL(field)                                \
					else if (token == #field)                                 \
					{                                                         \
						object_node->field.reserve(object_node->particle_num);\
						object_node->field.reset();                           \
						get_attribute_value_double(token);                    \
						double_tmp = std::atof(token.c_str());                \
						object_node->field.add(double_tmp);                   \
						while (get_more_attribute_value_double(token))        \
						{                                                     \
							double_tmp = std::atof(token.c_str());            \
							object_node->field.add(double_tmp);               \
						}                                                     \
					}
					ELSE_IF_TOKEN_EQUAL(x)
					ELSE_IF_TOKEN_EQUAL(y)
					ELSE_IF_TOKEN_EQUAL(vol)
					ELSE_IF_TOKEN_EQUAL(n)
					ELSE_IF_TOKEN_EQUAL(density_s)
					ELSE_IF_TOKEN_EQUAL(density_f)
					ELSE_IF_TOKEN_EQUAL(density)
					else // unknown attribute
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unknown Output Attribute",
							ParserException::Reason::ContentError);
					}
					break;
				}
			}
		}

		int parse_model_state(ModelStateNode *model_state_node)
		{
			while (true)
			{
				skip_blank_char();
				switch (*cur_state)
				{
				case '\0':
					throw ParserException(cur_state.line_index, cur_state.column_index,
						"Unexpected end of model state section, expecting \"*end\"",
						ParserException::Reason::SyntaxError);
				case '*':
					++cur_state;
					if (!get_token(token))
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Expect section name or \"end\" after \"*\".",
							ParserException::Reason::SyntaxError);
					if (token == "end") // end
					{
						return 0;
					}
					else if (token == "BackgroundMesh")
					{
						if (model_state_node->background_mesh)
							throw ParserException(cur_state.line_index, cur_state.column_index,
								"Each model should have only one background mesh.",
								ParserException::Reason::ContentError);
						model_state_node->background_mesh = file_info->new_background_mesh();
						parse_background_mesh(model_state_node->background_mesh);
					}
					else if (token == "Object")
					{
						ObjectNode *object_node = file_info->new_object();
						parse_object(object_node);
						model_state_node->add_object(object_node);
					}
					else
					{
						UnknownNode unknown_node(token.c_str());
						parse_unknown(&unknown_node);
						// print unknown_node
					}
					break;
				default:
					get_attribute_name(token);
					skip_blank_char();
					if (token == "Type")
					{
						get_attribute_value(token);
						model_state_node->type = token.c_str();
					}
					else // unknown attribute
					{
						throw ParserException(cur_state.line_index, cur_state.column_index,
							"Unknown attribute of model state",
							ParserException::Reason::ContentError);
					}
					break;
				}
			}
			return 0;
		}

	protected: // utility functions
		void skip_blank_char(void)
		{
			while (*cur_state == ' '
				|| *cur_state == '\r'
				|| *cur_state == '\n')
				++cur_state;
		}

		bool get_token(Internal::StrBuf &buffer)
		{
			State tmp_state = cur_state;
			buffer.reset();

			while (isalnum(*tmp_state) ||
				*tmp_state == '_' ||
				*tmp_state == '-')
			{
				buffer << *tmp_state;
				++tmp_state;
			}

			if (buffer.get_length())
			{
				cur_state = tmp_state;
				return true;
			}
			return false;
		}

		bool get_equality_sign(void)
		{
			State tmp_state = cur_state;
			if (*tmp_state == '=')
			{
				++tmp_state;
				cur_state = tmp_state;
				return true;
			}
			return false;
		}

		inline bool get_comma(void)
		{
			State tmp_state = cur_state;
			if (*tmp_state == ',')
			{
				++tmp_state;
				cur_state = tmp_state;
				return true;
			}
			return false;
		}

		void get_attribute_name(Internal::StrBuf &token)
		{
			if (!get_token(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect attribute name.", ParserException::Reason::SyntaxError);
		}

		void get_attribute_value(Internal::StrBuf &token)
		{
			// get '='
			if (!get_equality_sign())
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect \"=\".", ParserException::Reason::SyntaxError);
			skip_blank_char();
			if (!get_token(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect attribute value.", ParserException::Reason::SyntaxError);
		}

		// return false if no more attribute
		bool get_more_attribute_value(Internal::StrBuf &token)
		{
			// get ','
			if (!get_comma())
				return false;
			skip_blank_char();
			if (!get_token(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect more attribute value.", ParserException::Reason::SyntaxError);
			return true;
		}

		// parse unsigned integer
		bool parse_unsigned_int(Internal::StrBuf &buffer)
		{
			State tmp_state(cur_state);
			buffer.reset();
			while (isdigit(*tmp_state))
			{
				buffer << *tmp_state;
				++tmp_state;
			}
			if (buffer.get_length()) // success
			{
				cur_state = tmp_state;
				return true;
			}
			else // fail
				return false;
		}

		void get_attribute_value_unsigned_int(Internal::StrBuf &token)
		{
			if (!get_equality_sign())
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect \"=\".", ParserException::Reason::SyntaxError);
			skip_blank_char();
			if (!parse_unsigned_int(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect unsigned integer as attribute value.", ParserException::Reason::TypeError);
		}

		bool get_more_attribute_value_unsigned_int(Internal::StrBuf &token)
		{
			// get ','
			if (!get_comma())
				return false;
			skip_blank_char();
			if (!parse_unsigned_int(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
						"Expect more unsigned integer as attribute value.",
						ParserException::Reason::TypeError);
			return true;
		}

		bool parse_double(Internal::StrBuf &buffer)
		{
			State tmp_state(cur_state);
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
				goto complete_parsing;
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

		void get_attribute_value_double(Internal::StrBuf &token)
		{
			if (!get_equality_sign())
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect \"=\".", ParserException::Reason::SyntaxError);
			skip_blank_char();
			if (!parse_double(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect double as attribute value.", ParserException::Reason::TypeError);
		}

		bool get_more_attribute_value_double(Internal::StrBuf &token)
		{
			// get ','
			if (!get_comma())
				return false;
			skip_blank_char();
			if (!parse_double(token))
				throw ParserException(cur_state.line_index, cur_state.column_index,
					"Expect more double as attribute value.",
					ParserException::Reason::TypeError);
			return true;
		}
	};

};

#endif