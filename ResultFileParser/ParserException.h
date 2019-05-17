#ifndef _PARSER_EXCEPTION_H_
#define _PARSER_EXCEPTION_H_

#include <exception>
#include <string>

namespace Internal
{
// change this macro to add reason of parsing
#define PARSER_EXCEPTION_ERROR_INFO_NUM 7
template <int Dummy = 0>
struct ExceptionReason
{
	static const char *reason[PARSER_EXCEPTION_ERROR_INFO_NUM];
	static const size_t reason_num;
};

template <int Dummy>
const char *ExceptionReason<Dummy>::reason[PARSER_EXCEPTION_ERROR_INFO_NUM] = {
	nullptr,
	"End of parsed content",
	"Incorrent type",
	"Incorrect syntax",
	"Incorrect content",
	"Content with unknown meaning",
	"File error"
};

template <int Dummy>
const size_t ExceptionReason<Dummy>::reason_num = PARSER_EXCEPTION_ERROR_INFO_NUM;
#undef PARSER_EXCEPTION_ERROR_INFO_NUM
};

#ifndef PARSER_EXECEPTION_MESSAGE_LENGTH
#define PARSER_EXECEPTION_MESSAGE_LENGTH 200
#endif
class ParserException : public std::exception
{
public:
	enum class Type : unsigned char
	{
		Error   = 0,
		Warning = 1,
		Note    = 2
	};
	enum class Reason : size_t
	{
		UserDefined      = 0,
		EndOfContent     = 1,
		TypeError        = 2,
		SyntaxError      = 3,
		ContentError     = 4,
		ContentOfUnknownMeaning = 5,
		FileError = 6,
		ExceptionClassError = std::numeric_limits<size_t>::max()
	};
public:
	ParserException(size_t line_id, size_t col_id,
		const char *error_msg,
		Reason reason_id = Reason::UserDefined, Type type_id = Type::Error) noexcept :
		line_index(line_id), column_index(col_id),
		type(type_id), reason(reason_id), message_len(0)
	{
		int len_tmp;

		switch (type)
		{
		case ParserException::Type::Error:
			len_tmp = snprintf(message, PARSER_EXECEPTION_MESSAGE_LENGTH,
				"(line %llu, column %llu) Error code %llu: ", line_index, column_index, size_t(type));
			break;
		case ParserException::Type::Warning:
			len_tmp = snprintf(message, PARSER_EXECEPTION_MESSAGE_LENGTH,
				"(line %llu, column %llu) Warning code %llu: ", line_index, column_index, size_t(type));
			break;
		case ParserException::Type::Note:
			len_tmp = snprintf(message, PARSER_EXECEPTION_MESSAGE_LENGTH,
				"(line %llu, column %llu) Note code %llu: ", line_index, column_index, size_t(type));
			break;
		default:
			type = Type::Error;
			reason = Reason::ExceptionClassError;
			strncpy(message, "Error: ParserError class internal error: Unknown type.",
				PARSER_EXECEPTION_MESSAGE_LENGTH);
			message_len = strlen(message);
			return;
		}
		if (len_tmp < 0)
		{
			type = Type::Error;
			reason = Reason::ExceptionClassError;
			strncpy(message, "Error: ParserError class internal error: Fail to get message.",
					PARSER_EXECEPTION_MESSAGE_LENGTH);
			message_len = strlen(message);
			return;
		}
		message_len += len_tmp;

		if (size_t(reason) < Internal::ExceptionReason<0>::reason_num &&
			Internal::ExceptionReason<0>::reason[size_t(reason)]
			&& (PARSER_EXECEPTION_MESSAGE_LENGTH - message_len) > 0)
		{
			len_tmp = snprintf(message + message_len,
				PARSER_EXECEPTION_MESSAGE_LENGTH - message_len,
				"%s: ", Internal::ExceptionReason<0>::reason[size_t(reason)]);
		}
		if (len_tmp < 0)
		{
			type = Type::Error;
			reason = Reason::ExceptionClassError;
			strncpy(message, "Error: ParserError class internal error: Fail to get message.",
					PARSER_EXECEPTION_MESSAGE_LENGTH);
			message_len = strlen(message);
			return;
		}
		message_len += len_tmp;

		if (error_msg && (PARSER_EXECEPTION_MESSAGE_LENGTH - message_len) > 0)
		{
			len_tmp = snprintf(message + message_len,
				PARSER_EXECEPTION_MESSAGE_LENGTH - message_len,
				"%s", error_msg);
		}
		if (len_tmp < 0)
		{
			type = Type::Error;
			reason = Reason::ExceptionClassError;
			strncpy(message, "Error: ParserError class internal error: Fail to get message.",
					PARSER_EXECEPTION_MESSAGE_LENGTH);
			message_len = strlen(message);
			return;
		}
		message_len += len_tmp;
	}
	ParserException(size_t line_index, size_t col_index,
		const char *error_msg,
		Type type_id, Reason reason_id = Reason::UserDefined) noexcept :
		ParserException(line_index, col_index, error_msg, reason_id, type_id) {}
	~ParserException() {}

	virtual const char* what() const { return message; }
	inline Type get_type(void) const { return type; }
	inline size_t get_line(void) const { return line_index; }
	inline size_t get_column(void) const { return column_index; }
	inline Reason get_reason(void) const { return reason; }
	inline const char *get_message(void) const { return message; }

protected:
	Type type;
	size_t line_index;
	size_t column_index;
	Reason reason;
	size_t message_len;
	char message[PARSER_EXECEPTION_MESSAGE_LENGTH];
};
#undef PARSER_EXECEPTION_MESSAGE_LENGTH

#endif