#ifndef _FILE_CHAR_ARRAY_H_
#define _FILE_CHAR_ARRAY_H_

#include <fstream>

#define next_buffer_id(buffer_id) ((buffer_id + 1) & 0b00000001)
#define swap_buffer { cur_fb_id = next_buffer_id(cur_fb_id); cur_fb = fb + cur_fb_id; }
#define is_in_buffer(pbuf, file_pos) ((pbuf)->begin <= (file_pos) && (file_pos) < (pbuf)->end)

class PFileCharArray;

// This class abstract file content as array
class FileCharArray
{
protected:
	// file
	std::fstream file;
	size_t file_size;
	bool end_of_file; // reason for this flag: sometimes the file size is larger than the size that can be read
	// buffer size
	static const size_t default_buffer_size = 64 * 1024;
	size_t buffer_size;
	// file buffer
	struct FileBuffer
	{
		// range of file content in the buffer
		size_t begin, end;
		char *buffer;
		FileBuffer() : 
			begin(0), end(0), buffer(nullptr) {}
		~FileBuffer() {}
	};
	// double buffers
	union
	{
		FileBuffer fb[2];
		struct { FileBuffer fb1, fb2; };
		
	};
	FileBuffer *cur_fb;
	unsigned char cur_fb_id;
	
public:
	FileCharArray(size_t buf_size = default_buffer_size) :
		file_size(0), end_of_file(false),
		buffer_size(buf_size),
		fb1(), fb2(), cur_fb_id(1), cur_fb(fb)
	{
		if (buf_size)
		{
			fb1.buffer = new char[buffer_size + buffer_size];
			fb2.buffer = fb1.buffer + buffer_size;
		}
	}

	FileCharArray(const char *filename, size_t buf_size = default_buffer_size) :
		FileCharArray(buf_size)
	{
		if (filename)
			open_file(filename);
	}

	~FileCharArray()
	{
		if (fb1.buffer)
			delete[] fb1.buffer;
		file.close();
	}

	inline size_t get_file_size(void) const { return file_size; }

	void open_file(const char* file_name)
	{
		if (file.is_open())
			file.close();
		file.open(file_name, ios::in | ios::binary);
		end_of_file = false;
		// get file size
		file.seekp(0, ios::end);
		file_size = file.tellp();
		file.seekp(0, ios::beg);
		file_size -= file.tellp();
	}

	void create_buffer(size_t buf_size)
	{
		if (!buf_size) return;
		if (fb1.buffer)
			delete[] fb1.buffer;
		fb1.buffer = new char[buffer_size + buffer_size];
		fb2.buffer = fb1.buffer ? fb1.buffer + buffer_size : nullptr;
	}

	inline bool is_valid(void) const { return file.is_open() && fb1.buffer && fb2.buffer; }
	inline bool is_eof(void) const { return end_of_file; }

	inline char get_char(size_t file_pos)
	{
		if (is_in_buffer(cur_fb, file_pos))
		{
			end_of_file = false;
			return cur_fb->buffer[file_pos - cur_fb->begin];
		}
		
		if (file_pos < file_size)
		{
			swap_buffer;
			if (is_in_buffer(cur_fb, file_pos))
			{
				end_of_file = false;
				return cur_fb->buffer[file_pos - cur_fb->begin];
			}

			fill_buffer(cur_fb_id, file_pos);
			return end_of_file ? '\0' : cur_fb->buffer[file_pos - cur_fb->begin];
		}
		
		end_of_file = true;
		return '\0';
	}

	inline char operator[] (size_t i) { return get_char(i); }

private:
	size_t fill_buffer(unsigned char buf_id, size_t file_pos)
	{
		if (file_pos >= file_size)
		{
			end_of_file = true;
			return 0;
		}

		size_t read_size = 0;
		fb[buf_id].begin = file_pos - file_pos % buffer_size;
		if (fb[buf_id].begin < file_size)
		{
			file.seekp(fb[buf_id].begin, ios::beg);
			size_t remain_size = file_size - fb[buf_id].begin;
			read_size = remain_size < buffer_size ? remain_size : buffer_size;
			file.read(fb[buf_id].buffer, read_size);
			read_size = file.gcount();
		}
		fb[buf_id].end = fb[buf_id].begin + read_size;
		end_of_file = file_pos >= fb[buf_id].end;
		return read_size;
	}
};

class PFileCharArray
{
public:
	PFileCharArray():
		file(nullptr), cur_pos(0) {}
	PFileCharArray(FileCharArray &fs, size_t pos = 0) :
		file(&fs), cur_pos(pos) {}
	PFileCharArray(const PFileCharArray &another) :
		file(another.file), cur_pos(another.cur_pos) {}
	~PFileCharArray() {}
	inline bool is_eof(void)
	{
		if (file)
			return file->is_eof();
		return true;
	}
	inline void offset(const PFileCharArray &base, long off)
	{
		file = base.file;
		cur_pos = base.cur_pos + off;
	}
	inline void offset(const PFileCharArray &base, size_t off)
	{
		file = base.file;
		cur_pos = base.cur_pos + off;
	}
	inline void operator= (const PFileCharArray &another)
	{
		file = another.file;
		cur_pos = another.cur_pos;
	}
	inline void operator=(const FileCharArray &fca)
	{
		file = const_cast<FileCharArray *>(&fca);
		cur_pos = 0;
	}
	inline void operator=(size_t id)
	{
		cur_pos = id;
	}
	inline char operator* (void) { return file->get_char(cur_pos); }
	inline char operator[] (size_t id) { return file->get_char(cur_pos + id); }
	inline PFileCharArray &operator++() { ++cur_pos; return *this; }
	inline PFileCharArray &operator--() { --cur_pos; return *this; }
	inline PFileCharArray operator++(int)
	{
		PFileCharArray prev(*this);
		++cur_pos;
		return prev;
	}
	inline PFileCharArray operator--(int)
	{
		PFileCharArray next(*this);
		--cur_pos;
		return next;
	}
	inline PFileCharArray &operator+= (long off)
	{
		cur_pos += off;
		return *this;
	}
	inline PFileCharArray &operator-= (long off)
	{
		cur_pos -= off;
		return *this;
	}
	inline PFileCharArray operator+ (long off)
	{
		PFileCharArray res;
		res.offset(*this, off);
		return res;
	}
	inline PFileCharArray operator- (long off)
	{
		PFileCharArray res;
		res.offset(*this, off);
		return res;
	}
	inline PFileCharArray operator+ (size_t off)
	{
		PFileCharArray res;
		res.offset(*this, off);
		return res;
	}
	inline PFileCharArray operator- (size_t off)
	{
		PFileCharArray res;
		res.offset(*this, off);
		return res;
	}
	inline bool operator == (const PFileCharArray &other)
	{
		return file == other.file && cur_pos == other.cur_pos;
	}
	inline bool operator != (const PFileCharArray &other)
	{
		return file != other.file || cur_pos != other.cur_pos;
	}
	inline size_t operator- (const PFileCharArray &other)
	{
		return cur_pos - other.cur_pos;
	}
	inline bool operator < (const PFileCharArray &other)
	{
		return cur_pos < other.cur_pos;
	}

private:
	FileCharArray *file;
	size_t cur_pos;
};

#undef next_buffer_id
#undef swap_buffer
#undef is_in_buffer

#endif