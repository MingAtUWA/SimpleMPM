#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <string>

struct Particle {};

struct ParticleVar {};

struct Object
{
protected:
	size_t index;
	std::string name;
	
public:
	Object() : name(20, '\0'), index(0) {}

	inline void set_name(const char *na) { name = na; }
	inline const char *get_name(void) const { return name.c_str(); }
	inline void set_index(size_t id) { index = id; }
	inline size_t get_index(void) const { return index; }
};

#endif