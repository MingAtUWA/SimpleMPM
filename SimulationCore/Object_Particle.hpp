#ifndef _OBJECT_PARTICLE_HPP_
#define _OBJECT_PARTICLE_HPP_

#include "ItemArray.hpp"
#include "Object.h"

template <class ParticleType>
struct Object_Particle : public Object
{
public:
	size_t pcl_num;
	ParticleType *pcls;

protected:
	MemoryUtilities::ItemArray<ParticleType> pcls_mem;

public:
	Object_Particle() :	pcl_num(0), pcls(nullptr) {}
	~Object_Particle() { clear();	}

	inline void set_pcl_num(size_t num) { pcls_mem.reserve(num); }
	inline void add_pcl(ParticleType &pcl) { pcls_mem.add(pcl);	}
	inline void add_pcl(size_t num, ParticleType *_pcls)
	{
		ParticleType *tmp = pcls_mem.alloc(num);
		memcpy(tmp, _pcls, sizeof(ParticleType) * num);
	}
	void update(void)
	{
		pcl_num = pcls_mem.get_num();
		pcls = pcl_num ? pcls_mem.get_mem() : nullptr;
	}
	void clear(void)
	{
		pcl_num = 0;
		pcls = nullptr;
		pcls_mem.reset();
	}
};

#endif