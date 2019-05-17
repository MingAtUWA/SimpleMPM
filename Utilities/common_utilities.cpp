#include <windows.h>

#include "common_utilities.h"

int make_dir(const char *path, void *_attr)
{
	LPSECURITY_ATTRIBUTES attr = static_cast<LPSECURITY_ATTRIBUTES>(_attr);
	return CreateDirectory(path, attr);
}
