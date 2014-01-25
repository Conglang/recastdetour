//
// Copyright (c) 2013 MASA Group recastdetour@masagroup.net
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef UTILS_H
#define UTILS_H

#include <Recast.h>

#ifdef _MSC_VER
#   pragma warning(push, 0)
#   include <catch.hpp>
#   pragma warning(pop)
#else
#   pragma GCC diagnostic push
#   pragma GCC diagnostic ignored "-Wall"
#   include <catch.hpp>
#   pragma GCC diagnostic pop
#endif

class TestBuildContext : public rcContext
{
private:
	virtual void doLog(const rcLogCategory /*category*/, const char* msg, const int len)
	{
		INFO(std::string(msg,len));
	}
};
#endif