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

#include <DetourCrowd.h>

#include <DetourMesh.h>
#include <DetourNavmeshCreator.h>

#include <DetourNavMesh.h>

#include <Recast.h>

#include <string>

#ifdef _MSC_VER
#pragma warning(push, 0)
#include <catch.hpp>
#pragma warning(pop)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#include <catch.hpp>
#pragma GCC diagnostic pop
#endif

class TestBuildContext : public rcContext
{
private:
	virtual void doLog(const rcLogCategory /*category*/, const char* msg, const int len)
	{
		WARN(std::string(msg,len));
	}
};

/// Retrieve a square mesh
const dtNavmeshInputGeometry& getSquareMesh();

/// Square navmesh
const dtNavMesh& getSquareNavmesh();

/// Creates a tiled navigation mesh from the contents of a file.
/// @note DT_TILE_FREE_DATA is used as an option to the initialization process.
bool loadTiledNavMesh(dtNavMesh* navmesh, const char* path);

bool saveTileNavmesh(const char* path, const dtNavMesh* navmesh, unsigned* tileIndices, unsigned tileCount);

#endif
