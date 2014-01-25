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

#define CATCH_CONFIG_MAIN // Generate automatically the main (one occurrence only)

#include "Utils.h"

#include <cstring>

const dtNavmeshInputGeometry& getSquareMesh()
{
	static dtNavmeshInputGeometry square;
	if (square.mesh.countFaces() == 0)
	{
		unsigned v0, v1, v2, v3;
		square.mesh.addVertex(20.0, 0.0, 20.0, v0);
		square.mesh.addVertex(20.0, 0.0, -20.0, v1);
		square.mesh.addVertex(-20.0, 0.0, -20.0, v2);
		square.mesh.addVertex(-20.0, 0.0, 20.0, v3);

		unsigned f0, f1;
		square.mesh.addFace(v2, v3, v0, f0);
		square.mesh.addFace(v0, v1, v2, f1);
		REQUIRE(square.initialize());
	}
	return square;
}

const dtNavMesh& getSquareNavmesh()
{
	static dtNavMesh square;
	if (square.getParams()->maxPolys == 0)
{
		const dtNavmeshInputGeometry& geometry = getSquareMesh();

		dtTiledNavmeshCfg configuration;
		configuration.computeTileCount(geometry.bmin, geometry.bmax, 20);

		TestBuildContext context;
		REQUIRE(dtCreateTiledNavmesh(geometry, configuration, square, &context));
	}
	return square;
}

bool defaultInitializeAgent(dtCrowd& crowd, int index)
{
	if (index == -1)
		return false;

	dtCrowdAgent ag;
	crowd.fetchAgent(ag, index);

	ag.radius = 0.2;
	ag.height = 1.7;
	ag.maxSpeed = 2.0;
	ag.maxAcceleration = 10.0;
	ag.behavior = 0;
	ag.detectionRange = 4.f;

	crowd.pushAgent(ag);

	return true;
}

namespace
{
	static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
	static const int NAVMESHSET_VERSION = 1;
	
	struct NavMeshSetHeader
	{
		int magic;
		int version;
		int numTiles;
		dtNavMeshParams params;
	};
	
	struct NavMeshTileHeader
	{
		dtTileRef tileRef;
		int dataSize;
	};
}

bool loadTiledNavMesh(dtNavMesh* navmesh, const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp)
	{
		// can't open the file
		return false;
	}
	
	// Read header.
	NavMeshSetHeader header;
	fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		// invalid binary file header
		fclose(fp);
		return false;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		// invalid version
		fclose(fp);
		return false;
	}
	
	dtStatus status = navmesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return false;
	}
	
	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		fread(&tileHeader, sizeof(tileHeader), 1, fp);
		
		if (!tileHeader.tileRef || !tileHeader.dataSize)
		{
			// unable to read tile header #i
			fclose(fp);
			return false;
		}
		
		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data)
		{
			// unable to allocate navigation mesh data for tile #i
			fclose(fp);
			return false;
		}
		memset(data, 0, tileHeader.dataSize);
		fread(data, tileHeader.dataSize, 1, fp);
		
		navmesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	
	fclose(fp);
	return true;
}

bool saveTileNavmesh(const char* path, const dtNavMesh* navmesh, unsigned* tileIndices, unsigned tileCount)
{
	if (!navmesh)
	{
		// no navmesh to save.
		return false;
	}
	
	FILE* fp = fopen(path, "wb");
	if (!fp)
	{
		// can't open the file
		return false;
	}
	
	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	memcpy(&header.params, navmesh->getParams(), sizeof(dtNavMeshParams));
	if (tileIndices)
	{
		// Saving some tiles.
		for (unsigned i = 0; i < tileCount; ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(tileIndices[i]);
			if (!tile || !tile->header || !tile->dataSize) continue;
			header.numTiles++;
		}
	}
	else
	{
		// Saving all tiles.
		for (unsigned i = 0; i < navmesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile || !tile->header || !tile->dataSize) continue;
			header.numTiles++;
		}
	}
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);
	
	// Store tiles.
	if (tileIndices)
	{
		// Saving some tiles.
		for (unsigned i = 0; i < tileCount; ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(tileIndices[i]);
			if (!tile || !tile->header || !tile->dataSize) continue;
			
			NavMeshTileHeader tileHeader;
			tileHeader.tileRef = navmesh->getTileRef(tile);
			tileHeader.dataSize = tile->dataSize;
			fwrite(&tileHeader, sizeof(tileHeader), 1, fp);
			
			fwrite(tile->data, tile->dataSize, 1, fp);
		}
	}
	else
	{
		// Saving all tiles.
		for (unsigned i = 0; i < navmesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile || !tile->header || !tile->dataSize) continue;
			NavMeshTileHeader tileHeader;
			tileHeader.tileRef = navmesh->getTileRef(tile);
			tileHeader.dataSize = tile->dataSize;
			fwrite(&tileHeader, sizeof(tileHeader), 1, fp);
			
			fwrite(tile->data, tile->dataSize, 1, fp);
		}
	}
	
	fclose(fp);
	return true;
}
