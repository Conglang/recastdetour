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

#include "Utils.h"

#include <DetourNavmeshCfg.h>

#include <DetourCommon.h>

#include <cstring>

SCENARIO("DetourNavmeshCfg/Basics", "[navmeshCfg]")
{
	GIVEN("A default tiled navmesh configuration")
	{
		dtTiledNavmeshCfg configuration;
		THEN("Default values are the expected ones")
		{
			CHECK(configuration.bmin[0] == 0.f);
			CHECK(configuration.bmin[1] == 0.f);
			CHECK(configuration.bmin[2] == 0.f);

			CHECK(configuration.bmax[0] == 0.f);
			CHECK(configuration.bmax[1] == 0.f);
			CHECK(configuration.bmax[2] == 0.f);

			CHECK(configuration.voxels.size == 0.3f);
			CHECK(configuration.voxels.height == 0.2f);

			CHECK(configuration.tiles.xCount == 0);
			CHECK(configuration.tiles.zCount == 0);
			CHECK(configuration.tiles.size == 32);

			CHECK(configuration.navigation.minimumCeilingClearance == 2.f);
			CHECK(configuration.navigation.maximumStepHeight == 0.9f);
			CHECK(configuration.navigation.minimumObstacleClearance == 0.3f);
			CHECK(configuration.navigation.maximumSlope == 45.f);

			CHECK(configuration.regions.monotonePartioning == false);
			CHECK(configuration.regions.minSize == 64);
			CHECK(configuration.regions.mergeSize == 400);

			CHECK(configuration.polyMesh.edgeMaxError == 1.3f);
			CHECK(configuration.polyMesh.edgeMaxLength == 12.f);
			CHECK(configuration.polyMesh.polyMaxNbVertices == 6);
			CHECK(configuration.polyMesh.sampleDist == 6);
			CHECK(configuration.polyMesh.sampleMaxError == 1);

			CHECK(configuration.offmeshConnectionsCount == 0);
		}

		WHEN("Computing the tile count")
		{
			const float bmin[] = {0.f, 0.f, 0.f};
			const float bmax[] = {10.f, 10.f, 22.f};
			const float voxelSize = 0.501f;
			const unsigned tileSize = 10;

			configuration.voxels.size = voxelSize;
			configuration.computeTileCount(bmin, bmax, tileSize);

			THEN("The bbox is updated")
			{
				CHECK(configuration.bmin[0] == bmin[0]);
				CHECK(configuration.bmin[1] == bmin[1]);
				CHECK(configuration.bmin[2] == bmin[2]);

				CHECK(configuration.bmax[0] == bmax[0]);
				CHECK(configuration.bmax[1] == bmax[1]);
				CHECK(configuration.bmax[2] == bmax[2]);
			}

			THEN("The tile size is updated")
			{
				CHECK(configuration.tiles.size == tileSize);
			}

			THEN("The tile count is computed")
			{
				CHECK(configuration.tiles.xCount == 2);
				CHECK(configuration.tiles.zCount == 5);
			}
		}
	}
}