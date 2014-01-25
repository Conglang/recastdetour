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

#include <DetourNavmeshCreator.h>

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourCommon.h>

#include <cstring>

SCENARIO("DetourNavmeshCreator/Basics", "[navmeshCreator]")
{
	TestBuildContext context;
	const char* sampleObj = "# vertices \n\
							v 12.0 0.0 6.0 \n\
							v 12.0 0.0 -6.0 \n\
							v -12.0 0.0 -6.0 \n\
							v -12.0 0.0 6.0 \n\
							# faces \n\
							f 3 4 1 \n\
							f 1 2 3";
	unsigned sampleObjSize = strlen(sampleObj);

	GIVEN("A simple configuration and geometry")
	{
		dtNavmeshInputGeometry geometry;
		geometry.maximumFacesPerTreeNode = 1;
		REQUIRE(loadObjBuffer(sampleObj, sampleObjSize, geometry.mesh));

		CHECK(geometry.initialize());

		THEN("The input geometry is well initialized")
		{
			CHECK(geometry.bmin[0] == -12.f);
			CHECK(geometry.bmin[1] == 0.f);
			CHECK(geometry.bmin[2] == -6.f);
			CHECK(geometry.bmax[0] == 12.f);
			CHECK(geometry.bmax[1] == 0.f);
			CHECK(geometry.bmax[2] == 6.f);
			CHECK(geometry.tree.m_facesCount == 2);
			CHECK(geometry.tree.m_nodesCount == 3);
		}

		dtTiledNavmeshCfg configuration;
		configuration.voxels.size = 0.5f;
		configuration.computeTileCount(geometry.bmin, geometry.bmax, 10);

		configuration.offmeshConnectionsCount = 1;
		configuration.offmeshConnections[0] = dtOffmeshConnectionCfg();
		dtVset(configuration.offmeshConnections[0].start, 1.f, 0.f, 1.f);
		dtVset(configuration.offmeshConnections[0].end, 3.f, 0.f, 3.f);

		THEN("The configuration is well initialized")
		{
			CHECK(configuration.bmin[0] == geometry.bmin[0]);
			CHECK(configuration.bmin[1] == geometry.bmin[1]);
			CHECK(configuration.bmin[2] == geometry.bmin[2]);
			CHECK(configuration.bmax[0] == geometry.bmax[0]);
			CHECK(configuration.bmax[1] == geometry.bmax[1]);
			CHECK(configuration.bmax[2] == geometry.bmax[2]);

			CHECK(configuration.tiles.xCount == 5);
			CHECK(configuration.tiles.zCount == 3);
		}

		WHEN("A navmesh is computed")
		{
			dtNavMesh navmesh;
			dtTiledNavmeshCreatorIntermediateResults intermediateResults;
			CHECK(dtCreateTiledNavmesh(geometry, configuration, intermediateResults, navmesh, &context));

			THEN("The navmesh has the expected tiles")
			{
				CHECK(navmesh.getParams()->maxTiles == 16); // next power of 2 after 15
				CHECK(navmesh.getParams()->tileHeight == configuration.tiles.size * configuration.voxels.size);
				CHECK(navmesh.getParams()->tileWidth == configuration.tiles.size * configuration.voxels.size);
			}

			dtNavMeshQuery query;
			query.init(&navmesh, 36);
			dtQueryFilter filter;
			const float extent[] = {1,1,1};

			dtPolyRef nearestPoly;
			float nearestPosition[3];

			THEN("A position in the middle is valid")
			{
				const float pos[] = {5,0,5};

				CHECK(dtStatusSucceed(query.findNearestPoly(pos, extent, &filter, &nearestPoly, nearestPosition)));
				CHECK(nearestPoly != 0);
				CHECK(nearestPosition[0] == pos[0]);
				CHECK(nearestPosition[2] == pos[2]);
			}

			THEN("A position near the edge is valid")
			{
				const float pos[] = {-11,0,5};

				CHECK(dtStatusSucceed(query.findNearestPoly(pos, extent, &filter, &nearestPoly, nearestPosition)));
				CHECK(nearestPoly != 0);
				CHECK(nearestPosition[0] == pos[0]);
				CHECK(nearestPosition[2] == pos[2]);
			}
		}
	}
}