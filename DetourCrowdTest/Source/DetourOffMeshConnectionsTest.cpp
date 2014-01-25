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

SCENARIO("DetourCrowdTest/OffMeshConnections", "[offmesh] Check if the agents know when they are walking on offMesh connections or using them")
{
	GIVEN("An offMesh connection at (0, 0, 0) with a radius of 1")
	{
		const dtNavmeshInputGeometry& squareGeometry = getSquareMesh();

		dtTiledNavmeshCfg configuration;
		configuration.offmeshConnectionsCount = 1;
		// Creation of the offMesh connection
		configuration.offmeshConnections[0] = dtOffmeshConnectionCfg();
		dtVset( configuration.offmeshConnections[0].start, 0, 0.2, 0);
		dtVset( configuration.offmeshConnections[0].end, 5, 0.2, 5);
		configuration.offmeshConnections[0].radius = 1.f;
		configuration.offmeshConnections[0].isBidirectionnal = true;
		configuration.computeTileCount(squareGeometry.bmin, squareGeometry.bmax, 20);

		dtNavMesh navmesh;
		TestBuildContext context;
		dtCreateTiledNavmesh(squareGeometry, configuration, navmesh, &context);
		dtCrowd crowd;
		crowd.init(2, 0.5, &navmesh);
		
		float posAgt1[] = {0, 0, 0};
		dtCrowdAgent ag;

		// Adding the agent to the crowd
		REQUIRE(crowd.addAgent(ag, posAgt1));
		
		WHEN("An agent is placed at (0, 0, 0")
		{
			THEN("The agent detects the offMesh connection it is on")
			{
				CHECK(crowd.getCrowdQuery()->getOffMeshConnection(ag.id) != 0);
			}			
		}
		WHEN("The agent is moved to (-2, 0, 0)")
		{
			float pos2[] = {-2, 0, 0};
			CHECK(crowd.pushAgentPosition(ag.id, pos2));

			THEN("It doesn't detect the offMesh connection anymore")
			{
				CHECK(crowd.getCrowdQuery()->getOffMeshConnection(ag.id) == 0);
			}
		}
		WHEN("We ask the same to check for a radius of 1.1 around him")
		{
			dtOffMeshConnection* offmeshConnection = crowd.getCrowdQuery()->getOffMeshConnection(ag.id, 1.1f);
			THEN("It detects the offMesh connection again")
			{
				CHECK(offmeshConnection != 0);
			}

			THEN("The retrieved connection have the right 'start' position.")
			{
				CHECK(offmeshConnection->pos[0] == configuration.offmeshConnections[0].start[0]);
				CHECK(offmeshConnection->pos[1] == configuration.offmeshConnections[0].start[1]);
				CHECK(offmeshConnection->pos[2] == configuration.offmeshConnections[0].start[2]);
			}

			THEN("The retrieved connection have the right 'end' position.")
			{
				CHECK(offmeshConnection->pos[3] == configuration.offmeshConnections[0].end[0]);
				CHECK(offmeshConnection->pos[4] == configuration.offmeshConnections[0].end[1]);
				CHECK(offmeshConnection->pos[5] == configuration.offmeshConnections[0].end[2]);
			}
		}
		WHEN("We try to detect an offMesh connection with wrong parameters")
		{
			THEN("We don't find it and it does not crash")
			{
				CHECK(crowd.getCrowdQuery()->getOffMeshConnection(9999) == 0);
				CHECK(crowd.getCrowdQuery()->getOffMeshConnection(ag.id, -5.f) == 0);
			}
		}
		WHEN("The agent passes over the connection")
		{
			float pos[] = {-0.2, 0, -0.2};

			CHECK(crowd.pushAgentPosition(ag.id, pos));
			CHECK(crowd.getCrowdQuery()->getOffMeshConnection(ag.id) != 0);

			crowd.fetchAgent(ag, ag.id);
			crowd.getCrowdQuery()->startOffMeshConnection(ag,
				*crowd.getCrowdQuery()->getOffMeshConnection(ag.id));
			crowd.pushAgent(ag);

			THEN("It moves toward the other side of the connection")
			{
				crowd.updatePosition(0.1f);

				crowd.fetchAgent(ag, ag.id);
				CHECK(ag.position[0] > pos[0]);
				CHECK(ag.position[2] > pos[2]);
			}

			THEN("After several updates it reaches the other side")
			{
				for (unsigned i = 0; i < 100; ++i)
					crowd.updatePosition(0.1f);

				crowd.fetchAgent(ag, ag.id);
				CHECK(ag.position[0] > 4.5f);
				CHECK(ag.position[2] > 4.5f);
			}
		}
	}
}
