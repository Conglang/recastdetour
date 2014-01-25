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

#include <DetourCollisionAvoidance.h>
#include <DetourCrowd.h>
#include <DetourPathFollowing.h>
#include <DetourPipelineBehavior.h>

#include <cstring>

SCENARIO("Regression/SinkingAgent", "[regression]")
{
	GIVEN("The computed navmesh for the map 'generation bug #1'")
	{
		dtNavMesh navmesh;
		CHECK(loadTiledNavMesh(&navmesh, "../Data/sinking_agent.dtNavmesh"));

		WHEN("An agent is created at (13912.80, 6.25, -12548.90) with a path following target at (13888.73, 12.60, -12586.23)")
		{
			dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();
			dtPathFollowing* pathFollowing = dtPathFollowing::allocate(5);
			dtCollisionAvoidance* collisionAvoidance = dtCollisionAvoidance::allocate(5);
			dtBehavior* behaviors[] = {pathFollowing, collisionAvoidance};
			pipeline->setBehaviors(behaviors, 2);
			
			const float from[] = {13912.80, 6.25, -12548.90};
			const float to[] = {13888.73, 12.60, -12586.23};
			
			dtCrowd crowd;
			crowd.init(5, 1.f, &navmesh);
			
			dtCrowdAgent ag;
			CHECK(crowd.addAgent(ag, from));
			CHECK(crowd.pushAgentBehavior(ag.id, pipeline));
			
			pathFollowing->init(*crowd.getCrowdQuery());
			pathFollowing->getBehaviorParams(ag.id)->submitTarget(to);
			
			THEN("It goes smoothly to its target")
			{
				const float dt = 0.1f;
				float t = 0.f;
				unsigned i = 0;
				
				while (t < 16.f)
				{
					crowd.update(dt);
					t += dt;
					CAPTURE(t);
					
					++i;
					CAPTURE(i);
					
					float previousPosition[3];
					dtVcopy(previousPosition, ag.position);
					CAPTURE(previousPosition[0]);
					CAPTURE(previousPosition[1]);
					CAPTURE(previousPosition[2]);
					dtPolyRef previousNavmeshPoly;
					previousNavmeshPoly = ag.poly;
					CAPTURE(previousNavmeshPoly);
					
					crowd.fetchAgent(ag, ag.id);
					CAPTURE(ag.position[0]);
					CAPTURE(ag.position[1]);
					CAPTURE(ag.position[2]);
					CAPTURE(ag.poly);
					
					CHECK(crowd.getCrowdQuery()->getNavMeshQuery()->getAttachedNavMesh()->isValidPolyRef(ag.poly));
					
					
					float navmeshPositionCheck[3];
					CHECK(dtStatusSucceed(crowd.getCrowdQuery()->getNavMeshQuery()->closestPointOnPoly(ag.poly, ag.position, navmeshPositionCheck)));
					CHECK(ag.position[0] == navmeshPositionCheck[0]);
					CHECK(ag.position[1] == navmeshPositionCheck[1]);
					CHECK(ag.position[2] == navmeshPositionCheck[2]);
					
					CAPTURE(ag.velocity[0]);
					CAPTURE(ag.velocity[1]);
					CAPTURE(ag.velocity[2]);
										
					CHECK(dtVdist(previousPosition, ag.position) < (ag.maxSpeed + 1.f) * dt);
				}
			}
			
			dtPipelineBehavior::free(pipeline);
			dtPathFollowing::free(pathFollowing);
			dtCollisionAvoidance::free(collisionAvoidance);
		}
		
		WHEN("An agent is created at (13896.5, 13.6869, -12574.2) with a velocity of (-1.0824, 0, -1.68179)")
		{
			const float from[] = {13896.5, 13.6869, -12574.2};
			const dtPolyRef fromPoly = 4980805;
			const float velocity[] = {-1.0824, 0, -1.68179};
			
			dtCrowd crowd;
			crowd.init(5, 1.f, &navmesh);
			
			dtCrowdAgent ag;
			CHECK(crowd.addAgent(ag, from));
			CHECK(ag.poly == fromPoly);
			CHECK(dtVdist(ag.position, from) < 0.0001f);

			dtVcopy(ag.velocity, velocity);
			CHECK(crowd.pushAgent(ag));
			
			THEN("The position is well updated")
			{
				const float dt = 0.1f;
				
				crowd.updatePosition(dt);
				
				float previousPosition[3];
				dtVcopy(previousPosition, ag.position);
				CAPTURE(previousPosition[0]);
				CAPTURE(previousPosition[1]);
				CAPTURE(previousPosition[2]);
				dtPolyRef previousNavmeshPoly;
				previousNavmeshPoly = ag.poly;
				CAPTURE(previousNavmeshPoly);
				
				float expectedPosition[3];
				dtVmad(expectedPosition, previousPosition, velocity, dt);
				CAPTURE(expectedPosition[0]);
				CAPTURE(expectedPosition[1]);
				CAPTURE(expectedPosition[2]);
				
				crowd.fetchAgent(ag, ag.id);
				CAPTURE(ag.position[0]);
				CAPTURE(ag.position[1]);
				CAPTURE(ag.position[2]);
				CAPTURE(ag.poly);
				
				CHECK(dtVdist2D(ag.position, ag.position) < EPSILON);
				CHECK(dtVdist2D(expectedPosition, ag.position) < 0.001f);
				CHECK(fabs(expectedPosition[1] - ag.position[1]) < 0.01f);
			}
		}
		
		WHEN("Moving along navmesh surface from (13896.5, 13.6869, -12574.2) to (13896.4, 13.6869, -12574.4)")
		{
			const float from[] = {13896.5, 13.6869, -12574.2};
			const dtPolyRef fromPoly = 4980805;
			const float to[] = {13896.4, 13.6869, -12574.4};
			
			dtCrowd crowd;
			crowd.init(5, 1.f, &navmesh);
			
			const dtCrowdQuery* crowdQuery = crowd.getCrowdQuery();
			REQUIRE(crowdQuery);
			
			const dtNavMeshQuery* navmeshQuery = crowdQuery->getNavMeshQuery();
			REQUIRE(navmeshQuery);

			float actualTo[3];
			dtPolyRef actualToPoly;
			const int maxVisited = 5;
			dtPolyRef visited[maxVisited];
			memset(visited, 0, maxVisited * sizeof(dtPolyRef));
			int visitedCount;
			
			CHECK(dtStatusSucceed(navmeshQuery->moveAlongSurface(fromPoly,
																 from,
																 to,
																 crowdQuery->getQueryFilter(),
																 &actualToPoly,
																 actualTo,
																 visited,
																 &visitedCount,
																 maxVisited)));
			CAPTURE(visitedCount);
			CAPTURE(visited[0]);
			CAPTURE(visited[1]);
			CAPTURE(visited[2]);
			CAPTURE(visited[3]);
			CAPTURE(visited[4]);
			CAPTURE(visited[5]);
			
			navmeshQuery->getPolyHeight(actualToPoly, actualTo, &actualTo[1]);
			CAPTURE(actualTo[0]);
			CAPTURE(actualTo[1]);
			CAPTURE(actualTo[2]);
			CAPTURE(actualToPoly);
			
			THEN("The located origin is the expected one")
			{
				dtPolyRef locatedFromPoly;
				float locatedFrom[3];
				
				REQUIRE(dtStatusSucceed(navmeshQuery->findNearestPoly(from,
																	  crowdQuery->getQueryExtents(),
																	  crowdQuery->getQueryFilter(),
																	  &locatedFromPoly,
																	  locatedFrom)));
				CAPTURE(locatedFrom[0]);
				CAPTURE(locatedFrom[1]);
				CAPTURE(locatedFrom[2]);
				CHECK(dtVdist(from, locatedFrom) < 0.0001f);
				CHECK(locatedFromPoly == fromPoly);
			}
			
			THEN("The actual arrival position is (close to) the expected one")
			{
				CHECK(dtVdist2D(actualTo, to) < 0.0001f);
				CHECK(fabs(actualTo[1] - to[1]) < 0.1f);
			}
			
			THEN("The arrival position and poly are coherent")
			{
				float checkTo[3];
				CHECK(dtStatusSucceed(crowd.getCrowdQuery()->getNavMeshQuery()->closestPointOnPoly(actualToPoly, actualTo, checkTo)));
				CAPTURE(checkTo[0]);
				CAPTURE(checkTo[1]);
				CAPTURE(checkTo[2]);
				
				CHECK(dtVdist(actualTo, checkTo) < 0.0001f);
			}
		}
	}
}
