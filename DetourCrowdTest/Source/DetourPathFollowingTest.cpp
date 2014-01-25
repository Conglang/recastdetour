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

#include "DetourPathFollowing.h"
#include "DetourCollisionAvoidance.h"

#include <cstring>

SCENARIO("DetourPathFollowingTest/Default", "[detourPathFollowing]")
{
	GIVEN("A default constructed dtPathFollowing")
	{
		dtPathFollowing* pf = dtPathFollowing::allocate(2);
		
		THEN("The default initial pathfind iterations count is 20")
		{
			CHECK(pf->initialPathfindIterCount == 20);
		}
		
		dtPathFollowing::free(pf);
	}
}

SCENARIO("DetourPathFollowingTest/PathFollowingOnly", "[detourPathFollowing]")
{
	const float posAgt1[] = {0, 0, 0};
	const float posAgt2[] = {0, 0, 1};
	
	const dtNavMesh& square = getSquareNavmesh();
	dtCrowd crowd;
	crowd.init(20, 0.5, &square);
	
	dtCrowdAgent ag1, ag2;
	REQUIRE(crowd.addAgent(ag1, posAgt1));
	REQUIRE(crowd.addAgent(ag2, posAgt2));
	
	ag1.maxSpeed = 2.f;
	REQUIRE(crowd.pushAgent(ag1));
	
	ag2.maxSpeed = 3.f;
	REQUIRE(crowd.pushAgent(ag2));
	
	dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
	dtPathFollowingParams* pfParams1 = pf1->getBehaviorParams(crowd.getAgent(ag1.id)->id);
	dtPathFollowingParams* pfParams2 = pf1->getBehaviorParams(crowd.getAgent(ag2.id)->id);
	
	REQUIRE(crowd.pushAgentBehavior(ag1.id, pf1));
	REQUIRE(crowd.pushAgentBehavior(ag2.id, pf1));
	
	pf1->init(*crowd.getCrowdQuery());
	
	GIVEN("Two valid destinations")
	{
		const float destAgt1[] = {-18, 0, 0};
		const float destAgt2[] = {18, 0, 1};
		
		// Set the destinations
		pfParams1->submitTarget(destAgt1,0);
		pfParams2->submitTarget(destAgt2,0);
		
		WHEN("Updated for 3s at 10 Hz")
		{
			for (int i = 0; i < 30; ++i)
				crowd.update(0.1f);
			
			dtCrowdAgent ag1_step1, ag2_step1;
			crowd.fetchAgent(ag1_step1, ag1.id);
			crowd.fetchAgent(ag2_step1, ag2.id);
			
			THEN("Agents have moved at (roughly) their maximum speed")
			{
				CHECK(fabs(dtVdist2D(ag1_step1.position, posAgt1) - ag1.maxSpeed * 3.f) < 0.05f * ag1.maxSpeed * 3.f);
				CHECK(fabs(dtVdist2D(ag2_step1.position, posAgt1) - ag2.maxSpeed * 3.f) < 0.05f * ag2.maxSpeed * 3.f);
			}
			
			THEN("Agents have moved in straight line toward their destination")
			{
				CHECK(fabs(ag1_step1.position[0]-destAgt1[0]) < fabs(ag1.position[0]-destAgt1[0]));
				CHECK(fabs(ag1_step1.position[2]-destAgt1[2]) < 0.1f);
				
				CHECK(fabs(ag2_step1.position[0]-destAgt2[0]) < fabs(ag2.position[0]-destAgt2[0]));
				CHECK(fabs(ag2_step1.position[2]-destAgt2[2]) < 0.1f);
				
				AND_WHEN("The second agent's target is cleared")
				{
					pfParams2->clearTarget();
					
					for (int i = 0; i < 30; ++i)
						crowd.update(0.1f);
					
					dtCrowdAgent ag1_step2, ag2_step2;
					crowd.fetchAgent(ag1_step2, ag1.id);
					crowd.fetchAgent(ag2_step2, ag2.id);
					
					THEN("The first agent have continued to move in straight line, the second is standing still")
					{
						CHECK(fabs(ag1_step1.position[0]-destAgt1[0]) < fabs(ag1.position[0]-destAgt1[0]));
						CHECK(fabs(ag1_step1.position[2]-destAgt1[2]) < 0.1f);
						
						CHECK(dtVdist2D(ag2_step2.position, ag2_step1.position) < 0.5f); // Larger threshold to take into account the non-instant stop
					}
				}
			}
		}
		
		WHEN("Updated for 3s at 10000 Hz")
		{
			for (int i = 0; i < 30000; ++i)
				crowd.update(0.0001f);
			
			THEN("Agents have moved at (roughly) their maximum speed")
			{
				CHECK(fabs(dtVdist2D(crowd.getAgent(ag1.id)->position, posAgt1) - ag1.maxSpeed * 3.f) < 0.05f * ag1.maxSpeed * 3.f);
				CHECK(fabs(dtVdist2D(crowd.getAgent(ag2.id)->position, posAgt1) - ag2.maxSpeed * 3.f) < 0.05f * ag2.maxSpeed * 3.f);
			}
		}
	}
	
	dtPathFollowing::free(pf1);
}

SCENARIO("DetourPathFollowingTest/PathFollowingAndCollisionAvoidance", "[detourPathFollowing]")
{
	const float a1Position[] = {0, 0, 0};
	
	const dtNavMesh& square = getSquareNavmesh();
	dtCrowd crowd;
	crowd.init(20, 0.5, &square);
	
	dtCrowdAgent a1;
	REQUIRE(crowd.addAgent(a1, a1Position));
	
	a1.maxSpeed = 2.f;
	REQUIRE(crowd.pushAgent(a1));
	
	dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();
	dtPathFollowing* pathFollowing = dtPathFollowing::allocate(12);
	dtCollisionAvoidance* collisionAvoidance = dtCollisionAvoidance::allocate(12);
	dtBehavior* pipelineBehaviors[] = {pathFollowing, collisionAvoidance};
	pipeline->setBehaviors(pipelineBehaviors, 2);
	
	dtPathFollowingParams* pathFollowingParams = pathFollowing->getBehaviorParams(a1.id);
	
	crowd.pushAgentBehavior(a1.id, pipeline);
	
	pathFollowing->init(*crowd.getCrowdQuery());
	collisionAvoidance->init();
	
	GIVEN("A valid destination")
	{
		const float a1Destination[] = {-18, 0, 0};
		pathFollowingParams->submitTarget(a1Destination, 0);
		
		WHEN("Updated once")
		{
			crowd.update(0.1f);
			
			THEN("The target polygon has been computed")
			{
				CHECK(crowd.getCrowdQuery()->getNavMeshQuery()->isValidPolyRef(pathFollowingParams->targetRef, crowd.getCrowdQuery()->getQueryFilter()));
				dtPolyRef poly;
				CHECK(dtStatusSucceed(crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(
																								 a1Destination,
																								 crowd.getCrowdQuery()->getQueryExtents(),
																								 crowd.getCrowdQuery()->getQueryFilter(),
																								 &poly,
																								 0)));
				CHECK(poly == pathFollowingParams->targetRef);
			}
		}
		
		WHEN("Updated for 3s at 10 Hz")
		{
			for (int i = 0; i < 30; ++i)
				crowd.update(0.1f);
			
			THEN("Agent have moved at (roughly) its maximum speed")
			{
				CHECK(fabs(dtVdist2D(crowd.getAgent(a1.id)->position, a1Position) - a1.maxSpeed * 3.f) < 0.05f * a1.maxSpeed * 3.f);
			}
		}
		
		WHEN("Updated for 3s at 10000 Hz")
		{
			for (int i = 0; i < 30000; ++i)
				crowd.update(0.0001f);
			
			THEN("Agent have moved at (roughly) its maximum speed")
			{
				CHECK(fabs(dtVdist2D(crowd.getAgent(a1.id)->position, a1Position) - a1.maxSpeed * 3.f) < 0.05f * a1.maxSpeed * 3.f);
			}
		}
		
		WHEN("Updated for 3s at 100 Hz, new request each tick")
		{
			for (int i = 0; i < 300; ++i)
			{
				pathFollowingParams->submitTarget(pathFollowingParams->targetPos, pathFollowingParams->targetRef);
				crowd.update(0.01f);
			}
			
			THEN("Agent have moved at (roughly) its maximum speed")
			{
				CHECK(fabs(dtVdist2D(crowd.getAgent(a1.id)->position, a1Position) - a1.maxSpeed * 3.f) < 0.05f * a1.maxSpeed * 3.f);
			}
		}
	}
	
	GIVEN("An invalid destination")
	{
		const float invalidTarget[] = {-1000, 0, 0};
		pathFollowing->getBehaviorParams(a1.id)->submitTarget(invalidTarget, 0);
		WHEN("Updated once")
		{
			crowd.update(0.1f);
			
			THEN("The target is invalid")
			{
				CHECK(pathFollowing->getBehaviorParams(a1.id)->targetRef == 0);
				CHECK(pathFollowing->getBehaviorParams(a1.id)->state == pathFollowingParams->INVALID_TARGET);
			}
		}
	}
	
	dtPipelineBehavior::free(pipeline);
	dtPathFollowing::free(pathFollowing);
	dtCollisionAvoidance::free(collisionAvoidance);
}