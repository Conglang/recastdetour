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

#include "DetourAlignmentBehavior.h"
#include "DetourPathFollowing.h"
#include "DetourSeekBehavior.h"

SCENARIO("DetourCrowdTest/DefaultCrowd", "[detourCrowd]")
{
	dtCrowd crowd;
	
	THEN("It has no agents")
	{
		CHECK(crowd.getAgentCount() == 0);
	}
}

SCENARIO("DetourCrowdTest/DefaultCrowdAgent", "[detourCrowd]")
{
	dtCrowdAgent ag;
	
	WHEN("Initialized to default")
	{
		ag.init();
		
		THEN("Radius is 0.2")
		{
			CHECK(ag.radius == 0.2f);
		}
		
		THEN("Height is 1.7")
		{
			CHECK(ag.height == 1.7f);
		}
		
		THEN("Maximum Acceleration is 10")
		{
			CHECK(ag.maxAcceleration == 10.f);
		}
		
		THEN("Maximum Speed is 2")
		{
			CHECK(ag.maxSpeed == 2.f);
		}
		THEN("Perception distance is 4")
		{
			CHECK(ag.detectionRange == 4.f);
		}
		
		THEN("ID is invalid")
		{
			CHECK(ag.id == UINT_MAX);
		}
	}
}

SCENARIO("DetourCrowdTest/FetchingAndUpdatingAgents", "[detourCrowd]")
{
	const dtNavMesh& square = getSquareNavmesh();
	dtCrowd crowd;
	crowd.init(20, 0.5, &square);
	
	float initialPosition[] = {0, 0, 0};
	dtCrowdAgent ag;
	
	// Adding the agent to the crowd
	REQUIRE(crowd.addAgent(ag, initialPosition));
	
	WHEN("The agent has just been added")
	{
		THEN("It has its default values")
		{
			CHECK(ag.radius == 0.2f);
			CHECK(ag.height == 1.7f);
			CHECK(ag.maxAcceleration == 10.f);
			CHECK(ag.maxSpeed == 2.f);
			CHECK(ag.detectionRange == 4.f);
		}
		
		THEN("It has a valid ID")
		{
			CHECK(ag.id != UINT_MAX);
		}
	}
	
	WHEN("The position is updated to valid coordinates")
	{
		float correctPosition[] = {19, 0, 0};
		CHECK(crowd.pushAgentPosition(ag.id, correctPosition));
		
		THEN("The fetched agent's position is updated")
		{
			crowd.fetchAgent(ag, ag.id);
			CHECK(ag.position[0]==correctPosition[0]);
			CHECK(ag.position[2]==correctPosition[2]);
		}
		
		THEN("The directly accessed agent's position is updated")
		{
			CHECK(crowd.getAgent(ag.id)->position[0]==correctPosition[0]);
			CHECK(crowd.getAgent(ag.id)->position[2]==correctPosition[2]);
		}
	}
	
	WHEN("The position is updated to invalid coordinates")
	{
		float wrongPosition[] = {100, 0, 10};
		CHECK_FALSE(crowd.pushAgentPosition(ag.id, wrongPosition));
		
		THEN("The fetched agent's position is not updated")
		{
			crowd.fetchAgent(ag, ag.id);
			CHECK(ag.position[0]==initialPosition[0]);
			CHECK(ag.position[2]==initialPosition[2]);
		}
	}
	
	dtCrowdAgent fetched;
	
	WHEN("Fetching an agent with an invalid id")
	{
		fetched.maxAcceleration = 999;
		
		crowd.fetchAgent(fetched, UINT_MAX);
		
		THEN("The given agent data structure is left unchanged")
		{
			CHECK(fetched.maxAcceleration == 999);
		}
	}
	
	WHEN("Fetching an agent with an existing agent id")
	{
		crowd.fetchAgent(fetched, ag.id);
		
		THEN("The given agent data structure is updated")
		{
			CHECK(fetched.maxAcceleration == ag.maxAcceleration);
		}
	}
	
	dtCrowdAgent toApply;
	toApply.init();
	toApply.radius = 9.f;
	REQUIRE(toApply.radius != ag.radius);
	
	WHEN("Applying new values for an agent data with an invalid id")
	{
		toApply.id = UINT_MAX;
		
		CHECK_FALSE(crowd.pushAgent(toApply));
		
		THEN("The existing agent's data structure is not updated")
		{
			CHECK(crowd.getAgent(ag.id)->radius == ag.radius);
		}
	}
	
	WHEN("Applying new values for an agent data with a valid id")
	{
		toApply.id = ag.id;
		
		CHECK(crowd.pushAgent(toApply));
		
		THEN("The existing agent's data structure is updated")
		{
			CHECK(crowd.getAgent(ag.id)->radius == toApply.radius);
		}
	}
}

SCENARIO("DetourCrowdTest/UpdateCrowd", "[detourCrowd] Test the different ways to update the agents inside a crowd")
{
	dtCrowdAgent ag1, ag2, ag3, ag4;
	
	const float posAgt1[] = {-18, 0, -18};
	const float posAgt2[] = {18, 0, 18};
	const float posAgt3[] = {-18, 0, 18};
	const float posAgt4[] = {18, 0, -18};
	
	const float destAgt1[] = {18, 0, 18};
	const float destAgt2[] = {-18, 0, -18};
	const float destAgt3[] = {18, 0, -18};
	const float destAgt4[] = {-18, 0, 18};
	
	GIVEN("A scene with 2 agents having path following behaviors")
	{
		// Creation of the simulation
		const dtNavMesh& square = getSquareNavmesh();
		dtCrowd crowd;
		crowd.init(4, 0.5, &square);
		
		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));
		
		dtPathFollowing* pf1 = dtPathFollowing::allocate(5);
		
		pf1->init(*crowd.getCrowdQuery());
		
		crowd.pushAgentBehavior(ag1.id, pf1);
		crowd.pushAgentBehavior(ag2.id, pf1);
		
		// Set the destination
		dtPolyRef dest1, dest2;
		crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest1, 0);
		crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest2, 0);
		
		pf1->getBehaviorParams(ag1.id)->submitTarget(destAgt1, dest1);
		pf1->getBehaviorParams(ag2.id)->submitTarget(destAgt2, dest2);
		
		WHEN("The given dt is nil")
		{
			crowd.update(0.f);
			
			THEN("The agents have not moved")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] == posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] == posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] == posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] == posAgt2[2]);
			}
		}
		
		WHEN("Updating all agents")
		{
			crowd.update(1.0, 0);
			
			THEN("ag1 and ag2 have moved")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] != posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] != posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
			}
			
			THEN("Their navmesh polygon is valid")
			{
				const dtNavMesh* nm = crowd.getCrowdQuery()->getNavMeshQuery()->getAttachedNavMesh();
				crowd.fetchAgent(ag1, ag1.id);
				CHECK(nm->isValidPolyRef(ag1.poly));
				
				crowd.fetchAgent(ag2, ag2.id);
				CHECK(nm->isValidPolyRef(ag2.poly));
		}
		}
		
		WHEN("Updating all agents specifically")
		{
			unsigned list[] = {ag2.id, ag1.id};
			
			crowd.update(1.0, list, 2);
			
			THEN("ag1 and ag2 have moved")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] != posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] != posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
			}
		}
		
		WHEN("Updating using a list that is too large")
		{
			unsigned list[] = {15, ag2.id, ag1.id, 12};
			
			crowd.update(1.0, list, 4);
			
			THEN("ag1 and ag2 have moved")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] != posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] != posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
			}
		}
		
		WHEN("Updating using a list having only invalid ids")
		{
			unsigned list[] = {15, 12};
			
			crowd.update(1.0, list, 2);
			
			THEN("Neither ag1 or ag2 have moved")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] == posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] == posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] == posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] == posAgt2[2]);
			}
		}
		
		WHEN("Updating the first agent")
		{
			crowd.update(1.0, &ag1.id, 1);
			
			THEN("ag1 have moved, ag2 haven't")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] != posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] != posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] == posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] == posAgt2[2]);
			}
		}
		
		WHEN("Updating the second agent")
		{
			crowd.update(1.0, &ag2.id, 1);
			
			THEN("ag2 have moved, ag1 haven't")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] == posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] == posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
			}
		}
		
		WHEN("Updating for 1 second at 1000Hz")
		{
			for (unsigned i(0) ; i < 1000 ; ++i)
			{
				crowd.update(1.f / 1000.f);
			}
			
			THEN("ag1 and ag2 have moved")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] != posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] != posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
			}
		}
		
		float velAgt1[3], velAgt2[3];
		dtVcopy(velAgt1, crowd.getAgent(ag1.id)->velocity);
		dtVcopy(velAgt2, crowd.getAgent(ag2.id)->velocity);
		
		WHEN("Updating only the environment")
		{
			crowd.updateEnvironment();
			
			THEN("Agents have not moved and their velocity has not been updated")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] == posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] == posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] == posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] == posAgt2[2]);
				
				CHECK(crowd.getAgent(ag1.id)->velocity[0] == velAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->velocity[2] == velAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->velocity[0] == velAgt1[0]);
				CHECK(crowd.getAgent(ag2.id)->velocity[2] == velAgt1[2]);
			}
		}
		
		WHEN("Creating an agent near Agent 1")
		{
			float posAgt3[3] = {
				crowd.getAgent(ag1.id)->position[0] + 1,
				0 ,
				crowd.getAgent(ag1.id)->position[2] + 1};
			dtCrowdAgent ag3;
			
			REQUIRE(crowd.addAgent(ag3, posAgt3));
			
			crowd.pushAgentBehavior(ag3.id, pf1);
			
			THEN("The new agent can 'see' agent 1")
			{
				crowd.updateEnvironment(&ag3.id, 1);
				
				CHECK(crowd.getAgentEnvironment(ag3.id)->nbNeighbors == 1);
				CHECK(crowd.getAgentEnvironment(ag3.id)->neighbors[0].idx == ag1.id);
				
				// The other agents on the other hand haven't updated its environment, and therefore should have no neighbors
				CHECK(crowd.getAgentEnvironment(ag1.id)->nbNeighbors == 0);
			}
			
			THEN("A nil perception blinds it")
			{
				crowd.fetchAgent(ag3, ag3.id);
				ag3.detectionRange = 0.f;
				crowd.pushAgent(ag3);
				
				crowd.updateEnvironment(&ag3.id, 1);
				
				CHECK(crowd.getAgentEnvironment(ag3.id)->nbNeighbors == 0);
			}
			
			THEN("Updating the environment for all make them 'see' each others")
			{
				// Now we update everyone's environment, thus every agent has one neighbor
				crowd.updateEnvironment();
				CHECK(crowd.getAgentEnvironment(ag3.id)->nbNeighbors == 1);
				CHECK(crowd.getAgentEnvironment(ag1.id)->nbNeighbors == 1);
			}
		}
		
		WHEN("Updating only the velocity")
		{
			crowd.updateVelocity(0.2f);
			
			THEN("Agents have not moved but their velocity has been updated")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] == posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] == posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] == posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] == posAgt2[2]);
				
				CHECK(crowd.getAgent(ag1.id)->velocity[0] != velAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->velocity[2] != velAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->velocity[0] != velAgt1[0]);
				CHECK(crowd.getAgent(ag2.id)->velocity[2] != velAgt1[2]);
			}
		}
		
		WHEN("Updating velocity, then position")
		{
			crowd.updateVelocity(0.2f, 0);
			crowd.updatePosition(0.2f, 0);
			
			THEN("Agents have moved and their velocity have been updated")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] != posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] != posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
				
				CHECK(crowd.getAgent(ag1.id)->velocity[0] != velAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->velocity[2] != velAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->velocity[0] != velAgt1[0]);
				CHECK(crowd.getAgent(ag2.id)->velocity[2] != velAgt1[2]);
			}
		}
	}
	
	GIVEN("A scene with 4 agents having path following behaviors")
	{
		// Creation of the simulation
		const dtNavMesh& square = getSquareNavmesh();
		dtCrowd crowd;
		crowd.init(4, 0.5, &square);
		
		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));
		REQUIRE(crowd.addAgent(ag3, posAgt3));
		REQUIRE(crowd.addAgent(ag4, posAgt4));
		
		dtPathFollowing* pf1 = dtPathFollowing::allocate(5);
		
		pf1->init(*crowd.getCrowdQuery());
		
		crowd.pushAgentBehavior(ag1.id, pf1);
		crowd.pushAgentBehavior(ag2.id, pf1);
		crowd.pushAgentBehavior(ag3.id, pf1);
		crowd.pushAgentBehavior(ag4.id, pf1);
		
		// Set the destination
		dtPolyRef dest1, dest2, dest3, dest4;
		crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest1, 0);
		crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest2, 0);
		crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt1, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest3, 0);
		crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(posAgt2, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest4, 0);
		
		pf1->getBehaviorParams(ag1.id)->submitTarget(destAgt1, dest1);
		pf1->getBehaviorParams(ag2.id)->submitTarget(destAgt2, dest2);
		pf1->getBehaviorParams(ag3.id)->submitTarget(destAgt3, dest3);
		pf1->getBehaviorParams(ag4.id)->submitTarget(destAgt4, dest4);
		
		WHEN("The first agent is removed and then the crowd updated")
		{
			crowd.removeAgent(ag1.id);
			crowd.update(0.5);
			
			THEN("every agent has moved but the removed one")
			{
				CHECK(crowd.getAgent(ag1.id)->position[0] == posAgt1[0]);
				CHECK(crowd.getAgent(ag1.id)->position[2] == posAgt1[2]);
				CHECK(crowd.getAgent(ag2.id)->position[0] != posAgt2[0]);
				CHECK(crowd.getAgent(ag2.id)->position[2] != posAgt2[2]);
				CHECK(crowd.getAgent(ag3.id)->position[0] != posAgt3[0]);
				CHECK(crowd.getAgent(ag3.id)->position[2] != posAgt3[2]);
				CHECK(crowd.getAgent(ag4.id)->position[0] != posAgt4[0]);
				CHECK(crowd.getAgent(ag4.id)->position[2] != posAgt4[2]);
			}
		}
	}
}


