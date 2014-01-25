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
#include "DetourCohesionBehavior.h"
#include "DetourGoToBehavior.h"
#include "DetourPathFollowing.h"
#include "DetourSeekBehavior.h"
#include "DetourSeparationBehavior.h"
#include "DetourFlockingBehavior.h"
#include <cstring>

TEST_CASE("DetourBehaviorsTests/CustomBehavior", "[debug] Test whether the custom behaviors behave correctly")
{
	const dtNavMesh& square = getSquareNavmesh();
	dtCrowd crowd;
	CHECK(crowd.init(20, 0.5, &square));
	
	SECTION("Seek Behavior", "With the seeking behavior, an agent must move towards its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {15, 0, 0};

		dtCrowdAgent ag1, ag2;

		dtSeekBehavior* seek = dtSeekBehavior::allocate(5);
		dtSeekBehaviorParams* params = seek->getBehaviorParams(crowd.getAgent(0)->id);
		params->targetID = 1;
		params->distance = 0;
		params->predictionFactor = 0;

		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));

		crowd.pushAgentBehavior(ag1.id, seek);
				
		crowd.update(2.0, 0);

		float agt1NewPos[3];

		dtVcopy(agt1NewPos, crowd.getAgent(ag1.id)->position);	

		// The first agent should move to the right (since he is seeking the second agent)
		CHECK(posAgt1[0] < crowd.getAgent(ag2.id)->position[0]);

		dtSeekBehavior::free(seek);
		crowd.pushAgentBehavior(ag1.id, 0);
	}

	SECTION("Flocking Behavior", "Unit tests about the flocking behavior")
	{
		dtPathFollowing* pf = dtPathFollowing::allocate(1);
		dtFlockingBehavior* flocking = dtFlockingBehavior::allocate(5, 2, 1, 1, 1);
		dtCrowdAgent ag1, ag2, ag3, ag4, ag5;

		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {1, 0, 1};
		float posAgt3[] = {1.5f, 0, 1};
		float posAgt4[] = {1, 0, 1.5f};
		float posLeader[] = {-0.5f, 0, -0.5f};
		float destLeader[] = {-14, 0, -14};

		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));
		REQUIRE(crowd.addAgent(ag3, posAgt3));
		REQUIRE(crowd.addAgent(ag4, posAgt4));
		REQUIRE(crowd.addAgent(ag5, posLeader));

		dtPathFollowingParams* pfParams = pf->getBehaviorParams(crowd.getAgent(4)->id);

		dtFlockingBehaviorParams* flockParams = flocking->getBehaviorParams(crowd.getAgent(0)->id);
		dtFlockingBehaviorParams* flockParams2 = flocking->getBehaviorParams(crowd.getAgent(1)->id);
		dtFlockingBehaviorParams* flockParams3 = flocking->getBehaviorParams(crowd.getAgent(2)->id);
		dtFlockingBehaviorParams* flockParams4 = flocking->getBehaviorParams(crowd.getAgent(3)->id);

		crowd.pushAgentBehavior(ag1.id, flocking);
		crowd.pushAgentBehavior(ag2.id, flocking);
		crowd.pushAgentBehavior(ag3.id, flocking);
		crowd.pushAgentBehavior(ag4.id, flocking);
		
		flocking->separationDistance = 2;
		flockParams->nbflockingTargets = 4;
		flockParams2->nbflockingTargets = 4;
		flockParams3->nbflockingTargets = 4;
		flockParams4->nbflockingTargets = 4;

		unsigned* toFlockWith1 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams->nbflockingTargets, DT_ALLOC_TEMP);
		unsigned* toFlockWith2 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams2->nbflockingTargets, DT_ALLOC_TEMP);
		unsigned* toFlockWith3 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams3->nbflockingTargets, DT_ALLOC_TEMP);
		unsigned* toFlockWith4 = (unsigned*) dtAlloc(sizeof(unsigned) * flockParams4->nbflockingTargets, DT_ALLOC_TEMP);

		toFlockWith1[0] = 1; toFlockWith1[1] = 2; toFlockWith1[2] = 3; toFlockWith1[3] = 4;
		toFlockWith2[0] = 0; toFlockWith2[1] = 2; toFlockWith2[2] = 3; toFlockWith2[3] = 4;
		toFlockWith3[0] = 0; toFlockWith3[1] = 1; toFlockWith3[2] = 3; toFlockWith3[3] = 4;
		toFlockWith4[0] = 0; toFlockWith4[1] = 1; toFlockWith4[2] = 2; toFlockWith4[3] = 4;

		flockParams->toFlockWith = toFlockWith1;
		flockParams2->toFlockWith = toFlockWith2;
		flockParams3->toFlockWith = toFlockWith3;
		flockParams4->toFlockWith = toFlockWith4;
				
		pf->init(*crowd.getCrowdQuery());

		crowd.pushAgentBehavior(ag2.id, crowd.getAgent(ag1.id)->behavior);
		crowd.pushAgentBehavior(ag3.id, crowd.getAgent(ag1.id)->behavior);
		crowd.pushAgentBehavior(ag4.id, crowd.getAgent(ag1.id)->behavior);
		crowd.pushAgentBehavior(ag5.id, pf);

		// Set the destination
		pfParams->submitTarget(destLeader, 0);

		// We perform several update on the flocking group.
		for (int i = 0; i < 100; ++i)
			crowd.update(0.1f);

		// Each agent should be heading toward the upper left corner
		for (int i = 0; i < 4; ++i)
		{
			float vel[3];
			dtVcopy(vel, crowd.getAgent(i)->velocity);

			CHECK(vel[0] < 0);
			CHECK(vel[2] < 0);
		}

		// We perform several update in order to reach the point where the agents aren't moving anymore
		for (int i = 0; i < 1000; ++i)
			crowd.update(0.1f, 0);

		// We check that the minimal space between each agent is respected
		CHECK(dtVdist2D(crowd.getAgent(ag1.id)->position, crowd.getAgent(ag2.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd.getAgent(ag1.id)->position, crowd.getAgent(ag3.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd.getAgent(ag1.id)->position, crowd.getAgent(ag4.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd.getAgent(ag4.id)->position, crowd.getAgent(ag2.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd.getAgent(ag4.id)->position, crowd.getAgent(ag3.id)->position) > flocking->separationDistance / 2.f);
		CHECK(dtVdist2D(crowd.getAgent(ag3.id)->position, crowd.getAgent(ag2.id)->position) > flocking->separationDistance / 2.f);

		dtFree(toFlockWith1);
		dtFree(toFlockWith2);
		dtFree(toFlockWith3);
		dtFree(toFlockWith4);

		dtFlockingBehavior::free(flocking);
		dtPathFollowing::free(pf);
	}

	SECTION("Separation Behavior", "With the separation behavior, an agent tries to flee its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {2, 0, 0};
		dtCrowdAgent ag1, ag2;

		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));

		dtCrowdAgent ag;
		crowd.fetchAgent(ag, ag1.id);
		crowd.pushAgent(ag);

		// Set the behavior
		dtSeparationBehavior* separation = dtSeparationBehavior::allocate(5);
		dtSeparationBehaviorParams* params = separation->getBehaviorParams(crowd.getAgent(ag1.id)->id);
		dtSeparationBehaviorParams* params2 = separation->getBehaviorParams(crowd.getAgent(ag2.id)->id);
		params->targetsID = &ag2.id;
		params->nbTargets = 1;
		params->weight = 1.f;
		params->distance = 0.f;
		params2->targetsID = &ag1.id;
		params2->nbTargets = 1;
		params2->weight = 1.f;
		params2->distance = 0.f;

		crowd.pushAgentBehavior(ag1.id, separation);
		crowd.pushAgentBehavior(ag2.id, separation);

		crowd.update(2.0, 0);

		float agt1NewVel[3];
		float nilVector[] = {0, 0, 0};

		dtVcopy(agt1NewVel, crowd.getAgent(ag1.id)->velocity);	

		// Since the distance is set to 0, the velocity of the agent should be nil
		CHECK(dtVequal(agt1NewVel, nilVector));

		// We now set the distance
		separation->getBehaviorParams(crowd.getAgent(ag1.id)->id)->distance = 5.f;
		separation->getBehaviorParams(crowd.getAgent(ag2.id)->id)->distance = 5.f;

		crowd.update(2.0, 0);

		dtVcopy(agt1NewVel, crowd.getAgent(ag1.id)->velocity);	

		// However the agent has not moved yet since no target was specified
		CHECK(!dtVequal(agt1NewVel, nilVector));
		
		crowd.update(2.0, 0);

		// The agent should now be moving to the left (away from the target)
		CHECK(dtVdist2D(crowd.getAgent(ag1.id)->position, crowd.getAgent(ag2.id)->position) > 2.1);

		// We perform several updates
		for (int i = 0; i < 100; ++i)
			crowd.update(1.0, 0);

		// The agent should have moved to the left far enough to respect the minimal distance
		CHECK(dtVdist2D(crowd.getAgent(ag1.id)->position, crowd.getAgent(ag2.id)->position) >= params->distance);

		dtSeparationBehavior::free(separation);
	}

	SECTION("Alignment Behavior", "With the alignment behavior, an agent will align its velocity to its target")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {0, 0, 1};
		float posAgt3[] = {0, 0, -1};
		float destAgt2[] = {15, 0, 1};
		float destAgt3[] = {15, 0, -1};
		dtCrowdAgent ag1, ag2, ag3;

		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));
		REQUIRE(crowd.addAgent(ag3, posAgt3));

		dtPathFollowing* pf1 = dtPathFollowing::allocate(2);
		dtPathFollowingParams* pfParams = pf1->getBehaviorParams(crowd.getAgent(ag2.id)->id);
		dtPathFollowingParams* pfParams2 = pf1->getBehaviorParams(crowd.getAgent(ag3.id)->id);

		pf1->init(*crowd.getCrowdQuery());
		
		crowd.pushAgentBehavior(ag2.id, pf1);
		crowd.pushAgentBehavior(ag3.id, pf1);

		// Set the destination
		pfParams->submitTarget(destAgt2);
		pfParams2->submitTarget(destAgt3);

		unsigned targets[] = {1, 2};
		dtAlignmentBehavior* align = dtAlignmentBehavior::allocate(1);
		dtAlignmentBehaviorParams* params = align->getBehaviorParams(crowd.getAgent(ag1.id)->id);
		params->targets = targets;
		params->nbTargets = 2;
		
		crowd.pushAgentBehavior(ag1.id, align);

		for (int i = 0; i < 10; ++i)
			crowd.update(0.1f, 0);

		// The velocity should not be nil
		CHECK(dtVlen(crowd.getAgent(ag1.id)->velocity) > 0);
		// But not as fast as the others agents'
		CHECK(dtVlen(crowd.getAgent(ag1.id)->velocity) < dtVlen(crowd.getAgent(ag3.id)->velocity));


		for (int i = 0; i < 50; ++i)
			crowd.update(0.1f, 0);

		// After a while, the agent should almost be at max speed
		CHECK(dtVlen(crowd.getAgent(ag1.id)->velocity) >= 1.8f);


		for (int i = 0; i < 100; ++i)
			crowd.update(0.1f, 0);

		// After even longer, the agent should have stopped
		CHECK(dtVlen(crowd.getAgent(ag1.id)->velocity) <= 0.2f);

		dtAlignmentBehavior::free(align);
		dtPathFollowing::free(pf1);
	}

	SECTION("Cohesion Behavior", "With the cohesion behavior, an agent will be attracted to its targets")
	{
		float posAgt1[] = {0, 0, 0};
		float posAgt2[] = {-5, 0, 1};
		float posAgt3[] = {-5, 0, -1};
		dtCrowdAgent ag1, ag2, ag3;

		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));
		REQUIRE(crowd.addAgent(ag2, posAgt2));
		REQUIRE(crowd.addAgent(ag3, posAgt3));

		unsigned targets[] = {1, 2};
		dtCohesionBehavior* cohesion = dtCohesionBehavior::allocate(5);
		dtCohesionBehaviorParams* params = cohesion->getBehaviorParams(crowd.getAgent(0)->id);
		params->targets = targets;
		params->nbTargets = 2;
		
		crowd.pushAgentBehavior(ag1.id, cohesion);

		for (int i = 0; i < 10; ++i)
			crowd.update(0.1f, 0);

		// The agent should be moving towards the targets
		CHECK(crowd.getAgent(ag1.id)->position[0] < 0);

		for (int i = 0; i < 1000; ++i)
			crowd.update(0.1f, 0);

		dtCohesionBehavior::free(cohesion);
	}

	SECTION("GoTo Behavior", "With the goto behavior, an agent must move towards the given position")
	{
		dtArriveBehavior* go = dtArriveBehavior::allocate(5);
		dtArriveBehaviorParams* params = go->getBehaviorParams(crowd.getAgent(0)->id);

		float posAgt1[] = {0, 0, 0};
		float destAgt1[] = {15, 0, 0};
		dtCrowdAgent ag1;

		// Adding the agents to the crowd
		REQUIRE(crowd.addAgent(ag1, posAgt1));

		params->target = destAgt1;

		REQUIRE(crowd.pushAgentBehavior(ag1.id, go));

		crowd.update(2.0, 0);

		// The agent should move to the right
		dtCrowdAgent updatedAg1;
		crowd.fetchAgent(updatedAg1, ag1.id);
		CHECK(ag1.position[0] < updatedAg1.position[0]);

		for (int i = 0; i < 1000; ++i)
			crowd.update(0.01f, 0);

		// The agent should have arrived to its target
		crowd.fetchAgent(updatedAg1, ag1.id);
		CHECK(updatedAg1.position[0] < 15.1f);
		CHECK(updatedAg1.position[0] > 14.9f);
		CHECK(updatedAg1.velocity[0] < 0.0001f);

		params->target[0] = 25.f;

		for (int i = 0; i < 1000; ++i)
			crowd.update(0.1f, 0);

		// The agent should have reached the boundaries of the navigation mesh
		crowd.fetchAgent(updatedAg1, ag1.id);
		CHECK(updatedAg1.position[0] < 20.1f);
		CHECK(updatedAg1.position[0] > 19.5f);

		dtArriveBehavior::free(go);
	}
}

TEST_CASE("DetourBehaviorsTests/HashTable", "Testing the hash table containing the pairs <agentID, AgentBehaviorData>")
{
	// Creation of the simulation
	const dtNavMesh& square = getSquareNavmesh();
	dtCrowd crowd;
	crowd.init(10, 0.5, &square);
	
	// Using a behavior with an empty HT
	dtAlignmentBehavior* align = dtAlignmentBehavior::allocate(0);
	
	// Adding an element to the HT
	dtAlignmentBehaviorParams* param = align->getBehaviorParams(crowd.getAgent(0)->id);
	CHECK(param == 0);
	
	// Adding another element
	param = align->getBehaviorParams(crowd.getAgent(1)->id);
	CHECK(param == 0);
	
	// Adding another element (id is greater or equal to the size of the HT)
	param = align->getBehaviorParams(crowd.getAgent(2)->id);
	CHECK(param == 0);
	
	// This time the HT is not empty
	dtSeekBehavior* seek = dtSeekBehavior::allocate(2);
	
	// Adding an element to the HT
	dtSeekBehaviorParams* params = seek->getBehaviorParams(crowd.getAgent(0)->id);
	CHECK(params != 0);
	
	// Getting an already existing elements
	params = seek->getBehaviorParams(crowd.getAgent(0)->id);
	CHECK(params != 0);
	
	// Adding another element
	params = seek->getBehaviorParams(crowd.getAgent(1)->id);
	CHECK(params != 0);
	
	// Adding another element (id is greater or equal to the size of the HT)
	params = seek->getBehaviorParams(crowd.getAgent(2)->id);
	CHECK(params != 0);
	
	// Adding another element (id greater than HT size)
	params = seek->getBehaviorParams(crowd.getAgent(9)->id);
	CHECK(params != 0);
	
	dtSeekBehavior::free(seek);
	dtAlignmentBehavior::free(align);
}
