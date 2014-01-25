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

#include <vector>

TEST_CASE("DetourPipelineTest/Pipeline", "Tests about the pipeline behavior")
{
	const dtNavMesh& square = getSquareNavmesh();
	dtCrowd crowd;
	crowd.init(20, 0.5, &square);

	dtPipelineBehavior* pipeline = dtPipelineBehavior::allocate();

	float posAgt1[] = {0, 0.2f, 0};
	float destAgt1[] = {15, 0, 0};

	dtCrowdAgent ag;

	// Adding the agents to the crowd
	REQUIRE(crowd.addAgent(ag, posAgt1));
	CHECK(crowd.pushAgentBehavior(ag.id, pipeline));

	crowd.update(0.5, 0);

	dtCrowdAgent updatedAgt;
	crowd.fetchAgent(updatedAgt, ag.id);

	// Since no behavior is affected to the pipeline, the agent must not have moved
	CHECK(dtVequal(updatedAgt.position, ag.position));

	dtPathFollowing* pf = dtPathFollowing::allocate(5);

	// Set the destination
	dtPolyRef dest;
	float nearest[3];
	crowd.getCrowdQuery()->getNavMeshQuery()->findNearestPoly(destAgt1, crowd.getCrowdQuery()->getQueryExtents(), crowd.getCrowdQuery()->getQueryFilter(), &dest, nearest);

	REQUIRE(dest != 0);		
	REQUIRE(pf->init(*crowd.getCrowdQuery()));
	pf->getBehaviorParams(ag.id)->submitTarget(destAgt1, dest);

	SECTION("Adding and Removing behaviors to the pipeline", "Trying to add and remove behaviors into the pipeline, should not crash")
	{
		CHECK(pipeline->setBehaviors(0, 1));

		// Still no behavior was given (null pointer), so nothing should have moved.
		crowd.update(0.5, 0);

		// Since no behavior is affected to the pipeline, the agent must not have moved
		crowd.fetchAgent(updatedAgt, ag.id);
		CHECK(dtVequal(updatedAgt.position, ag.position));

		dtBehavior* behavior = pf;

		// This time we affect the right behavior but with a size of 0
		CHECK(pipeline->setBehaviors(&behavior, 0));

		crowd.update(0.5, 0);

		// The agent should not have moved
		crowd.fetchAgent(updatedAgt, ag.id);
		CHECK(dtVequal(updatedAgt.position, ag.position));

		// This time we affect the behavior with the right size
		CHECK(pipeline->setBehaviors(&behavior, 1));

		crowd.update(0.5, 0);

		// A behavior has been affected to the pipeline, the agent should have moved
		crowd.fetchAgent(updatedAgt, ag.id);
		CHECK(!dtVequal(updatedAgt.position, ag.position));

	}

	SECTION("Using a container", "Using a container to store behaviors, set them to the pipeline and then destroy the container. The behaviors should still be in the pipeline")
	{
		dtCollisionAvoidance* ca = dtCollisionAvoidance::allocate(crowd.getAgentCount());
		ca->init();

		dtCollisionAvoidanceParams* params = ca->getBehaviorParams(ag.id);

		if (params)
		{
			params->debug = 0;
		}

		std::vector<dtBehavior*> behaviors;
		behaviors.push_back(pf);
		behaviors.push_back(ca);

		CHECK(pipeline->setBehaviors(behaviors.data(), behaviors.size()));

		behaviors.clear();

		dtVcopy(posAgt1, crowd.getAgent(ag.id)->position);	
		crowd.update(0.5);

		// The agent should have moved
		crowd.fetchAgent(updatedAgt, ag.id);
		CHECK(!dtVequal(updatedAgt.position, ag.position));

		dtCollisionAvoidance::free(ca);
	}

	dtPathFollowing::free(pf);
	dtPipelineBehavior::free(pipeline);
}
