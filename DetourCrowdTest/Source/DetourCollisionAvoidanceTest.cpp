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

#include "DetourCollisionAvoidance.h"

#include <cstring>

SCENARIO("DetourCollisionAvoidanceTest/DefaultParams", "[detourCollisionAvoidance]")
{
	GIVEN("A default constructed collision avoidance parameters")
	{
		dtCollisionAvoidanceParams p;
		
		THEN("The parameters have the expected default values")
		{
			CHECK(p.debug == 0);
		}
	}
	
	GIVEN("A default constructed collision avoidance")
	{
		dtCollisionAvoidance b(34);
		
		THEN("The parameters have the expected default values")
		{
			CHECK(b.maximumSegmentObstaclesCount == 8);
			CHECK(b.maximumCircleObstaclesCount == 6);
			CHECK(b.weightDesiredVelocity == 2.f);
			CHECK(b.weightCurrentVelocity == 0.75f);
			CHECK(b.weightCurrentAvoidanceSide == 0.75f);
			CHECK(b.weightTimeToCollision == 2.5f);
			CHECK(b.horizonTime == 2.5f);
			CHECK(b.sampleOriginScale == 0.4f);
			CHECK(b.sampleSectorsCount == 7);
			CHECK(b.sampleRingsCount == 2);
			CHECK(b.sampleLevelsCount == 5);
		}
	}
}
