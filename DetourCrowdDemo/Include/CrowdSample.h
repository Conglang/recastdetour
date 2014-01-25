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

#ifndef CROWDSAMPLE_H
#define CROWDSAMPLE_H


#include "StaticConfiguration.h"

#include <DetourNavmeshCreator.h>

#include <DetourCrowd.h>

#include <map>
#include <vector>
#include <string>

class dtCollisionAvoidance;
class dtPipelineBehavior;
class dtPathFollowing;

class rcContext;

class JSONValue;

class CrowdSample
{
public:
	CrowdSample();
	~CrowdSample();

	bool loadFromFile(const char* fileName, rcContext& context);

	dtMesh m_mesh;
	dtNavMesh m_navMesh;
	dtCrowd m_crowd;
	float m_maxRadius;
	unsigned m_agentsCount;
	
private:
	struct BehaviorCfg
	{
		dtBehavior* activeBehaviors[2];
		unsigned activeBehaviorsCount;
		dtPipelineBehavior* pipeline;
		dtCollisionAvoidance* collisionAvoidance;
		dtPathFollowing* pathFollowing;
	};

	bool loadJSONFile(const char* fileName, JSONValue** root, rcContext& context);
	bool loadScene(JSONValue& root, rcContext& context);
	bool retrieveAgentsInfo(JSONValue& root, rcContext& context);
	bool computeNavmesh(rcContext& context);
	bool parseBehaviors(JSONValue& root, rcContext& context);
	bool parseCollisionAvoidance(JSONValue& behavior, dtCollisionAvoidance** collisionAvoidance, rcContext& context);
	bool parsePathFollowing(JSONValue& behavior, dtPathFollowing** pathFollowing, rcContext& context);
	bool createAgents(JSONValue& root, rcContext& context);

	std::map<std::string, BehaviorCfg> m_behaviors;
};

#endif
