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
#include <DetourFlockingBehavior.h>

#include <map>
#include <vector>
#include <string>

#include <map>

class rcContext;
class InputGeom;
class JSONValue;

struct AgentConfiguration
{
	int index;
	float position[3];
	float destination[3];
	dtPolyRef destinationPoly;

	float radius;
	float height;
	float maxAcceleration;
	float maxSpeed;

	dtBehavior* steeringBehavior;

	float collisionQueryRange;
	unsigned char updateFlags;
};

class CrowdSample
{
public:
	CrowdSample();
	~CrowdSample();
	
	bool loadFromBuffer(const char* data);
	bool loadFromFile(const char* fileName);

	char* getSceneFile();
	void parseCrowd(dtCrowd* crowd);
	void parseAgentsInfo();

	bool initialize(dtMesh& mesh, dtNavMesh& navMesh, dtCrowd& crowd);

	bool initializeScene(dtMesh& mesh);
	bool initializeCrowd(dtCrowd& crowd);
	bool initializeNavmesh(const dtMesh& mesh, dtNavMesh& navMesh);
	
	AgentConfiguration m_agentCfgs[maxAgentCount];
	int m_agentCount;
	rcContext* m_context;
	char m_sceneFileName[maxPathLen];
	float m_maxRadius;
	
private:
	void parseBehavior(JSONValue* behavior, std::size_t iAgent, dtCrowd* crowd, bool pipeline);
	void parsePipeline(JSONValue* pipelineChild, std::size_t iAgent, dtCrowd* crowd);

	struct Flocking
	{
		float distance;
		float desiredSeparation;
		float separationWeight;
		float cohesionWeight;
		float alignmentWeight;
	};

	void computeMaximumRadius();
	
	JSONValue* m_root;
	std::vector<Flocking> m_flockingsGroups;
	std::vector<dtFlockingBehavior*> m_flockingBehaviors;
	std::map<int, std::vector<int> > m_agentsFlockingNeighbors;
	std::map<int, int> m_seekTargets;
	std::vector<int> m_separationTargets;
	std::vector<int> m_alignmentTargets;
	std::vector<int> m_cohesionTargets;
	std::map<int, std::vector<dtBehavior*> > m_pipeline;
};

#endif
