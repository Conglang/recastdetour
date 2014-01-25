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

#include "CrowdSample.h"

#include "JSON.h"

#include <DetourCollisionAvoidance.h>
#include <DetourPathFollowing.h>
#include <DetourPipelineBehavior.h>

#include <DetourAssert.h>

#include <Recast.h>

#include <cstdio>
#include <cstring>

CrowdSample::CrowdSample()
: m_mesh()
, m_navMesh()
, m_crowd()
, m_maxRadius(-1.f)
, m_agentsCount(0)
, m_behaviors()
{
	// NOTHING
}

CrowdSample::~CrowdSample()
{
	// NOTHING
}

bool CrowdSample::loadFromFile(const char* fileName, rcContext& context)
{
	JSONValue* root;
	bool success = loadJSONFile(fileName, &root, context);
	if (success) dtAssert(root);
	success = success && loadScene(*root, context);
	success = success && retrieveAgentsInfo(*root, context);
	success = success && computeNavmesh(context);
	if (m_agentsCount > 0)
	{
		success = success && m_crowd.init(m_agentsCount, m_maxRadius, &m_navMesh);
		success = success && parseBehaviors(*root, context);
		success = success && createAgents(*root, context);
	}
	return success;
}

bool CrowdSample::loadJSONFile(const char* fileName, JSONValue** root, rcContext& context)
{
	char* buf = 0;
	FILE* fp = fopen(fileName, "rb");
	if (!fp)
	{
		context.log(RC_LOG_ERROR, "Unable to load the crowd sample from '%s', error while opening the file.",fileName);
		return false;
	}
	fseek(fp, 0, SEEK_END);
	int bufSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	buf = new char[bufSize+1];
	if (!buf)
	{
		fclose(fp);
		context.log(RC_LOG_ERROR, "Unable to load the crowd sample from '%s', error while allocating the buffer.",fileName);
		return false;
	}
	fread(buf, bufSize, 1, fp);
	fclose(fp);
	buf[bufSize]=0;

	*root = JSON::Parse(buf);

	if (!(*root))
	{
		context.log(RC_LOG_ERROR, "Unable to load the crowd sample from '%s', error while parsing the JSON.",fileName);
		return false;
	}

	return true;
}

bool CrowdSample::loadScene(JSONValue& root, rcContext& context)
{
	JSONValue* scene = root.Child(L"scene");
	if (scene && scene->IsObject())
	{
		JSONValue* file = scene->Child(L"file");
		if (file && file->IsString())
		{
			char sceneFileName[maxStringLen + 1];
			wcstombs(sceneFileName,file->AsString().c_str(), maxStringLen + 1);
			m_mesh.clear();
			if (!loadObjFile(sceneFileName, m_mesh))
			{
				context.log(RC_LOG_ERROR, "Unable to load the scene, error loading '%s'.",sceneFileName);
				return false;
			}
		}
	}
	return true;
}

bool CrowdSample::retrieveAgentsInfo(JSONValue& root, rcContext& context)
{
	m_maxRadius = 0.2f;
	m_agentsCount = 0;
	JSONValue* agents = root.Child(L"agents");
	if (agents && agents->IsArray())
	{
		m_agentsCount = rcMin<unsigned>(agents->CountChildren(),maxAgentCount);
		for (std::size_t iAgent(0) ; iAgent < m_agentsCount ; ++iAgent)
		{
			JSONValue* agent = agents->Child(iAgent);
			if (agent && agent->IsObject())
			{
				JSONValue* radius = agent->Child(L"radius");
				if (radius && radius->IsNumber())
					m_maxRadius = rcMax(m_maxRadius, (float) radius->AsNumber());
			}
		}
	}
	return true;
}

bool CrowdSample::computeNavmesh(rcContext& context)
{
	dtNavmeshInputGeometry inputGeometry;
	inputGeometry.mesh = m_mesh;
	if (!inputGeometry.initialize())
	{
		context.log(RC_LOG_ERROR, "Unable to compute the navmesh, error while initialization the input geometry.");
		return false;
	}

	dtTiledNavmeshCfg configuration;

	dtAssert(m_maxRadius > 0.f);
	configuration.voxels.size = m_maxRadius * 0.25f;
	configuration.navigation.minimumObstacleClearance = m_maxRadius;

	configuration.computeTileCount(inputGeometry.bmin, inputGeometry.bmax, 512);

	if (!dtCreateTiledNavmesh(inputGeometry,
							  configuration,
							  m_navMesh,
							  &context))
	{
		context.log(RC_LOG_ERROR, "Unable to compute the navmesh, error during the computation.");
		return false;
	}
	
	return true;
}

bool CrowdSample::parseBehaviors(JSONValue& root, rcContext& context)
{
	JSONValue* behaviors = root.Child(L"behaviors");
	if (behaviors && behaviors->IsObject())
	{
		const JSONObject& behaviorsObject = behaviors->AsObject();
		for (JSONObject::const_iterator it(behaviorsObject.begin()), end(behaviorsObject.end()) ; it != end ; ++it)
		{
			char behaviorName[maxStringLen + 1];
			wcstombs(behaviorName,it->first.c_str(), maxStringLen + 1);
			JSONValue* behaviorPipeline = it->second;
			BehaviorCfg behaviorCfg;
			memset(&behaviorCfg, 0, sizeof(behaviorCfg));
			if (behaviorPipeline && behaviorPipeline->IsArray())
			{
				for (std::size_t i(0), size(behaviorPipeline->CountChildren()) ; i < size ; ++i)
				{
					JSONValue* behavior = behaviorPipeline->Child(i);
					if (behavior && behavior->IsObject())
					{
						JSONValue* type = behavior->Child(L"type");
						if (type && type->IsString() && type->AsString() == L"collisionAvoidance")
						{
							if (behaviorCfg.collisionAvoidance)
							{
								context.log(RC_LOG_ERROR, "Unable to parse the behavior '%s', can't have duplicate collision avoidance behavior.", behaviorName);
								return false;
							}
							if (!parseCollisionAvoidance(*behavior, &behaviorCfg.collisionAvoidance, context))
							{
								context.log(RC_LOG_ERROR, "Unable to parse the behavior '%s', error while parsing the collision avoidance behavior.", behaviorName);
								return false;
							}

							behaviorCfg.activeBehaviors[behaviorCfg.activeBehaviorsCount] = behaviorCfg.collisionAvoidance;
							behaviorCfg.activeBehaviorsCount++;
							dtAssert(behaviorCfg.activeBehaviorsCount <= 2);
						}
						else if (type && type->IsString() && type->AsString() == L"pathFollowing")
						{
							if (behaviorCfg.pathFollowing)
							{
								context.log(RC_LOG_ERROR, "Unable to parse the behavior '%s', can't have duplicate path following behavior.", behaviorName);
								return false;
							}
							if (!parsePathFollowing(*behavior, &behaviorCfg.pathFollowing, context))
							{
								context.log(RC_LOG_ERROR, "Unable to parse the behavior '%s', error while parsing the path following behavior.", behaviorName);
								return false;
							}

							behaviorCfg.activeBehaviors[behaviorCfg.activeBehaviorsCount] = behaviorCfg.pathFollowing;
							behaviorCfg.activeBehaviorsCount++;
							dtAssert(behaviorCfg.activeBehaviorsCount <= 2);
						}
						else if (type && type->IsString())
						{
							context.log(RC_LOG_ERROR, "Unable to parse the behavior '%s', '%s' is an unknown behavior type.", behaviorName, type->AsString().c_str());
							return false;
						}
					}
				}

			}
			behaviorCfg.pipeline = dtPipelineBehavior::allocate();
			behaviorCfg.pipeline->setBehaviors(behaviorCfg.activeBehaviors, behaviorCfg.activeBehaviorsCount);
			m_behaviors.insert(std::make_pair(behaviorName, behaviorCfg));
		}
	}
	return true;
}

bool CrowdSample::parseCollisionAvoidance(JSONValue& behavior, dtCollisionAvoidance** collisionAvoidance, rcContext& /*context*/)
{
	*collisionAvoidance = dtCollisionAvoidance::allocate(m_agentsCount);
	(*collisionAvoidance)->init();

	JSONValue* weightDesiredVelocity = behavior.Child(L"weightDesiredVelocity");
	if (weightDesiredVelocity && weightDesiredVelocity->IsNumber())
		(*collisionAvoidance)->weightDesiredVelocity = (float)weightDesiredVelocity->AsNumber();

	JSONValue* weightCurrentVelocity = behavior.Child(L"weightCurrentVelocity");
	if (weightCurrentVelocity && weightCurrentVelocity->IsNumber())
		(*collisionAvoidance)->weightCurrentVelocity = (float)weightCurrentVelocity->AsNumber();

	JSONValue* weightCurrentAvoidanceSide = behavior.Child(L"weightCurrentAvoidanceSide");
	if (weightCurrentAvoidanceSide && weightCurrentAvoidanceSide->IsNumber())
		(*collisionAvoidance)->weightCurrentAvoidanceSide = (float)weightCurrentAvoidanceSide->AsNumber();

	JSONValue* weightTimeToCollision = behavior.Child(L"weightTimeToCollision");
	if (weightTimeToCollision && weightTimeToCollision->IsNumber())
		(*collisionAvoidance)->weightTimeToCollision = (float)weightTimeToCollision->AsNumber();

	JSONValue* sampleOriginScale = behavior.Child(L"sampleOriginScale");
	if (sampleOriginScale && sampleOriginScale->IsNumber())
		(*collisionAvoidance)->sampleOriginScale = (float)sampleOriginScale->AsNumber();

	JSONValue* sampleSectorsCount = behavior.Child(L"sampleSectorsCount");
	if (sampleSectorsCount && sampleSectorsCount->IsNumber())
		(*collisionAvoidance)->sampleSectorsCount = (unsigned char)sampleSectorsCount->AsNumber();

	JSONValue* sampleRingsCount = behavior.Child(L"sampleRingsCount");
	if (sampleRingsCount && sampleRingsCount->IsNumber())
		(*collisionAvoidance)->sampleRingsCount = (unsigned char)sampleRingsCount->AsNumber();

	JSONValue* sampleLevelsCount = behavior.Child(L"sampleLevelsCount");
	if (sampleLevelsCount && sampleLevelsCount->IsNumber())
		(*collisionAvoidance)->sampleLevelsCount = (unsigned char)sampleLevelsCount->AsNumber();

	JSONValue* horizonTime = behavior.Child(L"horizonTime");
	if (horizonTime && horizonTime->IsNumber())
		(*collisionAvoidance)->horizonTime = (float)horizonTime->AsNumber();

	return true;
}

bool CrowdSample::parsePathFollowing(JSONValue& behavior, dtPathFollowing** pathFollowing, rcContext& /*context*/)
{
	*pathFollowing = dtPathFollowing::allocate(1);
	(*pathFollowing)->init(*m_crowd.getCrowdQuery());

	JSONValue* visibilityPathOptimizationRange = behavior.Child(L"visibilityPathOptimizationRange");
	if (visibilityPathOptimizationRange && visibilityPathOptimizationRange->IsNumber())
		(*pathFollowing)->visibilityPathOptimizationRange = (float)visibilityPathOptimizationRange->AsNumber();

	JSONValue* initialPathfindIterCount = behavior.Child(L"initialPathfindIterCount");
	if (initialPathfindIterCount && initialPathfindIterCount->IsNumber())
		(*pathFollowing)->initialPathfindIterCount = (unsigned)initialPathfindIterCount->AsNumber();

	JSONValue* localPathReplanningInterval = behavior.Child(L"localPathReplanningInterval");
	if (localPathReplanningInterval && localPathReplanningInterval->IsBool())
		(*pathFollowing)->localPathReplanningInterval = localPathReplanningInterval->AsBool();

	JSONValue* anticipateTurns = behavior.Child(L"anticipateTurns");
	if (anticipateTurns && anticipateTurns->IsBool())
		(*pathFollowing)->anticipateTurns = anticipateTurns->AsBool();

	return true;
}

bool CrowdSample::createAgents(JSONValue& root, rcContext& context)
{
	JSONValue* agents = root.Child(L"agents");
	if (agents && agents->IsArray())
	{
		m_agentsCount = rcMin<unsigned>(agents->CountChildren(),maxAgentCount);
		for (std::size_t iAgent(0) ; iAgent < m_agentsCount ; ++iAgent)
		{
			JSONValue* agent = agents->Child(iAgent);
			if (agent && agent->IsObject())
			{
				dtCrowdAgent ag;
				JSONValue* position = agent->Child(L"position");
				if (position && position->IsArray())
				{
					ag.position[0] = (float)position->Child((size_t)0)->AsNumber();
					ag.position[1] = (float)position->Child(1)->AsNumber();
					ag.position[2] = (float)position->Child(2)->AsNumber();
					if (!m_crowd.addAgent(ag, ag.position))
					{
						context.log(RC_LOG_ERROR, "Unable to create agent #%d, addition to crowd failed.", iAgent);
						return false;
					}
					if (ag.state == DT_CROWDAGENT_STATE_INVALID)
					{
						context.log(RC_LOG_ERROR, "Unable to create agent #%d, its initial position (%.2f, %.2f, %.2f) doesn't belong to the navmesh.", ag.id, (float)position->Child((size_t)0)->AsNumber(), (float)position->Child(1)->AsNumber(), (float)position->Child(2)->AsNumber());
						return false;
					}
				}

				JSONValue* maxSpeed = agent->Child(L"maxSpeed");
				if (maxSpeed && maxSpeed->IsNumber())
					ag.maxSpeed= (float)maxSpeed->AsNumber();

				JSONValue* maxAcceleration = agent->Child(L"maxAcceleration");
				if (maxAcceleration && maxAcceleration->IsNumber())
					ag.maxAcceleration = (float)maxAcceleration->AsNumber();

				JSONValue* radius = agent->Child(L"radius");
				if (radius && radius->IsNumber())
					ag.radius = (float)radius->AsNumber();

				JSONValue* height = agent->Child(L"height");
				if (height && height->IsNumber())
					ag.height = (float)height->AsNumber();

				JSONValue* detectionRange = agent->Child(L"detectionRange");
				if (detectionRange && detectionRange->IsNumber())
					ag.detectionRange = (float)detectionRange->AsNumber();

				JSONValue* behavior = agent->Child(L"behavior");
				if (behavior && behavior->IsString())
				{
					char behaviorName[maxStringLen + 1];
					wcstombs(behaviorName,behavior->AsString().c_str(), maxStringLen + 1);
					std::map<std::string, BehaviorCfg>::iterator behaviorIt = m_behaviors.find(behaviorName);
					if (behaviorIt == m_behaviors.end())
					{
						context.log(RC_LOG_ERROR, "Unable to create agent #%d, set behavior (%s) is unknown.", iAgent, behaviorName);
						return false;
					}
					dtAssert(behaviorIt->second.pipeline);
					ag.behavior = behaviorIt->second.pipeline;

					JSONValue* behaviorParams = agent->Child(L"behaviorParams");
					if (behaviorParams && behaviorParams->IsObject())
					{
						JSONValue* pathFollowingParams = behaviorParams->Child(L"pathFollowing");
						if (pathFollowingParams && pathFollowingParams->IsObject())
						{
							if (!behaviorIt->second.pathFollowing)
							{
								context.log(RC_LOG_ERROR, "Unable to create agent #%d, path following parameters provided while its behavior doesn't include path following.", iAgent, behaviorName);
								return false;
							}
							JSONValue* destination = pathFollowingParams->Child(L"destination");
							if (destination && destination->IsArray())
							{
								float dest[3];
								dest[0] = (float)destination->Child((size_t)0)->AsNumber();
								dest[1] = (float)destination->Child(1)->AsNumber();
								dest[2] = (float)destination->Child(2)->AsNumber();
								behaviorIt->second.pathFollowing->getBehaviorParams(ag.id)->submitTarget(dest);
							}
						}
					}
				}

				if (!m_crowd.pushAgent(ag))
				{
					context.log(RC_LOG_ERROR, "Unable to create agent #%d, update of its parameters failed.", iAgent);
					return false;
				}
			}
		}
	}
	return true;
}
