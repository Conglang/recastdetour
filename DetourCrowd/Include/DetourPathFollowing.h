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

#ifndef DETOURPATHFOLLOWING_H
#define DETOURPATHFOLLOWING_H

#include "DetourPathCorridor.h"
#include "DetourPathQueue.h"
#include "DetourNavMeshQuery.h"
#include "DetourParametrizedBehavior.h"

class dtNavMesh;
struct dtCrowdAgent;
struct dtCrowdAgentDebugInfo;
struct dtCrowdAgentEnvironment;
class dtCrowdQuery;

template <typename T>
class dtParametrizedBehavior;




/// This class gives informations about the current path of the agent
struct dtCrowdAgentDebugInfo
{
	unsigned idx;						///< Index of  the agent
	float optStart[3], optEnd[3];		///< Begin and end of the path
	dtObstacleAvoidanceDebugData* vod;	///< Informations about how the agent avoids obstacles
};

/// Parameters for the path following behavior
/// The parameters will be automatically initialized when used for the first time by the PathFollowing behavior.
/// However the user still has the possibility to do it himself. For that he must call the init() method.
/// @ingroup behavior
struct dtPathFollowingParams
{
	dtPathFollowingParams();
    
    enum State
    {
        NO_TARGET = 0,      ///< No target submitted.
        INVALID_TARGET,     ///< The submitted target is invalid.
        FOLLOWING_PATH,     ///< The target is valid, the agent is following a path to it.
        TARGET_SUBMITTED,	///< The target has been succesfully submitted
        WAITING_FOR_QUEUE,	///< The target is being computed (TBC)
        WAITING_FOR_PATH,	///< The target is being computed (TBC)
    };
    
    unsigned char state;			///< State of the movement request.

	/// Initializes the parameters.
	/// The corridor will be initialized according to the given position
	/// @param[in]	maxPathResult	The maximum number of polygons that can be stored in a corridor.
	/// @param[in]	position	The position of the agent. Used to determine which polygon it is on.
	/// @param[in]	query		Used to access the navigation mesh query in order to get the polygon the agent is on.
	/// @return	True if the initialization was successful, false otherwise
	bool init(unsigned maxPathResults, const float* position, const dtCrowdQuery& query);
    
    /** @name Target */
    //@{
    /// Submits a new target.
    /// @param[in] pos The position of the target [(x, y, z)].
	/// @param[in] ref The navmesh polygon reference to which the target belong, used as a hint to locate the target, default value is no hint.
	/// @return True if the request was successfully submitted.
	void submitTarget(const float* pos, dtPolyRef polyRef = 0);
    
    /// Clear the current target.
	/// @return True if the request was successfully clear.
	void clearTarget();
    
    float targetPos[3];             ///< Target position
    dtPolyRef targetRef;            ///< Target polygon reference
    float targetReplanTime;         ///< Time since the agent's target was replanned.
    bool targetReplan;              ///< Flag indicating that the current path is being replanned.
    dtPathQueueRef targetPathqRef;  ///< Reference of the pathfind in the queue
    //@}
    
    /** @name Path */
    //@{
    dtPathCorridor corridor;    ///< The path corridor the agent is using.	
    unsigned ncorners;          ///< The number of corners.
    /// The maximum number of corners a crowd agent will look ahead in the path.
	/// This value is used for sizing the crowd agent corner buffers.
	/// Due to the behavior of the crowd manager, the actual number of useful
	/// corners will be one less than this number.
	static const unsigned MAX_NCORNERS = 4;
    
	float cornerVerts[MAX_NCORNERS*3]; ///< The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	unsigned char cornerFlags[MAX_NCORNERS]; ///< The local path corridor corner flags. (See: #dtStraightPathFlags)
	dtPolyRef cornerPolys[MAX_NCORNERS]; ///< The reference id of the polygon being entered at the corner.
	float topologyOptTime; ///< Time since the agent's path corridor was optimized.
    //@}
    
    /** @name Debug */
    //@{
    dtCrowdAgentDebugInfo* debugInfos;	///< Optionnal debug retrieval object.
    unsigned debugIndex;				///< The index of the agent for debug purpose.
    //@}
};

/// Defines a behavior for pathfollowing.
///
/// Using a navigation mesh, the pathfollowing behavior works on 
/// a list of agent in order to update their velocity so they can
/// reach their goal.
/// @ingroup behavior
class dtPathFollowing : public dtParametrizedBehavior<dtPathFollowingParams>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtPathFollowing* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtPathFollowing* ptr);

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	dtPathFollowing(unsigned nbMaxAgents);
	~dtPathFollowing();

	/// Initializes the behavior.
	///
	/// Must be called before using the behavior.
	/// @param[in]		crowdQuery	An object granting access to several elements of the crowd (animations, navigation mesh queries, etc.)
	/// @param[in]		maxPathRes	Maximum number of polygons for a path.
	///
	/// @return True if the initialization succeeded, false otherwise.
	bool init(dtCrowdQuery& crowdQuery, unsigned maxPathRes = 256);

	/// Cleans the class before destroying
	void purge();
    
    /// @name Parameters
    //@{
    /// During the first update after a request move target, the path starts to be computed
    /// for this given number of iterations (that way this path is not added to the queue if
    /// it is a short one).
    ///
    /// @remark Default value is 20.
    unsigned initialPathfindIterCount;
    
    /// Visibility-based path optimization distance.
    ///
    /// Inaccurate locomotion or dynamic obstacle avoidance can force the agent position
    /// significantly outside the original path corridor. Over time this can result in the
    /// formation of a non-optimal corridor.
    /// Non-optimal paths can also form near the corners of tiles.
    ///
    /// Setting this parameter to a positive distance will perform, during the update, an
    /// efficient local visibility search to try to optimize the corridor, finding
    /// shortcuts.
    ///
    /// @remark Default value is -1 (the optimization is disabled).
    float visibilityPathOptimizationRange;
    
    /// Local replanning time interval.
    ///
    /// Setting this parameter to a positive time will perform, at the specified frequency
    /// a local area path find to try to re-optimize the corridor.
    ///
    /// This method compliment the one controlled by
    /// dtPathFollowing::visibilityPathOptimizationRange
    ////
    /// @remark Default value is -1 (the replanning is disabled).
    float localPathReplanningInterval;
    
    /// Enable the turn anticipations.
    ///
    /// @remark Default value is false.
    bool anticipateTurns;
    //@}
    
    /// @see dtParametrizedBehavior::doUpdate
    virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, const dtPathFollowingParams& currentParams, dtPathFollowingParams& newParams, float dt);
private:
    dtPathFollowing(const dtPathFollowing&);
    dtPathFollowing& operator=(const dtPathFollowing&);

	/// Checks that the given agents still have valid paths.
	/// 
	/// @param[in]		ag				The agent to work on.
	/// @param[in]		dt				The time, in seconds, to update the simulation. [Limit: > 0]
	void checkPathValidity(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, const float dt, dtPathFollowingParams* agParams);

	/// Update async move request and path finder.
	void updateMoveRequest(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
		dtPathFollowingParams& newParams);

	/// Optimize path topology.
	/// 
	/// @param[in]		ag			The agent to work on.
	/// @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void updateTopologyOptimization(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& ag, const float dt, dtPathFollowingParams* agParams);

	/// Performs some checking and optimization on the agent.
	///
	/// @param[in]		ag			The agent to work on.
	/// @param[in]		dt			The time, in seconds, to update the simulation. [Limit: > 0]
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void prepare(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, float dt,dtPathFollowingParams& newParam);

	/// Computes the new velocity of the old agent according to its parameters, and puts the result into the new agent.
	///
	/// @param[in]		oldAgent	The agent whose velocity must be updated.
	/// @param[out]		newAgent	The agent storing the new parameters.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void getVelocity(const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, dtPathFollowingParams& agParams);

	/// Finds the next corner the agent should aim to
	/// 
	/// @param[in]		ag			The agent to work on.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void getNextCorner(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& ag, dtPathFollowingParams& agParams);

	/// Checks whether the agent is on an offmesh connection. If so, the path is adjusted.
	/// 
	/// @param[in]		agents			List of active agents.
	/// @param[out]		newAgent		The agent storing the new parameters.
	/// @param[in]		agParams		The parameters of the agent for this behavior
	void triggerOffMeshConnections(const dtCrowdQuery& crowdQuery, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, dtPathFollowingParams* agParams);

	/// Moves an agent into a list according to the last time since its target was replanned.
	///
	/// @param[in]		newag			Agent we want to move.
	/// @param[in]		agents			The list of agents.
	/// @param[in]		nagents			Size of the list.
	/// @param[in]		maxAgents		Maximal number of agents.
	int addToPathQueue(const dtCrowdAgent& newag, dtCrowdAgent** agents, const unsigned nagents, const unsigned maxAgents);

	/// Moves an agent into a list according to the last time since its path corridor was updated.
	///
	/// @param[in]		newag			Agent we want to move.
	/// @param[in]		agents			The list of agents.
	/// @param[in]		nagents			Size of the list.
	/// @param[in]		maxAgents		Maximal number of agents.
	int addToOptQueue(const dtCrowdAgent& newag, dtCrowdAgent** agents, const unsigned nagents, const unsigned maxAgents);

	/// Is the agent standing over an offmesh connection?
	///
	/// @param[in]		ag			The agent.
	/// @param[in]		radius		The radius of the agent.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	///
	/// @return Returns true if the agent is standing over an offmesh connection.
	bool overOffmeshConnection(const dtCrowdAgent& ag, const float radius, dtPathFollowingParams* agParams);

	/// Computes the distance from a given agent to its goal.
	///
	/// @param[in]		ag			The agent.
	/// @param[in]		range		The range of the agent (usually radius * 2).
	/// @param[in]		agParams	The parameters of the agent for this behavior
	///
	/// @return Returns  the distance from the agent to its goal.
	float getDistanceToGoal(const dtCrowdAgent& ag, const float range, dtPathFollowingParams* agParams);

	/// Computes the direction heading for the next corner target in a smooth way (as opposed to straight).
	///
	/// @param[in]		ag			The agent.
	/// @param[out]		dir			The direction resulting of the computation.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void calcSmoothSteerDirection(const dtCrowdAgent& ag, float* dir, dtPathFollowingParams* agParams);

	/// Computes the direction heading for the next corner target.
	///
	/// @param[in]		ag			The agent.
	/// @param[out]		dir			The direction resulting of the computation.
	/// @param[in]		agParams	The parameters of the agent for this behavior
	void calcStraightSteerDirection(const dtCrowdAgent& ag, float* dir, dtPathFollowingParams* agParams);

	dtPathQueue m_pathQueue;				///< A Queue of destination in order to reach the target.

	dtPolyRef* m_pathResult;				///< The path results
	unsigned m_maxAgents;					///< Maximal number of agents.
	unsigned m_maxPathRes;					///< Maximal number of path results

	const unsigned m_maxCommonNodes;		///< Maximal number of common nodes.
	const unsigned m_maxPathQueueNodes;		///< Maximal number of nodes in the path queue.
	const unsigned m_maxIterPerUpdate;		///< Maximal number of iterations per update.
};

#endif
