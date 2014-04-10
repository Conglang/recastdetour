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

#ifndef DETOURCOLLISIONAVOIDANCE_H
#define DETOURCOLLISIONAVOIDANCE_H

#include "DetourParametrizedBehavior.h"


struct dtCrowdAgent;
struct dtCrowdAgentEnvironment;

/// Obstacle represented as a circle
struct dtObstacleCircle
{
	float position[3];			
	float velocity[3];			
	float desiredVelocity[3];	
	float radius;
	float direction[3];         ///< Normalized vector from the agent to the obstacle
    float directionNormal[3];   ///< Vector normal to 'direction'
};

/// Obstacle represented as a segment
struct dtObstacleSegment
{
	float p[3], q[3];	///< End points of the obstacle segment
	bool touch;			///< Is the obstacles touched?
};


/// Debug data for the collision behavior
class dtObstacleAvoidanceDebugData
{
public:
	dtObstacleAvoidanceDebugData();
	~dtObstacleAvoidanceDebugData();

	/// Initializes the data
	///
	/// @param[in]	maxSamples	Max number of samples we want for debuging
	bool init(const int maxSamples);

	/// Resets the number of samples
	void reset();

	/// Add a sample to the data
	///
	/// @param[in]	vel		Velocity of the sample
	/// @param[in]	ssize	Size of the sample
	/// @param[in]	pen		Penalty of the sample
	/// @param[in]	vpen	Desired velocity penalty of the sample
	/// @param[in]	vcpen	Current velocity of the sample
	/// @param[in]	spen	Preferred side penalty of the sample
	/// @param[in]	tpen	Collision time penalty of the sample
	void addSample(const float* vel, const float ssize, const float pen,
		const float vpen, const float vcpen, const float spen, const float tpen);

	/// Normalizes the samples
	void normalizeSamples();

	/// @name Getters for the sample data
	/// @{
	inline int getSampleCount() const { return m_nsamples; }
	inline const float* getSampleVelocity(const int i) const { return &m_vel[i*3]; }
	inline float getSampleSize(const int i) const { return m_ssize[i]; }
	inline float getSamplePenalty(const int i) const { return m_pen[i]; }
	inline float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	inline float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	inline float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	inline float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }
	/// @}

private:
	int m_nsamples;		///< Number of sample
	int m_maxSamples;	///< Maximum number of sample
	float* m_vel;		///< Velocity
	float* m_ssize;		///< Size of the sample
	float* m_pen;		///< Penalty of the sample
	float* m_vpen;		///< Desired velocity penalty
	float* m_vcpen;		///< Current velocity penalty
	float* m_spen;		///< Preferred side penalty
	float* m_tpen;		///< Collision time penalty
};

/// Custom allocator for ObstacleAvoidanceData
dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
/// Custom deallocator for ObstacleAvoidanceData
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);


static const unsigned DT_MAX_PATTERN_DIVS = 32;	///< Max number of adaptive divs.
static const unsigned DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

/// Parameters for the collision avoidance behavior
/// @ingroup behavior
struct dtCollisionAvoidanceParams
{
    dtCollisionAvoidanceParams();
	dtObstacleAvoidanceDebugData* debug;	///< A debug object to load with debug information. [Opt]
};


/// Defines a behavior for collision avoidance.
///
/// The agents use this behavior in order to change their velocity 
/// to avoid obstacles.
/// @ingroup behavior
class dtCollisionAvoidance : public dtParametrizedBehavior<dtCollisionAvoidanceParams>
{
public:
	dtCollisionAvoidance(unsigned nbMaxAgents);
	~dtCollisionAvoidance();

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtCollisionAvoidance* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtCollisionAvoidance* ptr);

	/// Initializes the behavior.
	///
	/// @return True if the initialization succeeded.
	bool init();

	/// Cleans the behavior.
	void purge();
	
    /// @name Number of considered obstacles parameters
    ///
    /// @remark Changes to these require a call to @ref resizeObstaclesContainer to be
    /// taken into account.
    //@{
    /// The maximum number of circle obstacles (i.e. only agents at the moment) that
    /// can be taken into account by the avoidance algorithm
    ///
    /// @remark Default value is 6.
    unsigned maximumCircleObstaclesCount;
    
    /// The maximum number of segment obstacles (i.e. walls) that can be taken
    /// into account by the avoidance algorithm
    ///
    /// @remark Default value is 8.
    unsigned maximumSegmentObstaclesCount;
    
    /// Resize the container for obstacles according to the set sizes.
    bool resizeObstaclesContainer();
    //@}
    
    /// @name Velocity samples generation parameters
    ///
    /// The collision avoidance algorithm generate an adaptive pattern
    /// of velocity candidate then choses the "cheapest" candidate.
    //@{
    /// Samples origin scale.
    ///
    /// The center of the first-level pattern is chosen to be the desired velocity
    /// scaled by this value. The larger it is, the more the agent will accelerate
    /// to avoid collisions.
    ///
    /// @remark Default value is 0.4.
    float sampleOriginScale;
    
    /// The number of samples levels.
    ///
    /// @remark Default value is 5.
	unsigned char sampleLevelsCount;
    /// The number of samples sectors per level.
    ///
    /// The total amount of samples per level is
    /// @ref sampleSectorsCount * @ref sampleRingsCount
    ///
    /// @remark Default value is 7.
    unsigned char sampleSectorsCount;
    /// The number of samples rings per level.
    ///
    /// The total amount of samples per level is
    /// @ref sampleSectorsCount * @ref sampleRingsCount
    ///
    /// @remark Default value is 2.
	unsigned char sampleRingsCount;
    //@}
    
    /// @name Velocity sample weight factors
    ///
    /// The cost of any given candidate velocity is the **sum** of 4 costs that each
    // depends on:
    ///     - the distance to the desired velocity;
    ///     - the distance to the current velocity;
    ///     - the current avoidance side;
    ///     - the time to the first collision.
    ///
    /// Each of this cost is first normalized between 0 and 1 and then weighted by
    /// its matching factor.
    //@{
    
    /// The weight of the desired velocity
    ///
    /// Increase to prefer candidates close to the current desired velocity.
    ///
    /// @remark Default value is 2.
    float weightDesiredVelocity;
    
    /// The weight of the current velocity
    ///
    /// Increase to prefer candidates close to the current velocity.
    ///
    /// @remark Default value is 0.75.
	float weightCurrentVelocity;
    
    /// The weight of the current avoidance side.
    ///
    /// Increase to prefer candidates that will keep the currently chosen avoidance side.
    /// In practice, if the agent is already passing to the left of its neighbors this
    /// weight will favor candidates that keeps this side of avoidance.
    ///
    /// @remark Default value is 0.75.
	float weightCurrentAvoidanceSide;
    
    /// The weight of the time to collision.
    ///
    /// Increase to prefer "safer" candidates.
    ///
    /// @remark Default value is 2.5.
	float weightTimeToCollision;
    //@}
    
    /// @name Other parameters
    //@{
    /// The time under which incoming collision are taken into account.
    ///
    /// @remark Default value is 2.5s.
    float horizonTime;
    //@}
    
    /// @see dtParametrizedBehavior::doUpdate 
    virtual void doUpdate(const dtCrowdQuery& query, const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, const dtCollisionAvoidanceParams& currentParams, dtCollisionAvoidanceParams& newParams, float dt);
    
private:
    dtCollisionAvoidance(const dtCollisionAvoidance&);
    dtCollisionAvoidance& operator=(const dtCollisionAvoidance&);

	/// Registers all the neighbors of the given agent as obstacles.
	///
	/// @param[in]		ag		The index we want to change.
	/// @param[in]		query	Allows the user to query data from the crowd.
	void addObtacles(const dtCrowdAgent& ag, const dtCrowdQuery& query);

	/// Updates the velocity of the old agent according to its parameters, and puts the result into the new agent.
	///
	/// @param[in]	oldAgent		The agent whose velocity must be updated.
	/// @param[out]	newAgent		The agent storing the new parameters.
	/// @param[in]	currentParams	The parameters of the agent.
	/// @param[out]	newParams		The new parameters of the agent.
	void updateVelocity(const dtCrowdAgent& oldAgent, dtCrowdAgent& newAgent, 
		const dtCollisionAvoidanceParams& currentParams, dtCollisionAvoidanceParams& newParams);

	
	
	/// Computes the desired velocity of an agent.
	///
	/// @param[in]		pos			The position of the agent.
	/// @param[in]		rad			The radius of the agent.
	/// @param[in]		vmax		The maximal speed of the agent.
	/// @param[in]		vel			The current velocity of the agent.
	/// @param[in]		dvel		The desired velocity of the agent.
	/// @param[in]		nvel		The new velocity of the agent.
	/// @param[in]		oldParams	The parameters of the agent.
	/// @param[out]		newParams	The new parameters of the agent.
	int sampleVelocityAdaptive(const float* pos, const float rad, const float vmax,
							   const float* vel, const float* dvel, float* nvel,
							   const dtCollisionAvoidanceParams& oldParams, dtCollisionAvoidanceParams& newParams);

	/// Checks if the agent is in conflict with the registered obstacles.
	///
	/// @param[in]		pos		The position of the agent.
	/// @param[in]		dvel	The desired velocity of the agent.
	void prepare(const float* pos, const float* dvel);

	/// Checks if a collision is going to happen with the given velocity sample.
	///
	/// @param[in]		vcand		The samples velocity.
	/// @param[in]		pos			The position of the agent.
	/// @param[in]		rad			The radius of the agent.
	/// @param[in]		vel			The current velocity of the agent.
	/// @param[in]		dvel		The desired velocity of the agent.
	/// @param[in]		nvel		The new velocity of the agent.
	/// @param[in]		oldParams	The parameters of the agent.
	/// @param[out]		newParams	The new parameters of the agent.
	float processSample(const float* vcand, const float cs,
		const float* pos, const float rad,
		const float* vel, const float* dvel,
		const dtCollisionAvoidanceParams& oldParams, 
		dtCollisionAvoidanceParams& newParams);

	float m_invHorizonTime; ///< Inverse of 'horizonTime', used to speed up some processes
	float m_invVmax;						///< The inverse of the maximal speed.

    /// Adds a circle to the obstacles list.
	///
	/// @param[in]		pos		The position of the circle.
	/// @param[in]		rad		The radius of the circle.
	/// @param[in]		vel		The current velocity of the obstacle.
	/// @param[in]		dvel	The desired velocity of the obstacle.
	void addCircle(const float* pos, const float rad,
                   const float* vel, const float* dvel);
	
	/// Adds a segment to the obstacles list.
	///
	/// @param[in]	p	The position of the segment.
	/// @param[in]	q	The radius of the segment.
	void addSegment(const float* p, const float* q);
    
	dtObstacleCircle* m_circles;    ///< The circle obstacles.
	unsigned m_circlesCount;		///< Circle obstacles count.
	dtObstacleSegment* m_segments;	///< The segment obstacles.
	unsigned m_segmentsCount;		///< Segment obstacles count.
};

#endif
