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

#ifndef DETOURCOHESIONBEHAVIOR_H
#define DETOURCOHESIONBEHAVIOR_H

#include "DetourSteeringBehavior.h"

struct dtCrowdAgent;
class dtArriveBehavior;
class dtCrowdQuery;


/// Parameters for the alignment behavior
/// @ingroup behavior
struct dtCohesionBehaviorParams
{
	const unsigned* targets;	///< The indices of the targets
	unsigned nbTargets;			///< The number of target
};


/// Defines the cohesion behavior.
///
/// An agent using the cohesion behavior will move towards the 
/// center of gravity of its targets.
/// @ingroup behavior
class dtCohesionBehavior : public dtSteeringBehavior<dtCohesionBehaviorParams>
{
public:
	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	dtCohesionBehavior(unsigned nbMaxAgents);
	~dtCohesionBehavior();

	/// Creates an instance of the behavior
	///
	/// @param[in]	nbMaxAgents		Estimation of the maximum number of agents using this behavior
	///
	/// @return		A pointer on a newly allocated behavior
	static dtCohesionBehavior* allocate(unsigned nbMaxAgents);

	/// Frees the given behavior
	///
	/// @param[in]	ptr	A pointer to the behavior we want to free
	static void free(dtCohesionBehavior* ptr);
	
	virtual void computeForce(const dtCrowdQuery& query, const dtCrowdAgent& ag, float* force, 
							  const dtCohesionBehaviorParams& currentParams, dtCohesionBehaviorParams& newParams);

private:
	/// Computes the average position of the given agents.
	///
	/// @param[in]	agents		The agents used to find the center of gravity.
	/// @param[in]	nbTargets	The number of targets.
	/// @param[out]	center		The computed center of gravity.
	void getGravityCenter(const dtCrowdQuery& query, const unsigned* targets, int nbTargets, float* center);
};

#endif
