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

#ifndef DETOURNAVMESHCREATOR_H
#define DETOURNAVMESHCREATOR_H

#include "DetourMesh.h"
#include "DetourChunkyTriMesh.h"
#include "DetourNavmeshCfg.h"

class dtNavMesh;

struct rcCompactHeightfield;
class rcContext;
struct rcContourSet;
struct rcHeightfield;
struct rcPolyMesh;
struct rcPolyMeshDetail;

/** @defgroup navmeshCreator Navmesh Creator
    @ingroup detour

    This module is composed of helpers to create @ref Detour usable navigation meshes
    with Recast.
 */

/// Intermediate results of the tiled mesh computation.
///
/// @ingroup navmeshCreator
struct dtTiledNavmeshCreatorIntermediateResults
{
	dtTiledNavmeshCreatorIntermediateResults();
	~dtTiledNavmeshCreatorIntermediateResults();

	rcHeightfield* heightfield;
	unsigned char* triAreas;
	rcCompactHeightfield* compactHeightfield;
	rcContourSet* contourSet;
	rcPolyMesh* mesh;
	rcPolyMeshDetail* detailedMesh;

	void reset();
};

/// Needed geometry data for navmesh computation.
///
/// @ingroup navmeshCreator
struct dtNavmeshInputGeometry
{
	/// Default constructor
	///
	/// - mesh = dtMesh()
	/// - maximumFacesPerTreeNode = 256
	/// - bmin = {0.f, 0.f, 0.f}
	/// - bmax = {0.f, 0.f, 0.f}
	/// - tree = dtChunkyTriMesh()
	dtNavmeshInputGeometry();

    /// Triangle mesh.
	dtMesh mesh;

    /// The maximum number of faces in generated @ref tree.
	unsigned maximumFacesPerTreeNode;

    /// Lower bound of the mesh's axis-aligned bounding box.
    ///
    /// Computed in @ref initialize().
	float bmin[3];

    /// Upper bound of the mesh's axis-aligned bounding box.
    ///
    /// Computed in @ref initialize().
	float bmax[3];

    /// Mesh's AABB Tree.
    ///
    /// Computed in @ref initialize().
	dtChunkyTriMesh tree;

	// Compute the bounding box and the tree from current mesh.
	bool initialize();
};

/// Create a tiled navigation mesh from the given input and settings
/// @ingroup navmeshCreator
bool dtCreateTiledNavmesh(const dtNavmeshInputGeometry& geometry,
						  const dtTiledNavmeshCfg& configuration,
						  dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
						  dtNavMesh& navmesh,
						  rcContext* context);

/// Create a tiled navigation mesh from the given input and settings
/// @note This version doesn't allow the retrieval of intermediate results.
///
/// @ingroup navmeshCreator
bool dtCreateTiledNavmesh(const dtNavmeshInputGeometry& geometry,
						  const dtTiledNavmeshCfg& configuration,
						  dtNavMesh& navmesh,
						  rcContext* context);

/// Create a navigation mesh tile from the given input and settings
///
/// @ingroup navmeshCreator
bool dtCreateTiledNavmeshTile(const dtNavmeshInputGeometry& geometry,
							  const dtTiledNavmeshCfg& configuration,
							  dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
							  dtNavMesh& navmesh,
							  unsigned row,
							  unsigned column,
							  rcContext* context);

#endif
