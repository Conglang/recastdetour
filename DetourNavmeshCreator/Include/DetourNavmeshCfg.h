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

#ifndef DETOURNAVMESHCFG_H
#define DETOURNAVMESHCFG_H

/// Voxels generation configuration.
///
/// @ingroup navmeshCreator
struct dtNavmeshVoxelCfg
{
	/// Default constructor
	///
	/// - size = 0.3 wu
	/// - height = 0.2 wu
	dtNavmeshVoxelCfg();

	float size; ///< The size of the voxels along 'horizontal', ie x and z, axises (in world units).
	float height; ///< The size of the voxel along the y axis (in world units).
};

/// Tiles generation configuration.
///
/// @ingroup navmeshCreator
struct dtNavmeshTilesCfg
{
	/// Default constructor
	///
	/// - xCount = 0
	/// - zCount = 0
	/// - size = 32 vx
	dtNavmeshTilesCfg();

	unsigned xCount; ///< Number of tiles along the x axis, ie the number of tile colums.
	unsigned zCount; ///< Number of tiles along the z axis, ie the number of tile rows.
	unsigned size; ///< Size of a tile, along x and z axises (in voxels).
};

/// Navigation constraints configuration.
///
/// @ingroup navmeshCreator
struct dtNavmeshNavigationCfg
{
	/// Default constructor
	///
	/// - minimumCeilingClearance = 2 wu
	/// - maximumStepHeight = 0.9 wu
	/// - minimumObstacleClearance = 0.3 wu
	/// - maximumSlope = 45 degrees
	dtNavmeshNavigationCfg();

	float minimumCeilingClearance; ///< The minimum needed clearance to the ceiling (in world units).
	float maximumStepHeight; ///< The maximum climbable step height (in world units).
	float minimumObstacleClearance; ///< The minimum needed 'horizontal' clearance to obstacles (in world units).
	float maximumSlope; ///< The maximum navigable slope (in degrees)
};

/// Region generation configuration.
///
/// @ingroup navmeshCreator
struct dtNavmeshRegionCfg
{
	/// Default constructor
	///
	/// - monotonePartioning = false
	/// - minSize = 64
	/// - mergeSize = 400
	dtNavmeshRegionCfg();

	bool monotonePartioning; //!< Use monotone partitionning.
	unsigned minSize; //!< Regions having less that this number of voxels are pruned.
	unsigned mergeSize; //!< Regions having less than this number of voxels are merged.
};

/// Polymesh generation configuration.
///
/// @ingroup navmeshCreator
struct dtNavmeshPolymeshCfg
{
	/// Default constructor
	///
	/// - edgeMaxError = 1.3 wu
	/// - edgeMaxLength = 12 wu
	/// - polyMaxNbVertices = 6
	/// - sampleDist = 6 vx
	/// - sampleMaxError = 1 vx
	dtNavmeshPolymeshCfg();

	float edgeMaxError; ///< The maximum distance the contour should deviate from the raw border.
	float edgeMaxLength; ///< The maximum length of edges.

	unsigned polyMaxNbVertices; ///< Maximum number of vertices in generated polygons.

	unsigned sampleDist; ///< The distance between ground samples (in voxels).
	unsigned sampleMaxError; ///< The maximum vertical distance the detail mesh surface should deviate from heightfield data (in voxels).
};

/// Navmesh polygons area types and flags configuration.
///
/// @ingroup navmeshCreator
struct dtNavmeshPolygonsCfg
{
	/// Default constructor
	///
	/// - walkableFlag = 1 << 1
	/// - nonWalkableFlag = 1 << 0
	/// - groundArea = 1
	/// - obstacleArea = 0
	dtNavmeshPolygonsCfg();

	unsigned short walkableFlag; ///< Flag used for walkable polygons.
	unsigned short nonWalkableFlag; ///< Flag used for non-walkable polygons.
	
	unsigned char groundArea; ///< Ground area id.
	unsigned char obstacleArea; ///< Obstacle area id.
};

/// Navmesh offmesh connection configuration.
///
/// @ingroup navmeshCreator
struct dtOffmeshConnectionCfg
{
	/// Default constructor
	///
	/// - start = {0.f, 0.f, 0.f}
	/// - end = {0.f, 0.f, 0.f}
	/// - radius = 1.f
	/// - isBidirectionnal = false
	/// - areaType = 2
	/// - flags = 1 << 1 (matchs the default walkable flag)
	dtOffmeshConnectionCfg();

	float start[3]; ///< Start position for the offmesh connection.
	float end[3];   ///< End position.
	float radius;   ///< Radius under which agents can use the connection
	bool isBidirectionnal;	///< Is the connection bi directional?
	unsigned char areaType; ///< Area type for the connection.
	unsigned short flags; ///< Flags for the connection.
};

/// Tiled navmesh generation configuration.
///
/// @ingroup navmeshCreator
struct dtTiledNavmeshCfg
{
	/// Default constructor
	///
	/// - bmin = {0.f, 0.f, 0.f}
	/// - bmax = {0.f, 0.f, 0.f}
	/// - voxels = dtNavmeshVoxelCfg()
	/// - tiles = dtNavmeshTilesCfg()
	/// - navigation = dtNavmeshNavigationCfg()
	/// - regions = dtNavmeshRegionCfg()
	/// - polyMeshCfg = dtNavmeshPolymeshCfg()
	/// - polygons = dtNavmeshPolygonsCfg()
	/// - offmeshConnections = {}
	/// - offmeshConnectionsCount = 0
	dtTiledNavmeshCfg();

	/// Compute the needed count of tiles.
	///
	/// @param navmeshMin The lower bound of the desired navmesh's axis aligned bounding box.
	/// @param navmeshMax The upper bound of the desired navmesh's axis aligned bounding box.
	/// @param tileSize The desired tile size (in voxels)
	///
	/// @note replace the existing `bmin`, `bmax` and `tilesCfg` content.
	void computeTileCount(const float navmeshMin[3], const float navmeshMax[3], unsigned tileSize);

	/// Compute the voxelized minimum ceiling clearance.
	unsigned computeVoxelMinimumCeilingClearance() const;

	/// Compute the voxelized maximum step height.
	unsigned computeVoxelMaximumStepHeight() const;

	/// Compute the voxelized minimum obstacle clearance.
	unsigned computeVoxelMinimumObstacleClearance() const;

	/// Compute the tile overlapping border size (in voxels).
	///
	/// This border size is used to ensure a good enough overlapping between neighbor tiles.
	unsigned computeTileOverlappingBorder() const;

	/// Compute a tile size in voxels (including the overlapping border)
	unsigned computeTilePaddedSize() const;

	/// Compute the bounding box of the given tile.
	void computeTileBbox(unsigned xIndex, unsigned zIndex, float tileBMmin[3], float tileBMax[3], bool padded = false) const;

	float bmin[3];
	float bmax[3];
	dtNavmeshVoxelCfg voxels;
	dtNavmeshTilesCfg tiles;
	dtNavmeshNavigationCfg navigation;
	dtNavmeshRegionCfg regions;
	dtNavmeshPolymeshCfg polyMesh;
	dtNavmeshPolygonsCfg polygons;

	static const unsigned offmeshConnectionsCapacity = 512;
	dtOffmeshConnectionCfg offmeshConnections[offmeshConnectionsCapacity];
	unsigned offmeshConnectionsCount;
};

#endif
