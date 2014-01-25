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

#include "DetourNavmeshCfg.h"

#include <DetourCommon.h>
#include <DetourMath.h>

#include <Recast.h>

#include <math.h>

/*** dtNavmeshVoxelCfg ***/

dtNavmeshVoxelCfg::dtNavmeshVoxelCfg()
: size(0.3f)
, height(0.2f)
{
	// NOTHING
}

/*** dtNavmeshTilesCfg ***/

dtNavmeshTilesCfg::dtNavmeshTilesCfg()
: xCount(0)
, zCount(0)
, size(32)
{
	// NOTHING
}

/*** dtNavmeshNavigationCfg ***/

dtNavmeshNavigationCfg::dtNavmeshNavigationCfg()
: minimumCeilingClearance(2.f)
, maximumStepHeight(0.9f)
, minimumObstacleClearance(0.3f)
, maximumSlope(45.f)
{
	// NOTHING
}

/*** dtNavmeshPolymeshCfg ***/

dtNavmeshRegionCfg::dtNavmeshRegionCfg()
: monotonePartioning(false)
, minSize(64)
, mergeSize(400)
{
	// NOTHING
}

/*** dtNavmeshPolymeshCfg ***/

dtNavmeshPolymeshCfg::dtNavmeshPolymeshCfg()
: edgeMaxError(1.3f)
, edgeMaxLength(12.f)
, polyMaxNbVertices(6)
, sampleDist(6)
, sampleMaxError(1)
{
	// NOTHING
}

/*** dtNavmeshPolygonsCfg ***/

dtNavmeshPolygonsCfg::dtNavmeshPolygonsCfg()
: walkableFlag(1 << 1)
, nonWalkableFlag(1 << 0)
, groundArea(1)
, obstacleArea(0)
{
	// NOTHING
};

/*** dtOffmeshConnectionCfg ***/

dtOffmeshConnectionCfg::dtOffmeshConnectionCfg()
: start()
, end()
, radius(1.f)
, isBidirectionnal(false)
, areaType(2)
, flags(1 << 1)
{
	dtVset(start, 0.f, 0.f, 0.f);
	dtVset(end, 0.f, 0.f, 0.f);
};

/*** dtTiledNavmeshCfg ***/

dtTiledNavmeshCfg::dtTiledNavmeshCfg()
: bmin()
, bmax()
, voxels()
, tiles()
, navigation()
, regions()
, polyMesh()
, polygons()
, offmeshConnections()
, offmeshConnectionsCount(0)
{
	dtVset(bmin, 0.f, 0.f, 0.f);
	dtVset(bmax, 0.f, 0.f, 0.f);
}

void dtTiledNavmeshCfg::computeTileCount(const float navmeshMin[3], const float navmeshMax[3], unsigned tileSize)
{
	dtVcopy(bmin, navmeshMin);
	dtVcopy(bmax, navmeshMax);

	int xVoxelCount = 0;
	int zVoxelCount = 0;
	rcCalcGridSize(bmin, bmax, voxels.size, &xVoxelCount, &zVoxelCount);

	tiles.size = tileSize;
	tiles.xCount = (xVoxelCount + tileSize - 1) / tileSize;
	tiles.zCount = (zVoxelCount + tileSize - 1) / tileSize;
}

unsigned dtTiledNavmeshCfg::computeVoxelMinimumCeilingClearance() const
{
	return (unsigned)dtMathCeilf(navigation.minimumCeilingClearance / voxels.height);
}

unsigned dtTiledNavmeshCfg::computeVoxelMaximumStepHeight() const
{
	return (unsigned)dtMathFloorf(navigation.maximumStepHeight / voxels.height);
}

unsigned dtTiledNavmeshCfg::computeVoxelMinimumObstacleClearance() const
{
	return (unsigned)dtMathCeilf(navigation.minimumObstacleClearance / voxels.size);
}

unsigned dtTiledNavmeshCfg::computeTileOverlappingBorder() const
{
	return computeVoxelMinimumObstacleClearance() + 3; // Arbitrary but legacy :)
}

unsigned dtTiledNavmeshCfg::computeTilePaddedSize() const
{
	return tiles.size + computeTileOverlappingBorder() * 2;
}

void dtTiledNavmeshCfg::computeTileBbox(unsigned xIndex, unsigned zIndex, float tileBMin[3], float tileBMax[3], bool padded) const
{
	if (xIndex >= tiles.xCount)
		return;

	if (zIndex >= tiles.zCount)
		return;

	dtVcopy(tileBMin, bmin);
	tileBMin[0] += static_cast<float>(xIndex) * tiles.size * voxels.size;
	tileBMin[2] += static_cast<float>(zIndex) * tiles.size * voxels.size;

	dtVcopy(tileBMax, bmin);
	tileBMax[1] = bmax[1];

	tileBMax[0] += static_cast<float>(xIndex + 1) * tiles.size * voxels.size;
	tileBMax[2] += static_cast<float>(zIndex + 1) * tiles.size * voxels.size;

	if (padded)
	{
		float padding = computeTileOverlappingBorder() * voxels.size;
		tileBMin[0] -= padding;
		tileBMin[2] -= padding;
		tileBMax[0] += padding;
		tileBMax[2] += padding;
	}
}