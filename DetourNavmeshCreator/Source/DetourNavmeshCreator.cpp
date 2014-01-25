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

#include "DetourNavmeshCreator.h"

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>

#include <RecastDump.h>

#include <Recast.h>
#include <RecastAssert.h>

#include <cmath>
#include <cstring>

namespace
{
	unsigned nextPow2(unsigned v)
	{
		v--;
		v |= v >> 1;
		v |= v >> 2;
		v |= v >> 4;
		v |= v >> 8;
		v |= v >> 16;
		v++;
		return v;
	}

	unsigned ilog2(unsigned v)
	{
		unsigned r;
		unsigned shift;
		r = (v > 0xffff) << 4; v >>= r;
		shift = (v > 0xff) << 3; v >>= shift; r |= shift;
		shift = (v > 0xf) << 2; v >>= shift; r |= shift;
		shift = (v > 0x3) << 1; v >>= shift; r |= shift;
		r |= (v >> 1);
		return r;
	}

	unsigned char* createTileData(const dtNavmeshInputGeometry& geometry,
								  const dtTiledNavmeshCfg& configuration,
								  dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
								  size_t& dataSize,
								  unsigned xIndex,
								  unsigned zIndex,
								  rcContext* context)
	{
		rcAssert(context);

		intermediateResults.reset();

		float tileMin[3];
		float tileMax[3];
		configuration.computeTileBbox(xIndex, zIndex, tileMin, tileMax, true);

		// Reset build times gathering.
		context->resetTimers();

		// Start the build process.
		context->startTimer(RC_TIMER_TOTAL);

		context->log(RC_LOG_PROGRESS, "Building navigation mesh tile");
		context->log(RC_LOG_PROGRESS, " - (%.2f, %.2f, %.2f)->(%.2f, %.2f, %.2f)",tileMin[0],tileMin[1], tileMin[2], tileMax[0], tileMax[1], tileMax[2]);
		context->log(RC_LOG_PROGRESS, " - %d x %d voxels", configuration.tiles.size, configuration.tiles.size);

		intermediateResults.triAreas = new unsigned char[geometry.tree.m_maximumFacesPerNode];
		if (!intermediateResults.triAreas)
		{
			context->log(RC_LOG_ERROR, "Out of memory 'triareas' (%d).", geometry.tree.m_maximumFacesPerNode);
			return 0;
		}

		float tileProjMin[2];
		float tileProjMax[2];
		tileProjMin[0] = tileMin[0];
		tileProjMin[1] = tileMin[2];
		tileProjMax[0] = tileMax[0];
		tileProjMax[1] = tileMax[2];

		// Find the chunkytrimesh nodes that overlapps the tile.
		static const unsigned nodesCapacity = 512; // TODO: Make grow when returning too many items.
		unsigned nodes[nodesCapacity];
		unsigned nodesCount;
		geometry.tree.retrieveNodesOverlappingBox(tileProjMin, tileProjMax, nodes, &nodesCount, nodesCapacity);
		if (nodesCount == 0)
		{
			context->log(RC_LOG_ERROR, "No overlapping triangles");
			return 0;
		}

		// Allocate voxel heightfield where we rasterize our input data to.
		intermediateResults.heightfield = rcAllocHeightfield();
		if (!intermediateResults.heightfield)
		{
			context->log(RC_LOG_ERROR, "Out of memory 'heightfield'.");
			return 0;
		}

		unsigned tilePaddedSize = configuration.computeTilePaddedSize();
		if (!rcCreateHeightfield(context, *intermediateResults.heightfield, tilePaddedSize, tilePaddedSize, tileMin, tileMax, configuration.voxels.size, configuration.voxels.height))
		{
			context->log(RC_LOG_ERROR, "Could not create solid heightfield.");
			return 0;
		}

		// Iterate over the chunk that overlap the tile.
		for (unsigned i = 0; i < nodesCount; ++i)
		{
			const dtChunkyTriMesh::Node& node = geometry.tree.m_nodes[nodes[i]];
			const int* faces = &geometry.tree.m_faces[node.index*3];
			const unsigned facesCount = node.count;

			memset(intermediateResults.triAreas, 0, facesCount*sizeof(unsigned char));

			// Compute walkable triangles (in terms of slope)
			rcMarkWalkableTriangles(context,
									configuration.navigation.maximumSlope,
									geometry.mesh.getVertices(),
									geometry.mesh.countVertices(),
									faces,
									facesCount,
									intermediateResults.triAreas);

			// Rasterize the triangles
			rcRasterizeTriangles(context,
								 geometry.mesh.getVertices(),
								 geometry.mesh.countVertices(),
								 faces,
								 intermediateResults.triAreas,
								 facesCount,
								 *intermediateResults.heightfield,
								 configuration.computeVoxelMaximumStepHeight());
		}

		// Once all geometry is rasterized, we do initial pass of filtering to
		// remove unwanted overhangs caused by the conservative rasterization
		// as well as filter spans where the character cannot possibly stand.
		rcFilterLowHangingWalkableObstacles(context, configuration.computeVoxelMaximumStepHeight(), *intermediateResults.heightfield);
		rcFilterLedgeSpans(context, configuration.computeVoxelMinimumCeilingClearance(), configuration.computeVoxelMaximumStepHeight(), *intermediateResults.heightfield);
		rcFilterWalkableLowHeightSpans(context, configuration.computeVoxelMinimumCeilingClearance(), *intermediateResults.heightfield);

		// Compact the heightfield so that it is faster to handle from now on.
		// This will result more cache coherent data as well as the neighbours
		// between walkable cells will be calculated.
		intermediateResults.compactHeightfield = rcAllocCompactHeightfield();
		if (!intermediateResults.compactHeightfield)
		{
			context->log(RC_LOG_ERROR, "Out of memory 'compactHeightField'.");
			return 0;
		}
		if (!rcBuildCompactHeightfield(context,
									   configuration.computeVoxelMinimumCeilingClearance(),
									   configuration.computeVoxelMaximumStepHeight(),
									   *intermediateResults.heightfield,
									   *intermediateResults.compactHeightfield))
		{
			context->log(RC_LOG_ERROR, "Could not build compact data.");
			return 0;
		}

		// Erode the walkable area by agent radius.
		if (!rcErodeWalkableArea(context, configuration.computeVoxelMinimumObstacleClearance(), *intermediateResults.compactHeightfield))
		{
			context->log(RC_LOG_ERROR, "Could not erode.");
			return 0;
		}

		if (configuration.regions.monotonePartioning)
		{
			// Partition the walkable surface into simple regions without holes.
			if (!rcBuildRegionsMonotone(context,
										*intermediateResults.compactHeightfield,
										configuration.computeTileOverlappingBorder(),
										configuration.regions.minSize,
										configuration.regions.mergeSize))
			{
				context->log(RC_LOG_ERROR, "Could not build regions.");
				return 0;
			}
		}
		else
		{
			// Prepare for region partitioning, by calculating distance field along the walkable surface.
			if (!rcBuildDistanceField(context,
									  *intermediateResults.compactHeightfield))
			{
				context->log(RC_LOG_ERROR, "Could not build distance field.");
				return 0;
			}

			// Partition the walkable surface into simple regions without holes.
			if (!rcBuildRegions(context,
								*intermediateResults.compactHeightfield,
								configuration.computeTileOverlappingBorder(),
								configuration.regions.minSize,
								configuration.regions.mergeSize))
			{
				context->log(RC_LOG_ERROR, "Could not build regions.");
				return 0;
			}
		}

		// Create contours.
		intermediateResults.contourSet = rcAllocContourSet();
		if (!intermediateResults.contourSet)
		{
			context->log(RC_LOG_ERROR, "Out of memory 'contourSet'.");
			return 0;
		}
		if (!rcBuildContours(context,
							 *intermediateResults.compactHeightfield,
							 configuration.polyMesh.edgeMaxError,
							 static_cast<int>(ceil(configuration.polyMesh.edgeMaxLength / configuration.voxels.size)),
							 *intermediateResults.contourSet))
		{
			context->log(RC_LOG_ERROR, "Could not create contours.");
			return 0;
		}

		if (intermediateResults.contourSet->nconts == 0)
		{
			context->log(RC_LOG_ERROR, "No contours were built.");
			return 0;
		}

		// Build polygon navmesh from the contours.
		intermediateResults.mesh = rcAllocPolyMesh();
		if (!intermediateResults.mesh)
		{
			context->log(RC_LOG_ERROR, "Out of memory 'mesh'.");
			return 0;
		}
		if (!rcBuildPolyMesh(context,
							 *intermediateResults.contourSet,
							 configuration.polyMesh.polyMaxNbVertices,
							 *intermediateResults.mesh))
		{
			context->log(RC_LOG_ERROR, "Could not triangulate contours.");
			return 0;
		}

		// Build detail mesh.
		intermediateResults.detailedMesh = rcAllocPolyMeshDetail();
		if (!intermediateResults.detailedMesh)
		{
			context->log(RC_LOG_ERROR, "Out of memory 'detailedMesh'.");
			return 0;
		}

		if (!rcBuildPolyMeshDetail(context,
								   *intermediateResults.mesh,
								   *intermediateResults.compactHeightfield,
								   configuration.polyMesh.sampleDist * configuration.voxels.size,
								   configuration.polyMesh.sampleMaxError * configuration.voxels.height,
								   *intermediateResults.detailedMesh))
		{
			context->log(RC_LOG_ERROR, "Could build polymesh detail.");
			return 0;
		}

		if (configuration.polyMesh.polyMaxNbVertices > DT_VERTS_PER_POLYGON)
		{
			context->log(RC_LOG_ERROR, "The desired number of vertices per polygon (%u) is higher than the absolute maximum number (%u)", configuration.polyMesh.polyMaxNbVertices, DT_VERTS_PER_POLYGON);
			return 0;
		}

		if (intermediateResults.mesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			context->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", intermediateResults.mesh->nverts, 0xffff);
			return 0;
		}

		unsigned char* navData = 0;

		// Update poly flags from areas.
		for (int i = 0; i < intermediateResults.mesh->npolys; ++i)
		{
			switch (intermediateResults.mesh->areas[i])
			{
				case RC_WALKABLE_AREA:
					intermediateResults.mesh->areas[i] = configuration.polygons.groundArea;
					intermediateResults.mesh->flags[i] = configuration.polygons.walkableFlag;
					break;
				case RC_NULL_AREA:
				default:
					intermediateResults.mesh->areas[i] = configuration.polygons.obstacleArea;
					intermediateResults.mesh->flags[i] = configuration.polygons.nonWalkableFlag;
					break;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = intermediateResults.mesh->verts;
		params.vertCount = intermediateResults.mesh->nverts;
		params.polys = intermediateResults.mesh->polys;
		params.polyAreas = intermediateResults.mesh->areas;
		params.polyFlags = intermediateResults.mesh->flags;
		params.polyCount = intermediateResults.mesh->npolys;
		params.nvp = intermediateResults.mesh->nvp;
		params.detailMeshes = intermediateResults.detailedMesh->meshes;
		params.detailVerts = intermediateResults.detailedMesh->verts;
		params.detailVertsCount = intermediateResults.detailedMesh->nverts;
		params.detailTris = intermediateResults.detailedMesh->tris;
		params.detailTriCount = intermediateResults.detailedMesh->ntris;
		float offmeshVertices[dtTiledNavmeshCfg::offmeshConnectionsCapacity * 6];
		float offmeshRadii[dtTiledNavmeshCfg::offmeshConnectionsCapacity];
		unsigned char offmeshBidirectional[dtTiledNavmeshCfg::offmeshConnectionsCapacity];
		unsigned char offmeshAreas[dtTiledNavmeshCfg::offmeshConnectionsCapacity];
		unsigned short offmeshFlags[dtTiledNavmeshCfg::offmeshConnectionsCapacity];
		unsigned int offmeshIds[dtTiledNavmeshCfg::offmeshConnectionsCapacity];
		for (unsigned iOffmesh(0); iOffmesh < configuration.offmeshConnectionsCount ; ++iOffmesh)
		{
			dtVcopy(&offmeshVertices[6*iOffmesh], configuration.offmeshConnections[iOffmesh].start);
			dtVcopy(&offmeshVertices[6*iOffmesh + 3], configuration.offmeshConnections[iOffmesh].end);
			offmeshRadii[iOffmesh] = configuration.offmeshConnections[iOffmesh].radius;
			offmeshBidirectional[iOffmesh] = configuration.offmeshConnections[iOffmesh].isBidirectionnal ? DT_OFFMESH_CON_BIDIR : 0;
			offmeshAreas[iOffmesh] = configuration.offmeshConnections[iOffmesh].areaType;
			offmeshFlags[iOffmesh] = configuration.offmeshConnections[iOffmesh].flags;
			offmeshIds[iOffmesh] = iOffmesh;
		}
		params.offMeshConVerts = offmeshVertices;
		params.offMeshConRad = offmeshRadii;
		params.offMeshConDir = offmeshBidirectional;
		params.offMeshConAreas = offmeshAreas;
		params.offMeshConFlags = offmeshFlags;
		params.offMeshConUserID = offmeshIds;
		params.offMeshConCount = configuration.offmeshConnectionsCount;
		params.walkableHeight = configuration.navigation.minimumCeilingClearance;
		params.walkableRadius = configuration.navigation.minimumObstacleClearance;
		params.walkableClimb = configuration.navigation.maximumStepHeight;
		params.tileX = static_cast<int>(xIndex);
		params.tileY = static_cast<int>(zIndex);
		params.tileLayer = 0;
		rcVcopy(params.bmin, intermediateResults.mesh->bmin);
		rcVcopy(params.bmax, intermediateResults.mesh->bmax);
		params.cs = configuration.voxels.size;
		params.ch = configuration.voxels.height;
		params.buildBvTree = true;

		int intDataSize = 0;
		if (!dtCreateNavMeshData(&params, &navData, &intDataSize))
		{
			if (context)
				context->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
		dataSize = intDataSize;

		context->stopTimer(RC_TIMER_TOTAL);

		// Show performance stats.
		duLogBuildTimes(*context, context->getAccumulatedTime(RC_TIMER_TOTAL));
		context->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", intermediateResults.mesh->nverts, intermediateResults.mesh->npolys);
		return navData;
	}
}

dtTiledNavmeshCreatorIntermediateResults::dtTiledNavmeshCreatorIntermediateResults()
: heightfield(0)
, triAreas(0)
, compactHeightfield(0)
, contourSet(0)
, mesh(0)
, detailedMesh(0)
{
	// NOTHING
}

dtTiledNavmeshCreatorIntermediateResults::~dtTiledNavmeshCreatorIntermediateResults()
{
	reset();
}

void dtTiledNavmeshCreatorIntermediateResults::reset()
{
	if (heightfield)
	{
		rcFreeHeightField(heightfield);
		heightfield = 0;
	}

	if (triAreas)
	{
		delete [] triAreas;
		triAreas = 0;
	}

	if (compactHeightfield)
	{
		rcFreeCompactHeightfield(compactHeightfield);
		compactHeightfield = 0;
	}

	if (contourSet)
	{
		rcFreeContourSet(contourSet);
		contourSet = 0;
	}

	if (mesh)
	{
		rcFreePolyMesh(mesh);
		mesh = 0;
	}

	if (detailedMesh)
	{
		rcFreePolyMeshDetail(detailedMesh);
		detailedMesh = 0;
	}
}

dtNavmeshInputGeometry::dtNavmeshInputGeometry()
: mesh()
, maximumFacesPerTreeNode(256)
, bmin()
, bmax()
, tree()
{
	dtVset(bmin, 0.f, 0.f, 0.f);
	dtVset(bmax, 0.f, 0.f, 0.f);
}

bool dtNavmeshInputGeometry::initialize()
{
	mesh.computeAABB(bmin, bmax);
	return tree.build(mesh.getVertices(), mesh.getFaces(), mesh.countFaces(), maximumFacesPerTreeNode);
}

bool dtCreateTiledNavmesh(const dtNavmeshInputGeometry& geometry,
						  const dtTiledNavmeshCfg& configuration,
						  dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
						  dtNavMesh& navmesh,
						  rcContext* context)
{
	rcAssert(context);

	context->log(RC_LOG_PROGRESS, "Building tiled navigation mesh");
	context->log(RC_LOG_PROGRESS, " - (%.2f, %.2f, %.2f)->(%.2f, %.2f, %.2f)",geometry.bmin[0],geometry.bmin[1], geometry.bmin[2], geometry.bmax[0], geometry.bmax[1], geometry.bmax[2]);
	context->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", geometry.mesh.countVertices()/1000.0f, geometry.mesh.countFaces()/1000.0f);
	context->log(RC_LOG_PROGRESS, " - %u x %u tiles of size %.2fwu", configuration.tiles.xCount, configuration.tiles.zCount, configuration.tiles.size * configuration.voxels.size);

	if (geometry.mesh.countFaces() == 0)
	{
		context->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	// Max tiles and max polys affect how the tile IDs are calculated.
	// There are 22 bits available for identifying a tile and a polygon.
	unsigned int tileBits = rcMin(ilog2(nextPow2(static_cast<unsigned int>(configuration.tiles.xCount*configuration.tiles.zCount))), 14u);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;

	dtNavMeshParams params;
	rcVcopy(params.orig,geometry.bmin);
	params.tileWidth = params.tileHeight = configuration.tiles.size * configuration.voxels.size;
	params.maxTiles = 1 << tileBits;
	params.maxPolys = 1 << polyBits;

	dtStatus status;

	status = navmesh.init(&params);
	if (dtStatusFailed(status))
	{
		if (context)
			context->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	// Start the build process.
	context->startTimer(RC_TIMER_TEMP);

	for (unsigned xIndex(0), xCount(configuration.tiles.xCount); xIndex < xCount; ++xIndex)
	{
		for (unsigned zIndex(0), zCount(configuration.tiles.zCount); zIndex < zCount; ++zIndex)
		{
			context->log(RC_LOG_PROGRESS, "==== TILE %ux%u ===", xIndex, zIndex);
			size_t tileSize;
			unsigned char* tileData = createTileData(geometry, configuration, intermediateResults, tileSize, xIndex, zIndex, context);

			if (tileData)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				navmesh.removeTile(navmesh.getTileRefAt(static_cast<int>(xIndex),static_cast<int>(zIndex),0),0,0);

				// Let the navmesh own the data.
				dtStatus status = navmesh.addTile(tileData,static_cast<int>(tileSize),DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
				{
					dtFree(tileData);
					return false;
				}
			}
			context->log(RC_LOG_PROGRESS, "=================");
		}
	}
	context->stopTimer(RC_TIMER_TEMP);

	return true;
}

bool dtCreateTiledNavmesh(const dtNavmeshInputGeometry& geometry,
						  const dtTiledNavmeshCfg& configuration,
						  dtNavMesh& navmesh,
						  rcContext* context)
{
	dtTiledNavmeshCreatorIntermediateResults intermediateResults;
	return dtCreateTiledNavmesh(geometry, configuration, intermediateResults, navmesh, context);
}
bool dtCreateTiledNavmeshTile(const dtNavmeshInputGeometry& geometry,
							  const dtTiledNavmeshCfg& configuration,
							  dtTiledNavmeshCreatorIntermediateResults& intermediateResults,
							  dtNavMesh& navmesh,
							  unsigned xIndex,
							  unsigned zIndex,
							  rcContext* context)
{
	rcAssert(context);

	if (geometry.mesh.countFaces() == 0)
	{
		context->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	// Start the build process.
	context->startTimer(RC_TIMER_TEMP);

	size_t tileSize;
	unsigned char* tileData = createTileData(geometry, configuration, intermediateResults, tileSize, xIndex, zIndex, context);

	if (tileData)
	{
		// Remove any previous data (navmesh owns and deletes the data).
		navmesh.removeTile(navmesh.getTileRefAt(static_cast<int>(xIndex),static_cast<int>(zIndex),0),0,0);
		// Let the navmesh own the data.
		dtStatus status = navmesh.addTile(tileData,static_cast<int>(tileSize),DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
		{
			dtFree(tileData);
			return false;
		}
	}
	
	// Start the build process.
	context->stopTimer(RC_TIMER_TEMP);

	return true;
}