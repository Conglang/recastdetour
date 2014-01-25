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

#include "DetourChunkyTriMesh.h"

#include <DetourCommon.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace
{
	struct FaceBbox
	{
		float bmin[2];
		float bmax[2];
		unsigned face[3];
	};

	int compareItemX(const void* va, const void* vb)
	{
		const FaceBbox* a = (const FaceBbox*)va;
		const FaceBbox* b = (const FaceBbox*)vb;
		if (a->bmin[0] < b->bmin[0])
			return -1;
		if (a->bmin[0] > b->bmin[0])
			return 1;
		return 0;
	}

	int compareItemY(const void* va, const void* vb)
	{
		const FaceBbox* a = (const FaceBbox*)va;
		const FaceBbox* b = (const FaceBbox*)vb;
		if (a->bmin[1] < b->bmin[1])
			return -1;
		if (a->bmin[1] > b->bmin[1])
			return 1;
		return 0;
	}

	void computeBbox(const FaceBbox* faces, const unsigned begin, const unsigned end,
					 float* bmin, float* bmax)
	{
		bmin[0] = faces[begin].bmin[0];
		bmin[1] = faces[begin].bmin[1];

		bmax[0] = faces[begin].bmax[0];
		bmax[1] = faces[begin].bmax[1];

		for (unsigned i(begin+1); i < end; ++i)
		{
			const FaceBbox& it = faces[i];
			if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
			if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];

			if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
			if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
		}
	}

	inline int longestAxis(float x, float y)
	{
		return y > x ? 1 : 0;
	}

	void recursiveBuild(FaceBbox* facesBbox, unsigned facesCount,
						unsigned faceBegin, unsigned faceEnd,
						unsigned maximumFacesPerNode,
						unsigned& currentNode, dtChunkyTriMesh::Node* nodes, const unsigned nodesCount,
						unsigned& currentFace, int* outFaces)
	{
		if (currentNode > nodesCount)
			return;

		unsigned currentFacesCount = faceEnd - faceBegin;
		unsigned previousCurrentNode = currentNode;

		dtChunkyTriMesh::Node& node = nodes[currentNode++];

		if (currentFacesCount <= maximumFacesPerNode)
		{
			// Leaf
			computeBbox(facesBbox, faceBegin, faceEnd, node.bmin, node.bmax);

			// Copy triangles.
			node.index = currentFace;
			node.count = currentFacesCount;

			for (unsigned i = faceBegin; i < faceEnd; ++i)
			{
				int* dst = &outFaces[currentFace*3];
				currentFace++;
				dst[0] = (int)facesBbox[i].face[0];
				dst[1] = (int)facesBbox[i].face[1];
				dst[2] = (int)facesBbox[i].face[2];
			}
		}
		else
		{
			// Split
			computeBbox(facesBbox, faceBegin, faceEnd, node.bmin, node.bmax);
			
			int	axis = longestAxis(node.bmax[0] - node.bmin[0],
								   node.bmax[1] - node.bmin[1]);
			
			if (axis == 0)
			{
				// Sort along x-axis
				qsort(facesBbox+faceBegin, currentFacesCount, sizeof(FaceBbox), compareItemX);
			}
			else if (axis == 1)
			{
				// Sort along y-axis
				qsort(facesBbox+faceBegin, currentFacesCount, sizeof(FaceBbox), compareItemY);
			}
			
			unsigned faceSplit = faceBegin+currentFacesCount/2;
			
			// Left
			recursiveBuild(facesBbox, facesCount, faceBegin, faceSplit, maximumFacesPerNode, currentNode, nodes, nodesCount, currentFace, outFaces);
			// Right
			recursiveBuild(facesBbox, facesCount, faceSplit, faceEnd, maximumFacesPerNode, currentNode, nodes, nodesCount, currentFace, outFaces);

			node.index = currentNode - previousCurrentNode;
			node.count = 0;
		}
	}
}

dtChunkyTriMesh::dtChunkyTriMesh()
: m_nodes(0)
, m_nodesCount(0)
, m_faces(0)
, m_facesCount(0)
, m_maximumFacesPerNode(0)
{
	// NOTHING
}

dtChunkyTriMesh::~dtChunkyTriMesh()
{
	clear();
}

bool dtChunkyTriMesh::build(const float* vertices,
							const unsigned* faces,
							unsigned facesCount,
							unsigned maximumFacesPerNode)
{
	clear();

	if (facesCount == 0)
		return true;

	unsigned nodesCount = (facesCount + maximumFacesPerNode-1) / maximumFacesPerNode;

	m_nodes = new Node[nodesCount * 4]; // Legacy mulplier, is it really needed to allocate that much ?

	if (!m_nodes)
		return false;
		
	m_faces = new int[facesCount * 3];
	if (!m_faces)
		return false;
		
	m_facesCount = facesCount;

	// Build tree
	FaceBbox* items = new FaceBbox[m_facesCount];
	if (!items)
		return false;

	for (unsigned i = 0; i < m_facesCount; i++)
	{
		const unsigned* face = &faces[i*3];
		FaceBbox& it = items[i];
		it.face[0] = face[0];
		it.face[1] = face[1];
		it.face[2] = face[2];

		// Calc face XZ bounds.
		it.bmin[0] = it.bmax[0] = vertices[face[0]*3+0];
		it.bmin[1] = it.bmax[1] = vertices[face[0]*3+2];
		for (unsigned j = 1; j < 3; ++j)
		{
			const float* vertex = &vertices[face[j]*3];
			if (vertex[0] < it.bmin[0]) it.bmin[0] = vertex[0];
			if (vertex[2] < it.bmin[1]) it.bmin[1] = vertex[2];

			if (vertex[0] > it.bmax[0]) it.bmax[0] = vertex[0];
			if (vertex[2] > it.bmax[1]) it.bmax[1] = vertex[2];
		}
	}

	unsigned currentFace = 0;
	unsigned currentNode = 0;
	recursiveBuild(items, m_facesCount,
				   0, m_facesCount,
				   maximumFacesPerNode,
				   currentNode, m_nodes, nodesCount*4,
				   currentFace, m_faces);
	
	delete [] items;
	
	m_nodesCount = currentNode;
	
	// Calc max tris per node.
	m_maximumFacesPerNode = 0;
	for (unsigned i = 0; i < m_nodesCount; ++i)
	{
		Node& node = m_nodes[i];
		if (!node.isLeaf()) continue;
		m_maximumFacesPerNode = dtMax(m_maximumFacesPerNode, node.count);
	}
	 
	return true;
}

void dtChunkyTriMesh::clear()
{
	delete[] m_nodes;
	delete[] m_faces;
}

namespace
{
	bool checkOverlapRect(const float amin[2], const float amax[2],
						  const float bmin[2], const float bmax[2])
	{
		bool overlap = true;
		overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
		overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
		return overlap;
	}
}

void dtChunkyTriMesh::retrieveNodesOverlappingBox(const float bmin[2], const float bmax[2],
												  unsigned* indices, unsigned* indicesCount, const unsigned indicesCapacity) const
{
	// Traverse tree
	*indicesCount = 0;
	for (unsigned iNode(0) ; iNode < m_nodesCount ; )
	{
		const dtChunkyTriMesh::Node& node = m_nodes[iNode];
		const bool overlap = checkOverlapRect(bmin, bmax, node.bmin, node.bmax);
		if (node.isLeaf() && overlap)
		{
			if (*indicesCount < indicesCapacity)
			{
				indices[*indicesCount] = iNode;
				(*indicesCount)++;
			}
		}
		if (overlap || node.isLeaf())
		{
			iNode++;
		}
		else
		{
			iNode += node.index; // Escape to the next sibling of its father.
		}
	}
}


namespace
{
	bool checkOverlapSegment(const float p[2], const float q[2],
							 const float bmin[2], const float bmax[2])
	{
		static const float EPSILON = 1e-6f;

		float tmin = 0;
		float tmax = 1;
		float d[2];
		d[0] = q[0] - p[0];
		d[1] = q[1] - p[1];

		for (int i = 0; i < 2; i++)
		{
			if (fabsf(d[i]) < EPSILON)
			{
				// Ray is parallel to slab. No hit if origin not within slab
				if (p[i] < bmin[i] || p[i] > bmax[i])
					return false;
			}
			else
			{
				// Compute intersection t value of ray with near and far plane of slab
				float ood = 1.0f / d[i];
				float t1 = (bmin[i] - p[i]) * ood;
				float t2 = (bmax[i] - p[i]) * ood;
				if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;
				if (tmin > tmax) return false;
			}
		}
		return true;
	}
}

void dtChunkyTriMesh::retrieveNodesOverlappingSegment(const float p1[2], const float p2[2],
													  unsigned* indices, unsigned* indicesCount, const unsigned indicesCapacity) const
{
	// Traverse tree
	*indicesCount = 0;
	for (unsigned iNode(0) ; iNode < m_nodesCount ; )
	{
		const dtChunkyTriMesh::Node& node = m_nodes[iNode];
		const bool overlap = checkOverlapSegment(p1, p2, node.bmin, node.bmax);
		if (node.isLeaf() && overlap)
		{
			if (*indicesCount < indicesCapacity)
			{
				indices[*indicesCount] = iNode;
				(*indicesCount)++;
			}
		}
		if (overlap || node.isLeaf())
		{
			iNode++;
		}
		else
		{
			iNode += node.index; // Escape to the next sibling of its father.
		}
	}
}
