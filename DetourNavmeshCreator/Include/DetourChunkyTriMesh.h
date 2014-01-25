//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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

#ifndef DETOURCHUNKYTRIMESH_H
#define DETOURCHUNKYTRIMESH_H

/// A triangle mesh ABBB tree.
///
/// @ingroup navmeshCreator
class dtChunkyTriMesh
{
public:
	dtChunkyTriMesh();
	~dtChunkyTriMesh();

	bool build(const float* vertices,
			   const unsigned* faces,
			   unsigned facesCount,
			   unsigned maximumFacesPerNode);
	void clear();

	/// Returns the node indices which overlap the input box.
	void retrieveNodesOverlappingBox(const float bmin[2], const float bmax[2],
									 unsigned* indices, unsigned* indicesCount, const unsigned indicesCapacity) const;

	/// Returns the node indices which overlap the input segment.
	void retrieveNodesOverlappingSegment(const float p1[2], const float p2[2],
										 unsigned* indices, unsigned* indicesCount, const unsigned indicesCapacity) const;

	struct Node
	{
		float bmin[2];
		float bmax[2];
		unsigned index; //!< If a leaf, the index of the first face belonging to the node, else the escape index.
		unsigned count; //!< Number of consecutives faces belonging to the node (0 if not a leaf).
		inline bool isLeaf() const { return count > 0; }
	};

	Node* m_nodes;
	unsigned m_nodesCount;

	int* m_faces;
	unsigned m_facesCount;

	unsigned m_maximumFacesPerNode;

private:
	dtChunkyTriMesh(const dtChunkyTriMesh&);
	dtChunkyTriMesh& operator=(const dtChunkyTriMesh&);
};




#endif
