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

#include "Utils.h"

#include <DetourMesh.h>
#include <DetourChunkyTriMesh.h>

#include <cstring>

SCENARIO("DetourChunkyTriMesh/Basics", "[chunkyTriMesh]")
{
	const char* sampleObj = "# vertices \n\
							v 5.0 0.0 0.0 \n\
							v 0.0 0.0 -5.0 \n\
							v -5.0 0.0 0.0 \n\
							v 0.0 0.0 5.0 \n\
							# faces \n\
							f 3 4 1 \n\
							f 1 2 3";

	unsigned sampleObjSize = strlen(sampleObj);

	GIVEN("A default chunkyTriMesh")
	{
		dtChunkyTriMesh tree;

		WHEN("building it from an empty mesh")
		{
			dtMesh mesh;
			CHECK(tree.build(mesh.getVertices(), mesh.getFaces(), mesh.countFaces(), 16));
			THEN("it is empty")
			{
				CHECK(tree.m_facesCount == 0);
				CHECK(tree.m_nodesCount == 0);
			}
		}

		WHEN("building it from a simple mesh with 16 max faces per node")
		{
			dtMesh mesh;
			REQUIRE(loadObjBuffer(sampleObj, sampleObjSize, mesh));

			CHECK(tree.build(mesh.getVertices(), mesh.getFaces(), mesh.countFaces(), 16));

			THEN("it contains all the mesh faces")
			{
				CHECK(tree.m_facesCount == mesh.countFaces());
			}

			THEN("it has only one node")
			{
				CHECK(tree.m_nodesCount == 1);
			}
		}

		WHEN("building it from a simple mesh with 1 max faces per node")
		{
			dtMesh mesh;
			REQUIRE(loadObjBuffer(sampleObj, sampleObjSize, mesh));

			CHECK(tree.build(mesh.getVertices(), mesh.getFaces(), mesh.countFaces(), 1));

			THEN("it contains all the mesh faces")
			{
				CHECK(tree.m_facesCount == mesh.countFaces());
			}

			THEN("it has three nodes (one root, two leaves)")
			{
				CHECK(tree.m_nodesCount == 3);
			}

			const unsigned indicesCapacity = 2;
			unsigned indices[indicesCapacity];

			THEN("1 leaf overlaps the box {{-1., 4.},{1., 6.}}")
			{
				const float bmin[2] = {-1., 4.};
				const float bmax[2] = {1., 6.};

				unsigned indicesCount;

				tree.retrieveNodesOverlappingBox(bmin, bmax, indices, &indicesCount, indicesCapacity);
				CHECK(indicesCount == 1);

				CHECK(tree.m_nodes[indices[0]].count == 1);
				CHECK(tree.m_faces[3*tree.m_nodes[indices[0]].index] == 2);
				CHECK(tree.m_faces[3*tree.m_nodes[indices[0]].index + 1] == 3);
				CHECK(tree.m_faces[3*tree.m_nodes[indices[0]].index + 2] == 0);
			}

			THEN("2 leaves overlaps the box {{-1., -1.},{1., 1.}}")
			{
				const float bmin[2] = {-1., -1.};
				const float bmax[2] = {1., 1};

				unsigned indicesCount;

				tree.retrieveNodesOverlappingBox(bmin, bmax, indices, &indicesCount, indicesCapacity);
				CHECK(indicesCount == 2);
			}

			THEN("no leaves overlaps the box {{-30., -30.},{-25., -25.}}")
			{
				const float bmin[2] = {-30., -30.};
				const float bmax[2] = {-25., -25.};

				unsigned indicesCount;

				tree.retrieveNodesOverlappingBox(bmin, bmax, indices, &indicesCount, indicesCapacity);
				CHECK(indicesCount == 0);
			}

			THEN("1 leaf overlaps the segment {{-6., -1.},{0., -2.}}")
			{
				const float p1[2] = {-6., -1.};
				const float p2[2] = {0., -2.};

				unsigned indicesCount;

				tree.retrieveNodesOverlappingSegment(p1, p2, indices, &indicesCount, indicesCapacity);
				CHECK(indicesCount == 1);

				CHECK(tree.m_nodes[indices[0]].count == 1);
				CHECK(tree.m_faces[3*tree.m_nodes[indices[0]].index] == 0);
				CHECK(tree.m_faces[3*tree.m_nodes[indices[0]].index + 1] == 1);
				CHECK(tree.m_faces[3*tree.m_nodes[indices[0]].index + 2] == 2);
			}

			THEN("2 leaves overlaps the segment {{-1., -1.},{0., 2.}}")
			{
				const float p1[2] = {-1., -1.};
				const float p2[2] = {0., 2.};

				unsigned indicesCount;

				tree.retrieveNodesOverlappingSegment(p1, p2, indices, &indicesCount, indicesCapacity);
				CHECK(indicesCount == 2);
			}

			THEN("no leaves overlaps the segment {{6., 0.},{6., 6.}}")
			{
				const float p1[2] = {6., 0.};
				const float p2[2] = {6., 6.};

				unsigned indicesCount;

				tree.retrieveNodesOverlappingSegment(p1, p2, indices, &indicesCount, indicesCapacity);
				CHECK(indicesCount == 0);
			}
		}
	}
}
