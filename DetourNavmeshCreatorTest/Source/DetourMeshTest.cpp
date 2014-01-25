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

#include <DetourCommon.h>

#include <cstring>

SCENARIO("DetourMesh/Basics", "[mesh]")
{
	GIVEN("A default mesh")
	{
		dtMesh mesh;
		THEN("It is empty")
		{
			CHECK(mesh.countFaces() == 0);
			CHECK(mesh.countVertices() == 0);
		}

		THEN("It has no capacity")
		{
			CHECK(mesh.getVerticesCapacity() == 0);
			CHECK(mesh.getFacesCapacity() == 0);
		}

		THEN("Its bbox is safe to compute")
		{
			float bmin[3];
			float bmax[3];
			mesh.computeAABB(bmin, bmax);
		}

		WHEN("Reserving memory")
		{
			const unsigned verticesCount = 119;
			const unsigned facesCount = 60;

			mesh.reserve(verticesCount, facesCount);

			THEN("It is still empty")
			{
				CHECK(mesh.countFaces() == 0);
				CHECK(mesh.countVertices() == 0);
			}

			THEN("The capacities can fit the given sizes")
			{
				CHECK(mesh.getVerticesCapacity() >= verticesCount);
				CHECK(mesh.getFacesCapacity() >= facesCount);
			}

			THEN("The capacities are updated to the next power of 2")
			{
				CHECK(mesh.getVerticesCapacity() == 128);
				CHECK(mesh.getFacesCapacity() == 64);
			}
		}

		WHEN("A vertex is added")
		{
			const float x = 13.6;
			const float y = 10.5;
			const float z = -4.02;
			unsigned index;
			mesh.addVertex(x, y, z, index);
			THEN("Its index is 0")
			{
				CHECK(index == 0);
			}

			THEN("The mesh vertices capacity is 8")
			{
				CHECK(mesh.getVerticesCapacity() == 8);
			}

			THEN("The mesh vertices count is 1")
			{
				CHECK(mesh.countVertices() == 1);
			}

			THEN("It can be retrieved")
			{
				CHECK(mesh.getVertices()[0] == x);
				CHECK(mesh.getVertices()[1] == y);
				CHECK(mesh.getVertices()[2] == z);
			}
		}

		WHEN("3 vertices are created")
		{
			const float ax = 13.6;
			const float ay = 10.5;
			const float az = -4.02;
			unsigned a;
			mesh.addVertex(ax, ay, az, a);

			const float bx = 235.0;
			const float by = -1.5;
			const float bz = 23;
			unsigned b;
			mesh.addVertex(bx, by, bz, b);

			const float cx = 0.9;
			const float cy = -6.5;
			const float cz = 4;
			unsigned c;
			mesh.addVertex(cx, cy, cz, c);

			THEN("Their indices are different")
			{
				CHECK(a != b);
				CHECK(a != c);
				CHECK(b != c);
			}

			THEN("The mesh vertices capacity is 8")
			{
				CHECK(mesh.getVerticesCapacity() == 8);
			}

			THEN("The mesh vertices count is 3")
			{
				CHECK(mesh.countVertices() == 3);
			}

			THEN("They can be retrieved")
			{
				CHECK(mesh.getVertices()[0] == ax);
				CHECK(mesh.getVertices()[1] == ay);
				CHECK(mesh.getVertices()[2] == az);
				CHECK(mesh.getVertices()[3] == bx);
				CHECK(mesh.getVertices()[4] == by);
				CHECK(mesh.getVertices()[5] == bz);
				CHECK(mesh.getVertices()[6] == cx);
				CHECK(mesh.getVertices()[7] == cy);
				CHECK(mesh.getVertices()[8] == cz);
			}

			AND_WHEN("A faces is created from them")
			{
				const float nx = 0.707;
				const float ny = 0;
				const float nz = 0.707;
				unsigned index;
				mesh.addFace(a, b, c, nx, ny, nz, index);

				THEN("Its index is 0")
				{
					CHECK(index == 0);
				}

				THEN("The mesh faces capacity is 8")
				{
					CHECK(mesh.getFacesCapacity() == 8);
				}

				THEN("The mesh faces count is 1")
				{
					CHECK(mesh.countFaces() == 1);
				}

				THEN("It can be retrieved")
				{
					CHECK(mesh.getFaces()[0] == a);
					CHECK(mesh.getFaces()[1] == b);
					CHECK(mesh.getFaces()[2] == c);
				}

				THEN("Its normal can be retrieved")
				{
					CHECK(mesh.getNormals()[0] == nx);
					CHECK(mesh.getNormals()[1] == ny);
					CHECK(mesh.getNormals()[2] == nz);
				}

				THEN("Its bbox can be retrieved")
				{
					float bmin[3];
					float bmax[3];
					mesh.computeAABB(bmin, bmax);
					CHECK(bmin[0] == std::min(ax, std::min(bx, cx)));
					CHECK(bmin[1] == std::min(ay, std::min(by, cy)));
					CHECK(bmin[2] == std::min(az, std::min(bz, cz)));

					CHECK(bmax[0] == std::max(ax, std::max(bx, cx)));
					CHECK(bmax[1] == std::max(ay, std::max(by, cy)));
					CHECK(bmax[2] == std::max(az, std::max(bz, cz)));
				}
			}

			AND_WHEN("A faces is created from them w/o a normal")
			{
				unsigned index;
				mesh.addFace(a, b, c, index);

				THEN("Its index is 0")
				{
					CHECK(index == 0);
				}

				THEN("The mesh faces capacity is 8")
				{
					CHECK(mesh.getFacesCapacity() == 8);
				}

				THEN("The mesh faces count is 1")
				{
					CHECK(mesh.countFaces() == 1);
				}

				THEN("It can be retrieved")
				{
					CHECK(mesh.getFaces()[0] == a);
					CHECK(mesh.getFaces()[1] == b);
					CHECK(mesh.getFaces()[2] == c);
				}

				THEN("Its normal has been computed and can be retrieved")
				{
					float ab[3];
					dtVsub(ab, &mesh.getVertices()[3*b], &mesh.getVertices()[3*a]);

					float ac[3];
					dtVsub(ac, &mesh.getVertices()[3*c], &mesh.getVertices()[3*a]);

					float expectedNormal[3];
					dtVcross(expectedNormal, ab, ac);
					dtVnormalize(expectedNormal);

					CHECK(mesh.getNormals()[0] == expectedNormal[0]);
					CHECK(mesh.getNormals()[1] == expectedNormal[1]);
					CHECK(mesh.getNormals()[2] == expectedNormal[2]);
				}
			}
		}

		const float vertices[] = {
			20.0, 0.0, 20.0,
			20.0, 0.0, -20.0,
			-20.0, 0.0, -20.0,
			-20.0, 0.0, 20.0};

		const unsigned faces[] = {
			0, 1, 2,
			2, 3, 0};

		WHEN("Setting an external mesh")
		{
			const float normals[] = {
				0, 1, 0,
				0, 1, 0};

			mesh.set(vertices, 4, faces, normals, 2);

			THEN("The mesh capacities are 8")
			{
				CHECK(mesh.getVerticesCapacity() == 8);
				CHECK(mesh.getFacesCapacity() == 8);
			}

			THEN("The mesh vertices count is 4")
			{
				CHECK(mesh.countVertices() == 4);
			}

			THEN("The mesh faces count is 2")
			{
				CHECK(mesh.countFaces() == 2);
			}

			THEN("The vertices can be retrieved")
			{
				for (unsigned i(0) ; i < 12 ; ++i)
				{
					CAPTURE(i);
					CHECK(mesh.getVertices()[i] == vertices[i]);
				}
			}

			THEN("The faces can be retrieved")
			{
				for (unsigned i(0) ; i < 6 ; ++i)
				{
					CAPTURE(i);
					CHECK(mesh.getFaces()[i] == faces[i]);
					CHECK(mesh.getNormals()[i] == normals[i]);
				}
			}
		}

		WHEN("Setting an external mesh w/o normals")
		{
			mesh.set(vertices, 4, faces, 0, 2);

			THEN("The mesh capacities are 8")
			{
				CHECK(mesh.getVerticesCapacity() == 8);
				CHECK(mesh.getFacesCapacity() == 8);
			}

			THEN("The mesh vertices count is 4")
			{
				CHECK(mesh.countVertices() == 4);
			}

			THEN("The mesh faces count is 2")
			{
				CHECK(mesh.countFaces() == 2);
			}

			THEN("The vertices can be accessed")
			{
				for (unsigned i(0) ; i < 12 ; ++i)
				{
					CAPTURE(i);
					CHECK(mesh.getVertices()[i] == vertices[i]);
				}
			}

			THEN("The faces can be accessed")
			{
				for (unsigned i(0) ; i < 6 ; ++i)
				{
					CAPTURE(i);
					CHECK(mesh.getFaces()[i] == faces[i]);
				}
			}

			THEN("The faces can be retrieved")
			{
				float a[3];
				float b[3];
				float c[3];
				float n[3];
				mesh.retrieveFace(0, a, b, c, n);
				CHECK(a[0] == 20.f);
				CHECK(a[1] == 0.f);
				CHECK(a[2] == 20.f);

				CHECK(b[0] == 20.f);
				CHECK(b[1] == 0.f);
				CHECK(b[2] == -20.f);

				CHECK(c[0] == -20.f);
				CHECK(c[1] == 0.f);
				CHECK(c[2] == -20.f);

				CHECK(n[0] == 0.f);
				CHECK(n[1] == 1.f);
				CHECK(n[2] == 0.f);
			}

			THEN("The normals are the expected ones")
			{
				CHECK(mesh.getNormals()[0] == 0.f);
				CHECK(mesh.getNormals()[1] == 1.f);
				CHECK(mesh.getNormals()[2] == 0.f);
				CHECK(mesh.getNormals()[3] == 0.f);
				CHECK(mesh.getNormals()[4] == 1.f);
				CHECK(mesh.getNormals()[5] == 0.f);
			}
		}
	}
}

SCENARIO("DetourMesh/Obj", "[mesh]")
{
	const char* sampleObj = 	"# vertices \n\
								v 5.0 0.0 6.0 \n\
								v 5.0 0.0 -6.0 \n\
								v -5.0 0.0 -6.0 \n\
								v -5.0 0.0 6.0 \n\
								# faces \n\
								f 3 4 1 \n\
								f 1 2 3";
	unsigned sampleObjSize = strlen(sampleObj);

	dtMesh mesh;

	WHEN("Loading an .obj from a buffer")
	{
		CHECK(loadObjBuffer(sampleObj, sampleObjSize, mesh));

		THEN("The content of the .obj is in the mesh")
		{
			CHECK(mesh.countVertices() == 4);

			CHECK(mesh.getVertices()[0] == 5.f);
			CHECK(mesh.getVertices()[1] == 0.f);
			CHECK(mesh.getVertices()[2] == 6.f);

			CHECK(mesh.getVertices()[3] == 5.f);
			CHECK(mesh.getVertices()[4] == 0.f);
			CHECK(mesh.getVertices()[5] == -6.f);

			CHECK(mesh.getVertices()[6] == -5.f);
			CHECK(mesh.getVertices()[7] == 0.f);
			CHECK(mesh.getVertices()[8] == -6.f);

			CHECK(mesh.getVertices()[9] == -5.f);
			CHECK(mesh.getVertices()[10] == 0.f);
			CHECK(mesh.getVertices()[11] == 6.f);

			CHECK(mesh.countFaces() == 2);

			CHECK(mesh.getFaces()[0] == 2);
			CHECK(mesh.getFaces()[1] == 3);
			CHECK(mesh.getFaces()[2] == 0);

			CHECK(mesh.getFaces()[3] == 0);
			CHECK(mesh.getFaces()[4] == 1);
			CHECK(mesh.getFaces()[5] == 2);

			CHECK(mesh.getNormals()[0] == 0.f);
			CHECK(mesh.getNormals()[1] == 1.f);
			CHECK(mesh.getNormals()[2] == 0.f);

			CHECK(mesh.getNormals()[3] == 0.f);
			CHECK(mesh.getNormals()[4] == 1.f);
			CHECK(mesh.getNormals()[5] == 0.f);
		}
	}
}
