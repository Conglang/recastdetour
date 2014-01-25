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

#include "DetourMesh.h"

#include <DetourAlloc.h>
#include <DetourAssert.h>
#include <DetourCommon.h>

#include <Recast.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>

dtMesh::dtMesh()
: m_vertices(0)
, m_verticesCount(0)
, m_verticesCapacity(0)
, m_faces(0)
, m_normals(0)
, m_facesCount(0)
, m_facesCapacity(0)
{
	// NOTHING
}

dtMesh::dtMesh(const dtMesh& other)
: m_vertices(0)
, m_verticesCount(0)
, m_verticesCapacity(0)
, m_faces(0)
, m_normals(0)
, m_facesCount(0)
, m_facesCapacity(0)
{
	*this = other;
}

dtMesh::~dtMesh()
{
	dtFree(m_vertices);
	dtFree(m_faces);
	dtFree(m_normals);
}

dtMesh& dtMesh::operator=(const dtMesh& other)
{
	if (this != &other)
	{
		set(other.m_vertices,
			other.m_verticesCount,
			other.m_faces,
			other.m_normals,
			other.m_facesCount);
	}
	return *this;
}

void dtMesh::addVertex(float x, float y, float z, unsigned& index)
{
	if (m_verticesCount + 1> m_verticesCapacity)
		reserve(m_verticesCount + 1, m_facesCount);

	dtVset(&m_vertices[m_verticesCount*3], x, y, z);
	index = m_verticesCount;
	++m_verticesCount;
}

void dtMesh::addFace(unsigned a, unsigned b, unsigned c, unsigned& index)
{
	if (m_facesCount + 1> m_facesCapacity)
		reserve(m_verticesCount, m_facesCount + 1);

	unsigned* it = &m_faces[m_facesCount*3];
	*it++ = a;
	*it++ = b;
	*it++ = c;
	index = m_facesCount;
	++m_facesCount;

	computeNormals(index, index + 1);
}

void dtMesh::addFace(unsigned a, unsigned b, unsigned c, float nx, float ny, float nz, unsigned& index)
{
	if (m_facesCount + 1> m_facesCapacity)
		reserve(m_verticesCount, m_facesCount + 1);

	dtVset(&m_normals[m_facesCount*3], nx, ny, nz);
	unsigned* it = &m_faces[m_facesCount*3];
	*it++ = a;
	*it++ = b;
	*it++ = c;
	index = m_facesCount;
	++m_facesCount;
}

void dtMesh::retrieveFace(unsigned face, float a[3], float b[3], float c[3], float n[3]) const
{
	dtAssert(face < m_facesCount);
	dtVcopy(n, &m_normals[3*face]);
	dtVcopy(a, &m_vertices[3*m_faces[3*face]]);
	dtVcopy(b, &m_vertices[3*m_faces[3*face+1]]);
	dtVcopy(c, &m_vertices[3*m_faces[3*face+2]]);
}

namespace
{
	unsigned nextPow2(unsigned n)
	{
		n--;
		n |= n >> 1;
		n |= n >> 2;
		n |= n >> 4;
		n |= n >> 8;
		n |= n >> 16;
		n++;
		return n;
	}
}
void dtMesh::reserve(unsigned verticesCount, unsigned facesCount)
{
	unsigned verticesCapacity = nextPow2(dtMax(verticesCount, m_verticesCapacity));
	verticesCapacity = verticesCapacity > 0 ? dtMax<unsigned>(8, verticesCapacity) : 0;
	if (verticesCapacity != m_verticesCapacity)
	{
		float* nv = (float*)dtAlloc(3*verticesCapacity*sizeof(float), DT_ALLOC_PERM);
		if (m_verticesCount)
			memcpy(nv, m_vertices, 3*m_verticesCount*sizeof(float));
		dtFree(m_vertices);
		m_vertices = nv;
		m_verticesCapacity = verticesCapacity;
	}

	unsigned facesCapacity = nextPow2(dtMax(facesCount, m_facesCapacity));
	facesCapacity = facesCapacity > 0 ? dtMax<unsigned>(8, facesCapacity) : 0;
	if (facesCapacity != m_facesCapacity)
	{
		unsigned* nf = (unsigned*)dtAlloc(3*facesCapacity*sizeof(unsigned), DT_ALLOC_PERM);
		float* nn = (float*)dtAlloc(3*facesCapacity*sizeof(float), DT_ALLOC_PERM);
		if (m_facesCount)
		{
			memcpy(nf, m_faces, 3*m_facesCount*sizeof(unsigned));
			memcpy(nn, m_normals, 3*m_facesCount*sizeof(float));
		}
		dtFree(m_faces);
		m_faces = nf;
		dtFree(m_normals);
		m_normals = nn;
		m_facesCapacity = facesCapacity;
	}
}

void dtMesh::set(const float* vertices, unsigned verticesCount, const unsigned* faces, const float* normals, unsigned facesCount)
{
	dtAssert(vertices);
	dtAssert(faces);

	clear();
	reserve(verticesCount, facesCount);

	memcpy(m_vertices, vertices, 3*verticesCount*sizeof(float));
	m_verticesCount = verticesCount;

	memcpy(m_faces, faces, 3*facesCount*sizeof(unsigned));
	m_facesCount = facesCount;

	if (normals)
	{
		memcpy(m_normals, normals, 3*facesCount*sizeof(float));
	}
	else
	{
		computeNormals(0, m_facesCount);
	}
}

void dtMesh::clear()
{
	m_verticesCount = m_facesCount = 0;
}

void dtMesh::computeAABB(float* bmin, float* bmax) const
{
	rcCalcBounds(m_vertices, m_verticesCount, bmin, bmax);
}

void dtMesh::computeNormals(unsigned from, unsigned to)
{
	unsigned a, b, c;
	float ab[3], ac[3];
	for (unsigned i(from) ; i < to ; ++i)
	{
		a = m_faces[i*3];
		b = m_faces[i*3+1];
		c = m_faces[i*3+2];

		dtAssert(a < m_verticesCount);
		dtAssert(b < m_verticesCount);
		dtAssert(c < m_verticesCount);

		dtVsub(ab, &m_vertices[3*b], &m_vertices[3*a]);
		dtVsub(ac, &m_vertices[3*c], &m_vertices[3*a]);

		dtVcross(&m_normals[3*i], ab, ac);
		dtVnormalize(&m_normals[3*i]);
	}
}

bool loadObjFile(const char* filePath, dtMesh& mesh)
{
	char* buffer = 0;
	FILE* fp = fopen(filePath, "rb");
	if (!fp)
		return false;
	fseek(fp, 0, SEEK_END);
	unsigned bufferSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	buffer = (char*)dtAlloc(bufferSize * sizeof(char), DT_ALLOC_PERM);
	if (!buffer)
	{
		fclose(fp);
		return false;
	}
	fread(buffer, bufferSize, 1, fp);
	fclose(fp);
	if (!loadObjBuffer(buffer, bufferSize, mesh))
	{
		dtFree(buffer);
		return false;
	}
	dtFree(buffer);
	return true;
}

namespace
{
	const char* parseRow(const char* bufferIt, const char* bufferEnd, char* row, unsigned rowLen)
	{
		bool start = true;
		bool done = false;
		int n = 0;
		while (!done && bufferIt != bufferEnd)
		{
			char c = *bufferIt;
			bufferIt++;
			// multirow
			switch (c)
			{
				case '\\':
					break;
				case '\n':
					if (start) break;
					done = true;
					break;
				case '\r':
					break;
				case '\t':
				case ' ':
					if (start) break;
				default:
					start = false;
					row[n++] = c;
					if (n >= rowLen-1)
						done = true;
					break;
			}
		}
		row[n] = '\0';
		return bufferIt;
	}

	static int parseFace(char* row, int* data, int n, unsigned verticesCount)
	{
		int j = 0;
		while (*row != '\0')
		{
			// Skip initial white space
			while (*row != '\0' && (*row == ' ' || *row == '\t'))
				row++;
			char* s = row;
			// Find vertex delimiter and terminated the string there for conversion.
			while (*row != '\0' && *row != ' ' && *row != '\t')
			{
				if (*row == '/') *row = '\0';
				row++;
			}
			if (*s == '\0')
				continue;
			int vi = atoi(s);
			data[j++] = vi < 0 ? vi+verticesCount : vi-1;
			if (j >= n) return j;
		}
		return j;
	}
}

bool loadObjBuffer(const char* buffer, unsigned bufferSize, dtMesh& mesh)
{
	const char* bufferIt = buffer;
	const char* bufferEnd = buffer + bufferSize;
	char row[512];
	int face[32];
	float x,y,z;
	int nv;
	unsigned vIndex, fIndex;

	mesh.clear();

	while (bufferIt != bufferEnd)
	{
		// Parse one row
		row[0] = '\0';
		bufferIt = parseRow(bufferIt, bufferEnd, row, sizeof(row)/sizeof(char));
		// Skip comments
		if (row[0] == '#') continue;
		if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		{
			// Vertex pos
			sscanf(row+1, "%f %f %f", &x, &y, &z);
			mesh.addVertex(x, y, z, vIndex);
		}
		if (row[0] == 'f')
		{
			// Faces
			nv = parseFace(row+1, face, 32, mesh.countVertices());
			for (int i = 2; i < nv; ++i)
			{
				const int a = face[0];
				const int b = face[i-1];
				const int c = face[i];
				if ((unsigned)a < mesh.countVertices() && (unsigned)b < mesh.countVertices() && (unsigned)c < mesh.countVertices())
					mesh.addFace(a, b, c, fIndex);
			}
		}
	}

	return true;
}
