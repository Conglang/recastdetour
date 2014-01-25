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

#ifndef DETOURMESH_H
#define DETOURMESH_H

/// A triangle mesh.
///
/// @ingroup navmeshCreator
class dtMesh
{
public:
	/// @name Construction/Destruction
	//@{
	dtMesh();
	dtMesh(const dtMesh& other);
	~dtMesh();
	dtMesh& operator=(const dtMesh& other);
	//@}

	/// @name Vertices
	//@{
	inline const float* getVertices() const { return m_vertices; }
	inline unsigned countVertices() const { return m_verticesCount; }
	void addVertex(float x, float y, float z, unsigned& index);
	//@}

	/// @name Faces
	//@{
	inline const unsigned* getFaces() const { return m_faces; }
	inline const float* getNormals() const { return m_normals; }
	void retrieveFace(unsigned face, float a[3], float b[3], float c[3], float n[3]) const;
	const float* getNormal(unsigned face) const;
	inline unsigned countFaces() const { return m_facesCount; }
	void addFace(unsigned a, unsigned b, unsigned c, unsigned& index);
	void addFace(unsigned a, unsigned b, unsigned c, float nx, float ny, float nz, unsigned& index);
	//@}

	/// @name Capacity
	//@{
	/// Reserve memory to be able to fit the given amount of vertices and faces.
	void reserve(unsigned verticesCount, unsigned facesCount);
	inline unsigned getVerticesCapacity() const { return m_verticesCapacity; }
	inline unsigned getFacesCapacity() const { return m_facesCapacity; }
	//@}

	/// Set the content of the mesh from the given data.
	///
	/// @param vertices The vertices as an array of float of size 3*`verticesCount`.
	/// @param verticesCount The number of vertices.
	/// @param faces The faces as an array of unsigned of size 3*`facesCount`.
	/// @param normals The face normals as an array of float of size 3*`facesCount` [opt].
	/// @param facesCount The number of faces.
	void set(const float* vertices, unsigned verticesCount, const unsigned* faces, const float* normals, unsigned facesCount);

	/// Clear the content of the mesh (do not free memory)
	void clear();

	// Compute the axis-aligned bounding box of the the mesh
	void computeAABB(float* bmin, float* bmax) const;

private:

	void computeNormals(unsigned from, unsigned to);

	float* m_vertices;
	unsigned m_verticesCount;
	unsigned m_verticesCapacity;

	unsigned* m_faces;
	float* m_normals;
	unsigned m_facesCount;
	unsigned m_facesCapacity;
};

/// Load the content of the given `.obj` file to a mesh.
///
/// @ingroup navmeshCreator
bool loadObjFile(const char* filePath, dtMesh& mesh);

/// Load the content of the given `.obj` buffer to a mesh.
///
/// @ingroup navmeshCreator
bool loadObjBuffer(const char* buffer, unsigned bufferSize, dtMesh& mesh);


#endif
