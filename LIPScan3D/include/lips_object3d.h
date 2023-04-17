#pragma once

#include <iostream>
#include <memory>

#ifdef LIPSOBJ3D_EXPORTS
#define LIPSOBJ3D_API __declspec(dllexport)
#else
#define LIPSOBJ3D_API __declspec(dllimport)
#endif

#pragma region // Triangle Mesh Model data struct declaration
struct Point3f;
struct Point2f;
struct ColorRGB;
struct Index3i;

#pragma endregion

namespace lips
{
	typedef void* HandleTriMeshModel;
	//handle to triangle mesh data returned from ScannerProcess::FinishScan()

	extern "C" LIPSOBJ3D_API Point3f* GetVertexBuffer(HandleTriMeshModel, int*);
	//Access triangle mesh model vertices array through handle, return type is specific struct for store 3d point cooedinates.
	//	[in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//	[out] p_num : Amount of vertices in triangle mesh model.
	//	[return] Pointer to vertices of triangle mesh model.

	extern "C" LIPSOBJ3D_API Point3f* GetNormalBuffer(HandleTriMeshModel);
	//Access triangle mesh model normal vector array of vertices through handle, return type is specific struct for store 3d point vector, the amount of normal vectors is equal to vertices.
	//	[in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//	[return] Pointer to normal vectors of triangle mesh model.

	extern "C" LIPSOBJ3D_API ColorRGB* GetColorBuffer(HandleTriMeshModel);
	//Access triangle mesh model color array of vertices through handle, return type is specific struct for store color information of vertices, the size of array is equal to vertices.
	//	[in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//	[return] Pointer to color information array of triangle mesh model.

	extern "C" LIPSOBJ3D_API Point2f* GetTexcoordBuffer(HandleTriMeshModel);
	//Access triangle mesh model texture mapping coordinates array of vertices through handle, return type is specific struct for store mapping coordinates of texture, the size of array is equal to vertices.
	//	[in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//	[return] Pointer to texture mapping coordinates array of triangle mesh model.

	extern "C" LIPSOBJ3D_API Index3i* GetTriangleBuffer(HandleTriMeshModel, int*);
	//Access triangular topology in triangle mesh model through handle, return type is specific struct for store triangular topology, the size of array is equal to triangle meshes.
	//	[in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//	[out] t_num : Amount of Triangle meshes in model data.
	//	[return] Pointer to triangular topology in triangle mesh model.

	extern "C" LIPSOBJ3D_API void GetColorRGBComp(ColorRGB*, int, unsigned char&, unsigned char&, unsigned char&);
	// Get duplicate of red, green, blue components in color structure.
	// [in] color, a valid pointer get from GetColorBuffer() function.
	// [int ]pos, [in] pos, index of color pointer, a valid pos value is 0~vertex number-1, vertex number can get from ::GetVertexBuffer() function.
	// [out] r, duplicate of red   component in color pointer.
	// [out] g, duplicate of green component in color pointer.
	// [out] b, duplicate of blue  component in color pointer.

	extern "C" LIPSOBJ3D_API void GetIndex3iComp(Index3i*, int, int&, int&, int&);
	// Get duplicate of vertex index components in triangle vertex topology structure.
	// [in] id_s, a valid pointer get from GetTriangleBuffer() function.
	// [in] pos, index of triangle vertex topology pointer, a valid pos value is 0~triangle amount-1, triangle amount can get from ::GetTriangleBuffer() function.
	// [out] id_v1, duplicate of vertex 1 index in triangle vertex topology.
	// [out] id_v2, duplicate of vertex 2 index in triangle vertex topology.
	// [out] id_v3, duplicate of vertex 3 index in triangle vertex topology.

	extern "C" LIPSOBJ3D_API void GetPoint3fComp(Point3f*, int, float&, float&, float&);
	// Get duplicate of coordinate components in vertex structure.
	// [in] pt3d, a valid vertex pointer get from GetVertexBuffer() and GetNormalBuffer() function.
	// [in] pos, index of vertex pointer, a valid pos value is 0~vertex amount-1, vertex amount can get from ::GetVertexBuffer() function.
	// [out] x, duplicate of x component in vertex pointer.
	// [out] y, duplicate of y component in vertex pointer.
	// [out] z, duplicate of z component in vertex pointer.

	extern "C" LIPSOBJ3D_API void SetPoint3fComp(Point3f*, int, float, float, float);
	// Get duplicate of coordinate components in vertex structure.
	// [in] pt3d, a valid vertex pointer get from GetVertexBuffer() and GetNormalBuffer() function.
	// [in] pos, index of vertex pointer, a valid pos value is 0~vertex amount-1, vertex amount can get from ::GetVertexBuffer() function.
	// [out] x, replacement of x component in vertex pointer.
	// [out] y, replacement of y component in vertex pointer.
	// [out] z, replacement of z component in vertex pointer.

	extern "C" LIPSOBJ3D_API void SetColorRGBComp(ColorRGB*, int, unsigned char, unsigned char, unsigned char);
	// Set red, green, blue components of mesh vertex color.
	// [in] color, a valid pointer get from GetColorBuffer() function.
	// [in] id, index of specified vertex which to set color components.
	// [in] r, duplicate of red   component in color pointer.
	// [in] g, duplicate of green component in color pointer.
	// [in] b, duplicate of blue  component in color pointer.

	extern "C" LIPSOBJ3D_API int* GetTriangles(HandleTriMeshModel, int vtxIndex, int* p_Num);
	//Access triangular topology in triangle mesh model through handle, get connected triangle index array and neighboring points.
	//  [in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//  [in] vtxIndex : Index of specific vertex in triangle mesh .
	//  [in] p_Num : Amount of Neighboring vertex of specific vertex.
	//  [return] Pointer to connected triangle indexes through specified vertex index.

	extern "C" LIPSOBJ3D_API Point3f* GenerateNormal(HandleTriMeshModel);
	//Calculate Normal vectors of each vertex from triangular mesh model, then return pointer to normal vectors, the return type is specific struct for store 3d point vector.the amount of normal vectors is equal to vertices.
	//	[in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//	[return] Pointer to normal vectors of triangle mesh model.

	extern "C" LIPSOBJ3D_API Point3f* CalculateNormal(Point3f*, int, Index3i*, int);
	//Calculate Normal vectors of each vertex from triangular mesh model, then return pointer to normal vectors, the return type is specific struct for store 3d point vector.the amount of normal vectors is equal to vertices.
	//	[in] p3d_Buf : pointer to vertices of mesh model.
	//  [in] p_num : Amount of vertices in model data.
	//  [in] tri_Buf : pointer to triangle topology of mesh model.
	//  [in] t_num : Amount of Triangle meshes in model data.
	//	[return] Pointer to normal vectors of triangle mesh model.

	extern "C" LIPSOBJ3D_API bool ReleaseTriMeshModel(HandleTriMeshModel);
	// Release memory of triangle mesh data through access handle of mesh model.
	//  [in] handle : Pointer to 3D reconstructed triangle mesh model, an valid handle would be from ScannerProcess::FinishScan().
	//  [return] true if release memory successfully.
}
