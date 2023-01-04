#pragma once

#ifdef LIPSMODELEDIT_EXPORTS
#define LIPSMODELEDIT_API __declspec(dllexport)
#else
#define LIPSMODELEDIT_API __declspec(dllimport)
#endif

namespace lips
{
	typedef void* HandleTriMeshModel;
	//handle to triangle mesh data returned from ScannerProcess::FinishScan()
	
	extern "C" LIPSMODELEDIT_API int EOL_SelectAbnormalTriangles(HandleTriMeshModel handle, bool* SelectLabel);
	//Detect abnormal trinalge mesh in a whole model, and store detections to boolean table.
	//[in] handle, pointer to trinagle mesh model, valid handle is return from ScannerProcess::FinishScan().
	//[out] SelectLabel, true = abnormal, false = normal, table size = triangles in input model.
	//[return] Amounts of detected abnormal triangles.-1 = No detections.

	extern "C" LIPSMODELEDIT_API void OnEditDelTriangles(HandleTriMeshModel* handle_list, int idx, bool** SelTriLabel);
	//Delete specified triangles in a whole model, according to boolean table corresponding to triangles in model.
	//[in] handle_list, an pointer array to store handles of triangle mesh models(valid handle is returned from ScannerProcess::FinishScan())
	//[in] idx, specify model handle in 'handle_list' and specify boolean table in 'SelTriLabel', idx = 0~n-1.
	//[in] SelTriLabel, an pointer array to store pointers of boolean tables corresponding to handles.

	extern "C" LIPSMODELEDIT_API bool EOL_AverageMesh(HandleTriMeshModel handle, float *AbsMean, char *dsrPoint);
	//Smoothing surfaces of triangle mesh model with 
	//[in] handle, pointer to trinagle mesh model, valid handle is return from ScannerProcess::FinishScan().
	//[out] AbsMean, absolute mean deviation of mesh vertice computation.
	//[in] dsrPoint, boolean table to record selected vertex in triangle mesh, true means selected and false means un-selected, the size is equal to amount of vertices.
	//[return] true if smoothing mesh successfully, otherwise false.

	extern "C" LIPSMODELEDIT_API bool HC_Laplacian_SmoothMesh(HandleTriMeshModel handle, int n_step, float alpha, float beta, float* deviation);
	//Smoothing surfaces of triangle mesh model by preserve surface features.
	//[in] handle, pointer to trinagle mesh model, valid handle is return from ScannerProcess::FinishScan().
	//[return] true if smoothing mesh successfully, otherwise false.

	extern "C" LIPSMODELEDIT_API void getObjectHeightMinPosition(HandleTriMeshModel handle, float& x, float& y, float& z);
	//Get minima coordinates(x,y,z) in triangle mesh model vertices.
	//[in]handle, pointer to trinagle mesh model, valid handle is return from ScannerProcess::FinishScan().
	//[out]x, x coordinate of minima vertex in triangle mesh model.
	//[out]y, y coordinate of minima vertex in triangle mesh model.
 	//[out]z, z coordinate of minima vertex in triangle mesh model.

	extern "C" LIPSMODELEDIT_API void ResetSelectLabel(HandleTriMeshModel* handle_list, int idx, bool** SelTriLabel);
	//Reallocate memory of specified boolean table of corresponding triangle mesh model, default values of boolean table are 0.
	//[in]handle_list, an pointer array to store handles of triangle mesh models(valid handle is returned from ScannerProcess::FinishScan())
	//[in]idx, specify model handle in 'handle_list' and specify boolean table in 'SelTriLabel', idx = 0~n-1.
	//[in]SelTriLabel, an pointer array to store pointers of boolean tables corresponding to handles.
}