#pragma once

#ifdef LIPSALGO_EXPORTS
#define LIPSALGO_API __declspec(dllexport)
#else
#define LIPSALGO_API __declspec(dllimport)
#endif

#include <algorithm>
#include <atlstr.h>

namespace lips
{
	typedef void* HandleTriMeshModel;
	//handle to triangle mesh data returned from ScannerProcess::FinishScan()

	extern "C" LIPSALGO_API void GetRotMatrix(double *Rot, double l, double m, double n, double angle);
	extern "C" LIPSALGO_API void MultiplyMatrix33(double *m1, double *m2, double *Rsl);
	extern "C" LIPSALGO_API void NormalizeRotMatrix(double *Rot);
	extern "C" LIPSALGO_API void GetRotateMatrixByVector(double *Rotate, float vx, float vy, float vz, double theta);
	extern "C" LIPSALGO_API bool TransformObject(HandleTriMeshModel* handle_list, int idx);
	extern "C" LIPSALGO_API bool SelectTriangle(int px, int py, HandleTriMeshModel* handle_list, int MeshNum, int *idxNode, int *idxTri, int *idxEdge);
	extern "C" LIPSALGO_API void SpecificPointScale(int posX, int posY, int AposX, int AposY, int wheel_tmp, bool scale_bool);
	extern "C" LIPSALGO_API void RotateObj(double* RoateM, float rx, float ry, float rz);
	extern "C" LIPSALGO_API void RotateObjByAxis(double* RoateM, double* RoateAxis, float rx, float ry);
}

// eason-20210309
extern "C" LIPSALGO_API extern lips::HandleTriMeshModel pObj[10];
extern "C" LIPSALGO_API extern double Roate[10][9];
extern "C" LIPSALGO_API extern float Center[10][3];
extern "C" LIPSALGO_API extern float Scale[10][3];
extern "C" LIPSALGO_API extern float Trans[10][3];
extern "C" LIPSALGO_API extern float CameraTrans[3];
extern "C" LIPSALGO_API extern float Pivot[10][3];
extern "C" LIPSALGO_API extern double CameraRotate[9];
extern "C" LIPSALGO_API extern double RoateVector[9];
extern "C" LIPSALGO_API extern double Roate_tmp[9];
extern "C" LIPSALGO_API extern bool* SelTriLabel[10];
extern "C" LIPSALGO_API extern int BasePointNum[10];
extern "C" LIPSALGO_API extern int MeshNum;
extern "C" LIPSALGO_API extern int SelectNode;
extern "C" LIPSALGO_API extern int SelectTri;
extern "C" LIPSALGO_API extern int SelectEdge;
extern "C" LIPSALGO_API extern double ScaleObj;
extern "C" LIPSALGO_API extern int iHeight, iWidth;
extern "C" LIPSALGO_API extern float local_obj_centerx, local_obj_centery;
extern "C" LIPSALGO_API extern bool seltri_scale, after_seltri_scale;
extern "C" LIPSALGO_API extern int preindex_Triangle, aftersel_mousex, aftersel_mousey;
extern "C" LIPSALGO_API extern int seltripos_x, seltripos_y, pre_seltirpos_x, pre_seltirpos_y;
extern "C" LIPSALGO_API extern double Vx1, Vx2, Vy1, Vy2, Vz1, Vz2;
extern "C" LIPSALGO_API extern bool IsLighting;

