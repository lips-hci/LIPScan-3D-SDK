#pragma once

// LIPScan3D ConsoleApp.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//

#include <string>
#include <iostream>
#include <filesystem>
#include <windows.h>
#include <stdio.h>
#include <direct.h>
#include <gl/GL.h>
#include <gl/GLU.h>

#pragma region//LIPScan3D SDK console application, eason-20210813//

#include "configParser.h"
#include "lips_camera.h"
#include "lips_camera_set.h"
#include "lips_scanner.h"
#include "lips_scanner_set.h"
#include "lips_object3d.h"
#include "lips_model_edit.h"
#include "algorithm.h"

#ifdef LIPS_CLOUDCOMPARE
#include "lips_cloud_compare.h"
#endif

using namespace std;
using namespace lips;

#define LIPSCAN3D_PROCESS
#define DEPTH_TO_POINT
#define DepthEvaluation
#define ICP_3D
#define FINISH_3DSCAN
#define VIEWER_CAMERA
#define VIEWER_3DSCAN
#define MESHCOMPARISON_COLORGAUGE
#define DEFECTS_2D_DETECTION

#pragma region Process const char array
typedef std::uint64_t hash_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

hash_t hash_(char const* str)
{
	hash_t ret{ basis };
	while (*str) {
		ret ^= *str;
		ret *= prime;
		str++;
	}
	return ret;
}
constexpr hash_t hash_compile_time(char const* str, hash_t last_value = basis)
{
	return *str ? hash_compile_time(str + 1, (*str ^ last_value) * prime) : last_value;
}
#pragma endregion


// Configurations of Realsense Camera
bool FirstRun = true;
char RootDirectory[512];
typedef vector<string> RealsenseCfgVector;
typedef map<lips::RangeParam_lib, RealsenseCfgVector> Scan3dCfgMap;


typedef struct SIZE_MODE_CONFIG
{
	string scanner;
	string realsense;
};
typedef std::map<string, SIZE_MODE_CONFIG> SIZE_MODE_CONFIG_MAP;
SIZE_MODE_CONFIG_MAP camera_cfg_map;
SIZE_MODE_CONFIG_MAP Init3DscanConfig(const char* rootdir, const char* devicename, const char* resolution);

// Object for Implement stereo 3D reconstruction
bool connect_camera;
int camera_slot;
string camera_resolution = "1280x720"; //Default 
char* camera_devicename = new char[100];
int frame_width;
int frame_height;
string gScan_mode = "medium"; //Default
lips::RangeParam_lib gScan_range = lips::RangeParam_lib::RangeParam_lib_MEDIUM; //default scan mode: 25~40 cm
lips::Camera* gCamera = NULL; //high-level API class of AE400
lips::DepthmapProcess* gDepthpostproc = NULL; // depth image post-rocessing API class
lips::ScannerProcess gReconstructor; //point cloud registraion & triangulation API class
lips::Rect frame_roi;
cv::Rect cv_frame_roi;

//--- Initialize & Release LIPScan 3D SDK ---
bool InitLIPScan3D(char*, int&, int&); //Initialize API object
void ReleaseLIPScan3D(void); //Release API object

//--- Export Camera Frame ---
int FrameCount = 0;
bool dirExists(const std::string& path);
void SaveCameraScannerFrames(void);

//--- 3D Scan off-line Process---
unsigned int ColorTable[256];
void InitDepthToColorTable(void);
void DepthToColor(unsigned short*, unsigned char*, int, int);
void ImportCameraRawData(char*, char*, char*, unsigned char*, unsigned short*, float*, int, int);
HandleTriMeshModel Scan3D_Offline_Process(char*, int, int, int);

//--- Automatically cycling 3D scan process ---
UINT trigger_count;
BOOL trigger_scan_3D;
BOOL finish_scan_3D;
char localtime_str[512];
double scan_3D_duration;
HANDLE auto_cycling_scan3D_thread_handle = NULL;
DWORD auto_cycling_scan3D_ExitCode;
DWORD __stdcall auto_cycling_scan3D_ThreadProc(void);

//--- Variables for 3D reconstruction ---
float* points_data;             // Depth data in real world coordinate

//--- Variables for Display Camera Frame ---
IplImage* color_img;
IplImage* depth_output;
IplImage* depth_img;                // RGB scale of depth image
unsigned char* color_map = NULL;    // RGB image buffer
unsigned short* depth_data = NULL;  // Depth image buffer
unsigned char* depth_map = NULL;    // Pixel buffer for RGB scale of depth image

//--- Rotate Camera Images ---
void RotateDisplayImage(unsigned char*, int, int, int);
//--- Variables for Display Scanning Model Image -------
unsigned char* copy_color_map = NULL; //avoid bug of display image
IplImage* scan_img;             // 3D Model image
unsigned char* scan_map = NULL;	// 3D model image pixel data
//----- Variables for Access 3D mesh object ---------
bool bScanFinish = false;
lips::HandleTriMeshModel gHandle_obj3d = NULL; //handle for current 3D reconstruction

//----- OpenGL render mesh model -----
struct PrmLight
{
	float Ambient[4];
	float Diffuse[4];
	float Specular[4];
	float Position[4];
	float SpotDirection[3];
	float SpotExponent;
	float SpotCutoff;
	float ConstantAttenuation;
	float LinearAttenuation;
	float QuadraticAttenuation;
};
struct ORTHO_PROJECTION_POSITION
{
	GLdouble left;
	GLdouble right;

	GLdouble bottom;
	GLdouble top;

	GLdouble znear;
	GLdouble zfar;
};
PrmLight *m_pLight = new PrmLight;
ORTHO_PROJECTION_POSITION* orth_position = new ORTHO_PROJECTION_POSITION;
float pointMinHeightVertex[3];
float CameraTrans[3];
float CameraPos[3];
double CameraRotate[9];
double ScaleObj;
UINT RenderMode = 0; // 0:Color, 1:Mesh, 2:WireFrame
void InitGLCamera();
void InitLight(PrmLight *);
void SetLights(PrmLight *);
void ResetObjPosition(lips::HandleTriMeshModel*, int);
void ResetCameraView(float*, float*, double*, float);
void PrepareTransformation(lips::HandleTriMeshModel*, int);
void DrawGLScene(); //3D transformation API provided from liblipscan3d.lib
void ChangeCursor(HINSTANCE hinstance, LPCSTR cursor_name); //Display default cursor
void ChangeCursor_HourGlass(void); //Display hourglass cursor

//--- Mesh Data Buffer for OpenGL rendering ----
struct MeshRenderBuffer
{
	MeshRenderBuffer()
	{
		TriBuf = NULL;
		P3dBuf = NULL;
		NrmBuf = NULL;
		ColorBuf = NULL;
		SelBuf = NULL;
	}
	int TriNum;
	int* TriBuf;
	float* P3dBuf;
	float* NrmBuf;
	unsigned char* ColorBuf;
	bool* SelBuf;
};
MeshRenderBuffer MeshBuf[10];
void PrepareRenderBuf(lips::HandleTriMeshModel, MeshRenderBuffer&);
void ReleaseRenderBuf(MeshRenderBuffer&);
void RenderMeshList(MeshRenderBuffer*, ORTHO_PROJECTION_POSITION*, int, size_t, bool, bool*);

#pragma region CLOUDCOMPARE_MENU
#ifdef LIPS_CLOUDCOMPARE
//--- define event for mesh comparison pop-up menu ---
#define SWM_ICP       WM_APP + 3 // Fine registration (ICP)
#define SWM_DISTANCE  WM_APP + 4 // Compute mesh distances
#define SWM_LOAD_COMPMESH WM_APP + 6 // Load Ref and Comp mesh file
#define SWM_LOAD_REFMESH  WM_APP + 7 // Matching Point Pair Align
#define SWM_PICK_COMPMESH_POINTS WM_APP + 8
#define SWM_PICK_REFMESH_POINTS  WM_APP + 9
#define SWM_MATCHING_POINT_PAIRS WM_APP + 10
#define SWM_EXPORT_MESHDISTANCE_HISTOGRAM WM_APP + 11
#define SWM_EXPORT_DISTANCERGB_MESH WM_APP + 12
#define SWM_CLEAR_COMPMESH_POINTS WM_APP + 13
#define SWM_CLEAR_REFMESH_POINTS WM_APP + 14
#define SWM_RESET_MESHCOMPARISON WM_APP + 15
#define SWM_POSE_COMPMESH WM_APP + 16
#define SWM_POSE_REFMESH WM_APP + 17
#define SWM_COMPMESH_VISIBILITY WM_APP + 18
#define SWM_REFMESH_VISIBILITY WM_APP + 19
#define SWM_RESET WM_APP + 20
#define SWM_COLORSCALE WM_APP + 21
#define SWM_SET_SCAN_COMPMESH WM_APP + 22 // Set Scan Data to Comp Mesh
//----------------------------------------------------

enum MeshComparisonState {
	NONE,
	LOAD,
	POSE,
	PICK,
	ALIGN,
	ICP,
	COMPUTE,
	EXPORT
};
unsigned int process_mode = 0; //0: 3D Scan, 1:Mesh Comparison
void PopUpMenu_MeshComparison(HWND, MeshComparisonState&, MeshComparison&); //Pop-up mesh for mesh comparison
lips::MeshComparison compare_mesh_tool; //mesh comparison
MeshComparisonState meshcomparer_status = MeshComparisonState::NONE; //status recorder
int ICP_criteria = -5;
int ICP_sampling_num = 50000;
bool bVisibleObj[10] = {0,0,0,0,0,0,0,0,0,0};
bool bTransformObj[10] = { 0,0,0,0,0,0,0,0,0,0 };
string opensavefilename(const char*, HWND, bool);
void load_comp_mesh(MeshComparison&);
void load_ref_mesh(MeshComparison&);
void clear_comp_mesh(MeshComparison&);
void clear_ref_mesh(MeshComparison&);

vector<string> colorscale_names;
unsigned int colorscale_index = 0;
int octree_projection = 0;
bool SelectTriangleAll(int, int, HandleTriMeshModel*, bool*, int, int *, int *, int *);
void ResetObjPositionAll();
void TranslateMeshCompareObj(float m_x, float m_y, float m_z);
void RotateMeshCompareObj(float m_x, float m_y, float m_z);
bool ResizeMeshObj(HandleTriMeshModel, float);

//--- Texture mapping of MeshComparison ---
#define GL_CLAMP_TO_EDGE 0x812F
typedef struct GL_POINT2D
{
	int x;
	int y;
};
typedef struct RENDER_IMAGE_LOCATION
{
	GL_POINT2D p1;
	GL_POINT2D p2;
};
ORTHO_PROJECTION_POSITION* orth_image_position = new ORTHO_PROJECTION_POSITION;
void RenderImageTexture(GLuint, ORTHO_PROJECTION_POSITION*, RENDER_IMAGE_LOCATION, int, int);
void ByteArrayToTexture(unsigned char*, GLuint*, int, int);
ColorScaleBar mesh_comparison_gauge;
GLuint lipscan_color_bar_texture;
GLuint* lipscan_color_bar_word_texture = NULL;
RENDER_IMAGE_LOCATION colorbar_word_render_location;
RENDER_IMAGE_LOCATION colorbar_render_location;

const char* csv_filter;
string csv_extension;
string csv_filename;

#endif
#pragma endregion

//--- Win32 window with OpenGL setting ---
LPCSTR main_window_title = NULL;
RECT WindowRect;
WNDCLASS	wc;
DWORD		dwExStyle;
DWORD		dwStyle;
HWND hWnd = NULL;
HINSTANCE   hInstance;
HICON main_icon = NULL;
HCURSOR stand_hcur = NULL;
HGLRC hGLRC = NULL;
HDC hDC = NULL;
GLuint		PixelFormat;
LRESULT	CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
bool CreateWin32glWindow(LPCSTR);
int InitGL(GLvoid);
GLvoid ReSizeGLScene(GLsizei, GLsizei);
GLvoid KillGLWindow(GLvoid);

//--- Win32 openGL window interaction ---
bool mouseDownEvent = false;
POINT mouseDown;
POINT mouseUp;
float mouse_x, mouse_y, mouse_dx, mouse_dy;
bool MoveObj = false;
bool MouseMoveMode = false;
void PushMouse(float, float, float); //使用滑鼠平移
void RotateMouse(float, float); //使用滑鼠旋轉

//--- Export Mesh File ---
char Typename[256];
char main_name[256];
char open_model_filename[512];

//--- Information of Camera Stream ---
double fps;
double FPS();

//--- Information of 3D scan proceesing's FPS ---
UINT scan_count;
double scan_fps;
fstream cost_file;

//--- Window name for camera & scanner ---
string camera_color_wnd_name;
string camera_depth_wnd_name;
string scanner_wnd_name;
LPCSTR gl_wnd_name;
LPCSTR gl_wnd_name_cmpmesh;

//--- Win32 basic process ---
char* initalCurrentWorkDirectory(char * directory);
void terminateCurrentProcess();
HANDLE threadStart(HANDLE threadHandle, LPTHREAD_START_ROUTINE thread_function);


int ConsoleAppScript(int argc, char** argv)
{
#pragma region Check Device Connection
	if (FirstRun) {
		initalCurrentWorkDirectory(RootDirectory);
		FirstRun = false;
	}
	bScanFinish = false;


	gCamera = new lips::Camera(camera_slot, camera_devicename);

	if (camera_slot == -1)
	{
		wstring title;
		wstring text;

		title = L"Error";
		text = L"can't found camera";
		MessageBoxW(NULL, text.c_str(), title.c_str(), MB_OK);
		terminateCurrentProcess();
	}
	delete gCamera;
	gCamera = NULL;
#pragma endregion

#pragma region Print Device Information
	std::cout << "-------LIPScan 3D SDK console application--------" << endl;
	std::cout << "Device Name : " << camera_devicename << endl;
#pragma endregion

#pragma region GUI for Camera and Scanner
#ifdef VIEWER_CAMERA
	camera_color_wnd_name = "Camera Color Frame";
	camera_depth_wnd_name = "Camera Depth Frame";

	//cvNamedWindow(camera_color_wnd_name.c_str(), CV_WINDOW_AUTOSIZE);
	//cvNamedWindow(camera_depth_wnd_name.c_str(), CV_WINDOW_AUTOSIZE);
#endif
#ifdef VIEWER_3DSCAN

	scanner_wnd_name = "3D Reconstruction";
	gl_wnd_name = "LIPScan3D 3D viewer";

	//cvNamedWindow(scanner_wnd_name.c_str(), CV_WINDOW_AUTOSIZE);
#endif
#pragma endregion

#pragma region Choose Operation Mode
	do {
		std::cout << "Choose program mode by key-in corresponding number and press 'Enter'." << endl;
		std::cout << "0 : 3D Scan" << endl;
		std::cout << "1 : Mesh Comparison" << endl;
		std::cout << "3 : Offline 3D Reconstruction" << endl;
		//std::cout << "2 : Automatic cycling 3D Scan" << endl;
		std::cin >> process_mode;
	} while ( (process_mode!=0) && (process_mode!=1) && (process_mode!=3));

#pragma endregion

#pragma region 3D Scan Opearation
	if (process_mode == 0 || process_mode == 2)
	{
		std::cout << "--- Start 3D Scan Operation ---" << endl;
#pragma region Read Realsense Configuration List
		int scanner_resolution_idx = 0; // 1282x720 = 0, 640x480 = 1

		switch (process_mode)
		{
		case 0:
#pragma region Manual Set Camera Resolution
			//--- Choose Camera Resolution
			do {
				std::cout << "Chooose Camera resolution mode, Key in and press Enter : 1280x720 = 0, 640x480 = 1" << endl;
				std::cin >> scanner_resolution_idx;
				if (scanner_resolution_idx == 0)
				{
					camera_resolution = "1280x720";

					//Message Hint for HD resolution 3D scan
					std::string title = "HD resolution 3D scan";
					std::string msg = "Please slow down scannig speed in HD resolution. Recommanded 40 seconds per revolution.";
					MessageBox(NULL, msg.c_str(), title.c_str(),  MB_OK);
				}
				else if (scanner_resolution_idx == 1)
					camera_resolution = "640x480";
				else
					std::cout << "Out of range, please choose 0,1!" << endl;

			} while (scanner_resolution_idx < 0 || scanner_resolution_idx > 1);
#pragma endregion
			break;

		case 2:
			//--- Choose Camera Resolution for Automatically cycling scan 3D ---
			do {
				std::cout << "Chooose Camera resolution mode, Key in and press Enter : 1280x720 = 0, 640x480 = 1" << endl;
				std::cin >> scanner_resolution_idx;
				if (scanner_resolution_idx == 0)
					camera_resolution = "1280x720";
				else if (scanner_resolution_idx == 1)
					camera_resolution = "640x480";
				else
					std::cout << "Out of range, please choose 0,1!" << endl;

			} while (scanner_resolution_idx < 0 || scanner_resolution_idx > 1);
			
			//camera_resolution = "640x480"; //HD resolution
			std::cout << "Automatically 3D scan resolution = " << camera_resolution << endl;
			break;
		default:
			std::cout << "Unexpected situation of camera resolution selection." << endl;
			terminateCurrentProcess();
			break;
		}

		camera_cfg_map = Init3DscanConfig(RootDirectory, camera_devicename, camera_resolution.c_str());
#pragma endregion

#pragma region Initialize LIPScan3D SDK
		//--- Choose Scan 3D Range
		int scanner_rgn_idx = 1; //default 'Medium' mode

		switch (process_mode)
		{
		case 0: //
#pragma region Manually Choose 3D Scan Distance
			if (strcmp(camera_devicename, "L210")) //Not L210
			{
				do {
					std::cout << "Chooose Scan 3D range mode, Key in and press Enter : Small = 0, Medium = 1, Large = 2" << endl;
					std::cin >> scanner_rgn_idx;
					if (scanner_rgn_idx < 0 || scanner_rgn_idx > 2)
						std::cout << "Out of range, please choose 0,1,2!" << endl;
				} while (scanner_rgn_idx < 0 || scanner_rgn_idx > 2);
			}
			else //L210
			{
				do {
					std::cout << "Chooose Scan 3D range mode, Key in and press Enter : Medium = 1, Large = 2" << endl;
					std::cin >> scanner_rgn_idx;
					if (scanner_rgn_idx < 1 || scanner_rgn_idx > 2)
						std::cout << "Out of range, please choose 0,1,2!" << endl;
				} while (scanner_rgn_idx < 1 || scanner_rgn_idx > 2);
			}
#pragma endregion
			break;
		case 2:
			//--- Choose Scan 3D Range for automatically cycling scan3D ---
			if (strcmp(camera_devicename, "L210")) //Not L210
			{
				do {
					std::cout << "Chooose Scan 3D range mode, Key in and press Enter : Small = 0, Medium = 1, Large = 2" << endl;
					std::cin >> scanner_rgn_idx;
					if (scanner_rgn_idx < 0 || scanner_rgn_idx > 2)
						std::cout << "Out of range, please choose 0,1,2!" << endl;
				} while (scanner_rgn_idx < 0 || scanner_rgn_idx > 2);
			}
			else //L210
			{
				do {
					std::cout << "Chooose Scan 3D range mode, Key in and press Enter : Medium = 1, Large = 2" << endl;
					std::cin >> scanner_rgn_idx;
					if (scanner_rgn_idx < 1 || scanner_rgn_idx > 2)
						std::cout << "Out of range, please choose 0,1,2!" << endl;
				} while (scanner_rgn_idx < 1 || scanner_rgn_idx > 2);
			}
			std::cout << "Automatically 3D scan distance mode = " << scanner_rgn_idx << endl;
			break;
		default:
			std::cout << "Unexpected situation of 3D scan distance mode selection." << endl;
			terminateCurrentProcess();
			break;
		}
		gScan_range = lips::RangeParam_lib(scanner_rgn_idx);

		//--- Initialize LIPScan3D SDK
		//string scan_mode;
		char cfg_filename[512];

		switch (gScan_range)
		{
		case 0:
			gScan_mode = "small";
			break;
		case 1:
			gScan_mode = "medium";
			break;
		case 2:
			gScan_mode = "large";
			break;
		default:
			gScan_mode = "";
			break;
		}
		if (InitLIPScan3D((char*)camera_cfg_map[gScan_mode].realsense.c_str(), frame_width, frame_height))
			std::cout << "Initialize LIPScan 3D SDK successfully." << endl;
		else
		{
			std::cout << "Initialize LIPScan 3D SDK failed." << endl;
			terminateCurrentProcess();
		}
#pragma endregion

#pragma region Set3DScanQuality
		if ((frame_width == 1280 && frame_height == 720) || (frame_width == 1280 && frame_height == 800))
		{
			double volume_pitch = sqrt(0.5);
			lips::SetScannerConfigFloatMember(gReconstructor.handle_context, lips::ScannerConfigParam_FloatMember::VolumePitch, volume_pitch);
			//std::cout << "Set 3D reconstruction volume pitch = " << volume_pitch << std::endl;

			int avg_times = 3;
			lips::SetScannerConfigIntMember(gReconstructor.handle_context, lips::ScannerConfigParam_IntMember::AverageTimes, avg_times);
			//std::cout << "Set 3D reconstruction smooth mesh times = " << avg_times << std::endl;
		}

#pragma endregion

#pragma region Setting 3D Scan Time Duration
		double scan_duration;

		switch (process_mode)
		{
		case 0:
			std::cout << "Key-in 3D scan duration(seconds) and press enter : ";
			std::cin >> scan_duration;
			break;
		case 2:
			scan_duration = 60.0;
			std::cout << "Automatically 3D scan duraion : " << scan_duration << "seconds." << endl;
			break;
		default:
			std::cout << "Unexpected situation of specify 3D scan duration." << endl;
			terminateCurrentProcess();
			break;
		}
#pragma endregion

#pragma region LIPScan 3D SDK process
		double time_duration = 0; //Count real 3D scan duration(seconds) 
		BOOL done = FALSE; //Flag of terminate 3D scan reconstruction

		switch (process_mode)
		{
		case 0: //Manually 3D scan mode
#ifdef LIPSCAN3D_PROCESS
			
			double proc_head, proc_end;
			double cycle_head, cycle_end;

			MeshNum = 0; // no mesh object before finish 3D scan process
			scan_count = 0;// count scan frames for compute average FPS

		
			while (!done) // Loop That Runs Until done=TRUE
			{
				cycle_head = clock();

				if (!gCamera->GetFrame(color_map, depth_data))
					terminateCurrentProcess();

				memcpy(copy_color_map, color_map, frame_width*frame_height * 3);

#ifdef DEPTH_TO_POINT
				lips::convertDepthTo3DWorld(points_data, gCamera, frame_roi);

				bool bDepthProc = gDepthpostproc->Process(depth_data, points_data, depth_map);
#endif

#pragma region Display Camera Frame

				RotateDisplayImage((unsigned char*)color_img->imageData, frame_height, frame_width, 2);
				RotateDisplayImage((unsigned char*)depth_img->imageData, frame_height, frame_width, 2);
				cvShowImage(camera_color_wnd_name.c_str(), color_img);
				cvShowImage(camera_depth_wnd_name.c_str(), depth_img);
				cvWaitKey(1);
#pragma endregion


#ifdef DepthEvaluation
				RECT depth_roi;
				int DepthPeak;

				depth_roi.left = depth_roi.bottom = 0;
				depth_roi.right = frame_width;
				depth_roi.top = frame_height;

				if (strcmp(camera_devicename, "L210") == 0) // depth unit is 100um
					DepthPeak = gDepthpostproc->DepthRangeEvaluation(depth_data, 30, 24, 16, depth_roi, 10, 10.0);
				else //depth unit is 1mm
					DepthPeak = gDepthpostproc->DepthRangeEvaluation(depth_data, 30, 24, 16, depth_roi, 10, 1.0);
#endif			

#ifdef ICP_3D
				bool bScan3D = gReconstructor.ScanData(copy_color_map, points_data, frame_width, frame_height, scan_map);

#pragma region Display 3D Reconstruction Frame
				RotateDisplayImage((unsigned char*)scan_img->imageData, frame_height, frame_width, 2);
				cvShowImage(scanner_wnd_name.c_str(), scan_img);
				cvWaitKey(1);
#pragma endregion
#endif
				scan_count++; //count 3D Scan cycle numbers

				cycle_end = clock();
				time_duration += ((cycle_end - cycle_head) / CLOCKS_PER_SEC);

				// Export 3D Scan Mesh data to file
				if (time_duration > scan_duration)
				{
#ifdef FINISH_3DSCAN
					gHandle_obj3d = gReconstructor.FinishScan();
					if (gHandle_obj3d)
					{
#pragma region Prepare Rendering Data for 3D Viewer
						bScanFinish = true;
						MeshNum = 1;
						pObj[0] = gHandle_obj3d; //assign mesh address to container in API
						bVisibleObj[0] = true;
						PrepareTransformation(pObj, 0);
						PrepareRenderBuf(pObj[0], MeshBuf[0]); //copy triangle mesh data to buffer for opengl rendering
#pragma endregion

#pragma region Export 3D Scan Mesh File
						strcpy(Typename, "ply");
						strcpy(main_name, "LIPScan_SDK_example_scan");
						sprintf(open_model_filename, "%s\\%s.%s", RootDirectory, main_name, Typename);
						bool bExport = lips::ExportFile(open_model_filename, gHandle_obj3d, 4);

						if (bExport)
							std::cout << "Finish export mesh file!" << endl;
						else
							std::cout << "License error, unsupported function." << endl;
#pragma endregion
					}
					else {
						bScanFinish = false; //FinishScan() costruct mesh object failure
						std::cout << "Reconstrcut 3D mesh object failure, please check the camera and object setting and repeat operation again." << endl;
					}
#endif
					done = true;
					std::cout << "3D Scan times up : " << time_duration << " Seconds " << endl;

					scan_fps = scan_count / time_duration;
					std::cout << "Average FPS = " << scan_fps << endl;
				}
			}
#endif
			break;
		case 2: //Automatically 3D scan mode
			std::cout << "Automatically cycling 3D scan process will trigger when 3D viewer alive." << endl;
			break;
		default:
			std::cout << "Unexpected situation when execute 3D scan process." << endl;
			terminateCurrentProcess();
			break;
		}

#pragma endregion
		std::cout << "--- End 3D Scan Operation ---" << endl;
	}
#pragma endregion

#pragma region Mesh Comparison Operation
	else if(process_mode == 1)
	{
		std::cout << "--- Start Mesh Comparison Operation ---" << endl;

		//Initialize LIPScan3D SDK with default settiing
		camera_resolution = "640x480";  //set camera resolution as VGA
		camera_cfg_map = Init3DscanConfig(RootDirectory, camera_devicename, camera_resolution.c_str());
		gScan_range = lips::RangeParam_lib(1);  //set scan range as medium
		if (InitLIPScan3D((char*)camera_cfg_map["medium"].realsense.c_str(), frame_width, frame_height))
			std::cout << "Initialize LIPScan 3D SDK successfully." << endl;
		else
		{
			std::cout << "Initialize LIPScan 3D SDK failed." << endl;
			terminateCurrentProcess();
		}

		compare_mesh_tool.Init(gCamera->GetSerialNumber());


		MeshNum = 10; // no mesh object before mesh comparison process
		std::cout << "--- End Mesh Comparison Operation ---" << endl;
	}
#pragma endregion

#pragma region Offline 3D Reconstruction
	else if (process_mode == 3)
	{
		//--- 3D reconstruction from camera binary frame files ---

		char cfg_path[512];
		sprintf_s(cfg_path, 512, "%s\\offline3dscan.json", RootDirectory);
		string cfg_str(cfg_path);
		
		fstream tmp_file;
		tmp_file.open(cfg_path);
		if (!tmp_file.is_open()) {
			std::cout << "Open file failure! Please check file path!" << endl;
		}
		lips::JsonParser* jsonParser = new lips::JsonParser();
		jsonParser->jsontree = jsonParser->Read(cfg_str);

		string path = jsonParser->jsontree.get<string>("dir");
		int src_w = jsonParser->jsontree.get<int>("width");
		int src_h = jsonParser->jsontree.get<int>("height");
		int src_count = jsonParser->jsontree.get<int>("frame");

		char src_path[512];
		char dst_mesh_path[512];

		sprintf_s(src_path, 512, "%s\\%s", RootDirectory, path.c_str());
		sprintf_s(dst_mesh_path, 512, "%s\\Offline_3DReconstruction.ply", src_path);

		if (gHandle_obj3d) {
			ReleaseTriMeshModel(gHandle_obj3d);
			gHandle_obj3d = NULL;
		}
		 gHandle_obj3d = Scan3D_Offline_Process(src_path, src_w, src_h, src_count);
			
		 if (gHandle_obj3d)
		 {
#pragma region Prepare Rendering Data for 3D Viewer
			 bScanFinish = true;
			 MeshNum = 1;
			 pObj[0] = gHandle_obj3d; //assign mesh address to container in API
			 bVisibleObj[0] = true;
			 PrepareTransformation(pObj, 0);
			 PrepareRenderBuf(pObj[0], MeshBuf[0]); //copy triangle mesh data to buffer for opengl rendering
#pragma endregion

#pragma region Export 3D Scan Mesh File
			 bool bExport = lips::ExportFile(dst_mesh_path, gHandle_obj3d, 4);

			 if (bExport) {
				 std::cout << "Finish export mesh file! filename:" << endl;
				 std::cout << dst_mesh_path << endl;
			 }
			 else
				 std::cout << "License error, unsupported function." << endl;
#pragma endregion
			 frame_width = src_w;
			 frame_height = src_h;
		 }
		 else {
			 bScanFinish = false; //FinishScan() costruct mesh object failure
			 std::cout << "Reconstrcut 3D mesh object failure, please check the configurations in 'offline3dscan.json' file and source file(.bin)." << endl;
		 }
	//------------------------------------
	}
#pragma endregion

#pragma region Failure Of Operation Selection
	else
	{
		std::cout << "--- Failure Of Operation Selection ---" << endl;
	}
#pragma endregion

#pragma region Launch 3D Viewer

#ifdef VIEWER_3DSCAN

	std::cout << "Launch 3D Viewer window." << endl;
	std::cout << "If user want to repeat 3D Scan process, press 'Esc' button on 3D Viewer window." << endl;

	CreateWin32glWindow(gl_wnd_name);
	MSG	msg;
	BOOL done_viewer = FALSE;
	while (!done_viewer)
	{
#pragma region Start Thread Of Automatically Cycling Scan3D
		if ( (process_mode) == 2 && (auto_cycling_scan3D_thread_handle == NULL) )
			auto_cycling_scan3D_thread_handle = threadStart(auto_cycling_scan3D_thread_handle, (LPTHREAD_START_ROUTINE)auto_cycling_scan3D_ThreadProc);
#pragma endregion

		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_KEYDOWN && msg.wParam == VK_ESCAPE)
			{
#pragma region Terminate Thread Of Automatically Cycling Scan3D
				if ((process_mode == 2) && (auto_cycling_scan3D_thread_handle != NULL))
				{
					TerminateThread(auto_cycling_scan3D_thread_handle, 0); // Dangerous source of errors!
					CloseHandle(auto_cycling_scan3D_thread_handle);
					auto_cycling_scan3D_thread_handle = NULL;
				}
#pragma endregion

				done_viewer = TRUE;
			}
			else
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else
		{
			SendMessage(hWnd, WM_PAINT, 0, 0);
		}

		DrawGLScene();
		SwapBuffers(hDC);

#ifdef VIEWER_FPS
		double fps = FPS();
		std::cout << "3D Render FPS = " << fps << endl;
#endif
	}
#pragma endregion

#pragma region Release Rendering Data Of Mesh Object
	switch (process_mode)
	{
	case 0: //Manually 3D Scan 
		cvDestroyWindow(camera_color_wnd_name.c_str());
		cvDestroyWindow(camera_depth_wnd_name.c_str());
		cvDestroyWindow(scanner_wnd_name.c_str());
		KillGLWindow();

		ReleaseRenderBuf(MeshBuf[0]);
		ReleaseLIPScan3D();
		MeshNum = 0;
		
		std::cout << "Release resources of manually 3D scan process." << endl;
		break;

	case 1: //Mesh Comparison
		KillGLWindow();

		meshcomparer_status = MeshComparisonState::NONE;
		compare_mesh_tool.Reset();
		if (MeshBuf[0].TriNum > 0) ReleaseRenderBuf(MeshBuf[0]); //clear render buffer of compared mesh
		if (MeshBuf[1].TriNum > 0) ReleaseRenderBuf(MeshBuf[1]); //clear render buffer of referenced mesh
		ReleaseLIPScan3D();
		
		pObj[0] = NULL; //memeries released by lips::MeshComparison::Reset()
		pObj[1] = NULL; //memeries released by lips::MeshComparison::Reset()
		RenderMode = 0; //

		std::cout << "Release resources of mesh comparison." << endl;
		break;

	case 2: //Automatically 3D scan mode
		cvDestroyWindow(camera_color_wnd_name.c_str());
		cvDestroyWindow(camera_depth_wnd_name.c_str());
		cvDestroyWindow(scanner_wnd_name.c_str());
		KillGLWindow();
		Sleep(100);
		ReleaseRenderBuf(MeshBuf[0]);
		MeshNum = 0;

		std::cout << "Release resources of automatically cycling 3D scan process." << endl;
		break;

	case 3: //Offline 3D reconstruction

		KillGLWindow();
		Sleep(100);
		ReleaseRenderBuf(MeshBuf[0]);
		MeshNum = 0;

		std::cout << "Release resources of offline 3D reconstruction process." << endl;
		break;

	default:
		std::cout << "Exit unknown process mode." << endl;
		break;
	}
	std::cout << "Destroy all window." << endl;
#pragma endregion
#endif

#define REPEAT_SCAN3D
#ifdef REPEAT_SCAN3D
	char ch;
	//std::cout << "Repeat 3D scan agian? (Y/y)" << endl;
	std::cout << "Repeat LIPScan3D SDK example operation again? (Y/y)" << endl;
	std::cin >> ch;

	if (ch == 'Y' || ch == 'y')
		return ConsoleAppScript(argc, argv);
#endif


}


char* initalCurrentWorkDirectory(char * directory)
{
	GetModuleFileName(NULL, directory, 512);
	PathRemoveFileSpec(directory);

	if (strstr(directory, "WindowsApps") != NULL)
	{
		SetCurrentDirectory(directory);
		GetCurrentDirectory(512, directory);
	}
	else
	{
		GetCurrentDirectory(512, directory);
	}
	std::cout << "dirname: " << directory << std::endl;
	return directory;
}
void terminateCurrentProcess()
{
	DWORD pid = GetCurrentProcessId();
	HANDLE handle;
	handle = OpenProcess(SYNCHRONIZE | PROCESS_TERMINATE, TRUE, pid);
	TerminateProcess(handle, 0);
}
HANDLE threadStart(HANDLE threadHandle, LPTHREAD_START_ROUTINE thread_function)
{
	threadHandle = CreateThread(NULL, NULL, thread_function, NULL, NULL, NULL);
	return threadHandle;
}

SIZE_MODE_CONFIG_MAP Init3DscanConfig(const char* rootdir, const char* devicename, const char* resolution)
{
	SIZE_MODE_CONFIG_MAP config_map;
	string scan3dconfig_dir = rootdir;
	scan3dconfig_dir += "\\Config_files\\";
	scan3dconfig_dir += devicename;
	scan3dconfig_dir += "\\";
	scan3dconfig_dir += resolution;

	string scan3d_config_list_filename = scan3dconfig_dir + "\\scan3d_config.txt";
	string config_json_filename;

	fstream scan3d_config_list;
	scan3d_config_list.open(scan3d_config_list_filename.c_str());
	if (scan3d_config_list.is_open())
	{
		string config_mode;
		SIZE_MODE_CONFIG config_info;
		while (!scan3d_config_list.eof())
		{
			scan3d_config_list >> config_mode;
			scan3d_config_list >> config_info.realsense;

			config_json_filename = scan3dconfig_dir + "\\";
			config_json_filename += config_info.realsense;

			config_info.realsense = config_json_filename;
			config_map.insert(std::pair<string, SIZE_MODE_CONFIG>(config_mode, config_info));
		}
	}
	else
		MessageBox(NULL, "Could Not open scan3d_config.txt.", "Init3DscanConfig error", MB_OK | MB_ICONINFORMATION);

	return config_map;
}

void InitScan3DCfg(const char* rootdir, const char* devicename, Scan3dCfgMap& cfgmap)
{
	string scan3dconfig_dir = rootdir;
	scan3dconfig_dir += "\\Config_files\\";
	scan3dconfig_dir += devicename;

	string scan3d_config_group_filename = scan3dconfig_dir + "\\scan3d_config_group.txt";
	string config_json_filename;
	fstream scan3d_config_file;

	int mode;
	char line[512];

	scan3d_config_file.open(scan3d_config_group_filename.c_str());
	if (scan3d_config_file.is_open())
	{
		std::cout << "Read LIPScan 3D Realsense configuration files:" << endl;
		string config_filename;

		while (!scan3d_config_file.eof())
		{
			scan3d_config_file.getline(line, sizeof(line));

			string text = line;
			string space_delimiter = " ";
			vector<string> words{};
			size_t pos = 0;
			while ((pos = text.find(space_delimiter)) != string::npos) {
				words.push_back(text.substr(0, pos));
				text.erase(0, pos + space_delimiter.length());
			}
			words.push_back(text); //last config filename in line

			lips::RangeParam_lib range = (lips::RangeParam_lib)std::stoi(words[0]);
			RealsenseCfgVector config_vec;
			for (int i = 1; i < words.size(); i++)
				config_vec.push_back(words[i]);

			cfgmap.insert(std::pair<lips::RangeParam_lib, RealsenseCfgVector>(range, config_vec));
		}
		scan3d_config_file.close();
	}
	//end

	//print config_group.txt information
	Scan3dCfgMap::iterator it = cfgmap.begin();
	while (it != cfgmap.end())
	{
		std::cout << "Scan Range Index:" << it->first << std::endl;
		for (int j = 0; j < it->second.size(); j++)
			std::cout << "[" << it->second[j] << "]" << std::endl;
		it++;
	}
	std::cout << std::endl;
	//
}

bool InitLIPScan3D(char* cfg_filename, int& width, int& height)
{
	bool bInit = false;

#pragma region Camera Initialization
	string str_cfg_filename(cfg_filename);
	lips::JsonParser* jsonParser = new lips::JsonParser();
	jsonParser->jsontree = jsonParser->Read(str_cfg_filename);
	gCamera = new lips::Camera(camera_slot, camera_devicename);
	bInit = gCamera->InitCamera(str_cfg_filename.c_str());
	if (!bInit)
	{
		std::cout << "Load configuration file failed! Please check the file content." << endl;
		return false;
	}

	std::cout << "Load configuration file:" << str_cfg_filename << endl;

	int stream_width = jsonParser->jsontree.get<int>("stream-width");
	int stream_height = jsonParser->jsontree.get<int>("stream-height");
	width = stream_width;
	height = stream_height;
	std::cout << "Camera frame : " << endl;
	std::cout << "Width  = " << stream_width << endl;
	std::cout << "Height = " << stream_height << endl;

	cv_frame_roi = cv::Rect(0, 0, stream_width, stream_height);

	frame_roi.x = cv_frame_roi.x;
	frame_roi.y = cv_frame_roi.y;
	frame_roi.width = cv_frame_roi.width;
	frame_roi.height = cv_frame_roi.height;
#pragma endregion

#pragma region ScannerProcess Initialization
	//Initialize lipscan3d API ScannerProcess object by device type and default scan mode
	const char* SN = gCamera->GetSerialNumber();

	//int iRangeIndex = 1; //small:0, medium:1, large:2
	switch (hash_(camera_devicename))
	{
	case hash_compile_time("Intel RealSense D415"):
		bInit = gReconstructor.Init(stream_width, stream_height, lips::ScannerDeviceType::IntelRealsense_D415, gScan_range, SN);
		break;
	case hash_compile_time("Intel RealSense D435"):
		bInit = gReconstructor.Init(stream_width, stream_height, lips::ScannerDeviceType::IntelRealsense_D435, gScan_range, SN);
		break;
	case hash_compile_time("Intel RealSense D435I"):
		bInit = gReconstructor.Init(stream_width, stream_height, lips::ScannerDeviceType::IntelRealsense_D435I, gScan_range, SN);
		break;
	case hash_compile_time("Intel RealSense D455"):
		bInit = gReconstructor.Init(stream_width, stream_height, lips::ScannerDeviceType::IntelRealsense_D455, gScan_range, SN);
		break;
	case hash_compile_time("L210"):
		bInit = gReconstructor.Init(stream_width, stream_height, lips::ScannerDeviceType::LIPS_L210, gScan_range, SN);
		break;
	default:
		bInit = false;
		MessageBoxA(NULL, camera_devicename, "ScannerProcess::Init(), unsupported device", MB_OK);
		return bInit;
		break;
	}
#pragma endregion

#pragma region DepthmapProcess Initialization
	points_data = new float[stream_width * stream_height * 3];
	gDepthpostproc = new lips::DepthmapProcess(stream_width, stream_height, gReconstructor.handle_context);
#pragma endregion

#pragma region Displayer Initialization
	color_img = cvCreateImage(cvSize(stream_width, stream_height), 8, 3);
	depth_output = cvCreateImage(cvSize(stream_width, stream_height), IPL_DEPTH_16U, 1);
	depth_img = cvCreateImage(cvSize(stream_width, stream_height), 8, 3);
	scan_img = cvCreateImage(cvSize(stream_width, stream_height), 8, 3); //size must equal to Camera & ScannerProcess

	cvSetZero(depth_output);
	cvSetZero(color_img);
	cvSetZero(depth_img);
	cvSetZero(scan_img);

	color_map = (unsigned char*)color_img->imageData;
	depth_data = (unsigned short*)depth_output->imageData;
	depth_map = (unsigned char*)depth_img->imageData;
	scan_map = (unsigned char*)scan_img->imageData;

	copy_color_map = new unsigned char[frame_width * frame_height * 3];
#pragma endregion

	return bInit;
}

void ReleaseLIPScan3D(void)
{
	cvReleaseImage(&color_img);
	cvReleaseImage(&depth_output);
	cvReleaseImage(&depth_img);
	cvReleaseImage(&scan_img);

	if (copy_color_map != NULL)
	{
		delete copy_color_map;
		copy_color_map = NULL;
	}
	if (points_data != NULL)
	{
		delete[] points_data;
		points_data = NULL;
	}
	if (gCamera != NULL)
	{
		delete gCamera;
		gCamera = NULL;
	}
	if (gDepthpostproc != NULL)
	{
		delete gDepthpostproc;
		gDepthpostproc = NULL;
	}

	if (bScanFinish) //Construct mesh object successfully
	{
		bool bDeleteMesh = lips::ReleaseTriMeshModel(gHandle_obj3d);
		if (bDeleteMesh)
			std::cout << "Release Triangle Mesh memory successfully." << endl;
		else
			std::cout << "Release Triangle Mesh memory failed!" << endl;
	}

}

void RotateDisplayImage(unsigned char* src_Ptr, int img_height, int img_width, int mode) {

	cv::Mat tmp(img_height, img_width, CV_8UC3, src_Ptr);
	cv::Mat result(img_height, img_width, CV_8UC3);

	//設定旋轉中心、旋轉角度和縮放倍率
	cv::Point center(img_width / 2, img_height / 2);
	double angle = 0;
	double scale = 1;

	switch (mode) {
	case 1: {
		angle = 0;
		scale = 1.0 * img_width / img_width;
		break;
	}
	case 2: {
		angle = -90.0;
		scale = 1.0 * img_height / img_width;
		break;
	}
	case 3: {
		angle = 180.0;
		scale = 1.0 * img_width / img_width;
		break;
	}
	case 4: {
		angle = 90.0;
		scale = 1.0 * img_height / img_width;
		break;
	}
	default: {
		angle = 0;
		scale = 1.0 * img_width / img_width;
		break;
	}
	}

	cv::Mat rot_mat = getRotationMatrix2D(center, angle, scale);
	warpAffine(tmp, result, rot_mat, result.size());

	cv::Rect roi((img_width - img_width) / 2, (img_height - img_height) / 2, img_width, img_height);
	cv::Mat crop = result(roi);
	cv::Mat output = crop.clone();
	memcpy(src_Ptr, output.data, sizeof(unsigned char) * img_width * img_height * 3);
	return;
}


bool dirExists(const std::string& path)
{
	DWORD ftyp = GetFileAttributesA(path.c_str());
	if (ftyp == INVALID_FILE_ATTRIBUTES)
		return false;  //something is wrong with your path!

	if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
		return true;   // this is a directory!

	return false;    // this is not a directory!
}

void SaveCameraScannerFrames(void)
{
	//Creating File
	char str[128];
	char dir[128];
	sprintf_s(dir, 128, "%s\\tmp", RootDirectory);
	if (!dirExists(dir))
	{
		if (_mkdir(dir) == -1)
			std::cerr << "Error : " << strerror(errno) << endl;
		else
			std::cout << "Directory Created" << endl;
	}
	FrameCount++;
	sprintf_s(str, 128, "%s\\depth%02d.jpg", dir, FrameCount);
	cvSaveImage(str, depth_img);

	sprintf_s(str, 128, "%s\\color%02d.jpg", dir, FrameCount);
	cvSaveImage(str, color_img);

	sprintf_s(str, 128, "%s\\3dscan%02d.jpg", dir, FrameCount);
	cvSaveImage(str, scan_img);
}

void ImportCameraRawData(char* color_fname, char* depth_fname, char* cloud_fname, unsigned char* color_buf, unsigned short* depth_buf, float* cloud_buf, int width, int height)
{
	int depth_bytes = sizeof(unsigned short)*width*height;
	int color_bytes = sizeof(unsigned char)*width*height*3;
	int cloud_bytes = sizeof(float)*width*height * 3;

	//Read depth and color frame raw data from binary file
	auto depth_file = std::fstream(depth_fname, std::ios::in | std::ios::binary);
	if (depth_file.is_open())
		std::cout << "Read depth frame binary file : "<< depth_fname << ", done."<<endl;
	else
		std::cout << "Read depth frame binary file : "<< depth_fname << ", fail."<<endl;

	depth_file.read((char*)depth_buf, depth_bytes); //depth frame buffer
	depth_file.close();


	auto color_file = std::fstream(color_fname, std::ios::in | std::ios::binary);
	if (color_file.is_open())
		std::cout << "Read color frame binary file : " << color_fname << ", done." << endl;
	else
		std::cout << "Read color frame binary file : " << color_fname << ", fail." << endl;

	color_file.read((char*)color_buf, color_bytes); //color frame buffer
	color_file.close();


	auto cloud_file = std::fstream(cloud_fname, std::ios::in | std::ios::binary);
	if (cloud_file.is_open())
		std::cout << "Read cloud frame binary file : " << cloud_fname << ", done." << endl;
	else
		std::cout << "Read cloud frame binary file : " << cloud_fname << ", fail." << endl;

	cloud_file.read((char*)cloud_buf, cloud_bytes); //color frame buffer
	cloud_file.close();
}

void InitDepthToColorTable(void)
{
	int i;
	for (i = 0; i < 32; i++) ColorTable[i] = (i << 8) | (8 + 6 * i);
	for (i = 32; i < 64; i++) ColorTable[i] = ((32 + 4 * (i - 32)) << 8) | (200 - (i - 32));
	for (i = 64; i < 96; i++) ColorTable[i] = ((160 + 2 * (i - 64)) << 8) | (168 - 3 * (i - 64));
	for (i = 96; i < 128; i++) ColorTable[i] = ((8 * (i - 96)) << 16) | ((224 + (i - 96)) << 8) | (70 - 2 * (i - 96));
	for (i = 128; i < 160; i++) ColorTable[i] = (255 << 16) | ((255 - 4 * (i - 128)) << 8);
	for (i = 160; i < 192; i++) ColorTable[i] = (255 << 16) | ((124 - 4 * (i - 160)) << 8) | (2 * (i - 160));
	for (i = 192; i < 224; i++) ColorTable[i] = (255 << 16) | ((2 * (i - 192)) << 8) | (64 + 6 * (i - 192));
	for (i = 224; i < 256; i++) ColorTable[i] = (255 << 16) | ((63 + 6 * (i - 224)) << 8) | 255;
}

void DepthToColor(unsigned short* depthBuf, unsigned char* rgbBuf, int Width, int Height)
{
	//Calculate Max and Min
	int depth_min = 65535;
	int depth_max = 0;
	int Size = Width * Height;
	for (int j = 0; j < Height; j++)
	{
		for (int i = 0; i < Width; i++)
		{
			unsigned short val;
			//val = depthMat.at<cv::Vec2b>(j, i)[0];
			val = depthBuf[i + ( j  *Width )];
			if (val > depth_max)
				depth_max = val;
			if (val < depth_min)
				depth_min = val;
		}
	}

	//Depth to Color
	int x, y, img_idx = 0, color_idx = 0;
	int val;
	for (y = 0; y < Height; ++y)
	{
		for (x = 0; x < Width; ++x)
		{
			unsigned short val;
			//val = depthMat.at<cv::Vec2b>(y, x)[0];
			val = depthBuf[x + (y * Width)];

			if (val != 0)
			{
				val = 255 - (int)((((255.0 - 0)*(val - depth_min)) / (depth_max - depth_min)) + 0);
			}
			if (val < 0) { val = 0; }
			if (val > 255) { val = 255; }

		
			rgbBuf[(x + (y*Width)) * 3 + 0] = ColorTable[val] % 256;
			rgbBuf[(x + (y*Width)) * 3 + 1] = (ColorTable[val] >> 8) % 256;
			rgbBuf[(x + (y*Width)) * 3 + 2] = (ColorTable[val] >> 16) % 256;
		}
	}
	//
}

HandleTriMeshModel Scan3D_Offline_Process(char* dir, int w, int h, int frame_count)
{

	cv::Mat ColorBin(h, w, CV_8UC3);
	cv::Mat DepthBin(h, w, CV_16U);
	cv::Mat DepthClr(h, w, CV_8UC3);
	cv::Mat CloudBin(h, w, CV_32FC3);
	cv::Mat ScanImag(h, w, CV_8UC3);

	char depth_fname[512]; //depth frame bin file name
	char color_fname[512]; //color frame bin file name
	char cloud_fname[512]; //point cloud bin file name
	
	if (!dirExists(dir))
		return false;

	////Initialize lipscan3d API ScannerProcess object by device type and default scan mode
	gCamera = new lips::Camera(camera_slot, camera_devicename);
	camera_cfg_map = Init3DscanConfig(RootDirectory, camera_devicename, camera_resolution.c_str());

	if (gCamera->InitCamera((char*)camera_cfg_map[gScan_mode].realsense.c_str()) == FALSE)
	{
		MessageBoxA(NULL, "Failed to connect camera! Please check that 'camera.json' matches the camera type.", "Offline 3D Scan", MB_OK);
		return NULL;
	}

	const char* SN = gCamera->GetSerialNumber();
	bool bInit;
	gScan_range = lips::RangeParam_lib::RangeParam_lib_MEDIUM; //medium mode
	//int iRangeIndex = 1; //small:0, medium:1, large:2
	switch (hash_(camera_devicename))
	{
	case hash_compile_time("Intel RealSense D415"):
		bInit = gReconstructor.Init(w, h, lips::ScannerDeviceType::IntelRealsense_D415, gScan_range, SN);
		break;
	case hash_compile_time("Intel RealSense D435"):
		bInit = gReconstructor.Init(w, h, lips::ScannerDeviceType::IntelRealsense_D435, gScan_range, SN);
		break;
	case hash_compile_time("Intel RealSense D435I"):
		bInit = gReconstructor.Init(w, h, lips::ScannerDeviceType::IntelRealsense_D435I, gScan_range, SN);
		break;
	case hash_compile_time("Intel RealSense D455"):
		bInit = gReconstructor.Init(w, h, lips::ScannerDeviceType::IntelRealsense_D455, gScan_range, SN);
		break;
	case hash_compile_time("L210"):
		bInit = gReconstructor.Init(w, h, lips::ScannerDeviceType::LIPS_L210, gScan_range, SN); 
		break;
	default:
		bInit = false;
		MessageBoxA(NULL, camera_devicename, "ScannerProcess::Init(), unsupported device", MB_OK);
		break;
	}

	//Set volume pitch for HD resolution 3D scan
	if (w == 1280) {
		double volume_pitch;// = sqrt(0.5);
		double ori_vol_pitch;
		ori_vol_pitch = lips::GetScannerConfigFloatMember(gReconstructor.handle_context, lips::ScannerConfigParam_FloatMember::VolumePitch);
		volume_pitch = ori_vol_pitch * sqrt(0.5);
		lips::SetScannerConfigFloatMember(gReconstructor.handle_context, lips::ScannerConfigParam_FloatMember::VolumePitch, volume_pitch);

		std::cout << "Original Volume Pitch = " << ori_vol_pitch << endl;
		std::cout << "Set Volume Pitch = " << volume_pitch << endl;
	}

	if (gCamera != NULL)
	{
		delete gCamera;
		gCamera = NULL;
	}
	//End of LIPScan3D SDK Initialization

	for (int i = 0; i <= frame_count; i++)
	{
		sprintf_s(depth_fname, 512, "%s\\depth%03d.bin", dir, i);
		sprintf_s(color_fname, 512, "%s\\color%03d.bin", dir, i);
		sprintf_s(cloud_fname, 512, "%s\\cloud%03d.bin", dir, i);

		ImportCameraRawData(color_fname, depth_fname, cloud_fname, (unsigned char*)ColorBin.data, (unsigned short*)DepthBin.data, (float*)CloudBin.data, w, h);

		gReconstructor.ScanData((unsigned char*)ColorBin.data, (float*)CloudBin.data, w, h, (unsigned char*)ScanImag.data);

		//--- Display progress by opencv
		cv::Mat progess_img(256, 640, CV_8UC3);
		progess_img.setTo(cv::Scalar(255, 128, 128));

		// set string and variables
		float progress_ratio = 100 * (i + 1) / (float)frame_count;
		int progress_ratio_int = (int)(progress_ratio + 0.5);
		std::string text = std::to_string(progress_ratio_int);
		text.append("%");
		int font_face = cv::FONT_HERSHEY_SIMPLEX;
		double font_scale = 9;
		int thickness = 20;
		int baseline;

		// get text size
		cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

		// decide text position
		cv::Point origin;
		origin.x = 24;
		origin.y = 232;
		cv::putText(progess_img, text, origin, font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 8, 0);
		cv::imshow("Offline Scan3D Progress", progess_img);
		cv::waitKey(1);
	}
	cv::destroyWindow("Offline Scan3D Progress");

	lips::HandleTriMeshModel hmesh = gReconstructor.FinishScan();

	char mesh_fname[512];
	sprintf_s(mesh_fname, 512, "%s\\Offline_3D_reconstruction.ply", dir);
	if (ExportFile(mesh_fname, hmesh, 2))
		std::cout << "Export full 3D scan mesh to PLY file : " << mesh_fname << endl;
	else
		std::cout << "Export full 3D scan mesh fail." << endl;

	return hmesh;
}

DWORD WINAPI auto_cycling_scan3D_ThreadProc()try
{
	trigger_count = 0;
	trigger_scan_3D = false;
	finish_scan_3D = false;
	
	while (true)
	{
#pragma region Get Local Time
		time_t tNow = time(0);
		tm *ltm = localtime(&tNow);
		if (ltm->tm_min % 5 == 0 && !trigger_scan_3D)
		{
			trigger_count++;
			trigger_scan_3D = true;
			scan_3D_duration = 0.0;

			sprintf_s(localtime_str, "%d-%d-%d-%d-%d-%d", ltm->tm_year+1900, ltm->tm_mon+1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
			std::cout << "Trigger count : "<< trigger_count <<" ,Automatically scan 3D process at time : " << localtime_str << endl;
		}
#pragma endregion

#pragma region Camera Streaming
		if (!gCamera->GetFrame(color_map, depth_data))
			terminateCurrentProcess();

		memcpy(copy_color_map, color_map, frame_width*frame_height * 3);
#pragma endregion

#pragma region Display Camera Image After Trigger Scan 3D
		RotateDisplayImage((unsigned char*)color_img->imageData, frame_height, frame_width, 2);
		RotateDisplayImage((unsigned char*)depth_img->imageData, frame_height, frame_width, 2);
		cvShowImage(camera_color_wnd_name.c_str(), color_img);
		cvShowImage(camera_depth_wnd_name.c_str(), depth_img);
		cvWaitKey(1);
#pragma endregion

#pragma region Depth Image Process
		lips::convertDepthTo3DWorld(points_data, gCamera, frame_roi);

		bool bDepthProc = gDepthpostproc->Process(depth_data, points_data, depth_map);
#pragma endregion

#pragma region Trigger 3D Scan Reconstruction
		if (trigger_scan_3D)
		{
			clock_t proc_head = clock();

			bool bScan3D = gReconstructor.ScanData(copy_color_map, points_data, frame_width, frame_height, scan_map);

			clock_t proc_end = clock();
			scan_3D_duration += double(proc_end - proc_head) / CLOCKS_PER_SEC;
			if (scan_3D_duration > 60.0)
			{
				trigger_scan_3D = false;
				finish_scan_3D = true;

				cvDestroyWindow(scanner_wnd_name.c_str()); //Disable 3D reconstruction image window
			}
		}
#pragma endregion

#pragma region Display 3D Reconstruction Image After Trigger Scan 3D
		if (trigger_scan_3D) 
		{
			RotateDisplayImage((unsigned char*)scan_img->imageData, frame_height, frame_width, 2);
			cvShowImage(scanner_wnd_name.c_str(), scan_img);
			cvWaitKey(1);
		}
#pragma endregion

#pragma region Reconstruct Mesh Object
		if (finish_scan_3D)
		{
			if (MeshBuf[0].TriNum > 0) //Release Render buffer of current 3D viewer
			{
				MeshNum = 0;
				Sleep(100); //block opengl rendering function, avoid memory violation, it works.
				ReleaseRenderBuf(MeshBuf[0]); //during above blocking, release render data of opengl graphic function.
			}

			gHandle_obj3d = gReconstructor.FinishScan();
			if (gHandle_obj3d)
			{
				pObj[0] = gHandle_obj3d; //assign mesh address to container in API
				bVisibleObj[0] = true;
				PrepareTransformation(pObj, 0);
				PrepareRenderBuf(pObj[0], MeshBuf[0]); //copy triangle mesh data to buffer for opengl rendering
				MeshNum = 1;

				std::cout << "Finish automatically cycling scan 3D."<<endl;
				std::cout << "Triangle Face Num = " << MeshBuf[0].TriNum << endl;
			}
			else {
				MeshNum = 0;

				std::cout << "Reconstrcut 3D mesh object failure, please check the camera and object setting." << endl;
			}
		}
#pragma endregion

#pragma region Export Mesh File With Deafault Name
		if (finish_scan_3D) 
		{
			strcpy(Typename, "ply");
			sprintf(open_model_filename, "%s\\tmp\\%s.%s", RootDirectory, localtime_str/*main_name*/, Typename);
			bool bExport = lips::ExportFile(open_model_filename, gHandle_obj3d, 4);

			if (bExport)
				std::cout << "Automatically 3D scan finish and export mesh file : " << open_model_filename << endl;

			else
				std::cout << "License error, unsopported function." << endl;

			finish_scan_3D = false;
		}
#pragma endregion
	}

	return 0;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	getchar();
	//return EXIT_FAILURE;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM	lParam)
{
	clock_t startTime, stopTime;
	double duration;
	int tmp = 0;

#ifdef LIPS_CLOUDCOMPARE
	int mx, my;
	int triangle_index;
	int objectIndex, selected_trangle_index, SelectEdge;
	bool hit_trangle_on_mouse_down;
	
#endif

	switch (msg)
	{
	case WM_SIZE:
		iWidth = LOWORD(lParam);
		iHeight = HIWORD(lParam);
		ReSizeGLScene(iWidth, iHeight);
		break;
	case WM_LBUTTONDOWN:
		mouseDownEvent = true;
		mouse_x = LOWORD(lParam);
		mouse_y = HIWORD(lParam);
		mouseDown.x = mouse_dx = mouse_x;
		mouseDown.y = mouse_dy = mouse_y;
		
		break;
	case WM_LBUTTONUP:

		if (process_mode == 1) //Mesh Comparison
		{
#ifdef LIPS_CLOUDCOMPARE
		
			mx = LOWORD(lParam);
			my = HIWORD(lParam);
			triangle_index = 0;
			objectIndex = -1;
			selected_trangle_index = -1;
			SelectEdge = -1;
			hit_trangle_on_mouse_down = false;
			if ((meshcomparer_status == MeshComparisonState::PICK) && mouseDownEvent && (mouseDown.x == mx) && (mouseDown.y == my))
			{
				hit_trangle_on_mouse_down = SelectTriangleAll(mx, my, pObj, bVisibleObj, 10, &objectIndex, &selected_trangle_index, &SelectEdge);

				if (hit_trangle_on_mouse_down)
				{
					HandleTriMeshModel pickObj = pObj[objectIndex];
					int Pnum, Tnum;
					Index3i* TriBuf = lips::GetTriangleBuffer(pickObj, &Tnum);

					int v1, v2, v3;
					lips::GetIndex3iComp(TriBuf, selected_trangle_index, v1, v2, v3);
					Point3f* PtBuf = lips::GetVertexBuffer(pickObj, &Pnum);
					//Point3f PickTriCenter;
					float x[3], y[3], z[3];
					float xc, yc, zc;
					lips::GetPoint3fComp(PtBuf, v1, x[0], y[0], z[0]);
					lips::GetPoint3fComp(PtBuf, v2, x[1], y[1], z[1]);
					lips::GetPoint3fComp(PtBuf, v3, x[2], y[2], z[2]);

					xc = (x[0] + x[1] + x[2]) / 3.0;
					yc = (y[0] + y[1] + y[2]) / 3.0;
					zc = (z[0] + z[1] + z[2]) / 3.0;

#ifdef _DEBUG
					std::cout << "Pick up mesh object:" << objectIndex << endl;
					std::cout << "Pick up triangle index:" << selected_trangle_index << endl;
					std::cout << "Pick up Trianlge center:" << endl;
					std::cout << "x:" << xc << endl;
					std::cout << "y:" << yc << endl;
					std::cout << "z:" << zc << endl;
#endif
					if (bVisibleObj[0] && !bVisibleObj[1]) //Pick-up Comp Mesh
						compare_mesh_tool.addAlignedPoint(xc, yc, zc);
					if (!bVisibleObj[0] && bVisibleObj[1]) //Pick-up Ref Mesh
						compare_mesh_tool.addReferencePoint(xc, yc, zc);
				}
			}
#endif
		}

		mouseUp.x = mx;
		mouseUp.y = my;
		mouseDownEvent = false;

		break;
	case WM_RBUTTONUP:
		switch (process_mode)
		{
		case 0: //3D Scan Mode
			RenderMode += 1;
			RenderMode = RenderMode % 3;
			break;
		case 1: //Mesh Comparison Mode
			//UserLang = GetUserDefaultUILanguage();
			//std::cout << "GetUserDefaultUILanguage = " << UserLang << endl;

			if (compare_mesh_tool.IsEnable())
				PopUpMenu_MeshComparison(hWnd, meshcomparer_status, compare_mesh_tool);
			break;
		default:
			break;
		}

		break;
	case WM_MOUSEMOVE:

		mouse_dx = LOWORD(lParam);
		mouse_dy = HIWORD(lParam);
		if (mouseDownEvent)
		{
			if (MouseMoveMode)
			{
				PushMouse(mouse_dx - mouse_x, mouse_dy - mouse_y, 0);
			}
			else
			{
				RotateMouse(mouse_dx - mouse_x, mouse_dy - mouse_y);
			}
		}

		mouse_x = mouse_dx;
		mouse_y = mouse_dy;

		break;
	case WM_MOUSEWHEEL:
		tmp = (int)wParam;

		if (MeshNum >= 1)
		{
			if (MoveObj)
			{
				if (tmp > 0)
				{
					Scale[0][0] = Scale[0][0] / 0.8;
					Scale[0][1] = Scale[0][1] / 0.8;
					Scale[0][2] = Scale[0][2] / 0.8;
				}
				else
				{
					Scale[0][0] = Scale[0][0] * 0.8;
					Scale[0][1] = Scale[0][1] * 0.8;
					Scale[0][2] = Scale[0][2] * 0.8;
				}
			}
			else
			{
				if (tmp > 0)
				{
					ScaleObj = ScaleObj / 0.8;
				}
				else
				{
					ScaleObj = ScaleObj * 0.8;
				}
				if (ScaleObj < 0.01)
				{
					ScaleObj = 0.01;
				}
				if (ScaleObj > 50000)
				{
					ScaleObj = 50000;
				}
			}
		}
		break;

	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_SHIFT:
			MouseMoveMode = true;
			break;
		default:
			break;
		}
		break;
	case WM_KEYUP:
		switch (wParam)
		{
		case VK_SHIFT:
			MouseMoveMode = false;
			break;
		default:
			break;
		}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
#ifdef LIPS_CLOUDCOMPARE
		case SWM_SET_SCAN_COMPMESH:
			if (meshcomparer_status == MeshComparisonState::NONE)
			{
				if (pObj[0] != NULL)
				{
					if (compare_mesh_tool.setCompMesh(pObj[0]))
						MessageBox(NULL, "Set 3D scanned mesh to compared mesh file.", "Mesh Comparison", MB_OK);

					if (pObj[0] && pObj[1])
						meshcomparer_status = MeshComparisonState::LOAD;
				}
				else
					std::cout << "Mesh container is empty! No Mesh can set as compared entity!" << std::endl;
			}

			break;

		case SWM_LOAD_COMPMESH:
			if (meshcomparer_status == MeshComparisonState::NONE)
			{
				load_comp_mesh(compare_mesh_tool);
			
				if (pObj[0] && pObj[1])
					meshcomparer_status = MeshComparisonState::LOAD;
			}
			break;

		case SWM_LOAD_REFMESH:
			if (meshcomparer_status == MeshComparisonState::NONE)
			{
				load_ref_mesh(compare_mesh_tool);
				
				if (pObj[0] && pObj[1])
					meshcomparer_status = MeshComparisonState::LOAD;
			}
			break;

		case SWM_COMPMESH_VISIBILITY:
			bVisibleObj[0] = !bVisibleObj[0];//flag of comp mesh
			if (bVisibleObj[0])
				MessageBox(NULL, "Turn On", "Compared Mesh Visibility", MB_OK);
			else
				MessageBox(NULL, "Turn Off", "Compared Mesh Visibility", MB_OK);
			break;

		case SWM_REFMESH_VISIBILITY:
			bVisibleObj[1] = !bVisibleObj[1];//flag of ref  mesh
			if (bVisibleObj[1])
				MessageBox(NULL, "Turn On", "Referenced Mesh Visibility", MB_OK);
			else
				MessageBox(NULL, "Turn Off", "Referenced Mesh Visibility", MB_OK);
			break;

		case SWM_POSE_COMPMESH:
			meshcomparer_status = MeshComparisonState::POSE;
			bTransformObj[0] = !bTransformObj[0];
			if (bTransformObj[0])
				MessageBox(NULL, "Turn On", "Compared Mesh Transformation", MB_OK);
			else
				MessageBox(NULL, "Turn Off", "Compared Mesh Transformation", MB_OK);
			break;

		case SWM_POSE_REFMESH:
			meshcomparer_status = MeshComparisonState::POSE;
			bTransformObj[1] = !bTransformObj[1];
			if (bTransformObj[1])
				MessageBox(NULL, "Turn On", "Referenced Mesh Transformation", MB_OK);
			else
				MessageBox(NULL, "Turn Off", "Referenced Mesh Transformation", MB_OK);
			break;

		case SWM_PICK_COMPMESH_POINTS:
			MessageBox(NULL, "Pick feature points on comp mesh model", "MeshComparison", MB_OK);
			//show comp mesh & hide ref mesh
			bVisibleObj[0] = TRUE; //flag of comp mesh
			bVisibleObj[1] = FALSE;//flag of ref  mesh
			//change mesh compariosn status
			meshcomparer_status = MeshComparisonState::PICK;
			break;

		case SWM_PICK_REFMESH_POINTS:
			MessageBox(NULL, "Pick feature points on reference mesh model", "MeshComparison", MB_OK);
			//hide comp mesh & show ref mesh
			bVisibleObj[0] = FALSE; //flag of comp mesh
			bVisibleObj[1] = TRUE;  //flag of ref  mesh
			//change mesh compariosn status
			meshcomparer_status = MeshComparisonState::PICK;
			break;

		case SWM_CLEAR_COMPMESH_POINTS:
			compare_mesh_tool.clearAlignedPoints();
			MessageBox(NULL, "Clear feature points of aligned mesh.", "MeshComparison", MB_OK);
			meshcomparer_status = MeshComparisonState::LOAD;
			break;

		case SWM_CLEAR_REFMESH_POINTS:
			compare_mesh_tool.clearRefPoints();
			MessageBox(NULL, "Clear feature points of referenced mesh.", "MeshComparison", MB_OK);
			meshcomparer_status = MeshComparisonState::LOAD;
			break;

		case SWM_MATCHING_POINT_PAIRS:
			
			MessageBox(NULL, "Start point pairs matching.", "MeshComparison", MB_OK);
			compare_mesh_tool.Align();
			MessageBox(NULL, "Complete point pairs matching!", "MeshComparison", MB_OK);

			ReleaseRenderBuf(MeshBuf[0]);
			PrepareRenderBuf(compare_mesh_tool.getCompMesh(), MeshBuf[0]);

			bVisibleObj[0] = true;
			bVisibleObj[1] = true;
			meshcomparer_status = MeshComparisonState::ALIGN;
			break;

		case SWM_ICP:
			MessageBox(NULL, "Please specify parameters for mesh object registration in console window.", "MeshComparison", MB_OK);

			std::cout << "Registration default criteria : " << pow(10, ICP_criteria) << endl;
			do {
				std::cout << "Specify exponent of criteria and press 'Enter', range limitation = -2 ~ -5 (for example exponent = -2 for 10E-2) : ";
				std::cin >> ICP_criteria;
			} while ( (ICP_criteria>-2) || (ICP_criteria<-5) );

			std::cout << "Registration default sampling point number : " << ICP_sampling_num << endl;
			do {
				std::cout << "Specify sampling number and press 'Enter', range limitation = 10000 ~ 50000 : ";
				std::cin >> ICP_sampling_num;
			} while ((ICP_sampling_num>50000)||(ICP_sampling_num<10000));

			std::cout << "Fine-registration, criteria = " << pow(10, ICP_criteria) << ", sampling number = " << ICP_sampling_num  <<endl;
			MessageBox(NULL, "Start fine-registration.", "MeshComparison", MB_OK);
			startTime = clock();
			ChangeCursor_HourGlass();

			compare_mesh_tool.Register(pow(10, ICP_criteria), ICP_sampling_num, true);

			ChangeCursor(NULL, IDC_ARROW);

			stopTime = clock();
			duration = double(stopTime - startTime) / CLOCKS_PER_SEC;
			std::cout << "LIPS_MeshComparison::Register(), " << duration << "(s)" << std::endl;
			MessageBox(NULL, "Complete finregistration!", "MeshComparison", MB_OK);

			ReleaseRenderBuf(MeshBuf[0]);
			PrepareRenderBuf(compare_mesh_tool.getCompMesh(), MeshBuf[0]);
			meshcomparer_status = MeshComparisonState::ICP;
			break;

		case SWM_DISTANCE:
		
			MessageBox(NULL, "Start mesh distances computation.", "MeshComparison", MB_OK);
			bool bComputeDistance;
			startTime = clock();
			ChangeCursor_HourGlass();
			bComputeDistance = compare_mesh_tool.ComputeMeshDistances(true);
			ChangeCursor(NULL, IDC_ARROW);
			stopTime = clock();
			duration = double(stopTime - startTime) / CLOCKS_PER_SEC;
			std::cout << "LIPS_MeshComparison::ComputeMeshDistances(), " << duration << "(s)" << std::endl;
			MessageBox(NULL, "Complete mesh distances computation!", "MeshComparison", MB_OK);

			// Post-processing after Mesh Distance Computation
			if (bComputeDistance)
			{
				std::cout << "Compute compared mesh to reference mesh distances finish!" << endl;

				//Mesh Distances Histogram
				int n_steps = 256;
				compare_mesh_tool.GenHistogram(n_steps);

				//Initialize color scales
				compare_mesh_tool.ClearColorScale();

				colorscale_names.push_back("Default");
				colorscale_names.push_back("Small");
				colorscale_names.push_back("Medium");
				colorscale_names.push_back("Large");

				compare_mesh_tool.AddColorScale(colorscale_names[0]);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 0.0, 255, 0, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 0.2, 255, 140, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 0.4, 127, 127, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 0.5, 0, 255, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 0.6, 0, 127, 127);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 0.8, 0, 0, 255);
				compare_mesh_tool.AddColorStep(colorscale_names[0], 1.0, 223, 115, 255);

				compare_mesh_tool.AddColorScale(colorscale_names[1]);
				compare_mesh_tool.AddColorStep(colorscale_names[1], 0.0, 255, 0, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[1], 1.0, 0, 0, 255);

				compare_mesh_tool.AddColorScale(colorscale_names[2]);
				compare_mesh_tool.AddColorStep(colorscale_names[2], 0.0, 255, 0, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[2], 0.5, 0, 255, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[2], 1.0, 0, 0, 255);

				compare_mesh_tool.AddColorScale(colorscale_names[3]);
				compare_mesh_tool.AddColorStep(colorscale_names[3], 0.0, 255, 0, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[3], 0.25, 255, 255, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[3], 0.5, 0, 255, 0);
				compare_mesh_tool.AddColorStep(colorscale_names[3], 0.75, 0, 0, 255);
				compare_mesh_tool.AddColorStep(colorscale_names[3], 1.0, 128, 0, 128);

				//Mesh Distance Shading Colors
				bool bGenMeshColor = compare_mesh_tool.GenCompMeshDistanceColors(colorscale_names[colorscale_index], 2.0);
				if (bGenMeshColor)
				{
					ReleaseRenderBuf(MeshBuf[0]);
					PrepareRenderBuf(compare_mesh_tool.getCompMesh(), MeshBuf[0]);
					bVisibleObj[0] = true;
					bVisibleObj[1] = false;
				
					RenderMode = 0;

					std::cout << "Generate distance colors of compared mesh : " << colorscale_names[colorscale_index] << std::endl;
				}
				else
					std::cout << "Failure of generate distance colors!!" << std::endl;

#ifdef MESHCOMPARISON_COLORGAUGE
				//--- Mesh Distance Color Scale Image ---
				mesh_comparison_gauge.Img = compare_mesh_tool.GenColorbarBuf(colorscale_names[colorscale_index], mesh_comparison_gauge.ImgWitdh, mesh_comparison_gauge.ImgHeight);
				ByteArrayToTexture(mesh_comparison_gauge.Img, &lipscan_color_bar_texture, mesh_comparison_gauge.ImgWitdh, mesh_comparison_gauge.ImgHeight);
				
				//--- Mesh Distance Color Scale Text Image ---
				mesh_comparison_gauge.TextNum = 5;
				mesh_comparison_gauge.TextSize = new SIZE[mesh_comparison_gauge.TextNum];
				mesh_comparison_gauge.TextImg = compare_mesh_tool.GenColorbarText(2.0, mesh_comparison_gauge.TextNum, mesh_comparison_gauge.TextSize, false);
				
				if (lipscan_color_bar_word_texture) delete lipscan_color_bar_word_texture;
				lipscan_color_bar_word_texture = new GLuint[mesh_comparison_gauge.TextNum = 5];
				for (int i = 0; i < mesh_comparison_gauge.TextNum; i++)
					ByteArrayToTexture(mesh_comparison_gauge.TextImg[i], &(lipscan_color_bar_word_texture[i]), mesh_comparison_gauge.TextSize[i].cx, mesh_comparison_gauge.TextSize[i].cy);
#endif
				//Set Mesh Comparison Status
				meshcomparer_status = MeshComparisonState::COMPUTE;
			}
			else
			{
				std::cout << "Compute mesh distances occurred failure!" << endl;
				MessageBox(NULL, "Compute mesh distances occurred failure!", "MeshComparison", MB_OK);
			}

			break;

		case SWM_EXPORT_MESHDISTANCE_HISTOGRAM:
			//--- Export distance field to histogram csv file ---
			csv_filter = "CSV Files (*.csv*)\0*.*\0";
			csv_extension = ".csv";
			csv_filename = opensavefilename((char*)csv_filter, NULL, false);
			if (csv_filename.size())
				csv_filename += csv_extension;

			if (csv_filename.c_str())
			{
				if (compare_mesh_tool.ExportToCSV(csv_filename.c_str()))
					MessageBoxW(NULL, L"Export Histogram to CSV file(.csv)", L"Mesh Distances Histogram", MB_OK);
			}
			//---------------------------------------------------
			break;

		case SWM_RESET:
			//--- Rest flowchart of Mesh Comparison ---
			compare_mesh_tool.Reset();
			clear_comp_mesh(compare_mesh_tool);
			clear_ref_mesh(compare_mesh_tool);
			MeshNum = 0; //non-zero will crash at mouse action

			//Set Mesh Comparison Status
			meshcomparer_status = MeshComparisonState::NONE;
			
			//--- Reset OpenGL camera setting ---
			InitGLCamera();

			MessageBox(NULL, "Reset work phase.", "MeshComparison", MB_OK);
			//---------------------------------------------------
			break;

		case SWM_COLORSCALE:
			InitGLCamera();
			//--- Change color scale to mesh distances rendering ---
			colorscale_index = (colorscale_index + 1) % colorscale_names.size();
			//Mesh Distance Shading Colors

			if (compare_mesh_tool.GenCompMeshDistanceColors(colorscale_names[colorscale_index], 2.0))
			{
				ReleaseRenderBuf(MeshBuf[0]);
				pObj[0] = compare_mesh_tool.getCompMesh();
				PrepareTransformation(pObj, 0);
				PrepareRenderBuf(pObj[0]/*compare_mesh_tool.getCompMesh()*/, MeshBuf[0]);

#ifdef MESHCOMPARISON_COLORGAUGE
				//Mesh Distance Color Scale Image
				mesh_comparison_gauge.Img = compare_mesh_tool.GenColorbarBuf(colorscale_names[colorscale_index], mesh_comparison_gauge.ImgWitdh, mesh_comparison_gauge.ImgHeight);
				ByteArrayToTexture(mesh_comparison_gauge.Img, &lipscan_color_bar_texture, mesh_comparison_gauge.ImgWitdh, mesh_comparison_gauge.ImgHeight);
#endif

				bVisibleObj[1] = false;
				std::cout << "Generate distance colors of compared mesh : " << colorscale_names[colorscale_index] << std::endl;
			}
			else
				std::cout << "Failure of generate distance colors!! " << colorscale_names[colorscale_index] << std::endl;
			//------------------------------------------------------
			break;
#endif
		}
	}

	default:
		return (DefWindowProc(hWnd, msg, wParam, lParam));
	}
	return 0;
}

bool CreateWin32glWindow(LPCSTR WndName)
{
	WindowRect.left = (long)0;
	WindowRect.right = (long)frame_width;
	WindowRect.top = (long)0;
	WindowRect.bottom = (long)frame_height;

	main_window_title = WndName;
	hInstance = GetModuleHandle(NULL);
	main_icon = LoadIcon(NULL, IDI_APPLICATION);
	stand_hcur = LoadCursor(NULL, IDC_ARROW);

	wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wc.lpfnWndProc = (WNDPROC)WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = main_icon;
	wc.hCursor = stand_hcur;
	wc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = WndName;

	if (!RegisterClass(&wc))
	{
		MessageBox(NULL, "Failed To Register The Window Class.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}

	dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;
	dwStyle = (WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX);
	AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);

	//
	hWnd = CreateWindowEx(dwExStyle,
		wc.lpszClassName,
		main_window_title,
		WS_CLIPSIBLINGS |
		WS_CLIPCHILDREN |
		dwStyle,
		0, 0,
		(int)(WindowRect.right - WindowRect.left),
		(int)(WindowRect.bottom - WindowRect.top),
		NULL,
		NULL,
		hInstance,
		NULL);

	if (!(hWnd))
	{
		KillGLWindow();
		MessageBox(NULL, "Window Creation Error.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}
#ifdef _DEBUG
	std::cout << "Create win32 window , handle adress : " << hWnd << endl;
#endif

	static PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW |
		PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		16,
		0, 0, 0, 0, 0, 0,
		0,
		0,
		0,
		0, 0, 0, 0,
		16,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0, 0, 0
	};

	if (!(hDC = GetDC(hWnd)))
	{
		KillGLWindow();
		MessageBox(NULL, "Can't Create A GL Device Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}

	if (!(PixelFormat = ChoosePixelFormat(hDC, &pfd)))
	{
		KillGLWindow();
		MessageBox(NULL, "Can't Find A Suitable PixelFormat.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}

	if (!SetPixelFormat(hDC, PixelFormat, &pfd))
	{
		KillGLWindow();
		MessageBox(NULL, "Can't Set The PixelFormat.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}

	if (!(hGLRC = wglCreateContext(hDC)))
	{
		KillGLWindow();
		MessageBox(NULL, "Can't Create A GL Rendering Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}

	if (!wglMakeCurrent(hDC, hGLRC))
	{
		KillGLWindow();
		MessageBox(NULL, "Can't Activate The GL Rendering Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}

	if (!InitGL())
	{
		KillGLWindow();
		MessageBox(NULL, "Initialization Failed.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return 0;
	}

	InitLight(m_pLight);
	SetLights(m_pLight);
	ShowWindow(hWnd, SW_SHOW);

	return TRUE;
}

GLvoid ReSizeGLScene(GLsizei width, GLsizei height)
{
	if (width == 0)
	{
		width = 1;
	}
	if (height == 0)
	{
		height = 1;
	}

	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	double centerx = 0, centery = 0, centerz = 0, upx = 0, upy = 1, upz = 0;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_pLight->Position[0] = CameraPos[0];
	m_pLight->Position[1] = CameraPos[1];
	m_pLight->Position[2] = CameraPos[2];
}

int InitGL(GLvoid)
{
	glShadeModel(GL_SMOOTH);

	glEnable(GL_TEXTURE_2D);

	glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glColor4f(1.0f, 1.0f, 1.0f, 0.3f);
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	float Color[4];
	Color[0] = 0.0f;	Color[1] = 0.2f;	Color[2] = 0.4f;	Color[3] = 0.2f;
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, Color);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(2.0f);

	Color[0] = 1.0f;	Color[1] = 1.0f;	Color[2] = 1.0f;	Color[3] = 0.0f;
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, Color);

	float amb[4];
	amb[0] = amb[1] = amb[2] = 0.0f; amb[3] = 1.0f;
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_DITHER);
	glEnable(GL_NORMALIZE);

	return TRUE;
}

GLvoid KillGLWindow(GLvoid)
{
	if (hGLRC)
	{
		if (!wglMakeCurrent(NULL, NULL))
		{
			MessageBox(NULL, "Release Of DC And RC Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		}

		if (!wglDeleteContext(hGLRC))
		{
			MessageBox(NULL, "Release Rendering Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		}
		hGLRC = NULL;
	}

	if (hDC && !ReleaseDC(hWnd, hDC))
	{
		MessageBox(NULL, "Release Device Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		hDC = NULL;
	}

	if (hWnd && !DestroyWindow(hWnd))
	{
		MessageBox(NULL, "Could Not Release hWnd.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		hWnd = NULL;
	}

	if (!UnregisterClass(wc.lpszClassName, hInstance))
	{
		MessageBox(NULL, "Could Not Unregister Class.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		hInstance = NULL;
	}
}

void InitGLCamera()
{
	CameraTrans[0] = 0.0; CameraTrans[1] = 0.0; CameraTrans[2] = 0.0;

	CameraPos[0] = 0.0; CameraPos[1] = 0.0; CameraPos[2] = 1.0;

	CameraRotate[0] = -0.64147977141606727; CameraRotate[1] = 0.76713990350957051; CameraRotate[2] = 0.00026703429895608144;
	CameraRotate[3] = -0.23098111283193304; CameraRotate[4] = -0.19347753291939468; CameraRotate[5] = 0.95352722550032443;
	CameraRotate[6] = 0.73154044890143433; CameraRotate[7] = 0.61160674677340765; CameraRotate[8] = 0.30130675220170028;

	ScaleObj = 1.0;
}

void InitLight(PrmLight *m_pLight)
{
	m_pLight->Ambient[0] = 0.01f; m_pLight->Ambient[1] = 0.01f; m_pLight->Ambient[2] = 0.01f; m_pLight->Ambient[3] = 1.0f;
	m_pLight->Diffuse[0] = 1.0f; m_pLight->Diffuse[1] = 1.0f; m_pLight->Diffuse[2] = 1.0f; m_pLight->Diffuse[3] = 1.0f;
	m_pLight->Specular[0] = 1.0f; m_pLight->Specular[1] = 1.0f; m_pLight->Specular[2] = 1.0f; m_pLight->Specular[3] = 1.0f;
	m_pLight->Position[0] = 0.0f; m_pLight->Position[1] = 0.0f; m_pLight->Position[2] = 1000.0f; m_pLight->Position[3] = 0.0f;
	m_pLight->SpotDirection[0] = 0.0f; m_pLight->SpotDirection[1] = 0.0f; m_pLight->SpotDirection[2] = -1.0f;
	m_pLight->SpotExponent = 0.0f;
	m_pLight->SpotCutoff = 180.0f;
	m_pLight->ConstantAttenuation = 1.0f;
	m_pLight->LinearAttenuation = 0.0f;
	m_pLight->QuadraticAttenuation = 0.0f;
}
void SetLights(PrmLight *ppLight)
{
	glLightfv(GL_LIGHT0, GL_AMBIENT, ppLight->Ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, ppLight->Diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, ppLight->Specular);
	glLightfv(GL_LIGHT0, GL_POSITION, ppLight->Position);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, ppLight->SpotDirection);

	glEnable(GL_LIGHT0);
}
//讀取物件資訊，設定物件中心
void ResetObjPosition(lips::HandleTriMeshModel* handlelist, int pos)
{
	lips::HandleTriMeshModel hMesh = handlelist[pos];

	int i;
	int vnum = 0;
	Point3f* vt = lips::GetVertexBuffer(hMesh, &vnum);

	float x, y, z;
	float x1 = FLT_MAX, x2 = FLT_MIN, y1 = FLT_MAX, y2 = FLT_MIN, z1 = FLT_MAX, z2 = FLT_MIN;
	for (i = 0; i < vnum; i++)
	{
		lips::GetPoint3fComp(vt, i, x, y, z);

		if (x1 > x) { x1 = x; }
		if (x2 < x) { x2 = x; }
		if (y1 > y) { y1 = y; }
		if (y2 < y) { y2 = y; }
		if (z1 > z) { z1 = z; }
		if (z2 < z) { z2 = z; }
	}

	Center[pos][0] = (x1 + x2) / 2;
	Center[pos][1] = (y1 + y2) / 2;
	Center[pos][2] = (z1 + z2) / 2;

	Pivot[pos][0] = Center[pos][0];
	Pivot[pos][1] = Center[pos][1];
	Pivot[pos][2] = Center[pos][2];

	double max_dis = 0;
	if (pos == 0)
	{
		if (max_dis < (Center[pos][0] + 100)) { max_dis = Center[pos][0] + 100; }
		if (max_dis < (Center[pos][1] + 100)) { max_dis = Center[pos][1] + 100; }
		if (max_dis < (Center[pos][2] + 100)) { max_dis = Center[pos][2] + 100; }

		if (max_dis < abs(x1 - Center[pos][0])) { max_dis = abs(x1 - Center[pos][0]); }
		if (max_dis < abs(x2 - Center[pos][0])) { max_dis = abs(x2 - Center[pos][0]); }
		if (max_dis < abs(y1 - Center[pos][1])) { max_dis = abs(y1 - Center[pos][1]); }
		if (max_dis < abs(y2 - Center[pos][1])) { max_dis = abs(y2 - Center[pos][1]); }
		if (max_dis < abs(z1 - Center[pos][2])) { max_dis = abs(z1 - Center[pos][2]); }
		if (max_dis < abs(z2 - Center[pos][2])) { max_dis = abs(z2 - Center[pos][2]); }
		max_dis = sqrt(max_dis*max_dis * 3);
		Vx1 = x1; Vx2 = x2;
		Vy1 = y1; Vy2 = y2;
		Vz1 = -max_dis; Vz2 = max_dis;
		ScaleObj = 0.8;
	}
	else
	{
		Center[pos][0] = Center[0][0];
		Center[pos][1] = Center[0][1];
		Center[pos][2] = Center[0][2];
	}
	Vz1 = -5000; Vz2 = 5000;

	Point3f *vn = lips::GetNormalBuffer(hMesh);
	if (vn == NULL) { lips::GenerateNormal(hMesh); }

	Trans[pos][0] = 0; Trans[pos][1] = 0; Trans[pos][2] = 0;
	Scale[pos][0] = 1; Scale[pos][1] = 1; Scale[pos][2] = 1;

	Roate[pos][0] = 1; Roate[pos][1] = 0; Roate[pos][2] = 0;
	Roate[pos][3] = 0; Roate[pos][4] = 1; Roate[pos][5] = 0;
	Roate[pos][6] = 0; Roate[pos][7] = 0; Roate[pos][8] = 1;

	BasePointNum[pos] = 0;
}

void PrepareTransformation(lips::HandleTriMeshModel* pMeshList, int pos)
{
	ResetObjPosition(pMeshList, pos); //calculate mesh center point and set unit transformation 
	lips::TransformObject(pMeshList, pos); //
	ResetObjPosition(pMeshList, pos); //update mesh center point and set unit transformation 
	ResetCameraView(CameraTrans, CameraPos, CameraRotate, ScaleObj); //reset matrix of opengl camera transformation
	lips::RotateObj(Roate[pos], 0, 90, 0); //apply default rotation to mesh 
}

//繪製網格模型(容器)
void RenderMeshList(MeshRenderBuffer*  MeshBufList, ORTHO_PROJECTION_POSITION* orth_projection_position, int COLORSPACETYPE, size_t Size, bool MouseSelect, bool* Visible)
{
	if (Size <= 0) { return; }

	int i, k, Tnum, Pnum;

	float Mat[4];
	double Matrix[16];

	double w = iWidth;
	double h = iHeight;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(orth_projection_position->left, orth_projection_position->right, orth_projection_position->bottom, orth_projection_position->top, orth_projection_position->znear, orth_projection_position->zfar);	// Set Up An Ortho Screen

	glMatrixMode(GL_MODELVIEW);

	Mat[0] = Mat[1] = Mat[2] = 0.0f; Mat[3] = 1.0f;
	glMaterialfv(GL_FRONT, GL_SPECULAR, Mat);
	glMaterialf(GL_FRONT, GL_SHININESS, 0.0f);

	glPushMatrix();
	glLoadIdentity();

	Matrix[0] = CameraRotate[0];	Matrix[4] = CameraRotate[1];	Matrix[8] = CameraRotate[2];	Matrix[12] = 0;
	Matrix[1] = CameraRotate[3];	Matrix[5] = CameraRotate[4];	Matrix[9] = CameraRotate[5];	Matrix[13] = 0;
	Matrix[2] = CameraRotate[6];	Matrix[6] = CameraRotate[7];	Matrix[10] = CameraRotate[8];	Matrix[14] = 0;
	Matrix[3] = 0;				Matrix[7] = 0;				Matrix[11] = 0;				Matrix[15] = 1;

	glMultMatrixd(Matrix);

	for (i = 0; i < Size; i++)
	{
		if (MeshBufList[i].TriNum && Visible[i]) //if mesh model exist and visible
		{
			// create render buffer of triangle mesh, Eason, 20211224
			Tnum = MeshBufList[i].TriNum;
			float* NrmBuf = MeshBufList[i].NrmBuf;
			float* P3dBuf = MeshBufList[i].P3dBuf;
			unsigned char* ColorBuf = MeshBufList[i].ColorBuf;
			bool* SelBuf = MeshBufList[i].SelBuf;

			glPushMatrix();
			glTranslatef(-Center[i][0], -Center[i][1], -Center[i][2]);

			Matrix[0] = Roate[i][0];	Matrix[4] = Roate[i][1];	Matrix[8] = Roate[i][2];	Matrix[12] = 0;
			Matrix[1] = Roate[i][3];	Matrix[5] = Roate[i][4];	Matrix[9] = Roate[i][5];	Matrix[13] = 0;
			Matrix[2] = Roate[i][6];	Matrix[6] = Roate[i][7];	Matrix[10] = Roate[i][8];	Matrix[14] = 0;
			Matrix[3] = 0;				Matrix[7] = 0;				Matrix[11] = 0;				Matrix[15] = 1;

			glTranslatef(Trans[i][0], Trans[i][1], Trans[i][2]);
			glTranslatef(Pivot[0][0], Pivot[0][1], Pivot[0][2]);
			glMultMatrixd(Matrix);
			glScalef(Scale[i][0], Scale[i][1], Scale[i][2]);
			glTranslatef(-Pivot[0][0], -Pivot[0][1], -Pivot[0][2]);

			switch (RenderMode)
			{
#pragma region Mesh With Image Color
			case 0: //Mesh with RGB
				glEnable(GL_LIGHTING);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glBegin(GL_TRIANGLES);
				if (ColorBuf != NULL)
				{
					for (k = 0; k < Tnum; k++)
					{
						glColor3ubv(&(ColorBuf[k * 9]));
						glNormal3fv(&(NrmBuf[k * 9]));
						glVertex3fv(&(P3dBuf[k * 9]));

						glColor3ubv(&(ColorBuf[k * 9 + 3]));
						glNormal3fv(&(NrmBuf[k * 9 + 3]));
						glVertex3fv(&(P3dBuf[k * 9 + 3]));

						glColor3ubv(&(ColorBuf[k * 9 + 6]));
						glNormal3fv(&(NrmBuf[k * 9 + 6]));
						glVertex3fv(&(P3dBuf[k * 9 + 6]));
					}
				}
				glEnd();
				glDisable(GL_LIGHTING);
				break;
#pragma endregion

#pragma region Mesh With White Shading
			case 1: //Mesh with white color
				glEnable(GL_LIGHTING);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glBegin(GL_TRIANGLES);
				for (k = 0; k < Tnum; k++)
				{
					glColor3ub(255, 255, 255);
					glNormal3fv(&(NrmBuf[k * 9]));
					glVertex3fv(&(P3dBuf[k * 9]));

					glColor3ub(255, 255, 255);
					glNormal3fv(&(NrmBuf[k * 9 + 3]));
					glVertex3fv(&(P3dBuf[k * 9 + 3]));

					glColor3ub(255, 255, 255);
					glNormal3fv(&(NrmBuf[k * 9 + 6]));
					glVertex3fv(&(P3dBuf[k * 9 + 6]));
				}
				glEnd();
				glDisable(GL_LIGHTING);
				break;
#pragma endregion

#pragma region WireFrame
			case 2: //Wireframe
				glEnable(GL_LIGHTING);
				glEnable(GL_POLYGON_OFFSET_LINE);
				glPolygonOffset(-1.0f, -1.0f);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glBegin(GL_TRIANGLES);
				for (k = 0; k < Tnum; k++)
				{
					glColor3ub(0, 127, 255);
					glNormal3fv(&(NrmBuf[k * 9]));
					glVertex3fv(&(P3dBuf[k * 9]));

					glColor3ub(0, 127, 255);
					glNormal3fv(&(NrmBuf[k * 9 + 3]));
					glVertex3fv(&(P3dBuf[k * 9 + 3]));

					glColor3ub(0, 127, 255);
					glNormal3fv(&(NrmBuf[k * 9 + 6]));
					glVertex3fv(&(P3dBuf[k * 9 + 6]));
				}
				glEnd();
				glDisable(GL_POLYGON_OFFSET_LINE);
				glDisable(GL_LIGHTING);
				break;
#pragma endregion
			default:
				break;
			}

#ifdef LIPS_CLOUDCOMPARE
			// Rendering Aligned Points & Referenced Points of mesh comparison
			if (meshcomparer_status == MeshComparisonState::PICK || MeshComparisonState::ALIGN)
			{
				// Create a new quadric and render it normally
				GLUquadricObj *pObj = gluNewQuadric();
				gluQuadricDrawStyle(pObj, GLU_FILL);

				glPointSize(10);
				glBegin(GL_POINTS);
				if (i == 0) //Pick-up Aligned points for Compared Mesh
				{
					glColor3ub(255, 0, 0);
					//ccPointCloud alignPts = meshcmp.getAlignedPoints();
					std::vector<VERTEX3F> alignPts = compare_mesh_tool.getAlignedPoints();
					for (int j = 0;j < alignPts.size(); j++)
					{
						VERTEX3F Pt = alignPts[j];
						glVertex3f(Pt.x, Pt.y, Pt.z);
					}
				}

				if (i == 1) //Pick-up Reference points for Reference Mesh
				{
					glColor3ub(0, 255, 0);
					//ccPointCloud refPts = meshcmp.getReferencePoints();
					std::vector<VERTEX3F> refPts = compare_mesh_tool.getReferencePoints();
					for (int j = 0;j < refPts.size(); j++)
					{
						VERTEX3F Pt = refPts[j];
						glVertex3f(Pt.x, Pt.y, Pt.z);
					}
				}
				glEnd();
				glPointSize(1);

				// Free the quadric object
				gluDeleteQuadric(pObj);
			}
#endif
			glDisable(GL_LIGHTING);

			glPopMatrix();
		}
	}

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glFlush();
}

void RenderImageTexture(GLuint texture, ORTHO_PROJECTION_POSITION* projection_location, RENDER_IMAGE_LOCATION location, int alpha, int zOrder)
{
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);								// Select The Projection Matrix
	glPushMatrix();												// Store The Projection Matrix
	glLoadIdentity();											// Reset The Projection Matrix
	glOrtho(projection_location->left, projection_location->right, projection_location->bottom, projection_location->top, projection_location->znear, projection_location->zfar);				// Set Up An Ortho Screen
																// glOrtho(-100,iWidth,0,iHeight,-1,1);							// Set Up An Ortho Screen
																// glOrtho(-iWidth/2,iWidth/2, -iHeight/2,iHeight/2, -1,1);							// Set Up An Ortho Screen
	glMatrixMode(GL_MODELVIEW);									// Select The Modelview Matrix

	//Draw
	glEnable(GL_TEXTURE_2D);

	glEnable(GL_BLEND);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBindTexture(GL_TEXTURE_2D, texture);

	//glBegin(GL_POLYGON);
	glBegin(GL_QUADS);

	glColor3ub(alpha, alpha, alpha);
	glTexCoord2f(0, 1); glVertex3f(location.p1.x, location.p1.y, zOrder);
	glTexCoord2f(0, 0); glVertex3f(location.p1.x, location.p2.y, zOrder);
	glTexCoord2f(1, 0); glVertex3f(location.p2.x, location.p2.y, zOrder);
	glTexCoord2f(1, 1); glVertex3f(location.p2.x, location.p1.y, zOrder);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);								// Select The Projection Matrix
	glPopMatrix();												// Restore The Old Projection Matrix
	glMatrixMode(GL_MODELVIEW);									// Select The Modelview Matrix
	glFlush();													// Flush The GL Rendering Pipeline
}

void ByteArrayToTexture(unsigned char* buffer, GLuint* texture, int width, int height)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glGenTextures(1, texture);
	glBindTexture(GL_TEXTURE_2D, *texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, buffer);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void ResetCameraView(float* CameraTrans, float* CameraPos, double* CameraRotate, float ScaleObj)
{
	CameraTrans[0] = 0; CameraTrans[1] = 0; CameraTrans[2] = 0;
	CameraPos[0] = 0; CameraPos[1] = 0; CameraPos[2] = 1;
	CameraRotate[0] = 0; CameraRotate[1] = 0; CameraRotate[2] = 1;
	CameraRotate[3] = 1; CameraRotate[4] = 0; CameraRotate[5] = 0;
	CameraRotate[6] = 0; CameraRotate[7] = 1; CameraRotate[8] = 0;
	ScaleObj = 0.8;
}

#pragma region Mesh Data Buffer for OpenGL rendering
//初始化網格模型的繪圖暫存器
void PrepareRenderBuf(lips::HandleTriMeshModel HMesh, MeshRenderBuffer& buf)
{
	buf.TriNum = 0;
	if (buf.TriBuf != NULL) delete buf.TriBuf;
	if (buf.ColorBuf != NULL) delete buf.ColorBuf;
	if (buf.P3dBuf != NULL) delete buf.P3dBuf;
	if (buf.NrmBuf != NULL) delete buf.NrmBuf;
	if (buf.SelBuf != NULL) delete buf.SelBuf;

	int Tnum;
	int Pnum;
	Index3i* tribuf = lips::GetTriangleBuffer(HMesh, &Tnum);
	Point3f* p3dbuf = lips::GetVertexBuffer(HMesh, &Pnum);
	Point3f* nrmbuf = lips::GetNormalBuffer(HMesh);
	if (!nrmbuf) nrmbuf = lips::GenerateNormal(HMesh);
	ColorRGB* clrbuf = lips::GetColorBuffer(HMesh);
	buf.TriNum = Tnum;
	buf.TriBuf = new int[Tnum * 3];
	buf.P3dBuf = new float[Tnum * 9];
	buf.NrmBuf = new float[Tnum * 9];
	buf.ColorBuf = new unsigned char[Tnum * 9];
	buf.SelBuf = new bool[Tnum];

	int* tmp_tri = buf.TriBuf;
	float* tmp_p3d = buf.P3dBuf;
	float* tmp_nrm = buf.NrmBuf;
	unsigned char* tmp_clr = buf.ColorBuf;
	bool* tmp_sel = buf.SelBuf;

#pragma region Color Model
	if (clrbuf != nullptr)
	{
		for (int i = 0; i < Tnum; i++)
		{
			//Triangle i
			lips::GetIndex3iComp(tribuf, i, tmp_tri[i * 3], tmp_tri[i * 3 + 1], tmp_tri[i * 3 + 2]);

			//Vertex 1
			lips::GetPoint3fComp(p3dbuf, tmp_tri[i * 3], tmp_p3d[i * 9], tmp_p3d[i * 9 + 1], tmp_p3d[i * 9 + 2]);
			lips::GetPoint3fComp(nrmbuf, tmp_tri[i * 3], tmp_nrm[i * 9], tmp_nrm[i * 9 + 1], tmp_nrm[i * 9 + 2]);
			lips::GetColorRGBComp(clrbuf, tmp_tri[i * 3], tmp_clr[i * 9], tmp_clr[i * 9 + 1], tmp_clr[i * 9 + 2]);

			//Vertex2
			lips::GetPoint3fComp(p3dbuf, tmp_tri[i * 3 + 1], tmp_p3d[i * 9 + 3], tmp_p3d[i * 9 + 4], tmp_p3d[i * 9 + 5]);
			lips::GetPoint3fComp(nrmbuf, tmp_tri[i * 3 + 1], tmp_nrm[i * 9 + 3], tmp_nrm[i * 9 + 4], tmp_nrm[i * 9 + 5]);
			lips::GetColorRGBComp(clrbuf, tmp_tri[i * 3 + 1], tmp_clr[i * 9 + 3], tmp_clr[i * 9 + 4], tmp_clr[i * 9 + 5]);

			//Vertex3
			lips::GetPoint3fComp(p3dbuf, tmp_tri[i * 3 + 2], tmp_p3d[i * 9 + 6], tmp_p3d[i * 9 + 7], tmp_p3d[i * 9 + 8]);
			lips::GetPoint3fComp(nrmbuf, tmp_tri[i * 3 + 2], tmp_nrm[i * 9 + 6], tmp_nrm[i * 9 + 7], tmp_nrm[i * 9 + 8]);
			lips::GetColorRGBComp(clrbuf, tmp_tri[i * 3 + 2], tmp_clr[i * 9 + 6], tmp_clr[i * 9 + 7], tmp_clr[i * 9 + 8]);

			//Selection
			tmp_sel[i] = false;
		}
	}
#pragma endregion

#pragma region No Color Model
	else
	{
		for (int i = 0; i < Tnum; i++)
		{
			//Triangle i
			lips::GetIndex3iComp(tribuf, i, tmp_tri[i * 3], tmp_tri[i * 3 + 1], tmp_tri[i * 3 + 2]);

			//Vertex 1
			lips::GetPoint3fComp(p3dbuf, tmp_tri[i * 3], tmp_p3d[i * 9], tmp_p3d[i * 9 + 1], tmp_p3d[i * 9 + 2]);
			lips::GetPoint3fComp(nrmbuf, tmp_tri[i * 3], tmp_nrm[i * 9], tmp_nrm[i * 9 + 1], tmp_nrm[i * 9 + 2]);
			tmp_clr[i * 9] = tmp_clr[i * 9 + 1] = tmp_clr[i * 9 + 2] = 128;

			//Vertex2
			lips::GetPoint3fComp(p3dbuf, tmp_tri[i * 3 + 1], tmp_p3d[i * 9 + 3], tmp_p3d[i * 9 + 4], tmp_p3d[i * 9 + 5]);
			lips::GetPoint3fComp(nrmbuf, tmp_tri[i * 3 + 1], tmp_nrm[i * 9 + 3], tmp_nrm[i * 9 + 4], tmp_nrm[i * 9 + 5]);
			tmp_clr[i * 9 + 3] = tmp_clr[i * 9 + 4] = tmp_clr[i * 9 + 5] = 128;

			//Vertex3
			lips::GetPoint3fComp(p3dbuf, tmp_tri[i * 3 + 2], tmp_p3d[i * 9 + 6], tmp_p3d[i * 9 + 7], tmp_p3d[i * 9 + 8]);
			lips::GetPoint3fComp(nrmbuf, tmp_tri[i * 3 + 2], tmp_nrm[i * 9 + 6], tmp_nrm[i * 9 + 7], tmp_nrm[i * 9 + 8]);
			tmp_clr[i * 9 + 6] = tmp_clr[i * 9 + 7] = tmp_clr[i * 9 + 8] = 128;

			//Selection
			tmp_sel[i] = false;
		}
	}
#pragma endregion
}
//釋放網格模型的繪圖暫存器
void ReleaseRenderBuf(MeshRenderBuffer& buf)
{
	if (buf.TriNum > 0)
		return;

	delete buf.TriBuf; buf.TriBuf = NULL;
	delete buf.P3dBuf; buf.P3dBuf = NULL;
	delete buf.NrmBuf; buf.NrmBuf = NULL;
	delete buf.ColorBuf; buf.ColorBuf = NULL;
	delete buf.SelBuf; buf.SelBuf = NULL;
	buf.TriNum = 0;
}
#pragma endregion

void DrawGLScene(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (MeshNum > 0)
	{
		double Znear = (Vz1);
		double Zfar = (Vz2);

		orth_position->left = (-(iWidth / 2) + CameraTrans[0]) / ScaleObj;
		orth_position->right = ((iWidth / 2) + CameraTrans[0]) / ScaleObj;
		orth_position->bottom = (-iHeight / 2 + CameraTrans[1]) / ScaleObj;
		orth_position->top = (iHeight / 2 + CameraTrans[1]) / ScaleObj;
		orth_position->znear = Znear + CameraTrans[2];
		orth_position->zfar = Zfar + CameraTrans[2];

	
		RenderMeshList(MeshBuf, orth_position, 0, 10, false, bVisibleObj);

#ifdef LIPS_CLOUDCOMPARE
		if (meshcomparer_status == MeshComparisonState::COMPUTE)
		{
			RECT WndRect;
			//GetWindowRect(hWnd, &WndRect);
			GetClientRect(hWnd, &WndRect);

			orth_image_position->left = 0;
			orth_image_position->right = WndRect.right - WndRect.left; //iWidth;
			orth_image_position->bottom = 0;
			orth_image_position->top = WndRect.bottom - WndRect.top; //iHeight;
			orth_image_position->znear = -15000;
			orth_image_position->zfar = 15000;

			colorbar_render_location = { WndRect.right/*iWidth*/ - 150, WndRect.top + 50/*iHeight - 500 - 150*/, WndRect.right/*iWidth*/ - 100, WndRect.bottom/*iHeight*/ - 50 };
			RenderImageTexture(lipscan_color_bar_texture, orth_image_position, colorbar_render_location, 255, 4995);

			//Color Scale Texts
			float deltaHeight = (WndRect.bottom - WndRect.top - 100)/ (float)(mesh_comparison_gauge.TextNum-1);
			for (int i = 0; i < mesh_comparison_gauge.TextNum; i++)
			{
				colorbar_word_render_location.p1.x = iWidth - 300;
				colorbar_word_render_location.p1.y = WndRect.top + 50/*iHeight - 500 - 150*/ + 12 + (i * deltaHeight/*125*/) - 25;
				colorbar_word_render_location.p2.x = iWidth - 150;
				colorbar_word_render_location.p2.y = WndRect.top + 50/*iHeight - 500 - 150*/ + 12 + (i * deltaHeight/*125*/);
				RenderImageTexture((lipscan_color_bar_word_texture[i]), orth_image_position, colorbar_word_render_location, 255, 4995);
			}
		}
#endif
	}
}

void ChangeCursor(HINSTANCE hinstance, LPCSTR cursor_name) //Display default cursor
{
	DestroyCursor(stand_hcur);
	stand_hcur = LoadCursor(hinstance, cursor_name);
	SetCursor(stand_hcur);
}

void ChangeCursor_HourGlass(void) //Display hourglass cursor
{
	DestroyCursor(stand_hcur);
	stand_hcur = LoadCursorFromFile("Hourglass.ani");
	SetCursor(stand_hcur);
}

void PushMouse(float m_x, float m_y, float m_z)
{
	int idx;
	for (idx = 0;idx < MeshNum;idx++)
	{
		if (MoveObj)
		{
			Trans[idx][0] = Trans[idx][0] + (m_x)*RoateVector[0] + (-m_y)*RoateVector[3];
			Trans[idx][1] = Trans[idx][1] + (m_x)*RoateVector[1] + (-m_y)*RoateVector[4];
			Trans[idx][2] = Trans[idx][2] + (m_x)*RoateVector[2] + (-m_y)*RoateVector[5];
		}
		else
		{
			CameraTrans[0] = CameraTrans[0] + (-m_x);
			CameraTrans[1] = CameraTrans[1] + (m_y);
		}
	}
}

void RotateMouse(float m_x, float m_y)
{
	int idx;

	if ((m_x == 0) && (m_y == 0))
		return;
	if (MoveObj)
	{
		for (idx = 0;idx < MeshNum;idx++)
		{
			lips::RotateObjByAxis(Roate[idx], RoateVector, m_x, m_y);
		}
	}
	else
	{
		lips::RotateObj(CameraRotate, m_x, m_y, 0);
	}
}

double FPS()
{
	const int time_stack_num = 10;
	static clock_t time_start = clock();
	static double time_stack[time_stack_num];
	static int time_idx = 0;

	clock_t time_current;
	double time_FPS, diff_time;

	time_current = clock();
	diff_time = time_current - time_start;
	time_stack[time_idx] = diff_time;
	time_idx = (time_idx + 1) % time_stack_num;

	time_FPS = 0;
	for (int i = 0; i < time_stack_num; i++)
	{
		time_FPS = time_FPS + time_stack[i];
	}
	time_FPS = CLOCKS_PER_SEC / (time_FPS / time_stack_num);

	time_start = clock();

	return time_FPS;
}

#pragma region CLOUDCOMPARE
#ifdef LIPS_CLOUDCOMPARE
//網格誤差分析模組選單動作
void PopUpMenu_MeshComparison(HWND _hWnd, MeshComparisonState& status, MeshComparison& cc)
{
	int UserLang = GetUserDefaultUILanguage();

	POINT pt;
	GetCursorPos(&pt);
	HMENU hMenu = CreatePopupMenu();
	HMENU sMenu_load = CreatePopupMenu();
	//HMENU sMenu_pose = CreatePopupMenu();
	HMENU sMenu_align = CreatePopupMenu();
	HMENU sMenu_export = CreatePopupMenu();
	HMENU sMenu_visible = CreatePopupMenu();

	if (hMenu)
	{
		if (sMenu_load)
		{
			if (UserLang == 1028) //CH menu
			{
				//InsertMenu(sMenu_load, -1, MF_BYPOSITION, SWM_SET_SCAN_COMPMESH, "3D Scan->比對模型"); //Disable 
				InsertMenu(sMenu_load, -1, MF_BYPOSITION, SWM_LOAD_COMPMESH, "比對模型");
				InsertMenu(sMenu_load, -1, MF_BYPOSITION, SWM_LOAD_REFMESH, "參照模型");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_load, "匯入");
			}
			else //ENG menu
			{
				InsertMenu(sMenu_load, -1, MF_BYPOSITION, SWM_LOAD_COMPMESH, "Compared Entity");
				InsertMenu(sMenu_load, -1, MF_BYPOSITION, SWM_LOAD_REFMESH, "Referenced Entity");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_load, "Import");
			}
		}
		if (sMenu_visible)
		{
			if (UserLang == 1028) //CH menu
			{
				InsertMenu(sMenu_visible, -1, MF_BYPOSITION, SWM_COMPMESH_VISIBILITY, "比對模型");
				InsertMenu(sMenu_visible, -1, MF_BYPOSITION, SWM_REFMESH_VISIBILITY, "參照模型");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_visible, "顯示");
			}
			else //ENG menu
			{
				InsertMenu(sMenu_visible, -1, MF_BYPOSITION, SWM_COMPMESH_VISIBILITY, "Compared Entity");
				InsertMenu(sMenu_visible, -1, MF_BYPOSITION, SWM_REFMESH_VISIBILITY, "Referenced Entity");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_visible, "Display");
			}
		}
		//if (sMenu_pose) //Disable 
		//{
		//	InsertMenu(sMenu_pose, -1, MF_BYPOSITION, SWM_POSE_COMPMESH, "比對模型");
		//	InsertMenu(sMenu_pose, -1, MF_BYPOSITION, SWM_POSE_REFMESH, "參照模型");
		//	AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_pose, "調整姿態");
		//}
		if (sMenu_align)
		{
			if (UserLang == 1028) //CH menu
			{
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_PICK_COMPMESH_POINTS, "選取定位點(比對)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_PICK_REFMESH_POINTS, "選取定位點(參照)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_CLEAR_COMPMESH_POINTS, "清除定位點(比對)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_CLEAR_REFMESH_POINTS, "清除定位點(參照)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_MATCHING_POINT_PAIRS, "執行");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_align, "三點定位");
			}
			else //ENG menu
			{
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_PICK_COMPMESH_POINTS, "Pick Anchor Point(Compared Entity)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_PICK_REFMESH_POINTS, "Pick Anchor Point(Referenced Entity)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_CLEAR_COMPMESH_POINTS, "Clear Anchor Point(Compared Entity)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_CLEAR_REFMESH_POINTS, "Clear Anchor Point(Referenced Entity)");
				InsertMenu(sMenu_align, -1, MF_BYPOSITION, SWM_MATCHING_POINT_PAIRS, "Execution");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_align, "Point Pair Matching");
			}
		}

		if (UserLang == 1028) //CH menu
		{
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_ICP, "精確定位");
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_DISTANCE, "計算網格距離");
		}
		else //ENG menu
		{
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_ICP, "Mesh Registration");
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_DISTANCE, "Compute Mesh Distances");
		}

		if (sMenu_export)
		{
			if (UserLang == 1028) //CH menu
			{
				InsertMenu(sMenu_export, -1, MF_BYPOSITION, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, "網格距離直方圖");
				//InsertMenu(sMenu_export, -1, MF_BYPOSITION, SWM_EXPORT_DISTANCERGB_MESH, "比對網格模型(渲染)");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_export, "匯出");
			}
			else //ENG menu
			{
				InsertMenu(sMenu_export, -1, MF_BYPOSITION, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, "Mesh Distance Histogram");
				AppendMenu(hMenu, MF_POPUP, (UINT_PTR)sMenu_export, "Export");
			}
		}

		if (UserLang == 1028) //CH menu
		{
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_RESET, "重置");
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_COLORSCALE, "切換顏色表");
		}
		else //ENG menu
		{
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_RESET, "Reset");
			InsertMenu(hMenu, -1, MF_BYPOSITION, SWM_COLORSCALE, "Change Color Scale");
		}

		//Enable/Disable menu items
		switch (status)
		{
		case NONE:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_COMPMESH_VISIBILITY, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_REFMESH_VISIBILITY, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_POSE_COMPMESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_POSE_REFMESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_RESET, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_DISABLED);
			break;
		case LOAD:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_ENABLED);
			if (cc.getCompMesh())
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_UNCHECKED);

			if (cc.getRefMesh())
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_UNCHECKED);
			EnableMenuItem(hMenu, SWM_COMPMESH_VISIBILITY, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_REFMESH_VISIBILITY, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_POSE_COMPMESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_POSE_REFMESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_RESET, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_DISABLED);
			break;
		case PICK:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_DISABLED);
			if (cc.getCompMesh())
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_UNCHECKED);

			if (cc.getRefMesh())
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_UNCHECKED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_RESET, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_DISABLED);
			break;
		case ALIGN:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_DISABLED);
			if (cc.getCompMesh())
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_UNCHECKED);

			if (cc.getRefMesh())
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_UNCHECKED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_RESET, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_DISABLED);
			break;
		case ICP:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_DISABLED);
			if (cc.getCompMesh())
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_UNCHECKED);

			if (cc.getRefMesh())
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_UNCHECKED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_DISABLED);
			break;
		case COMPUTE:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_DISABLED);
			if (cc.getCompMesh())
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED | MF_UNCHECKED);

			if (cc.getRefMesh())
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_UNCHECKED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_ENABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_ENABLED);
			break;
		case EXPORT:

			break;
		default:
			EnableMenuItem(hMenu, SWM_SET_SCAN_COMPMESH, MF_DISABLED);
			if (cc.getCompMesh())
			{
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED);
				CheckMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_CHECKED);
			}
			else
			{
				EnableMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_ENABLED);
				CheckMenuItem(hMenu, SWM_LOAD_COMPMESH, MF_UNCHECKED);
			}

			if (cc.getRefMesh())
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_CHECKED);
			else
				EnableMenuItem(hMenu, SWM_LOAD_REFMESH, MF_ENABLED | MF_UNCHECKED);
			EnableMenuItem(hMenu, SWM_PICK_COMPMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_PICK_REFMESH_POINTS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_MATCHING_POINT_PAIRS, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_ICP, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_DISTANCE, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_MESHDISTANCE_HISTOGRAM, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_EXPORT_DISTANCERGB_MESH, MF_DISABLED);
			EnableMenuItem(hMenu, SWM_COLORSCALE, MF_DISABLED);
			break;
		}

		// note:	must set window to the foreground or the
		//			menu won't disappear when it should
		SetForegroundWindow(_hWnd);

		TrackPopupMenu(hMenu, TPM_BOTTOMALIGN,
			pt.x, pt.y, 0, _hWnd, NULL);
		DestroyMenu(hMenu);
	}
	//End
}

//開啟[讀檔/存檔]對話盒(Returns an empty string if dialog is canceled)
string opensavefilename(const char *filter = "All Files (*.*)\0*.*\0", HWND owner = NULL, bool bOpen = true) {
	OPENFILENAME ofn;
	char fileName[MAX_PATH] = "";
	ZeroMemory(&ofn, sizeof(ofn));

	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = owner;
	ofn.lpstrFilter = filter;
	ofn.lpstrFile = fileName;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	ofn.lpstrDefExt = "";

	string fileNameStr;

	if (bOpen) //Open File
	{
		if (GetOpenFileName(&ofn))
			fileNameStr = fileName;
	}
	else // Save File
	{
		if (GetSaveFileName(&ofn))
			fileNameStr = fileName;
	}

	return fileNameStr;
}

//載入網格誤差分析模組的[比對模型]
void load_comp_mesh(MeshComparison& cc)
{
	const char* loadfilter = "PLY Files (*.ply*)\0*.*\0";
	MessageBoxW(NULL, L"Please choose compared mesh file(.ply)", L"Load Compared Mesh File", MB_OK);
	string s_cmp_filename = opensavefilename((char*)loadfilter, NULL, true);
	const char* cmp_mesh_filename = s_cmp_filename.c_str();

	bool bLoadComp = cc.loadCompMesh(cmp_mesh_filename);
	if (bLoadComp)
	{
		lips::HandleTriMeshModel hand_lips_comp_mesh = cc.getCompMesh();

		if (hand_lips_comp_mesh)
		{
			
			pObj[0] = hand_lips_comp_mesh; //compared mesh & RGB distance rendering
			PrepareTransformation(pObj, 0);
			PrepareRenderBuf(pObj[0], MeshBuf[0]);
			bVisibleObj[0] = true; //set visible flag to true;
			RenderMode = 1; //only display mesh 

			int triNum = MeshBuf[0].TriNum;
			std::cout << "MeshComparison compared mesh has " << triNum << " triangles" << endl;

			std::cout << "Mesh Comparison load compared mesh successfully." << endl;
			std::cout << "Compared mesh file : " << cmp_mesh_filename << endl;
		}
	}
}

//載入網格誤差分析模組的[參照模型]
void load_ref_mesh(MeshComparison& cc)
{
	const char* loadfilter = "PLY Files (*.ply*)\0*.*\0";
	MessageBoxW(NULL, L"Please choose reference mesh file(.ply)", L"Load Reference Mesh File", MB_OK);
	string s_ref_filename = opensavefilename((char*)loadfilter, NULL, true);
	const char* ref_mesh_filename = s_ref_filename.c_str();

	bool bLoadRef = cc.loadRefMesh(ref_mesh_filename);
	if (bLoadRef)
	{
		lips::HandleTriMeshModel hand_lips_ref_mesh = cc.getRefMesh();

		if (hand_lips_ref_mesh)
		{
			std::cout << "Copy triangle mesh data to lips API finish!" << endl;

			
			pObj[1] = hand_lips_ref_mesh; //compared mesh & RGB distance rendering
			PrepareTransformation(pObj, 1);
			PrepareRenderBuf(pObj[1], MeshBuf[1]);
			bVisibleObj[1] = true; //set visible flag to true 
			RenderMode = 1; // only display mesh

			int triNum = MeshBuf[1].TriNum;
			std::cout << "MeshComparison reference mesh has " << triNum << " triangles" << endl;

			std::cout << "Mesh Comparison load reference mesh successfully." << endl;
			std::cout << "Referenced mesh file : " << ref_mesh_filename << endl;
		}
	}
}

//清除網格誤差分析模組的[比對模型]
void clear_comp_mesh(MeshComparison& cc)
{
	HandleTriMeshModel hmesh = cc.getCompMesh();
	if (hmesh)
	{
		ReleaseTriMeshModel(hmesh);
		ReleaseRenderBuf(MeshBuf[0]);
		pObj[0] = NULL;
		bVisibleObj[0] = false;
	}
	//
}

//清除網格誤差分析模組的[參照模型]
void clear_ref_mesh(MeshComparison& cc)
{
	HandleTriMeshModel hmesh = cc.getRefMesh();
	if (hmesh)
	{
		ReleaseTriMeshModel(hmesh);
		ReleaseRenderBuf(MeshBuf[1]);
		pObj[1] = NULL;
		bVisibleObj[1] = false;
	}
	//
}
//將網格模型容器參照第一個網格的中心點進行座標轉換
void ResetObjPositionAll()
{
	//process for each HandleTriMesh pointer
	int num_mesh_capcity = 0;

	for (int k = 0; k < 10; k++)
	{
		if (pObj[k]) //mesh pointer not null
		{
			int i;
			int vnum = 0;
			Point3f* vt = lips::GetVertexBuffer(pObj[k], &vnum);

			float x1 = FLT_MAX, x2 = FLT_MIN, y1 = FLT_MAX, y2 = FLT_MIN, z1 = FLT_MAX, z2 = FLT_MIN;
			for (i = 0; i < vnum; i++)
			{
				float x, y, z;
				lips::GetPoint3fComp(vt, i, x, y, z);
				if (x1 > x) { x1 = x; }
				if (x2 < x) { x2 = x; }
				if (y1 > y) { y1 = y; }
				if (y2 < y) { y2 = y; }
				if (z1 > z) { z1 = z; }
				if (z2 < z) { z2 = z; }
			}

			Center[k][0] = (x1 + x2) / 2;
			Center[k][1] = (y1 + y2) / 2;
			Center[k][2] = (z1 + z2) / 2;

			Pivot[k][0] = Center[k][0];
			Pivot[k][1] = Center[k][1];
			Pivot[k][2] = Center[k][2];

			double max_dis = 0;
			if (k == 0)
			{
				if (max_dis < (Center[k][0] + 100)) { max_dis = Center[k][0] + 100; }
				if (max_dis < (Center[k][1] + 100)) { max_dis = Center[k][1] + 100; }
				if (max_dis < (Center[k][2] + 100)) { max_dis = Center[k][2] + 100; }

				if (max_dis < abs(x1 - Center[k][0])) { max_dis = abs(x1 - Center[k][0]); }
				if (max_dis < abs(x2 - Center[k][0])) { max_dis = abs(x2 - Center[k][0]); }
				if (max_dis < abs(y1 - Center[k][1])) { max_dis = abs(y1 - Center[k][1]); }
				if (max_dis < abs(y2 - Center[k][1])) { max_dis = abs(y2 - Center[k][1]); }
				if (max_dis < abs(z1 - Center[k][2])) { max_dis = abs(z1 - Center[k][2]); }
				if (max_dis < abs(z2 - Center[k][2])) { max_dis = abs(z2 - Center[k][2]); }
				max_dis = sqrt(max_dis*max_dis * 3);
				Vx1 = x1; Vx2 = x2;
				Vy1 = y1; Vy2 = y2;
				Vz1 = -max_dis; Vz2 = max_dis;
				ScaleObj = 0.8;
			}
			else
			{
				Center[k][0] = Center[0][0];
				Center[k][1] = Center[0][1];
				Center[k][2] = Center[0][2];
			}
			Vz1 = -5000; Vz2 = 5000;


			int Tnum = 0;
			Index3i* tri = GetTriangleBuffer(pObj[k], &Tnum);
			Point3f *vn = GetNormalBuffer(pObj[k]);
			if (vn == NULL)
			{
				GenerateNormal(pObj[k]);
			}

			ColorRGB *vc = GetColorBuffer(pObj[k]);

			Trans[k][0] = 0; Trans[k][1] = 0; Trans[k][2] = 0;
			Scale[k][0] = 1; Scale[k][1] = 1; Scale[k][2] = 1;

			Roate[k][0] = 1; Roate[k][1] = 0; Roate[k][2] = 0;
			Roate[k][3] = 0; Roate[k][4] = 1; Roate[k][5] = 0;
			Roate[k][6] = 0; Roate[k][7] = 0; Roate[k][8] = 1;

			BasePointNum[k] = 0;
		}
	}
	IsLighting = true;
	SetLights(m_pLight);
	glEnable(GL_LIGHTING);
}

//判斷游標選取的網格
bool SelectTriangleAll(int px, int py, HandleTriMeshModel* Meshptr, bool* VisiblePtr, int MeshNum, int *idxNode, int *idxTri, int *idxEdge)
{
	int i, k, max, min, Pnum, Tnum, Triangle, idxObj;
	int *ImgX, *ImgY;
	double Ts[3];
	float dx1, dx2, dy1, dy2, d, p, s, RatioX, RatioY, OffsetX, OffsetY, Cnt[3];
	double Rot[9];
	float x, y, z, z1, z2, z3, Depth;
	//TriangleMesh *pMesh;
	Point3f *p3D;
	Index3i *tIdx = NULL;

	int m_Viewport_left = 0;
	int m_Viewport_bottom = 0;
	int m_Viewport_right = iWidth;
	int m_Viewport_top = iHeight;

	double m_ViewVolume_x1 = (-(iWidth / 2) + CameraTrans[0]) / ScaleObj;
	double m_ViewVolume_x2 = ((iWidth / 2) + CameraTrans[0]) / ScaleObj;
	double m_ViewVolume_y1 = (-iHeight / 2 + CameraTrans[1]) / ScaleObj;
	double m_ViewVolume_y2 = (iHeight / 2 + CameraTrans[1]) / ScaleObj;

	float Piv[3], Tns[3], Scl[3];
	double m_RotMatrix[9];
	double m_Center[3];
	Piv[0] = Pivot[0][0]; Piv[1] = Pivot[0][1]; Piv[2] = Pivot[0][2];
	Tns[0] = Trans[0][0]; Tns[1] = Trans[0][1]; Tns[2] = Trans[0][2];
	Scl[0] = Scale[0][0]; Scl[1] = Scale[0][1]; Scl[2] = Scale[0][2];

	m_RotMatrix[0] = CameraRotate[0]; m_RotMatrix[1] = CameraRotate[1];	m_RotMatrix[2] = CameraRotate[2];
	m_RotMatrix[3] = CameraRotate[3]; m_RotMatrix[4] = CameraRotate[4];	m_RotMatrix[5] = CameraRotate[5];
	m_RotMatrix[6] = CameraRotate[6]; m_RotMatrix[7] = CameraRotate[7];	m_RotMatrix[8] = CameraRotate[8];

	m_Center[0] = Center[0][0];
	m_Center[1] = Center[0][1];
	m_Center[2] = Center[0][2];

	max = 0;
	for (i = 0;i < MeshNum;i++)
	{
		if (Meshptr[i] && VisiblePtr[i])
		{
			Pnum = 0;
			Point3f* ptr3D = lips::GetVertexBuffer(Meshptr[i], &Pnum);
			if (Pnum > max) max = Pnum;
		}
	}
	if (max == 0)
	{
		*idxNode = *idxTri = -1;
		return false;
	}
	ImgX = new int[max];
	ImgY = new int[max];
	if (ImgY == NULL) //allocate failure
	{
		delete[] ImgX;
		*idxNode = *idxTri = -1;
		return false;
	}

	RatioX = (float)((m_Viewport_right - m_Viewport_left) / (m_ViewVolume_x2 - m_ViewVolume_x1));
	RatioY = (float)((m_Viewport_bottom - m_Viewport_top) / (m_ViewVolume_y2 - m_ViewVolume_y1));
	local_obj_centerx = OffsetX = (float)((m_Viewport_right + m_Viewport_left) / 2 - (m_ViewVolume_x2 + m_ViewVolume_x1)*RatioX / 2);
	local_obj_centery = OffsetY = (float)((m_Viewport_bottom + m_Viewport_top) / 2 - (m_ViewVolume_y2 + m_ViewVolume_y1)*RatioY / 2);

	Triangle = -1;
	idxObj = -1;
	for (i = 0;i < MeshNum;i++)
	{
		if (bVisibleObj[i] && Meshptr[i])
		{
			
			p3D = lips::GetVertexBuffer(Meshptr[i], &Pnum);
			tIdx = lips::GetTriangleBuffer(Meshptr[i], &Tnum);

			if (true)
			{
				Ts[0] = Piv[0] + Tns[0] - m_Center[0];
				Ts[1] = Piv[1] + Tns[1] - m_Center[1];
				Ts[2] = Piv[2] + Tns[2] - m_Center[2];
				Cnt[0] = float(Piv[0] - (Roate[0][0] * Ts[0] + Roate[0][3] * Ts[1] + Roate[0][6] * Ts[2]) / Scl[0]);
				Cnt[1] = float(Piv[1] - (Roate[0][1] * Ts[0] + Roate[0][4] * Ts[1] + Roate[0][7] * Ts[2]) / Scl[1]);
				Cnt[2] = float(Piv[2] - (Roate[0][2] * Ts[0] + Roate[0][5] * Ts[1] + Roate[0][8] * Ts[2]) / Scl[2]);
				Rot[0] = ((m_RotMatrix[0] * Roate[0][0] + m_RotMatrix[1] * Roate[0][3] + m_RotMatrix[2] * Roate[0][6])*Scl[0]);
				Rot[1] = ((m_RotMatrix[0] * Roate[0][1] + m_RotMatrix[1] * Roate[0][4] + m_RotMatrix[2] * Roate[0][7])*Scl[1]);
				Rot[2] = ((m_RotMatrix[0] * Roate[0][2] + m_RotMatrix[1] * Roate[0][5] + m_RotMatrix[2] * Roate[0][8])*Scl[2]);
				Rot[3] = ((m_RotMatrix[3] * Roate[0][0] + m_RotMatrix[4] * Roate[0][3] + m_RotMatrix[5] * Roate[0][6])*Scl[0]);
				Rot[4] = ((m_RotMatrix[3] * Roate[0][1] + m_RotMatrix[4] * Roate[0][4] + m_RotMatrix[5] * Roate[0][7])*Scl[1]);
				Rot[5] = ((m_RotMatrix[3] * Roate[0][2] + m_RotMatrix[4] * Roate[0][5] + m_RotMatrix[5] * Roate[0][8])*Scl[2]);
				Rot[6] = ((m_RotMatrix[6] * Roate[0][0] + m_RotMatrix[7] * Roate[0][3] + m_RotMatrix[8] * Roate[0][6])*Scl[0]);
				Rot[7] = ((m_RotMatrix[6] * Roate[0][1] + m_RotMatrix[7] * Roate[0][4] + m_RotMatrix[8] * Roate[0][7])*Scl[1]);
				Rot[8] = ((m_RotMatrix[6] * Roate[0][2] + m_RotMatrix[7] * Roate[0][5] + m_RotMatrix[8] * Roate[0][8])*Scl[2]);
			}

			for (k = 0;k < Pnum;k++)
			{
				float x0, y0, z0;
				lips::GetPoint3fComp(p3D, k, x0, y0, z0);
				x = x0 - Cnt[0];
				y = y0 - Cnt[1];
				z = z0 - Cnt[2];
				ImgX[k] = int((Rot[0] * x + Rot[1] * y + Rot[2] * z)*RatioX + OffsetX + 0.5f);
				ImgY[k] = int((Rot[3] * x + Rot[4] * y + Rot[5] * z)*RatioY + OffsetY + 0.5f);
			}
			for (k = 0;k < Tnum;k++)
			{
				int v1, v2, v3;
				lips::GetIndex3iComp(tIdx, k, v1, v2, v3);
				max = min = ImgX[v1];
				if (ImgX[v2] > max) max = ImgX[v2];
				if (ImgX[v3] > max) max = ImgX[v3];
				if (px > max) continue;
				if (ImgX[v2] < min) min = ImgX[v2];
				if (ImgX[v3] < min) min = ImgX[v3];
				if (px < min) continue;
				max = min = ImgY[v1];
				if (ImgY[v2] > max) max = ImgY[v2];
				if (ImgY[v3] > max) max = ImgY[v3];
				if (py > max) continue;
				if (ImgY[v2] < min) min = ImgY[v2];
				if (ImgY[v3] < min) min = ImgY[v3];
				if (py < min) continue;
				//Is this point inside the triangle
				dx1 = float(ImgX[v2] - ImgX[v1]);
				dy1 = float(ImgY[v2] - ImgY[v1]);
				dx2 = float(ImgX[v3] - ImgX[v1]);
				dy2 = float(ImgY[v3] - ImgY[v1]);
				d = dx1 * dy2 - dx2 * dy1;
				if (d == 0.f) continue;
				p = (dy2*(px - ImgX[v1]) - dx2 * (py - ImgY[v1])) / d;
				if (p < 0.f) continue;
				if (p > 1.f) continue;
				s = (-dy1 * (px - ImgX[v1]) + dx1 * (py - ImgY[v1])) / d;
				if (s < 0.f) continue;
				if (s > 1.f) continue;
				if (p + s > 1.f) continue;
				lips::GetPoint3fComp(p3D, v1, x, y, z);
				x = x - Cnt[0];
				y = y - Cnt[1];
				z = z - Cnt[2];
				z1 = (float)(Rot[6] * x + Rot[7] * y + Rot[8] * z);
				lips::GetPoint3fComp(p3D, v2, x, y, z);
				x = x - Cnt[0];
				y = y - Cnt[1];
				z = z - Cnt[2];
				z2 = (float)(Rot[6] * x + Rot[7] * y + Rot[8] * z);
				lips::GetPoint3fComp(p3D, v3, x, y, z);
				x = x - Cnt[0];
				y = y - Cnt[1];
				z = z - Cnt[2];
				z3 = (float)(Rot[6] * x + Rot[7] * y + Rot[8] * z);
				z = (float)(z1 + p * (z2 - z1) + s * (z3 - z1));
				if ((Triangle < 0) || ((Triangle >= 0) && (z > Depth)))
				{
					Triangle = k;
					idxObj = i;
					Depth = z;
					if (idxEdge)
					{
						if ((s < p) && (s < 1.f - p - s)) *idxEdge = 1;
						else if ((p < s) && (p < 1.f - p - s)) *idxEdge = 3;
						else *idxEdge = 2;
					}
				}
			}
		}
	}
	//std::cout << preindex_Triangle << ":" << Tnum << std::endl;
	if (preindex_Triangle >= 0 && preindex_Triangle <= Tnum)
	{
		int v1, v2, v3;
		lips::GetIndex3iComp(tIdx, preindex_Triangle, v1, v2, v3);
		pre_seltirpos_x = (ImgX[v1] + ImgX[v2] + ImgX[v3]) / 3;
		pre_seltirpos_y = (ImgY[v1] + ImgY[v2] + ImgY[v3]) / 3;
	}

	*idxNode = idxObj;
	*idxTri = Triangle;

	if (Triangle >= 0)
	{
		int v1, v2, v3;
		lips::GetIndex3iComp(tIdx, *idxTri, v1, v2, v3);
		seltripos_x = (ImgX[v1] + ImgX[v2] + ImgX[v3]) / 3.0;
		seltripos_y = (ImgY[v1] + ImgY[v2] + ImgY[v3]) / 3.0;
	}

	delete[] ImgX;
	delete[] ImgY;
	if (Triangle >= 0)  return true;
	else return false;
}

//參照游標移動,轉換網格誤差分析物件的位置
void TranslateMeshCompareObj(float m_x, float m_y, float m_z)
{
	for (int idx = 0;idx < MeshNum; idx++)
	{
		if (bTransformObj[idx])
		{
			double R[9];
			float T[3];
			float C[3];
			float S[3];
			R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
			R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
			R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;
			T[0] = (m_x)*CameraRotate[0] + (-m_y)*CameraRotate[3];
			T[1] = (m_x)*CameraRotate[1] + (-m_y)*CameraRotate[4];
			T[2] = (m_x)*CameraRotate[2] + (-m_y)*CameraRotate[5];
			S[0] = S[1] = S[2] = 1.0;
			C[0] = C[1] = C[2] = 0.0;
#ifndef _DEBUG
			std::cout << "Tx=" << T[0] << endl;
			std::cout << "Ty=" << T[1] << endl;
			std::cout << "Tz=" << T[2] << endl;
#endif
			lipsApplyTransformMesh(pObj[idx], R, T, C, S);
			ReleaseRenderBuf(MeshBuf[idx]);
			PrepareRenderBuf(pObj[idx], MeshBuf[idx]);
		}
	}
	//End
}

//參照游標旋轉,轉換網格誤差分析物件的位置
void RotateMeshCompareObj(float m_x, float m_y, float m_z)
{
	int idx;

	if ((m_x == 0) && (m_y == 0) && (m_z == 0))
		return;

	for (idx = 0;idx < MeshNum;idx++)
	{
		if (bTransformObj[idx])
		{
			double l, m, n, angle, rot[9];

			l = m_x;
			m = m_y;
			n = m_z;
			angle = sqrt(l*l + m * m + n * n);
			l /= angle; m /= angle; n /= angle;

			double R[9];
			float T[3];
			float C[3];
			float S[3];
			R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
			R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
			R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;

			T[0] = T[1] = T[2] = 0.0;
			C[0] = C[1] = C[2] = 0.0;
			S[0] = S[1] = S[2] = 1.0;

			GetRotMatrix(rot, l, m, n, angle);
			memcpy(Roate_tmp, R, sizeof(double) * 9);

			R[0] = rot[0] * Roate_tmp[0] + rot[1] * Roate_tmp[3] + rot[2] * Roate_tmp[6];
			R[1] = rot[0] * Roate_tmp[1] + rot[1] * Roate_tmp[4] + rot[2] * Roate_tmp[7];
			R[2] = rot[0] * Roate_tmp[2] + rot[1] * Roate_tmp[5] + rot[2] * Roate_tmp[8];
			R[3] = rot[3] * Roate_tmp[0] + rot[4] * Roate_tmp[3] + rot[5] * Roate_tmp[6];
			R[4] = rot[3] * Roate_tmp[1] + rot[4] * Roate_tmp[4] + rot[5] * Roate_tmp[7];
			R[5] = rot[3] * Roate_tmp[2] + rot[4] * Roate_tmp[5] + rot[5] * Roate_tmp[8];
			R[6] = rot[6] * Roate_tmp[0] + rot[7] * Roate_tmp[3] + rot[8] * Roate_tmp[6];
			R[7] = rot[6] * Roate_tmp[1] + rot[7] * Roate_tmp[4] + rot[8] * Roate_tmp[7];
			R[8] = rot[6] * Roate_tmp[2] + rot[7] * Roate_tmp[5] + rot[8] * Roate_tmp[8];
			NormalizeRotMatrix(R);

			//computer mesh center(x,y,z)
			int pNum;
			Point3f* pBuf = lips::GetVertexBuffer(pObj[idx], &pNum);
			VERTEX3F pAvg;
			float x, y, z;
			pAvg.x = pAvg.y = pAvg.z = 0.0;
			for (int i = 0; i < pNum; i++)
			{
				lips::GetPoint3fComp(pBuf, i, x, y, z);
				pAvg.x += x;
				pAvg.y += y;
				pAvg.z += z;
			}
			C[0] = pAvg.x / pNum;
			C[1] = pAvg.y / pNum;
			C[2] = pAvg.z / pNum;

			lipsApplyTransformMesh(pObj[idx], R, T, C, S);
			ReleaseRenderBuf(MeshBuf[idx]);
			PrepareRenderBuf(pObj[idx], MeshBuf[idx]);
		}
	}

}

//參照指定z軸高度,縮放網格物件的點雲座標
bool ResizeMeshObj(HandleTriMeshModel hmesh, float z_height)
{
	double R[9];
	float T[3];
	float C[3];
	float S[3];
	R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
	R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
	R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;

	T[0] = T[1] = T[2] = 0.0;
	C[0] = C[1] = C[2] = 0.0;
	S[0] = S[1] = S[2] = 1.0;

	int pNum;
	Point3f* pBuf = lips::GetVertexBuffer(hmesh, &pNum);
	float x, y, z;
	float min_x, min_y, min_z;
	float max_x, max_y, max_z;
	float cen_x, cen_y, cen_z;
	cen_x = cen_y = cen_z = 0;
	for (int i = 0; i < pNum; i++)
	{
		lips::GetPoint3fComp(pBuf, i, x, y, z);
		cen_x += x;
		cen_y += y;
		cen_z += z;
	}
	cen_x /= pNum;
	cen_y /= pNum;
	cen_z /= pNum;
	max_x = min_x = cen_x;
	max_y = min_y = cen_y;
	max_z = min_z = cen_z;

	for (int i = 0; i < pNum; i++)
	{
		lips::GetPoint3fComp(pBuf, i, x, y, z);

		if (min_x > x) min_x = x;
		if (min_y > y) min_y = y;
		if (min_z > z) min_z = z;

		if (max_x < x) max_x = x;
		if (max_y < y) max_y = y;
		if (max_z < z) max_z = z;
	}

	float z_len = max_z - min_z;
	float z_scale = z_height / z_len;

	C[0] = cen_x;
	C[1] = cen_y;
	C[2] = cen_z;
	S[0] = S[1] = S[2] = z_scale;

	return lipsApplyTransformMesh(hmesh, R, T, C, S);;
}

#endif
#pragma endregion

// 執行程式: Ctrl + F5 或 [偵錯] > [啟動但不偵錯] 功能表
// 偵錯程式: F5 或 [偵錯] > [啟動偵錯] 功能表

// 開始使用的提示: 
//   1. 使用 [方案總管] 視窗，新增/管理檔案
//   2. 使用 [Team Explorer] 視窗，連線到原始檔控制
//   3. 使用 [輸出] 視窗，參閱組建輸出與其他訊息
//   4. 使用 [錯誤清單] 視窗，檢視錯誤
//   5. 前往 [專案] > [新增項目]，建立新的程式碼檔案，或是前往 [專案] > [新增現有項目]，將現有程式碼檔案新增至專案
//   6. 之後要再次開啟此專案時，請前往 [檔案] > [開啟] > [專案]，然後選取 .sln 檔案
