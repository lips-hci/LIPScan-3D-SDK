#pragma once

#ifdef LIPSCAMERA_EXPORTS
#define LIPSCAMERA_API __declspec(dllexport)
#else
#define LIPSCAMERA_API __declspec(dllimport)
#endif

#include <iostream>
#include <fstream>
#include <map>

#include "IDepthCam.h"

using namespace std;

namespace lips
{
	typedef struct Coefficient;
	//Store configurations of Camera class
	//It's only used in Camera Initialization.

	typedef void* HandleCameraContext;
	//handle to Camera class member datas, it's only used in Class Constructor and member function.

	class LIPSCAMERA_API Camera
	{
	public:
		HandleCameraContext hanlde_context;
	public:
		Camera(int&, char*);
		//This constructor is used to check camera connection.If deviceslot is - 1, then application can not get live image from API.
		//	[out] deviceslot: realsense device content index, if device query successfully, deviceslot is 0, otherwise - 1.
		//	[out] devicename : if device query successfully, devicename is ¡§Realsense D415¡¨, otherwise invalid pointer.

		Camera();
		~Camera();
		bool InitCamera(const char*);
		//Initialize configurations operation, enable realsense advanced mode and camera color and depth stream, then start camera pipeline and align stream.
		//	[in] filename: file path of realsense advanced model.json file.
		//	[return] True if Initialize Camera successfully, false otherwise.

		bool GetFrame(unsigned char *, unsigned short *);
		//Get camera color and depth frame from image stream, store pixel data in basic type pointer.
		//	[out] color_data : It is duplicate of camera color frame with alignment.
		//	[out] depth_data : It is duplicate of camera depth frame with alignment.
		//	[return] True if color_data and depth_data are both available, false otherwise.

		const char* GetSerialNumber(void);
		//Get serial number string of connecting device
		//  [return] constant string of serial number, null ponter if no connecting device.

		bool IsValidLicense(void);

	public:
        HINSTANCE hDLL;
        IDepthCam *my_cam;
	};

	extern "C" LIPSCAMERA_API void convertDepthTo3DWorld(float* points_buffer, Camera* camera, Rect roi);
	//Generate the pointcloud and texture mappings with 3D coordinate transform, apply ROI crop point cloud to remove background data.
	//	[in] camera: pointer of Camera class object streamed to AE400.
	//	[in] roi : Depth map crop region.In normal situation use default Rect.
	//	[out] points_buffer : point cloud after coordinates transformation.
}