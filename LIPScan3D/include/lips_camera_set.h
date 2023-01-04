#pragma once

#ifdef LIPSCAMERASET_EXPORTS
#define LIPSCAMERASET_API __declspec(dllexport)
#else
#define LIPSCAMERASET_API __declspec(dllimport)
#endif

namespace lips
{
	typedef void* HandleCameraContext;
	//handle to Camera class member datas, it's only used in Class Constructor and member function.

	extern "C" LIPSCAMERASET_API bool SetCameraConfig(HandleCameraContext handle, const char* filename);
	//Set configurations of camera device context.
	//	[in] handle_context : file path of realsense advanced model.json file.
	//  [in] filename : configuration file of RealSense advance mode.
	//	[return] bool : True if Initialize Camera successfully, false otherwise.
}