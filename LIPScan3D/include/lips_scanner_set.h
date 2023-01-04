#pragma once

#ifdef LIPSCANNERSET_EXPORTS
#define LIPSCANNERSET_API __declspec(dllexport)
#else
#define LIPSCANNERSET_API __declspec(dllimport)
#endif

namespace lips
{
	enum RangeParam_lib;
	//[Forward Declaration]
	//Specify parameters of initializations corresponding 3 scanning distance

	enum ScannerDeviceType;
	//Define 3 value corresponding 3 scanner device
	//      Parameter              Camera device
	//ScannerDeveice_lib_D415   Intel RealSense D415
	//ScannerDeveice_lib_D435I  Intel RealSense D435I
	//ScannerDeveice_lib_D455   Intel RealSense D455

	typedef void* HandleDepthmapProcessContext;
	//handle to DepthmapProcess class member datas, it's only used in Class Constructor and member function.

	typedef void* HandleScannerProcessContext;
	//handle to ScannerProcess class member datas, it's only used in Class Constructor and member function.

	typedef void* HandleScannerConfigParam;
	//handle to ScannerConfigParam struct, it's only used in generate and set scanner configuration parameters.

	extern "C" LIPSCANNERSET_API void SetRemoveDepthDetail(HandleDepthmapProcessContext handle_context, bool flag);
	//Enable depth image post-processing, remove noise and preserving feature by opencv library
	//[in] handle_context, handle to DepthmapProcess class member datas.
	//[in] flag, enable post-processing.

	extern "C" LIPSCANNERSET_API void SetRemoveExternalNoise(HandleDepthmapProcessContext handle_context, int threshold);
	//Enable depth image post-processing, remove non-objcet noise in background by opencv library
	//[in] handle_context, handle to DepthmapProcess class member datas.
	//[in] flag, enable post-processing.

	extern "C" LIPSCANNERSET_API void SetRemoveNearObjectNoise(HandleDepthmapProcessContext handle_context, int threshold);
	//Enable depth image post-processing, remove object connected noises by opencv library
	//[in] handle_context, handle to DepthmapProcess class member datas.
	//[in] flag, enable post-processing.

	extern "C" LIPSCANNERSET_API HandleScannerConfigParam GanerateScannerConfigParam(RangeParam_lib range, ScannerDeviceType device);
	//Generate Parameters with specified device type and scanning distance.
	//[in] range, scanning depth range limitation of depth images.
	//[in] device, type of connected Realsense device.
	//[return] Parameters for depth range¡B3D reconstruction and mesh postprocessing.

	extern "C" LIPSCANNERSET_API bool SetScannerConfig(HandleScannerProcessContext handle_context, HandleScannerConfigParam config);
	//Set Parameters to ScannerProcess context.
	//[in] handle_context, handle of ScannerProcess class member data.
	//[in] config, configurations to set in.
	//[return] True if set configurations successfully.

	extern "C" LIPSCANNERSET_API bool SetScannerConfigFloatMember(HandleScannerProcessContext handle_context, ScannerConfigParam_FloatMember config_para, float value);
	//Set Parameters to ScannerProcess context.
	//[in] handle_context, handle to ScannerProcess class member data.
	//[in] config_para, specify float parameter in scanner configuration.
	//[in] value, specify value of corresponding float parameter in scanner configuration.
	//[return] True if set configurations successfully.

	extern "C" LIPSCANNERSET_API bool SetScannerConfigIntMember(HandleScannerProcessContext handle_context, ScannerConfigParam_IntMember config_para, int value);
	//Set Parameters to ScannerProcess context.
	//[in] handle_context, handle to ScannerProcess class member data.
	//[in] config_para, specify integer parameter in scanner configuration.
	//[in] value, specify value of corresponding integer parameter in scanner configuration.
	//[return] True if set configurations successfully.

	extern "C" LIPSCANNERSET_API float GetScannerConfigFloatMember(HandleScannerProcessContext handle_context, ScannerConfigParam_FloatMember config_para);
	//Get Parameter from ScannerProcess context.
	//[in] handle_context, handle to ScannerProcess class member data.
	//[in] config_para, specified float parameter in configurations.
	//[return] value of specified float parameter.

	extern "C" LIPSCANNERSET_API int GetScannerConfigIntMember(HandleScannerProcessContext handle_context, ScannerConfigParam_IntMember config_para);
	//Get Parameter from ScannerProcess context.
	//[in] handle_context, handle to ScannerProcess class member data.
	//[in] config_para, specified integer parameter in configurations.
	//[return] value of specified integer parameter.


}
