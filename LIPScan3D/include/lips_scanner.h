#pragma once

#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <opencv\cvaux.h>
#include <opencv2\opencv.hpp>   
#include <opencv2\core\core.hpp>    
#include <opencv2\imgproc\imgproc.hpp>

#ifdef LIPSCANNER_EXPORTS
#define LIPSCANNER_API __declspec(dllexport)
#else
#define LIPSCANNER_API __declspec(dllimport)
#endif

namespace lips
{
	typedef void* HandleTriMeshModel;
	//handle to triangle mesh data returned from ScannerProcess::FinishScan()

	typedef void* HandleDepthmapProcessContext;
	//handle to DepthmapProcess class member datas, it's only used in Class Constructor and member function.

	typedef void* HandleScannerProcessContext;
	//handle to ScannerProcess class member datas, it's only used in Class Constructor and member function.

	class LIPSCANNER_API DepthmapProcess
	{
	public:
		HandleDepthmapProcessContext handle_context;
		//If delete or modify this pointer, member function calling will occur unexpected failure.

	public:
		DepthmapProcess(int w, int h, HandleScannerProcessContext handle_scanner);
		//Initialize resource of depth image post processing, if iWdth and iHeight is not correspond to Camera and ScannerProcess, the 3D reconstruction pipeline will occur unexpected failure.
		//	[in] iWidth : width of camera resolution.
		//	[in] iHeight : height of camera resolution.
		//	[in] handle_scanner : handle to ScannerProcess member handle_context adress.

		~DepthmapProcess();
	public:
		bool Process(unsigned short* depth_data, float* point_data, unsigned char* color_data);
		//This stage is consist of several image filtering to reduce noise¡Bremove outliers and depth range crop.To improve accuracy of point cloud registration.
		//	[in] depth_data : Pointer of depth image pixel array.
		//	[in] point_data : Pointer of 3D coordinate point array from calling convertDepthTo3DWorld().
		//	[out] depth_map : 8 bit depth image scaled from 16 bit depth image ¡¥depth_data¡¦.
		//	[return] True if process successfully, false otherwise.

		int DepthRangeEvaluation(unsigned short* depth_data, int step_num, int dx, int dy, RECT roi, int neglect_factor, float deepth_scale);
		//Evaluate depth value peak by compute histogram of depth image by specified parameters.
		//  [in] depth_data : Pointer of depth image pixel array.
		//  [in] step_num : Step count of histogram to evaluate peak depth value in depth image.
		//  [in] dx : Horizontal sampling rate to compute histogram.
		//  [in] dy : Vertical sampling rate to compute histogram.
		//  [in] roi : Region to compute histogram.
		//  [in] neglect_factor : Threshold to neglect irregular result.
		//  [in] depth_scale : Unit of depth_data x depth = Unit of 3D reconstruction, deepth_scale = 1.0 for Realsense camera & 0.1 for Himax camera.
		//  [return] Index of histogram peak value in depth image.
	};

	struct ScannerConfigParam;
	//Store configurations of ScannerProcess class
	//It's only used in ScannerProcess calculations.

	enum ScannerConfigParam_FloatMember
	{
		VolumePitch,
		DepthAccuracy,
		SearchRange,
		ColorSpreadRange,
		RenderingPitch,
		RegistrationMinimalPoints,
		RegistrationMaxDeviation,
		RegistrationMinDeviation
	};
	//Float member of ScannerConfigParam

	enum ScannerConfigParam_IntMember
	{
		WorkerThreads,
		DepthRangeMin,
		DepthRangeMax,
		PostProcAvgTimes,
		AverageTimes,
		PeelOffTimes,
		RemoveDepthDetail,
		RemoveNoiseByRgbContour,
		SearchBiggerObjectReserve,
		CameraFrameInterval,
		RecoderFrameInterval
	};
	//Integer member of ScannerConfigParam

	enum RangeParam_lib
	{
		RangeParam_lib_SMALL,
		RangeParam_lib_MEDIUM,
		RangeParam_lib_LARGE
	};
	//Define 3 value corresponding 3 scannig distance
	//      Parameter        Scanning distance
	//RangeParam_lib_SMALL           D<15cm
	//RangeParam_lib_MEDIUM     20cm<D<45cm
	//RangeParam_lib_LARGE      25cm<D<120cm

	enum ScannerDeviceType
	{
		None,
		IntelRealsense_D415,
		IntelRealsense_D435,
		IntelRealsense_D435I,
		IntelRealsense_D455,
		LIPS_L210
	};
	//Define 3 value corresponding 3 scanner device
	//      Parameter              Camera device
	//IntelRealsense_D415      Intel RealSense D415
	//IntelRealsense_D435      Intel RealSense D435
	//IntelRealsense_D435I     Intel RealSense D435I
	//IntelRealsense_D455      Intel RealSense D455
	//Himax_SH430              Himax SH430

	class LIPSCANNER_API ScannerProcess
	{
	public:
		HandleScannerProcessContext handle_context;
		//If delete or modify this pointer, member function calling will occur unexpected failure.

	public:
		ScannerProcess(void);
		virtual ~ScannerProcess();

		bool Init(int imageWidth, int imageHeight, const char* config_filename, const char* serial_number);
		//Initialize configuration of Scanner library.
		//	[in] imageWidth: width of depth image resolution.
		//	[in] imageHeight : height of depth image resolution.
		//	[in] config_filename : configuration file path for scanner library API.
		//  [in] serial_number : serial number of connected camera.

		bool Init(int imageWidth, int imageHeight, ScannerDeviceType device, RangeParam_lib range, const char* serial_number);
		//Initialize configuration of Scanner library with camera device type
		//	[in] imageWidth: width of depth image resolution.
		//	[in] imageHeight : height of depth image resolution.
		//	[in] device : camera deivce type
		//  [in] range : specify range about distance between object and scanner

		bool ScanData(unsigned char* color_map, float* points_data, int width, int height, unsigned char* ScanImgBuf);
		//Calculate point cloud registration and update accumulated point cloud of reconstruction.
		//	[in] color_map: color texture mapping to points_data.
		//	[in] points_data : point cloud input to registration process.
		//	[in] width : width of depth image of camera.
		//	[in] height : height of depth image of camera.
		//	[out] ScanImgBuf : image of accumulated registered object surface.
		//	[return] True if current registration successfully.

		HandleTriMeshModel FinishScan();
		//This should be call after scanning whole surface of object, point cloud accumulated by registration will assign to triangle mesh class pointer.
		//	[return] If 3D reconstruction successfully.Return address to triangle mesh model data, otherwise return null address.

	};

	extern "C" LIPSCANNER_API bool ExportFile(char *filename, HandleTriMeshModel handle_obj, int encode);
	//Export triangle mesh model to specified file format.
	//	[in] filename: file path to export triangle mesh model.
	//	[in] handle_obj : handle to triangle mesh model address, an available handle is from calling ScannerProcess::FinishScan().
	//	[in] encode : file format for export.Supported as below.
	//	[return] True if export successfully, false otherwise.
	//
	//file format            encode
	//ASCII STL	                0
	//Binary STL                1
	//ASCII PLY	                2
	//Binary PLY big endian     3
	//Binary PLY little endian  4
	//	OBJ	                    5
	//  TGL                     6

	extern "C" LIPSCANNER_API HandleTriMeshModel ImportFile(char *filename, int& model_format);

	//Import triangle mesh model from specified file.
	//	[in] filename: file path to import triangle mesh model.
	//	[in] model_format : encode for file format.
	//	[return] If import file successfully.Return handle to triangle mesh model data, otherwise return null handle.
	//
	//file format            encode
	//ASCII STL	                0
	//Binary STL                1
	//ASCII PLY	                2
	//Binary PLY big endian     3
	//Binary PLY little endian  4
	//	OBJ	                    5
	//  TGL                     6
}


