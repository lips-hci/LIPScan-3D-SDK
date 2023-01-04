#pragma once

namespace lips
{
    typedef struct Rect
    {
        int x;
        int y;
        int width;
        int height;
    };

    class IDepthCam
    {
    public:
        virtual ~IDepthCam() {};

        virtual bool InitCamera(const char*) = 0;
        virtual bool GetFrame(unsigned char *, unsigned short *) = 0;
		virtual const char* GetSerialNumber(void) = 0;
        //virtual const bool checkCameraModel_AE400(void);
        //virtual const char* getDeviceName(int& device_slot, rs2::device_list& device_list, rs2::context& rs2_context);
        //virtual int getDeviceSlot(rs2::device_list& device_list, rs2::context& rs2_context);
        //virtual int loadCameraConfig(rs2::device& device, int& device_slot, rs2::device_list& device_list, rs2::context& rs2_context, const char* filename);
        //virtual rs2_stream findStream2Align(const std::vector<rs2::stream_profile>& streams);

    private:

    };
}

typedef lips::IDepthCam* (*CREATE_CAMERA)(int&, char*);

typedef void* (*DELETE_CAMERA)(lips::IDepthCam*);

typedef void* (*CONVERTDEPTHTO3DWORLD)(float*, lips::IDepthCam*, lips::Rect);