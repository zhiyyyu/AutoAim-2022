//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_PARAMS_H
#define AUTOAIM_PARAMS_H

#include <string>
#include "../../driver/include/SerialPort.h"
#include "opencv2/opencv.hpp"

using namespace std;

namespace ly{
    class CameraParam{
    public:
        static int device_type;
        static string sn;
        static string video_path;
        static string picture_path;
        static int exposure_time;
        static double gain;
        static float gamma;
        
        static double fx; 
        static double fy;
        static double u0;
        static double v0;
        static double k1;
        static double k2;
        static double k3;
        static double p1;
        static double p2;
        static double camera_trans_x;
        static double camera_trans_y;
        static double camera_trans_z;
    };

    class DetectorParam{
    public:
        static string color;
        static int thresh;
    };

    class SerialParam{
    public:
        static bool enable;
        static string device_name;
        static SerialPortData recv_data;
        static SerialPortWriteData send_data;
    };

    class GlobalParam{
    public:
        static bool DEBUG_MODE;
        static bool SAVE_VIDEO;
        static bool SAVE_ARMOR;
        static bool SHOW_THRESH;
        static int save_step;
        static bool SHOW_COORD;
    };
}

#endif //AUTOAIM_PARAMS_H
