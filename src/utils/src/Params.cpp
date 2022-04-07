//
// Created by zhiyu on 2021/8/20.
//
#include "../include/Params.h"
#include "cmath"
using namespace std;

namespace ly{
    int CameraParam::device_type;
    string CameraParam::sn;
    string CameraParam::picture_path;
    string CameraParam::video_path;
    int CameraParam::exposure_time;
    float CameraParam::gamma;
    double CameraParam::gain;
    double CameraParam::fx;
    double CameraParam::fy;
    double CameraParam::u0;
    double CameraParam::v0;
    double CameraParam::k1;
    double CameraParam::k2;
    double CameraParam::k3;
    double CameraParam::p1;
    double CameraParam::p2;
    double CameraParam::camera_trans_x;
    double CameraParam::camera_trans_y;
    double CameraParam::camera_trans_z;

    string DetectorParam::color;
    int DetectorParam::thresh;

    string SerialParam::device_name;
    SerialPortData SerialParam::recv_data;
    SerialPortWriteData SerialParam::send_data;
    
    bool GlobalParam::DEBUG_MODE;
    bool GlobalParam::SAVE_VIDEO;
    bool GlobalParam::SAVE_ARMOR;
    int GlobalParam::save_step;
    bool GlobalParam::SHOW_THRESH;
    bool GlobalParam::SHOW_COORD;
}