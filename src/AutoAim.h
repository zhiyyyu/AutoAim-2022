//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_AUTOAIM_H
#define AUTOAIM_AUTOAIM_H

#include "utils/include/Config.h"
#include "utils/include/Log.h"
#include "driver/include/VideoCapture.h"
#include "driver/include/VideoSaver.h"
#include "armor_detector/include/Detector.h"
#include "Thread.h"
#include "trainSVM.h"

#include <string>

using namespace std;
using namespace ly;

namespace ly{
    const string confog_file_path = "../src/utils/tools/init.json";
}

typedef struct{
    cv::Mat mat;
    SerialPortData receiveData_;
    std::chrono::steady_clock::time_point Camtime_start; //获取照片读取的时间戳
    clock_t time;
} ly_Mat;

#endif //AUTOAIM_AUTOAIM_H
