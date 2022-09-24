//
// Created by zhiyu on 2021/8/20.
//

#include "../include/VideoCapture.h"
#include "SerialPort.h"

using namespace ly;
using namespace cv;

ly::VideoCapture::VideoCapture() {
    width = 1280;
    height = 1024;
    offset_x = 0;
    offset_y = 0;
}

ly::VideoCapture::~VideoCapture() {

}

void ly::VideoCapture::chooseCameraType(ly::VideoCapture *& video) {
    switch (CameraParam::device_type) {
        case DaHen:
            video = new DaHenCamera();
            break;
        case Video:
            video = new NativeVideo();
            break;
        case Picture:
            video = new NativePicture();
            break;
        default:
            video = new NativeVideo();
            break;
    }
    video->open();
}