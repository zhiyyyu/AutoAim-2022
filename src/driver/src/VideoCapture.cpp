//
// Created by zhiyu on 2021/8/20.
//

#include "../include/VideoCapture.h"
#include "SerialPort.h"

using namespace ly;
using namespace cv;
extern std::chrono::steady_clock::time_point TX2_BeginTime;
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

void DaHenCamera::open() {
    camera->initLib();
    camera->openDevice(CameraParam::sn.c_str());
    camera->setRoiParam(width, height, offset_x, offset_y);
    //1000,3000,5,16,127
    camera->setExposureGainParam( false, false, CameraParam::exposure_time, 1000, 3000, CameraParam::gain, 3, 10, 127,true);
    camera->setWhiteBalanceParam(true,GX_AWB_LAMP_HOUSE_ADAPTIVE);
    camera->acquisitionStart();
    
}

void DaHenCamera::startCapture(Params_ToVideo &params_to_video) {

    // params out
    _video_thread_params.frame_p = params_to_video.frame_p;
    _video_thread_params.time_stamp = params_to_video.time_stamp;

    int id = 0;
    constexpr int size = 10;
    Mat frame[size];
    for(auto & m : frame) m =  Mat(Size(1280, 1024), CV_32FC3);

    do {
        camera->ProcGetImage(&frame[id], _video_thread_params.time_stamp);
        *_video_thread_params.frame_p = &frame[id];
        id = (id+1) % size;
        LOG_IF(ERROR, (*_video_thread_params.frame_p)->empty()) << "get empty picture mat!";
    }
    while(!(*_video_thread_params.frame_p)->empty());
}

void* ly::saveFrameToNative(void *params_p) {
    auto params = reinterpret_cast<Params_ToVideo*>(params_p);
    const Mat& frame = **params->frame_p;
    VideoWriter& writer = params->writer;
    while(!frame.empty()){
        writer.write(frame);
    }
}

void ly::VideoCapture::startSave(Params_ToVideo &params_to_video) {
    // params in
    _video_thread_params.writer = writer;

    int nRet = pthread_create(&threadID2, nullptr, saveFrameToNative, static_cast<void*>(&_video_thread_params));
    LOG_IF(ERROR, nRet == -1) << "error in creating video writer thread!";

}

DaHenCamera::DaHenCamera() {
    camera = new GxCamera();
}

DaHenCamera::~DaHenCamera() {
    delete camera;
}

void NativeVideo::open() {
    this->video.open(CameraParam::video_path);
    LOG_IF(ERROR, !video.isOpened()) << "can't find video in " << CameraParam::video_path;
    rate = video.get(cv::CAP_PROP_FPS);
    DLOG(INFO) << "video fps: " << rate;
}

void NativeVideo::startCapture(Params_ToVideo& params){
    // params in
    _video_thread_params.video = this->video;
    _video_thread_params.__this = this;

    // params out
    _video_thread_params.frame_p = params.frame_p;

    _video_thread_params.video >> **(_video_thread_params.frame_p);
    LOG_IF(ERROR, (*_video_thread_params.frame_p)->empty()) << "get empty picture mat!";

    int id = 0;
    constexpr int size = 10;
    Mat frame[size];
    for(auto & m : frame) m =  Mat(Size(1280, 1024), CV_32FC3);

    unique_lock<mutex> umtx_video(Thread::mtx_image, defer_lock);
    while(!(*_video_thread_params.frame_p)->empty()){
        umtx_video.lock();
        id = (id+1) % size;
        _video_thread_params.video >> frame[id];
        *_video_thread_params.frame_p = &frame[id];
        Thread::image_is_update = true;
        Thread::cond_is_update.notify_all();
        umtx_video.unlock();
        LOG_IF(ERROR, (*_video_thread_params.frame_p)->empty()) << "get empty picture mat!";
        usleep(5000);
    }
}

NativeVideo::NativeVideo() {
//    string save_path = CameraParam::video_path.substr(0, CameraParam::video_path.rfind('/') + 1);
//    FILE* fp = popen(("ls -l " + save_path + " |grep ^- | wc -l").c_str(), "r");
//    std::fscanf(fp, "%d", &_id);
//    pclose(fp);
//    writer = VideoWriter(save_path + DetectorParam::color + to_string(_id) + ".avi", VideoWriter::fourcc('M','P','4','2'), 200, Size(1280, 1024));
//    DLOG(INFO) << "save video in: " << save_path + DetectorParam::color + to_string(_id) + ".avi";
}

NativeVideo::~NativeVideo() {
    video.release();
//    writer.release();
}

void NativePicture::open() {

}

void NativePicture::startCapture(Params_ToVideo &params_to_video) {
    // 开启线程
    return;
}
