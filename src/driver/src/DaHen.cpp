#include "VideoCapture.h"

using namespace ly;
using namespace cv;

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
    _video_thread_params.frame_pp = params_to_video.frame_pp;

    int id = 0;
    constexpr int size = 10;
    // Mat frame[size];
    // for(auto & m : frame) m =  Mat(Size(1280, 1024), CV_32FC3);
    Image frame[size];
    for(auto& m: frame) m.mat = new Mat(Size(1280, 1024), CV_32FC3);
    std::chrono::steady_clock::time_point start, end;

    do {
        unique_lock<mutex> umtx_video(Thread::mtx_image);
        while(Thread::image_is_update){ Thread::cond_is_process.wait(umtx_video); }
        start = std::chrono::steady_clock::now();  
        camera->ProcGetImage(frame[id].mat, &start);
        end = std::chrono::steady_clock::now();
        double delta_t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0;
        DLOG(INFO) << "get frame for " << delta_t << "(ms)";
        frame[id].time_stamp = start+ (end-start) / 2;
        frame[id].imu_data = SerialParam::recv_data;
        *_video_thread_params.frame_pp = &frame[id];
        Thread::image_is_update = true;
        Thread::cond_is_update.notify_one();
        umtx_video.unlock();
        id = (id+1) % size;
        LOG_IF(ERROR, (*_video_thread_params.frame_pp)->mat->empty()) << "get empty picture mat!";
    } while(!(*_video_thread_params.frame_pp)->mat->empty());
}

// void* ly::saveFrameToNative(void *params_p) {
//     auto params = reinterpret_cast<Params_ToVideo*>(params_p);
//     const Mat& frame = *(*params->frame_pp)->mat;
//     VideoWriter& writer = params->writer;
//     while(!frame.empty()){
//         writer.write(frame);
//     }
// }

// void ly::VideoCapture::startSave(Params_ToVideo &params_to_video) {
//     // params in
//     _video_thread_params.writer = writer;

//     int nRet = pthread_create(&threadID2, nullptr, saveFrameToNative, static_cast<void*>(&_video_thread_params));
//     LOG_IF(ERROR, nRet == -1) << "error in creating video writer thread!";

// }

DaHenCamera::DaHenCamera() {
    camera = new GxCamera();
}

DaHenCamera::~DaHenCamera() {
    delete camera;
}