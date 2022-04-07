//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_VIDEOCAPTURE_H
#define AUTOAIM_VIDEOCAPTURE_H

#include "opencv2/opencv.hpp"

#include "Params.h"
#include "Log.h"
#include "SerialPort.h"
#include "GxCamera.h"
#include "Thread.h"

//#define TIMEIT

using namespace cv;

namespace ly
{

    void *saveFrameToNative(void *params_p);
    void *getFrameFromPicture(void *params_p);

    enum CameraType
    {
        DaHen,
        Video,
        Picture,
    };

    struct Params_ToVideo
    {
        cv::VideoCapture video;
        //        uint8_t* id;
        Mat **frame_p;
        void *__this;
        std::chrono::steady_clock::time_point *time_stamp;
        VideoWriter writer;

        Params_ToVideo()
        {
            //            id = new uint8_t;
            frame_p = (Mat **)malloc(sizeof(Mat *));
            *frame_p = new Mat();
            time_stamp = new std::chrono::steady_clock::time_point;
        }
        ~Params_ToVideo(){
            free(*frame_p);
            free(frame_p);
        }
    };
    /**
     * @brief:
     */
    class VideoCapture
    {
    public:
        VideoCapture();
        ~VideoCapture();
        virtual void open() = 0;
        virtual void startCapture(Params_ToVideo &) = 0;
        void startSave(Params_ToVideo &params_to_video);
        void chooseCameraType(VideoCapture *&);

    protected:
        double rate{};
        int _id;
        uint16_t height;
        uint16_t width;
        uint16_t offset_x;
        uint16_t offset_y;

        pthread_t threadID{};
        pthread_t threadID2{};

        Params_ToVideo _video_thread_params;

        VideoWriter writer;

        SerialPort *_serial_port;
        SerialPortData *_data_read;
    };

    class DaHenCamera : public VideoCapture
    {
    public:
        explicit DaHenCamera();
        ~DaHenCamera();
        void startCapture(Params_ToVideo &) override;
        void open() override;

    private:
        GxCamera *camera;
        //        int _id;
    };

    class NativeVideo : public VideoCapture
    {
    public:
        explicit NativeVideo();
        ~NativeVideo();
        //        explicit NativeVideo(const string& path);
        void open() override;
        void startCapture(Params_ToVideo &params) override;
        //        void startCapture(Params_ToVideo &params_to_video) override;

    private:
        cv::VideoCapture video;
    };

    class NativePicture : public VideoCapture
    {
    public:
        explicit NativePicture() = default;
        void open() override;
        void startCapture(Params_ToVideo &) override;

    private:
        string base_dir;
        string suffix;
        int id = 0;
    };
}

#endif //AUTOAIM_VIDEOCAPTURE_H
