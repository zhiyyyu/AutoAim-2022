#ifndef VIDEO_SAVER
#define VIDEO_SAVER

#include "opencv2/opencv.hpp"
#include "Params.h"

using namespace cv;

class VideoSaver{
public: 
    VideoSaver();
    ~VideoSaver();
    void SaveVideo(Mat** frame);
private:
    int id;
    float gamma;
    unordered_map<int, float> gamma_table;
    VideoWriter writer;
    VideoWriter writer_visual;
};
#endif