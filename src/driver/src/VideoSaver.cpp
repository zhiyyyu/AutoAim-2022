#include "../include/VideoSaver.h"

using namespace ly;

VideoSaver::VideoSaver(){
    string save_path = CameraParam::video_path.substr(0, CameraParam::video_path.rfind('/') + 1);
    FILE* fp = popen(("ls -l " + save_path + " |grep ^- | wc -l").c_str(), "r");
    std::fscanf(fp, "%d", &id);
    pclose(fp);
    writer = VideoWriter(save_path + DetectorParam::color + to_string(id) + ".avi", VideoWriter::fourcc('M','P','4','2'), 210.2, Size(1280, 1024));
    LOG(INFO) << "save video in: " << save_path + DetectorParam::color + to_string(id) + ".avi";
//    writer_visual = VideoWriter(save_path + DetectorParam::color + to_string(id) + "_with_gamma" + ".avi", VideoWriter::fourcc('M','P','4','2'), 210.2, Size(1280, 1024));
//    LOG(INFO) << "save video in: " << save_path + DetectorParam::color + to_string(id) + "_with_gamma" + ".avi";
//    gamma = 1 / CameraParam::gamma;
//    for (int i = 0; i < 256; ++i)
//    {
//        gamma_table[i] = saturate_cast<uchar>(pow((float)(i / 255.0), gamma) * 255.0f);
//    }
}

VideoSaver::~VideoSaver(){
    writer.release();
//    writer_visual.release();
}

void VideoSaver::SaveVideo(Mat** frame_p){
    sleep(1);
    const Mat& frame = **frame_p;
    while(!frame.empty()){
//        Mat img = frame.clone();
        writer.write(frame);
    }
}