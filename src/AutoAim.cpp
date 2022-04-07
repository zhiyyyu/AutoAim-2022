#include "AutoAim.h"

using namespace ly;

int main(int argc, char** argv){
    
    Params_ToSerialPort params_to_serial_port(&SerialParam::recv_data);
    Params_ToVideo params_to_video;
    Params_ToDetector params_to_detector;

    /******* init log ********/
    auto log = new Log();
    log->init(argv[0]);

    /******* init config ********/
    auto config = new Config(confog_file_path);
    config->parse();

    /******* init serial port read ******/
    auto serial_port = new SerialPort(SerialParam::device_name);
    thread serial_port_thread(&SerialPort::read_data, serial_port, ref(params_to_serial_port));

    /******* init camera ********/
    ly::VideoCapture* video;
    video->chooseCameraType(video);
    thread video_thread(&ly::VideoCapture::startCapture, video, ref(params_to_video));

    /******* init detector ********/
    auto detector = new Detector();
    detector->setParams(params_to_video, params_to_serial_port);
    thread detector_thread(&Detector::startDetect, detector, ref(params_to_detector), serial_port);


   /******** init video writer ****/
   if(GlobalParam::SAVE_VIDEO){
        auto saver = new VideoSaver();
        thread saver_thread(&VideoSaver::SaveVideo, saver, params_to_video.frame_p);
        saver_thread.join();
   }

    serial_port_thread.join();
    video_thread.join();
    detector_thread.join();
    return 0;
}
