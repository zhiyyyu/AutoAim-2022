//
// Created by zhiyu on 2021/8/20.
//

#include "Detector.h"
#include "SerialPort.h"

namespace ly
{
    void Detector::calcGammaTable(float gamma)
    {
        for (int i = 0; i < 256; ++i)
        {
            gamma_table[i] = saturate_cast<uchar>(pow((float)(i / 255.0), gamma) * 255.0f);
        }
    }

    void Detector::setParams(const Params_ToVideo &params_to_video, const Params_ToSerialPort &params_to_serial_port)
    {

        // send params to detector thread
        _detector_thread_params.frame_pp = params_to_video.frame_pp;
    }

    void Detector::startDetect(const Params_ToDetector &params, SerialPort* SerialPort_)
    {
        // image to show
        Mat drawing = Mat();
        // temp armor
        ArmorBlob armor;
        auto lightBarFinder = new LightBarFinder();
        auto armorFinder = new ArmorFinder();
        // gamma
        float gamma = 1 / CameraParam::gamma;
        calcGammaTable(gamma);
        // svm
        auto score = new Score();
        Mat save;
        Ptr<ml::SVM> svm;
        svm = Algorithm::load<ml::SVM>(score->svmModel);
        Mat class_mat;
        auto classifier = new CNN();
        Mat kernel = getStructuringElement(0, Size(3, 3));
        Mat armor_to_show;
        HOGDescriptor hog(Size(32, 32), Size(16, 16), Size(8, 8), Size(8, 8), 10);
        // int feature = 1000;
        // kalman filter

        // pose
        auto solver = new PoseSolver();
        //  clahe
        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(4.);
        clahe->setTilesGridSize(Size(8, 8));
        // affine
        Point2f dst[3];
        dst[0] = {0, 0}; dst[1] = {32, 0}; dst[2] = {0, 32};
        // timer
        auto start = std::chrono::steady_clock::now();
        TimeSystem::time_zero = std::chrono::steady_clock::now();
        // serial check
        SerialPortData recv_data(SerialParam::recv_data);
        sleep(1);
        
        while (!(*_detector_thread_params.frame_pp)->mat->empty())
        {
            unique_lock<mutex> umtx_video(Thread::mtx_image);
            while(!Thread::image_is_update){ Thread::cond_is_update.wait(umtx_video); }
            const Image &image = **_detector_thread_params.frame_pp;
            Thread::image_is_update = false;
            Thread::cond_is_process.notify_one();
            umtx_video.unlock();
            const Mat& frame = *image.mat;
            DLOG(INFO) << " cam time_stamp: " << std::chrono::duration_cast<std::chrono::microseconds>((image.time_stamp) - TimeSystem::time_zero).count()/1000.0
                    << "  imu time_stamp: " << SerialParam::recv_data.time_stamp / 100.0;
            start = std::chrono::steady_clock::now();

            if(abs(SerialParam::recv_data.yaw-recv_data.yaw) < 18000 && abs(SerialParam::recv_data.pitch-recv_data.pitch) < 18000){
                recv_data = SerialParam::recv_data;
            }

            if(GlobalParam::DEBUG_MODE){
                frame.copyTo(drawing);
            }
            switch(SerialParam::recv_data.flag){
            case 0x06:
                if(StateParam::state != OUTPOST){
                    solver->clearCircle();
                }
                StateParam::state = OUTPOST; break;
            case 0x07:
                if(StateParam::state != SENTINEL){
                    solver->clearSentinel();
                }
                StateParam::state = SENTINEL; break;
            case 0x05:
            default:
                if(StateParam::state == OUTPOST || StateParam::state != ANTITOP){
                    StateParam::state = AUTOAIM; 
                }
                break;
            }
            // StateParam::state = AUTOAIM;
            StateParam::state = OUTPOST;
            // StateParam::state = SENTINEL;
            
            SerialParam::send_data.shootStatus = 0;

            LightBarBlobs lightBarBlobs;
            bool isFindBlob = lightBarFinder->findLightBarBlobs(frame, lightBarBlobs);

            uint8_t size = lightBarBlobs.size();
            if (!isFindBlob || size < 2)
            {
                if(GlobalParam::DEBUG_MODE){
                    DLOG(INFO) << "No Armor!";
                    imshow("frame", drawing);
                    waitKey(1);
                }
                continue;
            }
            ArmorBlobs armors;
            for (int i = 0; i < size - 1; i++)
            {
                if(GlobalParam::DEBUG_MODE){
//                    putText(drawing, to_string(i), lightBarBlobs[i].center, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 1, LINE_8);
                    ellipse(drawing, lightBarBlobs[i], Scalar(0, 255, 0), 2, 8);
                }
                for(int j = i + 1;j < size; j++){
                    if (
                        armorFinder->getArmor(lightBarBlobs[i], lightBarBlobs[j], armor) &&
                        armorFinder->matchTwoLightBar(lightBarBlobs[i], lightBarBlobs[j]) &&
                        armorFinder->judgeArmor(armor))
                    {
                        const vector<int>& extreme = armorFinder->getExtreme(armor);
                        
                        Mat armor_to_classify = frame(Rect(extreme[0], extreme[1], (extreme[2]-extreme[0]), extreme[3]-extreme[1]));
                        if (armor_to_classify.empty())
                            continue;
                        if(GlobalParam::DEBUG_MODE){
                            imshow("origin", armor_to_classify);
                            waitKey(1);
                        }
                        Point2f src[3];
                        src[0] = armor.corners[0] - Point2f(extreme[0], extreme[1] + min(armor.rect.height, armor.rect.width)/2.5f); 
                        src[1] = armor.corners[1] - Point2f(extreme[0], extreme[1] + min(armor.rect.height, armor.rect.width)/2.5f); 
                        src[2] = armor.corners[3] - Point2f(extreme[0], extreme[1] - min(armor.rect.height, armor.rect.width)/2.5f);
                        Mat affine_mat = getAffineTransform(src, dst);
                        warpAffine(armor_to_classify, armor_to_classify, affine_mat, Size(32, 32));

                        for(int ii=0;ii<32;ii++){
                            for(int jj=0;jj<32;jj++){
                                for(int c=0;c<3;c++){
                                    armor_to_classify.at<Vec3b>(ii, jj)[c] = (uchar)gamma_table[armor_to_classify.at<Vec3b>(ii, jj)[c]];
                                }
                            }
                        }

                        double alpha = 1.5, beta = 0;
                        Mat hsv;
                        
                        double avg = 0, min = 255, max = 0;
                        cvtColor(armor_to_classify, hsv, COLOR_BGR2HSV);
                        for(int i=0;i<hsv.rows;i++){
                            for(int j=0;j<hsv.cols;j++){
                                avg += hsv.at<Vec3b>(i, j)[2];
                                // min = fmin(min, hsv.at<Vec3b>(i, j)[2]);
                                // max = fmin(max, hsv.at<Vec3b>(i, j)[2]);
                            }
                        }
                        // avg /= hsv.cols*hsv.rows;
                        // DLOG(INFO) << "avg: " << avg;
                        // for(int i=0;i<hsv.rows;i++){
                        //     for(int j=8;j<hsv.cols-8;j++){
                        //         hsv.at<Vec3b>(i, j)[2] *= 50 / avg;
                        //     }
                        // }
                        cvtColor(hsv, armor_to_classify, COLOR_HSV2BGR);

                        if(GlobalParam::SAVE_ARMOR){
                            save = armor_to_classify.clone();
                        }

                        /******** illumination compensation ********/
                        cvtColor(armor_to_classify, armor_to_classify, COLOR_BGR2GRAY);
                        medianBlur(armor_to_classify, armor_to_classify, 3);
                        // GaussianBlur(armor_to_classify, armor_to_classify, Size(3, 3), 1, 1);
                        // morphologyEx(armor_to_classify, armor_to_classify, MORPH_CLOSE, kernel);
                        // morphologyEx(armor_to_classify, armor_to_classify, MORPH_OPEN, kernel);
                        equalizeHist(armor_to_classify, armor_to_classify);              
                        // threshold(armor_to_classify, armor_to_classify, 130, 255, THRESH_BINARY);
                        // adaptiveThreshold(armor_to_classify, armor_to_classify, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, 3);
                        
                        // vector<float> desc;
                        // hog.compute(armor_to_classify, desc, Size(8, 8), Size(0, 0));
                        // Mat x(1, desc.size(), CV_32FC1);
                        // for(float i=0;i<desc.size();i++){
                        //     x.at<float>(0, i) = desc[i] * 100;
                        // }

                        /******** end of compensation **************/
                        if(GlobalParam::DEBUG_MODE){
                            imshow("armor", armor_to_classify);
                            waitKey(1);
                        }
                        /******** begin classifier ***********/
                        // 普通svm
                        // armor_to_classify.convertTo(armor_to_classify, CV_32FC1);
                        // armor_to_classify = armor_to_classify.reshape(0, 1);
                        // svm->predict(armor_to_classify, class_mat);
                        // int class_ = class_mat.at<float>(0);

                        // hog+svm
                        // x.convertTo(x, CV_32FC1);
                        // x = x.reshape(0, 1);
                        // svm->predict(x, class_mat);
                        // int class_ = class_mat.at<float>(0);
                        
                        // ResNet18
                        // armor_to_classify.convertTo(armor_to_classify, CV_32FC1, 2.0/255, -1);
                        int class_ = classifier->predict(armor_to_classify);
                        /******** end of classifier *****/
                        if(GlobalParam::SAVE_ARMOR){
                            static int id = 0;
                            if(id++ % GlobalParam::save_step == 0){
                                imwrite(CameraParam::picture_path + "tmp_" + to_string(class_) + "/" + to_string(clock()) + "_" + to_string(id++) + ".png", save);
                            }
                        }
                        if(GlobalParam::DEBUG_MODE){
                            if (class_ > 0) {
                                putText(drawing, to_string(class_), armor.rect.tl(), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 1, LINE_8);
                                DEBUG_MODE(
                                    drawArmorCorners(drawing, armor, Scalar(0, 0, 255))
                            );
                            } else {
                                DEBUG_MODE(
                                        drawArmorCorners(drawing, armor, Scalar(255, 0, 0))
                                );
                            }
                        }
                        
                        armor._class = class_;

                        if (armor._class > 0) {
                            DLOG(INFO) << "armor class: " << armor._class;
                            armors.emplace_back(armor);
                            break;
                        }
                        // DLOG(INFO) << "armor class: " << armor._class;
                    }
                }
            }
            if(GlobalParam::DEBUG_MODE){
//                putText(drawing, to_string(size-1), lightBarBlobs[size-1].center, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 1, LINE_8);
                ellipse(drawing, lightBarBlobs[size - 1], Scalar(0, 255, 0), 2, 8);
            }
            if(armors.empty()){
                if(GlobalParam::DEBUG_MODE){
                    // Predictor::reset();
                    
                    DLOG(INFO) << "Detect BG";
                    imshow("frame", drawing);
                    waitKey(1);
                }
                continue;
            }
            double delta_t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count()/1000.0;
            // start = std::chrono::steady_clock::now();
            DLOG(INFO) << "processing one frame for " << delta_t << "(ms)";

            switch(StateParam::state){
            case OUTPOST:
                solver->outpostMode(armors, delta_t/1000, recv_data, SerialPort_); break;
            case SENTINEL:
                solver->sentinelMode(armors, delta_t/1000, recv_data, SerialPort_); break;
            default:
                solver->getPoseInCamera(armors, delta_t/1000, recv_data, SerialPort_); break;
            }
            // SerialParam::send_data.shootStatus = 1;
            SerialParam::send_data.state = StateParam::state;
            SerialParam::send_data.time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - TimeSystem::time_zero).count()/10.0;
            // DLOG(INFO) << " send time_stamp: " << SerialParam::send_data.time_stamp;
            DLOG(INFO); 
            SerialPort_->writeData(&SerialParam::send_data);
            if(GlobalParam::DEBUG_MODE){
                imshow("frame", drawing);
                waitKey(1);
            }
        }
        destroyAllWindows();
    }

    Detector::Detector()
    {
    }

    void Detector::drawArmorCorners(Mat &drawing, const ArmorBlob &armor, const Scalar &color)
    {
        for (auto i = 0; i < 4; i++)
        {
            // putText(drawing, to_string(i), armor.corners[i], FONT_HERSHEY_COMPLEX, 1, color, 1, LINE_8);
            line(drawing, armor.corners[i], armor.corners[(i + 1) % 4], color, 2, 8);
        }
    }

    void Detector::outpostMode(){

    }
}
