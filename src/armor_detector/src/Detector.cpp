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
        _detector_thread_params.frame_p = params_to_video.frame_p;
        _detector_thread_params.time_stamp = params_to_video.time_stamp;
        // _detector_thread_params.cache_p = params_to_serial_port.cache;
        // _detector_thread_params.cache_idx = params_to_serial_port.id;
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
        Mat class_;
        Mat kernel = getStructuringElement(0, Size(3, 1));
        Mat armor_to_show;
        // kalman filter

        // pose
        auto solver = new PoseSolver();
        pair<SE3, double> armor_pose;

        while (!(*_detector_thread_params.frame_p)->empty())
        {
            const Mat &frame = **_detector_thread_params.frame_p;
            clock_t start = clock();
            
            if(GlobalParam::DEBUG_MODE){
                frame.copyTo(drawing);
            }

            LightBarBlobs lightBarBlobs;
            bool isFindBlob = lightBarFinder->findLightBarBlobs(frame, lightBarBlobs);

            uint8_t size = lightBarBlobs.size();
            if (!isFindBlob || size < 2)
            {
                if(GlobalParam::DEBUG_MODE){
                    DLOG(INFO) << "No Armor!";
                    imshow("frame", frame);
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

                    armorFinder->getArmor(lightBarBlobs[i], lightBarBlobs[j], armor);
                    if (
                        armorFinder->matchTwoLightBar(lightBarBlobs[i], lightBarBlobs[j]) &&
                        armorFinder->judgeArmor(armor))
                    {
                        Mat armor_to_classify = frame(armorFinder->getScaleArmorToRoi(armor.rect));

                        if (armor_to_classify.empty())
                            continue;

                        resize(armor_to_classify, armor_to_classify, Size(32, 32));

                        for(int ii=0;ii<32;ii++){
                            for(int jj=0;jj<32;jj++){
                                for(int c=0;c<3;c++){
                                    armor_to_classify.at<Vec3b>(ii, jj)[c] = (uchar)gamma_table[armor_to_classify.at<Vec3b>(ii, jj)[c]];
                                }
                            }
                        }

                        if(GlobalParam::SAVE_ARMOR){
                            save = armor_to_classify.clone();
                        }
                        if(GlobalParam::DEBUG_MODE){
                            imshow("armor", armor_to_classify);
                            waitKey(1);
                        }

                        /******** illumination compensation ********/
                        cvtColor(armor_to_classify, armor_to_classify, COLOR_BGR2GRAY);
                        equalizeHist(armor_to_classify, armor_to_classify);
                        normalize(armor_to_classify, armor_to_classify, 0, 255, NORM_MINMAX);
                        /******** end of compensation **************/
                        if(GlobalParam::DEBUG_MODE){
                            imshow("cls", armor_to_classify);
                            waitKey(1);
                        }
                        /******** begin classifier ***********/
                        armor_to_classify.convertTo(armor_to_classify, CV_32FC1);
                        armor_to_classify = armor_to_classify.reshape(0, 1);
                        svm->predict(armor_to_classify, class_);
                        /******** end of classifier *****/
                        if(GlobalParam::SAVE_ARMOR){
                            static int id = 0;
                            if(id++ % GlobalParam::save_step == 0){
                                imwrite(CameraParam::picture_path + "tmp_" + to_string((int)class_.at<float>(0)) + "/" + to_string(clock()) + "_" + to_string(id++) + ".png", save);
                            }
                        }
                        if (GlobalParam::DEBUG_MODE && class_.at<float>(0) == 3) {
                            putText(drawing, to_string((int)class_.at<float>(0)), armor.rect.tl(), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 1, LINE_8);
                            DEBUG_MODE(
                                   drawArmorCorners(drawing, armor, Scalar(0, 0, 255))
                           );
                        } else {
//                            DEBUG_MODE(
//                                    drawArmorCorners(drawing, armor, Scalar(255, 0, 0))
//                            );
                        }
                        armor._class = (int)class_.at<float>(0);
                        
                        if (armor._class == 3) {
                            DLOG(INFO) << "armor class: " << armor._class;
                            armors.emplace_back(armor);
                            break;
                        }
                    }
                }
            }
            if(GlobalParam::DEBUG_MODE){
//                putText(drawing, to_string(size-1), lightBarBlobs[size-1].center, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0), 1, LINE_8);
                ellipse(drawing, lightBarBlobs[size - 1], Scalar(0, 255, 0), 2, 8);
            }
            if(armors.empty()){
                if(GlobalParam::DEBUG_MODE){
                    imshow("frame", drawing);
                    waitKey(1);
                }
                // DLOG(INFO) << "recv data: yaw " << SerialParam::recv_data.yaw << " pitch " << SerialParam::recv_data.pitch;
                continue;
            }
            sort(armors.begin(), armors.end(), [](const ArmorBlob& a, const ArmorBlob& b)->bool{
                const Rect& r1 = a.rect;
                const Rect& r2 = b.rect;
                return abs(r1.x+r1.y+r1.height/2+r1.width/2-1024/2-1280/2) < abs(r2.x+r2.height/2-1024/2+r2.y+r2.width/2-1280/2);
            });
            
            DLOG(INFO) << "                                           armor: x: " << armors.at(0).rect.x+armors.at(0).rect.width/2 << " y: " << armors.at(0).rect.y+armors.at(0).rect.height/2;
            // SerialParam::recv_data.yaw = 360.f*100;
            armor_pose = solver->getPoseInCamera(armors.at(0), SerialParam::recv_data);
            Point3d trans = {-armor_pose.first.translation()[0],
                                   -armor_pose.first.translation()[1],
                                   armor_pose.first.translation()[2]};

            SerialParam::send_data.shootStatus = 1;
            SerialParam::send_data.pitch = fitTrajectory(trans, 16)*100;
            DLOG(INFO) << "                                           angle: " << armor_pose.second;
            DLOG(INFO) << "                                           send pitch: " << fitTrajectory(trans, 16)*100;
            DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch;
            DLOG(INFO) << "                                           x: " << trans.x << " y: " << trans.y << " z: " << trans.z;

            SerialPort_->writeData(&SerialParam::send_data);
            clock_t end = clock();
            DLOG(INFO) << "processing one frame for " << (end-start)/1000 << "(ms)";
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

    double Detector::fitTrajectory(const Point3d& trans, double v){
        double distance = sqrt(trans.x*trans.x+trans.z*trans.z);
        double cur = atan(trans.y/distance);
        double delta_y;
        for(int i=0;i<10;i++){
            delta_y = trans.y - distance*tan(cur) + 4.9 * distance*distance/pow(v*cos(cur), 2);
//            DLOG(INFO) << "                                           err: " << delta_y << " cur: " << cur << " y: " << trans.y << " cur_y: " << distance*tan(cur) - 4.9 * distance*distance/pow(v*cos(cur), 2);
            if(abs(delta_y) < 0.00001) break;
            cur -= delta_y / (- distance / pow(cos(cur), 2) + 9.8 * distance * distance / (v * v) * sin(cur) / pow(cos(cur), 3));
        }
        return cur/M_PI*180+3.5;
    }
}
