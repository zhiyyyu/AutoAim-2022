//
// Created by zhiyu on 2021/8/20.
//

#include "../include/PoseSolver.h"

using namespace std;
namespace ly
{

    PoseSolver::PoseSolver()
    {
        setCameraMatrix(CameraParam::fx, CameraParam::fy, CameraParam::u0, CameraParam::v0);
        setDistortionCoefficients(CameraParam::k1, CameraParam::k2, CameraParam::p1, CameraParam::p2, CameraParam::k3);

        gimbal_to_world = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
        camera_to_gimbal = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(CameraParam::camera_trans_x, CameraParam::camera_trans_y, CameraParam::camera_trans_z));
        
        predictor = new Predictor();
    }

    void PoseSolver::setCameraMatrix(double fx, double fy, double u0, double v0)
    {
        camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        camera_matrix.ptr<double>(0)[0] = fx;
        camera_matrix.ptr<double>(0)[2] = u0;
        camera_matrix.ptr<double>(1)[1] = fy;
        camera_matrix.ptr<double>(1)[2] = v0;
        camera_matrix.ptr<double>(2)[2] = 1.0f;
    }
    //设置畸变系数矩阵
    void PoseSolver::setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3)
    {
        distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
        distortion_coefficients.ptr<double>(0)[0] = k_1;
        distortion_coefficients.ptr<double>(1)[0] = k_2;
        distortion_coefficients.ptr<double>(2)[0] = p_1;
        distortion_coefficients.ptr<double>(3)[0] = p_2;
        distortion_coefficients.ptr<double>(4)[0] = k_3;
    }

    void PoseSolver::setimu(float pitch, float yaw, float roll)
    {
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
        gimbal_to_world = Sophus::SE3(rotation_matrix3, Eigen::Vector3d(0, 0, 0));
    }

    float PoseSolver::cosineLaw(float a, float b, float c){
        double value = fmin(fmax((a*a+b*b-c*c) / (2*a*b), -1), 1);
        DLOG(INFO) << "a: " << a << " b: " << b << " c: " << c << " acos: " << value;
        return acos(value) / M_PI * 180;
    }

    void PoseSolver::solveArmor(ArmorBlob& armor, const SerialPortData& imu_data){
        const static vector<Point3f> points_small_3d = {Point3f(-0.0600f, -0.0275f, 0.f),
                                                         Point3f(0.0600f, -0.0275f, 0.f),
                                                         Point3f(0.0600f, 0.0275f, 0.f),
                                                         Point3f(-0.0600f, 0.0275f, 0.f)};
                                                         
        const static vector<Point3f> points_large_3d = {Point3f(-0.1070f, -0.0275f, 0.f),
                                                         Point3f(0.1070f, -0.0275f, 0.f),
                                                         Point3f(0.1070f, 0.0275f, 0.f),
                                                          Point3f(-0.1070f, 0.0275f, 0.f)};
                                                          
        const static Sophus::SE3 armor_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

        if (armor._class == 1 || armor._class == 6)
        { // large armor
            solvePnP(points_large_3d, armor.corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        }
        else
        { // small armor
            solvePnP(points_small_3d, armor.corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        }

        float rcv_yaw = imu_data.yaw / 100.0f * M_PI / 180.0f;
        float rcv_pitch = imu_data.pitch / 100.0f * M_PI / 180.0f;
        float rcv_roll = roll.getMetric() / roll.getSize();
        setimu(rcv_pitch, rcv_yaw, rcv_roll);
        
        // DLOG(INFO) << "                              corners: " << armor.corners;
        
        cv2eigen(tvec, e_T);
        Rodrigues(rvec, m_R);
        cv2eigen(m_R, e_R);
        
        // double z = atan2(m_R.at<double>(1, 0), m_R.at<double>(0, 0));
        yaw = atan2(-m_R.at<double>(2, 0), sqrt(pow(m_R.at<double>(2, 0), 2) + pow(m_R.at<double>(2, 2), 2))) / M_PI * 180;
        // double x = atan2(m_R.at<double>(2, 1), m_R.at<double>(2, 2));
        // DLOG(INFO) << "z: " << z << " y: " << yaw << " x: " << x;

        armor_to_camera = Sophus::SE3(e_R, e_T) * armor_pose;
        camera_to_world = gimbal_to_world * camera_to_gimbal;
        armor_to_gimbal = camera_to_gimbal * armor_to_camera;
        armor_to_world = camera_to_world * armor_to_camera;

        const auto& a = armor_to_gimbal;
        Point3d t = {a.translation()[0],
                                a.translation()[1],
                                a.translation()[2]};
        // DLOG(INFO) << t;
        Point3d trans = {-armor_to_world.translation()[0],
                                -armor_to_world.translation()[1],
                                armor_to_world.translation()[2]};
        
        armor.angle = yaw;
        armor.x = trans.x; armor.y = trans.y; armor.z = trans.z;
    }

    void PoseSolver::getPoseInCamera(vector<ArmorBlob> &armors, double delta_t, const SerialPortData& imu_data, SerialPort* SerialPort_)
    {
        if(armors.size() < 1) return;
        // 辅瞄模式不打哨兵和前哨站
        for(int i=0;i<armors.size();i++){
            if(armors[i]._class == 7 || armors[i]._class == 6) armors.erase(armors.begin() + i);
            else solveArmor(armors[i], imu_data);
        }
        vector<ArmorBlob> candidates;
        DLOG(INFO) << "state: " << (StateParam::state == ANTITOP?"ANTITOP":"AUTOAIM");
        if(StateParam::state == ANTITOP){ // 反陀螺模式
            for(const auto& a: armors){
                if(a._class == last_armor._class){
                    candidates.push_back(a);
                }
            }
            if(candidates.size() < 1){
                lost_cnt++;
                if(lost_cnt > 50){
                    StateParam::state = AUTOAIM;
                    lost_cnt = 0;
                    circle.clear();
                    DLOG(INFO) << "exit top mode";
                }
                DLOG(INFO) << "lost cnt: " << lost_cnt;
                return;
            }
            lost_cnt = 0;
            
            for(const auto& a: candidates){
                circle.update({a.x, a.y, a.z});
            }
            sort(candidates.begin(), candidates.end(), [&](const ArmorBlob& a, const ArmorBlob& b){
                return calcDiff(a, last_armor) < calcDiff(b, last_armor);
            });
            armor = candidates.at(0);
            center.x = armor.x; center.y = armor.y; center.z = armor.z;
            if(circle.getSize() == circle.size()) {
                center = circle.getMetric() / circle.size();
                DLOG(INFO) << "x: " << center.x << " z: " << center.z << " y: " << center.y;

                for(const auto& a: candidates){
                    if(abs(last_armor.angle - a.angle) < 15 && calcDiff(armor, last_armor) <= 0.2){
                        armor = a;
                    }
                }
                DLOG(INFO) << "last armor - armor: " << abs(last_armor.angle - armor.angle) << " diff: " << calcDiff(armor, last_armor);
                if(calcDiff(armor, last_armor) > 0.2){
                    top_exit = std::chrono::steady_clock::now();
                }
                exit_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - top_exit).count()/1000.0;
                DLOG(INFO) << "duration " << exit_duration << " top cnt: " << top_cnt;
                if(exit_duration > 2){
                    StateParam::state = AUTOAIM;
                    last_armor = armor;
                    top_cnt = 0;
                    return;
                }
                double w = 0.5 * 360;
                double distance = sqrt(center.x * center.x + center.z * center.z);
                double time = distance / speed;
                SerialParam::send_data.shootStatus = 0;

                for(const auto& a: candidates){
                    DLOG(INFO) << "center diff: " << abs(a.x-center.x);
                    if(abs(a.x-center.x) < 0.02){
                        shoot_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - top_exit).count()/1000.0;        
                        shoot_begin = std::chrono::steady_clock::now();
                        DLOG(INFO) << "        shoot duration: " << shoot_duration;
                        thread([this, a, time, w, SerialPort_]() {
                            int sleep_time = (90/w-time)*1000;
                            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
                            SerialParam::send_data.shootStatus = 1;
                            SerialPort_->writeData(&SerialParam::send_data);
                        }).detach();
                        break;
                    }
                }
            }

            DLOG(INFO) << "                                           x: " << center.x << " y: " << center.y << " z: " << center.z;
            SerialParam::send_data.yaw = predictor->getYaw(center, delta_t, speed, imu_data, false);
            SerialParam::send_data.pitch = predictor->fitTrajectory(center, speed);
            DLOG(INFO) << "                                           angle: " << yaw;
            DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
            DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch;
            DLOG(INFO) << "                                           x: " << -armor.x << " y: " << -armor.y << " z: " << armor.z;
            last_armor = armor;

        } else{ // AUTOAIM

            right_clicked = imu_data.right_clicked;
            if(last_right_clicked == 0 && right_clicked == 1) first = true;

            if(first){  // 按下右键时瞄准中心装甲
                if(armors.size() < 1) return;
                first = false;
                sort(armors.begin(), armors.end(), [](const ArmorBlob& a, const ArmorBlob& b)->bool{
                    const Rect& r1 = a.rect;
                    const Rect& r2 = b.rect;
                    return abs(r1.x+r1.y+r1.height/2+r1.width/2-1024/2-1280/2) < abs(r2.x+r2.height/2-1024/2+r2.y+r2.width/2-1280/2);
                });
                armor = armors.at(0);
                top_pri = armor._class;
            }
            
            int target = chooseArmor(armors);
            SerialParam::send_data.num = target;
            // 优先级最高的
            for(const auto& a: armors){
                if(a._class == target) candidates.push_back(a);
            }
            DLOG(INFO) << "target: " << target << " size: " << candidates.size();
            if(candidates.size() < 1) return ;
            // DLOG(INFO) << "target: " << target << " size: " << candidates.size();

            if(target == last_armor._class){ // 上一帧出现过
                if(candidates.size() > 1){ // 上一帧同装甲
                    sort(candidates.begin(), candidates.end(), [&](const ArmorBlob& a, const ArmorBlob& b){
                        return calcDiff(a, last_armor) < calcDiff(b, last_armor);
                    });
                    // DLOG(INFO) << "diff: " << calcDiff(candidates.at(0), last_armor) << " " << calcDiff(candidates.at(1), last_armor); 
                }
                armor = candidates.at(0);

                if(lost_cnt < 20 && calcDiff(armor, last_armor) > 0.25){ // 掉帧缓冲
                    armor = last_armor;
                    lost_cnt++;
                } else{
                    lost_cnt = 0;
                    armor = candidates.at(0);
                }
                DLOG(INFO) << "lost cnt: " << lost_cnt;
                
                if(calcDiff(armor, last_armor) > 0.3 && fabs(armor.angle-last_armor.angle) > 20){
                    top_cnt++;
                    predictor->setStatePost(armor.x, armor.z);
                    top_begin = std::chrono::steady_clock::now();
                }
                DLOG(INFO) << "last armor - armor: angle: " << fabs(last_armor.angle - armor.angle) << " diff: " << calcDiff(armor, last_armor);
                double duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - top_begin).count()/1000.0;
                if(duration > 1.5){
                    top_cnt = 0;
                }
                DLOG(INFO) << "duration " << duration << " top cnt: " << top_cnt;
                if(top_cnt > 5){
                    StateParam::state = ANTITOP;
                    top_exit = std::chrono::steady_clock::now();
                    top_cnt = 0;
                    DLOG(INFO) << "top mode";
                    SerialParam::send_data.shootStatus = 0;
                } else{
                    SerialParam::send_data.shootStatus = 1;
                }
            } else{ // 没有上一帧的数字
                DLOG(INFO) << "lost cnt: " << lost_cnt;
                DLOG(INFO) << "no same class.";
                predictor->reset();

                if(lost_cnt < 30 && calcDiff(armor, last_armor) > 0.1){ // 掉帧缓冲
                    armor = last_armor;
                    lost_cnt++;
                } else{
                    lost_cnt = 0;
                    sort(candidates.begin(), candidates.end(), [](const ArmorBlob& a, const ArmorBlob& b)->bool{
                        return a.x*a.x+a.z*a.z < b.x*b.x+b.z*b.z;
                    });
                    armor = candidates.at(0);
                }
            }

            cur = Point3d(armor.x, armor.y, armor.z);
            SerialParam::send_data.yaw = predictor->getYaw(cur, delta_t, speed, imu_data);
            SerialParam::send_data.pitch = predictor->fitTrajectory(cur, speed);
            DLOG(INFO) << "                                           right clicked: " << right_clicked;
            DLOG(INFO) << "                                           angle: " << yaw;
            DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
            DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch;
            DLOG(INFO) << "                                           x: " << -armor.x << " y: " << -armor.y << " z: " << armor.z;
            last_armor = armor;
            last_right_clicked = right_clicked;
        }
        DLOG(INFO) << "target class: " << armor._class;
    }

    void PoseSolver::outpostMode(vector<ArmorBlob> &armors, double delta_t, const SerialPortData& imu_data, SerialPort* SerialPort_){
        if(armors.size() < 1) return;
        for(ArmorBlob& a: armors) solveArmor(a, imu_data);
        vector<ArmorBlob> candidates;
        DLOG(INFO) << "state: OUTPOST";
        for(auto& a: armors){
            if(a._class == 7){
                a.x *= 6.2/5; a.y *= 6.2/5; a.z *= 6.2/5;
                candidates.push_back(a);
            }
        }
        if(candidates.size() < 1){
            last_armor = armors.at(0);
            return;
        }

        right_clicked = imu_data.right_clicked;
        if(last_right_clicked == 0 && right_clicked == 1) first = true;
        if(first){ outpost.clear(); first = false; }
        last_right_clicked = right_clicked;
        DLOG(INFO) << " right clicked: " << right_clicked;
             
        for(const auto& a: candidates){
            outpost.update({a.x, a.y, a.z});
        }
        
        sort(candidates.begin(), candidates.end(), [&](const ArmorBlob& a, const ArmorBlob& b){
            return calcDiff(a, last_armor) < calcDiff(b, last_armor);
        });
        armor = candidates.at(0);
        center.x = armor.x; center.y = armor.y; center.z = armor.z;

        if(abs(imu_data.roll-roll.getMetric()/roll.getSize()) < 1000){
            roll.update(imu_data.roll);
        }

        if(outpost.getSize() == outpost.size()) {
            center = outpost.getMetric() / outpost.size();

            double w = 0.4 * 360;
            double distance = sqrt(center.x * center.x + center.y * center.y + center.z * center.z);
            // 发弹延时0.01
            double time = distance / speed;
            SerialParam::send_data.shootStatus = 0;

            for(const auto& a: candidates){
                DLOG(INFO) << "center diff: " << abs(a.x-center.x);
                if(abs(a.x-center.x) < 0.02 && abs(a.y-center.y) < 0.05){                    
                    thread([this, a, time, w, SerialPort_]() {
                        int sleep_time = (120/w-time)*1000 - 0.01;
                        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
                        SerialParam::send_data.shootStatus = 1;
                        SerialPort_->writeData(&SerialParam::send_data);
                        DLOG(INFO) << "                shoot";
                    }).detach();
                    break;
                }
            }
        }

        DLOG(INFO) << "                                    center x: " << center.x << " y: " << center.y << " z: " << center.z << " roll: " << roll.getMetric()/roll.getSize();

        if(abs(roll.getMetric() / roll.getSize()) > 500){
            double offset = predictor->fitTrajectory(center, speed, true);
            double bias = 0;
            SerialParam::send_data.yaw = predictor->getYaw(center, delta_t, speed, imu_data, false) + (offset + bias) * sin(roll.getMetric()/roll.getSize()/100/180*3.14) - 300;
            SerialParam::send_data.pitch = predictor->fitTrajectory(center, speed, false) - offset + (offset + bias) * cos(- roll.getMetric()/roll.getSize()/100/180*3.14);
        } else{
            SerialParam::send_data.yaw = predictor->getYaw(center, delta_t, speed, imu_data, false);
            SerialParam::send_data.pitch = predictor->fitTrajectory(center, speed, false) + 50;
        }

        DLOG(INFO) << "                                           angle: " << yaw;
        DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
        DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch << "  recv roll: " << imu_data.roll;
        DLOG(INFO) << "                                           x: " << armor.x << " y: " << armor.y << " z: " << armor.z;
        last_armor = armor;
    }

    void PoseSolver::sentinelMode(vector<ArmorBlob> &armors, double delta_t, const SerialPortData& imu_data, SerialPort* SerialPort_){
        if(armors.size() < 1) return;
        for(ArmorBlob& a: armors) solveArmor(a, imu_data);
        vector<ArmorBlob> candidates;
        // DLOG(INFO) << "state: SENTINEL";
        for(const auto& a: armors){
            if(a._class == 6){
                candidates.push_back(a);
            }
        }
        if(candidates.size() < 1){
            // last_armor = armors.at(0);
			armor = last_armor;
            // return;
        } else armor = candidates.at(0);

        int mode = 1;
        if(mode){
            SerialParam::send_data.shootStatus = 1;
            cur = Point3d(armor.x, armor.y, armor.z);
            SerialParam::send_data.yaw = predictor->getYaw(cur, delta_t, speed, imu_data);
            SerialParam::send_data.pitch = predictor->fitTrajectory(cur, speed);

        } else{ // 变速
            // predictor->getYaw(armor, delta_t, speed, imu_data);
            // predictor->fitTrajectory(armor, speed);
            // const Mat& cur = predictor->getCurState();

            // if(sentinel.size() >= sentinel_cnt){
            //     sentinel[sentinel_id].x = armor.x;
            //     sentinel[sentinel_id].y = armor.y;
            //     sentinel[sentinel_id].z = armor.z;
            // } else{
            //     sentinel.push_back({armor.x, armor.y, armor.z});
            // }
            // sentinel_id = ((sentinel_id + 1) % sentinel_cnt + sentinel_cnt) % sentinel_cnt;
            // DLOG(INFO) << "sentinel id: " << sentinel_id << " size: " << sentinel.size();
            // x0 = 0; y0 = 0; z0 = 0;
            // if(sentinel.size() == sentinel_cnt){
            //     for(int i=0;i<sentinel_cnt;i++){
            //         x0 += sentinel[i].x; y0 += sentinel[i].y; z0 += sentinel[i].z; 
            //     }
            //     armor.x = x0/sentinel_cnt; armor.y = y0/sentinel_cnt; armor.z = z0/sentinel_cnt; 
            // }

            // double distance = sqrt(armor.x*armor.x + armor.z*armor.z), v = 15;
            // double x = cur.at<float>(0) +(distance/v)*cur.at<float>(2);
            // double theta = atan(armor.y/distance);
            // double delta_y;
            // // R = 42.50 mm, m = 41 g
            // double k1 = 0.47*1.169*(2*M_PI*0.02125*0.02125)/2/0.041;
            // for(int i=0;i<100;i++){
            //     double t = (pow(2.718281828, k1*distance)-1)/(k1*v*cos(theta));
            //     delta_y = armor.y - v*sin(theta)*t/cos(theta) + 4.9 * t*t/cos(theta)/cos(theta);
            //     // DLOG(INFO) << "delta_y: " << delta_y;
            //     if(fabs(delta_y) < 0.001) break;
            //     theta -= delta_y / (- (v*t) / pow(cos(theta), 2) + 9.8 * t * t / (v * v) * sin(theta) / pow(cos(theta), 3));
            // }
            // SerialParam::send_data.shootStatus = fabs(x - armor.x) < 0.02;
            // SerialParam::send_data.yaw = (atan(armor.x/armor.z)/M_PI*180)*100;
            // SerialParam::send_data.pitch = (theta/M_PI*180+2)*100;
            
            // DLOG(INFO) << "abs: " << fabs(x - armor.x) << " x: " << x;
            // DLOG(INFO) << "x: " << armor.x << " y: " << armor.y << " z: " << armor.z;
        }

        DLOG(INFO) << "                                           angle: " << yaw;
        DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
        DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch << "  recv roll: " << SerialParam::recv_data.roll;
        DLOG(INFO) << "                                           x: " << -armor.x << " y: " << -armor.y << " z: " << armor.z;
        last_armor = armor;

    }

    float PoseSolver::calcDiff(const ArmorBlob& a, const ArmorBlob& b){
        return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2)+pow(a.z-b.z, 2));
    }

    void PoseSolver::clearCircle(){
        outpost.clear();
    }

    void PoseSolver::clearSentinel(){
        sentinel.clear();
    }

    int PoseSolver::chooseArmor(const vector<ArmorBlob>& armors){
        bool has_same_class = false;
        bool has_hero = false;
        bool has_sentry = false;
        bool has_engineer = false;
        for(const auto& a: armors){
            if(a._class == top_pri) return top_pri;
            if(a._class == last_armor._class) has_same_class = true;
            switch(a._class){
            case 1: has_hero = true; break;
            case 2: has_engineer = true; break;
            case 3: case 4: case 5: has_sentry = true; break;
            }
        }
        if(has_same_class) return last_armor._class;
        else if(has_hero) return 1;
        else if(has_sentry){
            double d = 10; int c = 0;
            for(const auto& a: armors){
                if((a._class == 3 || a._class == 4 || a._class == 5) && d > (a.x*a.x+a.z*a.z)){
                    d = a.x*a.x+a.z*a.z; c = a._class;
                }
            }
            return c;
        } else if(has_engineer) return 2;
        return 0;
    }
}
