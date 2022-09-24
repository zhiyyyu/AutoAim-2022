//
// Created by zhiyu on 2022/04/06.
//

#include "Predictor.h"
namespace ly
{

    Predictor::Predictor()
    {        
        position_predictor = KalmanFilter(6, 2, 0);        // 状态量：6，预测量：2，控制量：0
        pos_m = Mat::zeros(2, 1, CV_32F);   // 观测矩阵H
        double t = 0.005;
        position_predictor.transitionMatrix = (Mat_<float>(6, 6) << 
            1, 0, t, 0, 0.5*t*t, 0,
            0, 1, 0, t, 0, 0.5*t*t,
            0, 0, 1, 0, t, 0,
            0, 0, 0, 1, 0, t,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);                          // 状态转移矩阵A
        setIdentity(position_predictor.measurementMatrix); // 观测矩阵H
        // setIdentity(position_predictor.processNoiseCov, cv::Scalar::all(1e-6));        // 过程噪声Q
        // 调参顺序：x->v->a
        // 可视化出来调
        // float x_n = 5e-3, z_n = 2e-3;
        // float v_x = 2e-3, v_z = 1e-1;
        // float a_x = 1e-1, a_z = 1e-3;
        float x_n, z_n, v_x, v_z, a_x, a_z;
        x_n = 5e-3; z_n = 5e-3;
        v_x = 1e-3; v_z = 1e-3;
        a_x = 1e-3; a_z = 5e-4;
        position_predictor.processNoiseCov = (Mat_<float>(6, 6) << x_n*x_n, 0, 0, 0, 0, 0,
                                                                    0, z_n*z_n, 0, 0, 0, 0,
                                                                    0, 0, v_x*v_x, 0, 0, 0,
                                                                    0, 0, 0, v_z*v_z, 0, 0,
                                                                    0, 0, 0, 0, a_x*a_x, 0,
                                                                    0, 0, 0, 0, 0, a_z*a_z);
        // setIdentity(position_predictor.measurementNoiseCov, cv::Scalar::all(1e-4));   // 测量噪声R,0.1
        // 最好用实测值
        // 较小的测量噪声，快速收敛
        float x_m = 1e-2, z_m = 1e-2;
        position_predictor.measurementNoiseCov = (Mat_<float>(2, 2) << x_m*x_m, 0,
            0, z_m*z_m);
        reset();


        sentinel_predictor = KalmanFilter(6, 2, 0);        // 状态量：6，预测量：2，控制量：0
        sentinel_m = Mat::zeros(2, 1, CV_32F);   // 观测矩阵H
        // double t = 0.005;
        sentinel_predictor.transitionMatrix = (Mat_<float>(6, 6) << 
            1, 0, t, 0, 0.5*t*t, 0,
            0, 1, 0, t, 0, 0.5*t*t,
            0, 0, 1, 0, t, 0,
            0, 0, 0, 1, 0, t,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);                          // 状态转移矩阵A
        setIdentity(sentinel_predictor.measurementMatrix); // 观测矩阵H
        // setIdentity(sentinel_predictor.processNoiseCov, cv::Scalar::all(1e-6));        // 过程噪声Q
        // 调参顺序：x->v->a
        // 可视化出来调
        // x_n = 5e-3; z_n = 2e-3;
        // v_x = 5e-3; v_z = 1e-3;
        // a_x = 1e-5; a_z = 1e-3;
        x_n = 5e-3; z_n = 1e-2;
        v_x = 6e-3; v_z = 1e-4;
        a_x = 1e-1; a_z = 1e-5;
        sentinel_predictor.processNoiseCov = (Mat_<float>(6, 6) << x_n*x_n, 0, 0, 0, 0, 0,
                                                                    0, z_n*z_n, 0, 0, 0, 0,
                                                                    0, 0, v_x*v_x, 0, 0, 0,
                                                                    0, 0, 0, v_z*v_z, 0, 0,
                                                                    0, 0, 0, 0, a_x*a_x, 0,
                                                                    0, 0, 0, 0, 0, a_z*a_z);
        // setIdentity(sentinel_predictor.measurementNoiseCov, cv::Scalar::all(1e-4));   // 测量噪声R,0.1
        // 最好用实测值
        // 较小的测量噪声，快速收敛
        x_m = 1e-2; z_m = 1e-2;
        sentinel_predictor.measurementNoiseCov = (Mat_<float>(2, 2) << x_m*x_m, 0,
            0, z_m*z_m);
        setIdentity(sentinel_predictor.errorCovPost, cv::Scalar::all(1e-5));            // 真实噪声P
        sentinel_predictor.statePost = (Mat_<float>(1, 6) << 0.1, 2, 0, 0, 0, 0);

        if(GlobalParam::SOCKET){
            client_socket = socket(AF_INET, SOCK_STREAM, 0);
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
            server_addr.sin_port = htons(9999);
            connect(client_socket,(sockaddr *)&server_addr, sizeof(sockaddr));
        }
    }

    Predictor::~Predictor(){
        close(client_socket);
    }

    void Predictor::reset(){
        setIdentity(position_predictor.errorCovPost, cv::Scalar::all(1e-6));            // 真实噪声P
        position_predictor.statePost = (Mat_<float>(1, 6) << 0.1, 2, 0, 0, 0, 0);

        setIdentity(sentinel_predictor.errorCovPost, cv::Scalar::all(1e-6));            // 真实噪声P
        sentinel_predictor.statePost = (Mat_<float>(1, 6) << 0.1, 2, 0, 0, 0, 0);
    }

    double Predictor::fitPNP(const Point3d& armor, bool usePredictor){
        double d = sqrt(position_cur.at<float>(0)*position_cur.at<float>(0)+position_cur.at<float>(1)*position_cur.at<float>(1));
        double r = sqrt(armor.x*armor.x+armor.z*armor.z);
        double ret = usePredictor ? d : r;
        return ret;
    }

    double Predictor::fitTrajectory(const Point3d& armor, double v, bool useRoll){
        // double theta = atan(armor.y/distance);
        // double delta_y;
        // for(int i=0;i<50;i++){
        //     delta_y = armor.y - distance*tan(theta) + 4.9 * distance*distance/pow(v*cos(theta), 2);
        //     // DLOG(INFO) << "delta_y: " << delta_y;
        //     if(fabs(delta_y) < 0.000001) break;
        //     theta -= delta_y / (- distance / pow(cos(theta), 2) + 9.8 * distance * distance / (v * v) * sin(theta) / pow(cos(theta), 3));
        // }
        // if(useRoll) (theta/M_PI*180+distance*0.2)*100;
        // return (theta/M_PI*180+distance*0.2)*100;
        // 考虑空气阻力
        double theta = atan(armor.y/distance);
        double delta_y;
        // R = 42.50 mm, m = 41 g
        double k1 = 0.47*1.169*(2*M_PI*0.02125*0.02125)/2/0.041;
        for(int i=0;i<100;i++){
            double t = (pow(2.718281828, k1*distance)-1)/(k1*v*cos(theta));
            delta_y = armor.y - v*sin(theta)*t/cos(theta) + 4.9 * t*t/cos(theta)/cos(theta);
            // DLOG(INFO) << "delta_y: " << delta_y;
            if(fabs(delta_y) < 0.000001) break;
            theta -= delta_y / (- (v*t) / pow(cos(theta), 2) + 9.8 * t * t / (v * v) * sin(theta) / pow(cos(theta), 3));
        }
        // cos(-roll) ?
        if(useRoll) return (theta-atan(armor.y/distance))/M_PI*180*100;
        return (theta/M_PI*180+1)*100;
    }

    void Predictor::setStatePost(float x, float z){
        position_predictor.statePost.at<float>(0) = x;
        position_predictor.statePost.at<float>(1) = z;
    }

    double Predictor::getYaw(const Point3d& armor, double delta_t, double v, const SerialPortData& imu_data, bool usePredictor){
        auto& predictor = StateParam::state == SENTINEL ? sentinel_predictor : position_predictor;
        // DLOG(INFO) << "state: " << (StateParam::state == SENTINEL ? "sentinel" : "autoaim");

        predictor.transitionMatrix = (Mat_<float>(6, 6) << 
            1, 0, delta_t, 0, 0.5*delta_t*delta_t, 0,
            0, 1, 0, delta_t, 0, 0.5*delta_t*delta_t,
            0, 0, 1, 0, delta_t, 0,
            0, 0, 0, 1, 0, delta_t,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);
        
        if(StateParam::state == SENTINEL){
            sentinel_m.at<float>(0) = armor.x;
            sentinel_m.at<float>(1) = armor.z;
            predictor.correct(sentinel_m);
        } else{
            pos_m.at<float>(0) = armor.x;
            pos_m.at<float>(1) = armor.z;
            predictor.correct(pos_m);
        }
        position_cur = predictor.predict();
        // for both yaw and pitch
        distance = fitPNP(armor, false);
        shoot_t = distance/v;
        double offset_x = fmin(fmax(position_cur.at<float>(2), -1), 1)*shoot_t;
        double offset_z = fmin(fmax(position_cur.at<float>(3), -1), 1)*shoot_t;

        double tmp = ((atan((position_cur.at<float>(0)+offset_x) /
                (position_cur.at<float>(1))+offset_z)/M_PI*180))*100;

        if(!usePredictor) {
            tmp = ((atan((armor.x) /
                (armor.z))/M_PI*180))*100;
        }
        tmp -= 0.3*100;

        double a = sqrt(pow(position_cur.at<float>(4), 2)+pow(position_cur.at<float>(5), 2));

        // if(a > 4 && StateParam::state == SENTINEL){
        //     SerialParam::send_data.shootStatus = 0;
        // }

        DLOG(INFO) << "                                           distance: " << distance << " time: " << shoot_t;
        DLOG(INFO) << "                                           pred: x " << position_cur.at<float>(0)+offset_x << " z " << position_cur.at<float>(1)+offset_z;
        DLOG(INFO) << "                                           speed: x " << position_cur.at<float>(2) << " z " << position_cur.at<float>(3);
        DLOG(INFO) << "                                           accel: x " << position_cur.at<float>(4) << " z " << position_cur.at<float>(5);
        DLOG(INFO) << "                                           offset: x " << offset_x << " z " << offset_z;

        while(abs(tmp - imu_data.yaw) > 9000){
            if(tmp - imu_data.yaw > 9000){
                tmp -= 18000;
            } else{
                tmp += 18000;
            }
        }
        
        if(GlobalParam::SOCKET){
            // x
            // buffer = to_string(position_cur.at<float>(0)+offset_x) + " " + to_string(position_cur.at<float>(1)+offset_z) + " " + 
            //     to_string(armor.x) + " " + to_string(armor.z) + " ";

            // v
            buffer = to_string(position_cur.at<float>(2)+shoot_t*position_cur.at<float>(4)) + " " + to_string(position_cur.at<float>(3)+shoot_t*position_cur.at<float>(5)) + " " + 
                to_string((position_cur.at<float>(0)+offset_x-last_x)/delta_t) + " " + to_string((position_cur.at<float>(1)+offset_z-last_z)/delta_t) + " ";
            last_x = position_cur.at<float>(0)+offset_x;
            last_z = position_cur.at<float>(1)+offset_z;

            // a
            // buffer = to_string(position_cur.at<float>(4)) + " " + to_string(position_cur.at<float>(5)) + " " + 
            //     to_string((position_cur.at<float>(2)+shoot_t*position_cur.at<float>(4)-last_x)/0.005) + " " + to_string((position_cur.at<float>(3)+shoot_t*position_cur.at<float>(5)-last_z)/0.005) + " ";
            // last_x = position_cur.at<float>(2)+shoot_t*position_cur.at<float>(4);
            // last_z = position_cur.at<float>(3)+shoot_t*position_cur.at<float>(5);

            // buffer = to_string(a) + " " + to_string(position_cur.at<float>(3)+0.1*position_cur.at<float>(5)) + " " + 
            //     to_string((position_cur.at<float>(0)+offset_x-last_x)/0.005) + " " + to_string((position_cur.at<float>(1)+offset_z-last_z)/0.005) + " ";
            
            strcpy(write_str, buffer.c_str());
            write(client_socket, write_str, sizeof(write_str));
        }

        return tmp;
    }

    Mat Predictor::getCurState(){
        return position_cur;
    }
}
