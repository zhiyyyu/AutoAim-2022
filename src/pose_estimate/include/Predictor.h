//
// Created by zhiyu on 2021/8/20.
//

#ifndef PRIDICTOR_H
#define PRIDICTOR_H

#include "Config.h"
#include <utility>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "Params.h"
#include "../../armor_detector/include/ArmorFinder.h"
// #include "Kalman.h"
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace cv;
using namespace std;

namespace ly
{
    class Predictor
    {
    public:
        Predictor();
        ~Predictor();
        double fitTrajectory(const Point3d& armor, double v, bool useRoll=false);
        double getYaw(const Point3d& armor, double delta_t, double v, const SerialPortData& imu_data, bool usePredictor=true);
        void setStatePost(float x, float z);
        Mat getCurState();
        void reset();

    private:
        double fitPNP(const Point3d& armor, bool usePredictor=true);

        KalmanFilter position_predictor;
        Mat pos_m;
        bool init = false;

        KalmanFilter sentinel_predictor;
        Mat sentinel_m;

        float distance;
        float shoot_t;
        Mat position_cur;

        sockaddr_in server_addr;
        int client_socket;
        string buffer;
        char write_str[40];
        float last_x;
        float last_z;
        float last_a;
    };

}

#endif //AUTOAIM_POSESOLVER_H
