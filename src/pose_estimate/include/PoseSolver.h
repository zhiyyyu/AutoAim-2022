//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_POSESOLVER_H
#define AUTOAIM_POSESOLVER_H

#include "Config.h"
#include "Predictor.h"
#include "../../armor_detector/include/ArmorFinder.h"

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace Sophus;

namespace ly
{
    class PoseSolver
    {
    public:
        explicit PoseSolver();
        pair<SE3, double> getPoseInCamera(const ArmorBlob &armor, const SerialPortData& imu_data);

    private:
        void setCameraMatrix(double fx, double fy, double u0, double v0);
        void setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3);
        void setimu(float pitch, float yaw, float roll);
        //        void setArmorPoints();

        //        inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d& pc, const Eigen::Matrix3d& );
        //        inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d& pc, const Eigen::Matrix3d& );

        Mat camera_matrix;
        Mat distortion_coefficients;
        Mat tvec;
        Mat rvec;
        Mat m_T;
        Mat m_R;

        double camera_trans_x = 0;
        double camera_trans_y = -0.1085;
        double camera_trans_z = 0.05941;

        Matrix3d e_R;
        Vector3d e_T;

        double yaw;

        vector<Point3f> points_large_3d;
        vector<Point3f> points_small_3d;


        Sophus::SE3 armor_to_camera;
        Sophus::SE3 camera_to_gimbal; // imu's world
        Sophus::SE3 armor_to_gimbal;
        Sophus::SE3 armor_to_world;
        Sophus::SE3 gimbal_to_world;

        Sophus::SE3 camera_to_world; // imu's world
    };
}

#endif //AUTOAIM_POSESOLVER_H
