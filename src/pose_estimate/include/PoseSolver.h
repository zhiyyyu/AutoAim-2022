//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_POSESOLVER_H
#define AUTOAIM_POSESOLVER_H

#include "Config.h"
#include "Predictor.h"
#include "../../armor_detector/include/ArmorFinder.h"
#include "Params.h"
#include "Array.hpp"

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <fstream>
#include <string>
#include <chrono>
#include <ceres/ceres.h>

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
        void getPoseInCamera(vector<ArmorBlob> &armors, double delta_t, const SerialPortData& imu_data, SerialPort* SerialPort_);
        void outpostMode(vector<ArmorBlob> &armors, double delta_t, const SerialPortData& imu_data, SerialPort* SerialPort_);
        void sentinelMode(vector<ArmorBlob> &armors, double delta_t, const SerialPortData& imu_data, SerialPort* SerialPort_);
        void clearCircle();
        void clearSentinel();

    private:
        void setCameraMatrix(double fx, double fy, double u0, double v0);
        void setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3);
        void setimu(float pitch, float yaw, float roll);
        //        void setArmorPoints();
        void solveArmor(ArmorBlob& armors, const SerialPortData& imu_data);
        float calcDiff(const ArmorBlob& a, const ArmorBlob& b);
        float cosineLaw(float a, float b, float c);
        int chooseArmor(const vector<ArmorBlob>& armors);
        
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

        Predictor* predictor;

        ArmorBlob last_armor;
        ArmorBlob armor;
        bool right_clicked = true;
        bool last_right_clicked = true;
        bool first = true;
        bool has_same_class = false;
        bool find_outpost = false;
        bool shoot = false;

        int top_pri = 3;

        int top_cnt = 0;
        int lost_cnt = 0;
        std::chrono::time_point<std::chrono::steady_clock> top_begin;
        std::chrono::time_point<std::chrono::steady_clock> top_exit;
        double exit_duration = 0;
        std::chrono::time_point<std::chrono::steady_clock> shoot_begin;
        double shoot_duration = 0;
        std::chrono::time_point<std::chrono::steady_clock> w_begin;
        double w_duration = 0;
        bool w_init = false;

        // double x0, y0, z0, r, angle0;
        Point3d center;
        Point3d cur;
        // double armor_y = 0;

        RollingArray<Point3d> circle = RollingArray<Point3d>(100);
        RollingArray<Point3d> outpost = RollingArray<Point3d>(100);
        RollingArray<double> roll = RollingArray<double>(100);
        RollingArray<Point3d> sentinel = RollingArray<Point3d>(100);
        
        ceres::CostFunction *cost = nullptr;
        ceres::LossFunction* loss = nullptr;

        double speed = 15;
    };

    class DistanceFromCircleCost {
    public:
        DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
        template <typename T> bool operator()(const T* const x,
                                            const T* const y,
                                            const T* const m,  // r = m^2
                                            T* residual) const {
            // Since the radius is parameterized as m^2, unpack m to get r.
            T r = *m * *m;
            // Get the position of the sample in the circle's coordinate system.
            T xp = xx_ - *x;
            T yp = yy_ - *y;
            residual[0] = r*r - xp*xp - yp*yp;
            return true;
        }
    private:
        // The measured x,y coordinate that should be on the circle.
        double xx_, yy_;
    };
}

#endif //AUTOAIM_POSESOLVER_H
