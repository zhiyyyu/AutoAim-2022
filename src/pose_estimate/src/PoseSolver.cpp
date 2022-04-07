//
// Created by zhiyu on 2021/8/20.
//

#include "../include/PoseSolver.h"
#include "../../armor_detector/include/ArmorFinder.h"

using namespace std;
namespace ly
{

    PoseSolver::PoseSolver()
    {
        Predictor predictor();
        setCameraMatrix(CameraParam::fx, CameraParam::fy, CameraParam::u0, CameraParam::v0);
        setDistortionCoefficients(CameraParam::k1, CameraParam::k2, CameraParam::p1, CameraParam::p2, CameraParam::k3);

        gimbal_to_world = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
        camera_to_gimbal = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(CameraParam::camera_trans_x, CameraParam::camera_trans_y, CameraParam::camera_trans_z));
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

    pair<Sophus::SE3, double> PoseSolver::getPoseInCamera(const ArmorBlob &armor, const SerialPortData& imu_data)
    {
         const static vector<Point3f> points_small_3d = {Point3f(-0.0650f, -0.0275f, 0.f),
                                                         Point3f(0.0650f, -0.0275f, 0.f),
                                                         Point3f(0.0650f, 0.0275f, 0.f),
                                                         Point3f(-0.0650f, 0.0275f, 0.f)};
                                                         
        const static vector<Point3f> points_large_3d = {Point3f(-0.1150f, -0.0275f, 0.f),
                                                         Point3f(0.1150f, -0.0275f, 0.f),
                                                         Point3f(0.1150f, 0.0275f, 0.f),
                                                          Point3f(-0.1150f, 0.0275f, 0.f)};
                                                          
        const static Sophus::SE3 armor_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

        if (armor._class == 1 || armor._class == 6)
        { // large armor
            solvePnP(points_large_3d, armor.corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        }
        else
        { // small armor
            solvePnP(points_small_3d, armor.corners, camera_matrix, distortion_coefficients,
                     rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        }
        
        float rcv_yaw = imu_data.yaw / 100.0f * M_PI / 180.0f;
        float rcv_pitch = imu_data.pitch / 100.0f * M_PI / 180.0f;
        setimu(rcv_pitch, rcv_yaw, 0);
        
        // DLOG(INFO) << "                              corners: " << armor.corners;
        
        cv2eigen(tvec, e_T);
        Rodrigues(rvec, m_R);
        cv2eigen(m_R, e_R);
        
        yaw = atan2(-m_R.at<double>(2, 0), sqrt(pow(m_R.at<double>(2, 0), 2) + pow(m_R.at<double>(2, 2), 2))) / M_PI * 180;
        
        armor_to_camera = Sophus::SE3(e_R, e_T) * armor_pose;
//        DLOG(INFO) << "                                           armor_to_camera x: " << armor_to_camera.translation()[0] << " y: " << armor_to_camera.translation()[1] << " z: " << armor_to_camera.translation()[2];
        SE3 camera_to_world = gimbal_to_world * camera_to_gimbal;
//        DLOG(INFO) << "                                           camera_to_world x: " << camera_to_world.translation()[0] << " y: " << camera_to_world.translation()[1] << " z: " << camera_to_world.translation()[2];
        armor_to_world = camera_to_world * armor_to_camera;
        
        const auto& trans = armor_to_camera.translation();
        SerialParam::send_data.yaw = imu_data.yaw + (atan(-trans[0]/trans[2])/M_PI*180-2.0)*100;
        DLOG(INFO) << "                                           send yaw: " << SerialParam::send_data.yaw;
        
        return {armor_to_world, yaw};
    }

}
