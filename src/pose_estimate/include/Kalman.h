/**
 * dependence:opencv,eigen,sophus
 * usage:
 *      use runKalman(const ArmorPose &new_armor_pose, const float &delta_t)
 *      input the location vector (x,y,z) relative to camera , and the delta_t
 *      the function will return the vector predicted(corrected)
 * 
 *      if the speed or pose of armor need to be used after call runKalman()
 *          use getSpeed() to get the predict speed
 *          use getPose() to get the predict armor pose
 * 
 *      if the target has lost(this should be judged outside),use resetKalman() to reset Kalman Filter
 *
**/
#ifndef _KALMAN_H
#define _KALMAN_H
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
namespace ly
{
    typedef Eigen::Vector3d ArmorPose;  //x,y,z translation vector supported by pnp
    typedef Eigen::Vector3d ArmorSpeed; //x_v,y_v,z_v
    class Kalman
    {
        struct ArmorState
        {
            ArmorPose pose;
            ArmorSpeed speed;
        };
    public:
        cv::KalmanFilter KF_;
    private:
        cv::Mat measurement_;

        bool is_kalman_init = false; //kalman init symbol
        bool is_second_find = false;
        bool is_continuous_find = false; //continously find armor

        void resetTransitionMatrix();
        void setTransitionMatrix(float delta_t); //ms
        void updateArmorState(const ArmorPose &new_armor_pose);
        void setUpdateTime(const float &delta_t);
        void calculateSpeed(const ArmorPose &new_armor_pose);
        void updateMeasurement(const ArmorPose &new_armor_pose);
        void rebootKalman(const ArmorPose &new_armor_pose);
        ArmorPose correct();
        ArmorPose predict();

        ArmorState last_armor_state;
        ArmorState this_armor_pre_estimate;
        ArmorSpeed this_armor_speed; //the newest armor speed estimate
        float update_time;

    public:
        Kalman(/* args */);
        ArmorPose runKalman(const ArmorPose &new_armor_pose, const float &delta_t);
        ArmorSpeed getSpeed() { return last_armor_state.speed; };
        ArmorPose getPose() { return last_armor_state.pose; };
        void resetKalman();
        ~Kalman();
    };
}
#endif
