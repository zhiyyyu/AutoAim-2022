#include "Kalman.h"
namespace ly
{
    Kalman::Kalman()
    {
        KF_ = cv::KalmanFilter(6, 6, 0, CV_32F);
        measurement_ = cv::Mat::zeros(6, 1, CV_32F);
        resetTransitionMatrix();
        setIdentity(KF_.measurementMatrix);
        setIdentity(KF_.processNoiseCov, cv::Scalar::all(0.0001));
        setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(1));
        setIdentity(KF_.errorCovPost, cv::Scalar::all(1));
        randn(KF_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
    }

    Kalman::~Kalman()
    {
    }

    void Kalman::rebootKalman(const ArmorPose &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            //reset kalman previous estimate and corrected estimate
            KF_.statePre.at<float>(i) = new_armor_pose[i];
            KF_.statePre.at<float>(i + 3) = 0.0f;
            KF_.statePost.at<float>(i) = new_armor_pose[i];
            KF_.statePost.at<float>(i + 3) = 0.0f;
        }
        setIdentity(KF_.errorCovPost, cv::Scalar::all(1));
        setIdentity(KF_.errorCovPre, cv::Scalar::all(1));
        resetTransitionMatrix();
    }
    void Kalman::resetKalman()
    {
        is_kalman_init = false;
        is_second_find = false;
        is_continuous_find = false;
    }
    ArmorPose Kalman::runKalman(const ArmorPose &new_armor_pose, const float &delta_t)
    {
        if (!is_kalman_init)
        {
            //set signal values
            is_kalman_init = true;
            is_second_find = true;
            is_continuous_find = false;

            //reset kalman
            rebootKalman(new_armor_pose);
            updateArmorState(new_armor_pose);

            //return values
            return new_armor_pose;
        }
        else if (is_second_find)
        {
            //set signal values
            is_second_find = false;
            is_continuous_find = true;

            //set time
            setUpdateTime(delta_t);

            //calculate speed
            calculateSpeed(new_armor_pose);

            //not use the predict,correct directly
            updateMeasurement(new_armor_pose);

            //return the consequence of corrected armor pose
            return correct();
        }
        else
        {
            //set update time
            setUpdateTime(delta_t);

            //update transition matrix
            setTransitionMatrix(delta_t);

            //calculate armor speed
            calculateSpeed(new_armor_pose);

            //predict by the previous data
            predict();

            updateMeasurement(new_armor_pose);

            return correct();
        }
    }
    ArmorPose Kalman::predict()
    {
        KF_.predict();
        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            this_armor_pre_estimate.pose[i] = KF_.statePre.at<float>(i);
        }
        for (int i = 0; i < 3; i++)
        {
            //update speed and return
            this_armor_pre_estimate.speed[i] = KF_.statePre.at<float>(i + 3);
        }
        return this_armor_pre_estimate.pose;
    }
    void Kalman::updateArmorState(const ArmorPose &new_armor_pose)
    {
        last_armor_state.pose = new_armor_pose;
    }
    ArmorPose Kalman::correct()
    {
        KF_.correct(measurement_);
        for (int i = 0; i < 3; i++)
        {
            //update armor status and return
            last_armor_state.pose[i] = KF_.statePost.at<float>(i);
        }
        for (int i = 0; i < 3; i++)
        {
            //update speed and return
            last_armor_state.speed[i] = KF_.statePost.at<float>(i + 3);
        }
        return last_armor_state.pose;
    }
    void Kalman::setUpdateTime(const float &delta_t)
    {
        if (fabs(delta_t) < 1e-5)
        {
            update_time = 15.0f; //if not given the time,to set the time automatically
        }
        else
        {
            update_time = delta_t;
        }
    }
    void Kalman::updateMeasurement(const ArmorPose &new_armor_pose)
    {
        measurement_ = (cv::Mat_<float>(6, 1) << new_armor_pose[0], new_armor_pose[1],
                        new_armor_pose[2], this_armor_speed[0], this_armor_speed[1], this_armor_speed[2]);
    }
    void Kalman::calculateSpeed(const ArmorPose &new_armor_pose)
    {
        //std::cout<<"new_armor_pose: "<<new_armor_pose<<"last_armor_state.pose: "<<last_armor_state.pose<<std::endl;
        this_armor_speed = (new_armor_pose - last_armor_state.pose) / (update_time / 1000);
    }
    void Kalman::resetTransitionMatrix()
    {
        //x,y,z,x_v,y_v,z_v
        KF_.transitionMatrix = (cv::Mat_<float>(6, 6) << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
                                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1.0 //A 状态转移矩阵
        );
    }
    void Kalman::setTransitionMatrix(float delta_t) //ms
    {
        //x,y,z,x_v,y_v,z_v
        KF_.transitionMatrix = (cv::Mat_<float>(6, 6) << 1.0, 0.0, 0.0, delta_t / 1000, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0, delta_t / 1000, 0.0,
                                0.0, 0.0, 1.0, 0.0, 0.0, delta_t / 1000,
                                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1.0 //A 状态转移矩阵
        );
    }
}
