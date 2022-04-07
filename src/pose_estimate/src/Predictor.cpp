//
// Created by zhiyu on 2022/04/06.
//

#include "Predictor.h"
namespace ly
{

    Predictor::Predictor()
    {
        predictor = KalmanFilter(2, 2, 0);        // 状态量：2，预测量：2，控制量：0
        measurement = Mat::zeros(2, 1, CV_32F);   // 观测矩阵H
        predictor.transitionMatrix = (Mat_<float>(4, 4) << 
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);                          // 状态转移矩阵A
        setIdentity(predictor.measurementMatrix); // 观测矩阵H
        setIdentity(predictor.processNoiseCov, cv::Scalar::all(0.01));        // 过程噪声Q
        setIdentity(predictor.measurementNoiseCov, cv::Scalar::all(0.001));   // 测量噪声R
        setIdentity(predictor.errorCovPost, cv::Scalar::all(0.1));            // 真实噪声P
        randn(predictor.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1)); // 随机初始状态
    }
    pair<double, double> Predictor::predict(const Point3d& trans, double v)
    {
        double distance = sqrt(trans.x*trans.x+trans.z*trans.z);
        double cur = atan(trans.y/distance);
        double delta_y;
        for(int i=0;i<10;i++){
            delta_y = trans.y - distance*tan(cur) + 4.9 * distance*distance/pow(v*cos(cur), 2);
            if(abs(delta_y) < 0.00001) break;
            cur -= delta_y / (- distance / pow(cos(cur), 2) + 9.8 * distance * distance / (v * v) * sin(cur) / pow(cos(cur), 3));
        }
        measurement.at<float>(0) = trans.x;
        measurement.at<float>(1) = trans.z;
        predictor.correct(measurement);
        const Mat& coord = predictor.predict();
        double pred_yaw = SerialParam::recv_data.yaw + (-atan(coord.at<float>(0)/coord.at<float>(1))/M_PI*180-2.0)*100;
        return {cur/M_PI*180+3.0, pred_yaw};
    }
    pair<float, float> Predictor::getCurrentPos(){
        return {predictor.statePost.at<float>(0), predictor.statePost.at<float>(1)};
    };
}
