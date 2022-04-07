//
// Created by zhiyu on 2021/8/20.
//

#ifndef PRIDICTOR_H
#define PRIDICTOR_H

#include "Config.h"
#include <utility>
#include "Kalman.h"

using namespace cv;
using namespace std;



namespace ly
{
    class Predictor
    {
    public:
        Predictor();
        pair<double, double> predict(const Point3d& trans, double v);
        pair<float, float> getCurrentPos();
    private:
        KalmanFilter predictor;
        Mat measurement;
    };

}

#endif //AUTOAIM_POSESOLVER_H
