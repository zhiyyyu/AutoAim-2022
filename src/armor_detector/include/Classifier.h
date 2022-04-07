//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_CLASSIFIER_H
#define AUTOAIM_CLASSIFIER_H

#include "opencv2/opencv.hpp"

#include "Log.h"
#include "Detector.h"

using namespace cv;

namespace ly{
    void* getArmorID(void* params_p);

    struct Params_ToClassifier{
        Mat* armor;
        uint8_t * armor_class;

        Params_ToClassifier(){
            armor = new Mat();
            armor_class = new uint8_t;
        }
    };

    class Classifier {
    public:
        explicit Classifier() = default;
        ~Classifier() = default;
        void setParams(const Params_ToDetector &params_to_detector);
        void startClassify(Params_ToClassifier &parmas_to_classifier);

        static Ptr<ml::SVM> svm_model;

    private:
        Params_ToClassifier thread_params;
        pthread_t threadID{};

    };
}


#endif //AUTOAIM_CLASSIFIER_H
