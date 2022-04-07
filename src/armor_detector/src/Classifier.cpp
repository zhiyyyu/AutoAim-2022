//
// Created by zhiyu on 2021/8/20.
//

#include "Classifier.h"

using namespace ly;

void* ly::getArmorID(void *params_p) {
    auto params = reinterpret_cast<Params_ToClassifier*>(params_p);
}

void Classifier::setParams(const Params_ToDetector &params_to_detector) {
    thread_params.armor = params_to_detector.armor;
}

void Classifier::startClassify(Params_ToClassifier &parmas_to_classifier) {

    int nRet = pthread_create(&threadID, nullptr, getArmorID, static_cast<void*>(&thread_params));
    LOG_IF(ERROR, nRet) << "error in creating classify thread" ;

}
