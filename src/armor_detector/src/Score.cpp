/**
 * @file
 * @author
 * @brief
 *
 * @note
 *
 */
#include "Score.h"

using namespace cv;

namespace ly
{

    void Score::trainSVM(Mat sampleMat, Mat labelMat) {
        /*************** show data ******************/
//        int cols = 32;
//        int rows = 32;
//        for(int i=0;i<sampleMat.rows;i++){
//            for(int j=0;j<rows*cols;j++){
//                cout << sampleMat.at<float>(i, j) << " ";
//            }
//            cout << endl;
//            cout << labelMat.at<int>(i, 0) << endl;
//        }
        /**************** end of show data ***********/
        // create SVM
        svm = ml::SVM::create();
        // set params
        svm->setType(ml::SVM::C_SVC);
        svm->setKernel(ml::SVM::LINEAR);
        svm->setC(10);

        svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 3000, 1e-7));
        // train
        Ptr<ml::TrainData> trainData = ml::TrainData::create(sampleMat, ml::ROW_SAMPLE, labelMat);
        svm->train(trainData);
        svm->save(svmModel);
    }
}


