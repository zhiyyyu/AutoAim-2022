//
// Created by zhiyu on 2021/9/11.
//

#include "trainSVM.h"

namespace fs = boost::filesystem;
using namespace ly;

int main(int argc, char** argv){
    auto Score_ = new Score();

    vector<vector<uchar>> samples;
    vector<int> labels;
    int id = 0;

    regex dir_regex("[0-9]+");

    fs::path root(picRoot);
    if(!fs::exists(root)){
        cout << "directory not exits." << endl;
        return -1;
    }

    Mat kernel = getStructuringElement(0, Size(3, 1));
    fs::directory_iterator root_iter(root), end_iter;
    for(;root_iter != end_iter; root_iter++){
        if(fs::is_directory(root_iter->status()) && regex_match(root_iter->path().filename().string(), dir_regex)){
            fs::path picPath(root_iter->path().string());
            fs::directory_iterator picIter(picPath), endIter;
            string label = root_iter->path().filename().string();
            for(; picIter != endIter; picIter++){

                cv::Mat pic = cv::imread(picIter->path().string(), 0);
                equalizeHist(pic, pic);
                normalize(pic, pic, 0, 255, NORM_MINMAX);
//                threshold(pic, pic, 200, 255, THRESH_BINARY);

                cv::MatIterator_<uchar> matIter = pic.begin<uchar>(), end = pic.end<uchar>();
                vector<uchar> picVec;
                for(; matIter != end; matIter++){
                    picVec.emplace_back((uchar)*matIter);
                }
                samples.emplace_back(picVec);
                labels.emplace_back(stoi(label));
//                cout << stoi(label) << " ";
                id++;
            }
//            cout << endl;
        }
    }
    int sampleNum = samples.size();
    int cols = 32;
    int rows = 32;
    cv::Mat sampleMat(sampleNum, rows*cols, CV_32FC1);
    cv::Mat labelMat(sampleNum, 1, CV_32SC1);

    for(int i=0;i<sampleNum;i++){
        for(int j=0;j<rows*cols;j++){
            sampleMat.at<float>(i, j) = (float)samples[i][j];
        }
        labelMat.at<int>(i, 0) = (int)labels[i];
//        cout << labels[i] << " ";
//        cout << labelMat.at<int>(i, 0) << " ";
    }

//    PCA pca = PCA(sampleMat, Mat(), PCA::DATA_AS_ROW, 20);
//    FileStorage storage("../src/utils/tools/pca.xml", FileStorage::WRITE);
//    pca.write(storage);
//    storage.release();
//    sampleMat = pca.project(sampleMat);
    sampleMat.convertTo(sampleMat, CV_32FC1);
    labelMat.convertTo(labelMat, CV_32SC1);
//    cout << sampleMat;
//    cout << labelMat;

    Score_->trainSVM(sampleMat, labelMat);
}