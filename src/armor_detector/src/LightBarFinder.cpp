//
// Created by zhiyu on 2021/8/24.
//

#include "LightBarFinder.h"

using namespace ly;
using namespace cv;

bool LightBarFinder::findLightBarBlobs(const Mat &frame, LightBarBlobs &lightBarBlobs) {
    Mat mat;
    vector<Mat> channels;
    vector<vector<Point>> contours;
    split(frame, channels);
    if(DetectorParam::color == "blue"){
        subtract(channels[0], channels[2], mat);
    } else if(DetectorParam::color == "red"){
        subtract(channels[2], channels[0], mat);
    }
    threshold(mat, mat, DetectorParam::thresh, 255, THRESH_BINARY);
    
    if(GlobalParam::SHOW_THRESH){
        imshow("thresh", mat);
        waitKey(1);
    }
    
    findContours(mat, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for(const auto& contour: contours){
        const RotatedRect& rrect = minAreaRect(contour);
        if(isValidLightBarBlob(rrect)){
            lightBarBlobs.emplace_back(rrect);
        }
    }

    sort(lightBarBlobs.begin(), lightBarBlobs.end(), [](const RotatedRect& a, const RotatedRect& b)->bool{
        if(a.center.x != b.center.x) return a.center.x < b.center.x;
        return a.center.y > b.center.y;
    });
    return lightBarBlobs.size() >= 2;
}

bool LightBarFinder::isValidLightBarBlob(const RotatedRect& rrect){
    if(
            checkAspectRatio(rrect.size.aspectRatio()) &&
            checkArea(rrect.size.area())
            ){
        return true;
    }
    return false;
}

bool LightBarFinder::checkAspectRatio(double ratio) {
    return ratio <= 10 && ratio >= 2;
}

bool LightBarFinder::checkArea(double area) {
    return area >= 20;
}

bool LightBarFinder::checkAngle(double angle) {
    return abs(angle) <= 20;
}

LightBarFinder::LightBarFinder(){
    kernel = getStructuringElement(0, Size(3, 3));
}

