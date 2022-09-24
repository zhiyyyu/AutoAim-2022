//
// Created by zhiyu on 2021/8/24.
//

#include "ArmorFinder.h"

using namespace ly;

bool ArmorFinder::matchTwoLightBar(const RotatedRect &l, const RotatedRect &r) {
    status = checkAngleDiff(l, r);
    // DLOG_IF(INFO, !status) << "angle differs: left: " << getAngle(l) << " right: " << getAngle(r);
    if(!status) return false;
    status = checkHeightDiff(l, r);
    // DLOG_IF(INFO, !status) << "height differs: " << abs((l.center-r.center).y);
    if(!status) return false;
    // status = checkRatio(l, r);
    // if(!status) return false;
    status = checkHeightMatch(l, r);
    // DLOG_IF(INFO, status) << "height differs: " << l.size.height << " " << r.size.height;
    if(!status) return false;
    return true;
}

bool ArmorFinder::judgeArmor(const ArmorBlob &armor_blob) {
    return armor_blob.rect.size().aspectRatio() <= 5 && armor_blob.rect.size().aspectRatio() > 1.5;
}

/***
 * @brief 将两个灯条拼接成一个装甲
 * @param l
 * @param r
 * @param armor
 * @return
 */
bool ArmorFinder::getArmor(const RotatedRect &l, const RotatedRect &r, ArmorBlob& armor) {
    Point2f points_of_rrect[4];
    l.points(points_of_rrect);
    float height = fmax(l.size.width, l.size.height);
    armor.rect = Rect(l.center.x, l.center.y-height/2, r.center.x-l.center.x, height);

    // armor
    // 0 1
    // 3 2
//    cout << l.angle << " " << r.angle << endl;

    // if(l.angle > 45){
    //     armor.corners[0] = points_of_rrect[0];
    //     armor.corners[3] = points_of_rrect[3];
    // } else{
    //     armor.corners[0] = points_of_rrect[1];
    //     armor.corners[3] = points_of_rrect[0];
    // }
    // r.points(points_of_rrect);
    // if(r.angle > 45){
    //     armor.corners[1] = points_of_rrect[1];
    //     armor.corners[2] = points_of_rrect[2];
    // } else{
    //     armor.corners[1] = points_of_rrect[2];
    //     armor.corners[2] = points_of_rrect[3];
    // }

    if(l.angle > 45){
        armor.corners[0] = points_of_rrect[1];
        armor.corners[3] = points_of_rrect[2];
    } else{
        armor.corners[0] = points_of_rrect[2];
        armor.corners[3] = points_of_rrect[3];
    }
    r.points(points_of_rrect);
    if(r.angle > 45){
        armor.corners[1] = points_of_rrect[0];
        armor.corners[2] = points_of_rrect[3];
    } else{
        armor.corners[1] = points_of_rrect[1];
        armor.corners[2] = points_of_rrect[0];
    }

    if(max(armor.corners[0].x, armor.corners[3].x) > min(armor.corners[1].x, armor.corners[2].x)) return false;
    return true;
}

bool ArmorFinder::checkAngleDiff(const RotatedRect &l, const RotatedRect &r) {
    const float & angle_l = getAngle(l);
    const float & angle_r = getAngle(r);
    return abs(angle_l-angle_r) < 30;
}

float ArmorFinder::getAngle(const RotatedRect &rrect) {
    return rrect.size.width > rrect.size.height ? rrect.angle-90 : rrect.angle;
}

bool ArmorFinder::checkHeightDiff(const RotatedRect &l, const RotatedRect &r) {
    const Point2f& diff = l.center - r.center;
    return abs(diff.y) < min(max(l.size.height, l.size.width), max(r.size.height, r.size.width))*1.5;
}

bool ArmorFinder::checkHorizontalDistance(const RotatedRect &l, const RotatedRect &r) {
    return false;
}

bool ArmorFinder::checkDislocation(const RotatedRect &l, const RotatedRect &r) {
    return false;
}

// Rect ArmorFinder::getScaleArmorToRoi(const Rect& rect) {
//     // w*5/4 h*2
//     int x = max(0, rect.x-rect.width/8);
//     int y = max(0, rect.y-rect.height/2);
//     int width = min(x + rect.width*5/4, 1280) - x;
//     int height = min(y + rect.height*2, 1024) - y;
//     return {x, y, width, height};
// }

vector<int> ArmorFinder::getExtreme(const ArmorBlob& armor) {
    int x1 = min(1280.f, max(0.f, min(armor.corners[0].x, armor.corners[3].x)));
    int y1 = min(1280.f, max(0.f, min(armor.corners[0].y, armor.corners[1].y) - min(armor.rect.height, armor.rect.width)/2.5f));
    int x2 = min(1280.f, max(0.f, max(armor.corners[2].x, armor.corners[1].x)));
    int y2 = min(1024.f, max(0.f, max(armor.corners[2].y, armor.corners[3].y) + min(armor.rect.height, armor.rect.width)/2.5f));
    
    return {x1, y1, x2, y2};
}

bool ArmorFinder::checkHeightMatch(const RotatedRect &l, const RotatedRect &r) {
    float lh = max(l.size.height, l.size.width);
    float rh = max(r.size.height, r.size.width);
    return min(lh, rh) * 2 > max(lh, rh);
}


