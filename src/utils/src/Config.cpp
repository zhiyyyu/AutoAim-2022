//
// Created by zhiyu on 2021/8/20.
//

#include "../include/Config.h"

using namespace ly;
using namespace std;
using namespace Json;

Config::Config(const string &path)
{
    this->json_file_path = path;
}

void Config::parse()
{
    fstream json_file(json_file_path, std::ios::in);
    LOG_IF(ERROR, !json_file.is_open()) << "can't find json file in " << json_file_path;
    LOG_IF(INFO, json_file.is_open()) << "successfully open json file: " << json_file_path;

    JSONCPP_STRING errs;
    CharReaderBuilder builder;
    Value root;
    bool status = Json::parseFromStream(builder, json_file, &root, &errs);
    LOG_IF(ERROR, !status) << "json file parse error!";
    /*** camera param ***/
    CameraParam::device_type = root["camera"]["device_type"].asInt();
    CameraParam::sn = root["camera"]["sn"].asString();
    CameraParam::video_path = root["camera"]["video_path"].asString();
    CameraParam::picture_path = root["camera"]["picture_path"].asString();
    CameraParam::camera_type = root["camera"]["camera_type"].asInt();
    DLOG(INFO) << "camera type: " << CameraParam::camera_type << "mm";
    if(CameraParam::camera_type == 8){
        CameraParam::exposure_time = root["camera"]["camera_8mm_param"]["exposure_time"].asInt();
        CameraParam::gain = root["camera"]["camera_8mm_param"]["gain"].asDouble();
        CameraParam::gamma = root["camera"]["camera_8mm_param"]["gamma"].asFloat();
        CameraParam::fx = root["camera"]["camera_8mm_param"]["fx"].asDouble();
        CameraParam::fy = root["camera"]["camera_8mm_param"]["fy"].asDouble();
        CameraParam::u0 = root["camera"]["camera_8mm_param"]["u0"].asDouble();
        CameraParam::v0 = root["camera"]["camera_8mm_param"]["v0"].asDouble();
        CameraParam::k1 = root["camera"]["camera_8mm_param"]["k1"].asDouble();
        CameraParam::k2 = root["camera"]["camera_8mm_param"]["k2"].asDouble();
        CameraParam::k3 = root["camera"]["camera_8mm_param"]["k3"].asDouble();
        CameraParam::p1 = root["camera"]["camera_8mm_param"]["p1"].asDouble();
        CameraParam::p2 = root["camera"]["camera_8mm_param"]["p2"].asDouble();
    } else{
        CameraParam::exposure_time = root["camera"]["camera_param"]["exposure_time"].asInt();
        CameraParam::gain = root["camera"]["camera_param"]["gain"].asDouble();
        CameraParam::gamma = root["camera"]["camera_param"]["gamma"].asFloat();
        CameraParam::fx = root["camera"]["camera_param"]["fx"].asDouble();
        CameraParam::fy = root["camera"]["camera_param"]["fy"].asDouble();
        CameraParam::u0 = root["camera"]["camera_param"]["u0"].asDouble();
        CameraParam::v0 = root["camera"]["camera_param"]["v0"].asDouble();
        CameraParam::k1 = root["camera"]["camera_param"]["k1"].asDouble();
        CameraParam::k2 = root["camera"]["camera_param"]["k2"].asDouble();
        CameraParam::k3 = root["camera"]["camera_param"]["k3"].asDouble();
        CameraParam::p1 = root["camera"]["camera_param"]["p1"].asDouble();
        CameraParam::p2 = root["camera"]["camera_param"]["p2"].asDouble();
    }
    
    CameraParam::camera_trans_x = root["camera"]["camera_trans"]["x"].asDouble();
    CameraParam::camera_trans_y = root["camera"]["camera_trans"]["y"].asDouble();
    CameraParam::camera_trans_z = root["camera"]["camera_trans"]["z"].asDouble();

    DetectorParam::color = root["detector"]["color"].asString();
    DetectorParam::thresh = root["detector"]["thresh"].asInt();

    SerialParam::device_name = root["serialport"]["deviceName"].asString();

    GlobalParam::DEBUG_MODE = root["debug"]["enable"].asBool();
    GlobalParam::SAVE_VIDEO = root["debug"]["video"].asBool();
    GlobalParam::SAVE_ARMOR = root["debug"]["armor"].asBool();
    GlobalParam::SHOW_THRESH = root["debug"]["thresh"].asBool();
    GlobalParam::save_step = root["debug"]["save_step"].asInt();
    GlobalParam::SHOW_COORD = root["debug"]["coord"].asBool();
    GlobalParam::SOCKET = root["debug"]["socket"].asBool();

    json_file.close();
}
