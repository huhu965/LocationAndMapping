/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-30 19:18:17 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-02 21:06:56
 */

#include "glog/logging.h"

#include "mapping/front_end/front_end.hpp"



namespace loam_frame{

FrontEnd::FrontEnd(){
    InitWithConfig();
}

bool FrontEnd::Update(const PointCloudData::point_cloud & cloud_data){
    MatchingPtr->PointCloudInput(cloud_data.makeShared());
    MatchingPtr->Match();
    SavePose(laserOdomOfs_, MatchingPtr->GetCurrentLidarPose());
    SavePose(groundTruthOfs_, MatchingPtr->GetCurrentLidarTruthPose());
    return true;
}
//输入的是两帧间的运动估计
bool FrontEnd::SetPredictPose(const Eigen::Isometry3d& predict_pose){
    MatchingPtr->SetPredictPose(predict_pose);
    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Isometry3d& init_pose){
    MatchingPtr->SetInitPoseInWorld(init_pose);
    return true;
}

bool FrontEnd::SetGnssData(const Eigen::Vector3d & gnss_data){
    MatchingPtr->SetGnssData(gnss_data);
}

Eigen::Isometry3d FrontEnd::GetMotion(){
    return MatchingPtr->GetPoseTransformationFromLastToCurrent();
}

Eigen::Isometry3d FrontEnd::GetCurrentLidarPose(){
    return MatchingPtr->GetCurrentLidarPose();
}

Eigen::Isometry3d FrontEnd::GetCurrentLidarTruthPose(){
    return MatchingPtr->GetCurrentLidarTruthPose();
}

bool FrontEnd::InitWithConfig(){
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------前端初始化-------------------" << std::endl;
    InitMatching(config_node);
    InitDataPath(config_node);
    return true;
}

bool FrontEnd::InitMatching(const YAML::Node& config_node){
    std::string _matching_method = config_node["match_method"].as<std::string>();
    std::string _matching_path = config_node["path"].as<std::string>();
    std::string _matching_config_file_path = WORK_SPACE_PATH + _matching_path;
    YAML::Node _match_config_node = YAML::LoadFile(_matching_config_file_path);
    std::cout << "前端选择的点云匹配方式为：" << _matching_method << std::endl;

    if (_matching_method == "scan_to_scan") {
        MatchingPtr = std::make_shared<ScanToScan>(_match_config_node);
    } else if(_matching_method == "scan_to_map"){
        MatchingPtr = std::make_shared<ScanToMap>(_match_config_node);
    }
    else {
        LOG(ERROR) << "没找到与 " << _matching_method << " 相对应的点云匹配方式!";
        return false;
    }
    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node){
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./"){
        data_path = WORK_SPACE_PATH;
    }else{
        data_path = WORK_SPACE_PATH + data_path;
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

    trajectoryPath_ = data_path + "/slam_data/trajectory";


    if (!FileManager::InitDirectory(trajectoryPath_, "轨迹文件"))
        return false;

    if (!FileManager::CreateFile(groundTruthOfs_, trajectoryPath_ + "/ground_truth.txt"))
        return false;
    if (!FileManager::CreateFile(laserOdomOfs_, trajectoryPath_ + "/laser_odom.txt"))
        return false;
    std::cout << "轨迹存放位置：" << trajectoryPath_ << std::endl;
    return true;
}

bool FrontEnd::SavePose(std::ofstream& ofs, const Eigen::Isometry3d& pose){
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i,j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }
    return true;
}

}
