/*
 * @Author: Hu Ziwei 
 * @Description:  slam前端实现
 * @Date: 2021-11-27 15:18:34 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-02 19:36:50
 */

#ifndef LOAM_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_
#define LOAM_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <Eigen/Dense>

#include "global_defination/global_defination.h"
#include "sensor_data/point_cloud_data.hpp"
#include "modules/matching/interface_matching.hpp"
#include "modules/matching/scan_to_scan.hpp"
#include "modules/matching/scan_to_map.hpp"
#include "tools_/file_manager.hpp"

namespace loam_frame{
class FrontEnd{
public:
    FrontEnd();
    ~FrontEnd() = default;
    bool Update(const PointCloudData::point_cloud & cloud_data);
    bool SetInitPose(const Eigen::Isometry3d& init_pose);
    bool SetPredictPose(const Eigen::Isometry3d& predict_pose);
    bool SetGnssData(const Eigen::Vector3d & gnss_data);
    Eigen::Isometry3d GetMotion();//上一帧到当前帧的位姿移动
    Eigen::Isometry3d GetCurrentLidarPose();//获取当前帧雷达在世界坐标系下的位姿
    Eigen::Isometry3d GetCurrentLidarTruthPose();
public:
    std::shared_ptr<InterfaceMatching> MatchingPtr;
private:
    bool InitWithConfig();
    bool InitMatching(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool SavePose(std::ofstream& ofs, const Eigen::Isometry3d& pose);
private:
    std::string trajectoryPath_ = "";
    std::string data_path_ = "";
    std::ofstream groundTruthOfs_;
    std::ofstream laserOdomOfs_;
};

}
#endif