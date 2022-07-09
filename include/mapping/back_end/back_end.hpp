/*
 * @Author: Hu Ziwei 
 * @Description:  slam后端实现
 * @Date: 2021-11-27 15:18:34 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-02 19:37:18
 */

#ifndef LOAM_FRAME_MAPPING_BACK_END_BACK_END_HPP_
#define LOAM_FRAME_MAPPING_BACK_END_BACK_END_HPP_

#include <deque>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "global_defination/global_defination.h"
#include "sensor_data/point_cloud_data.hpp"
#include "tools_/file_manager.hpp"
#include "modules/optimization/g2o/g2o_graph_optimization.hpp"
#include "modules/map/map_handle_interface.hpp"

namespace loam_frame{
class BackEnd{
public:
    BackEnd(ros::NodeHandle& nh);
    ~BackEnd() = default;
    //添加两帧间的约束，会自动添加新节点的
    bool Update(const PointCloudData &segmented_cloud, Eigen::Isometry3d relative_pose, Eigen::Isometry3d gnss_data, bool have_prioi_gnss = false);
    bool GetOptimizedPose(std::deque<Eigen::Isometry3d> &optimized_pose);
    Eigen::Isometry3d GetCurrentOptimizedPose();
    PointCloudData::point_cloud_ptr GetWorldMap();
public:
    std::shared_ptr<G2oGraphOptimizer> graphOptimizationPtr;
    std::shared_ptr<MapHandleInterface> mapHandlePtr;
private:
    bool InitWithConfig();
    bool InitOptimize(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool SavePose(std::ofstream& ofs, const Eigen::Isometry3d& pose);
private:
    Eigen::Isometry3d keyframeRelativePose_;
    Eigen::Isometry3d lastKeyframeOptimizedPose_;
    bool isFristKeyframe_ = true;
    bool need_optimized_ = false;
    bool path_is_update = false;
    
    std::string trajectoryPath_ = "";
    std::string data_path_ = "";
    std::ofstream groundTruthOfs_;
    std::ofstream laserOdomOfs_;
};

}
#endif