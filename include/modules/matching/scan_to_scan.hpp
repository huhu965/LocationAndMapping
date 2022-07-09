/*
 * @Author: Hu Ziwei 
 * @Description:  scan to scan匹配
 * @Date: 2021-11-27 16:06:05 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:31:53
 */

#ifndef LOAM_FRAME_MODULES_MATCHING_SCAN_TO_SCAN_HPP_
#define LOAM_FRAME_MODULES_MATCHING_SCAN_TO_SCAN_HPP_

#include <yaml-cpp/yaml.h>

#include "modules/matching/interface_matching.hpp"
#include "modules/registration/registration_interface.hpp"
#include "modules/transformation/pose_transformation.hpp"

namespace loam_frame{
class ScanToScan: public InterfaceMatching{
public:
    ScanToScan(const YAML::Node& configNode);
    /*
    * @Description: 输入新一帧点云
    */
    bool PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr) override;
    /*
    * @Description: 执行匹配
    */
    bool Match() override;
    bool SetPredictPose(const Eigen::Isometry3d &predictPose) override;
    bool SetInitPoseInWorld(const Eigen::Isometry3d &initPose) override;
    bool SetGnssData(const Eigen::Vector3d & gnss_data) override;
    /*
    * @Description: 返回上一帧到当前帧的估计运动矩阵
    */
    Eigen::Isometry3d GetPoseTransformationFromLastToCurrent() override;
    Eigen::Isometry3d GetCurrentLidarPose() override;
    Eigen::Isometry3d GetCurrentLidarTruthPose() override;
private:
    /*
    * @Description: 初始化配置
    */
    bool InitWithConfig(const YAML::Node& configNode);
    /*
    * @Description: 初始化配准方式
    */
    bool InitRegistration(const YAML::Node& configNode);
    /*
    * @Description: 选择特征提取方式
    */
    bool InitFeatureExtract(const YAML::Node& configNode);
    /*
    * @Description: 更新位姿
    */
    void UpdatePose();
    /*
    * @Description: 保存新一帧数据
    */
    void SaveNewFrame();
// public:
//     std::shared_ptr<FeatureExtractInterface> featureExtractPtr;
private:
    PointCloudData::point_cloud_ptr newPointCloud_;
    std::shared_ptr<RegistrationInterface> registrationPtr_;

    PointCloudData::point_cloud_ptr lastCornerPointCloudPtr_;
    PointCloudData::point_cloud_ptr lastSurfacePointCloudPtr_;
    PoseTransformation poseSet_;

    bool fixedCoordinateSystem_;
    bool isFristFrame_;

};

}
#endif 