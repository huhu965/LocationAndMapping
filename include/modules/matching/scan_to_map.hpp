/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-08 13:23:32
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-08 15:55:20
 */

#ifndef LOAM_FRAME_MODULES_REGISTRATION_SCAN_TO_MAP_ICP__HPP_
#define LOAM_FRAME_MODULES_REGISTRATION_SCAN_TO_MAP_ICP__HPP_

#include <algorithm>
#include <vector>
#include <deque>

#include <yaml-cpp/yaml.h>

#include "modules/matching/interface_matching.hpp"
#include "modules/registration/registration_interface.hpp"
#include "modules/transformation/pose_transformation.hpp"
#include "modules/cloud_filter/cloud_filter_interface.hpp"
#include "data_struct/register_struct.hpp"

namespace loam_frame{
class ScanToMap: public InterfaceMatching{
public:
    ScanToMap(const YAML::Node& configNode);
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

    PointCloudData::point_cloud_ptr GetSubmapCornerPoint() override;
    PointCloudData::point_cloud_ptr GetSubmapSurfaceoint() override;
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
     * @Description: 按照配置初始化滤波器 
    */
    bool InitFilter(const YAML::Node& config_node, std::shared_ptr<CloudFilterInterface> &filterPtr);
    /*
    * @Description: 判断是否添加新关键帧
    */
    bool IsNewKeyFrame();
    /*
    * @Description: 添加新关键帧
    */    
    void AddKeyFrame();
    /*
    * @Description: 生成新子地图
    */ 
    void GenerateNewSubmap();
    /*
    * @Description: 更新位姿
    */
    void UpdatePose();
private:
    PointCloudData::point_cloud_ptr newPointCloud_;

    PointCloudData::point_cloud_ptr currentFrameCornerPointsPtr_; // 当前帧的角点集合
    PointCloudData::point_cloud_ptr currentFrameSurfacePointsPtr_; // 当前帧的角点集合

    PointCloudData::point_cloud_ptr cornerPointsFromSubmapPtr_; // 局部map的角点集合
    PointCloudData::point_cloud_ptr surfPointsFromSubmapPtr_; // 局部map的平面点集合
    
    std::shared_ptr<RegistrationInterface> registrationPtr_;
    std::shared_ptr<CloudFilterInterface> downSizeFilterPtr_;//降采样滤波
    std::shared_ptr<CloudFilterInterface> downSizeFilterSurroundingKeyPoses_;//降采样滤波
    PoseTransformation poseSet_;

    std::deque<Eigen::Isometry3d> keyframePose_; //关键帧位姿
    std::deque<PointCloudData::point_cloud_ptr> cornerPointsBuf_; //角点
    std::deque<PointCloudData::point_cloud_ptr> surfacePointsBuf_; //平面点

    PointCloudData::point_cloud_ptr translationKeyFramePtr_;

    bool isFristFrame_;
    double KeyframeAddDistanceThreshold_; //添加关键帧距离阈值
    double KeyframeAddAngleThreshold_; //添加关键帧角度阈值
    int submapFrameNumber_; //组成子地图的帧数

};

}

#endif  //LOAM_FRAME_MODULES_REGISTRATION_SCAN_TO_MAP_ICP__HPP_