/*
 * @Author: Hu Ziwei 
 * @Description:  匹配的接口
 * 目前是scan to scan 和 scan to map 两种
 * @Date: 2021-11-27 15:42:08 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:31:58
 */

#ifndef LOAM_FRAME_MODULES_MATCHING_INTERFACE_MATCHING_HPP_
#define LOAM_FRAME_MODULES_MATCHING_INTERFACE_MATCHING_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include "sensor_data/point_cloud_data.hpp"
#include "modules/feature_extract/feature_extract_interface.hpp"

namespace loam_frame{
class InterfaceMatching{
public:
    virtual ~InterfaceMatching() = default;
    /*
    * @Description: 输入新一帧点云
    */
    virtual bool PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr)=0;
    /*
    * @Description: 输入上一帧到当前帧的预测矩阵
    */
    virtual bool SetPredictPose(const Eigen::Isometry3d &predictPose)=0;
    /*
    * @Description: 输入gnss数据
    */
    virtual bool SetGnssData(const Eigen::Vector3d & gnss_data)=0;
    /*
    * @Description: 输入当前帧在世界地图下的预测位姿
    */
    // virtual bool SetPredictPoseInWorld(const Eigen::Isometry3d &predictPose);
    /*
    * @Description: 输入预测位姿矩阵
    */
    virtual bool SetInitPoseInWorld(const Eigen::Isometry3d &initPose)=0;
    /*
    * @Description: 执行匹配
    */
    virtual bool Match()=0;
    /*
    * @Description: 获取上一帧到当前帧的运动估计矩阵
    */
    virtual Eigen::Isometry3d GetPoseTransformationFromLastToCurrent()=0;
    /*
    * @Description: 获取当前帧雷达的估计位姿
    */
    virtual Eigen::Isometry3d GetCurrentLidarPose()=0;
    /*
    * @Description: 获取位姿真值
    */
    virtual Eigen::Isometry3d GetCurrentLidarTruthPose()=0;

    virtual PointCloudData::point_cloud_ptr GetSubmapCornerPoint(){
        return PointCloudData::point_cloud().makeShared();
    }
    
    virtual PointCloudData::point_cloud_ptr GetSubmapSurfaceoint(){
        return PointCloudData::point_cloud().makeShared();
    }
    
public:
    std::shared_ptr<FeatureExtractInterface> featureExtractPtr;
};

}

#endif
