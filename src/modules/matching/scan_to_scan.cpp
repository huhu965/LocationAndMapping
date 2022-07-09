/*
 * @Author: Hu Ziwei 
 * @Description:  scan to scan匹配
 * @Date: 2021-11-27 16:21:16 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:02:26
 */

#include "glog/logging.h"

#include "modules/matching/scan_to_scan.hpp"
#include "modules/registration/ICP/aloam_registration.hpp"
#include "modules/registration/ICP/fast_registration.hpp"
#include "modules/feature_extract/go_feature_extract.hpp"
#include "modules/feature_extract/normal_feature_extract.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"

namespace loam_frame{
    
ScanToScan::ScanToScan(const YAML::Node& configNode)
    :newPointCloud_(new PointCloudData::point_cloud()),
    lastCornerPointCloudPtr_(new PointCloudData::point_cloud()),
    lastSurfacePointCloudPtr_(new PointCloudData::point_cloud()){
    
    isFristFrame_ = true;
    InitWithConfig(configNode);
}

bool ScanToScan::PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr){
    newPointCloud_ = pointCloudPtr;
    return true;
}

bool ScanToScan::InitWithConfig(const YAML::Node& configNode){
    fixedCoordinateSystem_ = configNode["fixed_coordinate"].as<bool>();

    InitFeatureExtract(configNode);
    InitRegistration(configNode);
    return true;
}

bool ScanToScan::InitFeatureExtract(const YAML::Node& configNode){
    std::string _featureExtractMethod = configNode["feature_extract_method"].as<std::string>();
    std::cout << "ScanToScan特征提取方式为：" << _featureExtractMethod << std::endl;

    if (_featureExtractMethod == "ground_optimize") {
        featureExtractPtr = std::make_shared<GoFeatureExtract>(configNode[_featureExtractMethod]);
    } else if (_featureExtractMethod == "normal") {
        featureExtractPtr = std::make_shared<NormalFeatureExtract>(configNode[_featureExtractMethod]);
    } else {
        LOG(ERROR) << "没找到与 " << _featureExtractMethod << " 相对应的特征提取方式!";
        return false;
    }
    return true;
}

bool ScanToScan::InitRegistration(const YAML::Node& configNode){
    std::string _registrationMethod = configNode["registration_method"].as<std::string>();
    std::cout << "ScanToScan点云配准方式为：" << _registrationMethod << std::endl;

    if (_registrationMethod == "aloam") {
        registrationPtr_ = std::make_shared<AloamRegistration>(configNode[_registrationMethod]);
    }else if(_registrationMethod == "fast"){
        registrationPtr_ = std::make_shared<FastRegistration>(configNode[_registrationMethod]);
    }else {
        LOG(ERROR) << "没找到与 " << _registrationMethod << " 相对应的点云匹配方式!";
        return false;
    }
    return true;
}

void ScanToScan::UpdatePose(){
    //更新位姿结果
    if(fixedCoordinateSystem_){
        //如果是固定坐标系的话，返回的结果是世界坐标系的位姿
        poseSet_.currentFrameLidarPoseInWorldMap = registrationPtr_->GetMatchResult();
        //相邻帧的运动估计值
        poseSet_.poseTransformationFromLastToCurrent = poseSet_.lastFrameLidarPoseInWorldMap.inverse() *  poseSet_.currentFrameLidarPoseInWorldMap;
        //用当前帧位姿作为下一帧的上一位姿
        poseSet_.lastFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap;
        //预测当前到下一帧的运动，如果给的话，就会覆盖这个值
        poseSet_.predictNextposeTransformation = poseSet_.poseTransformationFromLastToCurrent;
        poseSet_.predictNextFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap * poseSet_.predictNextposeTransformation;
    }else{
        //如果是非固定坐标系，返回的结果是相邻两帧的变换位姿
        poseSet_.poseTransformationFromLastToCurrent = registrationPtr_->GetMatchResult();
        //雷达当前所在位姿,也就是累计运动
        poseSet_.currentFrameLidarPoseInWorldMap = poseSet_.lastFrameLidarPoseInWorldMap * poseSet_.poseTransformationFromLastToCurrent;
        poseSet_.lastFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap;
        //预测当前到下一帧的运动，如果给的话，就会覆盖这个值
        poseSet_.predictNextposeTransformation = poseSet_.poseTransformationFromLastToCurrent;
        poseSet_.predictNextFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap * poseSet_.predictNextposeTransformation;
    }
}

void ScanToScan::SaveNewFrame(){
    lastCornerPointCloudPtr_->clear();
    lastSurfacePointCloudPtr_->clear();
    *lastCornerPointCloudPtr_ = *featureExtractPtr->GETLessSharpCornerPoints();
    *lastSurfacePointCloudPtr_ = *featureExtractPtr->GETLessFlatSurfacePoints();

    if(fixedCoordinateSystem_){
        PointCloudTransformation::TransformToWorld(lastCornerPointCloudPtr_, lastCornerPointCloudPtr_, poseSet_.currentFrameLidarPoseInWorldMap);
        PointCloudTransformation::TransformToWorld(lastSurfacePointCloudPtr_, lastSurfacePointCloudPtr_, poseSet_.currentFrameLidarPoseInWorldMap);
    }
}

bool ScanToScan::Match(){
    //提取特征点
    featureExtractPtr->PointCloudInput(newPointCloud_);
    featureExtractPtr->RunOnceExtractFeatures();
    //如果是第一帧
    if(isFristFrame_){
        isFristFrame_ = false;
        SaveNewFrame();
        return true;
    }
    //给定预测初值
    if(fixedCoordinateSystem_){
        registrationPtr_->SetPredictPose(poseSet_.predictNextFrameLidarPoseInWorldMap);
    }else{
        registrationPtr_->SetPredictPose(poseSet_.predictNextposeTransformation);
    }

    //进行相邻帧的匹配
    registrationPtr_->PointCloudInput(featureExtractPtr->GETCornerPoints(),
                                        lastCornerPointCloudPtr_,
                                        featureExtractPtr->GETSurfacePoints(),
                                        lastSurfacePointCloudPtr_);
    registrationPtr_->ScanMatch(2);

    UpdatePose();
    //保存当前帧用于下一帧匹配
    SaveNewFrame();
    
    return true;
}
bool ScanToScan::SetPredictPose(const Eigen::Isometry3d &predictPose){
    poseSet_.predictNextposeTransformation = predictPose;
    poseSet_.predictNextFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap * predictPose;
    return true;
}

bool ScanToScan::SetGnssData(const Eigen::Vector3d & gnss_data){
    poseSet_.currentFrameGnssData = gnss_data;
}

bool ScanToScan::SetInitPoseInWorld(const Eigen::Isometry3d &initPose){
    poseSet_.currentFrameLidarPoseInWorldMap = initPose;
    poseSet_.lastFrameLidarPoseInWorldMap = initPose;
    return true;
}
Eigen::Isometry3d ScanToScan::GetPoseTransformationFromLastToCurrent(){
    return poseSet_.poseTransformationFromLastToCurrent;
}

Eigen::Isometry3d ScanToScan::GetCurrentLidarPose(){
    return poseSet_.currentFrameLidarPoseInWorldMap;
}

Eigen::Isometry3d ScanToScan::GetCurrentLidarTruthPose(){
    Eigen::Isometry3d _real_pose;
    _real_pose = poseSet_.currentFrameLidarPoseInWorldMap;
    _real_pose.translation()[0] = poseSet_.currentFrameGnssData[0];
    _real_pose.translation()[1] = poseSet_.currentFrameGnssData[1];
    _real_pose.translation()[2] = poseSet_.currentFrameGnssData[2];

    return _real_pose;
}

}