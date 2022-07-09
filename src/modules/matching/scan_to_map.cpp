/*
 * @Author: Hu Ziwei 
 * @Description:  scan to map匹配
 * @Date: 2021-11-27 16:21:16 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-10 16:43:09
 */

#include "glog/logging.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "modules/matching/scan_to_map.hpp"
#include "modules/registration/ICP/aloam_registration.hpp"
#include "modules/registration/ICP/fast_registration.hpp"
#include "modules/feature_extract/go_feature_extract.hpp"
#include "modules/feature_extract/normal_feature_extract.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"
#include "modules/cloud_filter/voxel_filter.hpp"

#include "tools_/tic_toc.hpp"

namespace loam_frame{
    
ScanToMap::ScanToMap(const YAML::Node& configNode)
    :newPointCloud_(new PointCloudData::point_cloud()),
    currentFrameCornerPointsPtr_(new PointCloudData::point_cloud()),
    currentFrameSurfacePointsPtr_(new PointCloudData::point_cloud()),
    cornerPointsFromSubmapPtr_(new PointCloudData::point_cloud()),
    surfPointsFromSubmapPtr_(new PointCloudData::point_cloud()),
    translationKeyFramePtr_(new PointCloudData::point_cloud()){
    
    keyframePose_.clear(); //关键帧位姿
    cornerPointsBuf_.clear(); //角点
    surfacePointsBuf_.clear(); //平面点
    isFristFrame_ = true;
    InitWithConfig(configNode);
}

bool ScanToMap::PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr){
    newPointCloud_ = pointCloudPtr;
    return true;
}

bool ScanToMap::InitWithConfig(const YAML::Node& configNode){
    KeyframeAddDistanceThreshold_ = configNode["key_frame_distance_threshold"].as<double>();
    KeyframeAddAngleThreshold_ = configNode["key_frame_theta_threshold"].as<double>();
    submapFrameNumber_ = configNode["submap_frame_num"].as<double>();
    InitFeatureExtract(configNode);
    InitRegistration(configNode);
    InitFilter(configNode["filter1"], downSizeFilterPtr_);
    InitFilter(configNode["filter2"], downSizeFilterSurroundingKeyPoses_);
    return true;
}

bool ScanToMap::InitFeatureExtract(const YAML::Node& configNode){
    std::string _featureExtractMethod = configNode["feature_extract_method"].as<std::string>();
    std::cout << "ScanToMap特征提取方式为：" << _featureExtractMethod << std::endl;

    if (_featureExtractMethod == "ground_optimize") {
        featureExtractPtr = std::make_shared<GoFeatureExtract>(configNode[_featureExtractMethod]);
    }else if (_featureExtractMethod == "normal") {
        featureExtractPtr = std::make_shared<NormalFeatureExtract>(configNode[_featureExtractMethod]);
    }else {
        LOG(ERROR) << "没找到与 " << _featureExtractMethod << " 相对应的特征提取方式!";
        return false;
    }
    return true;
}

bool ScanToMap::InitRegistration(const YAML::Node& configNode){
    std::string _registrationMethod = configNode["registration_method"].as<std::string>();
    std::cout << "ScanToMap点云配准方式为：" << _registrationMethod << std::endl;

    if(_registrationMethod == "fast"){
        registrationPtr_ = std::make_shared<FastRegistration>(configNode[_registrationMethod]);
    }else {
        LOG(ERROR) << "没找到与 " << _registrationMethod << " 相对应的点云匹配方式!";
        return false;
    }
    return true;
}

bool ScanToMap::InitFilter(const YAML::Node& config_node, std::shared_ptr<CloudFilterInterface> &filterPtr){
    std::string filter_method = config_node["method"].as<std::string>();
    std::cout << "ScanToMap 生成子地图选择的滤波器方式为：" << filter_method << std::endl;
    if(filter_method == "voxel_filter"){
        filterPtr = std::make_shared<VoxelFilter>(config_node[filter_method]);
    }else{
        std::cout<< "没找到与 " << filter_method << " 相对应的滤波方式!"<< std::endl;
        LOG(ERROR) << "没找到与 " << filter_method << " 相对应的滤波方式!";
        return false;
    }
    
    return true;
}

bool ScanToMap::IsNewKeyFrame(){
    Eigen::Affine3d _temp_pose(poseSet_.keyPoseTransformationFromLastToCurrent.matrix());
    double x, y, z, roll, pitch, yaw;
    //将旋转矩阵分解为欧拉角和平移，角度用弧度表示
    pcl::getTranslationAndEulerAngles(_temp_pose, x, y, z, roll, pitch, yaw);
    //如果超出阈值就认为是一个新关键帧
    if (abs(roll) > KeyframeAddAngleThreshold_ || 
        abs(pitch) > KeyframeAddAngleThreshold_ || 
        abs(yaw) > KeyframeAddAngleThreshold_ || 
        sqrt(x*x + y*y + z*z) > KeyframeAddDistanceThreshold_){
        return true;
    }
    return false;
}
 
void ScanToMap::AddKeyFrame(){
    //更新上一关键帧位姿
    poseSet_.lastKeyFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap;
    // //保存关键帧点云
    PointCloudData::point_cloud_ptr _thisCornerKeyFramePtr(new PointCloudData::point_cloud());
    PointCloudData::point_cloud_ptr _thisSurfKeyFramePtr(new PointCloudData::point_cloud());
    PointCloudTransformation::TransformToWorld(featureExtractPtr->GETLessSharpCornerPoints(), _thisCornerKeyFramePtr, poseSet_.lastKeyFrameLidarPoseInWorldMap);
    PointCloudTransformation::TransformToWorld(featureExtractPtr->GETLessFlatSurfacePoints(), _thisSurfKeyFramePtr, poseSet_.lastKeyFrameLidarPoseInWorldMap);
    //保存新关键和他的位姿
    cornerPointsBuf_.push_back(_thisCornerKeyFramePtr);
    surfacePointsBuf_.push_back(_thisSurfKeyFramePtr);
    keyframePose_.push_back(poseSet_.lastKeyFrameLidarPoseInWorldMap);
    //移除旧关键帧
    while(keyframePose_.size() > submapFrameNumber_){
        cornerPointsBuf_.pop_front();
        surfacePointsBuf_.pop_front();
        keyframePose_.pop_front();
    }
}

void ScanToMap::GenerateNewSubmap(){
    cornerPointsFromSubmapPtr_->clear(); // 局部map的角点集合
    surfPointsFromSubmapPtr_->clear(); // 局部map的平面点集合
    PointCloudData::point_cloud_ptr _downsizeCloudPtr(new PointCloudData::point_cloud());//降采样

    for (int i = 0; i < (int)cornerPointsBuf_.size(); ++i)
    {
        // int _index = translationKeyFramePtr_->points[i].intensity;
        *cornerPointsFromSubmapPtr_ += *cornerPointsBuf_[i];
        *surfPointsFromSubmapPtr_ += *surfacePointsBuf_[i];
    }
    _downsizeCloudPtr->clear();
    downSizeFilterPtr_->Filter(cornerPointsFromSubmapPtr_, _downsizeCloudPtr);
    *cornerPointsFromSubmapPtr_ = *_downsizeCloudPtr;
    _downsizeCloudPtr->clear();
    downSizeFilterPtr_->Filter(surfPointsFromSubmapPtr_, _downsizeCloudPtr);
    *surfPointsFromSubmapPtr_ = *_downsizeCloudPtr;
}

void ScanToMap::UpdatePose(){
    //返回的结果是世界坐标系的位姿
    poseSet_.currentFrameLidarPoseInWorldMap = registrationPtr_->GetMatchResult();
    // std::cout<<poseSet_.currentFrameLidarPoseInWorldMap.matrix()<<std::endl;
    //相邻帧的运动估计值
    poseSet_.poseTransformationFromLastToCurrent = poseSet_.lastFrameLidarPoseInWorldMap.inverse() *  poseSet_.currentFrameLidarPoseInWorldMap;
    //上一关键帧到当前帧的运动估计值
    poseSet_.keyPoseTransformationFromLastToCurrent = poseSet_.lastKeyFrameLidarPoseInWorldMap.inverse() * poseSet_.currentFrameLidarPoseInWorldMap;
    //用当前帧位姿作为下一帧的上一位姿
    poseSet_.lastFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap;
    //预测当前到下一帧的运动，如果给的话，就会覆盖这个值
    poseSet_.predictNextposeTransformation = poseSet_.poseTransformationFromLastToCurrent;
    poseSet_.predictNextFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap * poseSet_.poseTransformationFromLastToCurrent;
}

bool ScanToMap::Match(){
    //提取特征点
    featureExtractPtr->PointCloudInput(newPointCloud_);
    TicToc _time;
    _time.tic();
    featureExtractPtr->RunOnceExtractFeatures();
    // std::cout<<"特征提取时间:"<<_time.toc()<<std::endl;
    //如果是第一帧
    if(isFristFrame_){
        isFristFrame_ = false;
        AddKeyFrame();
        GenerateNewSubmap();
        poseSet_.predictNextFrameLidarPoseInWorldMap.setIdentity();
        return true;
    }

    //给定在世界坐标系下的预测值
    registrationPtr_->SetPredictPose(poseSet_.predictNextFrameLidarPoseInWorldMap);

    //进行相邻帧的匹配
    registrationPtr_->PointCloudInput(featureExtractPtr->GETCornerPoints(),
                                        cornerPointsFromSubmapPtr_,
                                        featureExtractPtr->GETSurfacePoints(),
                                        surfPointsFromSubmapPtr_);
    _time.tic();
    registrationPtr_->ScanMatch(1);
    // std::cout<<"位姿匹配时间:"<<_time.toc()<<std::endl;

    UpdatePose();//更新估计结果
    //添加关键帧，更新子地图
    if(IsNewKeyFrame()){
        AddKeyFrame();
        _time.tic();
        GenerateNewSubmap();
        // std::cout<<"子地图生成时间:"<<_time.toc()<<std::endl;
    }

    return true;
}

bool ScanToMap::SetPredictPose(const Eigen::Isometry3d &predictPose){
    poseSet_.predictNextposeTransformation = predictPose;
    poseSet_.predictNextFrameLidarPoseInWorldMap = poseSet_.currentFrameLidarPoseInWorldMap * predictPose;
    return true;
}

bool ScanToMap::SetGnssData(const Eigen::Vector3d & gnss_data){

    poseSet_.predictNextFrameLidarPoseInWorldMap.translation()[0] = poseSet_.lastFrameLidarPoseInWorldMap.translation()[0] + gnss_data[0] - poseSet_.currentFrameGnssData[0];
    poseSet_.predictNextFrameLidarPoseInWorldMap.translation()[1] = poseSet_.lastFrameLidarPoseInWorldMap.translation()[1] + gnss_data[1] - poseSet_.currentFrameGnssData[1];
    poseSet_.predictNextFrameLidarPoseInWorldMap.translation()[2] = poseSet_.lastFrameLidarPoseInWorldMap.translation()[2] + gnss_data[2] - poseSet_.currentFrameGnssData[2];

    poseSet_.currentFrameGnssData = gnss_data;
}

bool ScanToMap::SetInitPoseInWorld(const Eigen::Isometry3d &initPose){
    poseSet_.currentFrameLidarPoseInWorldMap = initPose;
    poseSet_.lastFrameLidarPoseInWorldMap = initPose;
    std::cout<<"初始位姿"<<std::endl<<poseSet_.lastFrameLidarPoseInWorldMap.matrix()<<std::endl;
    return true;
}

Eigen::Isometry3d ScanToMap::GetPoseTransformationFromLastToCurrent(){
    return poseSet_.poseTransformationFromLastToCurrent;
}

Eigen::Isometry3d ScanToMap::GetCurrentLidarPose(){
    return poseSet_.currentFrameLidarPoseInWorldMap;
}

Eigen::Isometry3d ScanToMap::GetCurrentLidarTruthPose(){
    Eigen::Isometry3d _real_pose;
    _real_pose = poseSet_.currentFrameLidarPoseInWorldMap;
    _real_pose.translation()[0] = poseSet_.currentFrameGnssData[0];
    _real_pose.translation()[1] = poseSet_.currentFrameGnssData[1];
    _real_pose.translation()[2] = poseSet_.currentFrameGnssData[2];

    return _real_pose;
}

PointCloudData::point_cloud_ptr ScanToMap::GetSubmapCornerPoint(){
    return cornerPointsFromSubmapPtr_;
}
PointCloudData::point_cloud_ptr ScanToMap::GetSubmapSurfaceoint(){
    return surfPointsFromSubmapPtr_;
}

}