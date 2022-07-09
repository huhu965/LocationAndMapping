/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-14 13:03:24
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-07 17:27:15
 */
#ifndef LOAM_FRAME_MODULES_MAP_MAP_VARIABLE_HPP_
#define LOAM_FRAME_MODULES_MAP_MAP_VARIABLE_HPP_

#include <vector>
#include <deque>
#include <algorithm>
#include <map>
#include <set> 

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <nav_msgs/Odometry.h>


#include "sensor_data/point_cloud_data.hpp"
#include "data_struct/pose_struct.hpp"

namespace loam_frame{

/*
 * @Description: 前端里程计抽去关键帧送到mapping模块，用来精确匹配位姿
*/
class MapVariable{
public:
    MapVariable();
    MapVariable(ros::NodeHandle& nh);
    ~MapVariable() = default;
public:
    double currentLaserCloudTime;
    PointCloudData::point_cloud_ptr worldMapPtr; //map的平面点集合
    PointCloudData::point_cloud_ptr worldMapDSPtr; // 局部map的角点集合，降采样
    pcl::VoxelGrid<PointCloudData::point> downSizeFilterWorldMap_;

    std::vector<PointCloudData::point_cloud_ptr> cornerCloudKeyFrames; // 历史所有关键帧的角点集合（降采样）
    std::vector<PointCloudData::point_cloud_ptr> surfCloudKeyFrames;  // 历史所有关键帧的平面点集合（降采样）
    std::vector<PointCloudData::point_cloud_ptr> fullCloudKeyFrames;  // 历史所有关键帧的集合
    std::vector<PoseParam> poseCloudKeyFrames; //关键帧位姿
    PointCloudData::point_cloud_ptr translationCloudKeyFrames; // 历史关键帧平移量（位置）
    //历史所有帧的角和面特征存储，1是角特征，2是面特征
    // std::map<int, std::pair<PointCloudData::point_cloud, PointCloudData::point_cloud>> laserCloudMapContainer;

    std::set<int> currentTranslationFromSubmapSet;
    PointCloudData::point_cloud_ptr laserCornerPointsFromSubmapPtr; // 局部map的角点集合
    PointCloudData::point_cloud_ptr laserSurfPointsFromSubmapPtr; // 局部map的平面点集合
    PointCloudData::point_cloud_ptr laserCornerPointsFromSubmapDSPtr; // 局部map的角点集合，降采样
    PointCloudData::point_cloud_ptr laserSurfPointsFromSubmapDSPtr; // 局部map的平面点集合，降采样

    std::vector<int> loopSubmapIndex;
    bool hasLoop;
    PointCloudData::point_cloud_ptr laserCornerPointsFromLoopSubmapPtr; // loop回环局部map的角点集合
    PointCloudData::point_cloud_ptr laserSurfPointsFromLoopSubmapPtr; // loop回环局部map的平面点集合
    PointCloudData::point_cloud_ptr laserCornerPointsFromLoopSubmapDSPtr; // loop回环局部map的角点集合，降采样
    PointCloudData::point_cloud_ptr laserSurfPointsFromLoopSubmapDSPtr; // loop回环局部map的平面点集合，降采样

protected:
    ros::NodeHandle nh_;

    pcl::KdTreeFLANN<PointCloudData::point>::Ptr kdtreeSurroundingKeyPosesPtr_;
    pcl::KdTreeFLANN<PointCloudData::point>::Ptr kdtreeHistoryKeyPosesPtr_;

    pcl::VoxelGrid<PointCloudData::point> downSizeFilterCorner_; //降采样
    pcl::VoxelGrid<PointCloudData::point> downSizeFilterSurface_;   //降采样
    pcl::VoxelGrid<PointCloudData::point> downSizeFilterSurroundingKeyPoses_;

    // 立方体cube的中心点坐标
    int cubeCenterWidthCoordinate_;
    int cubeCenterHeightCoordinate_;
    int cubeCenterDepthCoordinate_;
    //立方体的长宽高轴长度
    int cubeWidth_;
    int cubeHeight_;
    int cubeDepth_;
    //立方体cube中的子cube个数
    int subCubeNum_;

    double surroundingKeyframeAddingDistThreshold_; //添加关键帧距离阈值
    double surroundingKeyframeAddingAngleThreshold_; //添加关键帧角度阈值
    double surroundingKeyframeLeafSize_; 
    double surroundingKeyframeSearchRadius_; //局部地图的查找半径
    //体素滤波的分辨率大小
    double mappingCornerLeafSize_;
    double mappingSurfLeafSize_;
    
    double historyKeyFrameSearchRadius_; //回环时提取地图的范围
    int historyKeyFrameFearchNum_; //回环子地图的最大帧数
};

}
#endif      //LOAM_FRAME_MODULES_MAP_MAPPING_VARIABLE_HPP_