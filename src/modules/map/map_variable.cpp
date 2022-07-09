/*
 * @Author: Hu Ziwei 
 * @Date: 2021-10-11 12:57:53 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-07 15:54:11
 */

#include "modules/map/map_variable.hpp"

namespace loam_frame{
MapVariable::MapVariable(ros::NodeHandle& nh):nh_(nh){
    nh_.getParam("/loam_frame/mapping/cube_center_width_coordinate", cubeCenterWidthCoordinate_);
    nh_.getParam("/loam_frame/mapping/cube_center_height_coordinate", cubeCenterHeightCoordinate_);
    nh_.getParam("/loam_frame/mapping/cube_center_depth_coordinate", cubeCenterDepthCoordinate_);
    nh_.getParam("/loam_frame/mapping/cube_width", cubeWidth_);
    nh_.getParam("/loam_frame/mapping/cube_height", cubeHeight_);
    nh_.getParam("/loam_frame/mapping/cube_depth", cubeDepth_);

    nh_.getParam("/loam_frame/mapping/mapping_corner_leaf_size", mappingCornerLeafSize_);
    nh_.getParam("/loam_frame/mapping/mapping_surface_leaf_size", mappingSurfLeafSize_);

    nh_.getParam("/loam_frame/mapping/surrounding_keyframe_search_radius", surroundingKeyframeSearchRadius_);
    nh_.getParam("/loam_frame/mapping/surrounding_keyframe_leaf_size", surroundingKeyframeLeafSize_);

    nh_.getParam("/loam_frame/mapping/history_keyframe_search_radius", historyKeyFrameSearchRadius_);
    nh_.getParam("/loam_frame/mapping/history_keyframe_search_num", historyKeyFrameFearchNum_);
    

    subCubeNum_ = cubeWidth_ * cubeHeight_ * cubeDepth_;
    currentLaserCloudTime = 0;
    currentTranslationFromSubmapSet.clear();
    laserCornerPointsFromSubmapPtr.reset(new PointCloudData::point_cloud());
    laserSurfPointsFromSubmapPtr.reset(new PointCloudData::point_cloud());
    laserCornerPointsFromSubmapDSPtr.reset(new PointCloudData::point_cloud());
    laserSurfPointsFromSubmapDSPtr.reset(new PointCloudData::point_cloud());

    worldMapPtr.reset(new PointCloudData::point_cloud()); //map
    worldMapDSPtr.reset(new PointCloudData::point_cloud()); // map，降采样
    downSizeFilterWorldMap_.setLeafSize(mappingSurfLeafSize_,
                                        mappingSurfLeafSize_,
                                        mappingSurfLeafSize_); 

    translationCloudKeyFrames.reset(new PointCloudData::point_cloud());

    kdtreeSurroundingKeyPosesPtr_.reset(new pcl::KdTreeFLANN<PointCloudData::point>());
    kdtreeHistoryKeyPosesPtr_.reset(new pcl::KdTreeFLANN<PointCloudData::point>());

    downSizeFilterSurroundingKeyPoses_.setLeafSize(surroundingKeyframeLeafSize_, 
                                                    surroundingKeyframeLeafSize_, 
                                                    surroundingKeyframeLeafSize_);

    downSizeFilterCorner_.setLeafSize(mappingCornerLeafSize_,
                                        mappingCornerLeafSize_,
                                        mappingCornerLeafSize_);
                                        
    downSizeFilterSurface_.setLeafSize(mappingSurfLeafSize_,
                                        mappingSurfLeafSize_,
                                        mappingSurfLeafSize_);                                       
    cornerCloudKeyFrames.clear();
    surfCloudKeyFrames.clear();
    fullCloudKeyFrames.clear();
    poseCloudKeyFrames.clear();

    loopSubmapIndex.clear();
    hasLoop = false;
    laserCornerPointsFromLoopSubmapPtr.reset(new PointCloudData::point_cloud());
    laserSurfPointsFromLoopSubmapPtr.reset(new PointCloudData::point_cloud());
    laserCornerPointsFromLoopSubmapDSPtr.reset(new PointCloudData::point_cloud());
    laserSurfPointsFromLoopSubmapDSPtr.reset(new PointCloudData::point_cloud());

    // cornerLastBuf.clear();
    // surfLastBuf.clear();
    // fullResourceBuf.clear();
    // odometryBuf.clear();
}

} // namespace loam_frame
