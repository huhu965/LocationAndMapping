/*
 * @Author: Hu Ziwei 
 * @Description: 地图操作
 * @Date: 2021-11-07 15:24:12 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-07 17:12:53
 */
#ifndef LOAM_FRAME_MODULES_MAP_MAP_PROCESS_HPP_
#define LOAM_FRAME_MODULES_MAP_MAP_PROCESS_HPP_

#include <ros/ros.h>
#include "modules/map/map_variable.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"

namespace loam_frame{
/*
 * @Description: map 操作类
*/
class MapProcess:virtual public MapVariable{
public:
    MapProcess(ros::NodeHandle& nh);
    ~MapProcess() = default;
    void AddKeyFrame(PointCloudData::point_cloud_ptr _fullCloudPtr, 
                    PointCloudData::point_cloud_ptr _cornerCloudPtr, 
                    PointCloudData::point_cloud_ptr _surfaceCloudPtr);
 
    void AddKeyFrame(PointCloudData::point_cloud_ptr _fullCloudPtr);

    void AddKeyPose(PoseParam & _poseParam);

    void CorrectPoses(std::deque<Eigen::Isometry3d> &_optimizedPoses);

    void GenerateMap();
    void AddKeyframToMap();
    PointCloudData::point_cloud_ptr GetWorldMap();
};
}

#endif