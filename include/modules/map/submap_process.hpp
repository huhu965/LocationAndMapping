/*
 * @Author: Hu Ziwei 
 * @Date: 2021-10-12 15:29:15 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-07 15:54:49
 */

#ifndef LOAM_FRAME_MODULES_MAP_SUBMAP_PROCESS_HPP_
#define LOAM_FRAME_MODULES_MAP_SUBMAP_PROCESS_HPP_

#include <ros/ros.h>

#include "modules/map/map_variable.hpp"

namespace loam_frame{
/*
 * @Description: map 操作类
*/
class SubmapProcess:virtual public MapVariable{
public:
    SubmapProcess(ros::NodeHandle& nh);
    ~SubmapProcess() = default;
    /*
    * @Description:提取子地图内关键帧的平移位姿，同时也会进行回环检测
    * 提取当前关键帧一定距离内的关键帧，并返回所有相邻关键帧的平移位姿
    */
    PointCloudData::point_cloud ExtractSurroundKeyFramesPose(const PointCloudData::point &_currentKeyFrameTranslation, double _currentTime);
    /*
    * @Description:提取子地图内关键帧的平移位姿 ，同时也会进行回环检测
    * 提取当前关键帧一定距离内的关键帧，并返回所有相邻关键帧的平移位姿
    */
    PointCloudData::point_cloud ExtractSurroundKeyFramesPose(double _currentTime);
    /*
    * @Description:提取子地图的特征点云
    * 相邻关键帧的平移位姿,提取关键帧位姿获取到的点云
    * 放在laserCloudCornerPointFromSubmapDSPtr和laserCloudSurfPointFromSubmapDSPtr中
    */
    void ExtractSurroundCloud(const PointCloudData::point_cloud &_surroundKeyFramePoses);

    void ExtractLoopSurroundKeyFramesPose(const PointCloudData::point &_currentKeyFrameTranslation, double _currentTime);
    /*
    * @Description:回环检测到的
    * 相邻关键帧的平移位姿,提取关键帧位姿获取到的点云
    * 放在laserCloudCornerPointFromSubmapDSPtr和laserCloudSurfPointFromSubmapDSPtr中
    */
    void ExtractLoopSurroundCloud(std::vector<int> &_surroundLoopFramePoses);
    PointCloudData::point_cloud TransformToWorld(const PointCloudData::point_cloud_ptr cloud_in, Eigen::Isometry3d &_pose);

    /*
    * @Description:清楚子地图中暂存的数据，用于优化后的子地图更新，和平时子地图过大时的清理
    */
    void ClearSurroundCloud();
};
}

#endif