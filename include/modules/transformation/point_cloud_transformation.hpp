/*
 * @Author: Hu Ziwei 
 * @Description:  点云位姿投影变换
 * @Date: 2021-11-29 19:09:58 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-29 19:21:16
 */

#ifndef LOAM_FRAME_MODULES_TRANSFORMATION_POINT_CLOUD_TRANSFORMATION_HPP_
#define LOAM_FRAME_MODULES_TRANSFORMATION_POINT_CLOUD_TRANSFORMATION_HPP_

#include <eigen3/Eigen/Dense>
#include "sensor_data/point_cloud_data.hpp"

namespace loam_frame{
class PointCloudTransformation{
public:
    PointCloudTransformation() = default;
    ~PointCloudTransformation() = default;
    /*
     * @Description:将点在当前坐标系下的坐标转到上一帧坐标系下坐标
     * stepPose为上一帧到当前帧的位姿变换
    */
    static void TransformToLast(const PointCloudData::point &point_in, PointCloudData::point &point_out, const Eigen::Isometry3d &stepPose);
    static void TransformToWorld(const PointCloudData::point_cloud_ptr cloudIn, PointCloudData::point_cloud_ptr cloudOut, const Eigen::Isometry3d &_pose);
    static PointCloudData::point_cloud TransformToWorld(const PointCloudData::point_cloud_ptr cloudIn, const Eigen::Isometry3d &_pose);
};

}

#endif