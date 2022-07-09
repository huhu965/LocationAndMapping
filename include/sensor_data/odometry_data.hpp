/*
 * @Author: Hu Ziwei 
 * @Description:里程计数据格式
 * @Date: 2021-11-08 15:53:22 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-08 16:16:39
 */
#ifndef LOAM_FRAME_SENSOR_DATA_ODOMETRY_DATA_HPP_
#define LOAM_FRAME_SENSOR_DATA_ODOMETRY_DATA_HPP_
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace loam_frame {
class OdometryData {
  public:
    using pose_type = Eigen::Isometry3d;

  public:
    OdometryData(){
        pose.setIdentity();
    }

  public:
    double time = 0.0;
    Eigen::Isometry3d pose;
};
}

#endif //LOAM_FRAME_SENSOR_DATA_POINT_CLOUD_DATA_HPP_