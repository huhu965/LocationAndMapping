/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-29 12:12:52 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-30 19:47:42
 */

#ifndef LOAM_FRAME_MODULES_REGISTRATION_INTERFACE_HPP_
#define LOAM_FRAME_MODULES_REGISTRATION_INTERFACE_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sensor_data/point_cloud_data.hpp"

namespace loam_frame {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;
    
    virtual bool PointCloudInput(const PointCloudData::point_cloud_ptr sourceCornerCloud,
                                const PointCloudData::point_cloud_ptr targetCornerCloud,
                                const PointCloudData::point_cloud_ptr sourceSurfaceCloud,
                                const PointCloudData::point_cloud_ptr targetSurfaceCloud)=0;
    /*
    * @Description: 输入为相邻帧的位姿变换
    */
    virtual bool SetPredictPose(const Eigen::Isometry3d &predictPose)=0;

    virtual bool ScanMatch(int iterCount) = 0;

    virtual Eigen::Isometry3d GetMatchResult() = 0;
};
}

#endif  //LOAM_FRAME_MODULES_REGISTRATION_INTERFACE_HPP_