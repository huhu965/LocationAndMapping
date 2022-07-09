/*
 * @Author: Hu Ziwei 
 * @Description:  速度数据
 * @Date: 2021-12-30 16:15:09 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-30 16:18:50
 */

#ifndef LOAM_FRAME_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LOAM_FRAME_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace loam_frame{
class VelocityData {
  public:
    struct LinearVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
  
  public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
};
}

#endif