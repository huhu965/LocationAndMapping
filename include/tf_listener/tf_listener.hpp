/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:01:21
 */
#ifndef LOAM_FRAME_TF_LISTENER_HPP_
#define LOAM_FRAME_TF_LISTENER_HPP_

#include <string>

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace loam_frame {
class TFListener {
  public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;

    bool LookupData(Eigen::Matrix4f& transform_matrix);
  
  private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};
}

#endif