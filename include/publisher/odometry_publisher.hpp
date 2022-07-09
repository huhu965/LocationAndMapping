/* 
 * @Author: Hu Ziwei
 * @Description: 雷达里程计发布
 * @Date: 2021-09-09 10:37:29
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-08 16:04:48
 */
#ifndef LOAM_FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LOAM_FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "sensor_data/odometry_data.hpp"

namespace loam_frame {
class OdometryPublisher {
  public:
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;
    ~OdometryPublisher() = default;

    void Publish(const OdometryData::pose_type& transform_matrix);
    void Publish(const OdometryData::pose_type& transform_matrix, ros::Time stamp);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
};
}
#endif  //LOAM_FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_