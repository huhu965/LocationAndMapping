/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-23 11:01:38
 */
#ifndef LOAM_FRAME_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define LOAM_FRAME_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "sensor_data/odometry_data.hpp"

namespace loam_frame{

class OdometrySubscriber {
  public:
    OdometrySubscriber(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      int buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<OdometryData>& deque_laser_odometry);

  private:
    void msg_callback(const nav_msgs::Odometry::ConstPtr & laser_odometry_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<OdometryData> new_laser_odometry_deque_;
};

}
#endif //LOAM_FRAME_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_