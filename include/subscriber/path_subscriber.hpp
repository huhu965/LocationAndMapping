/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-23 11:01:38
 */
#ifndef LOAM_FRAME_SUBSCRIBER_PATH_SUBSCRIBER_HPP_
#define LOAM_FRAME_SUBSCRIBER_PATH_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_data/odometry_data.hpp"
#include "data_struct/pose_struct.hpp"

namespace loam_frame{

class PathSubscriber {
  public:
    PathSubscriber(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      int buff_size);
    PathSubscriber() = default;
    void ParseData(nav_msgs::Path &path);

  private:
    void msg_callback(const nav_msgs::Path &path);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    nav_msgs::Path new_path_;
};

}
#endif //LOAM_FRAME_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_