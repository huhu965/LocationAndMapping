/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-12-30 16:19:58 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-30 16:28:11
 */

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

#include "sensor_data/velocity_data.hpp"

namespace loam_frame{
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_; 
};

}