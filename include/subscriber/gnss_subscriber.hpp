/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-23 11:00:20 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-23 11:03:03
 */
#ifndef LOAM_FRAME_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LOAM_FRAME_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_data/gnss_data.hpp"

namespace loam_frame {
class GNSSSubscriber {
  public:
    GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParseData(std::deque<GNSSData>& deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;
};
}
#endif //LOAM_FRAME_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_