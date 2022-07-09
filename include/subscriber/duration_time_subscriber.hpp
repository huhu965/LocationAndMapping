/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-30 18:54:07
 */
#ifndef LOAM_FRAME_SUBSCRIBER_DURATION_TIME_SUBSCRIBER_HPP_
#define LOAM_FRAME_SUBSCRIBER_DURATION_TIME_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <std_msgs/Duration.h>

namespace loam_frame{
class DurationTimeSubscriber {
  public:
    DurationTimeSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    DurationTimeSubscriber() = default;
    void ParseData(std::deque<std_msgs::Duration>& deque_duration_data);

  private:
    void msg_callback(const std_msgs::Duration &time);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<std_msgs::Duration> new_duration_data_deque_;
};

}
#endif //LOAM_FRAME_SUBSCRIBER_DURATION_TIME_SUBSCRIBER_HPP_