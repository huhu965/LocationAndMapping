/*
 * @Author: Hu Ziwei 
 * @Description:  一帧点云的持续时长
 * @Date: 2021-12-30 18:50:12 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-30 18:53:58
 */

#include "subscriber/duration_time_subscriber.hpp"


namespace loam_frame {
DurationTimeSubscriber::DurationTimeSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &DurationTimeSubscriber::msg_callback, this);
}

void DurationTimeSubscriber::msg_callback(const std_msgs::Duration &time) {
    new_duration_data_deque_.push_back(time);
}

void DurationTimeSubscriber::ParseData(std::deque<std_msgs::Duration>& deque_duration_data) {
    if (new_duration_data_deque_.size() > 0) {
        deque_duration_data.insert(deque_duration_data.end(), new_duration_data_deque_.begin(), new_duration_data_deque_.end());
        new_duration_data_deque_.clear();
    }
}
}