/*
 * @Author: Hu Ziwei 
 * @Description:  imu数据订阅
 * @Date: 2021-12-22 16:16:29 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-22 16:36:49
 */

#include "subscriber/imu_subscriber.hpp"

namespace loam_frame {
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr) {
    IMUData _imu_data;

    _imu_data.time = imu_msg_ptr->header.stamp.toSec();

    _imu_data.orientation.x() = imu_msg_ptr->orientation.x;
    _imu_data.orientation.y() = imu_msg_ptr->orientation.y;
    _imu_data.orientation.z() = imu_msg_ptr->orientation.z;
    _imu_data.orientation.w() = imu_msg_ptr->orientation.w;
    _imu_data.orientation.normalize();//四元数归一化

    _imu_data.linear_acceleration.x() = imu_msg_ptr->linear_acceleration.x;
    _imu_data.linear_acceleration.y() = imu_msg_ptr->linear_acceleration.y;
    _imu_data.linear_acceleration.z() = imu_msg_ptr->linear_acceleration.z;

    _imu_data.angular_velocity.x() = imu_msg_ptr->angular_velocity.x;
    _imu_data.angular_velocity.y() = imu_msg_ptr->angular_velocity.y;
    _imu_data.angular_velocity.z() = imu_msg_ptr->angular_velocity.z;

    new_imu_data_deque_.push_back(_imu_data);
}

void IMUSubscriber::ParseData(std::deque<IMUData>& deque_data) {
    if (new_imu_data_deque_.size() > 0) {
        deque_data.insert(deque_data.end(), new_imu_data_deque_.begin(), new_imu_data_deque_.end());
        new_imu_data_deque_.clear();
    }
}
}