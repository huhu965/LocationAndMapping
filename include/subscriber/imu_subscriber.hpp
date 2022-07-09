/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-22 16:31:14
 */
#ifndef LOAM_FRAME_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LOAM_FRAME_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "sensor_data/imu_data.hpp"
namespace loam_frame{

class IMUSubscriber {
  public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUData>& deque_data);

  private:
    void msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> new_imu_data_deque_;
};

}
#endif //LOAM_FRAME_SUBSCRIBER_IMU_SUBSCRIBER_HPP_