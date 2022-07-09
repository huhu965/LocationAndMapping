/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-30 18:45:26
 */
#ifndef LOAM_FRAME_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LOAM_FRAME_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "sensor_data/point_cloud_data.hpp"
namespace loam_frame{

class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<PointCloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PointCloudData> new_cloud_data_deque_;
};

}
#endif //LOAM_FRAME_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_