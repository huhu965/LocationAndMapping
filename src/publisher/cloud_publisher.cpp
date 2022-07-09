/* 
 * @Author: Hu Ziwei
 * @Description: 发布点云
 * @Date: 2021-09-02 16:04:22
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-09-08 15:18:57
 */

#include "publisher/cloud_publisher.hpp"

namespace loam_frame{
    
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size,
                               std::string frame_id)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(PointCloudData::point_cloud_ptr cloud_ptr_input) {
    sensor_msgs::PointCloud2Ptr _cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *_cloud_ptr_output);
    _cloud_ptr_output->header.stamp = ros::Time::now();
    _cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*_cloud_ptr_output);
}
void CloudPublisher::Publish(PointCloudData::point_cloud_ptr cloud_ptr_input,ros::Time stamp){
    sensor_msgs::PointCloud2Ptr _cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *_cloud_ptr_output);
    _cloud_ptr_output->header.stamp = stamp;
    _cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*_cloud_ptr_output);
}

}