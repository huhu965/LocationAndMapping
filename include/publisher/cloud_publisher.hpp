/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-11 10:35:57
 */
#ifndef LOAM_FRAME_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LOAM_FRAME_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


#include "sensor_data/point_cloud_data.hpp"
namespace loam_frame{

class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    CloudPublisher() = default;
    ~CloudPublisher() = default;
    void Publish(PointCloudData::point_cloud_ptr cloud_ptr_input);
    void Publish(PointCloudData::point_cloud_ptr cloud_ptr_input,ros::Time stamp);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
}
#endif //LOAM_FRAME_PUBLISHER_CLOUD_PUBLISHER_HPP_
