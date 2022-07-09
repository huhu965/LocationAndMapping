/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:18:47
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 00:57:38
 */

#include "subscriber/cloud_subscriber.hpp"

#include <iomanip>

namespace loam_frame{

 CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}
//把信息保存到队列中，等待处理
void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    PointCloudData _cloud_data;
    _cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    
    // pcl::fromROSMsg(*cloud_msg_ptr, *_cloud_data.cloud_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg_ptr, *_cloud);
    pcl::copyPointCloud(*_cloud, *(_cloud_data.cloud_ptr));//xyz 转为XYZI

    new_cloud_data_deque_.push_back(_cloud_data);

}

//从队列中把数据取出来
void CloudSubscriber::ParseData(std::deque<PointCloudData>& cloud_data_buff) {
    if (new_cloud_data_deque_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_deque_.begin(), new_cloud_data_deque_.end());
        new_cloud_data_deque_.clear();
    }
}
}