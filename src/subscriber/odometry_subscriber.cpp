/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-19 20:42:22
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-08 16:16:21
 */
#include "subscriber/odometry_subscriber.hpp"

#include <geometry_msgs/Quaternion.h>

namespace loam_frame{

OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, int buff_size):nh_(nh){
    subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
}
//把信息保存到队列中，等待处理
void OdometrySubscriber::msg_callback(const nav_msgs::Odometry::ConstPtr & laser_odometry_ptr) {
    OdometryData _odometry_data;

    Eigen::Vector3d _translation;

    _odometry_data.time = laser_odometry_ptr->header.stamp.toSec();

    _odometry_data.pose.setIdentity();
    //注意不同的格式四元数顺序的问题
    _odometry_data.pose.rotate(Eigen::Quaterniond(laser_odometry_ptr->pose.pose.orientation.w,
                                                laser_odometry_ptr->pose.pose.orientation.x,
                                                laser_odometry_ptr->pose.pose.orientation.y,
                                                laser_odometry_ptr->pose.pose.orientation.z));
    _translation[0] = laser_odometry_ptr->pose.pose.position.x;
    _translation[1] = laser_odometry_ptr->pose.pose.position.y;
    _translation[2] = laser_odometry_ptr->pose.pose.position.z;
    _odometry_data.pose.pretranslate(_translation);

    new_laser_odometry_deque_.push_back(std::move(_odometry_data));
}

//从队列中把数据取出来
void OdometrySubscriber::ParseData(std::deque<OdometryData>& laser_odometry_buff) {
    if (new_laser_odometry_deque_.size() > 0) {
        laser_odometry_buff.insert(laser_odometry_buff.end(), new_laser_odometry_deque_.begin(), new_laser_odometry_deque_.end());
        new_laser_odometry_deque_.clear();
    }
}
}