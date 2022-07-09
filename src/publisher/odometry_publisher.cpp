/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-09 10:44:22
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-11 10:38:59
 */
#include "publisher/odometry_publisher.hpp"

namespace loam_frame {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
                                     :nh_(nh){

    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const OdometryData::pose_type& transform_matrix) {
    odometry_.header.stamp = ros::Time::now();

    //set the position
    odometry_.pose.pose.position.x = transform_matrix.translation()[0];
    odometry_.pose.pose.position.y = transform_matrix.translation()[1];
    odometry_.pose.pose.position.z = transform_matrix.translation()[2];

    Eigen::Quaterniond q;
    q = transform_matrix.rotation(); //转为四元数
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
}

void OdometryPublisher::Publish(const OdometryData::pose_type& transform_matrix, ros::Time stamp) {
    odometry_.header.stamp = stamp;

    //set the position
    odometry_.pose.pose.position.x = transform_matrix.translation()[0];
    odometry_.pose.pose.position.y = transform_matrix.translation()[1];
    odometry_.pose.pose.position.z = transform_matrix.translation()[2];

    Eigen::Quaterniond q;
    q = transform_matrix.rotation(); //转为四元数
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
}

}