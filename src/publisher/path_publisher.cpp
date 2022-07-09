/*
 * @Author: Hu Ziwei 
 * @Description:  路径发布
 * @Date: 2021-11-11 10:34:39 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-14 22:46:23
 */

#include "publisher/path_publisher.hpp"

namespace loam_frame{
PathPublisher::PathPublisher(ros::NodeHandle& nh,
                std::string topic_name,
                std::string base_frame_id,
                size_t buff_size)
                :nh_(nh) {
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    path_.header.frame_id = base_frame_id;
}

void PathPublisher::Publish(const OdometryData::pose_type& transform_matrix){

    geometry_msgs::PoseStamped _pose;
    _pose.header.stamp = ros::Time::now();
    _pose.header.frame_id = path_.header.frame_id;
    //set the position
    _pose.pose.position.x = transform_matrix.translation()[0];
    _pose.pose.position.y = transform_matrix.translation()[1];
    _pose.pose.position.z = transform_matrix.translation()[2];
    Eigen::Quaterniond _q;
    _q = transform_matrix.rotation(); //转为四元数
    _pose.pose.orientation.x = _q.x();
    _pose.pose.orientation.y = _q.y();
    _pose.pose.orientation.z = _q.z();
    _pose.pose.orientation.w = _q.w();

    path_.header.stamp = ros::Time::now();
    path_.poses.push_back(_pose);
    publisher_.publish(path_);
}

void PathPublisher::Publish(const OdometryData::pose_type& transform_matrix, ros::Time stamp){

    geometry_msgs::PoseStamped _pose;
    _pose.header.stamp = stamp;
    _pose.header.frame_id = path_.header.frame_id;
    //set the position
    _pose.pose.position.x = transform_matrix.translation()[0];
    _pose.pose.position.y = transform_matrix.translation()[1];
    _pose.pose.position.z = transform_matrix.translation()[2];
    Eigen::Quaterniond _q;
    _q = transform_matrix.rotation(); //转为四元数
    _pose.pose.orientation.x = _q.x();
    _pose.pose.orientation.y = _q.y();
    _pose.pose.orientation.z = _q.z();
    _pose.pose.orientation.w = _q.w();

    path_.header.stamp = stamp;
    path_.poses.push_back(_pose);
    
    publisher_.publish(path_);
}

void PathPublisher::UpdatePath(std::deque<Eigen::Isometry3d> &_poseCloudKeyFrames){
    int _num = _poseCloudKeyFrames.size() < path_.poses.size()? _poseCloudKeyFrames.size() : path_.poses.size();
    // path_.poses.clear();
    // geometry_msgs::PoseStamped _pose;
    // _pose.header.stamp = ros::Time::now();
    // _pose.header.frame_id = path_.header.frame_id;
    for(int i = 0; i <_num; i++){
        path_.poses[i].pose.position.x = _poseCloudKeyFrames[i+1].translation()[0];
        path_.poses[i].pose.position.y = _poseCloudKeyFrames[i+1].translation()[1];
        path_.poses[i].pose.position.z = _poseCloudKeyFrames[i+1].translation()[2];

        Eigen::Quaterniond _q = Eigen::Quaterniond(_poseCloudKeyFrames[i+1].rotation());
        path_.poses[i].pose.orientation.x = _q.x();
        path_.poses[i].pose.orientation.y = _q.y();
        path_.poses[i].pose.orientation.z = _q.z();
        path_.poses[i].pose.orientation.w = _q.w();

        // path_.poses.push_back(_pose);
    }
}

}