/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-02 16:05:26
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:36:26
 */
#ifndef LOAM_FRAME_PUBLISHER_PATH_PUBLISHER_HPP_
#define LOAM_FRAME_PUBLISHER_PATH_PUBLISHER_HPP_
#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_data/odometry_data.hpp"
#include "data_struct/pose_struct.hpp"
namespace loam_frame{
class PathPublisher {
  public:
    PathPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string base_frame_id,
                   size_t buff_size);
    PathPublisher() = default;
    ~PathPublisher() = default;

    void UpdatePath(std::deque<Eigen::Isometry3d> &_poseCloudKeyFrames);
    void Publish(const OdometryData::pose_type& transform_matrix);
    void Publish(const OdometryData::pose_type& transform_matrix, ros::Time stamp);

    nav_msgs::Path GetPath(){
      return path_;
    }
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;
};
}
#endif //LOAM_FRAME_PUBLISHER_PATH_PUBLISHER_HPP_
