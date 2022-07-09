/* 
 * @Author: Hu Ziwei
 * @Description: 
 * @Date: 2021-09-19 20:42:22
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-08 16:16:21
 */
#include "subscriber/path_subscriber.hpp"

namespace loam_frame{

PathSubscriber::PathSubscriber(ros::NodeHandle& nh, std::string topic_name, int buff_size):nh_(nh){
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PathSubscriber::msg_callback, this);
}
//把信息保存到队列中，等待处理
void PathSubscriber::msg_callback(const nav_msgs::Path &path) {
    new_path_ = path;
}

//从队列中把数据取出来
void PathSubscriber::ParseData(nav_msgs::Path &path) {
    path = new_path_;
}

}