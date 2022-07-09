/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-12-01 16:14:57 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-02 19:00:00
 */

#include <ros/ros.h>
#include "glog/logging.h"

#include "global_defination/global_defination.h"
#include "mapping/front_end/front_end_flow.hpp"

using namespace loam_frame;

int main(int argc, char **argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, gnss_topic;
    nh.getParam("/loam_frame/cloud_topic", cloud_topic);
    nh.getParam("/loam_frame/gnss_topic", gnss_topic);

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, cloud_topic, gnss_topic);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        rate.sleep();
    }
    LOG(INFO) << "正常结束!";
    return 0;
}