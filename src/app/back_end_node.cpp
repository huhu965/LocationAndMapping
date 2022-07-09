/*
 * @Author: Hu Ziwei 
 * @Description:  后端
 * @Date: 2022-04-02 17:04:30 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-02 17:09:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "global_defination/global_defination.h"
#include "mapping/back_end/back_end_flow.hpp"

using namespace loam_frame;

int main(int argc, char **argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_flow_ptr->Run();

        rate.sleep();
    }
    LOG(INFO) << "后端正常结束!";
    return 0;
}