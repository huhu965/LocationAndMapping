/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-12-01 15:18:41 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 00:39:15
 */

#include "mapping/back_end/back_end_flow.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"

#include "tools_/tic_toc.hpp"
#include "tools_/frame_rotate.hpp"

namespace loam_frame{
BackEndFlow::BackEndFlow(ros::NodeHandle& nh){
    //订阅去过畸变的点云
    keyframeCloudSubPtr_ = std::make_shared<CloudSubscriber>(nh, "downsize_point_cloud", 100000);
    PointCloudOptimizedPubPtr_ = std::make_shared<CloudPublisher>(nh, "optimize_point_cloud", 100, "/map");
    pathOptimizedPubPtr_ = std::make_shared<PathPublisher>(nh, "optimize_path", "/map", 100);

    frameRelativeOdometrySubPtr = std::make_shared<OdometrySubscriber>(nh, "/frame/relative_pose",100);
    frameGnssOdometrySubPtr = std::make_shared<OdometrySubscriber>(nh, "/frame/prioi_gnss",100);

    backEndPtr_ =  std::make_shared<BackEnd>(nh);
}

bool BackEndFlow::ReadData(){
    keyframeCloudSubPtr_->ParseData(keyframeCloudDataBuff_);
    frameRelativeOdometrySubPtr->ParseData(frameRelativeDataBuff_);
    frameGnssOdometrySubPtr->ParseData(frameGnssDataBuff_);
    return true;
}

bool BackEndFlow::HasData(){
    if (keyframeCloudDataBuff_.empty())
        return false;
    if (frameRelativeDataBuff_.empty())
        return false;
    if (frameGnssDataBuff_.empty())
        return false;
    return true;
}

bool BackEndFlow::VaildGnss(){
    while(!frameGnssDataBuff_.empty()){
        if(frameGnssDataBuff_.front().time < frameRelativeDataBuff_.front().time){
            frameGnssDataBuff_.pop_front();
            continue;
        }else if(frameGnssDataBuff_.front().time == frameRelativeDataBuff_.front().time){
            return true;
        }else{
            return false;
        }
    }
    return false;
}

bool BackEndFlow::UpdateGraph(){
    bool is_keyframe_update = false;
    if(keyframeCloudDataBuff_.front().time < frameRelativeDataBuff_.front().time){
        keyframeCloudDataBuff_.pop_front();
        return is_keyframe_update;
    }else if(keyframeCloudDataBuff_.front().time > frameRelativeDataBuff_.front().time){
        frameRelativeDataBuff_.pop_front();
        return is_keyframe_update;
    }

    if(VaildGnss()){
        is_keyframe_update = backEndPtr_->Update(keyframeCloudDataBuff_.front(),frameRelativeDataBuff_.front().pose,
                            frameGnssDataBuff_.front().pose,
                            true);
    }else{
        is_keyframe_update = backEndPtr_->Update(keyframeCloudDataBuff_.front(),frameRelativeDataBuff_.front().pose,
                    frameRelativeDataBuff_.front().pose, //因为给了false，所以这个值任意给
                    false);
    }
    current_stamp_ = ros::Time().fromSec(frameRelativeDataBuff_.front().time);
    frameRelativeDataBuff_.pop_front();
    keyframeCloudDataBuff_.pop_front();


    if(is_keyframe_update){
        if(backEndPtr_->GetOptimizedPose(optimizedPoses_)){
            pathOptimizedPubPtr_->UpdatePath(optimizedPoses_); //更新优化后的路径
        }
    }
    return is_keyframe_update;
}


bool BackEndFlow::PublishData(){
    pathOptimizedPubPtr_->Publish(backEndPtr_->GetCurrentOptimizedPose(), current_stamp_);
    PointCloudOptimizedPubPtr_->Publish(backEndPtr_->GetWorldMap(), current_stamp_);
}

bool BackEndFlow::Finally(){
    return true;
}

bool BackEndFlow::Run() {
    TicToc _time;
    if (!ReadData())    //读取数据
        return false;

    while(HasData()){
        _time.tic();
        if (UpdateGraph()){
            // std::cout<<"图优化时间"<<_time.toc()<<std::endl;
            PublishData();
            Finally();
        }
    }

    return true;
}

}