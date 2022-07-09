/*
 * @Author: Hu Ziwei 
 * @Description:  slam后端的实现流程
 * 输入：理论上没有畸变的点云
 *      该帧点云的运动的预测值（可选，如果不给就用雷达里程计做预测）
 *      预积分的IMU数据，用于和里程计做融合（可选）
 * 功能：相邻帧的运动估计
 * 输出：里程计给出的相邻帧运动估计
 *      关键帧点云
 *      相邻关键帧的运动约束
 * @Date: 2021-11-27 15:18:53 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-14 20:28:25
 */

#ifndef LOAM_FRAME_MAPPING_BACK_END_BACK_END_FLOW_HPP_
#define LOAM_FRAME_MAPPING_BACK_END_BACK_END_FLOW_HPP_

#include <ros/ros.h>

#include "publisher/cloud_publisher.hpp"
#include "publisher/path_publisher.hpp"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "mapping/back_end/back_end.hpp"

namespace loam_frame{
class BackEndFlow {
public:
    BackEndFlow(ros::NodeHandle& nh);

    bool Run();

private:
    bool ReadData();
    bool HasData();
    bool VaildGnss();
    bool UpdateGraph();
    bool PublishData();
    bool Finally();

private:
    std::shared_ptr<BackEnd> backEndPtr_;
    //订阅关键帧点云
    std::shared_ptr<CloudSubscriber> keyframeCloudSubPtr_;
    //发布用优化后位姿投影的点云
    std::shared_ptr<CloudPublisher> PointCloudOptimizedPubPtr_;
    //路径发布
    std::shared_ptr<PathPublisher> pathOptimizedPubPtr_;

    std::shared_ptr<OdometrySubscriber> frameRelativeOdometrySubPtr;
    std::shared_ptr<OdometrySubscriber> frameGnssOdometrySubPtr;

    std::deque<PointCloudData> keyframeCloudDataBuff_;
    std::deque<OdometryData> frameRelativeDataBuff_;
    std::deque<OdometryData> frameGnssDataBuff_;

    std::deque<Eigen::Isometry3d> optimizedPoses_;

    ros::Time current_stamp_;
};

}

#endif
