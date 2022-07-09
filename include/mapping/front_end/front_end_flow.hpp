/*
 * @Author: Hu Ziwei 
 * @Description:  slam前端的实现流程
 * 输入：理论上没有畸变的点云
 *      该帧点云的运动的预测值（可选，如果不给就用雷达里程计做预测）
 *      预积分的IMU数据，用于和里程计做融合（可选）
 * 功能：相邻帧的运动估计
 * 输出：里程计给出的相邻帧运动估计
 *      关键帧点云
 *      相邻关键帧的运动约束
 * @Date: 2021-11-27 15:18:53 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-14 21:55:11
 */

#ifndef LOAM_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define LOAM_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>

#include "publisher/cloud_publisher.hpp"
#include "publisher/path_publisher.hpp"
#include "publisher/odometry_publisher.hpp"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"

#include "mapping/front_end/front_end.hpp"
#include "tf_listener/tf_listener.hpp"
#include "data_pretreat/imu_preintegrate.hpp"
#include "modules/scan_adjust/distortion_adjust.hpp"

namespace loam_frame{
struct DataFrame{
    PointCloudData cloud;
    Eigen::Isometry3d preintegrate_imu_pose;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class FrontEndFlow {
public:
    FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string gnss_topic);

    bool Run();

private:
    bool ReadData();
    bool InitUnsyncedData();
    bool SyncDataClock();
    bool PreintegrateIMU(IMUData current_imu_data);
    bool HasData();
    bool ValidGNSSData();
    bool UpdateLaserOdometry();
    bool PublishData();
    bool Finally();

private:
    std::shared_ptr<FrontEnd> frontEndPtr_;
    std::shared_ptr<ImuPreintegrate> imuPreintegratePtr_;
    std::shared_ptr<DistortionAdjust> distortCloudPtr_;
    //订阅去过畸变的点云
    std::shared_ptr<CloudSubscriber> cloudSubPtr_;
    //订阅gnss数据
    std::shared_ptr<GNSSSubscriber> gnssSubPtr_;
    //订阅imu数据
    std::shared_ptr<IMUSubscriber> imuSubPtr_;
    //订阅坐标轴转换
    std::shared_ptr<TFListener> worldToBaselinkLisPtr;
    //发布降采样后的点云给后端生成地图
    std::shared_ptr<CloudPublisher> downsizePointCloudPubPtr_;
    //发布投影到2d的点云
    std::shared_ptr<CloudPublisher> imagePointCloudPubPtr_;
    //发布分割出的地面点
    std::shared_ptr<CloudPublisher> groundCloudPubPtr_;
    //发布分割出的障碍物
    std::shared_ptr<CloudPublisher> obstacleCloudPubPtr_;
    //发布地面点和障碍物点的集合点云
    std::shared_ptr<CloudPublisher> segmentedCloudPubPtr_;
    //发布提取出的四种特征点
    std::shared_ptr<CloudPublisher> cornerPointsSharpPubPtr_; 
    std::shared_ptr<CloudPublisher> cornerPointsLessSharpPubPtr_;
    std::shared_ptr<CloudPublisher> surfacePointsFlatPubPtr_;
    std::shared_ptr<CloudPublisher> surfacePointsLessFlatPubPtr_;
    //发布子地图
    std::shared_ptr<CloudPublisher> cornerSubmapPubPtr_;
    std::shared_ptr<CloudPublisher> surfaceSubmapPubPtr_;
    std::shared_ptr<CloudPublisher> cloudWorldPubPtr_;
    //发布去畸变后的点云
    std::shared_ptr<CloudPublisher> cloudAdjustPubPtr_;
    //路径发布
    std::shared_ptr<PathPublisher> pathPubPtr_;
    std::shared_ptr<PathPublisher> gnssPathPubPtr_;

    std::shared_ptr<OdometryPublisher> lidarOdometryPubPtr;
    std::shared_ptr<OdometryPublisher> gnssOdometryPubPtr;

    std::deque<PointCloudData> cloudDataBuff_;
    std::deque<GNSSData> unsyncedGnssDataBuff_;
    std::deque<GNSSData> syncedGnssDataBuff_;
    std::deque<IMUData> unsyncedImuDataBuff_;
    std::deque<IMUData> syncedImuDataBuff_;

    std::deque<DataFrame> dataFrames;

    double beginIntegrateTime;

    pcl::VoxelGrid<PointCloudData::point> downSizeFilterImagePointCloud_; //降采样
    PointCloudData::point_cloud_ptr ImagePointCloudDSPtr;

    //世界坐标系到本体坐标系的变换矩阵
    Eigen::Isometry3d worldToBaselink = Eigen::Isometry3d::Identity();
    // Eigen::Matrix4f worldToBaselink = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f imu_pose_ = Eigen::Matrix4f::Identity();
    bool gnssOriginPositionInited = false;
    bool transformReceived = false;
    bool initCloudData = false;
    bool adjust_distort_ = true;
};

}

#endif
