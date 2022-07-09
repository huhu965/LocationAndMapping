/*
 * @Author: Hu Ziwei 
 * @Description:  点云畸变补偿
 * @Date: 2021-12-10 13:17:13 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 00:46:01
 */

#ifndef LOAM_FRAME_MODULES_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LOAM_FRAME_MODULES_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "sensor_data/point_cloud_data.hpp"

namespace loam_frame {
class DistortionAdjust {
  public:
    DistortionAdjust();
    /*
    * @Description: scanMotion是当前帧，周期内的运动，我们假设一个周期内都是匀速运动，可能匀加速运动会更好？
    * scanpPeriod 一帧的周期时长
    */
    void PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr, const Eigen::Isometry3d &scanMotion);
    bool AdjustCloud();

    PointCloudData::point_cloud_ptr GetAdjustImagePointCloud();
    PointCloudData::point_cloud_ptr GetAdjustPointCloud();

  private:
    bool InitLidarParams(const YAML::Node& config_node);
    inline Eigen::Matrix3f UpdateMatrix(float real_time);
    void ProjectPointCloudToImage();  //投影成了类似image
    void Adjust(); //补偿畸变
    void ClearCloud(); //清理点云，并重置
    void GeneratePointCloud(); //生成新点云

  private:
    Eigen::Isometry3d scanMotion_;
    PointCloudData::point_cloud_ptr point_cloud_ptr_;
    PointCloudData::point_cloud_ptr image_point_cloud_ptr_;
    PointCloudData::point_cloud_ptr adjust_point_cloud_ptr_;
    //点云的起始角度，可能大于360，也可能小于
    float startOrientation;  //雷达扫描的开始角度
    float endOrientation; //雷达扫描的结束角度
    float orientationDifference; //雷达本次scan扫描到的角度。
    double currentFrameDuration_; //当前帧的扫描持续时间

    Eigen::MatrixXi number_mat_;   // 有几个点投影到像素点
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;

    int vertical_scans_;//垂直有几条线
    int horizontal_scans_;//水平扫描的点数
    float vertical_angle_resolution_;//垂直角度分布率
    float horizontal_angle_resolution_; //水平角度分辨率
    float vertical_angle_bottom_; //雷达扫描范围的垂直最低角度
    float vertical_angle_top_; //雷达扫描范围的最高角度
    float sensor_mount_angle_;//雷达的倾斜角度，没啥用，直接给0，用到再说

    std::vector<PointCloudData::point_cloud> laserCloudScans;
};

} // namespace lidar_slam
#endif