/* 
 * @Author: Hu Ziwei
 * @Description: icp 配准 寻找最近的5个点，对点云协方差矩阵进行主成份分析：
 * 若这5个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，与该特征值相关的特征向量表示所处直线的方向;
 * 若这5个点分布在平面上，协方差矩阵的特征值存在一个显著小的元素，与该特征值相关的特征向量表示所处平面的法线方向。
 * 参考论文:2016,IROS,fast and robust 3d feature extraction from sparse point clouds
 * @Date: 2021-09-08 13:23:32
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-30 19:47:52
 */

#ifndef LOAM_FRAME_MODULES_REGISTRATION_ICP_FAST_REGISTRATION_HPP_
#define LOAM_FRAME_MODULES_REGISTRATION_ICP_FAST_REGISTRATION_HPP_

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_data/point_cloud_data.hpp"
#include "modules/registration/registration_interface.hpp"
#include "modules/registration/ICP/lidar_factor.hpp"
#include "modules/transformation/pose_transformation.hpp"
#include "data_struct/register_struct.hpp"

namespace loam_frame{
class FastRegistration: public RegistrationInterface{
public:
    FastRegistration(const YAML::Node& configNode);
    bool PointCloudInput(const PointCloudData::point_cloud_ptr sourceCornerCloud,
                        const PointCloudData::point_cloud_ptr targetCornerCloud,
                        const PointCloudData::point_cloud_ptr sourceSurfaceCloud,
                        const PointCloudData::point_cloud_ptr targetSurfaceCloud) override;
    /*
     * @Description:给定预测位姿
    */
    bool SetPredictPose(const Eigen::Isometry3d &predictPose) override;
    /*
     * @Description:执行匹配
    */
    bool ScanMatch(int iterCount)override;

    Eigen::Isometry3d GetMatchResult() override;
private:
    /*
     * @Description:初始化
    */
    bool InitWithConfig(const YAML::Node& configNode);
    /*
     * @Description:查找对应的角特征
    */
    void FindCorrespondingCornerFeatures();
    /*
     * @Description:查找对应的面特征
    */
    void FindCorrespondingSurfaceFeatures();
    /*
     * @Description:根据查找到的线面特征，计算优化两帧之间的位姿变化
    */
    void CalculateTransformation();
 
public:

    pcl::KdTreeFLANN<PointCloudData::point>::Ptr kdtreeTargetCornerCloudPtr_; //存放目标点云的角点
    pcl::KdTreeFLANN<PointCloudData::point>::Ptr kdtreeTargetSurfaceCloudPtr_; //存放目标点云的平面点
    //接收外部给进的数据
    PointCloudData::point_cloud_ptr sourceCornerCloudPtr_; //源点云
    PointCloudData::point_cloud_ptr targetCornerCloudPtr_; //目标点云
    PointCloudData::point_cloud_ptr sourceSurfaceCloudPtr_; //源点云
    PointCloudData::point_cloud_ptr targetSurfaceCloudPtr_; //目标点云
    //上一帧到当前帧的位姿变换
    double param_q[4] = {0, 0, 0, 1}; //从上一帧到当前帧的旋转矩阵
    double param_t[3] = {0, 0, 0}; //从上一帧原点指向当前帧。是上一帧到当前帧的位移
    //Map类并没有自己申请一片空内存，只是一个引用，共享param_q的地址
    //Tc = Tl + Rl * T_LastPoseTranslationToCurrentPose
    //Rc = Rl * Q_LastPoseRotationToCurrentPose
    Eigen::Map<Eigen::Quaterniond> Q_LastPoseRotationToCurrentPose = Eigen::Map<Eigen::Quaterniond>(param_q);
    Eigen::Map<Eigen::Vector3d> T_LastPoseTranslationToCurrentPose = Eigen::Map<Eigen::Vector3d>(param_t);
    
private:
    double cornerDistanceSquareThreshold_; //查找对应点时的距离阈值的平方
    double surfaceDistanceSquareThreshold_; //查找对应点时的距离阈值的平方
    std::vector<CorrespondFeature> cornerCorrespondFeature_;
    std::vector<PlaneCorrespondFeature> surfaceCorrespondFeature_;
    Eigen::Isometry3d matchResult_;
    Eigen::Isometry3d predictMatchResult_;
};

}

#endif  //LOAM_FRAME_MODULES_REGISTRATION_ICP_FAST_REGISTRATION_HPP_