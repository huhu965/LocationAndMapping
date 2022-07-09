/*
 * @Author: Hu Ziwei 
 * @Description:  存放位姿变换相关的矩阵
 * @Date: 2021-11-27 15:02:08 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-29 19:21:20
 */

#ifndef LOAM_FRAME_MODULES_TRANSFORMATION_POSE_TRANSFORMATION_HPP_
#define LOAM_FRAME_MODULES_TRANSFORMATION_POSE_TRANSFORMATION_HPP_

#include <eigen3/Eigen/Dense>

namespace loam_frame{
/*
 * @Description: 位姿变换相关变量
*/
class PoseTransformation{
public:
    PoseTransformation();
    ~PoseTransformation() = default;
public:
    //欧式变换矩阵 4*4
    //子地图相关位姿
    Eigen::Isometry3d subMapOriginPoseInWorldMap; //子地图原点在世界地图下的位姿
    Eigen::Isometry3d currentFrameLidarPoseInSubMap; //当前激光雷达在子地图下的相对位姿
    Eigen::Isometry3d lastFrameLidarPoseInSubMap; //上一帧时刻激光雷达在子地图下的相对位姿
    Eigen::Isometry3d predictNextFrameLidarPoseInSubMap; //预测下一帧时激光雷达在世子地图下的相对位姿
    Eigen::Isometry3d lastKeyFrameLidarPoseInSubMap; //上一关键帧时刻激光雷达在子地图下的相对位姿
    //世界地图相关位姿
    Eigen::Isometry3d currentFrameLidarPoseInWorldMap; //当前时刻激光雷达在世界地图下的位姿
    Eigen::Isometry3d lastFrameLidarPoseInWorldMap; //上一关键帧时刻激光雷达在世界地图下的位姿
    Eigen::Isometry3d predictNextFrameLidarPoseInWorldMap; //预测下一帧时激光雷达在世界地图下的位姿
    Eigen::Isometry3d lastKeyFrameLidarPoseInWorldMap; //上一关键帧时刻激光雷达在世界地图下的位姿
    //相邻帧间的变换矩阵
    Eigen::Isometry3d poseTransformationFromLastToCurrent; //相邻两帧间的变换矩阵， lastPose*R=currentPose
    Eigen::Isometry3d keyPoseTransformationFromLastToCurrent; //相邻两关键帧间的变换矩阵，lastKeyFramePose*R=currentKeyFramePose
    Eigen::Isometry3d predictNextposeTransformation; //预测当前帧到下一帧的运动

    Eigen::Vector3d currentFrameGnssData; //当前帧的gnss数据
};

}
#endif      //LOAM_FRAME_MODULES_TRANSFORMATION_POSE_TRANSFORMATION_HPP_