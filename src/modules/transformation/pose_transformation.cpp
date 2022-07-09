/*
 * @Author: Hu Ziwei 
 * @Date: 2021-10-11 15:18:59 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-29 19:25:32
 */

#include "modules/transformation/pose_transformation.hpp"

namespace loam_frame{
PoseTransformation::PoseTransformation(){
    subMapOriginPoseInWorldMap.setIdentity(); //子地图原点在世界地图下的位姿
    currentFrameLidarPoseInSubMap.setIdentity(); //当前激光雷达在子地图下的相对位姿
    lastFrameLidarPoseInSubMap.setIdentity(); //上一帧时刻激光雷达在子地图下的相对位姿
    predictNextFrameLidarPoseInSubMap.setIdentity(); //预测下一帧时激光雷达在世子地图下的相对位姿
    lastKeyFrameLidarPoseInSubMap.setIdentity(); //上一关键帧时刻激光雷达在子地图下的相对位姿
    //世界地图相关位姿
    currentFrameLidarPoseInWorldMap.setIdentity(); //当前时刻激光雷达在世界地图下的位姿
    lastFrameLidarPoseInWorldMap.setIdentity(); //上一关键帧时刻激光雷达在世界地图下的位姿
    predictNextFrameLidarPoseInWorldMap.setIdentity(); //预测下一帧时激光雷达在世界地图下的位姿
    lastKeyFrameLidarPoseInWorldMap.setIdentity(); //上一关键帧时刻激光雷达在世界地图下的位姿
    //相邻帧间的变换矩阵
    poseTransformationFromLastToCurrent.setIdentity(); //相邻两帧间的变换矩阵， lastPose*R=currentPose
    keyPoseTransformationFromLastToCurrent.setIdentity(); //相邻两关键帧间的变换矩阵，lastKeyFramePose*R=currentKeyFramePose
    predictNextposeTransformation.setIdentity();
    currentFrameGnssData.setIdentity();
} 
}// namespace loam_frame