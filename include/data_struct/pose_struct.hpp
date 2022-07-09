/*
 * @Author: Hu Ziwei 
 * @Description:  存放一些通用的结构体
 * @Date: 2021-12-01 16:26:55 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:35:24
 */

#ifndef LOAM_FRAME_DATA_STRUCT_POSE_STRUCT_HPP_
#define LOAM_FRAME_DATA_STRUCT_POSE_STRUCT_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

namespace loam_frame{
struct PoseParam
{     
    Eigen::Isometry3d pose; //包含旋转和平移
    double roll;         
    double pitch;
    double yaw;
    double time;
    int index;

    PoseParam(){
        index = 0;
        pose.setIdentity();
        roll = 0;
        pitch = 0;
        yaw = 0;
        time = 0;
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}
#endif