/*
 * @Author: Hu Ziwei 
 * @Description:  存放一些配准时用到的结构体
 * @Date: 2021-12-01 16:26:55 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:35:24
 */

#ifndef LOAM_FRAME_DATA_STRUCT_REGISTER_STRUCT_HPP_
#define LOAM_FRAME_DATA_STRUCT_REGISTER_STRUCT_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

namespace loam_frame{
struct CorrespondFeature{
    Eigen::Vector3d current_point;
    Eigen::Vector3d last_point_a;
    Eigen::Vector3d last_point_b;
    Eigen::Vector3d last_point_c;
};

struct PlaneCorrespondFeature{
    Eigen::Vector3d current_point;
    Eigen::Vector3d norm; //平面的法向量
    double OA_dot_norm;  //1/norm的模长
};

}
#endif