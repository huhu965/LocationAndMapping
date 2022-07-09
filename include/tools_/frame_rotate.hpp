/*
 * @Author: Hu Ziwei 
 * @Description:  将角度转为四元数或旋转平移矩阵
 * @Date: 2022-03-31 22:12:33 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-03-31 22:31:08
 */

#ifndef LOAM_FRAME_TOOLS_FRAME_ROTATE_HPP_
#define LOAM_FRAME_TOOLS_FRAME_ROTATE_HPP_

#include <eigen3/Eigen/Dense>

namespace loam_frame {

//输入旋转角度和旋转轴，转为四元数
Eigen::Quaterniond RotateAngleToQuaternion(double angle, const Eigen::Vector3d &rotate_axis){
    double w = cos(angle/2/180.0*M_PI);
    Eigen::Vector3d imaginary_part = sin(angle/2/180.0*M_PI) * rotate_axis;
    Eigen::Quaterniond q(w, imaginary_part.x(), imaginary_part.y(), imaginary_part.z());
    q.normalize();
    return q;
}

//输入旋转角度和旋转轴，转为旋转平移矩阵
Eigen::Isometry3d RotateAngleToIsometry3d(double angle, const Eigen::Vector3d &rotate_axis){
    double w = cos(angle/2/180.0*M_PI);
    Eigen::Vector3d imaginary_part = sin(angle/2/180.0*M_PI) * rotate_axis;
    Eigen::Quaterniond q(w, imaginary_part.x(), imaginary_part.y(), imaginary_part.z());
    q.normalize();
    Eigen::Isometry3d rotation_matrix;
    rotation_matrix.setIdentity();
    rotation_matrix.rotate(q);
    return rotation_matrix;
}

}

#endif