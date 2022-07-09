/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-29 19:19:08 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 00:59:33
 */

#include "modules/transformation/point_cloud_transformation.hpp"

namespace loam_frame{
void  PointCloudTransformation::TransformToLast(const PointCloudData::point &point_in, 
                                                PointCloudData::point &point_out, 
                                                const Eigen::Isometry3d &stepPose){
    Eigen::Vector3d point(point_in.x, point_in.y, point_in.z);
    //四元数在代码中可以直接乘向量，数学上是 qvq*
    Eigen::Vector3d un_point = stepPose * point;

    point_out.x = un_point.x();
    point_out.y = un_point.y();
    point_out.z = un_point.z();
    // point_out.intensity = point_in.intensity;
}

void PointCloudTransformation::TransformToWorld(const PointCloudData::point_cloud_ptr cloudIn,
                        PointCloudData::point_cloud_ptr cloudOut,
                        const Eigen::Isometry3d &_pose){
    //如果大小不相等
    if(cloudIn->points.size() != cloudOut->points.size()){
        cloudOut->points.resize(cloudIn->points.size());
    }
    for (int i = 0; i < cloudIn->points.size(); i++){
        PointCloudData::point _point_out;
        Eigen::Vector3d _point(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
        Eigen::Vector3d _un_point = _pose * _point;
        cloudOut->points[i].x = _un_point.x();
        cloudOut->points[i].y = _un_point.y();
        cloudOut->points[i].z = _un_point.z();
        // cloudOut->points[i].intensity = cloudIn->points[i].intensity;
    }
}

PointCloudData::point_cloud PointCloudTransformation::TransformToWorld(const PointCloudData::point_cloud_ptr cloudIn,
                        const Eigen::Isometry3d &_pose){
    PointCloudData::point_cloud _cloud_out;
    for (int i = 0; i < cloudIn->points.size(); i++){
        PointCloudData::point _point_out;
        Eigen::Vector3d _point(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
        Eigen::Vector3d _un_point = _pose * _point;
        _point_out.x = _un_point.x();
        _point_out.y = _un_point.y();
        _point_out.z = _un_point.z();
        // _point_out.intensity = cloudIn->points[i].intensity;
        _cloud_out.push_back(_point_out);
    }
    return _cloud_out;
}

}