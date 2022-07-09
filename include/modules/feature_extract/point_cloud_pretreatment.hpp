/* 
 * @Author: Hu Ziwei
 * @Description: 点云预处理
 * 1.去除无效点
 * 2.去除距离雷达过近的点
 * 3.补偿雷达产生的畸变
 * @Date: 2021-08-31 15:03:16
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-29 16:31:45
 */
#ifndef LOAM_FRAME_MODULES_POINT_CLOUD_HANDLE_POINT_CLOUD_PRETREATMENT_HPP_
#define LOAM_FRAME_MODULES_POINT_CLOUD_HANDLE_POINT_CLOUD_PRETREATMENT_HPP_

#include <vector>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

#include "sensor_data/point_cloud_data.hpp"
namespace loam_frame{

class PointCloudPretreatment{
public:
    PointCloudPretreatment();
    ~PointCloudPretreatment() = default;
    /*
    * @Description:输入点云
    * @Params: _cloud_in 要放入的点云
    * @Returns:
    */
    void PointCloudInput(const PointCloudData::point_cloud &cloud_in);
    /*
        * @Description:输出点云
    */
    PointCloudData::point_cloud PointCloudOutput();
    /*
        * @Description:去除点云中的NAN点
    */
    void RemoveNullPointFromPointCloud(){
        std::vector<int> indices;//指数
        pcl::removeNaNFromPointCloud(*point_cloud_ptr_, *point_cloud_ptr_, indices);
    }
    /*
        * @Description:去除点云中距离雷达过近的点
        * @Params: _threshold 要移除点的阈值
    */
    void RemoveClosedPointFromPointCloud(float threshold){
        size_t _point_count = 0;

        for (size_t i = 0; i < point_cloud_ptr_->points.size(); ++i)
        {
            if ( (point_cloud_ptr_->points[i].x * point_cloud_ptr_->points[i].x 
                + point_cloud_ptr_->points[i].y * point_cloud_ptr_->points[i].y 
                + point_cloud_ptr_->points[i].z * point_cloud_ptr_->points[i].z) < threshold * threshold)
                continue;

            point_cloud_ptr_->points[_point_count] = point_cloud_ptr_->points[i];
            _point_count++;
        }
        if (_point_count != point_cloud_ptr_->points.size())
        {
            point_cloud_ptr_->points.resize(_point_count);
        }

        point_cloud_ptr_->height = 1;
        point_cloud_ptr_->width = static_cast<uint32_t>(_point_count);
        point_cloud_ptr_->is_dense = true; //是否是稠密的 
    }
    /*
        * @Description:去除雷达运动造成的点云畸变
    */
    void CompensateLidarMoveDistortion();
        void FindStartEndAngle();
private:
    PointCloudData::point_cloud_ptr point_cloud_ptr_;
};

}


#endif      //LOAM_FRAME_MODULES_POINT_CLOUD_HANDLE_POINT_CLOUD_PRETREATMENT_HPP_