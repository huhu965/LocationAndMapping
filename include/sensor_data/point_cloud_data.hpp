/* 
 * @Author: Hu Ziwei
 * @Description: 点云数据格式
 * @Date: 2021-08-31 17:04:25
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-09-08 17:04:02
 */
#ifndef LOAM_FRAME_SENSOR_DATA_POINT_CLOUD_DATA_HPP_
#define LOAM_FRAME_SENSOR_DATA_POINT_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace loam_frame {
class PointCloudData {
  public:
    using point = pcl::PointXYZI;
    using point_cloud = pcl::PointCloud<point>;
    using point_cloud_ptr = point_cloud::Ptr;

  public:
    PointCloudData()
      :cloud_ptr(new point_cloud()) {
    }

  public:
    double time = 0.0;
    point_cloud_ptr cloud_ptr;
};
}

#endif //LOAM_FRAME_SENSOR_DATA_POINT_CLOUD_DATA_HPP_