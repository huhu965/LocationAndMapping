/*
 * @Description: 点云滤波模块的接口
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:29:50
 */
#ifndef LOAM_FRAME_MODULES_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define LOAM_FRAME_MODULES_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "sensor_data/point_cloud_data.hpp"

namespace loam_frame {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const PointCloudData::point_cloud_ptr& input_cloud_ptr,PointCloudData::point_cloud_ptr& filtered_cloud_ptr) = 0;
};
}

#endif