/*
 * @Author: Hu Ziwei 
 * @Description:  不滤波
 * @Date: 2021-11-29 12:59:33 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-29 13:17:28
 */
#ifndef LOAM_FRAME_MODULES_CLOUD_FILTER_NO_FILTER_HPP_
#define LOAM_FRAME_MODULES_CLOUD_FILTER_NO_FILTER_HPP_

#include "modules/cloud_filter/cloud_filter_interface.hpp"

namespace loam_frame {
class NoFilter: public CloudFilterInterface{
  public:
    NoFilter();

    bool Filter(const PointCloudData::point_cloud_ptr& input_cloud_ptr,PointCloudData::point_cloud_ptr& filtered_cloud_ptr) override;
};
}
#endif