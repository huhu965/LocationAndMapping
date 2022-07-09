/*
 * @Author: Hu Ziwei 
 * @Description:  不滤波
 * @Date: 2021-11-29 13:26:56 
 * @Last Modified by: Hu Ziwei 
 * @Last Modified time: 2021-11-29 13:26:56 
 */
#include "modules/cloud_filter/no_filter.hpp"

namespace loam_frame {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const PointCloudData::point_cloud_ptr& input_cloud_ptr, 
                        PointCloudData::point_cloud_ptr& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new PointCloudData::point_cloud(*input_cloud_ptr));
    return true;
}
} 