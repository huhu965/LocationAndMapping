/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-29 13:26:16 
 * @Last Modified by: Hu Ziwei 
 * @Last Modified time: 2021-11-29 13:26:16 
 */

#ifndef LOAM_FRAME_MODULES_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define LOAM_FRAME_MODULES_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "modules/cloud_filter/cloud_filter_interface.hpp"

namespace loam_frame {
class VoxelFilter: public CloudFilterInterface {
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const PointCloudData::point_cloud_ptr& input_cloud_ptr,PointCloudData::point_cloud_ptr& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<PointCloudData::point> voxel_filter_;
};
}
#endif