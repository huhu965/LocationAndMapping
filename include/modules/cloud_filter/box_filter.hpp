/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2020-03-04 20:09:37
 */

#ifndef LOAM_FRAME_MODULES_CLOUD_FILTER_BOX_FILTER_HPP_
#define LOAM_FRAME_MODULES_CLOUD_FILTER_BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>
#include "modules/cloud_filter/cloud_filter_interface.hpp"

namespace loam_frame {
class BoxFilter: public CloudFilterInterface {
  public:
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const PointCloudData::point_cloud_ptr& input_cloud_ptr,PointCloudData::point_cloud_ptr& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<PointCloudData::point> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}

#endif 