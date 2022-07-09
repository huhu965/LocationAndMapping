/*
 * @Author: Hu Ziwei 
 * @Description:点云处理的使用接口
 * @Date: 2021-11-04 14:45:37 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:13:55
 */
#ifndef LOAM_FRAME_MODULES_FEATURE_EXTRACT_INTERFACE_HPP_
#define LOAM_FRAME_MODULES_FEATURE_EXTRACT_INTERFACE_HPP_

#include "sensor_data/point_cloud_data.hpp"

namespace loam_frame {

struct PointSmoothness{
    float value; //点的平滑度
    size_t index; //点在点云中的索引值
};
struct SegmentedPointParam{
    bool is_ground; //是否为地面点
    bool is_neighbor_picked;//是否被挑选或有附近点被挑选过
    size_t column_index; //点的列索引值
    size_t row_index; //点的行索引值
    float smothness; //点的平滑度
};

struct RingIndex{
    size_t start_index;
    size_t end_index;
};
struct by_value{ 
    bool operator()(PointSmoothness const &left, PointSmoothness const &right) { 
        return left.value < right.value;
    }
    bool operator()(PointSmoothness const &left) { 
        return false;
    }
};

/*
* @Description: 接口调用流程
* PointCloudInput()
* RunOnceExtractFeatures()
* GetXxxxx()
* ResetParameters()
*/
class FeatureExtractInterface {
  public:
    virtual ~FeatureExtractInterface() = default;
    virtual bool PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr) = 0;
    /*
    * @Description: 执行一次特征提取
    */
    virtual bool RunOnceExtractFeatures() = 0;
    /*
    * @Description: 重置参数
    */
    virtual void ResetParameters() = 0;

    virtual PointCloudData::point_cloud_ptr GETImageCloud(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETGroundCloud(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETObstacleCloud(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETSegmentedCloud(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETCornerPoints(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETLessSharpCornerPoints(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETSurfacePoints(){
        return PointCloudData::point_cloud().makeShared();
    }
    virtual PointCloudData::point_cloud_ptr GETLessFlatSurfacePoints(){
        return PointCloudData::point_cloud().makeShared();
    }
};
}
#endif //LOAM_FRAME_MODULES_FEATURE_EXTRACT_INTERFACE_HPP_