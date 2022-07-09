/* 
 * @Author: Hu Ziwei
 * @Description: 地面优化特征点提取，改自lego-loam
 * 1.计算平滑度
 * 2.提取四类特征点
 * @Date: 2021-08-31 15:03:16
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-01 16:01:03
 */
#ifndef LOAM_FRAME_MODULES_FEATURE_EXTRACT_GO_FEATURE_EXTRACT_HPP_
#define LOAM_FRAME_MODULES_FEATURE_EXTRACT_GO_FEATURE_EXTRACT_HPP_

#include <vector>
#include <algorithm> 
#include <cmath>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include "modules/feature_extract/feature_extract_interface.hpp"
#include "modules/cloud_filter/cloud_filter_interface.hpp"

namespace loam_frame{

#define GroundOrInvalid -1

class GoFeatureExtract:public FeatureExtractInterface{
public:
    GoFeatureExtract(const YAML::Node& config_node);
    ~GoFeatureExtract() = default;
    bool PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr) override;
    /*
    * @Description: 执行一次特征提取
    */
    bool RunOnceExtractFeatures() override; 
    /*
    * @Description: 重载 重置参数函数
    */
    void ResetParameters() override;

    PointCloudData::point_cloud_ptr GETImageCloud() override;
    PointCloudData::point_cloud_ptr GETGroundCloud() override;
    PointCloudData::point_cloud_ptr GETObstacleCloud() override;
    PointCloudData::point_cloud_ptr GETSegmentedCloud() override;
    PointCloudData::point_cloud_ptr GETCornerPoints() override;
    PointCloudData::point_cloud_ptr GETLessSharpCornerPoints() override;
    PointCloudData::point_cloud_ptr GETSurfacePoints() override;
    PointCloudData::point_cloud_ptr GETLessFlatSurfacePoints() override;
private:
    enum{
        SharpPoint=-2,
        LessSharpPoint=-1,
        NormalPoint=0,
        LessFlatPoint=1,
        FlatPoint=2
    }; //相当于宏定义，不存在在对象中，只是编译的时候会用数字替换掉。
    /*
     * @Description: 按照配置初始化参数
    */
    bool InitWithConfig(const YAML::Node& config_node);
    /*
     * @Description: 按照配置初始化雷达参数 
    */
    bool InitLidarParams(const YAML::Node& config_node);
    /*
     * @Description: 按照配置初始化特征提取参数
    */
    bool InitFeatureExtractParams(const YAML::Node& config_node);
    /*
     * @Description: 按照配置初始化滤波器 
    */
    bool InitFilter(const YAML::Node& config_node);
    /*
     * @Description:3d点云投影到2维
    */
    bool ProjectPointCloudToImage();
    /*
     * @Description:查找疑似地面点并标记下来
    */
    bool FindGroundPoint();
    /*
     * @Description:广度优先，lego-loam方法的夹角判断是否一类
    */
    void LabelComponents(int row, int col);
    /*
     * @Description:对所有点聚类，然后按类打上不同的标签
    */
    bool CloudSegmentation();
    /*
     * @Description:分别保存地面点和聚类出的点
    */
    bool SaveDifferentClouds();
    /*
        * @Description:计算平滑度
    */  
    bool CalculateSmoothness();
    /*
        * @Description:提取特征点
    */       
    bool ExtractFeatures();

private:
    PointCloudData::point_cloud_ptr point_cloud_ptr_; //原始点云,点表示为(x,y,z),无法确定点来自那条线，那个水平角度
    PointCloudData::point_cloud_ptr image_point_cloud_ptr_; //原始点云投影到2维平面，纵坐标0-16，表示来自那条线
    PointCloudData::point_cloud_ptr ground_cloud_ptr_; //分割出的地面点
    PointCloudData::point_cloud_ptr obstacle_cloud_ptr_; //分割聚类后的非地面点
    PointCloudData::point_cloud_ptr segmented_cloud_ptr_; //地面点和非地面点的集合,用于后面的特征提取

    PointCloudData::point_cloud_ptr corner_points_sharp_ptr_; //平滑度最大的几个角点
    PointCloudData::point_cloud_ptr corner_points_less_sharp_ptr_; //平滑度没那么大的角点，包含最大的几个角点
    PointCloudData::point_cloud_ptr surface_points_flat_ptr_; //平滑度最小的几个平面点，只在地面点提取
    PointCloudData::point_cloud_ptr surface_points_less_flat_ptr_; //平滑度没那么小的几个平面点，包括最小的几个平面点

    PointCloudData::point_cloud_ptr surf_points_less_flat_scan_; //存每条线的平面点，用于后面降采样
    PointCloudData::point_cloud_ptr surf_points_less_flat_scan_downsize_; //存每条线降采样后的平面点

    /*
     * @Description:点云存储是按照一维数组存储的，所以要手动记录每一行点云在数组中的开始和结束位置
        用于后面平均对每个部分进行特征提取
    */
    std::vector<RingIndex> segmented_cloud_ring_index_; //记录分割后每行点云在数组中的开始和结束位置
    std::vector<PointSmoothness> cloud_smoothness_;//用于点云弯曲度排序
    std::vector<SegmentedPointParam> segmented_cloud_point_param_; //记录分割后每行点云在数组中的开始和结束位置

    int vertical_scans_;//垂直有几条线
    int horizontal_scans_;//水平扫描的点数
    float vertical_angle_resolution_;//垂直角度分布率
    float horizontal_angle_resolution_; //水平角度分辨率
    float vertical_angle_bottom_; //雷达扫描范围的垂直最低角度
    float vertical_angle_top_; //雷达扫描范围的最高角度
    int ground_scan_index_; //低于该scan索引的水平扫描scan才有可能包括地面点，所以一般为0°时的scan
    float sensor_mount_angle_;//雷达的倾斜角度，没啥用，直接给0，用到再说
    float segment_theta_; //聚类两点连线和点到雷达的连线夹角阈值
    int label_count_; //类标签的值
    //下面两个变量是为了细节处理设置的，可以更好的检测到竖直狭窄障碍物
    int segment_valid_point_num_;//竖直狭窄障碍物可能聚类的点的数量阈值
    int segment_valid_line_num_;//竖直狭窄障碍物可能聚类的线的阈值
    //点云的起始角度，可能大于360，也可能小于
    float start_orientation;  //雷达扫描的开始角度
    float end_orientation; //雷达扫描的结束角度
    float orientation_difference; //雷达本次scan扫描到的角度。

    float edge_threshold_; //角点平滑度阈值
    float surf_threshold_; //平面点平滑度阈值

    double ground_theta_threshold_;
    /*
     * @Description: 下面四个参数都是描述投影到2维图点云的
    */
    Eigen::MatrixXf horizontal_range_mat_;   // 存储点到雷达水平距离的矩阵
    Eigen::MatrixXf range_mat_;   // 存储点到雷达距离的矩阵
    Eigen::MatrixXf intensity_mat_;   // 存储点信号强度的矩阵
    Eigen::MatrixXi label_mat_;   // 存储点云分割后每个点的标签的矩阵
    Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> ground_mat_;  //标记是否是地面的的矩阵

    std::shared_ptr<CloudFilterInterface> downSizeFilterPtr_;//降采样滤波
};

}
#endif      //LOAM_FRAME_MODULES_FEATURE_EXTRACT_FEATURE_EXTRACT_HPP_