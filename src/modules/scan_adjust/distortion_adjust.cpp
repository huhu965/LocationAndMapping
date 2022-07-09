/*
 * @Author: Hu Ziwei 
 * @Description: 点云畸变补偿 
 * @Date: 2021-12-10 13:49:11 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 00:51:29
 */

#include "glog/logging.h"

#include "global_defination/global_defination.h"
#include "modules/scan_adjust/distortion_adjust.hpp"

namespace loam_frame {
DistortionAdjust::DistortionAdjust():adjust_point_cloud_ptr_(new PointCloudData::point_cloud()),
                                    image_point_cloud_ptr_(new PointCloudData::point_cloud()){
    std::string _config_file_path = WORK_SPACE_PATH + "/config/lidar.yaml";
    YAML::Node _lidar_config_node = YAML::LoadFile(_config_file_path);
    InitLidarParams(_lidar_config_node);
}

bool DistortionAdjust::InitLidarParams(const YAML::Node& config_node){
    vertical_scans_ = config_node["num_vertical_scans"].as<int>();
    horizontal_scans_ = config_node["num_horizontal_scans"].as<int>();
    vertical_angle_bottom_ = config_node["vertical_angle_bottom"].as<float>();
    vertical_angle_top_ = config_node["vertical_angle_top"].as<float>();
    sensor_mount_angle_ = config_node["sensor_mount_angle"].as<float>();
    //转为弧度
    vertical_angle_top_ = M_PI / float(180) *vertical_angle_top_;
    vertical_angle_bottom_ = M_PI / float(180) *vertical_angle_bottom_;
    sensor_mount_angle_ = sensor_mount_angle_ * M_PI / float(180);
    //计算分辨率
    horizontal_angle_resolution_ = (M_PI*2) / (horizontal_scans_);
    vertical_angle_resolution_ = (vertical_angle_top_ - vertical_angle_bottom_) / float(vertical_scans_-1);
    return true;
}

void DistortionAdjust::PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr, const Eigen::Isometry3d &scanMotion){
    point_cloud_ptr_ = pointCloudPtr;
    scanMotion_ = scanMotion;
}

void DistortionAdjust::ClearCloud() {
    //清理投影到2维用到的变量
    number_mat_.resize(vertical_scans_, horizontal_scans_);
    number_mat_.fill(0);
    PointCloudData::point _nanPoint;
    _nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    _nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    _nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    laserCloudScans.resize(vertical_scans_);
    for(int i =0; i<vertical_scans_;i++){
        laserCloudScans[i].resize(horizontal_scans_);
        std::fill(laserCloudScans[i].begin(), laserCloudScans[i].end(), _nanPoint);
    }
    //清理补偿畸变后用的变量
    image_point_cloud_ptr_->clear();
    image_point_cloud_ptr_->points.resize(vertical_scans_ * horizontal_scans_);
    std::fill(image_point_cloud_ptr_->points.begin(), image_point_cloud_ptr_->points.end(), _nanPoint);
    
    adjust_point_cloud_ptr_->clear();
}

void DistortionAdjust::ProjectPointCloudToImage(){
    // range image projection
    //先投影成2维形式
    const size_t cloudSize = point_cloud_ptr_->points.size();

    for (size_t i = 0; i < cloudSize; ++i) {
        PointCloudData::point _point = point_cloud_ptr_->points[i];

        float _range = sqrt(_point.x * _point.x + _point.y * _point.y + _point.z * _point.z);
        if (_range < 0.1) continue; //如果距离雷达的距离过近，就直接筛掉

        // 求垂直角度
        float _vertical_angle = std::asin(_point.z / _range);
        //四舍五入取整
        int _row_index =  vertical_scans_ - round((_vertical_angle - vertical_angle_bottom_) / vertical_angle_resolution_) - 1;
        //限制范围
        _row_index = _row_index < 0? 0 : _row_index;
        _row_index = _row_index < vertical_scans_? _row_index : vertical_scans_-1;

        //加负号是因为雷达扫描是顺时针扫描，所以加个负号
        float _horizon_angle = -std::atan2(_point.y, _point.x);
        if(_horizon_angle < 0){
            _horizon_angle += 2 * M_PI;
        }
        //x正方向为起点，顺时针为正方向
        //x指向雷达的正前方
        int _column_index = round(_horizon_angle / horizontal_angle_resolution_);

        //限制范围
        _column_index = _column_index < 0? 0 : _column_index;
        _column_index = _column_index < horizontal_scans_? _column_index : horizontal_scans_-1;
        number_mat_(_row_index, _column_index) ++;
        //存放时间在周期内的时间
        _point.intensity = double( _column_index * vertical_scans_ + _row_index)/double(horizontal_scans_ * vertical_scans_);

        laserCloudScans[_row_index][_column_index] = _point;
    }
}

//给定帧周期内的运动变换矩阵，假设周期内是匀速运动，角度利用四元数球面差值，位移用线性差值
//然后将点都投影到点云开始时刻的坐标系下。
void DistortionAdjust::Adjust()
{
    Eigen::Quaterniond _q;
    double _time;
    _q =  scanMotion_.rotation(); //转为四元数
    _q.normalize();//归一化
    for(int _rowIndex=0; _rowIndex<vertical_scans_; _rowIndex++){
        for(int _columnIndex=0; _columnIndex<horizontal_scans_; _columnIndex++){
            if(number_mat_(_rowIndex, _columnIndex) <= 0){ //判断该位置是否有点
                continue;
            }
            //球面插值
            _time = laserCloudScans[_rowIndex][_columnIndex].intensity;
            Eigen::Quaterniond _q_point_last = Eigen::Quaterniond::Identity().slerp(_time, _q);
            Eigen::Vector3d _t_point_last =  _time * scanMotion_.translation();
            Eigen::Vector3d _point(laserCloudScans[_rowIndex][_columnIndex].x, 
                                    laserCloudScans[_rowIndex][_columnIndex].y, 
                                    laserCloudScans[_rowIndex][_columnIndex].z);
            //四元数在代码中可以直接乘向量，数学上是 qvq*
            Eigen::Vector3d un_point = _q_point_last * _point + _t_point_last;

            laserCloudScans[_rowIndex][_columnIndex].x = un_point.x();
            laserCloudScans[_rowIndex][_columnIndex].y = un_point.y();
            laserCloudScans[_rowIndex][_columnIndex].z = un_point.z();
            laserCloudScans[_rowIndex][_columnIndex].intensity = _rowIndex;
        }
    }
}

void DistortionAdjust::GeneratePointCloud(){
    for(int _rowIndex=0; _rowIndex<vertical_scans_; _rowIndex++){
        for(int _columnIndex=0; _columnIndex<horizontal_scans_; _columnIndex++){
            if(number_mat_(_rowIndex, _columnIndex) <= 0){ //如果没有点投影到该坐标
                continue;
            }
            size_t _index = _rowIndex * horizontal_scans_ + _columnIndex;
            adjust_point_cloud_ptr_->points.push_back(laserCloudScans[_rowIndex][_columnIndex]);
            float _range = sqrt(laserCloudScans[_rowIndex][_columnIndex].z * laserCloudScans[_rowIndex][_columnIndex].z);
            laserCloudScans[_rowIndex][_columnIndex].intensity = _range;
            image_point_cloud_ptr_->points[_index] = laserCloudScans[_rowIndex][_columnIndex];
        }
    }
}

bool DistortionAdjust::AdjustCloud(){
    ClearCloud();
    ProjectPointCloudToImage();
    Adjust();
    GeneratePointCloud();
    return true;
}

PointCloudData::point_cloud_ptr DistortionAdjust::GetAdjustImagePointCloud(){
    return image_point_cloud_ptr_;
}

PointCloudData::point_cloud_ptr DistortionAdjust::GetAdjustPointCloud(){
    return adjust_point_cloud_ptr_;
}

} 