/* 
 * @Author: Hu Ziwei
 * @Description: 提取特征点
 * 1.计算平滑度
 * 2.提取四类特征点
 * @Date: 2021-08-31 16:03:41
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-10 14:47:33
 */

#include "glog/logging.h"

#include "global_defination/global_defination.h"
#include "modules/feature_extract/normal_feature_extract.hpp"
#include "modules/cloud_filter/voxel_filter.hpp"

namespace loam_frame{
NormalFeatureExtract::NormalFeatureExtract(const YAML::Node& config_node):
    image_point_cloud_ptr_(new PointCloudData::point_cloud()),
    corner_points_sharp_ptr_(new PointCloudData::point_cloud()),
    corner_points_less_sharp_ptr_(new PointCloudData::point_cloud()),
    surface_points_flat_ptr_(new PointCloudData::point_cloud()),
    surface_points_less_flat_ptr_(new PointCloudData::point_cloud()),
    surf_points_less_flat_scan_(new PointCloudData::point_cloud()),
    surf_points_less_flat_scan_downsize_(new PointCloudData::point_cloud()){
    
    InitWithConfig(config_node);

    ResetParameters();
}

bool NormalFeatureExtract::InitWithConfig(const YAML::Node& config_node){
    std::string _lidar_param_path = config_node["lidar"]["path"].as<std::string>();
    std::string _config_file_path = WORK_SPACE_PATH + _lidar_param_path;
    YAML::Node _lidar_config_node = YAML::LoadFile(_config_file_path);
    InitLidarParams(_lidar_config_node);
    InitFeatureExtractParams(config_node["extract_params"]);
    InitFilter(config_node["filter"]);
    return true;
}

bool NormalFeatureExtract::InitFilter(const YAML::Node& config_node){
    std::string filter_method = config_node["method"].as<std::string>();
    std::cout << "特征提取选择的滤波器方式为：" << filter_method << std::endl;
    if(filter_method == "voxel_filter"){
        downSizeFilterPtr_ = std::make_shared<VoxelFilter>(config_node[filter_method]);
    }else{
        std::cout<< "没找到与 " << filter_method << " 相对应的滤波方式!"<< std::endl;
        LOG(ERROR) << "没找到与 " << filter_method << " 相对应的滤波方式!";
        return false;
    }
    
    return true;
}

bool NormalFeatureExtract::InitLidarParams(const YAML::Node& config_node){
    std::cout << "初始化雷达参数" << std::endl;
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

bool NormalFeatureExtract::InitFeatureExtractParams(const YAML::Node& config_node){
    std::cout << "初始化特征提取参数" << std::endl;
    edge_threshold_ = config_node["edge_threshold"].as<float>(); 
    surf_threshold_ = config_node["surf_threshold"].as<float>();
    return true;
}

void NormalFeatureExtract::ResetParameters() {
    const size_t _cloud_size = vertical_scans_ * horizontal_scans_;
    PointCloudData::point _nanPoint;
    _nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    _nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    _nanPoint.z = std::numeric_limits<float>::quiet_NaN();

    image_point_cloud_ptr_->clear();

    corner_points_sharp_ptr_->clear();
    corner_points_less_sharp_ptr_->clear();
    surface_points_flat_ptr_->clear();
    surface_points_less_flat_ptr_->clear();
    
    cloud_ring_index_.clear();
    cloud_point_param_.clear();
    cloud_smoothness_.clear();

    point_cloud_ptr_ = NULL;
    
    range_mat_.resize(vertical_scans_, horizontal_scans_);
    intensity_mat_.resize(vertical_scans_, horizontal_scans_);
    valid_mat_.resize(vertical_scans_, horizontal_scans_);

    range_mat_.fill(FLT_MAX); //设置为最大的浮点数
    valid_mat_.fill(-1);
    intensity_mat_.fill(-1);

    image_point_cloud_ptr_->points.resize(_cloud_size);
    std::fill(image_point_cloud_ptr_->points.begin(), image_point_cloud_ptr_->points.end(), _nanPoint);
}

bool NormalFeatureExtract::PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr)
{
    point_cloud_ptr_ = pointCloudPtr;
    return true;
}

bool NormalFeatureExtract::ProjectPointCloudToImage(){
    // range image projection
    //先投影成2维形式
    const size_t cloudSize = point_cloud_ptr_->points.size();
    for (size_t i = 0; i < cloudSize; ++i) {
        PointCloudData::point _point = point_cloud_ptr_->points[i];

        float _range = sqrt(_point.x * _point.x +
                        _point.y * _point.y +
                        _point.z * _point.z);

        if (_range < 0.1){  //如果距离雷达的距离过近，就直接筛掉
            continue;
        }

        // 求垂直角度
        float _vertical_angle = std::asin(_point.z / _range);
        
        int _row_index = round((_vertical_angle - vertical_angle_bottom_) / vertical_angle_resolution_);
        if (_row_index < 0 || _row_index >= vertical_scans_) {
            continue;
        }
        //加负号是因为雷达扫描是顺时针扫描，所以加个负号
        float _horizon_angle = -std::atan2(_point.x, _point.y);
        if(_horizon_angle < 0){
            _horizon_angle += 2 * M_PI;
        }
        //x正方向为起点，顺时针为正方向
        //x指向雷达的正前方
        int _column_index = round(_horizon_angle / horizontal_angle_resolution_);

        if (_column_index < 0 || _column_index >= horizontal_scans_){
            continue;
        }

        //如果该投影点已经放了一个值，选用其中离雷达更近的点
        //这里是对loam中同一线遮挡障碍物的判断，不过好像没啥用
        if(valid_mat_(_row_index, _column_index) > 0){
            if(range_mat_(_row_index, _column_index) < _range){
                continue;
            }
        }

        range_mat_(_row_index, _column_index) = _range;
        valid_mat_(_row_index, _column_index) = 1;
        //整数表示第几行，小数表示第几列
        _point.intensity = _row_index;

        size_t _index = _column_index + _row_index * horizontal_scans_;
        image_point_cloud_ptr_->points[_index] = _point;
    }
    //从2维图像中去掉无效点，同时记录每行扫描点的开始和结束
    int _sizeOfSegCloud = 0;
    PointCloudData::point_cloud_ptr _cloudPtr(new PointCloudData::point_cloud());
    for (size_t i = 0; i < vertical_scans_; ++i) {
        RingIndex _ring_index;
        _ring_index.start_index = _sizeOfSegCloud  + 5;
        for (size_t j = 0; j < horizontal_scans_; ++j) {
            if(valid_mat_(i, j) < 0)
                continue;
            _cloudPtr->push_back(image_point_cloud_ptr_->points[j + i * horizontal_scans_]);
            SegmentedPointParam _point_param = {.is_ground = false, 
                                                .is_neighbor_picked = false, 
                                                .column_index = j, 
                                                .row_index = i, 
                                                .smothness = -1};
            cloud_point_param_.push_back(_point_param);
            _sizeOfSegCloud ++;
        }
        _ring_index.end_index = _sizeOfSegCloud -1 - 5;
        cloud_ring_index_.push_back(_ring_index);
    }
    image_point_cloud_ptr_ = _cloudPtr;
    return true;
}

bool NormalFeatureExtract::CalculateSmoothness(){
    int _cloudSize = image_point_cloud_ptr_->points.size();
    cloud_smoothness_.resize(_cloudSize);
    //平滑度计算公式|| j=(j!=i | i-5,i+5)∑(xj-xi) ||
    //这是比较合理的，假设是平面的话，左边和右边的差值求和之后基本为0
    //如果不是平面的话，左边和右边求和之后，因为有某个方向相同，所以求出的差值会大
    //lego-loam中用距离远近也有道理，但是他忽略了一个大问题。
    //lego-loam中用的是分割后的点云，也就是说点云中两个连续属于不同边缘的点，可能在同一个圆上，直线距离差距很大，他们中间是没有点的
    //用距离的话，会发现他们差值为0，就会认为是平面点，但实际上他们都是边缘点。
    //所以这里选用了loam中的平滑度计算。
    for (int i = 5; i < _cloudSize - 5; i++) {
        float _diffX = image_point_cloud_ptr_->points[i - 5].x 
                    + image_point_cloud_ptr_->points[i - 4].x 
                    + image_point_cloud_ptr_->points[i - 3].x 
                    + image_point_cloud_ptr_->points[i - 2].x 
                    + image_point_cloud_ptr_->points[i - 1].x 
                    - 10 * image_point_cloud_ptr_->points[i].x 
                    + image_point_cloud_ptr_->points[i + 1].x 
                    + image_point_cloud_ptr_->points[i + 2].x 
                    + image_point_cloud_ptr_->points[i + 3].x 
                    + image_point_cloud_ptr_->points[i + 4].x 
                    + image_point_cloud_ptr_->points[i + 5].x;
                    
        float _diffY = image_point_cloud_ptr_->points[i - 5].y 
                    + image_point_cloud_ptr_->points[i - 4].y 
                    + image_point_cloud_ptr_->points[i - 3].y 
                    + image_point_cloud_ptr_->points[i - 2].y 
                    + image_point_cloud_ptr_->points[i - 1].y 
                    - 10 * image_point_cloud_ptr_->points[i].y 
                    + image_point_cloud_ptr_->points[i + 1].y 
                    + image_point_cloud_ptr_->points[i + 2].y 
                    + image_point_cloud_ptr_->points[i + 3].y 
                    + image_point_cloud_ptr_->points[i + 4].y 
                    + image_point_cloud_ptr_->points[i + 5].y;
                    
        float _diffZ = image_point_cloud_ptr_->points[i - 5].z 
                    + image_point_cloud_ptr_->points[i - 4].z 
                    + image_point_cloud_ptr_->points[i - 3].z 
                    + image_point_cloud_ptr_->points[i - 2].z 
                    + image_point_cloud_ptr_->points[i - 1].z 
                    - 10 * image_point_cloud_ptr_->points[i].z 
                    + image_point_cloud_ptr_->points[i + 1].z 
                    + image_point_cloud_ptr_->points[i + 2].z 
                    + image_point_cloud_ptr_->points[i + 3].z 
                    + image_point_cloud_ptr_->points[i + 4].z 
                    + image_point_cloud_ptr_->points[i + 5].z;

        cloud_point_param_[i].smothness = _diffX*_diffX + _diffY*_diffY + _diffZ*_diffZ;
        PointSmoothness _point_smoothness = {.value =cloud_point_param_[i].smothness, 
                                             .index = i};
        cloud_smoothness_[i]=_point_smoothness;
    }  
    return true; 
}

bool NormalFeatureExtract::ExtractFeatures(){
    std::vector<int> _cloud_lable; //用于标记点是平面点还是角点
    _cloud_lable.resize(image_point_cloud_ptr_->points.size());
    std::fill(_cloud_lable.begin(), _cloud_lable.end(), NormalPoint);

    for (int i = 0; i < vertical_scans_; i++) {
        surf_points_less_flat_scan_->clear();
        for (int j = 0; j < 6; j++) {  //每行划分为6个部分，然后计算出每部分的开始和结束索引
            // sp = start + (end-start)j/6
            //    = (start*(6-j) + end*j)/6 
            int sp = (cloud_ring_index_[i].start_index * (6 - j) + cloud_ring_index_[i].end_index * j) / 6;
            //ep和上面sp一样
            int ep = (cloud_ring_index_[i].start_index * (5 - j) + cloud_ring_index_[i].end_index * (j + 1)) / 6 -1;
            if (sp >= ep) continue;

            std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep,
                        by_value());//每个部分按照平滑度排序
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {    //挑选角点
                int _index = cloud_smoothness_[k].index;
                if ( !cloud_point_param_[_index].is_neighbor_picked
                    && cloud_point_param_[_index].smothness > edge_threshold_){
                    //如果自己没被挑选，同时周围没有点被挑选，平滑度大于可以判定为边特征的阈值，不是地面点
                    largestPickedNum++;
                    if (largestPickedNum <= 2) {
                        _cloud_lable[_index] = SharpPoint;
                        corner_points_sharp_ptr_->push_back(image_point_cloud_ptr_->points[_index]);
                        corner_points_less_sharp_ptr_->push_back(image_point_cloud_ptr_->points[_index]);
                    } else if (largestPickedNum <= 20) {
                        _cloud_lable[_index] = LessSharpPoint;
                        corner_points_less_sharp_ptr_->push_back(image_point_cloud_ptr_->points[_index]);
                    } else {
                        break; //超出20点就直接退出，不用挑选了。
                    }

                    cloud_point_param_[_index].is_neighbor_picked = true;  //标记自己或者周围有点被挑选过了
                    //前后 各10个索引内的点都要判断是否需要标记
                    for (int l = 1; l <= 5; l++) {
                        //如果列坐标差10以上，说明不临近，就可以直接退了
                        int _columnDiff =std::abs(int(cloud_point_param_[_index + l].column_index - cloud_point_param_[_index + l - 1].column_index));
                        if (_columnDiff > 10){
                            break;
                        } 
                        //如果被挑选点前后的5个点，每两个临近点之间的间距都小于0.05，就也标记为有相邻点被挑选过，
                        float diffX = image_point_cloud_ptr_->points[_index + l].x - image_point_cloud_ptr_->points[_index + l - 1].x;
                        float diffY = image_point_cloud_ptr_->points[_index + l].y - image_point_cloud_ptr_->points[_index + l - 1].y;
                        float diffZ = image_point_cloud_ptr_->points[_index + l].z - image_point_cloud_ptr_->points[_index + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                            break;
                        }
                        //否则就标记有临近点被挑选了。后面就直接跳过该点
                        cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int _columnDiff =std::abs(int(cloud_point_param_[_index + l].column_index - cloud_point_param_[_index + l + 1].column_index));
                        if (_columnDiff > 10)
                            break;
                        //如果被挑选点前后的5个点，每两个临近点之间的间距都小于0.05，就也标记为有相邻点被挑选过，
                        float diffX = image_point_cloud_ptr_->points[_index + l].x - image_point_cloud_ptr_->points[_index + l - 1].x;
                        float diffY = image_point_cloud_ptr_->points[_index + l].y - image_point_cloud_ptr_->points[_index + l - 1].y;
                        float diffZ = image_point_cloud_ptr_->points[_index + l].z - image_point_cloud_ptr_->points[_index + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                            break;
                        }
                        cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                }
            }

            //挑选平面点
            int smallestPickedNum = 0; 
            for (int k = sp; k <= ep; k++) {
                int _index = cloud_smoothness_[k].index;
                if (!cloud_point_param_[_index].is_neighbor_picked
                    && cloud_point_param_[_index].smothness < surf_threshold_){

                    smallestPickedNum++;
                    if (smallestPickedNum <= 4) {
                        _cloud_lable[_index] = FlatPoint;
                        surface_points_flat_ptr_->push_back(image_point_cloud_ptr_->points[_index]);
                    }
                    else{
                        break;
                    }

                    cloud_point_param_[_index].is_neighbor_picked = true;  //标记自己或者周围有点被挑选过了
                    for (int l = 1; l <= 5; l++) {
                        if( _index + l >= cloud_point_param_.size() ) {
                            continue;
                        }
                        int _columnDiff =std::abs(int(cloud_point_param_[_index + l].column_index - cloud_point_param_[_index + l - 1].column_index));
                        if (_columnDiff > 10) 
                            break;
                        //如果被挑选点前后的5个点，每两个临近点之间的间距都小于0.05，就也标记为有相邻点被挑选过，
                        float diffX = image_point_cloud_ptr_->points[_index + l].x - image_point_cloud_ptr_->points[_index + l - 1].x;
                        float diffY = image_point_cloud_ptr_->points[_index + l].y - image_point_cloud_ptr_->points[_index + l - 1].y;
                        float diffZ = image_point_cloud_ptr_->points[_index + l].z - image_point_cloud_ptr_->points[_index + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                            break;
                        }
                        cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                    for (int l = -1; l >= -5; l--) {
                        if (_index + l < 0) {
                            continue;
                        }
                        int _columnDiff =std::abs(int(cloud_point_param_[_index + l].column_index - cloud_point_param_[_index + l + 1].column_index));
                        if (_columnDiff > 10) 
                            break;
                        //如果被挑选点前后的5个点，每两个临近点之间的间距都小于0.05，就也标记为有相邻点被挑选过，
                        float diffX = image_point_cloud_ptr_->points[_index + l].x - image_point_cloud_ptr_->points[_index + l - 1].x;
                        float diffY = image_point_cloud_ptr_->points[_index + l].y - image_point_cloud_ptr_->points[_index + l - 1].y;
                        float diffZ = image_point_cloud_ptr_->points[_index + l].z - image_point_cloud_ptr_->points[_index + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                            break;
                        }
                        cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                }
            }
            // 保存其他点作为没那么平的平面点
            for (int k = sp; k <= ep; k++) {
                if (_cloud_lable[k] >= NormalPoint) {
                    surf_points_less_flat_scan_->push_back(image_point_cloud_ptr_->points[k]);
                }
            }
        }
        surf_points_less_flat_scan_downsize_->clear();
        downSizeFilterPtr_->Filter(surf_points_less_flat_scan_, surf_points_less_flat_scan_downsize_);
        *surface_points_less_flat_ptr_ += *surf_points_less_flat_scan_downsize_;//保存降采样之后的点云
    }
    return true;
}

bool NormalFeatureExtract::RunOnceExtractFeatures(){
    if(point_cloud_ptr_ == NULL){
        LOG(ERROR) << "未输入点云";
        std::cout<<"未输入点云"<<std::endl;
        return false;
    }
    ProjectPointCloudToImage(); //投影到2d
    CalculateSmoothness();//计算所有点的平滑度
    ExtractFeatures();//提取特征点
    
    return true;
}

PointCloudData::point_cloud_ptr NormalFeatureExtract:: GETImageCloud(){
    return image_point_cloud_ptr_;
}

PointCloudData::point_cloud_ptr NormalFeatureExtract::GETCornerPoints(){
    return corner_points_sharp_ptr_;
}

PointCloudData::point_cloud_ptr NormalFeatureExtract::GETLessSharpCornerPoints(){
    return corner_points_less_sharp_ptr_;
}

PointCloudData::point_cloud_ptr NormalFeatureExtract::GETSurfacePoints(){
    return surface_points_flat_ptr_;
}

PointCloudData::point_cloud_ptr NormalFeatureExtract::GETLessFlatSurfacePoints(){
    return surface_points_less_flat_ptr_;
}

}