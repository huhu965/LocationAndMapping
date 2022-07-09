/* 
 * @Author: Hu Ziwei
 * @Description:  地面优化提取特征点
 * 1.计算平滑度
 * 2.提取四类特征点
 * @Date: 2021-08-31 16:03:41
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-10 16:11:31
 */

#include "glog/logging.h"

#include "global_defination/global_defination.h"
#include "modules/feature_extract/go_feature_extract.hpp"
#include "modules/cloud_filter/voxel_filter.hpp"

namespace loam_frame{
GoFeatureExtract::GoFeatureExtract(const YAML::Node& config_node):
    image_point_cloud_ptr_(new PointCloudData::point_cloud()),
    ground_cloud_ptr_(new PointCloudData::point_cloud()),
    obstacle_cloud_ptr_(new PointCloudData::point_cloud()),
    segmented_cloud_ptr_(new PointCloudData::point_cloud()),
    corner_points_sharp_ptr_(new PointCloudData::point_cloud()),
    corner_points_less_sharp_ptr_(new PointCloudData::point_cloud()),
    surface_points_flat_ptr_(new PointCloudData::point_cloud()),
    surface_points_less_flat_ptr_(new PointCloudData::point_cloud()),
    surf_points_less_flat_scan_(new PointCloudData::point_cloud()),
    surf_points_less_flat_scan_downsize_(new PointCloudData::point_cloud()){
    
    InitWithConfig(config_node);

    ResetParameters();
}

bool GoFeatureExtract::InitWithConfig(const YAML::Node& config_node){
    std::string _lidar_param_path = config_node["lidar"]["path"].as<std::string>();
    std::string _config_file_path = WORK_SPACE_PATH + _lidar_param_path;
    YAML::Node _lidar_config_node = YAML::LoadFile(_config_file_path);
    InitLidarParams(_lidar_config_node);
    InitFeatureExtractParams(config_node["extract_params"]);
    InitFilter(config_node["filter"]);
    return true;
}

bool GoFeatureExtract::InitFilter(const YAML::Node& config_node){
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

bool GoFeatureExtract::InitLidarParams(const YAML::Node& config_node){
    std::cout << "初始化雷达参数" << std::endl;
    vertical_scans_ = config_node["num_vertical_scans"].as<int>();
    horizontal_scans_ = config_node["num_horizontal_scans"].as<int>();
    vertical_angle_bottom_ = config_node["vertical_angle_bottom"].as<float>();
    vertical_angle_top_ = config_node["vertical_angle_top"].as<float>();
    ground_scan_index_ = config_node["ground_scan_index"].as<int>();
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

bool GoFeatureExtract::InitFeatureExtractParams(const YAML::Node& config_node){
    std::cout << "初始化特征提取参数" << std::endl;
    segment_theta_ = config_node["segment_theta"].as<float>();    
    segment_valid_point_num_ = config_node["segment_valid_point_num"].as<int>();
    segment_valid_line_num_ = config_node["segment_valid_line_num"].as<int>();
    edge_threshold_ = config_node["edge_threshold"].as<float>(); 
    surf_threshold_ = config_node["surf_threshold"].as<float>();
    ground_theta_threshold_ = config_node["ground_theta_threshold"].as<double>();
    //转为弧度
    segment_theta_ = segment_theta_ * M_PI / float(180); 
    ground_theta_threshold_ = ground_theta_threshold_ * M_PI / float(180);
    return true;
}

void GoFeatureExtract::ResetParameters() {
    const size_t _cloud_size = vertical_scans_ * horizontal_scans_;
    PointCloudData::point _nanPoint;
    _nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    _nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    _nanPoint.z = std::numeric_limits<float>::quiet_NaN();

    image_point_cloud_ptr_->clear();
    ground_cloud_ptr_->clear();
    obstacle_cloud_ptr_->clear();
    segmented_cloud_ptr_->clear();

    corner_points_sharp_ptr_->clear();
    corner_points_less_sharp_ptr_->clear();
    surface_points_flat_ptr_->clear();
    surface_points_less_flat_ptr_->clear();
    
    segmented_cloud_ring_index_.clear();
    segmented_cloud_point_param_.clear();
    cloud_smoothness_.clear();

    label_count_ = 1;
    point_cloud_ptr_ = NULL;
    
    range_mat_.resize(vertical_scans_, horizontal_scans_);
    label_mat_.resize(vertical_scans_, horizontal_scans_);
    ground_mat_.resize(vertical_scans_, horizontal_scans_);
    intensity_mat_.resize(vertical_scans_, horizontal_scans_);

    range_mat_.fill(FLT_MAX); //设置为最大的浮点数
    label_mat_.setZero();
    ground_mat_.setZero();
    intensity_mat_.fill(-1);

    image_point_cloud_ptr_->points.resize(_cloud_size);
    std::fill(image_point_cloud_ptr_->points.begin(), image_point_cloud_ptr_->points.end(), _nanPoint);
}

bool GoFeatureExtract::PointCloudInput(const PointCloudData::point_cloud_ptr pointCloudPtr)
{
    point_cloud_ptr_ = pointCloudPtr;
    return true;
}

bool GoFeatureExtract::ProjectPointCloudToImage(){
    // range image projection
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

        range_mat_(_row_index, _column_index) = _range;
        // float _h_range = sqrt(_point.x * _point.x +_point.y * _point.y);
        // horizontal_range_mat_(_row_index, _column_index) = _h_range;
        // intensity_mat_(_row_index, _column_index) = _point.intensity;
        //整数表示第几行，小数表示第几列
        _point.intensity = _row_index;

        size_t _index = _column_index + _row_index * horizontal_scans_;
        image_point_cloud_ptr_->points[_index] = _point;
    }
    return true;
}

bool GoFeatureExtract::FindGroundPoint(){
    float _average_ground_altitude = 0;
    int _lower_average_ground_altitude_count = 0;
    int _ground_count = 0;
    int select_real_plan_point = 0;

    //标记出地面点
    // _ground_mat
    // -1, no valid info to check if ground of not 无效值
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (size_t j = 0; j < horizontal_scans_; ++j) {
        for (size_t i = 0; i < ground_scan_index_; ++i) {
            size_t _lowerInd = j + (i)*horizontal_scans_;
            size_t _upperInd = j + (i + 1) * horizontal_scans_;
            // no info to check, invalid points
            // if (intensity_mat_(i,j) == -1 || intensity_mat_(i+1,j) == -1) {
            //     ground_mat_(i, j) = -1;
            //     continue;
            // }

            float _dX = image_point_cloud_ptr_->points[_upperInd].x - image_point_cloud_ptr_->points[_lowerInd].x;
            float _dY = image_point_cloud_ptr_->points[_upperInd].y - image_point_cloud_ptr_->points[_lowerInd].y;
            float _dZ = image_point_cloud_ptr_->points[_upperInd].z - image_point_cloud_ptr_->points[_lowerInd].z;
            //求出同一列两点连线和地面的夹角
            float _vertical_angle = std::atan2(_dZ , sqrt(_dX * _dX + _dY * _dY));

            if ( (_vertical_angle - sensor_mount_angle_) <= ground_theta_threshold_){
                ground_mat_(i, j) = 1;
                ground_mat_(i + 1, j) = 1;
                //统计平均高度
                _average_ground_altitude += image_point_cloud_ptr_->points[_lowerInd].z;
                _ground_count++;
            }
        }
    }

    // 疑似地面点和无效点都打上标签，不参与后面的聚类
    for (size_t i = 0; i < vertical_scans_; ++i) {
        for (size_t j = 0; j < horizontal_scans_; ++j) {
            if (ground_mat_(i, j) == 1 || range_mat_(i, j) == FLT_MAX) {
                label_mat_(i, j) = GroundOrInvalid;
            }
        }
    }
    
    // 地面点和无效点都打上标签，不参与后面的聚类,这里增加了针对空旷场景平面点选择的优化
    // _average_ground_altitude = _average_ground_altitude / (float)_ground_count + 0.1;
    // std::cout<<_average_ground_altitude<<std::endl;
    // for (size_t i = 0; i < ground_scan_index_; ++i) {
    //     for (size_t j = 0; j < horizontal_scans_; ++j) {
    //         if(ground_mat_(i, j) == 1){
    //             if(image_point_cloud_ptr_->points[j + i*horizontal_scans_].z < _average_ground_altitude){
    //                 _lower_average_ground_altitude_count++;
    //             }
    //         }
    //     }
    // }
    // if(float(_lower_average_ground_altitude_count)/float(_ground_count) > 0.6)
    //     select_real_plan_point = -1;


    // for (size_t i = 0; i < vertical_scans_; ++i) {
    //     for (size_t j = 0; j < horizontal_scans_; ++j) {
    //         if(range_mat_(i, j) == FLT_MAX){
    //             label_mat_(i, j) = GroundOrInvalid;
    //             continue;
    //         }
    //         if (ground_mat_(i, j) == 1) {
    //             switch (select_real_plan_point){
    //                 case -1:  //选低于平均高度的点做为地面点
    //                     if(image_point_cloud_ptr_->points[j + i*horizontal_scans_].z < (_average_ground_altitude)){
    //                         label_mat_(i, j) = GroundOrInvalid;
    //                     }else{
    //                         ground_mat_(i, j) = 0;
    //                     }
    //                     break;
    //                 case 0:  //选所有可能的点做地面点
    //                     label_mat_(i, j) = GroundOrInvalid;
    //                     break;
                    
    //                 default:
    //                     break;
    //             }
    //         }
    //     }
    // }
    return true;
}

void GoFeatureExtract::LabelComponents(int row, int col){
    const float segmentThetaThreshold = tan(segment_theta_);

    std::vector<bool> lineCountFlag(vertical_scans_, false);
    const size_t cloud_size = vertical_scans_ * horizontal_scans_;
    using Coord2D = Eigen::Vector2i;
    boost::circular_buffer<Coord2D> queue(cloud_size);
    boost::circular_buffer<Coord2D> all_pushed(cloud_size);

    queue.push_back({ row,col } ); //存放搜索中的临时可能点
    all_pushed.push_back({ row,col } ); //存放一类中所有被挑选出的点

    const Coord2D neighborIterator[4] = { {0, -1}, {-1, 0}, {1, 0}, {0, 1} };
    //广度优先遍历 找完相邻的一类
    while (queue.size() > 0) {
        // Pop point
        Coord2D fromInd = queue.front();
        queue.pop_front();
        //x表示列，y表示行
        // Mark popped point
        label_mat_(fromInd.x(), fromInd.y()) = label_count_;  //标记为第几类
        // Loop through all the neighboring grids of popped grid

        for (const auto& iter : neighborIterator) {
            // new index
            int thisIndX = fromInd.x() + iter.x();
            int thisIndY = fromInd.y() + iter.y();
            // index should be within the boundary
            if (thisIndX < 0 || thisIndX >= vertical_scans_){
                continue;
            }
            // at range image margin (left or right side) 处理边界问题。
            if (thisIndY < 0){
                thisIndY = horizontal_scans_ - 1;
            }
            if (thisIndY >= horizontal_scans_){
                thisIndY = 0;
            }
            // prevent infinite loop (caused by put already examined point back)
            if (label_mat_(thisIndX, thisIndY) != 0){  //如果不为0，说明碰见边界了
                continue;
            }

            float d1 = std::max(range_mat_(fromInd.x(), fromInd.y()),
                            range_mat_(thisIndX, thisIndY));
            float d2 = std::min(range_mat_(fromInd.x(), fromInd.y()),
                            range_mat_(thisIndX, thisIndY));
            //判断是用水平分辨率夹角还是垂直分辨率夹角
            //x为0，垂直方向没变，变的是水平，所以用水平分别率
            float alpha = (iter.x() == 0) ? horizontal_angle_resolution_ : vertical_angle_resolution_;
            if(iter.x() == 0){//水平的处理
                float _range_diff = sqrt(d2 * sin(alpha)*d2 * sin(alpha)
                                        + (d1 - d2 * cos(alpha))*(d1 - d2 * cos(alpha)));
                if(_range_diff < 0.4 && _range_diff > 0.03){
                    queue.push_back( {thisIndX, thisIndY } );

                    label_mat_(thisIndX, thisIndY) = label_count_;
                    lineCountFlag[thisIndX] = true;

                    all_pushed.push_back(  {thisIndX, thisIndY } );
                }
            }else{//垂直的处理
                float tang = d2 * sin(alpha) / (d1 - d2 * cos(alpha));//然后求一手夹角
                //两点之间的连线和较长的点形成的线之间 夹角大于阈值角，就认为属于同一个物体
                if (tang > segmentThetaThreshold) {
                    queue.push_back( {thisIndX, thisIndY } );

                    label_mat_(thisIndX, thisIndY) = label_count_;
                    lineCountFlag[thisIndX] = true;

                    all_pushed.push_back(  {thisIndX, thisIndY } );
                }
            }
        }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (all_pushed.size() >= 30){
        feasibleSegment = true;
    }
    //这里是为了找出扫描到的垂直线条，如果一类大于5个点，且起码分别属于三条直线
    //就放为一类，可以更好的提高精准度和细节的聚类
    else if (all_pushed.size() >= segment_valid_point_num_) { 
        int lineCount = 0;
        for (size_t i = 0; i < vertical_scans_; ++i) {
            if (lineCountFlag[i] == true) 
                ++lineCount;
        }
        if (lineCount >= segment_valid_line_num_) 
            feasibleSegment = true;
    }

    // 如果聚出来一个有效类，就保存
    //否则就标记为999999，来表示杂点，没有用处
    if (feasibleSegment == true) {
        ++label_count_;
    } else {  // segment is invalid, mark these points
        for (size_t i = 0; i < all_pushed.size(); ++i) {
            label_mat_(all_pushed[i].x(), all_pushed[i].y()) = 999999;
        }
    }
}

bool GoFeatureExtract::CloudSegmentation(){
    // 聚类分割的过程
    for (size_t i = 0; i < vertical_scans_; ++i){
        for (size_t j = 0; j < horizontal_scans_; ++j){
            if (label_mat_(i, j) == 0){
                LabelComponents(i, j);
            }
        }
    }
    return true;
}

bool GoFeatureExtract::SaveDifferentClouds(){
    int _sizeOfSegCloud = 0;
    for (size_t i = 0; i < vertical_scans_; ++i) {
        RingIndex _ring_index;
        _ring_index.start_index = _sizeOfSegCloud  + 5;
        for (size_t j = 0; j < horizontal_scans_; ++j) {
            //保存聚类后的障碍物点
            if ( (label_mat_(i, j) > 0 && label_mat_(i, j) != 999999) ) {
                obstacle_cloud_ptr_->push_back(image_point_cloud_ptr_->points[j + i * horizontal_scans_]);
            }
            else if (ground_mat_(i, j) == 1){  //保存地面点
                ground_cloud_ptr_->push_back(image_point_cloud_ptr_->points[j + i * horizontal_scans_]);
            }
            else
                continue;

            segmented_cloud_ptr_->push_back(image_point_cloud_ptr_->points[j + i * horizontal_scans_]);
            SegmentedPointParam _point_param = {.is_ground = (ground_mat_(i,j) == 1), 
                                                .is_neighbor_picked = false, 
                                                .column_index = j, 
                                                .row_index = i, 
                                                .smothness = -1};
            segmented_cloud_point_param_.push_back(_point_param);
            _sizeOfSegCloud ++;
        }
        _ring_index.end_index = _sizeOfSegCloud -1 - 5;
        segmented_cloud_ring_index_.push_back(_ring_index);
    }
    return true;
}

bool GoFeatureExtract::CalculateSmoothness(){
    int _cloudSize = segmented_cloud_ptr_->points.size();
    cloud_smoothness_.resize(_cloudSize);
    //平滑度计算公式|| j=(j!=i | i-5,i+5)∑(xj-xi) ||
    //这是比较合理的，假设是平面的话，左边和右边的差值求和之后基本为0
    //如果不是平面的话，左边和右边求和之后，因为有某个方向相同，所以求出的差值会大
    //lego-loam中用距离远近也有道理，但是他忽略了一个大问题。
    //lego-loam中用的是分割后的点云，也就是说点云中两个连续属于不同边缘的点，可能在同一个圆上，直线距离差距很大，他们中间是没有点的
    //用距离的话，会发现他们差值为0，就会认为是平面点，但实际上他们都是边缘点。
    //所以这里选用了loam中的平滑度计算。
    for (int i = 5; i < _cloudSize - 5; i++) {
        float _diffX = segmented_cloud_ptr_->points[i - 5].x 
                    + segmented_cloud_ptr_->points[i - 4].x 
                    + segmented_cloud_ptr_->points[i - 3].x 
                    + segmented_cloud_ptr_->points[i - 2].x 
                    + segmented_cloud_ptr_->points[i - 1].x 
                    - 10 * segmented_cloud_ptr_->points[i].x 
                    + segmented_cloud_ptr_->points[i + 1].x 
                    + segmented_cloud_ptr_->points[i + 2].x 
                    + segmented_cloud_ptr_->points[i + 3].x 
                    + segmented_cloud_ptr_->points[i + 4].x 
                    + segmented_cloud_ptr_->points[i + 5].x;
                    
        float _diffY = segmented_cloud_ptr_->points[i - 5].y 
                    + segmented_cloud_ptr_->points[i - 4].y 
                    + segmented_cloud_ptr_->points[i - 3].y 
                    + segmented_cloud_ptr_->points[i - 2].y 
                    + segmented_cloud_ptr_->points[i - 1].y 
                    - 10 * segmented_cloud_ptr_->points[i].y 
                    + segmented_cloud_ptr_->points[i + 1].y 
                    + segmented_cloud_ptr_->points[i + 2].y 
                    + segmented_cloud_ptr_->points[i + 3].y 
                    + segmented_cloud_ptr_->points[i + 4].y 
                    + segmented_cloud_ptr_->points[i + 5].y;
                    
        float _diffZ = segmented_cloud_ptr_->points[i - 5].z 
                    + segmented_cloud_ptr_->points[i - 4].z 
                    + segmented_cloud_ptr_->points[i - 3].z 
                    + segmented_cloud_ptr_->points[i - 2].z 
                    + segmented_cloud_ptr_->points[i - 1].z 
                    - 10 * segmented_cloud_ptr_->points[i].z 
                    + segmented_cloud_ptr_->points[i + 1].z 
                    + segmented_cloud_ptr_->points[i + 2].z 
                    + segmented_cloud_ptr_->points[i + 3].z 
                    + segmented_cloud_ptr_->points[i + 4].z 
                    + segmented_cloud_ptr_->points[i + 5].z;

        segmented_cloud_point_param_[i].smothness = _diffX*_diffX + _diffY*_diffY + _diffZ*_diffZ;
        PointSmoothness _point_smoothness = {.value =segmented_cloud_point_param_[i].smothness, 
                                             .index = i};
        cloud_smoothness_[i]=_point_smoothness;
    }  
    return true; 
}

bool GoFeatureExtract::ExtractFeatures(){
    std::vector<int> _cloud_lable; //用于标记点是平面点还是角点
    _cloud_lable.resize(segmented_cloud_ptr_->points.size());
    std::fill(_cloud_lable.begin(), _cloud_lable.end(), NormalPoint);

    for (int i = 0; i < vertical_scans_; i++) {
        surf_points_less_flat_scan_->clear();
        for (int j = 0; j < 6; j++) {  //每行划分为6个部分，然后计算出每部分的开始和结束索引
            // sp = start + (end-start)j/6
            //    = (start*(6-j) + end*j)/6 
            int sp = (segmented_cloud_ring_index_[i].start_index * (6 - j) 
                    + segmented_cloud_ring_index_[i].end_index * j) / 6;
            //ep和上面sp一样
            int ep = (segmented_cloud_ring_index_[i].start_index * (5 - j) 
                    + segmented_cloud_ring_index_[i].end_index * (j + 1)) / 6 -1;
            // std::cout<<"sp："<<sp;
            // std::cout<<"  ep："<<ep<<std::endl;
            if (sp >= ep) continue;

            std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep,
                        by_value());//每个部分按照平滑度排序
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {    //挑选角点
                int _index = cloud_smoothness_[k].index;
                if ( !segmented_cloud_point_param_[_index].is_neighbor_picked
                    && segmented_cloud_point_param_[_index].smothness > edge_threshold_
                    //！！！！！！！！！！下面注释掉的话，就是说角点也可以在地面点提取
                    && !segmented_cloud_point_param_[_index].is_ground 
                    ){
                    //如果自己没被挑选，同时周围没有点被挑选，平滑度大于可以判定为边特征的阈值，不是地面点
                    largestPickedNum++;
                    if (largestPickedNum <= 4) {
                        _cloud_lable[_index] = SharpPoint;
                        corner_points_sharp_ptr_->push_back(segmented_cloud_ptr_->points[_index]);
                        corner_points_less_sharp_ptr_->push_back(segmented_cloud_ptr_->points[_index]);
                    } else if (largestPickedNum <= 20) {
                        _cloud_lable[_index] = LessSharpPoint;
                        corner_points_less_sharp_ptr_->push_back(segmented_cloud_ptr_->points[_index]);
                    } else {
                        break; //超出20点就直接退出，不用挑选了。
                    }
                    segmented_cloud_point_param_[_index].is_neighbor_picked = true;  //标记自己或者周围有点被挑选过了
                    //前后 各10个索引内的点都不被挑选
                    for (int l = 1; l <= 5; l++) {
                        if( _index + l >= segmented_cloud_point_param_.size() ) {
                            continue;
                        }
                        int _columnDiff =std::abs(int(segmented_cloud_point_param_[_index + l].column_index 
                                                    - segmented_cloud_point_param_[_index + l - 1].column_index));
                        if (_columnDiff > 10) 
                            break; //如果列坐标差10以上，说明不临近，就可以直接退了
                        //否则就标记有临近点被挑选了。后面就直接跳过该点
                        segmented_cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                    for (int l = -1; l >= -5; l--) {
                        if( _index + l < 0 ) {
                            continue;
                        }
                        int _columnDiff =std::abs(int(segmented_cloud_point_param_[_index + l].column_index 
                                                - segmented_cloud_point_param_[_index + l + 1].column_index));
                        if (_columnDiff > 10)
                            break;
                        segmented_cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                }
            }

            //挑选平面点
            int smallestPickedNum = 0; 
            for (int k = sp; k <= ep; k++) {
                int _index = cloud_smoothness_[k].index;
                if (!segmented_cloud_point_param_[_index].is_neighbor_picked
                    && segmented_cloud_point_param_[_index].smothness < surf_threshold_
                    //！！！！！！！！下面注释掉的话就是可以在障碍物上选平面点
                    && segmented_cloud_point_param_[_index].is_ground
                    ){

                    smallestPickedNum++;
                    if (smallestPickedNum <= 4) {
                        _cloud_lable[_index] = FlatPoint;
                        surface_points_flat_ptr_->push_back(segmented_cloud_ptr_->points[_index]);
                    }
                    // else if(smallestPickedNum <= 20){
                    //     surf_points_less_flat_scan_->push_back(segmented_cloud_ptr_->points[k]);
                    // }
                    else{
                        break;
                    }

                    segmented_cloud_point_param_[_index].is_neighbor_picked = true;  //标记自己或者周围有点被挑选过了
                    for (int l = 1; l <= 5; l++) {
                        if( _index + l >= segmented_cloud_point_param_.size() ) {
                            continue;
                        }
                        int _columnDiff =std::abs(int(segmented_cloud_point_param_[_index + l].column_index 
                                                    - segmented_cloud_point_param_[_index + l - 1].column_index));
                        if (_columnDiff > 10) 
                            break;

                        segmented_cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                    for (int l = -1; l >= -5; l--) {
                        if (_index + l < 0) {
                            continue;
                        }
                        int _columnDiff =std::abs(int(segmented_cloud_point_param_[_index + l].column_index 
                                                    - segmented_cloud_point_param_[_index + l + 1].column_index));
                        if (_columnDiff > 10) 
                            break;

                        segmented_cloud_point_param_[_index + l].is_neighbor_picked = true;
                    }
                }
            }
            // 保存其他点作为没那么平的平面点
            for (int k = sp; k <= ep; k++) {
                if (_cloud_lable[k] >= NormalPoint) {
                    surf_points_less_flat_scan_->push_back(segmented_cloud_ptr_->points[k]);
                }
            }
        }
        surf_points_less_flat_scan_downsize_->clear();
        downSizeFilterPtr_->Filter(surf_points_less_flat_scan_, surf_points_less_flat_scan_downsize_);
        *surface_points_less_flat_ptr_ += *surf_points_less_flat_scan_downsize_;//保存降采样之后的点云
    }
    return true;
}

bool GoFeatureExtract::RunOnceExtractFeatures(){
    if(point_cloud_ptr_ == NULL){
        LOG(ERROR) << "未输入点云";
        std::cout<<"未输入点云"<<std::endl;
        return false;
    }
    ProjectPointCloudToImage(); //投影到2d
    FindGroundPoint();//分割地面点
    CloudSegmentation();//非地面点聚类
    SaveDifferentClouds();//保存聚类后的点，地面点，两种点的集合
    CalculateSmoothness();//计算所有点的平滑度
    ExtractFeatures();//提取特征点
    
    return true;
}

PointCloudData::point_cloud_ptr GoFeatureExtract:: GETImageCloud(){
    return image_point_cloud_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETGroundCloud(){
    return ground_cloud_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETObstacleCloud(){
    return obstacle_cloud_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETSegmentedCloud(){
    return segmented_cloud_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETCornerPoints(){
    return corner_points_sharp_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETLessSharpCornerPoints(){
    return corner_points_less_sharp_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETSurfacePoints(){
    return surface_points_flat_ptr_;
}

PointCloudData::point_cloud_ptr GoFeatureExtract::GETLessFlatSurfacePoints(){
    return surface_points_less_flat_ptr_;
}

}