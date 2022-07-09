/* 
 * @Author: Hu Ziwei
 * @Description: 寻找特征点附近最近的几个点
 * 角特征对应的是一条直线，所以根据当前特征点，和不在同一线的最近点共同拟合出一条直线，作为匹配的特征线
 * 面特征对应的是一个平面，所以根据当前特征点，在同一线与不同线各找一个最近点，然后三点拟合出一个平面，作为匹配的特征面
 * 参考论文:2014 ，LOAM 代码，ALOAM
 * @Date: 2021-09-08 15:43:20
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-30 19:48:05
 */

#include "modules/registration/ICP/aloam_registration.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"

namespace loam_frame{
AloamRegistration::AloamRegistration(const YAML::Node& configNode):
    kdtreeTargetCornerCloudPtr_(new pcl::KdTreeFLANN<PointCloudData::point>()),
    kdtreeTargetSurfaceCloudPtr_(new pcl::KdTreeFLANN<PointCloudData::point>()){
    InitWithConfig(configNode);
}

bool AloamRegistration::InitWithConfig(const YAML::Node& configNode){
    distance_square_threshold_ = configNode["distance_square_threshold"].as<double>();
    nearby_scan_ = configNode["nearby_scan"].as<int>();
    return true;
}

bool AloamRegistration::PointCloudInput(const PointCloudData::point_cloud_ptr sourceCornerCloud,
                                    const PointCloudData::point_cloud_ptr targetCornerCloud,
                                    const PointCloudData::point_cloud_ptr sourceSurfaceCloud,
                                    const PointCloudData::point_cloud_ptr targetSurfaceCloud){
    sourceCornerCloudPtr_ = sourceCornerCloud;    
    targetCornerCloudPtr_ = targetCornerCloud;
    sourceSurfaceCloudPtr_ = sourceSurfaceCloud;
    targetSurfaceCloudPtr_ = targetSurfaceCloud;
    
    kdtreeTargetCornerCloudPtr_->setInputCloud(targetCornerCloudPtr_);
    kdtreeTargetSurfaceCloudPtr_->setInputCloud(targetSurfaceCloudPtr_);
    return true;
}

bool AloamRegistration::SetPredictPose(const Eigen::Isometry3d &predictPose){
    predictMatchResult_ = predictPose;
    //将预测位姿送进求解器
    Eigen::Quaterniond _q;
    _q = predictMatchResult_.rotation(); //转为四元数
    _q.normalize();//归一化
    param_q[0] = _q.x();
    param_q[1] = _q.y();
    param_q[2] = _q.z();
    param_q[3] = _q.w();

    param_t[0] = predictMatchResult_.translation()[0];
    param_t[1] = predictMatchResult_.translation()[1];
    param_t[2] = predictMatchResult_.translation()[2];
    return true;
}

bool AloamRegistration::ScanMatch(int iterCount){
    //执行匹配
    for(int i = 0; i < iterCount; ++i){
        FindCorrespondingCornerFeatures();
        FindCorrespondingSurfaceFeatures();
        // std::cout<<"角点:"<<corner_correspond_feature.size()<<std::endl;
        // std::cout<<"平面点:"<<surface_correspond_feature.size()<<std::endl;
        CalculateTransformation();
        //更新预测位姿，重新找特征点
        predictMatchResult_.setIdentity();
        predictMatchResult_.rotate(Q_LastPoseRotationToCurrentPose);
        predictMatchResult_.pretranslate(T_LastPoseTranslationToCurrentPose);
    }

    matchResult_.setIdentity();
    matchResult_.rotate(Q_LastPoseRotationToCurrentPose);
    matchResult_.pretranslate(T_LastPoseTranslationToCurrentPose);
    return true;
}

Eigen::Isometry3d AloamRegistration::GetMatchResult(){
    return matchResult_;
}

void AloamRegistration::FindCorrespondingCornerFeatures(){
    PointCloudData::point _point_to_last;
    std::vector<int> _point_search_index; //最近点索引值
    std::vector<float> _point_search_square_distance; //点和最近点的欧式距离的平方
    corner_correspond_feature.clear();

    for(int i = 0; i < sourceCornerCloudPtr_->points.size(); ++i){
        PointCloudTransformation::TransformToLast(sourceCornerCloudPtr_->points[i], _point_to_last, predictMatchResult_);
        //查找点变到上一帧坐标系下后和点最近的上一帧对应点
        kdtreeTargetCornerCloudPtr_->nearestKSearch(_point_to_last, 1, _point_search_index, _point_search_square_distance);

        int _closest_point_index = -1, _min_point_index = -1;
        if (_point_search_square_distance[0] < distance_square_threshold_)//欧式距离小于阈值
        {
            _closest_point_index = _point_search_index[0];
            //intensity，理论上讲存放的是强度，实际上是放什么就是什么，所以这个代表了来自第几线
            int _closest_point_scan_ID = int(targetCornerCloudPtr_->points[_closest_point_index].intensity);

            double _min_point_square_distance = distance_square_threshold_;

            //去找论文中的l点，要求是和j不同scan，但是离i最近的点
            //所以去前后n个scan中找距离i最近的点,n是限制别找偏了，因为角点基本都是在边线上，scan之间挨着最近的才更加可靠
            //所以下面分别对前面的scan和后面的scan查找

            // search in the direction of increasing scan line
            //存放结构是一维数组，前面的scan放在前面
            for (int j = _closest_point_index + 1; j < targetCornerCloudPtr_->points.size(); ++j)
            {
                // if in the same scan line, continue
                if (int(targetCornerCloudPtr_->points[j].intensity) <= _closest_point_scan_ID)
                    continue;

                // if not in nearby scans, end the loop
                if (int(targetCornerCloudPtr_->points[j].intensity) > (_closest_point_scan_ID + nearby_scan_))
                    break;

                double _point_square_distance = 
                        (targetCornerCloudPtr_->points[j].x - _point_to_last.x)*(targetCornerCloudPtr_->points[j].x - _point_to_last.x) 
                      + (targetCornerCloudPtr_->points[j].y - _point_to_last.y)*(targetCornerCloudPtr_->points[j].y - _point_to_last.y) 
                      + (targetCornerCloudPtr_->points[j].z - _point_to_last.z)*(targetCornerCloudPtr_->points[j].z - _point_to_last.z);

                if (_point_square_distance < _min_point_square_distance)//迭代找最近点
                {
                    // find nearer point
                    _min_point_square_distance = _point_square_distance;
                    _min_point_index = j;
                }
            }

            // search in the direction of decreasing scan line
            for (int j = _closest_point_index - 1; j >= 0; --j)
            {
                // if in the same scan line, continue
                if (int(targetCornerCloudPtr_->points[j].intensity) >= _closest_point_scan_ID)
                    continue;

                // if not in nearby scans, end the loop
                if (int(targetCornerCloudPtr_->points[j].intensity) < (_closest_point_scan_ID - nearby_scan_))
                    break;

                    double _point_square_distance = 
                            (targetCornerCloudPtr_->points[j].x - _point_to_last.x)*(targetCornerCloudPtr_->points[j].x - _point_to_last.x) 
                          + (targetCornerCloudPtr_->points[j].y - _point_to_last.y)*(targetCornerCloudPtr_->points[j].y - _point_to_last.y) 
                          + (targetCornerCloudPtr_->points[j].z - _point_to_last.z)*(targetCornerCloudPtr_->points[j].z - _point_to_last.z);

                if (_point_square_distance < _min_point_square_distance)
                {
                    // find nearer point
                    _min_point_square_distance = _point_square_distance;
                    _min_point_index = j;
                }
            }

            if(_min_point_index >= 0){
                CorrespondFeature _correspond_feature;
                _correspond_feature.current_point = Eigen::Vector3d(sourceCornerCloudPtr_->points[i].x,
                                                                    sourceCornerCloudPtr_->points[i].y,
                                                                    sourceCornerCloudPtr_->points[i].z);
                _correspond_feature.last_point_a = Eigen::Vector3d(targetCornerCloudPtr_->points[_closest_point_index].x,
                                                                    targetCornerCloudPtr_->points[_closest_point_index].y,
                                                                    targetCornerCloudPtr_->points[_closest_point_index].z);
                _correspond_feature.last_point_b = Eigen::Vector3d(targetCornerCloudPtr_->points[_min_point_index].x,
                                                                    targetCornerCloudPtr_->points[_min_point_index].y,
                                                                    targetCornerCloudPtr_->points[_min_point_index].z);
                corner_correspond_feature.push_back(_correspond_feature);
            }
        }
    }
}

void AloamRegistration::FindCorrespondingSurfaceFeatures(){
    PointCloudData::point _point_to_last;
    std::vector<int> _point_search_index; //最近点索引值
    std::vector<float> _point_search_square_distance; //点和最近点的欧式距离的平方
    surface_correspond_feature.clear();

    for(int i = 0; i < sourceSurfaceCloudPtr_->points.size(); ++i){
        PointCloudTransformation::TransformToLast(sourceSurfaceCloudPtr_->points[i], _point_to_last, predictMatchResult_);
        //查找点变到上一帧坐标系下后和点最近的上一帧对应点
        kdtreeTargetSurfaceCloudPtr_->nearestKSearch(_point_to_last, 1, _point_search_index, _point_search_square_distance);

        int _closest_point_index = -1, _min_point_index = -1, _min_point_index2 = -1;
        if (_point_search_square_distance[0] < distance_square_threshold_)//欧式距离小于阈值
        {
            _closest_point_index = _point_search_index[0];
            //intensity，理论上讲存放的是强度，实际上是放什么就是什么，所以这个代表了来自第几线
            int _closest_point_scan_ID = int(targetSurfaceCloudPtr_->points[_closest_point_index].intensity);
            double _min_point_square_distance = distance_square_threshold_;
            double _min_point_square_distance2 = distance_square_threshold_;

            // search in the direction of increasing scan line
            //存放结构是一维数组，前面的scan放在前面
            for (int j = _closest_point_index + 1; j < targetSurfaceCloudPtr_->points.size(); ++j)
            {
                // if not in nearby scans, end the loop
                if (int(targetSurfaceCloudPtr_->points[j].intensity) > (_closest_point_scan_ID + nearby_scan_))
                    break;

                double _point_square_distance = 
                        (targetSurfaceCloudPtr_->points[j].x - _point_to_last.x)*(targetSurfaceCloudPtr_->points[j].x - _point_to_last.x) 
                      + (targetSurfaceCloudPtr_->points[j].y - _point_to_last.y)*(targetSurfaceCloudPtr_->points[j].y - _point_to_last.y) 
                      + (targetSurfaceCloudPtr_->points[j].z - _point_to_last.z)*(targetSurfaceCloudPtr_->points[j].z - _point_to_last.z);

                if (_point_square_distance < _min_point_square_distance)//迭代找最近点
                {
                    //这里用等号也可以，因为排过序了。点属于的线要么相等，要么大于。
                    if(int(targetSurfaceCloudPtr_->points[j].intensity) <= _closest_point_scan_ID ){
                        _min_point_square_distance = _point_square_distance;
                        _min_point_index = j;
                    }else{
                        _min_point_square_distance2 = _point_square_distance;
                        _min_point_index2 = j;
                    }
                }
            }

            // search in the direction of decreasing scan line
            for (int j = _closest_point_index - 1; j >= 0; --j)
            {
                // if not in nearby scans, end the loop
                if (int(targetSurfaceCloudPtr_->points[j].intensity) < (_closest_point_scan_ID - nearby_scan_))
                    break;

                    double _point_square_distance = 
                            (targetSurfaceCloudPtr_->points[j].x - _point_to_last.x)*(targetSurfaceCloudPtr_->points[j].x - _point_to_last.x) 
                          + (targetSurfaceCloudPtr_->points[j].y - _point_to_last.y)*(targetSurfaceCloudPtr_->points[j].y - _point_to_last.y) 
                          + (targetSurfaceCloudPtr_->points[j].z - _point_to_last.z)*(targetSurfaceCloudPtr_->points[j].z - _point_to_last.z);

                if (_point_square_distance < _min_point_square_distance)
                {
                    //这里用等号也可以，因为排过序了。点属于的线要么相等，要么小于。
                    if(int(targetSurfaceCloudPtr_->points[j].intensity) >= _closest_point_scan_ID ){
                        _min_point_square_distance = _point_square_distance;
                        _min_point_index = j;
                    }else{
                        _min_point_square_distance2 = _point_square_distance;
                        _min_point_index2 = j;
                    }
                }
            }

            if(_min_point_index >= 0 && _min_point_index2 >= 0){
                CorrespondFeature _correspond_feature;
                _correspond_feature.current_point = Eigen::Vector3d(sourceSurfaceCloudPtr_->points[i].x,
                                                                    sourceSurfaceCloudPtr_->points[i].y,
                                                                    sourceSurfaceCloudPtr_->points[i].z);
                _correspond_feature.last_point_a = Eigen::Vector3d(targetSurfaceCloudPtr_->points[_closest_point_index].x,
                                                                    targetSurfaceCloudPtr_->points[_closest_point_index].y,
                                                                    targetSurfaceCloudPtr_->points[_closest_point_index].z);
                _correspond_feature.last_point_b = Eigen::Vector3d(targetSurfaceCloudPtr_->points[_min_point_index].x,
                                                                    targetSurfaceCloudPtr_->points[_min_point_index].y,
                                                                    targetSurfaceCloudPtr_->points[_min_point_index].z);
                _correspond_feature.last_point_c = Eigen::Vector3d(targetSurfaceCloudPtr_->points[_min_point_index2].x,
                                                                    targetSurfaceCloudPtr_->points[_min_point_index2].y,
                                                                    targetSurfaceCloudPtr_->points[_min_point_index2].z);
                surface_correspond_feature.push_back(_correspond_feature);
            }
        }
    }
}

void AloamRegistration::CalculateTransformation(){
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(param_q, 4, q_parameterization);
    problem.AddParameterBlock(param_t, 3);

    for(int i = 0; i < corner_correspond_feature.size(); ++i){
        //最小二乘问题是 min 1/2 ∑ ρ(||Fi(x)||²)  i∈(0,n)
        //创建functor，也就是给定计算公式Fi(x)
        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(corner_correspond_feature[i].current_point, 
                                                                    corner_correspond_feature[i].last_point_a, 
                                                                    corner_correspond_feature[i].last_point_b, 
                                                                    1);
        //添加残差块， 相当于 +ρ(||Fi(x)||²)，para_q，para_t就是待优化的变量
        //计算一个变换，使得根据这个变换计算出的点和上一个sweep之间特征的欧式距离最小
        problem.AddResidualBlock(cost_function, loss_function, param_q, param_t);
    }

    for(int i = 0; i < surface_correspond_feature.size(); ++i){
        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(surface_correspond_feature[i].current_point, 
                                                                    surface_correspond_feature[i].last_point_a, 
                                                                    surface_correspond_feature[i].last_point_b, 
                                                                    surface_correspond_feature[i].last_point_c,
                                                                    1);
        problem.AddResidualBlock(cost_function, loss_function, param_q, param_t);
    }

    if ((surface_correspond_feature.size() + corner_correspond_feature.size()) < 10){
        std::cout<<"less correspondence!  ************************************"<<std::endl;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 8;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

// ceres::TerminationType::CONVERGENCE;
//     std::cout<<summary.BriefReport()<<std::endl;
//     std::cout<<"init cost:"<<summary.initial_cost<<std::endl;
//     std::cout<<"final cost:"<<summary.final_cost<<std::endl;
//     std::cout<<"termianl_type:"<<summary.termination_type<<std::endl;
//     std::cout<<"is_constrained:"<<summary.is_constrained<<std::endl;
}

}