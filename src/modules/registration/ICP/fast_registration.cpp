/* 
 * @Author: Hu Ziwei
 * @Description: icp 配准 寻找最近的5个点，对点云协方差矩阵进行主成份分析：
 * 若这5个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，与该特征值相关的特征向量表示所处直线的方向;
 * 若这5个点分布在平面上，协方差矩阵的特征值存在一个显著小的元素，与该特征值相关的特征向量表示所处平面的法线方向。
 * 参考论文:2016,IROS,fast and robust 3d feature extraction from sparse point clouds
 * @Date: 2021-09-08 15:43:20
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-04 16:45:13
 */

#include "modules/registration/ICP/fast_registration.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"

namespace loam_frame{

FastRegistration::FastRegistration(const YAML::Node& configNode):
    kdtreeTargetCornerCloudPtr_(new pcl::KdTreeFLANN<PointCloudData::point>()),
    kdtreeTargetSurfaceCloudPtr_(new pcl::KdTreeFLANN<PointCloudData::point>()){
    InitWithConfig(configNode);
}

bool FastRegistration::InitWithConfig(const YAML::Node& configNode){
    cornerDistanceSquareThreshold_ = configNode["corner_distance_square_threshold"].as<double>();
    surfaceDistanceSquareThreshold_ = configNode["surface_distance_square_threshold"].as<double>();
    return true;
}

bool FastRegistration::PointCloudInput(const PointCloudData::point_cloud_ptr sourceCornerCloud,
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

bool FastRegistration::SetPredictPose(const Eigen::Isometry3d &predictPose){
    predictMatchResult_ = predictPose;
    // 用预测的位姿来更新优化器中的初始位姿
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

bool FastRegistration::ScanMatch(int iterCount){

    for(int i = 0; i < iterCount; ++i){
        FindCorrespondingCornerFeatures();
        // std::cout<<"角点匹配通过"<<std::endl;
        FindCorrespondingSurfaceFeatures();
        // std::cout<<"平面点匹配通过"<<std::endl;
        // std::cout<<"角点:"<<cornerCorrespondFeature_.size()<<std::endl;
        // std::cout<<"平面点:"<<surfaceCorrespondFeature_.size()<<std::endl;
        CalculateTransformation();
        //更新预测位姿，重新找特征点
        predictMatchResult_.setIdentity();
        predictMatchResult_.rotate(Q_LastPoseRotationToCurrentPose);
        predictMatchResult_.pretranslate(T_LastPoseTranslationToCurrentPose);
    }

    matchResult_.setIdentity();
    Q_LastPoseRotationToCurrentPose.normalize();
    matchResult_.rotate(Q_LastPoseRotationToCurrentPose);
    matchResult_.pretranslate(T_LastPoseTranslationToCurrentPose);

    return true;
}

Eigen::Isometry3d FastRegistration::GetMatchResult(){
    return matchResult_;
}

void FastRegistration::FindCorrespondingCornerFeatures(){
    PointCloudData::point _pointToWorld;
    std::vector<int> _pointSearchIndex; //最近点索引值
    std::vector<float> _pointSearchSquareDistance; //点和最近点的欧式距离的平方
    cornerCorrespondFeature_.clear();

    for (int i = 0; i < sourceCornerCloudPtr_->points.size(); i++)
    {
        PointCloudTransformation::TransformToLast(sourceCornerCloudPtr_->points[i], _pointToWorld, predictMatchResult_);
        //找最近的五个点
        kdtreeTargetCornerCloudPtr_->nearestKSearch(_pointToWorld, 5, _pointSearchIndex, _pointSearchSquareDistance); 

        if (_pointSearchSquareDistance[4] < cornerDistanceSquareThreshold_)//5个点都小于阈值
        { 
            std::vector<Eigen::Vector3d> _nearCorners;
            Eigen::Vector3d _center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d _tmp(targetCornerCloudPtr_->points[_pointSearchIndex[j]].x,
                                    targetCornerCloudPtr_->points[_pointSearchIndex[j]].y,
                                    targetCornerCloudPtr_->points[_pointSearchIndex[j]].z);
                _center = _center + _tmp;
                _nearCorners.push_back(_tmp);
            }
            _center = _center / 5.0;//计算出这5个最近点的中点
            /*
            寻找最近的5个点，对点云协方差矩阵进行主成份分析：
            若这5个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，与该特征值相关的特征向量表示所处直线的方向;
            若这5个点分布在平面上，协方差矩阵的特征值存在一个显著小的元素，与该特征值相关的特征向量表示所处平面的法线方向。
            参考论文:2016,IROS,fast and robust 3d feature extraction from sparse point clouds
            *  计算协方差矩阵，注意协方差矩阵计算的是样本不同维度间的相关性   而不是样本之间的相关性
            *  这里有5个样本点 ，每个样本3维   协方差矩阵 = 1/(n-1) * (X-EX)(X-EX)^T ,   X = (x1,x2,x3,x4,x5) 
            *  这里没有除 1/n-1 但关系不大，因为特征向量的方向不会改变
            */
            Eigen::Matrix3d _covMat = Eigen::Matrix3d::Zero();//计算协方差矩阵
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> _tmpZeroMean = _nearCorners[j] - _center;
                _covMat = _covMat + _tmpZeroMean * _tmpZeroMean.transpose(); //转置
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> _saes(_covMat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            //eigen库排序是从小到大的顺序
            Eigen::Vector3d _unit_direction = _saes.eigenvectors().col(2); // 最大的特征值对应的特征向量
            Eigen::Vector3d _curr_point(sourceCornerCloudPtr_->points[i].x, 
                                        sourceCornerCloudPtr_->points[i].y, 
                                        sourceCornerCloudPtr_->points[i].z);
            // 如果最大的特征向量  明显比第二大的大    则认为是直线 
            if (_saes.eigenvalues()[2] > 3 * _saes.eigenvalues()[1])
            {   //如果角特征对应的是线特征，就认为center在线上，此时沿着直线方向取两个点送进迭代器，因为两点确定一条直线
                CorrespondFeature _correspond_feature;
                _correspond_feature.current_point = _curr_point;
                _correspond_feature.last_point_a = 0.1 * _unit_direction + _center;
                _correspond_feature.last_point_b = -0.1 * _unit_direction + _center;
                cornerCorrespondFeature_.push_back(_correspond_feature);	
            }							
        }
    }			
}

void FastRegistration::FindCorrespondingSurfaceFeatures(){
    PointCloudData::point _pointToWorld;
    std::vector<int> _pointSearchIndex; //最近点索引值
    std::vector<float> _pointSearchSquareDistance; //点和最近点的欧式距离的平方
    surfaceCorrespondFeature_.clear();

    for (int i = 0; i < sourceSurfaceCloudPtr_->points.size(); i++)
    {
        PointCloudTransformation::TransformToLast(sourceSurfaceCloudPtr_->points[i], _pointToWorld, predictMatchResult_);
        kdtreeTargetSurfaceCloudPtr_->nearestKSearch(_pointToWorld, 5, _pointSearchIndex, _pointSearchSquareDistance);

        Eigen::Matrix<double, 5, 3> _matA0;
        Eigen::Matrix<double, 5, 1> _matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();//全1矩阵
        if (_pointSearchSquareDistance[4] < surfaceDistanceSquareThreshold_)
        {
            
            for (int j = 0; j < 5; j++)
            {
                _matA0(j, 0) = targetSurfaceCloudPtr_->points[_pointSearchIndex[j]].x;
                _matA0(j, 1) = targetSurfaceCloudPtr_->points[_pointSearchIndex[j]].y;
                _matA0(j, 2) = targetSurfaceCloudPtr_->points[_pointSearchIndex[j]].z;
            }
            // find the norm of plane
            //Ax+By+Cz+D = 0 =>  (x,y,z)(A/D,B/D,C/D)^T = -1  => 求解  AX = b ,X=A逆*b即为法向量，解方程嘛。
            //此时，给定的是5个xyz的值，然后去求解A/D,B/D,C/D，这三个数就是法向量
            Eigen::Vector3d _norm = _matA0.colPivHouseholderQr().solve(_matB0);
            double _negative_OA_dot_norm = 1 / _norm.norm(); //norm返回2范数，对向量来说，也就是自身点积然后开平方。求模长呗
            _norm.normalize(); //标准化为单位向量

            // Here n(pa, pb, pc) is unit norm of plane
            bool _planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                //这块相当于A/Dx+B/Dy+C/Dz+1 = 0
                //对法向量进行归一化，也就是求出来的类似A/D都要除以模长，所以上面直接用1除了模长
                //又因为求得也是一个最小二乘问题，给定的这几个点不可能说完全在一个平面上
                //下面这个式子相当于求点到平面的距离
                if (fabs(_norm(0) * targetSurfaceCloudPtr_->points[_pointSearchIndex[j]].x +
                            _norm(1) * targetSurfaceCloudPtr_->points[_pointSearchIndex[j]].y +
                            _norm(2) * targetSurfaceCloudPtr_->points[_pointSearchIndex[j]].z + _negative_OA_dot_norm) > 0.2)
                {
                    _planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d _curr_point(sourceSurfaceCloudPtr_->points[i].x, 
                                        sourceSurfaceCloudPtr_->points[i].y, 
                                        sourceSurfaceCloudPtr_->points[i].z);
            if (_planeValid)
            {
                PlaneCorrespondFeature _correspond_feature;
                _correspond_feature.current_point = _curr_point;
                _correspond_feature.norm = _norm;
                _correspond_feature.OA_dot_norm = _negative_OA_dot_norm;
                surfaceCorrespondFeature_.push_back(_correspond_feature);	
            }
        }
    }
}

void FastRegistration::CalculateTransformation(){
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);

    problem.AddParameterBlock(param_q, 4, q_parameterization);
    problem.AddParameterBlock(param_t, 3);

    for(int i = 0; i < cornerCorrespondFeature_.size(); ++i){
        //最小二乘问题是 min 1/2 ∑ ρ(||Fi(x)||²)  i∈(0,n)
        //创建functor，也就是给定计算公式Fi(x)
        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(cornerCorrespondFeature_[i].current_point, 
                                                                    cornerCorrespondFeature_[i].last_point_a, 
                                                                    cornerCorrespondFeature_[i].last_point_b, 
                                                                    1);
        //添加残差块， 相当于 +ρ(||Fi(x)||²)，para_q，para_t就是待优化的变量
        //计算一个变换，使得根据这个变换计算出的点和上一个sweep之间特征的欧式距离最小
        problem.AddResidualBlock(cost_function, loss_function, param_q, param_t);
    }

    for(int i = 0; i < surfaceCorrespondFeature_.size(); ++i){
        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(surfaceCorrespondFeature_[i].current_point, 
                                                                    surfaceCorrespondFeature_[i].norm, 
                                                                    surfaceCorrespondFeature_[i].OA_dot_norm);
        problem.AddResidualBlock(cost_function, loss_function, param_q, param_t);
    }

    if ((cornerCorrespondFeature_.size() + surfaceCorrespondFeature_.size()) < 10){
        std::cout<<"less correspondence!  ************************************"<<std::endl;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 16; //最大迭代次数
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout<<"求解残差"<<summary.final_cost<<std::endl;

    // summary.termination_type == ceres::TerminationType::CONVERGENCE; //收敛才结束

}

// bool ScanToMapIcp::LoopScanMatch(int iterCount, Eigen::Isometry3d loopBeginPose, Eigen::Isometry3d predictPose){

//     // ICP参数设置
//     static pcl::IterativeClosestPoint<PointCloudData::point, PointCloudData::point> icp;
//     icp.setMaxCorrespondenceDistance(15.0*2);
//     icp.setMaximumIterations(100);
//     icp.setTransformationEpsilon(1e-6);
//     icp.setEuclideanFitnessEpsilon(1e-6);
//     icp.setRANSACIterations(0);

//     cureKeyframeCloud.reset(new PointCloudData::point_cloud());
//     // *cureKeyframeCloud = TransformToWorld(cloudPointsPtr_, predictPose);
//     // *cureKeyframeCloud += TransformToWorld(sourceCornerCloudPtr_, predictPose);
//     // *cureKeyframeCloud += TransformToWorld(sourceSurfaceCloudPtr_, predictPose);
//     *cureKeyframeCloud += *sourceCornerCloudPtr_;
//     *cureKeyframeCloud += *sourceSurfaceCloudPtr_;
//     icp.setInputSource(cureKeyframeCloud);

//     PointCloudData::point_cloud_ptr prevKeyframeCloud;
//     prevKeyframeCloud.reset(new PointCloudData::point_cloud());
//     *prevKeyframeCloud += *targetCornerCloudPtr_;
//     *prevKeyframeCloud += *targetSurfaceCloudPtr_;
//     icp.setInputTarget(prevKeyframeCloud);

//     PointCloudData::point_cloud_ptr unused_result(new PointCloudData::point_cloud());
//     icp.align(*unused_result);

//     // 未收敛，或者匹配不够好
//     if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyFrameFitnessScore_){
//         std::cout<<"匹配结果不好"<<std::endl;
//         std::cout<<icp.hasConverged()<<std::endl;
//         std::cout<<icp.getFitnessScore()<<std::endl;
//         return false;
//     }
//     std::cout<<icp.getFitnessScore()<<std::endl;

//     Eigen::Isometry3d _stepPose(icp.getFinalTransformation().cast<double>());

//     std::cout<<"icp约束"<<_stepPose.matrix()<<std::endl;

//     currentPoseInWorldMap.setIdentity();
//     currentPoseInWorldMap = predictPose*_stepPose;
//     // currentPoseInWorldMap.rotate(Q_LastPoseRotationToCurrentPose);
//     // currentPoseInWorldMap.pretranslate(T_LastPoseTranslationToCurrentPose);

//     //获取从回环开始到当前矩阵的变换矩阵
//     loopStepPoseTransform = loopBeginPose.inverse() * currentPoseInWorldMap;

//     return true;
// }

}
