/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-30 19:18:17 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 01:07:48
 */

#include "glog/logging.h"

#include "mapping/back_end/back_end.hpp"
#include "tools_/tic_toc.hpp"



namespace loam_frame{

BackEnd::BackEnd(ros::NodeHandle& nh){
    InitWithConfig();
    mapHandlePtr = std::make_shared<MapHandleInterface>(nh);
    keyframeRelativePose_.setIdentity();
}

bool BackEnd::InitWithConfig(){
    std::string config_file_path = WORK_SPACE_PATH + "/config/back_end/back_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------后端初始化-------------------" << std::endl;
    InitOptimize(config_node);
    InitDataPath(config_node);
    return true;
}

bool BackEnd::InitOptimize(const YAML::Node& config_node){
    std::string _optimize_method = config_node["optimize_method"].as<std::string>();

    std::cout << "后端选择的图优化方式为：" << _optimize_method << std::endl;

    if (_optimize_method == "g2o") {
        graphOptimizationPtr = std::make_shared<G2oGraphOptimizer>();
        lastKeyframeOptimizedPose_.setIdentity();
        graphOptimizationPtr->AddSe3Node(lastKeyframeOptimizedPose_, true); //设定开始值，锁死
    }
    else {
        LOG(ERROR) << "没找到与 " << _optimize_method << " 相对应的优化方式!";
        return false;
    }
    return true;
}

bool BackEnd::InitDataPath(const YAML::Node& config_node){
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./"){
        data_path = WORK_SPACE_PATH;
    }else{
        data_path = WORK_SPACE_PATH + data_path;
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

    trajectoryPath_ = data_path + "/slam_data/optimize_trajectory";


    if (!FileManager::InitDirectory(trajectoryPath_, "轨迹文件"))
        return false;

    if (!FileManager::CreateFile(groundTruthOfs_, trajectoryPath_ + "/truth_path.txt"))
        return false;
    if (!FileManager::CreateFile(laserOdomOfs_, trajectoryPath_ + "/odome_path.txt"))
        return false;
    std::cout << "轨迹存放位置：" << trajectoryPath_ << std::endl;
    return true;
}

bool BackEnd::SavePose(std::ofstream& ofs, const Eigen::Isometry3d& pose){
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i,j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }
    return true;
}

bool BackEnd::Update(const PointCloudData &segmented_cloud, Eigen::Isometry3d relative_pose, Eigen::Isometry3d gnss_data, bool have_prioi_gnss){
    static bool is_frist_frame = true;
    keyframeRelativePose_ = keyframeRelativePose_ * relative_pose;
    if(is_frist_frame){
        //保存关键帧
        mapHandlePtr->AddKeyFrame(segmented_cloud.cloud_ptr);
        //保存关键帧的位姿
        PoseParam _poseParam;
        _poseParam.time = segmented_cloud.time;
        _poseParam.pose = lastKeyframeOptimizedPose_;
        mapHandlePtr->AddKeyPose(_poseParam);
        mapHandlePtr->AddKeyframToMap();
        is_frist_frame = false;
    }

    Eigen::Affine3d _temp_pose(keyframeRelativePose_.matrix());
    double x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(_temp_pose, x, y, z, roll, pitch, yaw);

    if (abs(roll) > 0.2 || abs(pitch) > 0.2 || abs(yaw) > 0.2 || sqrt(x*x + y*y + z*z) > 1.0){
        Eigen::Isometry3d _new_keyframe_pose = lastKeyframeOptimizedPose_ * keyframeRelativePose_;
        //保存关键帧
        mapHandlePtr->AddKeyFrame(segmented_cloud.cloud_ptr);
        //保存关键帧的位姿
        PoseParam _poseParam;
        _poseParam.time = segmented_cloud.time;
        _poseParam.pose = _new_keyframe_pose;
        mapHandlePtr->AddKeyPose(_poseParam);
        mapHandlePtr->AddKeyframToMap();

        //向图中添加关键帧作为顶点
        graphOptimizationPtr->AddSe3Node(_new_keyframe_pose, false);
        //为图中的顶点添加约束作为边
        int node_num = graphOptimizationPtr->GetNodeNum();
        if (node_num > 1) {
            Eigen::VectorXd _odom_edge_noise(6);
            _odom_edge_noise<<0.5, 0.5, 0.5, 0.001, 0.001, 0.001;
            graphOptimizationPtr->AddSe3Edge(node_num-2, node_num-1, keyframeRelativePose_, _odom_edge_noise);

            if(have_prioi_gnss){
                Eigen::Vector3d gnss_gap = _new_keyframe_pose.translation() - gnss_data.translation();
                //模长的平方
                double _gnss_gap_length = gnss_gap[0] * gnss_gap[0] + gnss_gap[1] * gnss_gap[1] + gnss_gap[2] * gnss_gap[2];
                if(_gnss_gap_length > 0.1){
                    Eigen::Vector3d xyz = gnss_data.translation();
                    Eigen::VectorXd _gnss_noise(3);
                    _gnss_noise<<2.0, 2.0, 2.0;
                    graphOptimizationPtr->AddSe3PriorXYZEdge(node_num - 1, xyz, _gnss_noise);
                    need_optimized_ = true;
                }
                SavePose(groundTruthOfs_, gnss_data);
                SavePose(laserOdomOfs_, lastKeyframeOptimizedPose_);
            }
        }
        lastKeyframeOptimizedPose_ = _new_keyframe_pose;
        keyframeRelativePose_.setIdentity();
        TicToc _time;
        _time.tic();
        if(need_optimized_){
            graphOptimizationPtr->Optimize();
            need_optimized_ = false;
            path_is_update = true;
        }
        // std::cout<<"优化总时间:"<<_time.toc()<<std::endl;
        return true;
    }
    return false;
}

bool BackEnd::GetOptimizedPose(std::deque<Eigen::Isometry3d> &optimized_pose){
    if(!path_is_update){
        return false;
    }
    bool _result = graphOptimizationPtr->GetOptimizedPose(optimized_pose);
    Eigen::Isometry3d _currentKeyFramePose = optimized_pose.back();
    //怀疑是精度的问题，所有行列式不为1
    Eigen::Quaterniond _q;
    _q = _currentKeyFramePose.rotation(); //转为四元数
    _q.normalize();//归一化
    Eigen::Vector3d _v;
    _v[0] = _currentKeyFramePose.translation()[0];
    _v[1] = _currentKeyFramePose.translation()[1];
    _v[2] = _currentKeyFramePose.translation()[2];

    _currentKeyFramePose.setIdentity();
    _currentKeyFramePose.rotate(_q);
    _currentKeyFramePose.pretranslate(_v);

    lastKeyframeOptimizedPose_ = _currentKeyFramePose;
    path_is_update = false;
    mapHandlePtr->CorrectPoses(optimized_pose);
    static int count = 0;
    if(count>1){
        count = 0;
        mapHandlePtr->GenerateMap();
    }
    count ++;
    return _result;
}

Eigen::Isometry3d BackEnd::GetCurrentOptimizedPose(){
    return lastKeyframeOptimizedPose_;
}

PointCloudData::point_cloud_ptr BackEnd::GetWorldMap(){
    return mapHandlePtr->GetWorldMap();
}


}
