/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-12-01 15:18:41 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-15 01:07:13
 */

#include "mapping/front_end/front_end_flow.hpp"
#include "modules/transformation/point_cloud_transformation.hpp"

#include "tools_/tic_toc.hpp"
#include "tools_/frame_rotate.hpp"

namespace loam_frame{
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string gnss_topic){
    nh.getParam("/loam_frame/adjust_distort", adjust_distort_);
    //订阅去过畸变的点云
    cloudSubPtr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    //订阅gnss数据
    gnssSubPtr_ = std::make_shared<GNSSSubscriber>(nh, gnss_topic, 1000000);
    imuSubPtr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    // "/kitti/oxts/gps/fix", 
    // "/kitti/oxts/gps/vel",
    //订阅世界坐标系到机器人本地坐标系的旋转矩阵
    worldToBaselinkLisPtr = std::make_shared<TFListener>(nh, "world", "base_link");
    //发布去畸变后的点云数据
    cloudAdjustPubPtr_ = std::make_shared<CloudPublisher>(nh, "adjust_distort_point_cloud", 100, "/map");
    downsizePointCloudPubPtr_ = std::make_shared<CloudPublisher>(nh, "downsize_point_cloud", 100, "/map");
    //发布投影到2d的点云
    imagePointCloudPubPtr_ = std::make_shared<CloudPublisher>(nh, "image_point_cloud", 100, "/map");
    //发布分割出的地面点
    groundCloudPubPtr_ = std::make_shared<CloudPublisher>(nh, "ground_cloud", 100, "/map");
    //发布分割出的障碍物
    obstacleCloudPubPtr_ = std::make_shared<CloudPublisher>(nh, "obstacle_cloud", 100, "/map");
    //发布地面点和障碍物点的集合点云
    segmentedCloudPubPtr_ = std::make_shared<CloudPublisher>(nh, "segmented_cloud", 100, "/map");
    //发布提取出的四种特征点
    cornerPointsSharpPubPtr_ = std::make_shared<CloudPublisher>(nh, "corner_points_sharp", 100, "/map");
    cornerPointsLessSharpPubPtr_ = std::make_shared<CloudPublisher>(nh, "corner_points_less_sharp", 100, "/map");
    surfacePointsFlatPubPtr_ = std::make_shared<CloudPublisher>(nh, "surface_points_flat", 100, "/map");
    surfacePointsLessFlatPubPtr_ = std::make_shared<CloudPublisher>(nh, "surface_points_less_flat", 100, "/map");

    cornerSubmapPubPtr_ = std::make_shared<CloudPublisher>(nh, "corner_submap", 100, "/map");
    surfaceSubmapPubPtr_ = std::make_shared<CloudPublisher>(nh, "surface_submap", 100, "/map");
    cloudWorldPubPtr_ = std::make_shared<CloudPublisher>(nh, "cloud_in_world", 100, "/map");
    pathPubPtr_ = std::make_shared<PathPublisher>(nh, "map_path", "/map", 100);
    gnssPathPubPtr_ =  std::make_shared<PathPublisher>(nh, "gnss_path", "/map", 100);

    lidarOdometryPubPtr = std::make_shared<OdometryPublisher>(nh, "/frame/relative_pose", "/map", "/map",100);
    gnssOdometryPubPtr = std::make_shared<OdometryPublisher>(nh, "/frame/prioi_gnss", "/map", "/map",100);

    imuPreintegratePtr_ = std::make_shared<ImuPreintegrate>();
    distortCloudPtr_ = std::make_shared<DistortionAdjust>();
    frontEndPtr_ =  std::make_shared<FrontEnd>();

    downSizeFilterImagePointCloud_.setLeafSize(0.2, 0.2, 0.2);
    ImagePointCloudDSPtr.reset(new PointCloudData::point_cloud());
}

bool FrontEndFlow::ReadData(){
    static int jump_scan = 0;
    cloudSubPtr_->ParseData(cloudDataBuff_);
    imuSubPtr_->ParseData(unsyncedImuDataBuff_);
    gnssSubPtr_->ParseData(unsyncedGnssDataBuff_);

    if (!transformReceived) {
        if(!unsyncedImuDataBuff_.empty()){
            Eigen::Quaterniond init_q =  unsyncedImuDataBuff_.front().orientation;
            worldToBaselink.setIdentity();
            worldToBaselink.rotate(init_q);
            std::cout<<"trans"<<std::endl<<worldToBaselink.inverse().matrix()<<std::endl;
            transformReceived = true;
        }
    }

    return true;
}

bool FrontEndFlow::InitUnsyncedData(){
    static bool initImuData = false;
    static bool initGnssData = false;
    if(!initGnssData || !initImuData){
        while(!unsyncedGnssDataBuff_.empty() && !unsyncedImuDataBuff_.empty() &&!cloudDataBuff_.empty()){
            if(unsyncedImuDataBuff_.front().time < cloudDataBuff_.front().time){
                initImuData = true;
            }else{
                cloudDataBuff_.pop_front();
                continue;
            }

            if(unsyncedGnssDataBuff_.front().time < cloudDataBuff_.front().time){
                initGnssData = true;
            }else{
                cloudDataBuff_.pop_front();
                continue;
            }

            if(initGnssData && initImuData){
                break;
            }
        }
    }
    return (initGnssData && initImuData);
}

bool FrontEndFlow::SyncDataClock(){
    static double last_sync_time = 0;
    static std::list<double> _gnss_sync_time_list;
    static std::list<double> _imu_sync_time_list;
    std::deque<PointCloudData>::iterator _iter;
    for(auto _iter = cloudDataBuff_.begin(); _iter != cloudDataBuff_.end(); _iter++){
        if(_iter->time > last_sync_time){
            _imu_sync_time_list.push_back(_iter->time);
            //同步数据 在该时刻插个值就完事了呗
            IMUData::SyncData(unsyncedImuDataBuff_, syncedImuDataBuff_, _imu_sync_time_list);
            _gnss_sync_time_list.push_back(_iter->time);
            //同步数据 在该时刻插个值就完事了呗
            GNSSData::SyncData(unsyncedGnssDataBuff_, syncedGnssDataBuff_, _gnss_sync_time_list);
            last_sync_time = _iter->time;
            // std::cout<<"insert data time: "<<std::fixed<<std::setprecision(4)<<_iter->time<<std::endl;
        }
    }
    return true;
}

bool FrontEndFlow::PreintegrateIMU(IMUData current_imu_data){
    static IMUData last_imu_data;
    static bool is_frist_frame = true;
    static bool init_begin_cloud = false;
    bool integrate_complete = false;
    if(is_frist_frame){
        last_imu_data = current_imu_data;
        is_frist_frame = false;
        return false;
    }
    // std::cout<<"last imu time: "<<std::fixed<<std::setprecision(4)<<last_imu_data.time<<std::endl;
    // std::cout<<"current imu time: "<<std::fixed<<std::setprecision(4)<<current_imu_data.time<<std::endl;
    if(!initCloudData){
        if((last_imu_data.time < cloudDataBuff_.at(0).time) && (current_imu_data.time >= cloudDataBuff_.at(0).time)){
            ImuBias _b;
            imuPreintegratePtr_->Initialize(_b);
            initCloudData = true;
            std::cout<<"init imu cloud first frame"<<std::endl;
        }
    }else{
        double dt = current_imu_data.time - last_imu_data.time;
        // std::cout<<"dt: "<<std::fixed<<std::setprecision(4)<<dt<<std::endl; 
        cv::Point3f acc,ang_vel;
        acc.x = last_imu_data.linear_acceleration.x();
        acc.y = last_imu_data.linear_acceleration.y();
        acc.z = last_imu_data.linear_acceleration.z();

        ang_vel.x = last_imu_data.angular_velocity.x();
        ang_vel.y = last_imu_data.angular_velocity.y();
        ang_vel.z = last_imu_data.angular_velocity.z();

        imuPreintegratePtr_->IntegrateNewMeasurement(acc, ang_vel, dt);
        if((last_imu_data.time < cloudDataBuff_.at(1).time) && (current_imu_data.time >= cloudDataBuff_.at(1).time)){
            ImuBias _b;
            cv::Mat _rotation = imuPreintegratePtr_->GetDeltaRotation(_b);
            imu_pose_(0,0) = _rotation.at<float>(0,0);
            imu_pose_(0,1) = _rotation.at<float>(0,1);
            imu_pose_(0,2) = _rotation.at<float>(0,2);
            imu_pose_(1,0) = _rotation.at<float>(1,0);
            imu_pose_(1,1) = _rotation.at<float>(1,1);
            imu_pose_(1,2) = _rotation.at<float>(1,2);
            imu_pose_(2,0) = _rotation.at<float>(2,0);
            imu_pose_(2,1) = _rotation.at<float>(2,1);
            imu_pose_(2,2) = _rotation.at<float>(2,2);

            cv::Mat _pose = imuPreintegratePtr_->GetDeltaPosition(_b);
            imu_pose_(0,3) = _pose.at<float>(0);
            imu_pose_(1,3) = _pose.at<float>(1);
            imu_pose_(2,3) = 0;

            // std::cout<<"imu data: "<<std::fixed<<std::setprecision(4)<<current_imu_data.time<<std::endl; 
            // std::cout<<"imu 预积分的位姿"<<std::endl<<imu_pose_.matrix()<<std::endl;
            imuPreintegratePtr_->Initialize(_b);
            integrate_complete = true;
        }
    }
    last_imu_data = current_imu_data;
    return integrate_complete;
}

bool FrontEndFlow::UpdateLaserOdometry(){
    static bool matchingPoseInited = false;
    static double last_cloud_time = 0;
    if (!matchingPoseInited) {
        matchingPoseInited = true;
        frontEndPtr_->SetInitPose(Eigen::Isometry3d::Identity());
    }
    //接收坐标系转换矩阵，不能用了，比较蛋疼
    // if (!transformReceived) {
    //     if (worldToBaselinkLisPtr->LookupData(worldToBaselink)) {
    //         Eigen::Isometry3d tran = Eigen::Isometry3d(worldToBaselink.cast<double>()).inverse();
    //         std::cout<<"trans"<<std::endl<<tran.matrix()<<std::endl;
    //         transformReceived = true;
    //     }
    // }
    // std::cout<<"cloud: "<<std::fixed<<std::setprecision(4)<<dataFrames.front().cloud.time<<std::endl;

    Eigen::Vector3d velo = frontEndPtr_->GetMotion().translation(); //上一帧到当前帧的运动
    dataFrames.front().preintegrate_imu_pose.translation() += velo;
    Eigen::Isometry3d _predict_delta_pose = dataFrames.front().preintegrate_imu_pose;
    frontEndPtr_->SetPredictPose(_predict_delta_pose);

    if(ValidGNSSData()){
        //初始化gnss的位置
        syncedGnssDataBuff_.front().UpdateXYZ();
        Eigen::Vector3d _gnss_tranlation;
        _gnss_tranlation[0] = syncedGnssDataBuff_.front().local_E;
        _gnss_tranlation[1] = syncedGnssDataBuff_.front().local_N;
        _gnss_tranlation[2] = syncedGnssDataBuff_.front().local_U;
        // std::cout<<"gnss: "<<std::endl<<std::fixed<<std::setprecision(4)<<syncedGnssDataBuff_.front().time<<std::endl;
        static Eigen::Vector3d _last_result;
        Eigen::Vector3d _result = worldToBaselink.inverse() * _gnss_tranlation;
        frontEndPtr_->SetGnssData(_result);
        _last_result = _result;
        syncedGnssDataBuff_.pop_front();
    }

    bool match_result = false;
    if(adjust_distort_){
        distortCloudPtr_->PointCloudInput(dataFrames.front().cloud.cloud_ptr, _predict_delta_pose);
        distortCloudPtr_->AdjustCloud();
        // std::cout<<"补偿畸变："<<std::endl;
        match_result = frontEndPtr_->Update(*distortCloudPtr_->GetAdjustPointCloud());
    }else{
        match_result = frontEndPtr_->Update(*dataFrames.front().cloud.cloud_ptr); 
    }
    // std::cout<<"匹配位姿："<<std::endl<<frontEndPtr_->GetMotion().matrix()<<std::endl;
    return match_result;
}

bool FrontEndFlow::PublishData(){
    ros::Time _stamp = ros::Time().fromSec(dataFrames.front().cloud.time);//记录点云的时间戳
    //发布去畸变后的点云
    cloudAdjustPubPtr_->Publish(distortCloudPtr_->GetAdjustPointCloud(), _stamp);

    downSizeFilterImagePointCloud_.setInputCloud(distortCloudPtr_->GetAdjustPointCloud());
    downSizeFilterImagePointCloud_.filter(*ImagePointCloudDSPtr);
    // ImagePointCloudDSPtr->clear();
    // *ImagePointCloudDSPtr += *frontEndPtr_->MatchingPtr->featureExtractPtr->GETLessSharpCornerPoints();
    // *ImagePointCloudDSPtr += *frontEndPtr_->MatchingPtr->featureExtractPtr->GETLessFlatSurfacePoints();
    downsizePointCloudPubPtr_->Publish(ImagePointCloudDSPtr,_stamp);
    //发布分割聚类后点云
    imagePointCloudPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETImageCloud(), _stamp);
    groundCloudPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETGroundCloud(), _stamp);
    obstacleCloudPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETObstacleCloud(), _stamp);
    segmentedCloudPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETSegmentedCloud(), _stamp);
    //发布提取的特征点
    cornerPointsSharpPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETCornerPoints(), _stamp);
    cornerPointsLessSharpPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETLessSharpCornerPoints(), _stamp);
    surfacePointsFlatPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETSurfacePoints(), _stamp);
    surfacePointsLessFlatPubPtr_->Publish(frontEndPtr_->MatchingPtr->featureExtractPtr->GETLessFlatSurfacePoints(), _stamp);
    cornerSubmapPubPtr_->Publish(frontEndPtr_->MatchingPtr->GetSubmapCornerPoint(), _stamp);
    surfaceSubmapPubPtr_->Publish(frontEndPtr_->MatchingPtr->GetSubmapSurfaceoint(), _stamp);
    PointCloudData::point_cloud_ptr _cloud_in_world(new PointCloudData::point_cloud());
    PointCloudTransformation::TransformToWorld(frontEndPtr_->MatchingPtr->featureExtractPtr->GETImageCloud(), 
                                                _cloud_in_world, 
                                                frontEndPtr_->GetCurrentLidarPose());
    cloudWorldPubPtr_->Publish(_cloud_in_world, _stamp);
    loam_frame::RotateAngleToIsometry3d(1,Eigen::Vector3d(0,0,1));
    pathPubPtr_->Publish(frontEndPtr_->GetCurrentLidarPose(), _stamp);
    gnssPathPubPtr_->Publish(frontEndPtr_->GetCurrentLidarTruthPose(), _stamp);

    lidarOdometryPubPtr->Publish(frontEndPtr_->GetMotion(), _stamp);
    gnssOdometryPubPtr->Publish(frontEndPtr_->GetCurrentLidarTruthPose(), _stamp);
}

bool FrontEndFlow::HasData(){
    if (dataFrames.size() == 0)
        return false;
    if (syncedGnssDataBuff_.size() == 0)
        return false;
    return true;
}

bool FrontEndFlow::ValidGNSSData(){
    double d_time;
    bool gnss_data_is_valid = false;
    while(!syncedGnssDataBuff_.empty()){//时间对齐
        if (!gnssOriginPositionInited) {
            syncedGnssDataBuff_.front().InitOriginPosition();
            gnssOriginPositionInited = true;
        }
        if(syncedGnssDataBuff_.front().time < dataFrames.front().cloud.time){
            if(gnssOriginPositionInited){
                syncedGnssDataBuff_.front().UpdateXYZ();
            }
            syncedGnssDataBuff_.pop_front();
        }else{
            gnss_data_is_valid = true;
            break;
        }
    }
    return gnss_data_is_valid;
}

bool FrontEndFlow::Finally(){
    dataFrames.pop_front();
    //重置参数，用于下一帧点云处理
    frontEndPtr_->MatchingPtr->featureExtractPtr->ResetParameters();
    return true;
}

bool FrontEndFlow::Run() {
    TicToc _time;
    if (!ReadData())    //读取数据
        return false;

    if(!InitUnsyncedData())     //保证处理的第一帧点云，前面是有imu和gnss数据的。
    {
        return false;
    }

    SyncDataClock();   //对imu和gnss数据插值，得到点云时刻的数据

    // if(syncedImuDataBuff_.empty()){
    //     for(auto _iter = unsyncedImuDataBuff_.begin(); _iter != unsyncedImuDataBuff_.end(); _iter++){
    //             std::cout<<"unsync imu time: "<<std::fixed<<std::setprecision(4)<<_iter->time<<std::endl;
    //     }
    //      std::cout<<"------------------------------------------"<<std::endl;
    // }

    while(!syncedImuDataBuff_.empty() && (cloudDataBuff_.size()>1)){
        // std::cout<<"开始预积分"<<std::endl;
        if(PreintegrateIMU(syncedImuDataBuff_.front())){
            Eigen::Isometry3d _preintegrate_pose = Eigen::Isometry3d(imu_pose_.cast<double>());
            DataFrame _frame;
            _frame.cloud = cloudDataBuff_.front();
            _frame.preintegrate_imu_pose = _preintegrate_pose;
            dataFrames.push_back(_frame);
            cloudDataBuff_.pop_front();
            break;
        }

        syncedImuDataBuff_.pop_front();
    }
    

    while(!dataFrames.empty()){
        if(!HasData()){
            return false;
        }
        // std::cout<<"有数据完成"<<std::endl;
        _time.tic();
        if (UpdateLaserOdometry()){
            std::cout<<"前端处理一帧总时间:"<<_time.toc()<<std::endl;
            PublishData();
            // std::cout<<"发布完成"<<std::endl;
            Finally();
        }
    }

    return true;
}

}