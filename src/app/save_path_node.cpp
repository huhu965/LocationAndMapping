/*
 * @Author: Hu Ziwei 
 * @Description:  发布真实轨迹，记录真实轨迹和里程计轨迹
 * @Date: 2022-04-14 19:22:02 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-14 23:41:19
 */
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "publisher/path_publisher.hpp"
#include "subscriber/path_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
#include "tools_/frame_rotate.hpp"
#include "tools_/file_manager.hpp"
#include "global_defination/global_defination.h"

using namespace loam_frame;

bool SavePose(std::ofstream& ofs, const Eigen::Isometry3d& pose){
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

//该函数插值用的是线性插值，所以path中相隔两帧变换较大的话，旋转方向的精度可能会下降
bool Sync_data(nav_msgs::Path &path, double sync_time, geometry_msgs::PoseStamped &synced_data){
    if(path.poses.empty() || path.poses.front().header.stamp.toSec() > sync_time || path.poses.back().header.stamp.toSec() < sync_time){
        return false;
    }

    geometry_msgs::PoseStamped front_data;
    geometry_msgs::PoseStamped back_data;
    bool find_front = false;
    bool find_back = false;
    for(auto _iter = path.poses.begin(); _iter != path.poses.end(); _iter++){
        if(_iter->header.stamp.toSec() <= sync_time){
            front_data = *_iter;
            find_front = true;
        }else{
            if(find_front){
                find_back = true;
                back_data= *_iter;
                break;
            }else{
                return false;
            }
        }
    }

    // std::cout<<"front: "<<std::fixed<<std::setprecision(4)<<front_data.header.stamp.toSec()<<std::endl;
    // std::cout<<"sync: "<<std::fixed<<std::setprecision(4)<<sync_time<<std::endl;
    // std::cout<<"back: "<<std::fixed<<std::setprecision(4)<<back_data.header.stamp.toSec()<<std::endl;

    if(!find_front || !find_back){
        return false;
    }

    double front_scale = (back_data.header.stamp.toSec() - sync_time) / (back_data.header.stamp.toSec() - front_data.header.stamp.toSec());
    double back_scale = (sync_time - front_data.header.stamp.toSec()) / (back_data.header.stamp.toSec() - front_data.header.stamp.toSec());

    synced_data.header.stamp = ros::Time().fromSec(sync_time);

    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    Eigen::Quaterniond _q;
    _q.x() = front_data.pose.orientation.x * front_scale + back_data.pose.orientation.x * back_scale;
    _q.y() = front_data.pose.orientation.y * front_scale + back_data.pose.orientation.y * back_scale;
    _q.z() = front_data.pose.orientation.z * front_scale + back_data.pose.orientation.z * back_scale;
    _q.w() = front_data.pose.orientation.w * front_scale + back_data.pose.orientation.w * back_scale;
    _q.normalize();

    synced_data.pose.orientation.x = _q.x();
    synced_data.pose.orientation.y = _q.y();
    synced_data.pose.orientation.z = _q.z();
    synced_data.pose.orientation.w = _q.w();

    //set the position
    synced_data.pose.position.x = front_data.pose.position.x * front_scale + back_data.pose.position.x * back_scale;
    synced_data.pose.position.y = front_data.pose.position.y * front_scale + back_data.pose.position.y * back_scale;
    synced_data.pose.position.z = front_data.pose.position.z * front_scale + back_data.pose.position.z * back_scale;

    return true;
}

bool SavePath(const YAML::Node& config_node, nav_msgs::Path &lidar_path, nav_msgs::Path &truth_path){
    std::string trajectoryPath_ = "";
    std::string data_path_ = "";
    std::ofstream groundTruthOfs_;
    std::ofstream laserOdomOfs_;

    std::string data_path = config_node["data_path"].as<std::string>();
    std::string method_name = config_node["method_name"].as<std::string>();

    if(data_path == "./"){
        data_path = WORK_SPACE_PATH;
    }else{
        data_path = WORK_SPACE_PATH + data_path;
    }

    if (!FileManager::CreateDirectory(data_path + "/path_data"))
        return false;

    if (!FileManager::CreateDirectory(data_path + "/path_data" + method_name))
        return false;

    trajectoryPath_ = data_path + "/path_data" + method_name + "/optimize_trajectory";


    if (!FileManager::InitDirectory(trajectoryPath_, "轨迹文件"))
        return false;

    if (!FileManager::CreateFile(groundTruthOfs_, trajectoryPath_ + "/truth_path.txt"))
        return false;
    if (!FileManager::CreateFile(laserOdomOfs_, trajectoryPath_ + "/odome_path.txt"))
        return false;
    std::cout << "轨迹存放位置：" << trajectoryPath_ << std::endl;


    std::cout<<lidar_path.poses.size()<<std::endl;
    std::cout<<truth_path.poses.size()<<std::endl;

    geometry_msgs::PoseStamped sync_gnss_pose;
    for(auto _iter = lidar_path.poses.begin(); _iter != lidar_path.poses.end(); _iter++){
        if(!Sync_data(truth_path,_iter->header.stamp.toSec(),sync_gnss_pose)){
            continue;
        }else{
            Eigen::Isometry3d pose;
            Eigen::Vector3d translation;
            //存里程计轨迹
            pose.setIdentity();
            Eigen::Quaterniond q = Eigen::Quaterniond(_iter->pose.orientation.w,
                                                        _iter->pose.orientation.x,
                                                        _iter->pose.orientation.y,
                                                        _iter->pose.orientation.z);
            pose.rotate(q);                                       
            translation = Eigen::Vector3d(_iter->pose.position.x, _iter->pose.position.y, _iter->pose.position.z);
            pose.pretranslate(translation);
            SavePose(laserOdomOfs_,pose);

            //存真实轨迹
            pose.setIdentity();
            Eigen::Quaterniond q2 = Eigen::Quaterniond(sync_gnss_pose.pose.orientation.w,
                                                        sync_gnss_pose.pose.orientation.x,
                                                        sync_gnss_pose.pose.orientation.y,
                                                        sync_gnss_pose.pose.orientation.z);
            pose.rotate(q2);  
            translation = Eigen::Vector3d(sync_gnss_pose.pose.position.x, sync_gnss_pose.pose.position.y, sync_gnss_pose.pose.position.z);
            pose.pretranslate(translation);
            SavePose(groundTruthOfs_,pose);
        }
    }

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "save_path_node");

    std::string config_file_path = WORK_SPACE_PATH + "/config/save_path/save_path_config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string odom_path_topic = config_node["odom_path_topic"].as<std::string>();
    std::string gnss_frame = config_node["gnss_frame"].as<std::string>();

    ros::NodeHandle nh;
    std::shared_ptr<GNSSSubscriber> gnssSubPtr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 100000);
    std::shared_ptr<PathSubscriber> odomPathSubPtr_ = std::make_shared<PathSubscriber>(nh, odom_path_topic, 100000); //估计出的位姿
    //订阅世界坐标系到机器人本地坐标系的旋转矩阵
    std::shared_ptr<TFListener> worldToBaselinkLisPtr = std::make_shared<TFListener>(nh, "world", "base_link");
    std::shared_ptr<PathPublisher> truthPathPubPtr_ = std::make_shared<PathPublisher>(nh, "/truth_path", gnss_frame, 100);
    std::deque<GNSSData> GnssBuff;

    Eigen::Isometry3d trans;
    trans(0,0) = -0.163585, trans(0,1) = 0.986258, trans(0,2) = -0.0231399, trans(0,3) = 0;
    trans(1,0) = -0.986282, trans(1,1) = -0.162974, trans(1,2) = 0.026214, trans(1,3) = 0;
    trans(2,0) = 0.0220825, trans(2,1) = 0.0271107, trans(2,2) = 0.999388, trans(2,3) = 0;
    trans(3,0) = 0, trans(3,1) = 0, trans(3,2) = 0, trans(3,3) = 1;


    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();//会读取所有的数据
        //从接收缓冲区中读取接收到的数据
        gnssSubPtr_->ParseData(GnssBuff);
        if (GnssBuff.empty()){
            continue;
        }

        static bool transformReceived = false;
        Eigen::Matrix4f worldToBaselink = Eigen::Matrix4f::Identity();
        // if (!transformReceived) {
        //     if (worldToBaselinkLisPtr->LookupData(worldToBaselink)) {
        //         std::cout<<worldToBaselink<<std::endl;
        //         std::cout<<"--------------"<<std::endl;
        //         trans = Eigen::Isometry3d(worldToBaselink.cast<double>());
        //         transformReceived = true;
        //     }
        // }
        if (!transformReceived) {
            if (worldToBaselinkLisPtr->LookupData(worldToBaselink)) {
                trans = Eigen::Isometry3d(worldToBaselink.cast<double>()).inverse();
                std::cout<<"save trans"<<std::endl<<trans.matrix()<<std::endl;

                Eigen::Affine3d _temp_pose(worldToBaselink.cast<double>().matrix());
                double x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(_temp_pose, x, y, z, roll, pitch, yaw);
                std::cout<<"roll: "<<roll<<
                        "pitch: "<<pitch<<
                        "yaw: "<<yaw<<std::endl;
                transformReceived = true;
            }
        }


        static bool gnssOriginPositionInited = false;
        if (!gnssOriginPositionInited) {
            GnssBuff.front().InitOriginPosition();
            gnssOriginPositionInited = true;
        }

        while (!GnssBuff.empty())
        {
            GnssBuff.front().UpdateXYZ();
            Eigen::Vector3d _gnss_tranlation;
            _gnss_tranlation[0] = GnssBuff.front().local_E;
            _gnss_tranlation[1] = GnssBuff.front().local_N;
            _gnss_tranlation[2] = GnssBuff.front().local_U;
            Eigen::Isometry3d gnss_result;

            Eigen::Vector3d _result = trans * _gnss_tranlation;
            double x_pose = config_node["x_pose"].as<double>();
            _result.x() = _result.x() + x_pose;
            gnss_result.setIdentity();
            gnss_result.translation() = _result;

            //针对lego_loam,这狗东西坐标轴瞎搞
            gnss_result = loam_frame::RotateAngleToIsometry3d(-88,Eigen::Vector3d(1,0,0)) * gnss_result;
            double yaw = config_node["yaw"].as<double>();
            gnss_result = loam_frame::RotateAngleToIsometry3d(yaw,Eigen::Vector3d(0,1,0)) * gnss_result;
            gnss_result = loam_frame::RotateAngleToIsometry3d(1.8,Eigen::Vector3d(1,0,0)) * gnss_result;
            ros::Time _stamp = ros::Time().fromSec(GnssBuff.front().time);
            truthPathPubPtr_->Publish(gnss_result, _stamp);
            GnssBuff.pop_front();
        }
    }

    bool need_save_pose = config_node["need_save_path"].as<bool>();
    std::cout<<"是否保存： "<<need_save_pose<<std::endl;
    if(need_save_pose){
        nav_msgs::Path laser_path;
        odomPathSubPtr_->ParseData(laser_path);
        nav_msgs::Path truth_path = truthPathPubPtr_->GetPath();

        if(!SavePath(config_node, laser_path, truth_path)){
            std::cout<<"save node:保存路径失败"<<std::endl;
        }else{
            std::cout<<"save node:保存路径成功"<<std::endl;
        }
    }

    return 0;
}