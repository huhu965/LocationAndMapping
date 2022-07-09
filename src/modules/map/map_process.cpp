/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-07 15:25:56 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-08 16:16:48
 */

#include "modules/map/map_process.hpp"

namespace loam_frame{
    MapProcess::MapProcess(ros::NodeHandle& nh):MapVariable(nh){}
    //保存关键帧点云
    void MapProcess::AddKeyFrame(PointCloudData::point_cloud_ptr _fullCloudPtr, 
                                PointCloudData::point_cloud_ptr _cornerCloudPtr, 
                                PointCloudData::point_cloud_ptr _surfaceCloudPtr){
        // 当前帧激光角点、平面点，降采样集合
        PointCloudData::point_cloud_ptr _thisCornerKeyFramePtr(new PointCloudData::point_cloud());
        PointCloudData::point_cloud_ptr _thisSurfKeyFramePtr(new PointCloudData::point_cloud());
        PointCloudData::point_cloud_ptr _thisFullKeyFramePtr(new PointCloudData::point_cloud());

        pcl::copyPointCloud(*_fullCloudPtr, *_thisFullKeyFramePtr);
        pcl::copyPointCloud(*_cornerCloudPtr, *_thisCornerKeyFramePtr);
        pcl::copyPointCloud(*_surfaceCloudPtr, *_thisSurfKeyFramePtr);

        //保存关键帧
        fullCloudKeyFrames.push_back(_thisFullKeyFramePtr);
        cornerCloudKeyFrames.push_back(_thisCornerKeyFramePtr);
        surfCloudKeyFrames.push_back(_thisSurfKeyFramePtr);
    }

    //保存关键帧点云
    void MapProcess::AddKeyFrame(PointCloudData::point_cloud_ptr _fullCloudPtr){
        // 当前帧激光点
        PointCloudData::point_cloud_ptr _thisFullKeyFramePtr(new PointCloudData::point_cloud());
        pcl::copyPointCloud(*_fullCloudPtr, *_thisFullKeyFramePtr);
        //保存关键帧
        fullCloudKeyFrames.push_back(_thisFullKeyFramePtr);
    }

    //保存关键帧的位姿
    void MapProcess::AddKeyPose(PoseParam & _poseParam){
            //添加关键帧
            Eigen::Affine3d _temp_pose(_poseParam.pose.matrix());
            double x, y, z;
            pcl::getTranslationAndEulerAngles(_temp_pose, x, y, z, _poseParam.roll, _poseParam.pitch, _poseParam.yaw);
            _poseParam.index = poseCloudKeyFrames.size();
            poseCloudKeyFrames.push_back(std::move(_poseParam));
            //额外保存关键帧的平移量，用于子地图的提取
            PointCloudData::point _translation;
            _translation.x = _poseParam.pose.translation().x();
            _translation.y = _poseParam.pose.translation().y();
            _translation.z = _poseParam.pose.translation().z();
            _translation.intensity = _poseParam.index;
            translationCloudKeyFrames->push_back(std::move(_translation));
    }

    void MapProcess::CorrectPoses(std::deque<Eigen::Isometry3d> &_optimizedPoses){
        int _num = _optimizedPoses.size();
        for(int i = 0; i <_num; i++){
            poseCloudKeyFrames[i].pose = _optimizedPoses[i];
            Eigen::Affine3d _temp_pose(_optimizedPoses[i].matrix());
            double x, y, z;
            pcl::getTranslationAndEulerAngles(_temp_pose, x, y, z, 
                                                poseCloudKeyFrames[i].roll,
                                                poseCloudKeyFrames[i].pitch,
                                                poseCloudKeyFrames[i].yaw);

            translationCloudKeyFrames->points[i].x = _optimizedPoses[i].translation().x();
            translationCloudKeyFrames->points[i].y = _optimizedPoses[i].translation().y();
            translationCloudKeyFrames->points[i].z = _optimizedPoses[i].translation().z();
        }
    }

    void MapProcess::GenerateMap(){
        worldMapPtr->clear();
        worldMapDSPtr->clear();

        for(int i = 0;i<fullCloudKeyFrames.size();i++){
            *worldMapPtr += PointCloudTransformation::TransformToWorld(fullCloudKeyFrames[i],
                                                        poseCloudKeyFrames[i].pose);
        }
        std::cout<<"存的关键帧："<<fullCloudKeyFrames.size()<<std::endl;
        std::cout<<"存的位姿："<<poseCloudKeyFrames.size()<<std::endl;

        // 降采样局部角点map
        downSizeFilterWorldMap_.setInputCloud(worldMapPtr);
        downSizeFilterWorldMap_.filter(*worldMapDSPtr);
    }

    void MapProcess::AddKeyframToMap(){
        *worldMapDSPtr += PointCloudTransformation::TransformToWorld(fullCloudKeyFrames.back(),
                                                        poseCloudKeyFrames.back().pose);
    }

    PointCloudData::point_cloud_ptr MapProcess::GetWorldMap(){
        return worldMapDSPtr;
    }

}